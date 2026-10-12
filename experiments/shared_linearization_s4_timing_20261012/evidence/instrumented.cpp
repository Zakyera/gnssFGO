#include "Timing.h"
#include "NativeGaussianHistory.h"
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/JointMarginal.h>
#include <gtsam/linear/Scatter.h>
#include <cbs/utils/relative_pose_covariance.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace shared_native {
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void finiteSymmetric(const gtsam::Matrix& h) {
  require(h.rows() == h.cols() && h.allFinite(), "nonfinite Gaussian equations");
  require((h-h.transpose()).norm() <= 1e-12 * std::max(1.0,h.norm()),
          "asymmetric Gaussian equations");
}
void positiveDefinite(const gtsam::Matrix& h) {
  finiteSymmetric(h);
  require(h.rows() > 0, "empty native information");
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h);
  require(eigen.info() == Eigen::Success, "eigenvalue failure");
  // Explicit rejection only: never regularize an unobservable covariance query.
  require(eigen.eigenvalues().minCoeff() >
      1e-12 * std::max(1.0, eigen.eigenvalues().maxCoeff()),
      "unobservable or numerically singular native information");
}
bool sameChart(const gtsam::Values& a, const gtsam::Values& b, gtsam::Key key) {
  return a.exists(key) && b.exists(key) && a.at(key).equals_(b.at(key), 0.0);
}
}
struct NativeGaussianHistory::State {
  struct Entry { Id version; std::size_t slot; gtsam::GaussianFactor::shared_ptr factor; };
  Id generation = 1, epoch = 0, highest_id = 0;
  std::size_t admitted = 0, absorbed = 0, removed = 0;
  std::map<Id, Entry> active;
  gtsam::GaussianFactorGraph prior;
  gtsam::Values theta;
  gtsam::GaussianFactorGraph graph() const {
    auto result = prior.clone();
    for (const auto& item : active) result.add(*item.second.factor);
    return result;
  }
};
NativeGaussianHistory::NativeGaussianHistory(Mode mode)
    : mode_(mode), committed_(new State) {}
NativeGaussianHistory::~NativeGaussianHistory() = default;
Id NativeGaussianHistory::epoch() const { return committed_->epoch; }
std::size_t NativeGaussianHistory::activeCount() const { return committed_->active.size(); }
std::size_t NativeGaussianHistory::absorbedCount() const { return committed_->absorbed; }
std::size_t NativeGaussianHistory::removedCount() const { return committed_->removed; }
std::size_t NativeGaussianHistory::admittedCount() const { return committed_->admitted; }
void NativeGaussianHistory::abort() { pending_.reset(); }
Result NativeGaussianHistory::reset(Id generation) {
  if (generation <= committed_->generation) return {false, "nonmonotonic reset generation"};
  pending_.reset();
  committed_.reset(new State);
  committed_->generation = generation;
  return {true, "reset; history must be rebuilt from native initialization"};
}
Result NativeGaussianHistory::stage(const Snapshot& s) {
  try {
    diagnostic_timing::Span timing("stage.");
    require(mode_ == Mode::Shadow, "disabled");
    require(!pending_, "pending transaction exists");
    require(s.generation == committed_->generation, "wrong generation; reset required");
    require(s.epoch == committed_->epoch + 1, "stale or skipped epoch");
    auto next = std::make_unique<State>(*committed_);
    // Prior references never move. In MAIN's normal fixed-lag path these keys
    // must remain fixed; a solver reset/rebase requires invalidation, not copying
    // the external-information-bearing MAIN marginal into this history.
    for (auto key : committed_->prior.keys()) {
      require(sameChart(committed_->theta, s.theta, key), "prior reference chart changed");
      require(s.retiring.count(key) || s.fixed_after.count(key), "prior separator not fixed");
    }
    timing.mark("copy_and_prior_checks_s");
    std::set<Id> seen;
    std::set<std::size_t> used_slots;
    std::map<Id, State::Entry> active;
    Id highest = committed_->highest_id;
    for (const auto& ref : s.native) {
      require(ref.id > 0 && seen.insert(ref.id).second, "duplicate/zero native identity");
      require(!s.removed.count(ref.id), "identity both active and removed");
      require(used_slots.insert(ref.slot).second, "duplicate native cache slot");
      require(ref.slot < s.cache.size() && s.cache[ref.slot], "missing native cached equation");
      const auto& f = s.cache[ref.slot];
      require(!f->empty(), "native admission cannot be a constant-only factor");
      const auto jacobian = std::dynamic_pointer_cast<gtsam::JacobianFactor>(f);
      require(!jacobian || !jacobian->get_model() || !jacobian->get_model()->isConstrained(),
              "constrained Gaussian equations need a separate rank policy");
      auto aug = f->augmentedInformation();
      finiteSymmetric(aug);
      std::set<gtsam::Key> unique;
      for (auto it = f->begin(); it != f->end(); ++it) {
        require(unique.insert(*it).second, "duplicate factor key");
        require(s.theta.exists(*it) && s.theta.at(*it).dim() == f->getDim(it),
                "missing or wrong-dimensional reference value");
      }
      const auto old = committed_->active.find(ref.id);
      if (old != committed_->active.end()) {
        require(ref.slot == old->second.slot, "active identity moved cache slot");
        require(ref.version >= old->second.version, "stale native version");
        require(f->keys() == old->second.factor->keys(), "identity changed keys");
        if (ref.version == old->second.version) {
          require(aug.isApprox(old->second.factor->augmentedInformation(), 0.0),
                  "equations changed without version");
          for (auto key : f->keys())
            require(sameChart(committed_->theta, s.theta, key),
                    "reference changed without refreshed equation version");
        }
      } else {
        require(ref.id > committed_->highest_id, "retired/removed identity resurrected");
        highest = std::max(highest, ref.id);
        ++next->admitted;
      }
      active.emplace(ref.id, State::Entry{ref.version, ref.slot, f->clone()});
    }
    for (const auto& old : committed_->active)
      require(seen.count(old.first) || s.removed.count(old.first), "undeclared native removal");
    for (auto id : s.removed) {
      require(committed_->active.count(id), "removal of unknown native identity");
      ++next->removed;
    }
    timing.mark("validate_and_clone_native_s");
    auto incident = committed_->prior.clone();
    for (auto it = active.begin(); it != active.end();) {
      bool touches = false;
      for (auto key : it->second.factor->keys()) touches |= s.retiring.count(key) != 0;
      if (touches) {
        incident.add(*it->second.factor);
        ++next->absorbed;
        it = active.erase(it);
      } else ++it;
    }
    for (auto key : s.retiring)
      require(s.theta.exists(key) && incident.keys().count(key), "unknown/unconstrained retiring key");
    timing.mark("partition_incident_s");
    if (!s.retiring.empty()) {
      // GTSAM Gaussian elimination only. Keep the remaining Hessian including
      // eta and constant. Never include nonincident active factors here.
      gtsam::Ordering order(s.retiring.begin(), s.retiring.end());
      auto eliminated = gtsam::EliminateCholesky(incident, order);
      finiteSymmetric(eliminated.second->augmentedInformation());
      next->prior.resize(0);
      next->prior.add(*eliminated.second);
    }
    timing.mark("retirement_elimination_s");
    next->active = std::move(active);
    next->theta = s.theta;
    for (auto key : s.retiring) next->theta.erase(key);
    for (auto key : next->prior.keys())
      require(s.fixed_after.count(key), "new prior separator not fixed by MAIN");
    const auto native_keys = next->graph().keys();
    // Do not retain unrelated MAIN/external-only Values in a bounded native history.
    for (auto key : next->theta.keys()) if (!native_keys.count(key)) next->theta.erase(key);
    next->highest_id = highest;
    next->epoch = s.epoch;
    require(next->admitted == next->absorbed + next->removed + next->active.size(),
            "native contribution conservation failure");
    pending_ = std::move(next);
    timing.mark("finalize_s");
    return {true, "staged; MAIN success still required"};
  } catch (const std::exception& e) { return {false, e.what()}; }
    catch (...) { return {false, "unknown candidate exception"}; }
}
Result NativeGaussianHistory::commit(Id generation, Id epoch) {
  if (!pending_ || pending_->generation != generation || pending_->epoch != epoch)
    return {false, "no matching staged transaction"};
  committed_.swap(pending_);
  pending_.reset();
  return {true, "committed"};
}
gtsam::GaussianFactorGraph NativeGaussianHistory::graph(Id expected) const {
  require(mode_ == Mode::Shadow && expected != 0 && expected == committed_->epoch,
          "disabled, uninitialized or stale query");
  return committed_->graph();
}
gtsam::Values NativeGaussianHistory::theta(Id expected) const {
  (void)graph(expected);
  return committed_->theta;
}
EndpointCovariance NativeGaussianHistory::covariance(Id expected, gtsam::Key from,
    gtsam::Key to, const gtsam::Vector6& df, const gtsam::Vector6& dt) const {
  diagnostic_timing::Span timing("query.");
  require(from != to && df.allFinite() && dt.allFinite(), "invalid endpoint query");
  auto native = graph(expected);
  require(committed_->theta.exists(from) && committed_->theta.exists(to),
          "endpoint absent from committed native history");
  timing.mark("clone_graph_s");
  auto h = native.hessian().first;
  timing.mark("assemble_dense_hessian_s");
  positiveDefinite(h);
  timing.mark("dense_rank_check_s");
  auto tree = native.eliminateMultifrontal(gtsam::Ordering::COLAMD, gtsam::EliminateCholesky);
  timing.mark("sparse_elimination_s");
  auto joint = tree->jointMarginalCovariance({from, to});
  timing.mark("joint_marginal_s");
  EndpointCovariance out;
  out.joint_theta.resize(12,12);
  out.joint_theta << joint.at(from,from), joint.at(from,to),
                     joint.at(to,from), joint.at(to,to);
  out.transport = gtsam::Matrix::Zero(12,12);
  gtsam::Matrix6 af, at;
  const auto pf = committed_->theta.at<gtsam::Pose3>(from).retract(df);
  const auto pt = committed_->theta.at<gtsam::Pose3>(to).retract(dt);
  // ChartAtOrigin supplies the derivative for the actually compiled retraction.
  gtsam::Pose3::ChartAtOrigin::Retract(df, af);
  gtsam::Pose3::ChartAtOrigin::Retract(dt, at);
  out.transport.topLeftCorner<6,6>() = af;
  out.transport.bottomRightCorner<6,6>() = at;
  out.joint_current = out.transport * out.joint_theta * out.transport.transpose();
  timing.mark("extract_and_transport_s");
  positiveDefinite(out.joint_current);
  timing.mark("endpoint_rank_check_s");
  gtsam::Matrix info = out.joint_current.llt().solve(gtsam::Matrix::Identity(12,12));
  gtsam::Scatter scatter;
  scatter.emplace_back(from,6);
  scatter.emplace_back(to,6);
  auto result = cbs::relative_pose_covariance::compute(gtsam::JointMarginal(info,scatter),
      from,to,pf,pt,pf.between(pt),cbs::relative_pose_covariance::Mode::SchurRelativeBetween);
  require(!result.used_jitter, "relative helper required jitter; candidate rejected");
  out.relative = result.covariance;
  timing.mark("relative_covariance_helper_s");
  return out;
}
Result NativeGaussianHistory::tryCovariance(Id expected, gtsam::Key from,
    gtsam::Key to, const gtsam::Vector6& df, const gtsam::Vector6& dt,
    EndpointCovariance& output) const {
  try {
    auto candidate = covariance(expected,from,to,df,dt);
    output = std::move(candidate);
    return {true,"native covariance available"};
  } catch (const std::exception& e) { return {false,e.what()}; }
    catch (...) { return {false,"unknown candidate covariance exception"}; }
}
}  // namespace shared_native
