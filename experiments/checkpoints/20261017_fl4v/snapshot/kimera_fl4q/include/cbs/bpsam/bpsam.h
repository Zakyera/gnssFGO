#pragma once

#include <aria_common/benchmark.h>
#include <aria_common/logging.h>
#include <aria_common/macros.h>
#include <aria_common/types.h>
#include <cbs/gbp/belief_metric.h>
#include <cbs/gbp/gbp.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cbs/bpsam/batch_sam_wrapper.h"
#include "cbs/key.h"
#include "cbs/utils/graph_io.h"
#include "cbs/utils/gtsam_compat.h"
#include "cbs/utils/health_aware_relative_cbs.h"

using namespace gtsam;

namespace cbs {
using GncParamsGN = GncParams<GaussNewtonParams>;
using GncOptimizerGN = GncOptimizer<GncParamsGN>;
// using BaseSAM = cbs::SAMWrapper<GncParamsGN, GncOptimizerGN>;
// using BaseSAMParams = ISAM2Params;
// using BaseSAM = cbs::SAMWrapper<GaussNewtonParams, GaussNewtonOptimizer>;
// using BaseSAMParams = ISAM2Params;
using BaseSAM = gtsam::ISAM2;
using BaseSAMParams = gtsam::ISAM2Params;

class BPSAM : public BaseSAM {
 public:
  ARIA_POINTER_TYPEDEFS(BPSAM);
  auto fl4qMarginalCacheRestoreAction() {
    const auto saved=tmp_marginals_;
    return [this,saved] { tmp_marginals_=saved; };
  }
  // Private FL4q candidate. Rebuild only the ISAM2 core, not CBS ownership maps.
  // All linear containers must already have been given immutable original
  // charts; no arbitrary change of an unanchored Gaussian's coordinate system.
  void fl4qRebuild(const NonlinearFactorGraph& graph, const Values& accepted,
                  const std::optional<FastMap<Key,int>>& constraints) {
    if (!std::holds_alternative<ISAM2GaussNewtonParams>(BaseSAM::params_.optimizationParams) ||
        !BaseSAM::params_.cacheLinearizedFactors ||
        params_.use_temporary_cbs_linear_factors ||
        params_.cbs_odom_factor_mode != CbsOdomFactorMode::ActiveWindowTemporary)
      throw std::runtime_error("FL4Q incompatible core mode");
    if (graph.size()!=nonlinearFactors_.size() || accepted.keys()!=theta_.keys())
      throw std::runtime_error("FL4Q changed core keys/slots");
    for (size_t i=0;i<graph.size();++i) {
      if (bool(graph[i])!=bool(nonlinearFactors_[i]) ||
          (graph[i] && graph[i]->keys()!=nonlinearFactors_[i]->keys()))
        throw std::runtime_error("FL4Q changed factor identity/keys");
    }
    const auto saved_params=BaseSAM::params_;
    auto build_params=saved_params;
    build_params.findUnusedFactorSlots=false; // Keep holes and exact slot IDs.
    build_params.evaluateNonlinearError=false;
    BaseSAM rebuilt(build_params);
    ISAM2UpdateParams update;
    update.constrainedKeys=constraints;
    const auto result=rebuilt.update(graph,accepted,update);
    for(size_t i=0;i<graph.size();++i)
      if(result.newFactorsIndices.at(i)!=i)
        throw std::runtime_error("FL4Q rebuilt factor slot changed");
    const int saved_update_count=update_count_;
    BaseSAM::operator=(rebuilt);
    BaseSAM::params_=saved_params;
    update_count_=saved_update_count;
    // Tree and cached Gaussians are undamped at accepted. The accepted
    // nonlinear step is zero in this new chart, not the next raw GN step.
    delta_=theta_.zeroVectors();
    deltaNewton_=delta_;
    RgProd_=delta_;
    deltaReplacedMask_.clear();
    tmp_marginals_.reset();
  }

  const GaussianFactorGraph& fl4nLinearCache() const { return linearFactors_; }

  enum class CbsOdomFactorMode {
    Legacy,
    Persistent,
    TemporaryLinear,
    ActiveWindowTemporary,
  };

  struct Params {
    AgentId robot_id;
    BaseSAMParams sam_params_;
    gbp::UpdateParams gbp_update_params;
    NonlinearOptimizerParams tol;
    bool enable_belief_dcs{false};
    // When true, the first received belief for a given (agent,key) only
    // initializes GBP state and is not injected into SAM as an external factor.
    bool reject_first_message{true};
    // Distance threshold under the selected belief metric below which a belief
    // is considered too similar to the last one sent to the requesting robot
    // and is omitted.
    // 0.0 disables the filter.
    double belief_similarity_threshold{0.01};
    // When enabled, receiver-side hard rejection compares the current raw
    // incoming belief against the previous raw incoming belief from the same
    // sender/key. The receiver's GBP peer belief is still used for contraction,
    // but no longer decides the hard reset gate.
    bool use_raw_previous_belief_gate{false};
    // Optional receiver-side covariance scale used only when an accepted CBS
    // odometry belief is converted into the final GTSAM BetweenFactor.
    // GBP gating and contraction continue to use the raw incoming covariance.
    std::map<AgentId, double> external_factor_covariance_scale_by_source;
    // Accepted CBS odometry beliefs are linearized inside the current iSAM2
    // update and used only by a temporary augmented linear delta solve. They
    // are never inserted into the nonlinear factor graph or Bayes tree.
    bool use_temporary_cbs_linear_factors{false};
    CbsOdomFactorMode cbs_odom_factor_mode{CbsOdomFactorMode::Legacy};
    // Sender-side covariance used for outgoing relative odometry beliefs.
    // "schur_relative_between" is the corrected default. Use
    // "conditional_to_pose" only as a legacy ablation.
    std::string outgoing_odom_covariance_mode{"schur_relative_between"};
    bool relative_covariance_matrix_dump_enable{false};
    int relative_covariance_matrix_dump_skip{0};
    int relative_covariance_matrix_dump_limit{3};
    double relative_covariance_matrix_dump_trace_max{0.0};
    // In temporary-linear mode, skip beliefs for a receiver-side source/edge
    // that has already been applied as a temporary linear factor. This prevents
    // repeated sender windows from acting as repeated evidence.
    bool temporary_linear_already_applied_gate_enable{true};
    double temporary_linear_already_applied_metric_threshold{0.01};
    double temporary_linear_already_applied_dmu_threshold{1e-3};
    double temporary_linear_already_applied_cov_rel_threshold{1e-3};
    bool active_cbs_odom_factor_diagnostics_enable{false};
    bool active_cbs_odom_factor_detail_diagnostics_enable{false};
    size_t active_cbs_odom_factor_detail_max_rows_per_update{1000u};
    // Bounded, diagnostics-only audit of the exact covariance provenance and
    // linearized receiver objective for accepted CBS odometry factors. Zero
    // disables the audit and preserves the normal receiver path.
    size_t covariance_exchange_audit_max_samples{0u};
    // Select every Nth distinct accepted receiver edge for the bounded audit.
    // One preserves the original first-N behavior.
    size_t covariance_exchange_audit_stride{1u};
    // Legacy health-aware params. Existing callers may still populate only this
    // field; BPSAM will copy it to health_aware_receiver_params unless the
    // receiver-specific field is explicitly marked as set.
    health_aware::Params health_aware_params;
    health_aware::Params health_aware_receiver_params;
    bool has_health_aware_receiver_params{false};
  };

  struct UpdateParams : ISAM2UpdateParams {
    std::set<NonlinearFactor::shared_ptr> unreliable_factors_;
    std::filesystem::path output_dir;
  };

  enum class AddBeliefStatus {
    Accepted,
    AcceptedButSkippedAlreadyApplied,
    RejectedShape,
    RejectedFirstMessage,
    RejectedUpdateStatus,
    RejectedInactiveWindow,
    Exception,
  };

  struct AddBeliefDetail {
    AgentId source_agent = 0;
    Key pose_key = 0;
    AddBeliefStatus status = AddBeliefStatus::Exception;
    bool existing_gbp_var = false;
    bool is_first_message = false;
    bool enable_soft_reset = false;
    gbp::MetricType metric_type = gbp::MetricType::Hellinger;
    double d_reset = 0.0;
    double contract_alpha = 0.0;
    double dxycurr = std::numeric_limits<double>::quiet_NaN();
    double dxy = std::numeric_limits<double>::quiet_NaN();
    double contraction_step_size = std::numeric_limits<double>::quiet_NaN();
    double incoming_trace = std::numeric_limits<double>::quiet_NaN();
    bool has_previous_raw_belief = false;
    std::string raw_previous_metric_status = "no_previous";
    double raw_previous_metric_distance =
        std::numeric_limits<double>::quiet_NaN();
    double raw_previous_dmu = std::numeric_limits<double>::quiet_NaN();
    double raw_previous_trace = std::numeric_limits<double>::quiet_NaN();
    bool raw_previous_gate_enabled = false;
    bool raw_previous_gate_checked = false;
    bool raw_previous_gate_rejected = false;
    bool has_last_applied_temporary_linear_belief = false;
    std::string last_applied_metric_status = "no_previous";
    double last_applied_metric_distance =
        std::numeric_limits<double>::quiet_NaN();
    double last_applied_dmu = std::numeric_limits<double>::quiet_NaN();
    double last_applied_trace = std::numeric_limits<double>::quiet_NaN();
    double last_applied_cov_rel_frobenius =
        std::numeric_limits<double>::quiet_NaN();
    bool temporary_linear_already_applied_gate_enabled = false;
    bool temporary_linear_already_applied_gate_checked = false;
    bool accepted_but_skipped_already_applied = false;
    bool repeated_impulse_prevented = false;
    bool applied_incremental = false;
    std::string gbp_peer_metric_status = "no_peer";
    double gbp_peer_metric_distance =
        std::numeric_limits<double>::quiet_NaN();
    double gbp_peer_dmu = std::numeric_limits<double>::quiet_NaN();
    double gbp_peer_trace = std::numeric_limits<double>::quiet_NaN();
    bool gbp_update_soft_reset_effective = false;
    std::string message;
  };

  struct AddBeliefsResult {
    size_t accepted = 0u;
    size_t accepted_but_skipped_already_applied = 0u;
    size_t rejected_shape = 0u;
    size_t rejected_first_message = 0u;
    size_t rejected_update_status = 0u;
    size_t rejected_inactive_window = 0u;
    size_t rejected_exception = 0u;
    std::vector<AddBeliefDetail> details;

    size_t rejected() const {
      return rejected_shape + rejected_first_message + rejected_update_status +
             rejected_inactive_window + rejected_exception;
    }

    void addDetail(const AddBeliefDetail& detail) {
      details.push_back(detail);
      switch (detail.status) {
        case AddBeliefStatus::Accepted:
          ++accepted;
          break;
        case AddBeliefStatus::AcceptedButSkippedAlreadyApplied:
          ++accepted_but_skipped_already_applied;
          break;
        case AddBeliefStatus::RejectedShape:
          ++rejected_shape;
          break;
        case AddBeliefStatus::RejectedFirstMessage:
          ++rejected_first_message;
          break;
        case AddBeliefStatus::RejectedUpdateStatus:
          ++rejected_update_status;
          break;
        case AddBeliefStatus::RejectedInactiveWindow:
          ++rejected_inactive_window;
          break;
        case AddBeliefStatus::Exception:
          ++rejected_exception;
          break;
      }
    }

    void merge(const AddBeliefsResult& other) {
      accepted += other.accepted;
      accepted_but_skipped_already_applied +=
          other.accepted_but_skipped_already_applied;
      rejected_shape += other.rejected_shape;
      rejected_first_message += other.rejected_first_message;
      rejected_update_status += other.rejected_update_status;
      rejected_inactive_window += other.rejected_inactive_window;
      rejected_exception += other.rejected_exception;
      details.insert(details.end(), other.details.begin(), other.details.end());
    }
  };

  struct CbsOdometryBelief {
    AgentId source_agent = 0;
    // Receiver-side pose keys where this external odometry belief is applied.
    Key from_pose_key = 0;
    Key to_pose_key = 0;
    // Original sender-side pose keys that identify the belief. These are
    // distinct from the receiver keys after timestamp matching.
    Key sender_from_pose_key = 0;
    Key sender_to_pose_key = 0;
    Pose3 measured_from_to;
    Matrix covariance;
    double relax_factor = 0.0;
    double from_pose_trace = std::numeric_limits<double>::quiet_NaN();
    double to_pose_trace = std::numeric_limits<double>::quiet_NaN();
    Matrix to_pose_covariance;
    bool to_pose_covariance_valid = false;
  };

  enum class AddOdometryBeliefStatus {
    Accepted,
    AcceptedButSkippedAlreadyApplied,
    RejectedShape,
    RejectedInactiveWindow,
    Exception,
  };

  struct AddOdometryBeliefDetail {
    AgentId source_agent = 0;
    Key from_pose_key = 0;
    Key to_pose_key = 0;
    AddOdometryBeliefStatus status = AddOdometryBeliefStatus::Exception;
    double covariance_trace = std::numeric_limits<double>::quiet_NaN();
    bool accepted_but_skipped_already_applied = false;
    bool repeated_impulse_prevented = false;
    std::string message;
  };

  struct AddOdometryBeliefsResult {
    size_t accepted = 0u;
    size_t accepted_but_skipped_already_applied = 0u;
    size_t rejected_shape = 0u;
    size_t rejected_inactive_window = 0u;
    size_t rejected_exception = 0u;
    std::vector<AddOdometryBeliefDetail> details;

    size_t rejected() const {
      return rejected_shape + rejected_inactive_window + rejected_exception;
    }

    void addDetail(const AddOdometryBeliefDetail& detail) {
      details.push_back(detail);
      switch (detail.status) {
        case AddOdometryBeliefStatus::Accepted:
          ++accepted;
          break;
        case AddOdometryBeliefStatus::AcceptedButSkippedAlreadyApplied:
          ++accepted_but_skipped_already_applied;
          break;
        case AddOdometryBeliefStatus::RejectedShape:
          ++rejected_shape;
          break;
        case AddOdometryBeliefStatus::RejectedInactiveWindow:
          ++rejected_inactive_window;
          break;
        case AddOdometryBeliefStatus::Exception:
          ++rejected_exception;
          break;
      }
    }
  };

  explicit BPSAM(const Params& params);
  BPSAM(const BPSAM& other);
  BPSAM& operator=(const BPSAM& other);

  const NonlinearFactor::shared_ptr getFactor(FactorIndex factor_index) const {
    const auto& factors = BaseSAM::getFactorsUnsafe();
    if (factor_index >= factors.size() || !factors.exists(factor_index)) {
      return nullptr;
    }
    return factors.at(factor_index);
  }

  /**
   * @brief all keys, excluding consensus keys, including gbp keys
   *
   * @return KeyVector
   */
  KeySet allKeys() const;
  KeyList allKeysInSAM() const {
    KeyList keys;
    for (const auto& [key, _] : this->getVariableIndex()) {
      keys.push_back(key);
    }
    return keys;
  }

  auto beliefGraph() const { return gbp_graph_; }

  bool existsInGbpGraph(Symbol symbol) const;

  static Values getInvolvedValues(const Values& values, const KeySet& keys);

  AddBeliefsResult addBeliefsDetailed(
      std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs);

  AddBeliefsResult addBeliefsDetailed(
      std::map<Key, std::map<AgentId, gbp::Gaussian>> beliefs) {
    std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs_vec;
    for (auto const& [key, belief] : beliefs) {
      for (auto const& [agent_id, gauss] : belief) {
        beliefs_vec[key].emplace_back(agent_id, gauss);
      }
    }
    return addBeliefsDetailed(beliefs_vec);
  }

  int addBeliefs(
      std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs);

  AddOdometryBeliefsResult addOdometryBeliefsDetailed(
      std::vector<CbsOdometryBelief> beliefs);

  int addOdometryBeliefs(std::vector<CbsOdometryBelief> beliefs) {
    return static_cast<int>(
        addOdometryBeliefsDetailed(std::move(beliefs)).rejected());
  }

  int addBeliefs(std::map<Key, std::map<AgentId, gbp::Gaussian>> beliefs) {
    std::map<Key, std::vector<std::pair<AgentId, gbp::Gaussian>>> beliefs_vec;
    for (auto const& [key, belief] : beliefs) {
      for (auto const& [agent_id, gauss] : belief) {
        beliefs_vec[key].emplace_back(agent_id, gauss);
      }
    }
    return addBeliefs(beliefs_vec);
  }

  static const char* addBeliefStatusName(AddBeliefStatus status);
  static const char* metricTypeName(gbp::MetricType metric_type);

  auto getCovariance(Key key) { return this->marginalCovariance(key); }

  auto getSigmas(Key key) {
    auto cov = getCovariance(key);
    return cov.diagonal().array().sqrt();
  }

  bool addGBPNodePose(AgentId agent_id, Key symbol, gbp::Gaussian prior);

  void pruneTrackedCbsFactors(bool prune_active_window_factors = true);

  bool isTrackedCbsFactorSlot(FactorIndex factor_index,
                             const NonlinearFactor::shared_ptr& factor) const;

  size_t rootFrontalKeyCount() const;

  size_t rootCliqueCount() const;

  size_t trackedCbsFactorSlotCount() const {
    return active_cbs_odom_slots_.size();
  }
  NonlinearFactorGraph trackedCbsOdomFactors() const;

  CbsOdomFactorMode effectiveCbsOdomFactorMode() const;

  bool usesActiveWindowTemporaryCbsOdomFactors() const {
    return effectiveCbsOdomFactorMode() ==
           CbsOdomFactorMode::ActiveWindowTemporary;
  }

  struct ActiveWindowCbsOdomFactorRemoval {
    AgentId source_agent = 0;
    Key from_pose_key = 0;
    Key to_pose_key = 0;
    Key sender_from_pose_key = 0;
    Key sender_to_pose_key = 0;
    FactorIndex factor_index = 0;
    bool factor_slot_valid = false;
    std::string reason;
  };

  std::vector<ActiveWindowCbsOdomFactorRemoval>
  activeWindowTemporaryCbsOdomFactorRemovalsOutsideKeys(
      const KeySet& active_keys) const;
  FactorIndices activeWindowTemporaryCbsOdomFactorRemovalSlots(
      const std::vector<ActiveWindowCbsOdomFactorRemoval>& removals) const;
  size_t commitActiveWindowTemporaryCbsOdomFactorRemovals(
      const std::vector<ActiveWindowCbsOdomFactorRemoval>& removals);
  size_t activeWindowTemporaryCbsOdomFactorCount() const {
    return active_window_cbs_odom_factors_.size();
  }
  NonlinearFactorGraph activeWindowTemporaryCbsOdomFactors() const;

  void logActiveCbsOdomFactorDiagnostics();

  FactorIndices cbsFactorSlotsOutsideKeys(const KeySet& active_keys) const;
  size_t pruneActiveWindowTemporaryCbsOdomFactorsOutsideKeys(
      const KeySet& active_keys);

  ISAM2Result update(const NonlinearFactorGraph& newFactors = {},
                     const Values& newTheta = {},
                     const UpdateParams& update_params = UpdateParams());

  ISAM2Result update(
      const NonlinearFactorGraph& newFactors,
      const Values& newTheta,
      const FactorIndices& removeFactorIndices,
      const std::optional<FastMap<Key, int>>& constrainedKeys = std::nullopt,
      const std::optional<FastList<Key>>& noRelinKeys = std::nullopt,
      const std::optional<FastList<Key>>& extraReelimKeys = std::nullopt,
      bool force_relinearize = false);

  // Diagnostic shadow solves must not share mutable smart-factor caches with
  // the production BPSAM instance copied from them.
  void isolateNonlinearFactorsForAudit() {
    nonlinearFactors_ = nonlinearFactors_.clone();
  }

  KeySet keysInGBP() {
    KeySet keys;
    for (auto const& [agent_id, graph] : gbp_graph_) {
      for (auto const& [key, _] : graph.vars()) {
        keys.insert(key);
      }
    }
    return keys;
  }

  bool factorExists(const NonlinearFactor::shared_ptr& factor) const {
    CHECK(factor);
    CHECK(factor->keys().size() > 0);
    auto key0 = factor->keys().at(0);
    auto factors_it = this->getVariableIndex().find(key0);
    if (factors_it == this->getVariableIndex().end()) return false;
    for (auto factor_index : factors_it->second) {
      if (getFactor(factor_index) == factor) return true;
    }
    return false;
  }

  std::map<Key, std::map<AgentId, gbp::Gaussian>> getBeliefs(
      KeySet keys,
      bool use_other_robots = false) const;

  /**
   * @brief Get beliefs for a specific requesting robot, filtering out beliefs
   * that are too similar under the selected metric
   * (distance < belief_similarity_threshold) to the last beliefs sent to that
   * robot. The bookkeeping is updated with whatever is actually returned.
   */
  std::map<Key, std::map<AgentId, gbp::Gaussian>> getBeliefs(
      KeySet keys,
      AgentId requesting_robot,
      bool use_other_robots = false) const;

  std::vector<CbsOdometryBelief> getOdometryBeliefs(
      KeySet keys,
      AgentId requesting_robot) const;

  KeySet poseKeys() const {
    KeySet keys;
    for (auto const& [key, _] : this->getVariableIndex()) {
      if (isPoseKey(LabeledSymbol(key))) {
        keys.insert(key);
      }
    }
    return keys;
  }

  Matrix marginalCovariance(Key key) const {
    if (not tmp_marginals_) {
      return this->BaseSAM::marginalCovariance(key);
    }

    return tmp_marginals_->marginalCovariance(key);
  }

  gtsam::JointMarginal jointMarginalInformation(const KeyVector& keys) const;

  KeyList allRobotKeys() const {
    KeyList keys;
    for (auto const& [key, _] : this->getVariableIndex()) {
      if (isRobotKey(key)) {
        keys.emplace_back(key);
      }
    }
    return keys;
  }

  AgentId id() const { return params_.robot_id; }

  auto marginalizationFactors() const {
    if (not tmp_marginals_) {
      return GaussianFactorGraph{};
    }
    return tmp_marginals_->graph();
  }

  size_t marginalizationGraphFactorCount() const {
    return marginalization_graph_factor_count_;
  }

  class MarginalsWithGraph : public gtsam::Marginals {
   public:
    MarginalsWithGraph(const NonlinearFactorGraph& factors,
                       const Values& values)
        : gtsam::Marginals(factors, values) {}

    const GaussianFactorGraph& graph() const { return graph_; }
  };

  enum class MarginalizationType { LOCAL, GLOBAL, FULL };
  void setMarginalizationGraph(MarginalizationType type);
  void setMarginalizationGraph(MarginalizationType type,
                               const KeySet& active_keys);

  std::map<AgentId, std::map<Key, double>> getDivergences() {
    // traver through the gbp graph and collect the Dxy of each key
    std::map<AgentId, std::map<Key, double>> divergences;
    for (auto const& [agent_id, graph] : gbp_graph_) {
      for (auto const& [key, var] : graph.vars()) {
        divergences[agent_id][key] = var->dxy();
      }
    }
    return divergences;
  }

  const Params& getParams() const { return params_; }

  bool saveLocalGraphG2o(const std::filesystem::path& filename,
                         bool remove_label) const {
    // traverse all factors and pick ones doesn't connect to anchors
    NonlinearFactorGraph local_graph;
    for (const auto& factor : this->getFactorsUnsafe()) {
      if (not factor) continue;
      bool is_local = true;
      for (const auto& key : factor->keys()) {
        if (isRobotKey(key)) {
          is_local = false;
          break;
        }
      }
      if (is_local) {
        local_graph.push_back(factor);
      }
    }

    // if remove char, then convert labeled symbols to normal symbols
    NonlinearFactorGraph rekeyed_graph;
    Values rekeyed_values;
    if (remove_label) {
      auto rekey = [](Key key) {
        auto labeled_key = LabeledSymbol(key);
        if (isPoseKey(labeled_key)) {
          return Symbol(labeled_key.chr(), labeled_key.index());
        } else {
          throw std::runtime_error(
              "shouln't reach here in BPSAM::saveLocalGraphG2o");
        }
      };

      std::map<Key, Key> rekey_map;
      for (const auto& key : local_graph.keys()) {
        rekey_map[key] = rekey(key);
      }

      rekeyed_graph = local_graph.rekey(rekey_map);

      for (const auto& [key, value] : this->calculateEstimate()) {
        if (rekey_map.find(key) != rekey_map.end()) {
          rekeyed_values.insert(rekey_map.at(key), value);
        }
      }
    } else {
      rekeyed_graph = local_graph;
      rekeyed_values = this->calculateEstimate();
    }

    // check if filename parent path exists, and throw error if not
    auto parent_path = filename.parent_path();
    if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
      throw std::runtime_error("BPSAM::saveLocalGraphG2o: Parent path " +
                               parent_path.string() + " does not exist.");
    }

    // check if filename exists, then remove it
    if (std::filesystem::exists(filename)) {
      std::filesystem::remove(filename);
    }

    cbs::writeG2oNoIndex(rekeyed_graph, rekeyed_values, filename.string());
    return true;
  }

 protected:
  void initializeBeliefMetric();
  void setMarginalizationGraph(MarginalizationType type,
                               const KeySet* active_keys);
  Ordering bayesTreeEliminationOrdering() const;
  void updateDeltaWithTemporaryFactors(
      const NonlinearFactorGraph& temporary_factors,
      const std::vector<std::string>& temporary_factor_diagnostics,
      bool force_full_solve) const;
  ISAM2Result commitDeltaAndRelinearizeClean(
      const ISAM2UpdateParams& update_params);

  Params params_;

  // map<"sending agent", graph>
  using AgentBPGraph = std::map<AgentId, gbp::Graph>;
  struct CbsEndpointBeliefKey {
    AgentId source_agent;
    Key pose_key;

    bool operator<(const CbsEndpointBeliefKey& other) const {
      return std::tie(source_agent, pose_key) <
             std::tie(other.source_agent, other.pose_key);
    }
  };

  struct CbsOdomReceiverKey {
    AgentId source_agent;
    Key from_pose_key;
    Key to_pose_key;

    bool operator<(const CbsOdomReceiverKey& other) const {
      return std::tie(source_agent, from_pose_key, to_pose_key) <
             std::tie(other.source_agent,
                      other.from_pose_key,
                      other.to_pose_key);
    }

    bool operator==(const CbsOdomReceiverKey& other) const {
      return source_agent == other.source_agent &&
             from_pose_key == other.from_pose_key &&
             to_pose_key == other.to_pose_key;
    }
  };

  struct CbsOdomKey {
    AgentId source_agent;
    // Receiver-side edge. These keys are used to build the factor.
    Key from_pose_key;
    Key to_pose_key;
    // Sender-side edge. These keys identify the belief across repeated sender
    // windows so different sender beliefs that match the same receiver edge do
    // not collide in the duplicate guard.
    Key sender_from_pose_key = 0;
    Key sender_to_pose_key = 0;

    Key identityFromPoseKey() const {
      return sender_from_pose_key == 0 ? from_pose_key : sender_from_pose_key;
    }

    Key identityToPoseKey() const {
      return sender_to_pose_key == 0 ? to_pose_key : sender_to_pose_key;
    }

    CbsOdomReceiverKey receiverKey() const {
      return CbsOdomReceiverKey{source_agent, from_pose_key, to_pose_key};
    }

    bool operator<(const CbsOdomKey& other) const {
      return std::make_tuple(source_agent,
                             identityFromPoseKey(),
                             identityToPoseKey(),
                             from_pose_key,
                             to_pose_key) <
             std::make_tuple(other.source_agent,
                             other.identityFromPoseKey(),
                             other.identityToPoseKey(),
                             other.from_pose_key,
                             other.to_pose_key);
    }

    bool operator==(const CbsOdomKey& other) const {
      return source_agent == other.source_agent &&
             from_pose_key == other.from_pose_key &&
             to_pose_key == other.to_pose_key &&
             sender_from_pose_key == other.sender_from_pose_key &&
             sender_to_pose_key == other.sender_to_pose_key;
    }
  };

  struct CbsOdomBelief {
    CbsOdomKey key;
    std::optional<gbp::Gaussian> from_belief;
    std::optional<gbp::Gaussian> to_belief;
    Pose3 measured_from_to;
    Matrix covariance;
    Matrix raw_covariance;
    double relax_factor = 0.0;
    bool sender_computed = false;
  };

  struct TrackedCbsFactorSlot {
    FactorIndex factor_index;
    NonlinearFactor::shared_ptr factor;
  };

  struct ActiveWindowCbsOdomFactor {
    CbsOdomBelief belief;
    NonlinearFactor::shared_ptr factor;
    size_t active_update_count = 0u;
  };

  AgentBPGraph gbp_graph_;
  Values gbp_values_;    // gbp values
  Values T_r_r_values_;  // Trr values
  std::map<CbsOdomKey, CbsOdomBelief> pending_cbs_odom_beliefs_;
  std::map<CbsOdomReceiverKey, CbsOdomKey>
      pending_active_window_cbs_odom_keys_by_receiver_;
  std::map<CbsOdomKey, TrackedCbsFactorSlot> active_cbs_odom_slots_;
  std::map<CbsOdomReceiverKey, CbsOdomKey>
      active_window_cbs_odom_keys_by_receiver_;
  std::map<CbsOdomKey, ActiveWindowCbsOdomFactor>
      active_window_cbs_odom_factors_;
  std::map<CbsEndpointBeliefKey, gbp::Gaussian>
      last_applied_temporary_cbs_linear_beliefs_;
  std::map<CbsOdomKey, CbsOdomBelief>
      last_applied_temporary_cbs_linear_odom_beliefs_;
  std::map<AgentId, std::map<Key, gbp::Gaussian>>
      accepted_odom_pose_beliefs_by_source_;
  std::map<AgentId, health_aware::RelativeTrustCalibrationState>
      relative_trust_states_by_source_;

  mutable std::shared_ptr<MarginalsWithGraph> tmp_marginals_;
  size_t marginalization_graph_factor_count_ = 0u;

  // Bookkeeping: last_sent_beliefs_[requesting_robot][(belief_key,
  // producing_agent)] stores the Gaussian most recently returned by
  // getBeliefs() for that robot.
  mutable std::map<AgentId, std::map<std::pair<Key, AgentId>, gbp::Gaussian>>
      last_sent_beliefs_;
  mutable size_t relative_covariance_matrix_dump_count_ = 0u;
  mutable size_t relative_covariance_matrix_dump_seen_count_ = 0u;
  mutable std::set<std::tuple<AgentId, Key, Key>>
      relative_covariance_matrix_dumped_edges_;
  mutable size_t covariance_exchange_audit_count_ = 0u;
  mutable size_t covariance_exchange_audit_seen_count_ = 0u;
  // Successful audit samples and all distinct eligible edges seen by the
  // stride selector. Rolling CBS arrays may republish one stable edge many
  // times; the seen set prevents those repetitions from consuming the stride.
  mutable std::set<CbsOdomKey> covariance_exchange_audited_edges_;
  mutable std::set<CbsOdomKey> covariance_exchange_audit_seen_edges_;

  // Bookkeeping: last_received_beliefs_[source_agent][belief_key] stores the
  // raw incoming Gaussian most recently received from that source, before GBP
  // contraction has a chance to accept, damp, or reject it.
  std::map<AgentId, std::map<Key, gbp::Gaussian>> last_received_beliefs_;

  // Metric used by the robot-aware getBeliefs() overload to decide whether a
  // belief has changed enough to be worth resending.
  std::unique_ptr<gbp::BeliefMetric> belief_metric_;
};

}  // namespace cbs
