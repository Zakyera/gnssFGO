#pragma once
#include "ExactGpuSnapshot.h"
#include "LocalUpdateBatching.h"
#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <cbs/utils/relative_pose_covariance.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianConditional.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gtsam {
class LocalSnapshotAccess {
 public:
  static void detach(ISAM2& solver,e1b4check::FactorCopies& copies) {
    solver.nonlinearFactors_=copies.graph(solver.nonlinearFactors_);
    for(auto& f:solver.linearFactors_)if(f)f=f->clone();
    // BayesTree's copy constructor clones cliques/links. Detach its numerical
    // conditionals and cached factors too. BayesTreeCliqueBase's copy omits
    // its lazily reconstructed separator cache; it is not shared or evaluated
    // to create the snapshot.
    for(auto& entry:solver.nodes()) {
      auto& c=*entry.second;
      c.conditional()=std::make_shared<GaussianConditional>(*c.conditional());
      if(c.cachedFactor())c.cachedFactor()=c.cachedFactor()->clone();
    }
  }
};
}
namespace cbs {
class LocalSnapshotAccess {
 public:
  static std::unique_ptr<IncrementalFixedLagBpsamSmoother> copy(
      const IncrementalFixedLagBpsamSmoother& s,e1b4check::FactorCopies& factors) {
    const auto& b=s.isam_;
    // The actual clean LOCAL sidecar does not receive GBP/external beliefs.
    // Refuse other BPSAM uses instead of silently shallow-copying those graphs.
    if(!b.gbp_graph_.empty() || !b.pending_cbs_odom_beliefs_.empty() ||
       !b.active_cbs_odom_slots_.empty() || !b.active_window_cbs_odom_factors_.empty() ||
       !b.relative_trust_states_by_source_.empty() ||
       s.unmarginalized_reference_isam_ || s.shadow_acceptance_audit_out_ ||
       s.shadow_acceptance_initialized_values_ || s.capture_bad_update_stages_once_ ||
       s.bad_update_stage_audit_)
      throw std::invalid_argument("snapshot supports clean LOCAL only");
    auto out=std::make_unique<IncrementalFixedLagBpsamSmoother>(s);
    gtsam::LocalSnapshotAccess::detach(out->isam_,factors);
    // BPSAM copy intentionally drops only lazily reconstructed marginals.
    // They are not used by LOCAL update; diagnostics reconstruct independently.
    return out;
  }
};
}
namespace e1b4check {
using Smoother=cbs::IncrementalFixedLagBpsamSmoother;
struct Differences {
  double position=0,orientation=0,velocity=0,bias=0,gaussian=0,covariance=0,relative=0;
  size_t states=0,slots=0;
};
inline void require(bool ok,const char* reason) {
  if(!ok)throw std::runtime_error(reason);
}
inline double matrixDiff(const gtsam::Matrix& a,const gtsam::Matrix& b,
                         double abs,double rel) {
  require(a.rows()==b.rows() && a.cols()==b.cols(),"matrix shape mismatch");
  require(a.allFinite() && b.allFinite(),"nonfinite matrix");
  if(!a.size())return 0;
  double d=(a-b).cwiseAbs().maxCoeff();
  require(d<=abs+rel*a.cwiseAbs().maxCoeff(),"matrix tolerance exceeded");return d;
}
inline void compareValues(const gtsam::Values& a,const gtsam::Values& b,Differences& d) {
  using namespace gtsam;
  require(a.keys()==b.keys(),"state key mismatch");
  for(Key k:a.keys()) {
    // Fail closed on value types rather than guessing from the key prefix.
    const auto& av=a.at(k);const auto& bv=b.at(k);
    require(typeid(av)==typeid(bv),"state type mismatch");
    if(typeid(av)==typeid(GenericValue<Pose3>)) {
      auto x=a.at<Pose3>(k),y=b.at<Pose3>(k);
      require(x.matrix().allFinite() && y.matrix().allFinite(),"nonfinite pose");
      d.position=std::max(d.position,(x.translation()-y.translation()).cwiseAbs().maxCoeff());
      d.orientation=std::max(d.orientation,Rot3::Logmap(x.rotation().between(y.rotation())).norm());
    }else if(typeid(av)==typeid(GenericValue<Vector3>)) {
      require(a.at<Vector3>(k).allFinite() && b.at<Vector3>(k).allFinite(),"nonfinite velocity");
      d.velocity=std::max(d.velocity,(a.at<Vector3>(k)-b.at<Vector3>(k)).cwiseAbs().maxCoeff());
    }else if(typeid(av)==typeid(GenericValue<imuBias::ConstantBias>)) {
      require(a.at<imuBias::ConstantBias>(k).vector().allFinite() &&
              b.at<imuBias::ConstantBias>(k).vector().allFinite(),"nonfinite bias");
      d.bias=std::max(d.bias,(a.at<imuBias::ConstantBias>(k).vector()-b.at<imuBias::ConstantBias>(k).vector()).cwiseAbs().maxCoeff());
    }else throw std::invalid_argument("unsupported LOCAL value type");
  }
  require(std::isfinite(d.position)&&std::isfinite(d.orientation)&&
          std::isfinite(d.velocity)&&std::isfinite(d.bias),"nonfinite state");
  require(d.position<=1e-9 && d.velocity<=1e-9 && d.bias<=1e-9 &&
          d.orientation<=1e-10,"state tolerance exceeded");
}
inline Differences compare(const Smoother& reference,const Smoother& candidate) {
  // Diagnostics never linearize authoritative factors or alter its lazy delta.
  FactorCopies ac,bc;
  auto a=cbs::LocalSnapshotAccess::copy(reference,ac);
  auto b=cbs::LocalSnapshotAccess::copy(candidate,bc);
  Differences d;auto x=a->calculateEstimate(),y=b->calculateEstimate();
  compareValues(x,y,d);compareValues(a->getLinearizationPoint(),b->getLinearizationPoint(),d);
  require(a->timestamps()==b->timestamps(),"timestamp map mismatch");
  require(a->getISAM2Result().variablesRelinearized==b->getISAM2Result().variablesRelinearized,
          "relinearization decision mismatch");
  require(a->getISAM2Result().variablesReeliminated==b->getISAM2Result().variablesReeliminated,
          "reelimination decision mismatch");
  require(a->getISAM2Result().unusedKeys==b->getISAM2Result().unusedKeys,"retired keys mismatch");
  const auto& ar=a->getISAM2Result();const auto& br=b->getISAM2Result();
  require(ar.newFactorsIndices==br.newFactorsIndices && ar.observedKeys==br.observedKeys &&
          ar.keysWithRemovedFactors==br.keysWithRemovedFactors && ar.markedKeys==br.markedKeys,
          "update identity/marked-key mismatch");
  require(ar.factorsRecalculated==br.factorsRecalculated && ar.cliques==br.cliques &&
          ar.treeNnz==br.treeNnz && ar.batchReorderTriggered==br.batchReorderTriggered,
          "solver work/tree decision mismatch");
  const auto& ag=a->getFactors();const auto& bg=b->getFactors();
  require(ag.size()==bg.size(),"factor slot count mismatch");
  // Preserve the identity partition as well as occupied-slot keys/types.
  std::map<const gtsam::NonlinearFactor*,const gtsam::NonlinearFactor*> ab,ba;
  for(size_t i=0;i<ag.size();++i)if(ag[i] && bg[i]) {
    auto aa=ab.emplace(ag[i].get(),bg[i].get()),bb=ba.emplace(bg[i].get(),ag[i].get());
    require(aa.first->second==bg[i].get() && bb.first->second==ag[i].get(),
            "factor alias partition mismatch");
  }
  auto al=ag.linearize(x),bl=bg.linearize(y);
  for(size_t i=0;i<ag.size();++i) {
    require(bool(ag[i])==bool(bg[i]),"occupied slots mismatch");if(!ag[i])continue;
    require(typeid(*ag[i])==typeid(*bg[i]) && ag[i]->keys()==bg[i]->keys(),"factor keys/type mismatch");
    bool gpu=dynamic_cast<gtsam_points::NonlinearFactorGPU*>(ag[i].get());
    d.gaussian=std::max(d.gaussian,matrixDiff((*al)[i]->augmentedInformation(),
        (*bl)[i]->augmentedInformation(),gpu?1e-8:1e-12,gpu?1e-6:1e-12));
  }
  if(!x.empty()) {
    gtsam::Marginals am(*al,x),bm(*bl,y);
    d.covariance=matrixDiff(am.jointMarginalCovariance(x.keys()).fullMatrix(),
        bm.jointMarginalCovariance(y.keys()).fullMatrix(),1e-10,1e-6);
    gtsam::KeyVector poses;
    for(auto k:x.keys())if(typeid(x.at(k))==typeid(gtsam::GenericValue<gtsam::Pose3>))poses.push_back(k);
    if(poses.size()>1) {
      auto ka=poses[poses.size()-2],kb=poses.back();
      auto cov=[&](auto& marginal,const auto& v){
        const auto p=v.template at<gtsam::Pose3>(ka),q=v.template at<gtsam::Pose3>(kb);
        return cbs::relative_pose_covariance::compute(marginal.jointMarginalInformation({ka,kb}),
          ka,kb,p,q,p.between(q),cbs::relative_pose_covariance::Mode::SchurRelativeBetween).covariance;
      };
      d.relative=matrixDiff(cov(am,x),cov(bm,y),1e-10,1e-6);
    }
  }
  d.states=x.size();d.slots=ag.size();return d;
}

// Call before authoritative ordinary update, then finish() after it. No
// production hook is enabled here. Diagnostic runtime must not enter timing.
class LocalUpdateCheck {
 public:
  LocalUpdateCheck(const Smoother& before,const gtsam::NonlinearFactorGraph& factors,
      const gtsam::Values& values,const gtsam::FixedLagSmoother::KeyTimestampMap& stamps,
      const gtsam::FactorIndices& remove={}) {
    ordinary_=cbs::LocalSnapshotAccess::copy(before,ordinary_factors_);
    batch_=cbs::LocalSnapshotAccess::copy(before,batch_factors_);
    auto a=ordinary_factors_.graph(factors),b=batch_factors_.graph(factors);
    ordinary_->update(a,values,stamps,remove);
    {gtsam::e1b4a::ScopedLinearizer scope(glim::e1b4a::linearize,&stats_);
     batch_->update(b,values,stamps,remove);}
    paired_=compare(*ordinary_,*batch_);
  }
  Differences finish(const Smoother& authoritative) const {
    return compare(authoritative,*ordinary_);
  }
  const Differences& paired() const{return paired_;}
  const glim::e1b4a::Stats& stats() const{return stats_;}
 private:
  FactorCopies ordinary_factors_,batch_factors_;
  std::unique_ptr<Smoother> ordinary_,batch_;
  Differences paired_;
  glim::e1b4a::Stats stats_;
};
}
