#pragma once
#include "RecordedFixedLagSmoother.h"
#include "RetirementCapture.h"
#include "../fixed_lag_receiver_fl2_20261013/MarginalAudit.h"
#include <map>

namespace fl3i_solver {
inline gtsam::ISAM2Params Parameters() {
  gtsam::ISAM2Params p;p.relinearizeThreshold=.01;p.relinearizeSkip=1;
  p.findUnusedFactorSlots=true;p.factorization=gtsam::ISAM2Params::QR;return p;
}
class NativeSmoother : public fgo::solvers::SlotFixedLagSmoother {
 public:
  using Parent=fgo::solvers::SlotFixedLagSmoother;
  using Parent::KeyTimestampMap;
  struct Marginal {gtsam::NonlinearFactor::shared_ptr factor;gtsam::Values anchor;};
  std::map<size_t,Marginal> marginals;
  size_t publicationCursor=0,retirements=0,retiredBlocks=0;
  // Explicit compatibility/audit counters: these must remain zero.
  size_t windowRebuilds=0,convertedMarginals=0;
  explicit NativeSmoother(double lag):Parent(lag*10.,Parameters()){setNotMarginalizing();}
  NativeSmoother(const NativeSmoother& b):Parent(b.smootherLag_,b.params()) {
    smootherLag_=b.smootherLag_;smootherLagInflation_=b.smootherLagInflation_;
    notMarginalizing_.store(b.notMarginalizing_.load());
    timestampKeyMap_=b.timestampKeyMap_;keyTimestampMap_=b.keyTimestampMap_;
    isam_=b.isam_;isamResult_=b.isamResult_;marginals=b.marginals;
    publicationCursor=b.publicationCursor;retirements=b.retirements;retiredBlocks=b.retiredBlocks;
  }
  const gtsam::ISAM2& isam()const{return isam_;}
  gtsam::Values estimate()const{return isam_.calculateEstimate();}
  gtsam::FactorIndices add(const gtsam::NonlinearFactorGraph& g,const gtsam::Values& v={},
      const KeyTimestampMap& t={},const gtsam::FactorIndices& remove={}) {
    setNotMarginalizing();Parent::update(g,v,t,remove);return isamResult_.newFactorsIndices;
  }
  void captureMarginals() {
    std::map<size_t,Marginal> next;const auto& graph=getFactors();
    for(size_t i=0;i<graph.size();++i) {
      const auto* f=dynamic_cast<const gtsam::LinearContainerFactor*>(graph[i].get());if(!f)continue;
      const auto old=marginals.find(i);
      if(old!=marginals.end()&&old->second.factor==graph[i]) {
        for(auto k:f->keys())if(!old->second.anchor.at(k).equals_(getLinearizationPoint().at(k),1e-14))
          throw std::runtime_error("native fixed marginal anchor moved");
        next.emplace(i,old->second);
      } else next.emplace(i,Marginal{graph[i],fl2::Subset(
          f->hasLinearizationPoint()?*f->linearizationPoint():getLinearizationPoint(),f->keys())});
    }
    marginals=std::move(next);
  }
  gtsam::Values retire() {
    if(fl3i_capture::before)throw std::runtime_error("nested retirement callback");
    gtsam::Values outgoing;size_t calls=0;
    fl3i_capture::before=[&](const gtsam::ISAM2& isam) {
      gtsam::ISAM2 copy(isam);outgoing=copy.calculateEstimate();++calls;
    };
    try {setMarginalizing();Parent::update();setNotMarginalizing();fl3i_capture::before={};}
    catch(...){setNotMarginalizing();fl3i_capture::before={};throw;}
    if(calls!=1)throw std::runtime_error("retirement capture not exactly once");
    captureMarginals();++retirements;return outgoing;
  }
  // Diagnostic on a disposable copy only; never used by optimization/publication.
  gtsam::NonlinearFactorGraph auditGraph()const {
    gtsam::NonlinearFactorGraph out;const auto& graph=getFactors();
    for(size_t i=0;i<graph.size();++i) {
      if(!graph[i])continue;
      if(auto* f=dynamic_cast<const gtsam::LinearContainerFactor*>(graph[i].get()))
        out.emplace_shared<fl2::ChartMarginal>(f->factor(),marginals.at(i).anchor);
      else out.push_back(graph[i]);
    }return out;
  }
};
}
