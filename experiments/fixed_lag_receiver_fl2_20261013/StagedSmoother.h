// Isolated FL2 staging prototype. Not wired into production or the journal runner.
#pragma once
#include "solver/IncrementalFixedLagSmoother.h"
#include "MarginalAudit.h"
#include <map>
#include <stdexcept>
#include <iostream>

namespace fl2 {
inline gtsam::ISAM2Params Parameters() {
  gtsam::ISAM2Params p; p.relinearizeThreshold=.01; p.relinearizeSkip=1;
  p.findUnusedFactorSlots=true; p.factorization=gtsam::ISAM2Params::QR;return p;
}
class StagedSmoother : public fgo::solvers::IncrementalFixedLagSmoother {
 public:
  using Parent=fgo::solvers::IncrementalFixedLagSmoother;
  using Parent::KeyTimestampMap;
  struct Marginal {gtsam::NonlinearFactor::shared_ptr factor; gtsam::Values anchor;};
  std::map<size_t,Marginal> marginals;
  size_t publicationCursor=0;
  // Metadata units: exact integer support ticks, 0.1 s each. Dynamics use seconds.
  explicit StagedSmoother(double lagSeconds=7.):Parent(lagSeconds*10.,Parameters()) {
    setNotMarginalizing();
  }
  StagedSmoother(const StagedSmoother& b):Parent(b.smootherLag_,b.params()) {restore(b);}
  void restore(const StagedSmoother& b) {
    smootherLag_=b.smootherLag_; smootherLagInflation_=b.smootherLagInflation_;
    notMarginalizing_.store(b.notMarginalizing_.load());
    timestampKeyMap_=b.timestampKeyMap_; keyTimestampMap_=b.keyTimestampMap_;
    isam_=b.isam_; isamResult_=b.isamResult_; marginals=b.marginals;
    publicationCursor=b.publicationCursor;
  }
  const gtsam::ISAM2& isam() const {return isam_;}
  gtsam::Values estimate() const {return isam_.calculateBestEstimate();}
  bool disabled() const {return notMarginalizing_.load();}
  gtsam::FactorIndices add(const gtsam::NonlinearFactorGraph& g,const gtsam::Values& v={},
                          const KeyTimestampMap& t={},const gtsam::FactorIndices& remove={}) {
    setNotMarginalizing(); Parent::update(g,v,t,remove);
    return isamResult_.newFactorsIndices;
  }
  void captureMarginals() {
    std::map<size_t,Marginal> next; const auto& graph=getFactors();
    for(size_t i=0;i<graph.size();++i) {
      const auto* f=dynamic_cast<const gtsam::LinearContainerFactor*>(graph[i].get());
      if(!f)continue;
      const auto old=marginals.find(i);
      if(old!=marginals.end() && old->second.factor==graph[i]) {
        for(auto k:f->keys())
          if(!old->second.anchor.at(k).equals_(getLinearizationPoint().at(k),1e-14))
            throw std::runtime_error("marginal anchor unexpectedly moved");
        next.emplace(i,old->second);
      } else next.emplace(i,Marginal{graph[i],Subset(
           f->hasLinearizationPoint()?*f->linearizationPoint():getLinearizationPoint(),f->keys())});
    }
    marginals=std::move(next);
  }
  // Calls existing constrained ordering + marginalizeLeaves, not a second solver.
  void retire() {
    setMarginalizing(); Parent::update(); setNotMarginalizing(); captureMarginals();
  }
  gtsam::NonlinearFactorGraph auditGraph() const {
    gtsam::NonlinearFactorGraph out;
    const auto& graph=getFactors();
    for(size_t i=0;i<graph.size();++i) {
      if(!graph[i])continue;
      const auto* f=dynamic_cast<const gtsam::LinearContainerFactor*>(graph[i].get());
      if(f) out.emplace_shared<ChartMarginal>(f->factor(),marginals.at(i).anchor);
      else out.push_back(graph[i]);
    }
    return out;
  }
  struct RefineResult {bool passed=false; size_t iterations=0;double before=0,after=0;};
  RefineResult refine() {
    const auto graph=auditGraph(); RefineResult r;
    r.before=Gradient(graph,estimate());r.after=r.before;
    for(;r.iterations<20 && r.after>1e-5;++r.iterations) {
      gtsam::ISAM2 candidate(isam_);
      gtsam::ISAM2UpdateParams p;p.force_relinearize=true;p.forceFullSolve=true;
      const auto result=candidate.update({}, {},p);
      const auto value=candidate.calculateBestEstimate();
      const double previous=graph.error(estimate()),cost=graph.error(value);
      std::cout<<"REFINE "<<r.iterations<<" previous="<<previous<<" proposed="<<cost
               <<" gradient="<<Gradient(graph,value)<<'\n';
      if(!std::isfinite(cost) || cost>previous+1e-12*(1+std::abs(previous))) break;
      isam_=candidate; isamResult_=result;r.after=Gradient(graph,value);
    }
    r.passed=std::isfinite(r.after)&&r.after<=1e-5;return r;
  }
  template<class Fn> void transaction(Fn fn) {
    StagedSmoother saved(*this);
    try{fn();}catch(...){restore(saved);throw;}
  }
};
} // namespace fl2
