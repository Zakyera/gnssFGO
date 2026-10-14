#pragma once
#include "../fixed_lag_receiver_fl2_20261013/StagedSmoother.h"
#include "AnchoredMarginal.h"
#include <functional>

namespace fl2a {
inline double CheckedGradient(const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& value) {
  const auto linear=graph.linearize(value);const auto g=linear->gradientAtZero(),d=linear->hessianDiagonal();
  double result=0;
  for(const auto& kv:g) {
    if(!kv.second.allFinite()||!d.at(kv.first).allFinite()||(d.at(kv.first).array()<0).any())
      throw std::runtime_error("nonfinite or invalid publication gradient");
    for(Eigen::Index i=0;i<kv.second.size();++i)
      result=std::max(result,std::abs(kv.second[i])/std::sqrt(std::max(1e-30,d.at(kv.first)[i])));
  }
  if(!std::isfinite(graph.error(value)))throw std::runtime_error("nonfinite publication cost");
  return result;
}
// Preserve holes during fresh loading; restore reuse before any ordinary update.
// No tree internals or fixed-variable sets are changed through this subclass.
class SlotLoadingISAM : public gtsam::ISAM2 {
  static gtsam::ISAM2Params Loading(gtsam::ISAM2Params p){p.findUnusedFactorSlots=false;return p;}
 public:
  explicit SlotLoadingISAM(const gtsam::ISAM2Params& p):gtsam::ISAM2(Loading(p)){}
  void enableReuse(){params_.findUnusedFactorSlots=true;}
};

class BoundarySmoother : public fl2::StagedSmoother {
 public:
  size_t windowRebuilds=0,convertedMarginals=0,retiredBlocks=0;
  enum class Failure {None,AfterRetirement,AfterRebuild,BeforeCommit};
  explicit BoundarySmoother(double lagSeconds=7.):fl2::StagedSmoother(lagSeconds){}
  explicit BoundarySmoother(const fl2::StagedSmoother& s):fl2::StagedSmoother(s){}
  BoundarySmoother(const BoundarySmoother& s):fl2::StagedSmoother(s),
    windowRebuilds(s.windowRebuilds),convertedMarginals(s.convertedMarginals),retiredBlocks(s.retiredBlocks){}
  void restore(const BoundarySmoother& b){fl2::StagedSmoother::restore(b);
    windowRebuilds=b.windowRebuilds;convertedMarginals=b.convertedMarginals;retiredBlocks=b.retiredBlocks;}
  template<class Fn>void transaction(Fn fn){BoundarySmoother saved(*this);try{fn();}catch(...){restore(saved);throw;}}

  // For an already-marginalized snapshot as well as a new retirement boundary.
  // Returns count of newly translated marginals. Only retained data is loaded.
  size_t adoptCharts() {
    const auto seed=estimate();const auto originalGraph=auditGraph();
    auto graph=getFactors();size_t converted=0;
    std::vector<size_t> newMarginalSlots;
    for(size_t slot=0;slot<graph.size();++slot) {
      const auto* raw=dynamic_cast<const gtsam::LinearContainerFactor*>(graph[slot].get());
      if(!raw)continue;
      const auto& m=marginals.at(slot);
      if(m.factor!=graph[slot])throw std::runtime_error("marginal slot owner mismatch");
      graph[slot]=boost::make_shared<AnchoredMarginal>(raw->factor(),m.anchor);++converted;
      newMarginalSlots.push_back(slot);
    }
    if(!converted)return 0;
    // marginalizeLeaves appends its marginal(s), independently of the ordinary
    // insertion-slot reuse policy. Move ONLY these new receiver-owned marginals
    // into lower vacant slots; source-owned slots must never move. Then trim
    // empty tail storage, otherwise graph.size() grows with total history.
    for(size_t from:newMarginalSlots) {
      size_t to=0;while(to<from&&graph[to])++to;
      if(to<from){graph[to]=graph[from];graph[from].reset();}
    }
    while(!graph.empty()&&!graph.back())graph.resize(graph.size()-1);
    const double oldCost=originalGraph.error(seed),newCost=graph.error(seed);
    if(std::abs(oldCost-newCost)>1e-9+1e-10*std::abs(oldCost))
      throw std::runtime_error("chart conversion changed objective");
    SlotLoadingISAM rebuilt(params());
    const auto result=rebuilt.update(graph,seed);
    rebuilt.enableReuse();
    if(result.newFactorsIndices.size()!=graph.size())throw std::runtime_error("rebuild slot count");
    for(size_t slot=0;slot<graph.size();++slot)
      if(result.newFactorsIndices[slot]!=slot||rebuilt.getFactorsUnsafe()[slot]!=graph[slot])
        throw std::runtime_error("rebuild changed source slot ownership");
    if(rebuilt.getLinearizationPoint().keys()!=seed.keys()||!rebuilt.getFixedVariables().empty())
      throw std::runtime_error("rebuild invalid retained state set");
    const double cost=graph.error(rebuilt.calculateBestEstimate());
    if(!std::isfinite(cost)||cost>newCost+1e-12*(1+std::abs(newCost)))
      throw std::runtime_error("retained-window load increases objective");
    // No exception-prone validation after committing the staged replacement.
    isam_=rebuilt;isamResult_=result;marginals.clear();
    ++windowRebuilds;convertedMarginals+=converted;return converted;
  }
  struct SolveResult {bool passed=false;size_t steps=0;double before=0,after=0;};
  SolveResult solve(size_t budget=20) {
    const auto graph=auditGraph();SolveResult r;
    r.before=CheckedGradient(graph,estimate());r.after=r.before;
    while(r.steps<budget&&r.after>1e-5) {
      gtsam::ISAM2 trial(isam_);gtsam::ISAM2UpdateParams p;
      p.force_relinearize=true;p.forceFullSolve=true;
      const auto result=trial.update({}, {},p);++r.steps;
      const auto value=trial.calculateBestEstimate();
      const double old=graph.error(estimate()),next=graph.error(value);
      if(!std::isfinite(next)||next>old+1e-12*(1+std::abs(old)))break;
      isam_=trial;isamResult_=result;r.after=CheckedGradient(graph,value);
    }
    r.passed=std::isfinite(r.after)&&r.after<=1e-5;return r;
  }
  // Call after support insertion and ordered pending-event dispatch. Its outer
  // receiver transaction must include those admissions too; this is solver-only.
  SolveResult publish(Failure failure=Failure::None) {
    SolveResult result;
    transaction([&]{
      result=solve();if(!result.passed)throw std::runtime_error("pre-retirement stationarity");
      const double cutoff=getCurrentTimestamp()-smootherLag_;
      const bool due=!keyTimestampMap_.empty()&&timestampKeyMap_.begin()->first<cutoff;
      if(due) {
        if(result.steps+2>20)throw std::runtime_error("retirement publication budget");
        const size_t old=keyTimestampMap_.size();
        fl2::StagedSmoother::retire();++result.steps;
        if(failure==Failure::AfterRetirement)throw std::runtime_error("injected_after_retirement");
        const auto converted=adoptCharts();if(converted)++result.steps;
        retiredBlocks+=old-keyTimestampMap_.size();
        if(failure==Failure::AfterRebuild)throw std::runtime_error("injected_after_rebuild");
        const auto post=solve(20-result.steps);result.steps+=post.steps;result.after=post.after;
        result.passed=post.passed;
        if(!result.passed)throw std::runtime_error("post-retirement stationarity");
      }
      if(failure==Failure::BeforeCommit)throw std::runtime_error("injected_before_commit");
      ++publicationCursor;
    });return result;
  }
  // Independent NUMERIC-chart view used only in tests, not optimization.
  gtsam::NonlinearFactorGraph referenceGraph() const {
    gtsam::NonlinearFactorGraph graph;
    for(const auto& factor:getFactors()) {
      if(!factor)continue;
      if(auto* p=dynamic_cast<const AnchoredMarginal*>(factor.get()))
        graph.emplace_shared<fl2::ChartMarginal>(p->gaussian().clone(),p->anchor());
      else if(dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get()))
        throw std::runtime_error("untranslated raw marginal in reference");
      else graph.push_back(factor);
    }
    return graph;
  }
};
} // namespace fl2a
