#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <optional>

namespace fl3a {
struct Step {
  gtsam::Values value;
  double oldCost=0,newCost=0;
  size_t relinearized=0,reeliminated=0;
  bool accepted=false;
};
struct Capture {
  gtsam::NonlinearFactorGraph graph,slots;
  gtsam::Values initial,last;
  std::vector<Step> steps;
  fl2a::BoundarySmoother::SolveResult result;
  std::string stop;
  size_t budget=0,fixedKeys=0;
};
inline std::optional<Capture> capture;
}
namespace fl2a {
// Observer-only. All non-target solve calls delegate to the original method.
class ObservedBoundarySmoother : public BoundarySmoother {
 public:
  using BoundarySmoother::BoundarySmoother;
  SolveResult solve(size_t budget=20) {
    size_t newest=0;for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    if(newest!=1224)return BoundarySmoother::solve(budget);
    if(fl3a::capture)throw std::runtime_error("unexpected second target solve");
    fl3a::capture.emplace();auto& c=*fl3a::capture;
    const auto graph=auditGraph();SolveResult r;
    r.before=CheckedGradient(graph,estimate());r.after=r.before;
    // Copies only; no additional linearization or error evaluation in this loop.
    c.graph=graph;c.slots=getFactors();c.initial=estimate();c.budget=budget;
    c.fixedKeys=isam_.getFixedVariables().size();
    while(r.steps<budget&&r.after>1e-5) {
      gtsam::ISAM2 trial(isam_);gtsam::ISAM2UpdateParams p;
      p.force_relinearize=true;p.forceFullSolve=true;
      const auto result=trial.update({}, {},p);++r.steps;
      const auto value=trial.calculateBestEstimate();
      const double old=graph.error(estimate()),next=graph.error(value);
      const bool rejected=!std::isfinite(next)||next>old+1e-12*(1+std::abs(old));
      c.steps.push_back({value,old,next,result.variablesRelinearized,result.variablesReeliminated,!rejected});
      if(rejected){c.stop="cost_rejected";break;}
      isam_=trial;isamResult_=result;r.after=CheckedGradient(graph,value);
    }
    r.passed=std::isfinite(r.after)&&r.after<=1e-5;
    c.result=r;c.last=estimate();
    if(c.stop.empty())c.stop=r.passed?"stationary":"budget_exhausted";
    return r;
  }
};
}
