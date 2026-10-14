#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <chrono>
#include <optional>

namespace fl3e {
using Clock=std::chrono::steady_clock;
inline double Seconds(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
struct Step {
  gtsam::Values value;
  double oldCost=0,newCost=0,gradient=0,updateSeconds=0,checkSeconds=0,totalSeconds=0;
  size_t relinearized=0,reeliminated=0;
  bool accepted=false;
};
struct Capture {
  gtsam::Values initial,last;
  std::vector<Step> steps;
  fl2a::BoundarySmoother::SolveResult result;
  std::string stop;
  size_t requestedBudget=0,fixedKeys=0;
  double initialCost=0,finalCost=0,seconds=0,initialCheckSeconds=0;
};
inline std::vector<Capture> captures;
inline double retirementSeconds=0,reloadSeconds=0;
inline size_t retirementCalls=0,reloadCalls=0;
}

namespace fl2a {
class ObservedCompleteUpdate : public BoundarySmoother {
 public:
  using BoundarySmoother::BoundarySmoother;
  bool target() const {
    size_t newest=0;for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    return newest==1224;
  }
  void retire() {
    if(!target()){BoundarySmoother::retire();return;}
    ++fl3e::retirementCalls;const auto start=fl3e::Clock::now();
    try{BoundarySmoother::retire();}catch(...){fl3e::retirementSeconds=fl3e::Seconds(start);throw;}
    fl3e::retirementSeconds=fl3e::Seconds(start);
  }
  size_t adoptCharts() {
    if(!target())return BoundarySmoother::adoptCharts();
    ++fl3e::reloadCalls;const auto start=fl3e::Clock::now();
    try{const auto count=BoundarySmoother::adoptCharts();fl3e::reloadSeconds=fl3e::Seconds(start);return count;}
    catch(...){fl3e::reloadSeconds=fl3e::Seconds(start);throw;}
  }
  SolveResult solve(size_t budget=20) {
    size_t newest=0;
    for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    if(newest!=1224)return BoundarySmoother::solve(budget);
    if(fl3e::captures.size()>=2)throw std::runtime_error("unexpected third target solve");
    fl3e::captures.emplace_back();auto& c=fl3e::captures.back();
    const auto start=fl3e::Clock::now();
    const auto graph=auditGraph();SolveResult r;
    const auto checkStart=fl3e::Clock::now();
    r.before=CheckedGradient(graph,estimate());r.after=r.before;
    c.initialCheckSeconds=fl3e::Seconds(checkStart);
    c.initial=estimate();c.requestedBudget=budget;
    c.fixedKeys=isam_.getFixedVariables().size();c.initialCost=graph.error(c.initial);
    // Same original copy/update/accept/gradient sequence. Only ceiling differs.
    while(r.steps<budget&&r.after>1e-5) {
      const auto stepStart=fl3e::Clock::now();
      gtsam::ISAM2 trial(isam_);gtsam::ISAM2UpdateParams p;
      p.force_relinearize=true;p.forceFullSolve=true;
      const auto result=trial.update({}, {},p);++r.steps;
      const auto value=trial.calculateBestEstimate();
      const double updateSeconds=fl3e::Seconds(stepStart);
      const double old=graph.error(estimate()),next=graph.error(value);
      const bool rejected=!std::isfinite(next)||next>old+1e-12*(1+std::abs(old));
      c.steps.push_back({value,old,next,r.after,updateSeconds,0,0,
                        result.variablesRelinearized,result.variablesReeliminated,!rejected});
      if(rejected){c.stop="cost_rejected";c.steps.back().totalSeconds=fl3e::Seconds(stepStart);break;}
      isam_=trial;isamResult_=result;
      const auto gradientStart=fl3e::Clock::now();
      r.after=CheckedGradient(graph,value);
      c.steps.back().checkSeconds=fl3e::Seconds(gradientStart);
      c.steps.back().gradient=r.after;c.steps.back().totalSeconds=fl3e::Seconds(stepStart);
    }
    r.passed=std::isfinite(r.after)&&r.after<=1e-5;
    c.result=r;c.last=estimate();c.finalCost=graph.error(c.last);
    if(c.stop.empty())c.stop=r.passed?"stationary":"budget_exhausted";
    c.seconds=fl3e::Seconds(start);
    return r;
  }
};
}
