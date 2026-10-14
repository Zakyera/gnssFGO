#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <chrono>
#include <optional>

namespace fl3d {
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
inline std::optional<Capture> capture;
}

namespace fl2a {
class ObservedISAMConvergence : public BoundarySmoother {
 public:
  using BoundarySmoother::BoundarySmoother;
  SolveResult solve(size_t budget=20) {
    size_t newest=0;
    for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    if(newest!=1224)return BoundarySmoother::solve(budget);
    if(fl3d::capture)throw std::runtime_error("unexpected second target solve");
    fl3d::capture.emplace();auto& c=*fl3d::capture;
    const auto start=fl3d::Clock::now();
    const auto graph=auditGraph();SolveResult r;
    const auto checkStart=fl3d::Clock::now();
    r.before=CheckedGradient(graph,estimate());r.after=r.before;
    c.initialCheckSeconds=fl3d::Seconds(checkStart);
    c.initial=estimate();c.requestedBudget=budget;
    c.fixedKeys=isam_.getFixedVariables().size();c.initialCost=graph.error(c.initial);
    // Same original copy/update/accept/gradient sequence. Only ceiling differs.
    while(r.steps<200&&r.after>1e-5) {
      const auto stepStart=fl3d::Clock::now();
      gtsam::ISAM2 trial(isam_);gtsam::ISAM2UpdateParams p;
      p.force_relinearize=true;p.forceFullSolve=true;
      const auto result=trial.update({}, {},p);++r.steps;
      const auto value=trial.calculateBestEstimate();
      const double updateSeconds=fl3d::Seconds(stepStart);
      const double old=graph.error(estimate()),next=graph.error(value);
      const bool rejected=!std::isfinite(next)||next>old+1e-12*(1+std::abs(old));
      c.steps.push_back({value,old,next,r.after,updateSeconds,0,0,
                        result.variablesRelinearized,result.variablesReeliminated,!rejected});
      if(rejected){c.stop="cost_rejected";c.steps.back().totalSeconds=fl3d::Seconds(stepStart);break;}
      isam_=trial;isamResult_=result;
      const auto gradientStart=fl3d::Clock::now();
      r.after=CheckedGradient(graph,value);
      c.steps.back().checkSeconds=fl3d::Seconds(gradientStart);
      c.steps.back().gradient=r.after;c.steps.back().totalSeconds=fl3d::Seconds(stepStart);
    }
    r.passed=std::isfinite(r.after)&&r.after<=1e-5;
    c.result=r;c.last=estimate();c.finalCost=graph.error(c.last);
    if(c.stop.empty())c.stop=r.passed?"stationary":"budget_exhausted";
    c.seconds=fl3d::Seconds(start);
    // Never admit this diagnostic estimate or test later retirement stages.
    throw std::runtime_error("FL3D_DIAGNOSTIC_STOP");
  }
};
}
