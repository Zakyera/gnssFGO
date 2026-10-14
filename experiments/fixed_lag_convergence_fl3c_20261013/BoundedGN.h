#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <chrono>
#include <functional>

namespace fl3c {
using Clock=std::chrono::steady_clock;
inline double Seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
inline gtsam::GaussNewtonParams Params() {
  gtsam::GaussNewtonParams p;p.setLinearSolverType("MULTIFRONTAL_QR");
  p.maxIterations=200;p.relativeErrorTol=0;p.absoluteErrorTol=0;p.errorTol=0;return p;
}
struct Step {size_t attempt;bool accepted;double cost,gradient,solveSeconds,checkSeconds;};
struct Result {
  bool passed=false;size_t attempts=0,accepted=0;std::string stop;
  double initialGradient=0,finalGradient=0;
  gtsam::Values diagnosticValues;
  std::vector<Step> steps;
};
using Observer=std::function<void(const Step&,const gtsam::Values&,const gtsam::Values&)>;
// Private same-objective diagnostic. No receiver or caller state is modified.
inline Result Solve(const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& initial,
                    size_t budget=200,Observer observe={}) {
  Result r;r.diagnosticValues=initial;
  try {
    r.initialGradient=r.finalGradient=fl2a::CheckedGradient(graph,initial);
    if(r.finalGradient<=1e-5){r.passed=true;r.stop="stationary";return r;}
    gtsam::GaussNewtonOptimizer opt(graph,initial,Params());
    while(r.attempts<budget) {
      const auto old=opt.values();const double cost=graph.error(old);
      ++r.attempts;const auto begin=Clock::now();opt.iterate();
      const double solverSeconds=Seconds(begin);const auto check=Clock::now();
      const double next=graph.error(opt.values());
      const double gradient=fl2a::CheckedGradient(graph,opt.values());
      const bool accepted=std::isfinite(next)&&next<=cost+1e-12*(1+std::abs(cost));
      const Step step{r.attempts,accepted,next,gradient,solverSeconds,Seconds(check)};
      r.steps.push_back(step);
      if(!accepted){r.stop="cost_rejected";return r;}
      ++r.accepted;r.diagnosticValues=opt.values();r.finalGradient=gradient;
      if(observe)observe(step,old,r.diagnosticValues);
      if(r.finalGradient<=1e-5){r.passed=true;r.stop="stationary";return r;}
    }
    r.stop="budget_exhausted";
  }catch(const std::exception& e){r.passed=false;r.stop=std::string("exception: ")+e.what();}
  return r;
}
} // namespace fl3c
