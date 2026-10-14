#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <chrono>
#include <optional>

namespace fl3f {
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
inline double referenceSeconds=0;
inline gtsam::Values referencePre,referenceFinal;
inline size_t referencePreSteps=0,referencePostSteps=0;
inline gtsam::NonlinearFactorGraph referenceMarginals,candidateMarginals;
inline gtsam::NonlinearFactorGraph Marginals(const fl2a::BoundarySmoother& s) {
  gtsam::NonlinearFactorGraph result;
  for(const auto& f:s.getFactors())
    if(dynamic_cast<const fl2a::AnchoredMarginal*>(f.get()))result.push_back(f);
  if(result.empty())throw std::runtime_error("missing joint boundary marginal");
  return result;
}
}

namespace fl2a {
class ObservedIncremental : public BoundarySmoother {
 public:
  using BoundarySmoother::BoundarySmoother;
  bool target() const {
    size_t newest=0;for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    return newest==1224;
  }
  void retire() {
    if(!target()){BoundarySmoother::retire();return;}
    ++fl3f::retirementCalls;const auto start=fl3f::Clock::now();
    try{BoundarySmoother::retire();}catch(...){fl3f::retirementSeconds=fl3f::Seconds(start);throw;}
    fl3f::retirementSeconds=fl3f::Seconds(start);
  }
  size_t adoptCharts() {
    if(!target())return BoundarySmoother::adoptCharts();
    ++fl3f::reloadCalls;const auto start=fl3f::Clock::now();
    try{const auto count=BoundarySmoother::adoptCharts();fl3f::reloadSeconds=fl3f::Seconds(start);return count;}
    catch(...){fl3f::reloadSeconds=fl3f::Seconds(start);throw;}
  }
  SolveResult solve(size_t budget=20) {
    size_t newest=0;
    for(const auto& [k,t]:timestamps())newest=std::max(newest,gtsam::Symbol(k).index());
    if(newest!=1224)return BoundarySmoother::solve(budget);
    if(fl3f::captures.size()>=2)throw std::runtime_error("unexpected third target solve");
    if(fl3f::captures.empty()) {
      const auto refStart=fl3f::Clock::now();
      BoundarySmoother reference(*this);
      const auto pre=reference.solve(200);
      if(!pre.passed||pre.steps+2>200)throw std::runtime_error("reference pre-solve failed");
      fl3f::referencePre=reference.estimate();fl3f::referencePreSteps=pre.steps;
      reference.retire();const auto converted=reference.adoptCharts();
      if(!converted)throw std::runtime_error("reference marginal not reloaded");
      const auto post=reference.solve(200-pre.steps-2);
      if(!post.passed)throw std::runtime_error("reference post-solve failed");
      fl3f::referenceFinal=reference.estimate();fl3f::referencePostSteps=post.steps;
      fl3f::referenceMarginals=fl3f::Marginals(reference);
      fl3f::referenceSeconds=fl3f::Seconds(refStart);
    }
    fl3f::captures.emplace_back();auto& c=fl3f::captures.back();
    const auto start=fl3f::Clock::now();
    const auto graph=auditGraph();SolveResult r;
    const auto checkStart=fl3f::Clock::now();
    r.before=CheckedGradient(graph,estimate());r.after=r.before;
    c.initialCheckSeconds=fl3f::Seconds(checkStart);
    c.initial=estimate();c.requestedBudget=budget;
    c.fixedKeys=isam_.getFixedVariables().size();c.initialCost=graph.error(c.initial);
    // Target only: one ordinary incremental update, not forced global refinement.
    while(r.steps<std::min<size_t>(1,budget)&&r.after>1e-5) {
      const auto stepStart=fl3f::Clock::now();
      gtsam::ISAM2 trial(isam_);gtsam::ISAM2UpdateParams p;
      p.force_relinearize=false;p.forceFullSolve=false;
      const auto result=trial.update({}, {},p);++r.steps;
      const auto value=trial.calculateBestEstimate();
      const double updateSeconds=fl3f::Seconds(stepStart);
      const double old=graph.error(estimate()),next=graph.error(value);
      const bool rejected=!std::isfinite(next)||next>old+1e-12*(1+std::abs(old));
      c.steps.push_back({value,old,next,r.after,updateSeconds,0,0,
                        result.variablesRelinearized,result.variablesReeliminated,!rejected});
      if(rejected){c.stop="cost_rejected";c.steps.back().totalSeconds=fl3f::Seconds(stepStart);break;}
      isam_=trial;isamResult_=result;
      const auto gradientStart=fl3f::Clock::now();
      r.after=CheckedGradient(graph,value);
      c.steps.back().checkSeconds=fl3f::Seconds(gradientStart);
      c.steps.back().gradient=r.after;c.steps.back().totalSeconds=fl3f::Seconds(stepStart);
    }
    // Numerical validity is not convergence; target output is explicitly approximate.
    r.passed=std::isfinite(r.after)&&c.stop!="cost_rejected";
    c.result=r;c.last=estimate();c.finalCost=graph.error(c.last);
    if(c.stop.empty())c.stop=r.after<=1e-5?"converged":"incremental_not_converged";
    c.seconds=fl3f::Seconds(start);
    if(fl3f::captures.size()==2)fl3f::candidateMarginals=fl3f::Marginals(*this);
    return r;
  }
};
}
