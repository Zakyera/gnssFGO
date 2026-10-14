#pragma once
#include "../fixed_lag_boundary_fl2a_20261013/BoundarySmoother.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Private fixture solver, NOT installed in the receiver. The nonlinear graph
// and its marginal information are never augmented with damping factors.
namespace fl3b {
inline gtsam::LevenbergMarquardtParams Params() {
  auto p=gtsam::LevenbergMarquardtParams::CeresDefaults();
  p.setLinearSolverType("MULTIFRONTAL_QR");
  p.maxIterations=20;p.relativeErrorTol=0;p.absoluteErrorTol=0;p.errorTol=0;
  return p;
}
inline gtsam::VectorValues SqrtDiagonal(const gtsam::GaussianFactorGraph& linear,
                                       const gtsam::LevenbergMarquardtParams& p) {
  auto d=linear.hessianDiagonal();
  for(auto& kv:d)
    kv.second=kv.second.cwiseMax(p.minDiagonal).cwiseMin(p.maxDiagonal).cwiseSqrt();
  return d;
}
struct Trial {
  size_t attempt,acceptedIterations;int libraryInnerIterations;
  bool accepted,libraryDone;
  double lambdaBefore,lambdaAfter,costBefore,costAfter,gradient;
};
struct Result {
  bool passed=false;size_t attempts=0,accepted=0,rejected=0;
  double before=0,after=0;std::string stop;
  // Diagnostic only, never implicitly committed to a caller or marginal prior.
  gtsam::Values diagnosticValues;
  std::vector<Trial> trace;
};
inline Result Solve(const gtsam::NonlinearFactorGraph& graph,
                    const gtsam::Values& initial,size_t budget=20) {
  Result r;r.diagnosticValues=initial;
  try {
    r.before=r.after=fl2a::CheckedGradient(graph,initial);
    if(r.after<=1e-5){r.passed=true;r.stop="stationary";return r;}
    const auto p=Params();gtsam::LevenbergMarquardtOptimizer opt(graph,initial,p);
    gtsam::GaussianFactorGraph::shared_ptr linear;gtsam::VectorValues diagonal;
    while(r.attempts<budget) {
      if(!linear){linear=opt.linearize();diagonal=SqrtDiagonal(*linear,p);}
      const double lambda=opt.lambda(),cost=graph.error(opt.values());
      const size_t iterations=opt.iterations();
      ++r.attempts; // Count this attempt even if the installed solve throws.
      const bool done=opt.tryLambda(*linear,diagonal);
      const bool accepted=opt.iterations()>iterations;
      if(accepted){++r.accepted;linear.reset();}else ++r.rejected;
      r.diagnosticValues=opt.values();
      r.after=fl2a::CheckedGradient(graph,r.diagnosticValues);
      const double next=graph.error(r.diagnosticValues);
      r.trace.push_back({r.attempts,opt.iterations(),opt.getInnerIterations(),
                        accepted,done,lambda,opt.lambda(),cost,next,r.after});
      if(!std::isfinite(opt.lambda())||next>cost+1e-12*(1+std::abs(cost)))
        throw std::runtime_error("invalid lambda or accepted cost increase");
      if(r.after<=1e-5){r.passed=true;r.stop="stationary";return r;}
      if(!accepted&&done){r.stop=opt.lambda()>=p.lambdaUpperBound?"maximum_lambda":"library_stall";return r;}
    }
    r.stop="budget_exhausted";
  }catch(const std::exception& e){r.passed=false;r.stop=std::string("exception: ")+e.what();}
  return r;
}
} // namespace fl3b
