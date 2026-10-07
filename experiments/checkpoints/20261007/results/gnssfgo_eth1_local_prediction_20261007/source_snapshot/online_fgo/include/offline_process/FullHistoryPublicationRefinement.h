#pragma once
#include "offline_process/BoundedISAM2Refinement.h"
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <vector>
#include "offline_process/StationaryQRConsensus.h"
#include "offline_process/TransactionalISAM2Rebuild.h"
#include "offline_process/StableGaussianCost.h"
#include "offline_process/PublicationRoundoffAcceptance.h"

namespace numerical_audit {
struct Step {
  size_t iteration, relinearized;
  double before, after, gradient;
  bool accepted;
  bool roundoff{false};
  double allowance{0}, state_step_norm{0};
  fgo::offline::StableCostChange cost;
  bool recovery{false};
  bool consensus{false};
  double qr_disagreement{0};
  bool local_prediction{false};
};
struct Result : fgo::offline::BoundedRefinementResult {
  std::vector<Step> steps;
};
inline Result Refine(std::unique_ptr<gtsam::ISAM2>& solver,
                     size_t limit=20, double tolerance=1e-5) {
  if (!solver || !std::isfinite(tolerance) || tolerance<=0)
    throw std::invalid_argument("invalid full publication refinement policy");
  Result result;
  const auto initial=solver->calculateEstimate();
  result.error_before=result.error_after=solver->getFactorsUnsafe().error(initial);
  result.gradient_before=result.gradient_after=fgo::offline::NormalizedGradient(solver->getFactorsUnsafe(),initial);
  if(!std::isfinite(result.error_before))throw std::runtime_error("nonfinite input objective");
  result.status="budget_exhausted";
  bool recovery_attempted=false;
  for(size_t iteration=1;iteration<=limit;++iteration) {
    if(result.gradient_after<=tolerance){result.status="stationary";return result;}
    ++result.attempted;
    auto candidate=std::make_unique<gtsam::ISAM2>(*solver);
    try {
      gtsam::ISAM2UpdateParams params;
      params.force_relinearize=true;
      params.forceFullSolve=true;
      const auto update=candidate->update(gtsam::NonlinearFactorGraph(),gtsam::Values(),params);
      const auto estimate=candidate->calculateBestEstimate();
      const double error=candidate->getFactorsUnsafe().error(estimate);
      const auto& before=solver->getFactorsUnsafe();
      const auto& after=candidate->getFactorsUnsafe();
      if(before.size()!=after.size())throw std::runtime_error("factor slots changed");
      for(size_t slot=0;slot<before.size();++slot)
        if(before[slot]!=after[slot])throw std::runtime_error("factor identity changed");
      const double gradient=fgo::offline::NormalizedGradient(after,estimate);
      fgo::offline::StableCostChange cost;
      const bool ordinary=std::isfinite(error) && error>=0 && error<=result.error_after;
      bool stable=false;
      if(!ordinary && std::isfinite(error) && error>=0) {
        cost=fgo::offline::GaussianCostChange(before,solver->calculateEstimate(),estimate);
        stable=fgo::offline::StableStationaryDescent(cost,result.gradient_after,gradient,tolerance);
      }
      const auto decision=ordinary ? fgo::offline::PublicationStepDecision::NonIncreasing :
        stable ? fgo::offline::PublicationStepDecision::RoundoffStationary : fgo::offline::PublicationStepDecision::Reject;
      const double allowance=0.;
      const double step=solver->calculateEstimate().localCoordinates(estimate).norm();
      if(!std::isfinite(step) || decision==fgo::offline::PublicationStepDecision::Reject) {
        result.steps.push_back({iteration,update.variablesRelinearized,result.error_after,error,gradient,false,false,allowance,step,cost});
        if(!recovery_attempted && std::isfinite(step) && std::isfinite(error)) {
          recovery_attempted=true;
          auto rebuilt=fgo::offline::RebuildFullHistoryQR(*solver,solver->calculateEstimate());
          const auto recovered=rebuilt->calculateBestEstimate();
          const auto recovery_cost=fgo::offline::GaussianCostChange(before,solver->calculateEstimate(),recovered);
          const double recovery_gradient=fgo::offline::NormalizedGradient(before,recovered);
          const double recovery_error=before.error(recovered);
          const double recovery_step=solver->calculateEstimate().localCoordinates(recovered).norm();
          const bool descent=std::isfinite(recovery_error) && std::isfinite(recovery_step) &&
            fgo::offline::StableStationaryDescent(recovery_cost,result.gradient_after,recovery_gradient,tolerance);
          const double disagreement=fgo::offline::QRDisagreement(solver->calculateEstimate(),estimate,recovered);
          const bool consensus=!descent && std::isfinite(recovery_error) && std::isfinite(recovery_step) &&
            fgo::offline::StationaryQRConsensus(cost,recovery_cost,result.error_after,result.gradient_after,gradient,recovery_gradient,disagreement,tolerance);
          const bool accepted=descent||consensus;
          result.steps.push_back({iteration,0,result.error_after,recovery_error,recovery_gradient,accepted,descent,0.,recovery_step,recovery_cost,true,consensus,disagreement});
          if(accepted) {
            solver.swap(rebuilt);++result.accepted;
            result.error_after=recovery_error;result.gradient_after=recovery_gradient;
            result.status="stationary";return result;
          }
          // A private numerical polish may use the remaining EXISTING budget.
          // No intermediate iterate is published or committed to the receiver.
          auto flat=[&](const fgo::offline::StableCostChange& c) {
            const long double bound=fgo::offline::kReferenceRelativeCostTolerance*std::max(1.,result.error_after);
            return std::isfinite(c.delta)&&std::isfinite(c.predicted_delta)&&
              std::isfinite(c.predicted_bound)&&c.predicted_bound>=0&&c.residual_components&&
              std::abs(c.delta)<=bound&&std::abs(c.predicted_delta)<=bound&&c.predicted_delta < -c.predicted_bound;
          };
          if(flat(cost)&&flat(recovery_cost)) {
            const auto anchor=solver->calculateEstimate();
            for(size_t pass=iteration+1;pass<=limit;++pass) {
              ++result.attempted;
              const auto seed=rebuilt->calculateBestEstimate();
              auto trial=std::make_unique<gtsam::ISAM2>(*rebuilt);
              const auto update2=trial->update(gtsam::NonlinearFactorGraph(),gtsam::Values(),params);
              const auto x=trial->calculateBestEstimate();
              auto fresh=fgo::offline::RebuildFullHistoryQR(*rebuilt,seed);
              const auto y=fresh->calculateBestEstimate();
              fgo::offline::RequireSameFactorSlots(before,trial->getFactorsUnsafe());
              fgo::offline::RequireSameFactorSlots(before,fresh->getFactorsUnsafe());
              const auto cx=fgo::offline::CumulativeCostWithLocalPrediction(
                fgo::offline::GaussianCostChange(before,anchor,x),
                fgo::offline::GaussianCostChange(before,seed,x));
              const auto cy=fgo::offline::CumulativeCostWithLocalPrediction(
                fgo::offline::GaussianCostChange(before,anchor,y),
                fgo::offline::GaussianCostChange(before,seed,y));
              const double gx=fgo::offline::NormalizedGradient(before,x),gy=fgo::offline::NormalizedGradient(before,y);
              const double ex=before.error(x),ey=before.error(y);
              const double sx=anchor.localCoordinates(x).norm(),sy=anchor.localCoordinates(y).norm();
              const double agreement=fgo::offline::QRDisagreement(seed,x,y);
              const bool descent2=std::isfinite(ey)&&std::isfinite(sy)&&
                fgo::offline::StableStationaryDescent(cy,result.gradient_after,gy,tolerance);
              const bool consensus2=!descent2&&std::isfinite(ex)&&std::isfinite(ey)&&std::isfinite(sx)&&std::isfinite(sy)&&
                fgo::offline::StationaryQRConsensus(cx,cy,result.error_after,result.gradient_after,gx,gy,agreement,tolerance);
              const bool accept2=descent2||consensus2;
              result.steps.push_back({pass,update2.variablesRelinearized,result.error_after,ex,gx,false,false,0.,sx,cx,false,false,0.,true});
              result.steps.push_back({pass,0,result.error_after,ey,gy,accept2,descent2,0.,sy,cy,true,consensus2,agreement,true});
              if(accept2) {
                solver.swap(fresh);++result.accepted;result.error_after=ey;result.gradient_after=gy;
                result.status="stationary";return result;
              }
              if(!flat(cx)||!flat(cy)||!std::isfinite(gx)||!std::isfinite(gy)) {
                result.status="rejected_objective";return result;
              }
              rebuilt.swap(fresh);
            }
            result.status="budget_exhausted";return result;
          }
        }
        result.status="rejected_objective";return result;
      }
      const bool roundoff=decision==fgo::offline::PublicationStepDecision::RoundoffStationary;
      result.steps.push_back({iteration,update.variablesRelinearized,result.error_after,error,gradient,true,roundoff,allowance,step,cost});
      solver.swap(candidate);
      ++result.accepted;result.error_after=error;result.gradient_after=gradient;
    } catch(const std::exception&) {
      result.status="rejected_exception";return result;
    }
  }
  if(result.gradient_after<=tolerance)result.status="stationary";
  return result;
}
} // namespace numerical_audit
