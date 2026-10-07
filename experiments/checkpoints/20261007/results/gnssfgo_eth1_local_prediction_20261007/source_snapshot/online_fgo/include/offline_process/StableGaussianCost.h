#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fgo::offline {

// Opt-in comparison only. Does not alter residuals, whitening, linearization,
// noise models, or the iSAM2 step. Not a bound on nonlinear residual evaluation.
struct StableSum {
  long double sum{0}, correction{0}, absolute{0};
  size_t terms{0};
  void add(long double x) {
    if (!std::isfinite(x)) throw std::runtime_error("nonfinite cost term");
    const long double next=sum+x;
    correction += std::abs(sum)>=std::abs(x) ? (sum-next)+x : (x-next)+sum;
    sum=next;absolute+=std::abs(x);++terms;
  }
  long double value()const{return sum+correction;}
  // Conservative basic-operation budget for the arithmetic on the supplied
  // floating-point residuals, even without credit for compensation. NOT an
  // error bound for Pose3 Log, interpolation, whitening, or numeric Jacobians.
  long double arithmeticBound()const {
    const long double n=8.L*terms+2;
    const long double nu=n*std::numeric_limits<long double>::epsilon();
    if(nu>=1)throw std::runtime_error("unbounded arithmetic comparison");
    return nu/(1-nu)*absolute;
  }
};
struct StableCostChange {
  long double delta{0}, arithmetic_bound{0}, predicted_delta{0}, predicted_bound{0};
  size_t residual_components{0};
};
inline void AccumulateResidualChange(const gtsam::Vector& before,const gtsam::Vector& after,StableSum& sum) {
  if(before.size()!=after.size()||!before.allFinite()||!after.allFinite())
    throw std::runtime_error("invalid residual pair");
  for(int j=0;j<before.size();++j) {
    const long double a=before[j],b=after[j];
    sum.add((b-a)*(b+a)/2.L);
  }
}
inline StableCostChange GaussianCostChange(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& before,const gtsam::Values& after) {
  StableSum actual,predicted;
  const auto displacement=before.localCoordinates(after);
  const auto linear=graph.linearize(before);
  for(size_t slot=0;slot<graph.size();++slot)if(graph[slot]) {
    const auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[slot]);
    if(!f||!boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(f->noiseModel())||
       boost::dynamic_pointer_cast<gtsam::noiseModel::Constrained>(f->noiseModel()))
      throw std::runtime_error("stable comparison requires unconstrained Gaussian factors");
    if(f->active(before)!=f->active(after))throw std::runtime_error("factor activation changed");
    if(!f->active(before))continue;
    AccumulateResidualChange(f->whitenedError(before),f->whitenedError(after),actual);
    const auto jf=boost::dynamic_pointer_cast<gtsam::JacobianFactor>((*linear)[slot]);
    if(!jf)throw std::runtime_error("expected Gaussian Jacobian factor");
    gtsam::Vector ad=gtsam::Vector::Zero(jf->rows());
    for(auto it=jf->begin();it!=jf->end();++it)ad.noalias()+=jf->getA(it)*displacement.at(*it);
    const auto rhs=jf->getb();
    for(int j=0;j<ad.size();++j)predicted.add((long double)ad[j]*((long double)ad[j]/2.L-(long double)rhs[j]));
  }
  return {actual.value(),actual.arithmeticBound(),predicted.value(),predicted.arithmeticBound(),actual.terms};
}
inline bool StableStationaryDescent(const StableCostChange& change,
    double old_gradient,double new_gradient,double tolerance) {
  if(!std::isfinite(old_gradient)||!std::isfinite(new_gradient)||!std::isfinite(tolerance)||
      old_gradient<0||new_gradient<0||tolerance<=0||new_gradient>tolerance||new_gradient>=old_gradient)
    return false;
  if(!std::isfinite(change.delta)||!std::isfinite(change.arithmetic_bound)||
      !std::isfinite(change.predicted_delta)||!std::isfinite(change.predicted_bound)||
      change.arithmetic_bound<0||change.predicted_bound<0||!change.residual_components)return false;
  // Require a resolved DECREASE; do not admit a positive change by enlarging
  // a tolerance. The predicted descent is a consistency check, not a proof of
  // accuracy of the nonlinear residual implementation.
  return change.delta < -change.arithmetic_bound && change.predicted_delta < -change.predicted_bound;
}
}  // namespace fgo::offline
