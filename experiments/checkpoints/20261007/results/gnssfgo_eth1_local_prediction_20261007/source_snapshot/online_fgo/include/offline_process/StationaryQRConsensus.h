#pragma once
#include "offline_process/StableGaussianCost.h"

namespace fgo::offline {
// Same relative objective tolerance as the frozen copied LM reference. This is
// a convergence tolerance, NOT a forward-error bound for SE(3) residuals and NOT
// a certificate of strict mathematical descent. Only used after two separately
// factored QR systems agree on a locally stationary solution of the same graph.
inline constexpr double kReferenceRelativeCostTolerance=1e-11;

// A private multi-step recovery has two distinct references: cumulative actual
// objective change is bounded against the retained (uncommitted) anchor, while
// predicted descent is evaluated in the CURRENT seed's freshly linearized model.
// A stale anchor linearization is not a valid descent test for later GN steps.
inline StableCostChange CumulativeCostWithLocalPrediction(
    const StableCostChange& cumulative, const StableCostChange& local) {
  auto result=cumulative;
  result.predicted_delta=local.predicted_delta;
  result.predicted_bound=local.predicted_bound;
  return result;
}

inline double QRDisagreement(const gtsam::Values& before,
    const gtsam::Values& incremental,const gtsam::Values& fresh) {
  const auto a=before.localCoordinates(incremental);
  const auto b=before.localCoordinates(fresh);
  double maximum=0;
  if(a.size()!=b.size())throw std::runtime_error("QR state key mismatch");
  for(const auto& [key,v]:a) {
    const auto& w=b.at(key);
    if(v.size()!=w.size()||!v.allFinite()||!w.allFinite())
      throw std::runtime_error("invalid QR displacement");
    for(int i=0;i<v.size();++i)
      maximum=std::max(maximum,std::abs(v[i]-w[i])/(1.+std::max(std::abs(v[i]),std::abs(w[i]))));
  }
  return maximum;
}
inline bool StationaryQRConsensus(const StableCostChange& a,const StableCostChange& b,
    double objective,double old_gradient,double incremental_gradient,
    double fresh_gradient,double disagreement,double gradient_tolerance) {
  if(!std::isfinite(objective)||objective<0||!std::isfinite(old_gradient)||
     !std::isfinite(incremental_gradient)||!std::isfinite(fresh_gradient)||
     !std::isfinite(disagreement)||!std::isfinite(gradient_tolerance)||
     disagreement<0||gradient_tolerance<=0||old_gradient<=gradient_tolerance||
     incremental_gradient<0||fresh_gradient<0||
     incremental_gradient>gradient_tolerance||fresh_gradient>gradient_tolerance||
     disagreement>std::sqrt(std::numeric_limits<double>::epsilon()))return false;
  const long double tolerance=kReferenceRelativeCostTolerance*std::max(1.,objective);
  for(const auto* c:{&a,&b}) {
    if(!std::isfinite(c->delta)||!std::isfinite(c->predicted_delta)||
       !std::isfinite(c->arithmetic_bound)||!std::isfinite(c->predicted_bound)||
       c->arithmetic_bound<0||c->predicted_bound<0||!c->residual_components||
       std::abs(c->delta)>tolerance||std::abs(c->predicted_delta)>tolerance||
       c->predicted_delta>=-c->predicted_bound)return false;
  }
  return true;
}
} // namespace fgo::offline
