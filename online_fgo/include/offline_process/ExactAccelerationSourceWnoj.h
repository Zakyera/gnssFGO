#pragma once
#include "offline_process/SourceWnojModel.h"
#include "offline_process/ExactWnojAcceleration.h"

namespace fgo::offline {
// Explicit opt-in factory. The standard FullWnojEndpoint::Between() and live
// receiver defaults remain untouched. Uses the same endpoint/factor type and
// native covariance; only its acceleration derivative implementation differs.
inline FullWnojEndpoint ExactAccelerationWnojEndpointBetween(
    size_t i,size_t j,double dt,double tau,const gtsam::SharedNoiseModel& qc) {
  auto endpoint=FullWnojEndpoint::Between(i,j,dt,tau,qc);
  endpoint.interpolator=std::make_shared<ExactAccelerationWnojInterpolator>(qc,dt,tau,true,true);
  return endpoint;
}
} // namespace fgo::offline
