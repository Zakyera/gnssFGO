#pragma once

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <typeinfo>

namespace cbs {

// Diagnostic only: this does not relinearize a factor or change its weight.
//
// iSAM2's unanchored LinearContainerFactors live in the owning solver's theta
// chart. Their ordinary error(Values) intentionally returns zero. To evaluate
// the saved quadratic at X, use q(Local(theta, X)), including its constant.
// Pass the theta belonging to THIS graph snapshot, before marginalizing any
// of its keys; an arbitrary current estimate is not a valid replacement.
// Anchored containers instead retain their own chart.
//
// Nonlinear error() may update mutable factor caches. Call this on a cloned
// diagnostic graph, not the live solver's factors. This helper does not provide
// a common-chart Jacobian or certify nonlinear stationarity.
inline double fixedLagFactorErrorInChart(
    const gtsam::NonlinearFactor& factor, const gtsam::Values& values,
    const gtsam::Values& owning_theta) {
  const auto* container =
      dynamic_cast<const gtsam::LinearContainerFactor*>(&factor);
  double error;
  if (!container) {
    error = factor.error(values);
  } else {
    const auto& gaussian = container->factor();
    if (!gaussian) {
      throw std::invalid_argument("marginal prior has no Gaussian factor");
    }
    const gtsam::Values& origin = container->hasLinearizationPoint()
                                     ? *container->linearizationPoint()
                                     : owning_theta;
    gtsam::VectorValues delta;
    for (auto key = gaussian->begin(); key != gaussian->end(); ++key) {
      if (!origin.exists(*key) || !values.exists(*key)) {
        throw std::invalid_argument("marginal prior chart/value key missing");
      }
      const auto& anchor = origin.at(*key);
      const auto& value = values.at(*key);
      // GenericValue::localCoordinates_ uses an unchecked cast in Release.
      if (typeid(anchor) != typeid(value)) {
        throw std::invalid_argument("marginal prior chart type mismatch");
      }
      const auto dimension = gaussian->getDim(key);
      if (anchor.dim() != static_cast<size_t>(dimension) ||
          value.dim() != static_cast<size_t>(dimension)) {
        throw std::invalid_argument("marginal prior chart dimension mismatch");
      }
      const gtsam::Vector local = anchor.localCoordinates_(value);
      if (local.size() != dimension || !local.allFinite()) {
        throw std::invalid_argument("marginal prior local coordinates invalid");
      }
      delta.insert(*key, local);
    }
    error = gaussian->error(delta);
  }
  if (!std::isfinite(error)) {
    throw std::invalid_argument("factor objective is nonfinite");
  }
  return error;
}

// A failed factor must invalidate the objective, not silently contribute zero.
inline double fixedLagFactorErrorInChartOrNan(
    const gtsam::NonlinearFactor& factor, const gtsam::Values& values,
    const gtsam::Values& owning_theta) {
  try {
    return fixedLagFactorErrorInChart(factor, values, owning_theta);
  } catch (const std::exception&) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace cbs
