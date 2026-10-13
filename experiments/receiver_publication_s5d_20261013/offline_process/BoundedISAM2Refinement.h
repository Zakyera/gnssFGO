#pragma once
#include "PublicationObserver.h"

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace fgo::offline {

// Experimental solver policy, not a motion/measurement uncertainty model.
struct BoundedRefinementPolicy {
  size_t maximum_updates{5};
  double normalized_gradient_tolerance{1e-5};
};

struct BoundedRefinementResult {
  size_t attempted{0}, accepted{0};
  double error_before{0}, error_after{0};
  double gradient_before{0}, gradient_after{0};
  std::string status{"not_checked"};
};

// ||D^-1/2 J^T r||_infinity, D = diag(J^T J), at the CURRENT estimate,
// not the cached iSAM2 linearization point. Avoids a dense full-history Hessian.
// A small value tests first-order stationarity, NOT observability/global optimality.
inline double NormalizedGradient(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
  const auto linear = graph.linearize(values);
  const auto gradient = linear->gradientAtZero();
  const auto diagonal = linear->hessianDiagonal();
  double maximum = 0;
  for (const auto& [key, g] : gradient) {
    const auto& d = diagonal.at(key);
    if (!g.allFinite() || !d.allFinite() || (d.array() < 0).any())
      throw std::runtime_error("nonfinite/negative linear system entry");
    for (int i = 0; i < g.size(); ++i) {
      if (d[i] == 0 && g[i] != 0)
        throw std::runtime_error("nonzero gradient in a zero-information coordinate");
      if (d[i] > 0) maximum = std::max(maximum, std::abs(g[i]) / std::sqrt(d[i]));
    }
  }
  s5d::Capture(values,gradient,diagonal,maximum);
  return maximum;
}

inline bool NonIncreasingFiniteError(double current, double candidate) {
  // Deliberately no allowed increase: every accepted step must be non-increasing.
  return std::isfinite(current) && std::isfinite(candidate) && candidate <= current;
}

inline BoundedRefinementResult RefineISAM2(
    std::unique_ptr<gtsam::ISAM2>& solver,
    const BoundedRefinementPolicy& policy = {}) {
  if (!solver || !std::isfinite(policy.normalized_gradient_tolerance) ||
      policy.normalized_gradient_tolerance <= 0)
    throw std::invalid_argument("invalid bounded refinement policy/solver");
  BoundedRefinementResult result;
  // Initial failures propagate to the caller: this is not a recovery mechanism
  // for a graph that already cannot produce a finite estimate.
  const auto initial = solver->calculateEstimate();
  result.error_before = result.error_after = solver->getFactorsUnsafe().error(initial);
  result.gradient_before = result.gradient_after = NormalizedGradient(solver->getFactorsUnsafe(), initial);
  if (!std::isfinite(result.error_before))
    throw std::runtime_error("nonfinite objective before refinement");
  result.status = "budget_exhausted";
  for (size_t i = 0; i < policy.maximum_updates; ++i) {
    if (result.gradient_after <= policy.normalized_gradient_tolerance) {
      result.status = "stationary";
      return result;
    }
    ++result.attempted;
    // Commit only a validated candidate. Factor pointers are shared but no
    // factor or payload is modified; mutable solver state is copied by ISAM2.
    auto candidate = std::make_unique<gtsam::ISAM2>(*solver);
    try {
      candidate->update({}, {}, {}, boost::none, boost::none, boost::none, true);
      const auto estimate = candidate->calculateEstimate();
      const double error = candidate->getFactorsUnsafe().error(estimate);
      if (!NonIncreasingFiniteError(result.error_after, error)) {
        result.status = "rejected_objective";
        return result;
      }
      const double gradient = NormalizedGradient(candidate->getFactorsUnsafe(), estimate);
      // Empty updates must not change any source factor's persistent slot.
      const auto& old_graph = solver->getFactorsUnsafe();
      const auto& new_graph = candidate->getFactorsUnsafe();
      if (old_graph.size() != new_graph.size())
        throw std::runtime_error("empty update changed factor slots");
      for (size_t slot = 0; slot < old_graph.size(); ++slot)
        if (old_graph[slot] != new_graph[slot])
          throw std::runtime_error("empty update changed factor identity");
      const double previous = result.error_after;
      solver.swap(candidate);
      ++result.accepted;
      result.error_after = error;
      result.gradient_after = gradient;
      if (gradient <= policy.normalized_gradient_tolerance) {
        result.status = "stationary";
        return result;
      }
      if (previous - error <= 1e-8 + 1e-6 * std::abs(previous)) {
        // A stalled cached linearization is NOT evidence of convergence.
        result.status = "stalled_nonstationary";
        return result;
      }
    } catch (const std::exception&) {
      result.status = "rejected_exception";
      return result;
    }
  }
  if (result.gradient_after <= policy.normalized_gradient_tolerance) result.status = "stationary";
  return result;
}
}  // namespace fgo::offline
