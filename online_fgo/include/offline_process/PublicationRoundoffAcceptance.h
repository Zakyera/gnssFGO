#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace fgo::offline {

// Opt-in numerical publication policy. Not a factor weight or a change to the
// legacy strict NonIncreasingFiniteError policy. This is a small floating-point
// comparison allowance, not a certified bound on every nonlinear factor's error.
enum class PublicationStepDecision { Reject, NonIncreasing, RoundoffStationary };

inline double PublicationRoundoffAllowance(double current, double candidate) {
  if (!std::isfinite(current) || !std::isfinite(candidate) || current < 0 || candidate < 0)
    return 0.;
  return 8. * std::numeric_limits<double>::epsilon() *
         std::max({1., current, candidate});
}

inline PublicationStepDecision CheckPublicationStep(
    double current_error, double candidate_error,
    double current_gradient, double candidate_gradient, double tolerance) {
  if (!std::isfinite(current_error) || !std::isfinite(candidate_error) ||
      !std::isfinite(current_gradient) || !std::isfinite(candidate_gradient) ||
      !std::isfinite(tolerance) || current_error < 0 || candidate_error < 0 ||
      current_gradient < 0 || candidate_gradient < 0 || tolerance <= 0)
    return PublicationStepDecision::Reject;
  if (candidate_error <= current_error)
    return PublicationStepDecision::NonIncreasing;
  // An increase, however tiny, is not a license to continue an unconverged chain
  // of updates. It is allowed only on a step that actually reaches stationarity.
  if (candidate_error - current_error <=
          PublicationRoundoffAllowance(current_error, candidate_error) &&
      candidate_gradient <= tolerance && candidate_gradient < current_gradient)
    return PublicationStepDecision::RoundoffStationary;
  return PublicationStepDecision::Reject;
}

}  // namespace fgo::offline
