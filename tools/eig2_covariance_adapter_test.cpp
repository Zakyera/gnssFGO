// Isolated synthetic verification of the GrandTour CSV -> GTSAM factor tangent.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include "factor/odometry/BetweenFactor.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  using gtsam::Pose3;
  gtsam::Matrix6 root = gtsam::Matrix6::Identity();
  root(0, 3) = 0.17;
  root(1, 4) = -0.11;
  root(2, 5) = 0.09;
  root(4, 0) = -0.07;
  const gtsam::Matrix6 covariance = root * root.transpose() * 1e-4;
  const gtsam::Vector6 z = (gtsam::Vector6() << 0.03, -0.04, 0.02, 0.4, -0.2, 0.15).finished();
  const gtsam::Vector6 delta = (gtsam::Vector6() << 1e-3, -2e-3, 7e-4, 3e-3, -4e-3, 2e-3).finished();
  const Pose3 from;
  const Pose3 measured = Pose3::Expmap(z);
  const Pose3 to = measured.compose(Pose3::Expmap(delta));
  const auto model = gtsam::noiseModel::Gaussian::Covariance(covariance);
  const fgo::factor::BetweenPose3Factor factor(0, 1, measured, model);
  const gtsam::Vector6 residual = factor.evaluateError(from, to);
  const auto whitened = model->whiten(residual);
  const double expected_squared = delta.dot(covariance.ldlt().solve(delta));
  const double residual_error = (residual - delta).norm();
  const double norm_error = std::abs(whitened.squaredNorm() - expected_squared);
  std::cout << "residual_error=" << residual_error << " whitened_norm_error=" << norm_error
            << " offdiagonal=" << covariance(0, 3) << '\n';
  if (residual_error > 1e-12 || norm_error > 1e-10 || covariance(0, 3) == 0.0) {
    throw std::runtime_error("covariance adapter tangent/whitening test failed");
  }
}
