#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using gtsam::symbol_shorthand::X;
using Factor = gtsam::BetweenFactor<gtsam::Pose3>;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

gtsam::Pose3 shift(double x) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0.0, 0.0));
}

double error(const gtsam::Values& lhs, const gtsam::Values& rhs, size_t n) {
  double maximum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    maximum = std::max(maximum,
      gtsam::Pose3::Logmap(lhs.at<gtsam::Pose3>(X(i)).between(
        rhs.at<gtsam::Pose3>(X(i)))).norm());
  }
  return maximum;
}

void test(size_t n) {
  gtsam::ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = 0.0;
  params.findUnusedFactorSlots = true;
  params.factorization = gtsam::ISAM2Params::QR;
  gtsam::ISAM2 isam(params);
  auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
  auto edge_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  gtsam::NonlinearFactorGraph initial;
  gtsam::Values values;
  initial.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), shift(0), prior_noise);
  values.insert(X(0), shift(0));
  for (size_t i = 1; i < n; ++i) {
    initial.emplace_shared<Factor>(X(i - 1), X(i), shift(1), edge_noise);
    values.insert(X(i), shift(static_cast<double>(i)));
  }
  const auto first = isam.update(initial, values);
  require(first.newFactorsIndices.size() == n, "initial indices missing");
  gtsam::FactorIndices active;
  for (size_t i = 1; i < n; ++i) {
    gtsam::NonlinearFactorGraph replacement;
    replacement.emplace_shared<Factor>(X(i - 1), X(i), shift(1.25), edge_noise);
    const auto update = isam.update(replacement, gtsam::Values(),
      gtsam::FactorIndices{first.newFactorsIndices.at(i)}, boost::none,
      boost::none, boost::none, true);
    require(update.newFactorsIndices.size() == 1, "replacement index missing");
    active.push_back(update.newFactorsIndices.at(0));
  }
  // Replacing again exercises persistent factor-index tracking and slot reuse.
  for (size_t i = 1; i < n; ++i) {
    gtsam::NonlinearFactorGraph replacement;
    replacement.emplace_shared<Factor>(X(i - 1), X(i), shift(0.8), edge_noise);
    const auto update = isam.update(replacement, gtsam::Values(),
      gtsam::FactorIndices{active.at(i - 1)}, boost::none, boost::none,
      boost::none, true);
    require(update.newFactorsIndices.size() == 1, "second replacement missing");
    active.at(i - 1) = update.newFactorsIndices.at(0);
  }
  gtsam::NonlinearFactorGraph clean;
  clean.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), shift(0), prior_noise);
  for (size_t i = 1; i < n; ++i) {
    clean.emplace_shared<Factor>(X(i - 1), X(i), shift(0.8), edge_noise);
  }
  const auto expected = gtsam::LevenbergMarquardtOptimizer(clean, values).optimize();
  const double maximum = error(isam.calculateEstimate(), expected, n);
  require(maximum < 1e-7, "replacement differs from clean graph");
  size_t active_factors = 0;
  for (const auto& factor : isam.getFactorsUnsafe()) {
    if (factor) ++active_factors;
  }
  require(active_factors == n, "old factors still active");
  std::cout << "poses=" << n << " max_pose_error=" << maximum
            << " active_factors=" << active_factors << '\n';
}
}  // namespace

int main() {
  try {
    test(2);
    test(5);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
