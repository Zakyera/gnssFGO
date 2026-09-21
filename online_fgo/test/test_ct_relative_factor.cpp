// Copyright 2026 Yerasimos Zaky
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "factor/motion/GPWNOAPrior.h"
#include "factor/odometry/GPInterpolatedSameIntervalPose3BetweenFactor.h"
#include "model/gp_interpolator/GPWNOAInterpolator.h"
#include "solver/IncrementalFixedLagSmoother.h"

namespace
{

using fgo::factor::GPInterpolatedSameIntervalPose3BetweenFactor;
using fgo::models::GPInterpolator;
using fgo::models::GPWNOAInterpolator;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kDeltaT = 1.0;
constexpr double kFirstMeasurementTime = 0.23;
constexpr double kSecondMeasurementTime = 0.71;

gtsam::SharedNoiseModel MakeQcModel()
{
  const gtsam::Vector6 variances =
    (gtsam::Vector6() << 0.03, 0.04, 0.05, 0.20, 0.25, 0.30).finished();
  return gtsam::noiseModel::Diagonal::Variances(variances);
}

gtsam::Matrix6 MakeMeasurementCovariance()
{
  gtsam::Matrix6 lower = gtsam::Matrix6::Zero();
  lower.diagonal() << 0.010, 0.012, 0.014, 0.025, 0.030, 0.035;
  lower(1, 0) = 0.002;
  lower(2, 0) = -0.001;
  lower(3, 0) = 0.003;
  lower(3, 2) = -0.002;
  lower(4, 1) = 0.004;
  lower(5, 0) = -0.002;
  lower(5, 4) = 0.005;
  return lower * lower.transpose();
}

std::shared_ptr<GPInterpolator> MakeInterpolator(double tau)
{
  return std::make_shared<GPWNOAInterpolator>(
    MakeQcModel(), kDeltaT, tau, false, true);
}

gtsam::Pose3 PoseAt(double time, const gtsam::Vector3 & velocity)
{
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(velocity * time));
}

GPInterpolatedSameIntervalPose3BetweenFactor MakeFactor(
  const gtsam::Pose3 & measurement,
  const gtsam::SharedNoiseModel & measurement_noise)
{
  return GPInterpolatedSameIntervalPose3BetweenFactor(
    X(0), V(0), W(0), X(1), V(1), W(1),
    measurement,
    MakeInterpolator(kFirstMeasurementTime),
    MakeInterpolator(kSecondMeasurementTime),
    measurement_noise);
}

gtsam::Vector EvaluateCommonSupport(
  const GPInterpolatedSameIntervalPose3BetweenFactor & factor,
  const gtsam::Pose3 & pose0,
  const gtsam::Vector3 & velocity0,
  const gtsam::Vector3 & omega0,
  const gtsam::Pose3 & pose1,
  const gtsam::Vector3 & velocity1,
  const gtsam::Vector3 & omega1)
{
  return factor.evaluateError(
    pose0, velocity0, omega0, pose1, velocity1, omega1);
}

gtsam::Matrix NumericalPoseJacobian(
  const GPInterpolatedSameIntervalPose3BetweenFactor & factor,
  const gtsam::Pose3 & pose0,
  const gtsam::Vector3 & velocity0,
  const gtsam::Vector3 & omega0,
  const gtsam::Pose3 & pose1,
  const gtsam::Vector3 & velocity1,
  const gtsam::Vector3 & omega1,
  bool perturb_first_pose)
{
  constexpr double epsilon = 1e-6;
  gtsam::Matrix jacobian(6, 6);
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(column) = epsilon;
    const gtsam::Pose3 pose0_plus = perturb_first_pose ? pose0.retract(delta) : pose0;
    const gtsam::Pose3 pose0_minus = perturb_first_pose ? pose0.retract(-delta) : pose0;
    const gtsam::Pose3 pose1_plus = perturb_first_pose ? pose1 : pose1.retract(delta);
    const gtsam::Pose3 pose1_minus = perturb_first_pose ? pose1 : pose1.retract(-delta);
    jacobian.col(column) =
      (EvaluateCommonSupport(
        factor, pose0_plus, velocity0, omega0,
        pose1_plus, velocity1, omega1) -
      EvaluateCommonSupport(
        factor, pose0_minus, velocity0, omega0,
        pose1_minus, velocity1, omega1)) /
      (2.0 * epsilon);
  }
  return jacobian;
}

gtsam::Matrix NumericalVectorJacobian(
  const GPInterpolatedSameIntervalPose3BetweenFactor & factor,
  const gtsam::Pose3 & pose0,
  const gtsam::Vector3 & velocity0,
  const gtsam::Vector3 & omega0,
  const gtsam::Pose3 & pose1,
  const gtsam::Vector3 & velocity1,
  const gtsam::Vector3 & omega1,
  int argument)
{
  constexpr double epsilon = 1e-6;
  gtsam::Matrix jacobian(6, 3);
  for (int column = 0; column < 3; ++column) {
    gtsam::Vector3 velocity0_plus = velocity0;
    gtsam::Vector3 velocity0_minus = velocity0;
    gtsam::Vector3 omega0_plus = omega0;
    gtsam::Vector3 omega0_minus = omega0;
    gtsam::Vector3 velocity1_plus = velocity1;
    gtsam::Vector3 velocity1_minus = velocity1;
    gtsam::Vector3 omega1_plus = omega1;
    gtsam::Vector3 omega1_minus = omega1;
    switch (argument) {
      case 0:
        velocity0_plus(column) += epsilon;
        velocity0_minus(column) -= epsilon;
        break;
      case 1:
        omega0_plus(column) += epsilon;
        omega0_minus(column) -= epsilon;
        break;
      case 2:
        velocity1_plus(column) += epsilon;
        velocity1_minus(column) -= epsilon;
        break;
      default:
        omega1_plus(column) += epsilon;
        omega1_minus(column) -= epsilon;
        break;
    }
    jacobian.col(column) =
      (EvaluateCommonSupport(
        factor, pose0, velocity0_plus, omega0_plus,
        pose1, velocity1_plus, omega1_plus) -
      EvaluateCommonSupport(
        factor, pose0, velocity0_minus, omega0_minus,
        pose1, velocity1_minus, omega1_minus)) /
      (2.0 * epsilon);
  }
  return jacobian;
}

void ExpectMatrixNear(
  const gtsam::Matrix & actual,
  const gtsam::Matrix & expected,
  double tolerance)
{
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), tolerance);
}

struct SyntheticMotion
{
  gtsam::Pose3 pose0{gtsam::Pose3()};
  gtsam::Vector3 velocity{1.1, -0.35, 0.20};
  gtsam::Vector3 omega{gtsam::Vector3::Zero()};
  gtsam::Pose3 pose1{PoseAt(kDeltaT, velocity)};

  gtsam::Pose3 relativeMeasurement() const
  {
    return PoseAt(kFirstMeasurementTime, velocity).between(
      PoseAt(kSecondMeasurementTime, velocity));
  }
};

TEST(CtRelativeFactor, InterpolatesBothEndpointsAndPreservesDenseCovariance) {
  const SyntheticMotion motion;
  const gtsam::Matrix6 covariance = MakeMeasurementCovariance();
  const auto measurement_noise = gtsam::noiseModel::Gaussian::Covariance(covariance);
  const auto first_interpolator = MakeInterpolator(kFirstMeasurementTime);
  const auto second_interpolator = MakeInterpolator(kSecondMeasurementTime);

  const gtsam::Pose3 interpolated_first = first_interpolator->interpolatePose(
    motion.pose0, motion.velocity, motion.omega,
    motion.pose1, motion.velocity, motion.omega);
  const gtsam::Pose3 interpolated_second = second_interpolator->interpolatePose(
    motion.pose0, motion.velocity, motion.omega,
    motion.pose1, motion.velocity, motion.omega);

  EXPECT_TRUE(
    interpolated_first.equals(
      PoseAt(kFirstMeasurementTime, motion.velocity), 1e-10));
  EXPECT_TRUE(
    interpolated_second.equals(
      PoseAt(kSecondMeasurementTime, motion.velocity), 1e-10));
  EXPECT_FALSE(interpolated_first.equals(motion.pose0, 1e-6));
  EXPECT_FALSE(interpolated_second.equals(motion.pose1, 1e-6));

  const auto factor = MakeFactor(motion.relativeMeasurement(), measurement_noise);
  const std::set<gtsam::Key> unique_keys(factor.keys().begin(), factor.keys().end());
  EXPECT_EQ(factor.keys().size(), 6U);
  EXPECT_EQ(unique_keys.size(), 6U);
  const gtsam::Vector residual = EvaluateCommonSupport(
    factor, motion.pose0, motion.velocity, motion.omega,
    motion.pose1, motion.velocity, motion.omega);
  EXPECT_LT(residual.norm(), 1e-10);

  const auto recovered_noise =
    boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel());
  ASSERT_TRUE(recovered_noise);
  ExpectMatrixNear(recovered_noise->covariance(), covariance, 1e-14);
  EXPECT_GT(std::abs(covariance(4, 1)), 0.0);
  EXPECT_GT(std::abs(covariance(5, 4)), 0.0);
}

TEST(CtRelativeFactor, CommonSupportJacobiansMatchFiniteDifferences) {
  const SyntheticMotion motion;
  const auto factor = MakeFactor(
    motion.relativeMeasurement(),
    gtsam::noiseModel::Gaussian::Covariance(MakeMeasurementCovariance()));

  std::array<gtsam::Matrix, 6> jacobians;
  const gtsam::Vector residual = factor.evaluateError(
    motion.pose0, motion.velocity, motion.omega,
    motion.pose1, motion.velocity, motion.omega,
    jacobians[0], jacobians[1], jacobians[2], jacobians[3],
    jacobians[4], jacobians[5]);
  EXPECT_LT(residual.norm(), 1e-10);

  ExpectMatrixNear(
    jacobians[0],
    NumericalPoseJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, true),
    1e-3);
  ExpectMatrixNear(
    jacobians[1],
    NumericalVectorJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, 0),
    1e-3);
  ExpectMatrixNear(
    jacobians[2],
    NumericalVectorJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, 1),
    1e-3);
  ExpectMatrixNear(
    jacobians[3],
    NumericalPoseJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, false),
    1e-3);
  ExpectMatrixNear(
    jacobians[4],
    NumericalVectorJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, 2),
    1e-3);
  ExpectMatrixNear(
    jacobians[5],
    NumericalVectorJacobian(
      factor, motion.pose0, motion.velocity, motion.omega,
      motion.pose1, motion.velocity, motion.omega, 3),
    1e-3);
}

TEST(CtRelativeFactor, RunsThroughIncrementalFixedLagSmoother) {
  const SyntheticMotion motion;
  const auto qc_model = MakeQcModel();
  const auto relative_factor = boost::make_shared<GPInterpolatedSameIntervalPose3BetweenFactor>(
    MakeFactor(
      motion.relativeMeasurement(),
      gtsam::noiseModel::Gaussian::Covariance(MakeMeasurementCovariance())));

  gtsam::NonlinearFactorGraph graph;
  const auto pose_prior_noise =
    gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Constant(1e-4));
  const auto vector_prior_noise =
    gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), motion.pose0, pose_prior_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    V(0), motion.velocity,
    vector_prior_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(0), motion.omega, vector_prior_noise);
  graph.emplace_shared<fgo::factor::GPWNOAPrior>(
    X(0), V(0), W(0), X(1), V(1), W(1), kDeltaT, qc_model, false, true);
  graph.push_back(relative_factor);

  gtsam::Values initial;
  const gtsam::Vector3 initial_velocity0 =
    motion.velocity + gtsam::Vector3(0.03, -0.02, 0.01);
  const gtsam::Vector3 initial_omega0 =
    motion.omega + gtsam::Vector3(0.005, -0.004, 0.003);
  const gtsam::Vector3 initial_velocity1 =
    motion.velocity + gtsam::Vector3(-0.04, 0.03, -0.02);
  const gtsam::Vector3 initial_omega1 =
    motion.omega + gtsam::Vector3(-0.004, 0.006, -0.002);
  initial.insert(
    X(0), motion.pose0.retract(
      (gtsam::Vector6() << 0.01, -0.01, 0.005, 0.03, -0.02, 0.01).finished()));
  initial.insert(V(0), initial_velocity0);
  initial.insert(W(0), initial_omega0);
  initial.insert(
    X(1), motion.pose1.retract(
      (gtsam::Vector6() << -0.015, 0.008, -0.004, -0.05, 0.04, -0.02).finished()));
  initial.insert(V(1), initial_velocity1);
  initial.insert(W(1), initial_omega1);

  fgo::solvers::FixedLagSmoother::KeyTimestampMap timestamps{
    {X(0), 0.0}, {V(0), 0.0}, {W(0), 0.0},
    {X(1), 1.0}, {V(1), 1.0}, {W(1), 1.0}};

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.0;
  params.relinearizeSkip = 1;
  params.findUnusedFactorSlots = true;
  fgo::solvers::IncrementalFixedLagSmoother smoother(2.0, params);

  const double initial_error = graph.error(initial);
  const auto linear_graph = graph.linearize(initial);
  ASSERT_EQ(linear_graph->size(), graph.size());
  EXPECT_NO_THROW(smoother.update(graph, initial, timestamps));

  const gtsam::Values estimate = smoother.calculateEstimate();
  ASSERT_TRUE(estimate.exists(X(0)));
  ASSERT_TRUE(estimate.exists(X(1)));
  const double optimized_error = graph.error(estimate);
  EXPECT_LT(optimized_error, initial_error);

  const gtsam::Matrix pose1_covariance = smoother.marginalCovariance(X(1));
  EXPECT_TRUE(pose1_covariance.allFinite());
  EXPECT_LT((pose1_covariance - pose1_covariance.transpose()).norm(), 1e-10);
  EXPECT_GE(
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(pose1_covariance)
    .eigenvalues().minCoeff(), -1e-12);

  const auto marginals = smoother.getMarginals(estimate);
  const gtsam::KeyVector support_keys{X(0), V(0), W(0), X(1), V(1), W(1)};
  const gtsam::Matrix support_covariance =
    marginals.jointMarginalCovariance(support_keys).fullMatrix();
  ASSERT_EQ(support_covariance.rows(), 24);
  ASSERT_EQ(support_covariance.cols(), 24);

  const auto first_interpolator = MakeInterpolator(kFirstMeasurementTime);
  const auto second_interpolator = MakeInterpolator(kSecondMeasurementTime);
  std::array<gtsam::Matrix, 6> first_interpolation_jacobians;
  std::array<gtsam::Matrix, 6> second_interpolation_jacobians;
  const gtsam::Pose3 first_interpolated_pose = first_interpolator->interpolatePose(
    estimate.at<gtsam::Pose3>(X(0)), estimate.at<gtsam::Vector3>(V(0)),
    estimate.at<gtsam::Vector3>(W(0)), estimate.at<gtsam::Pose3>(X(1)),
    estimate.at<gtsam::Vector3>(V(1)), estimate.at<gtsam::Vector3>(W(1)),
    first_interpolation_jacobians[0], first_interpolation_jacobians[1],
    first_interpolation_jacobians[2], first_interpolation_jacobians[3],
    first_interpolation_jacobians[4], first_interpolation_jacobians[5]);
  const gtsam::Pose3 second_interpolated_pose = second_interpolator->interpolatePose(
    estimate.at<gtsam::Pose3>(X(0)), estimate.at<gtsam::Vector3>(V(0)),
    estimate.at<gtsam::Vector3>(W(0)), estimate.at<gtsam::Pose3>(X(1)),
    estimate.at<gtsam::Vector3>(V(1)), estimate.at<gtsam::Vector3>(W(1)),
    second_interpolation_jacobians[0], second_interpolation_jacobians[1],
    second_interpolation_jacobians[2], second_interpolation_jacobians[3],
    second_interpolation_jacobians[4], second_interpolation_jacobians[5]);
  gtsam::Matrix joint_interpolation_jacobian = gtsam::Matrix::Zero(12, 24);
  joint_interpolation_jacobian.topRows(6) <<
    first_interpolation_jacobians[0], first_interpolation_jacobians[1],
    first_interpolation_jacobians[2], first_interpolation_jacobians[3],
    first_interpolation_jacobians[4], first_interpolation_jacobians[5];
  joint_interpolation_jacobian.bottomRows(6) <<
    second_interpolation_jacobians[0], second_interpolation_jacobians[1],
    second_interpolation_jacobians[2], second_interpolation_jacobians[3],
    second_interpolation_jacobians[4], second_interpolation_jacobians[5];
  const gtsam::Matrix joint_interpolation_covariance =
    joint_interpolation_jacobian * support_covariance *
    joint_interpolation_jacobian.transpose();
  EXPECT_TRUE(joint_interpolation_covariance.allFinite());
  EXPECT_LT(
    (joint_interpolation_covariance -
    joint_interpolation_covariance.transpose()).norm(), 1e-10);
  EXPECT_GE(
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(joint_interpolation_covariance)
    .eigenvalues().minCoeff(), -1e-12);
  EXPECT_GT(joint_interpolation_covariance.topRightCorner(6, 6).norm(), 0.0);

  gtsam::Matrix between_first;
  gtsam::Matrix between_second;
  first_interpolated_pose.between(
    second_interpolated_pose, between_first, between_second);
  gtsam::Matrix relative_jacobian(6, 12);
  relative_jacobian << between_first, between_second;
  const gtsam::Matrix relative_interpolation_covariance =
    relative_jacobian * joint_interpolation_covariance *
    relative_jacobian.transpose();
  const gtsam::Matrix exported_relative_covariance =
    MakeMeasurementCovariance() + relative_interpolation_covariance;
  EXPECT_TRUE(exported_relative_covariance.allFinite());
  EXPECT_GE(
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix>(exported_relative_covariance)
    .eigenvalues().minCoeff(), -1e-12);

  gtsam::NonlinearFactorGraph later_graph;
  constexpr double later_time = 2.5;
  later_graph.emplace_shared<fgo::factor::GPWNOAPrior>(
    X(1), V(1), W(1), X(2), V(2), W(2), later_time - kDeltaT,
    qc_model, false, true);
  gtsam::Values later_values;
  later_values.insert(X(2), PoseAt(later_time, motion.velocity));
  later_values.insert(V(2), motion.velocity);
  later_values.insert(W(2), motion.omega);
  fgo::solvers::FixedLagSmoother::KeyTimestampMap later_timestamps{
    {X(2), later_time}, {V(2), later_time}, {W(2), later_time}};
  EXPECT_NO_THROW(smoother.update(later_graph, later_values, later_timestamps));
  const gtsam::Values fixed_lag_estimate = smoother.calculateEstimate();
  EXPECT_FALSE(fixed_lag_estimate.exists(X(0)));
  EXPECT_TRUE(fixed_lag_estimate.exists(X(1)));
  EXPECT_TRUE(fixed_lag_estimate.exists(X(2)));
}

}  // namespace
