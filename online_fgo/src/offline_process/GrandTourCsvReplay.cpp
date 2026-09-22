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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "factor/motion/GPWNOJPrior.h"
#include "factor/odometry/BetweenFactor.h"
#include "offline_process/GrandTourImuSupport.h"
#include "solver/IncrementalFixedLagSmoother.h"

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kNanosecondsToSeconds = 1e-9;
constexpr double kSmootherLagSeconds = 3.0;

// Frozen CPT7 continuous densities used by the validated GrandTour 15D host.
constexpr double kGyroscopeDensity = 1.113913052107e-5;
constexpr double kAccelerometerDensity = 5.928679563445e-4;
constexpr double kGyroscopeBiasDensity = 0.000266;
constexpr double kAccelerometerBiasDensity = 0.0043;

struct CsvTable
{
  std::vector<std::string> header;
  std::vector<std::unordered_map<std::string, std::string>> rows;
};

struct ImuSample
{
  int64_t timestamp_ns{};
  gtsam::Vector3 acceleration{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_rate{gtsam::Vector3::Zero()};
};

struct RelativeBelief
{
  size_t ordinal{};
  uint64_t from_index{};
  uint64_t to_index{};
  int64_t from_timestamp_ns{};
  int64_t to_timestamp_ns{};
  int64_t record_timestamp_ns{};
  gtsam::Pose3 measurement;
  gtsam::Matrix6 covariance{gtsam::Matrix6::Zero()};
};

struct Options
{
  std::string imu_path;
  std::string beliefs_path;
  std::string output_directory;
};

std::vector<std::string> SplitCsvLine(std::string line)
{
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::stringstream stream(line);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

CsvTable ReadCsv(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open CSV: " + path);
  }
  CsvTable table;
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("empty CSV: " + path);
  }
  table.header = SplitCsvLine(line);
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() != table.header.size()) {
      throw std::runtime_error("CSV width mismatch in " + path);
    }
    std::unordered_map<std::string, std::string> row;
    for (size_t column = 0; column < fields.size(); ++column) {
      row.emplace(table.header[column], fields[column]);
    }
    table.rows.push_back(std::move(row));
  }
  return table;
}

const std::string & Field(
  const std::unordered_map<std::string, std::string> & row,
  const std::string & name)
{
  const auto found = row.find(name);
  if (found == row.end()) {
    throw std::runtime_error("missing CSV field: " + name);
  }
  return found->second;
}

Options ParseOptions(int argc, char ** argv)
{
  Options options;
  for (int argument = 1; argument < argc; ++argument) {
    const std::string option(argv[argument]);
    if (argument + 1 >= argc) {
      throw std::runtime_error("missing value for " + option);
    }
    const std::string value(argv[++argument]);
    if (option == "--imu") {
      options.imu_path = value;
    } else if (option == "--beliefs") {
      options.beliefs_path = value;
    } else if (option == "--out") {
      options.output_directory = value;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (options.imu_path.empty() || options.beliefs_path.empty() ||
    options.output_directory.empty())
  {
    throw std::runtime_error(
            "usage: grandtour_ct_replay --imu IMU.csv --beliefs BELIEFS.csv --out DIRECTORY");
  }
  return options;
}

std::vector<ImuSample> LoadImu(const std::string & path)
{
  std::vector<ImuSample> samples;
  for (const auto & row : ReadCsv(path).rows) {
    ImuSample sample;
    sample.timestamp_ns = std::stoll(Field(row, "timestamp_ns"));
    sample.acceleration << std::stod(Field(row, "ax")),
      std::stod(Field(row, "ay")), std::stod(Field(row, "az"));
    sample.angular_rate << std::stod(Field(row, "gx")),
      std::stod(Field(row, "gy")), std::stod(Field(row, "gz"));
    if (!sample.acceleration.allFinite() || !sample.angular_rate.allFinite() ||
      (!samples.empty() && sample.timestamp_ns <= samples.back().timestamp_ns))
    {
      throw std::runtime_error("invalid or non-monotonic IMU CSV");
    }
    samples.push_back(sample);
  }
  if (samples.size() < 2) {
    throw std::runtime_error("insufficient IMU samples");
  }
  return samples;
}

std::vector<RelativeBelief> LoadBeliefs(const std::string & path)
{
  std::vector<RelativeBelief> beliefs;
  for (const auto & row : ReadCsv(path).rows) {
    RelativeBelief belief;
    belief.ordinal = std::stoull(Field(row, "ordinal"));
    belief.from_index = std::stoull(Field(row, "from_index"));
    belief.to_index = std::stoull(Field(row, "to_index"));
    belief.from_timestamp_ns = std::stoll(Field(row, "from_timestamp_ns"));
    belief.to_timestamp_ns = std::stoll(Field(row, "to_timestamp_ns"));
    belief.record_timestamp_ns = std::stoll(Field(row, "record_timestamp_ns"));
    gtsam::Vector6 tangent;
    for (int index = 0; index < 6; ++index) {
      tangent(index) = std::stod(Field(row, "relative_mu_" + std::to_string(index)));
    }
    belief.measurement = gtsam::Pose3::Expmap(tangent);
    for (int row_index = 0; row_index < 6; ++row_index) {
      for (int column_index = 0; column_index < 6; ++column_index) {
        belief.covariance(row_index, column_index) = std::stod(
          Field(
            row, "covariance_" + std::to_string(row_index) + "_" +
            std::to_string(column_index)));
      }
    }
    const double symmetry_error =
      (belief.covariance - belief.covariance.transpose()).cwiseAbs().maxCoeff();
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eigen_solver(belief.covariance);
    if (belief.to_timestamp_ns <= belief.from_timestamp_ns ||
      !belief.covariance.allFinite() || symmetry_error > 1e-12 ||
      eigen_solver.info() != Eigen::Success ||
      eigen_solver.eigenvalues().minCoeff() <= 0.0)
    {
      throw std::runtime_error(
              "invalid relative belief at ordinal " +
              std::to_string(belief.ordinal));
    }
    beliefs.push_back(std::move(belief));
  }
  if (beliefs.empty()) {
    throw std::runtime_error("no relative beliefs");
  }
  return beliefs;
}

size_t LastImuAtOrBefore(const std::vector<ImuSample> & imu, int64_t timestamp_ns)
{
  const auto after = std::upper_bound(
    imu.begin(), imu.end(), timestamp_ns,
    [](int64_t timestamp, const ImuSample & sample) {
      return timestamp < sample.timestamp_ns;
    });
  if (after == imu.begin()) {
    throw std::runtime_error("IMU history does not cover requested timestamp");
  }
  return static_cast<size_t>(std::distance(imu.begin(), after) - 1);
}

// Integrate [start,end] with the frozen ALL_RIGHT interpretation: sample k,
// stamped t_k, owns [t_(k-1),t_k].  Only the exact fractional overlap of that
// support interval is integrated at the two boundaries.
void IntegrateAllRight(
  const std::vector<ImuSample> & imu,
  int64_t start_ns,
  int64_t end_ns,
  gtsam::PreintegratedCombinedMeasurements * preintegration)
{
  if (preintegration == nullptr) {
    throw std::runtime_error("invalid ALL_RIGHT integration interval");
  }
  std::vector<int64_t> timestamps_ns;
  timestamps_ns.reserve(imu.size());
  for (const ImuSample & sample : imu) {
    timestamps_ns.push_back(sample.timestamp_ns);
  }
  for (const fgo::offline::AllRightSegment & segment :
    fgo::offline::MakeAllRightSegments(timestamps_ns, start_ns, end_ns))
  {
    preintegration->integrateMeasurement(
      imu[segment.sample_index].acceleration,
      imu[segment.sample_index].angular_rate,
      segment.durationSeconds());
  }
}

gtsam::Vector3 CorrectedAngularRate(
  const std::vector<ImuSample> & imu,
  int64_t timestamp_ns,
  const gtsam::imuBias::ConstantBias & bias)
{
  const size_t index = LastImuAtOrBefore(imu, timestamp_ns);
  return bias.correctGyroscope(imu[index].angular_rate);
}

gtsam::Vector6 SupportAcceleration(
  const std::vector<ImuSample> & imu,
  int64_t timestamp_ns,
  const gtsam::Pose3 & pose,
  const gtsam::imuBias::ConstantBias & bias,
  const gtsam::Vector3 & gravity)
{
  const size_t index = LastImuAtOrBefore(imu, timestamp_ns);
  const size_t previous = index == 0 ? 0 : index - 1;
  gtsam::Vector3 angular_acceleration = gtsam::Vector3::Zero();
  if (index != previous) {
    const double duration = static_cast<double>(
      imu[index].timestamp_ns - imu[previous].timestamp_ns) * kNanosecondsToSeconds;
    angular_acceleration =
      (imu[index].angular_rate - imu[previous].angular_rate) / duration;
  }
  const gtsam::Vector3 gravity_body = pose.rotation().unrotate(gravity);
  const gtsam::Vector3 linear_acceleration = bias.correctAccelerometer(
    imu[index].acceleration + gravity_body);
  return (gtsam::Vector6() << angular_acceleration, linear_acceleration).finished();
}

std::string JsonNumber(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

void WriteTrajectoryRow(std::ofstream & output, int64_t timestamp_ns, const gtsam::Pose3 & pose)
{
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  output << std::fixed << std::setprecision(9)
         << static_cast<double>(timestamp_ns) * kNanosecondsToSeconds
         << std::setprecision(16) << ' ' << pose.x() << ' ' << pose.y() << ' ' << pose.z()
         << ' ' << quaternion.x() << ' ' << quaternion.y() << ' ' << quaternion.z()
         << ' ' << quaternion.w() << '\n';
}

double PathLength(const std::vector<gtsam::Pose3> & poses)
{
  double length = 0.0;
  for (size_t index = 1; index < poses.size(); ++index) {
    length += (poses[index].translation() - poses[index - 1].translation()).norm();
  }
  return length;
}

int Run(int argc, char ** argv)
{
  const Options options = ParseOptions(argc, argv);
  const std::vector<ImuSample> imu = LoadImu(options.imu_path);
  const std::vector<RelativeBelief> beliefs = LoadBeliefs(options.beliefs_path);

  std::set<int64_t> endpoint_set;
  std::map<int64_t, std::vector<const RelativeBelief *>> beliefs_by_end;
  for (const RelativeBelief & belief : beliefs) {
    endpoint_set.insert(belief.from_timestamp_ns);
    endpoint_set.insert(belief.to_timestamp_ns);
    beliefs_by_end[belief.to_timestamp_ns].push_back(&belief);
  }
  const std::vector<int64_t> endpoints(endpoint_set.begin(), endpoint_set.end());
  std::map<int64_t, size_t> state_index;
  for (size_t index = 0; index < endpoints.size(); ++index) {
    state_index[endpoints[index]] = index;
  }
  for (const RelativeBelief & belief : beliefs) {
    if (state_index.find(belief.from_timestamp_ns) == state_index.end() ||
      state_index.find(belief.to_timestamp_ns) == state_index.end())
    {
      throw std::runtime_error("belief endpoint is absent from support grid");
    }
  }
  if (endpoints.front() < imu.front().timestamp_ns || endpoints.back() > imu.back().timestamp_ns) {
    throw std::runtime_error("IMU does not bracket GrandTour support grid");
  }

  const int64_t initial_timestamp_ns = endpoints.front();
  gtsam::Vector3 mean_acceleration = gtsam::Vector3::Zero();
  gtsam::Vector3 mean_angular_rate = gtsam::Vector3::Zero();
  size_t startup_samples = 0;
  for (const ImuSample & sample : imu) {
    if (sample.timestamp_ns >= initial_timestamp_ns) {
      break;
    }
    mean_acceleration += sample.acceleration;
    mean_angular_rate += sample.angular_rate;
    ++startup_samples;
  }
  if (startup_samples < 20) {
    throw std::runtime_error("insufficient causal startup IMU samples");
  }
  mean_acceleration /= static_cast<double>(startup_samples);
  mean_angular_rate /= static_cast<double>(startup_samples);
  const gtsam::Vector3 gravity = -9.81 * mean_acceleration.normalized();
  const gtsam::Vector3 initial_accelerometer_bias = mean_acceleration + gravity;
  const gtsam::Vector3 initial_gyroscope_bias = mean_angular_rate;
  const gtsam::imuBias::ConstantBias initial_bias(
    initial_accelerometer_bias, initial_gyroscope_bias);

  auto preintegration_parameters =
    boost::make_shared<gtsam::PreintegrationCombinedParams>(gravity);
  preintegration_parameters->accelerometerCovariance =
    std::pow(kAccelerometerDensity, 2) * gtsam::I_3x3;
  preintegration_parameters->gyroscopeCovariance =
    std::pow(kGyroscopeDensity, 2) * gtsam::I_3x3;
  preintegration_parameters->biasAccCovariance =
    std::pow(kAccelerometerBiasDensity, 2) * gtsam::I_3x3;
  preintegration_parameters->biasOmegaCovariance =
    std::pow(kGyroscopeBiasDensity, 2) * gtsam::I_3x3;
  // Retain the stock gnssFGO integration and bias-integration settings. These
  // are not source measurement weights and are not tuned to GrandTour.
  preintegration_parameters->integrationCovariance = 1e-8 * gtsam::I_3x3;
  preintegration_parameters->biasAccOmegaInt = 1e-5 * gtsam::I_6x6;
  preintegration_parameters->omegaCoriolis = gtsam::Vector3::Zero();
  preintegration_parameters->setUse2ndOrderCoriolis(false);

  const gtsam::SharedNoiseModel qc_model =
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));

  gtsam::ISAM2Params isam_parameters;
  isam_parameters.relinearizeThreshold = 0.01;
  isam_parameters.relinearizeSkip = 1;
  isam_parameters.findUnusedFactorSlots = true;
  fgo::solvers::IncrementalFixedLagSmoother smoother(
    kSmootherLagSeconds, isam_parameters);

  std::filesystem::create_directories(options.output_directory);
  std::ofstream trajectory(options.output_directory + "/trajectory.tum");
  std::ofstream state_csv(options.output_directory + "/states.csv");
  std::ofstream residual_csv(options.output_directory + "/belief_residuals.csv");
  if (!trajectory || !state_csv || !residual_csv) {
    throw std::runtime_error("cannot create GrandTour output files");
  }
  state_csv << "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  residual_csv << "ordinal,from_index,to_index,translation_residual,rotation_residual\n";

  gtsam::NonlinearFactorGraph initial_graph;
  gtsam::Values initial_values;
  const gtsam::Pose3 initial_pose;
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::Vector3 initial_omega = CorrectedAngularRate(
    imu, initial_timestamp_ns, initial_bias);
  const gtsam::Vector6 pose_sigmas =
    (gtsam::Vector6() << 0.174533, 0.174533, 0.00174533,
    1e-5, 1e-5, 1e-5).finished();
  const gtsam::Vector3 velocity_sigmas = gtsam::Vector3::Constant(0.001);
  const gtsam::Vector6 bias_sigmas =
    (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished();
  const gtsam::Vector3 omega_sigmas = gtsam::Vector3::Constant(0.1);
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), initial_pose, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    V(0), initial_velocity, gtsam::noiseModel::Diagonal::Sigmas(velocity_sigmas));
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    B(0), initial_bias, gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    W(0), initial_omega, gtsam::noiseModel::Diagonal::Sigmas(omega_sigmas));
  initial_values.insert(X(0), initial_pose);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);
  initial_values.insert(W(0), initial_omega);
  fgo::solvers::FixedLagSmoother::KeyTimestampMap initial_timestamps{
    {X(0), 0.0}, {V(0), 0.0}, {B(0), 0.0}, {W(0), 0.0}};
  smoother.update(initial_graph, initial_values, initial_timestamps);

  std::vector<gtsam::Pose3> poses;
  poses.reserve(endpoints.size());
  poses.push_back(initial_pose);
  WriteTrajectoryRow(trajectory, initial_timestamp_ns, initial_pose);

  auto write_state = [&](size_t index, int64_t timestamp_ns, const gtsam::Values & estimate) {
      const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
      const gtsam::Vector3 velocity = estimate.at<gtsam::Vector3>(V(index));
      const auto bias = estimate.at<gtsam::imuBias::ConstantBias>(B(index));
      const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
      state_csv << index << ',' << timestamp_ns << std::setprecision(17)
                << ',' << pose.x() << ',' << pose.y() << ',' << pose.z()
                << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z()
                << ',' << quaternion.w() << ',' << velocity.x() << ',' << velocity.y()
                << ',' << velocity.z() << ',' << bias.accelerometer().x()
                << ',' << bias.accelerometer().y() << ',' << bias.accelerometer().z()
                << ',' << bias.gyroscope().x() << ',' << bias.gyroscope().y()
                << ',' << bias.gyroscope().z() << '\n';
    };
  write_state(0, initial_timestamp_ns, smoother.calculateEstimate());

  size_t accepted_beliefs = 0;
  size_t maximum_active_values = 4;
  const auto wall_start = std::chrono::steady_clock::now();
  for (size_t index = 1; index < endpoints.size(); ++index) {
    const int64_t previous_timestamp_ns = endpoints[index - 1];
    const int64_t timestamp_ns = endpoints[index];
    const double delta_t = static_cast<double>(
      timestamp_ns - previous_timestamp_ns) * kNanosecondsToSeconds;
    const double relative_time = static_cast<double>(
      timestamp_ns - initial_timestamp_ns) * kNanosecondsToSeconds;

    const gtsam::Values previous_estimate = smoother.calculateEstimate();
    const gtsam::Pose3 previous_pose = previous_estimate.at<gtsam::Pose3>(X(index - 1));
    const gtsam::Vector3 previous_velocity =
      previous_estimate.at<gtsam::Vector3>(V(index - 1));
    const auto previous_bias =
      previous_estimate.at<gtsam::imuBias::ConstantBias>(B(index - 1));

    gtsam::PreintegratedCombinedMeasurements preintegration(
      preintegration_parameters, previous_bias);
    IntegrateAllRight(
      imu, previous_timestamp_ns, timestamp_ns, &preintegration);
    const gtsam::NavState predicted = preintegration.predict(
      gtsam::NavState(previous_pose, previous_velocity), previous_bias);
    const gtsam::Vector3 predicted_omega = CorrectedAngularRate(
      imu, timestamp_ns, previous_bias);
    const gtsam::Vector6 acceleration_i = SupportAcceleration(
      imu, previous_timestamp_ns, previous_pose, previous_bias, gravity);
    const gtsam::Vector6 acceleration_j = SupportAcceleration(
      imu, timestamp_ns, predicted.pose(), previous_bias, gravity);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    graph.emplace_shared<gtsam::CombinedImuFactor>(
      X(index - 1), V(index - 1), X(index), V(index),
      B(index - 1), B(index), preintegration);
    graph.emplace_shared<fgo::factor::GPWNOJPrior>(
      X(index - 1), V(index - 1), W(index - 1),
      X(index), V(index), W(index), acceleration_i, acceleration_j,
      delta_t, qc_model, false, true);
    values.insert(X(index), predicted.pose());
    values.insert(V(index), predicted.v());
    values.insert(B(index), previous_bias);
    values.insert(W(index), predicted_omega);

    const auto ending = beliefs_by_end.find(timestamp_ns);
    if (ending != beliefs_by_end.end()) {
      for (const RelativeBelief * belief : ending->second) {
        const size_t from_state = state_index.at(belief->from_timestamp_ns);
        const size_t to_state = state_index.at(belief->to_timestamp_ns);
        graph.emplace_shared<fgo::factor::BetweenPose3Factor>(
          X(from_state), X(to_state), belief->measurement,
          gtsam::noiseModel::Gaussian::Covariance(belief->covariance));
        ++accepted_beliefs;
      }
    }

    fgo::solvers::FixedLagSmoother::KeyTimestampMap timestamps{
      {X(index), relative_time}, {V(index), relative_time},
      {B(index), relative_time}, {W(index), relative_time}};
    smoother.update(graph, values, timestamps);
    const gtsam::Values estimate = smoother.calculateEstimate();
    maximum_active_values = std::max(maximum_active_values, estimate.size());
    const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
    poses.push_back(pose);
    WriteTrajectoryRow(trajectory, timestamp_ns, pose);
    write_state(index, timestamp_ns, estimate);

    if (ending != beliefs_by_end.end()) {
      for (const RelativeBelief * belief : ending->second) {
        const size_t from_state = state_index.at(belief->from_timestamp_ns);
        if (!estimate.exists(X(from_state))) {
          throw std::runtime_error("fixed lag removed a belief endpoint before use");
        }
        const gtsam::Pose3 predicted_relative =
          estimate.at<gtsam::Pose3>(X(from_state)).between(pose);
        const gtsam::Vector6 residual = belief->measurement.localCoordinates(predicted_relative);
        residual_csv << belief->ordinal << ',' << belief->from_index << ','
                     << belief->to_index << ',' << std::setprecision(17)
                     << residual.tail<3>().norm() << ',' << residual.head<3>().norm()
                     << '\n';
      }
    }
  }
  const auto wall_end = std::chrono::steady_clock::now();
  const double wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();
  const double path_length = PathLength(poses);

  std::ofstream summary(options.output_directory + "/summary.json");
  summary << "{\n"
          << "  \"mode\": \"grandtour_all_right_glim_only\",\n"
          << "  \"imu_support\": \"right_interval_end\",\n"
          << "  \"motion_prior\": \"WNOJ\",\n"
          << "  \"smoother\": \"IncrementalFixedLagSmoother\",\n"
          << "  \"lag_seconds\": " << JsonNumber(kSmootherLagSeconds) << ",\n"
          << "  \"support_states\": " << endpoints.size() << ",\n"
          << "  \"beliefs_loaded\": " << beliefs.size() << ",\n"
          << "  \"beliefs_accepted\": " << accepted_beliefs << ",\n"
          << "  \"startup_imu_samples\": " << startup_samples << ",\n"
          << "  \"duration_seconds\": "
          << JsonNumber(
    static_cast<double>(endpoints.back() - endpoints.front()) *
    kNanosecondsToSeconds) << ",\n"
          << "  \"path_length_m\": " << JsonNumber(path_length) << ",\n"
          << "  \"maximum_active_values\": " << maximum_active_values << ",\n"
          << "  \"wall_seconds\": " << JsonNumber(wall_seconds) << ",\n"
          << "  \"gravity\": [" << JsonNumber(gravity.x()) << ", "
          << JsonNumber(gravity.y()) << ", " << JsonNumber(gravity.z()) << "],\n"
          << "  \"initial_accelerometer_bias\": ["
          << JsonNumber(initial_accelerometer_bias.x()) << ", "
          << JsonNumber(initial_accelerometer_bias.y()) << ", "
          << JsonNumber(initial_accelerometer_bias.z()) << "],\n"
          << "  \"initial_gyroscope_bias\": ["
          << JsonNumber(initial_gyroscope_bias.x()) << ", "
          << JsonNumber(initial_gyroscope_bias.y()) << ", "
          << JsonNumber(initial_gyroscope_bias.z()) << "]\n"
          << "}\n";

  std::cout << "GrandTour CT replay completed: " << accepted_beliefs
            << " beliefs, " << endpoints.size() << " support states, path "
            << path_length << " m, wall " << wall_seconds << " s\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return Run(argc, argv);
  } catch (const std::exception & exception) {
    std::cerr << "grandtour_ct_replay: " << exception.what() << '\n';
    return 1;
  }
}
