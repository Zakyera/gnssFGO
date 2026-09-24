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

#include <Eigen/Eigenvalues>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

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
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "factor/motion/GPWNOJPrior.h"
#include "factor/odometry/BetweenFactor.h"
#include "factor/odometry/GPInterpolatedDoublePose3BetweenFactor.h"
#include "factor/odometry/GPInterpolatedSameIntervalPose3BetweenFactor.h"
#include "factor/odometry/GPInterpolatedSinglePose3BetweenFactor.h"
#include "model/gp_interpolator/GPWNOJInterpolator.h"
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
  size_t revision{};
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
  std::string glim_beliefs_path;
  std::string kimera_beliefs_path;
  std::string output_directory;
  std::string revision_beliefs_path;
  std::string mode{"glim"};
  std::string timeline{"physical"};
  std::string solver{"fixed_lag"};
};

struct EndpointLocation
{
  bool exact{};
  size_t exact_index{};
  size_t left_index{};
  size_t right_index{};
};

struct ScheduledBelief
{
  const RelativeBelief * belief{};
  std::string source;
  EndpointLocation from;
  EndpointLocation to;
  size_t activation_index{};
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
    } else if (option == "--beliefs" || option == "--glim-beliefs") {
      options.glim_beliefs_path = value;
    } else if (option == "--kimera-beliefs") {
      options.kimera_beliefs_path = value;
    } else if (option == "--out") {
      options.output_directory = value;
    } else if (option == "--revision-beliefs") {
      options.revision_beliefs_path = value;
    } else if (option == "--mode") {
      options.mode = value;
    } else if (option == "--timeline") {
      options.timeline = value;
    } else if (option == "--solver") {
      options.solver = value;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (options.imu_path.empty() || options.glim_beliefs_path.empty() ||
    options.output_directory.empty() ||
    (options.mode != "imu" && options.mode != "glim" &&
    options.mode != "kimera" && options.mode != "joint") ||
    (options.timeline != "physical" && options.timeline != "record") ||
    (options.solver != "fixed_lag" && options.solver != "isam2") ||
    (!options.revision_beliefs_path.empty() &&
    (options.solver != "isam2" || options.timeline != "record" ||
    (options.mode != "glim" && options.mode != "kimera"))) ||
    (options.timeline == "record" && options.solver != "isam2") ||
    ((options.mode == "kimera" || options.mode == "joint") &&
    options.kimera_beliefs_path.empty()))
  {
    throw std::runtime_error(
            "usage: grandtour_ct_replay --imu IMU.csv --glim-beliefs GLIM.csv "
            "[--kimera-beliefs KIMERA.csv] --mode imu|glim|kimera|joint "
            "[--timeline physical|record] [--solver fixed_lag|isam2] "
            "[--revision-beliefs REVISION.csv] --out DIRECTORY");
  }
  return options;
}

EndpointLocation LocateEndpoint(
  const std::vector<int64_t> & support_timestamps,
  int64_t timestamp_ns)
{
  const auto at_or_after = std::lower_bound(
    support_timestamps.begin(), support_timestamps.end(), timestamp_ns);
  if (at_or_after != support_timestamps.end() && *at_or_after == timestamp_ns) {
    const size_t index = static_cast<size_t>(
      std::distance(support_timestamps.begin(), at_or_after));
    return {true, index, index, index};
  }
  if (at_or_after == support_timestamps.begin() || at_or_after == support_timestamps.end()) {
    throw std::out_of_range("relative endpoint lacks continuous-time support");
  }
  const size_t right = static_cast<size_t>(
    std::distance(support_timestamps.begin(), at_or_after));
  return {false, 0, right - 1, right};
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
    if (row.count("revision") != 0) {
      belief.revision = std::stoull(Field(row, "revision"));
    }
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

std::shared_ptr<fgo::models::GPInterpolator> MakeWnojInterpolator(
  const EndpointLocation & location,
  int64_t timestamp_ns,
  const std::vector<int64_t> & support_timestamps,
  const std::vector<gtsam::Vector6> & support_accelerations,
  const gtsam::SharedNoiseModel & qc_model)
{
  if (location.exact) {
    throw std::invalid_argument("an exact endpoint does not need an interpolator");
  }
  const double delta_t = static_cast<double>(
    support_timestamps[location.right_index] -
    support_timestamps[location.left_index]) * kNanosecondsToSeconds;
  const double tau = static_cast<double>(
    timestamp_ns - support_timestamps[location.left_index]) *
    kNanosecondsToSeconds;
  auto interpolator = std::make_shared<fgo::models::GPWNOJInterpolator>(
    qc_model, delta_t, tau, true, true);
  interpolator->recalculate(
    delta_t, tau, support_accelerations.at(location.left_index),
    support_accelerations.at(location.right_index));
  return interpolator;
}

gtsam::Pose3 PoseAtEndpoint(
  const EndpointLocation & location,
  int64_t timestamp_ns,
  const std::vector<int64_t> & support_timestamps,
  const std::vector<gtsam::Vector6> & support_accelerations,
  const gtsam::SharedNoiseModel & qc_model,
  const gtsam::Values & estimate)
{
  if (location.exact) {
    return estimate.at<gtsam::Pose3>(X(location.exact_index));
  }
  const auto interpolator = MakeWnojInterpolator(
    location, timestamp_ns, support_timestamps, support_accelerations, qc_model);
  return interpolator->interpolatePose(
    estimate.at<gtsam::Pose3>(X(location.left_index)),
    estimate.at<gtsam::Vector3>(V(location.left_index)),
    estimate.at<gtsam::Vector3>(W(location.left_index)),
    estimate.at<gtsam::Pose3>(X(location.right_index)),
    estimate.at<gtsam::Vector3>(V(location.right_index)),
    estimate.at<gtsam::Vector3>(W(location.right_index)));
}

std::string AddRelativeFactor(
  const ScheduledBelief & scheduled,
  const std::vector<int64_t> & support_timestamps,
  const std::vector<gtsam::Vector6> & support_accelerations,
  const gtsam::SharedNoiseModel & qc_model,
  gtsam::NonlinearFactorGraph * graph)
{
  if (graph == nullptr) {
    throw std::invalid_argument("relative factor graph is null");
  }
  const auto noise = gtsam::noiseModel::Gaussian::Covariance(
    scheduled.belief->covariance);
  const EndpointLocation & first = scheduled.from;
  const EndpointLocation & second = scheduled.to;
  if (first.exact && second.exact) {
    graph->emplace_shared<fgo::factor::BetweenPose3Factor>(
      X(first.exact_index), X(second.exact_index),
      scheduled.belief->measurement, noise);
    return "exact";
  }

  if (!first.exact && second.exact) {
    const auto first_interpolator = MakeWnojInterpolator(
      first, scheduled.belief->from_timestamp_ns, support_timestamps,
      support_accelerations, qc_model);
    graph->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
      X(first.left_index), V(first.left_index), W(first.left_index),
      X(first.right_index), V(first.right_index), W(first.right_index),
      X(second.exact_index), scheduled.belief->measurement, false,
      first_interpolator, noise);
    return "single_first";
  }

  if (first.exact && !second.exact) {
    const auto second_interpolator = MakeWnojInterpolator(
      second, scheduled.belief->to_timestamp_ns, support_timestamps,
      support_accelerations, qc_model);
    graph->emplace_shared<fgo::factor::GPInterpolatedSinglePose3BetweenFactor>(
      X(second.left_index), V(second.left_index), W(second.left_index),
      X(second.right_index), V(second.right_index), W(second.right_index),
      X(first.exact_index), scheduled.belief->measurement, true,
      second_interpolator, noise);
    return "single_second";
  }

  const auto first_interpolator = MakeWnojInterpolator(
    first, scheduled.belief->from_timestamp_ns, support_timestamps,
    support_accelerations, qc_model);
  const auto second_interpolator = MakeWnojInterpolator(
    second, scheduled.belief->to_timestamp_ns, support_timestamps,
    support_accelerations, qc_model);
  if (first.left_index == second.left_index &&
    first.right_index == second.right_index)
  {
    graph->emplace_shared<
      fgo::factor::GPInterpolatedSameIntervalPose3BetweenFactor>(
      X(first.left_index), V(first.left_index), W(first.left_index),
      X(first.right_index), V(first.right_index), W(first.right_index),
      scheduled.belief->measurement, first_interpolator,
      second_interpolator, noise);
    return "same_support";
  }

  graph->emplace_shared<fgo::factor::GPInterpolatedDoublePose3BetweenFactor>(
    X(first.left_index), V(first.left_index), W(first.left_index),
    X(first.right_index), V(first.right_index), W(first.right_index),
    X(second.left_index), V(second.left_index), W(second.left_index),
    X(second.right_index), V(second.right_index), W(second.right_index),
    scheduled.belief->measurement, first_interpolator,
    second_interpolator, noise);
  return "double";
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

class IncrementalBackend
{
public:
  IncrementalBackend(
    const std::string & solver,
    const gtsam::ISAM2Params & parameters)
  : solver_(solver)
  {
    if (solver_ == "isam2") {
      isam2_ = std::make_unique<gtsam::ISAM2>(parameters);
    } else {
      fixed_lag_ = std::make_unique<fgo::solvers::IncrementalFixedLagSmoother>(
        kSmootherLagSeconds, parameters);
    }
  }

  void Update(
    const gtsam::NonlinearFactorGraph & graph,
    const gtsam::Values & values,
    const fgo::solvers::FixedLagSmoother::KeyTimestampMap & timestamps = {})
  {
    if (isam2_) {
      isam2_->update(graph, values);
    } else {
      fixed_lag_->update(graph, values, timestamps);
    }
  }

  gtsam::FactorIndex ReplaceFactor(
    const gtsam::NonlinearFactorGraph & graph,
    const gtsam::FactorIndices & remove_indices)
  {
    if (!isam2_ || graph.size() != 1) {
      throw std::runtime_error("revision replacement requires one factor and full iSAM2");
    }
    const auto result = isam2_->update(graph, gtsam::Values(), remove_indices,
      boost::none, boost::none, boost::none, true);
    if (result.newFactorsIndices.size() != 1) {
      throw std::runtime_error("iSAM2 did not return one persistent factor index");
    }
    return result.newFactorsIndices.front();
  }

  size_t ActiveFactorCount() const
  {
    if (!isam2_) return 0;
    size_t count = 0;
    for (const auto & factor : isam2_->getFactorsUnsafe()) {
      if (factor) ++count;
    }
    return count;
  }

  gtsam::Pose3 Pose(size_t index) const
  {
    if (!isam2_) throw std::runtime_error("diagnostic pose query requires iSAM2");
    return isam2_->calculateEstimate<gtsam::Pose3>(X(index));
  }

  size_t ActiveValuesCount() const
  {
    return isam2_ ? isam2_->getLinearizationPoint().size() : 0;
  }

  gtsam::Values Estimate() const
  {
    return isam2_ ? isam2_->calculateEstimate() : fixed_lag_->calculateEstimate();
  }

private:
  std::string solver_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  std::unique_ptr<fgo::solvers::IncrementalFixedLagSmoother> fixed_lag_;
};

int Run(int argc, char ** argv)
{
  const Options options = ParseOptions(argc, argv);
  const std::vector<ImuSample> imu = LoadImu(options.imu_path);
  const std::vector<RelativeBelief> glim_beliefs = LoadBeliefs(
    options.glim_beliefs_path);
  const std::vector<RelativeBelief> revision_beliefs =
    options.revision_beliefs_path.empty() ? std::vector<RelativeBelief>() :
    LoadBeliefs(options.revision_beliefs_path);
  const std::vector<RelativeBelief> kimera_beliefs =
    options.kimera_beliefs_path.empty() ? std::vector<RelativeBelief>() :
    LoadBeliefs(options.kimera_beliefs_path);

  std::set<int64_t> endpoint_set;
  for (const RelativeBelief & belief : glim_beliefs) {
    endpoint_set.insert(belief.from_timestamp_ns);
    endpoint_set.insert(belief.to_timestamp_ns);
  }
  const std::vector<int64_t> endpoints(endpoint_set.begin(), endpoint_set.end());
  if (endpoints.front() < imu.front().timestamp_ns || endpoints.back() > imu.back().timestamp_ns) {
    throw std::runtime_error("IMU does not bracket GrandTour support grid");
  }

  std::vector<ScheduledBelief> scheduled_beliefs;
  scheduled_beliefs.reserve(glim_beliefs.size() + kimera_beliefs.size());
  size_t skipped_no_support = 0;
  auto schedule = [&](const RelativeBelief & belief, const std::string & source) {
      try {
        const EndpointLocation from = LocateEndpoint(
          endpoints, belief.from_timestamp_ns);
        const EndpointLocation to = LocateEndpoint(endpoints, belief.to_timestamp_ns);
        const size_t activation_index = std::max(
          from.exact ? from.exact_index : from.right_index,
          to.exact ? to.exact_index : to.right_index);
        scheduled_beliefs.push_back({&belief, source, from, to, activation_index});
      } catch (const std::out_of_range &) {
        ++skipped_no_support;
      }
    };
  if (options.mode == "glim" || options.mode == "joint") {
    const auto & selected = options.revision_beliefs_path.empty() ?
      glim_beliefs : revision_beliefs;
    for (const RelativeBelief & belief : selected) {
      schedule(belief, "glim");
    }
  }
  if (options.mode == "kimera" || options.mode == "joint") {
    const auto & selected = options.mode == "kimera" &&
      !options.revision_beliefs_path.empty() ? revision_beliefs : kimera_beliefs;
    for (const RelativeBelief & belief : selected) {
      schedule(belief, "kimera");
    }
  }
  std::vector<std::vector<const ScheduledBelief *>> scheduled_by_support(
    endpoints.size());
  std::vector<const ScheduledBelief *> scheduled_by_arrival;
  scheduled_by_arrival.reserve(scheduled_beliefs.size());
  for (const ScheduledBelief & scheduled : scheduled_beliefs) {
    scheduled_by_support.at(scheduled.activation_index).push_back(&scheduled);
    scheduled_by_arrival.push_back(&scheduled);
  }
  // Stable sort preserves CSV revision order for equal arrival time and ID.
  std::stable_sort(
    scheduled_by_arrival.begin(), scheduled_by_arrival.end(),
    [](const ScheduledBelief * lhs, const ScheduledBelief * rhs) {
      if (lhs->belief->record_timestamp_ns != rhs->belief->record_timestamp_ns) {
        return lhs->belief->record_timestamp_ns < rhs->belief->record_timestamp_ns;
      }
      if (lhs->source != rhs->source) {
        return lhs->source < rhs->source;
      }
      return lhs->belief->ordinal < rhs->belief->ordinal;
    });

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
  if (options.solver == "isam2") {
    // The 40 s inertial control becomes extremely ill-conditioned without an
    // external pose factor. QR preserves the factor-graph objective while
    // avoiding Cholesky's positive-definiteness requirement.
    isam_parameters.factorization = gtsam::ISAM2Params::QR;
  }
  IncrementalBackend backend(options.solver, isam_parameters);

  std::filesystem::create_directories(options.output_directory);
  std::ofstream trajectory(options.output_directory + "/trajectory.tum");
  std::ofstream state_csv(options.output_directory + "/states.csv");
  std::ofstream residual_csv(options.output_directory + "/belief_residuals.csv");
  std::ofstream event_csv(options.output_directory + "/measurement_events.csv");
  std::ofstream revision_csv;
  if (!options.revision_beliefs_path.empty()) {
    revision_csv.open(options.output_directory + "/revision_events.csv");
    revision_csv << "ordinal,from_index,to_index,revision,record_timestamp_ns,"
                    "outcome,old_factor_index,new_factor_index,update_seconds,"
                    "current_position_correction_m,endpoint_position_correction_m,"
                    "active_factors\n";
  }
  std::ofstream acceleration_csv(
    options.output_directory + "/support_accelerations.csv");
  std::ofstream online_trajectory;
  std::ofstream online_state_csv;
  if (options.solver == "isam2") {
    online_trajectory.open(options.output_directory + "/online_trajectory.tum");
    online_state_csv.open(options.output_directory + "/online_states.csv");
  }
  if (!trajectory || !state_csv || !residual_csv || !event_csv || !acceleration_csv ||
    (!options.revision_beliefs_path.empty() && !revision_csv) ||
    (options.solver == "isam2" && (!online_trajectory || !online_state_csv)))
  {
    throw std::runtime_error("cannot create GrandTour output files");
  }
  state_csv << "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  if (options.solver == "isam2") {
    online_state_csv <<
      "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  }
  residual_csv <<
    "source,factor_kind,ordinal,from_index,to_index,translation_residual,rotation_residual\n";
  event_csv <<
    "source,ordinal,from_timestamp_ns,to_timestamp_ns,record_timestamp_ns,"
    "insertion_timestamp_ns,delay_seconds,factor_kind\n";
  acceleration_csv << "state_index,timestamp_ns,alpha_x,alpha_y,alpha_z,a_x,a_y,a_z\n";

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
  backend.Update(initial_graph, initial_values, initial_timestamps);

  std::vector<gtsam::Pose3> poses;
  poses.reserve(endpoints.size());
  if (options.solver != "isam2") {
    poses.push_back(initial_pose);
  }
  std::vector<gtsam::Vector6> support_accelerations;
  support_accelerations.reserve(endpoints.size());
  support_accelerations.push_back(
    SupportAcceleration(
      imu, initial_timestamp_ns, initial_pose, initial_bias, gravity));
  acceleration_csv << 0 << ',' << initial_timestamp_ns;
  for (const double value : support_accelerations.back()) {
    acceleration_csv << ',' << std::setprecision(17) << value;
  }
  acceleration_csv << '\n';
  if (options.solver == "isam2") {
    WriteTrajectoryRow(online_trajectory, initial_timestamp_ns, initial_pose);
  } else {
    WriteTrajectoryRow(trajectory, initial_timestamp_ns, initial_pose);
  }

  auto write_state = [](
    std::ostream & output, size_t index, int64_t timestamp_ns,
    const gtsam::Values & estimate) {
      const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
      const gtsam::Vector3 velocity = estimate.at<gtsam::Vector3>(V(index));
      const auto bias = estimate.at<gtsam::imuBias::ConstantBias>(B(index));
      const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
      output << index << ',' << timestamp_ns << std::setprecision(17)
             << ',' << pose.x() << ',' << pose.y() << ',' << pose.z()
             << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z()
             << ',' << quaternion.w() << ',' << velocity.x() << ',' << velocity.y()
             << ',' << velocity.z() << ',' << bias.accelerometer().x()
             << ',' << bias.accelerometer().y() << ',' << bias.accelerometer().z()
             << ',' << bias.gyroscope().x() << ',' << bias.gyroscope().y()
             << ',' << bias.gyroscope().z() << '\n';
    };
  const gtsam::Values initial_estimate = backend.Estimate();
  if (options.solver == "isam2") {
    write_state(online_state_csv, 0, initial_timestamp_ns, initial_estimate);
  } else {
    write_state(state_csv, 0, initial_timestamp_ns, initial_estimate);
  }

  size_t accepted_beliefs = 0;
  size_t accepted_glim = 0;
  size_t accepted_kimera = 0;
  struct RevisionLedgerEntry {
    size_t latest_revision{};
    gtsam::FactorIndex factor_index{};
    const ScheduledBelief * latest{};
    std::string kind;
  };
  std::map<std::tuple<std::string, uint64_t, uint64_t>, RevisionLedgerEntry>
    revision_ledger;
  size_t revision_insertions = 0;
  size_t revision_removals = 0;
  size_t revision_duplicates = 0;
  size_t revision_stale = 0;
  double revision_update_seconds = 0.0;
  std::map<std::string, size_t> factor_kind_counts;
  size_t maximum_active_values = 4;
  std::vector<std::pair<const ScheduledBelief *, std::string>> accepted;
  accepted.reserve(scheduled_beliefs.size());
  auto record_factor = [&](const ScheduledBelief * scheduled, const std::string & kind,
      int64_t insertion_timestamp_ns) {
      accepted.emplace_back(scheduled, kind);
      ++factor_kind_counts[kind];
      ++accepted_beliefs;
      if (scheduled->source == "glim") {
        ++accepted_glim;
      } else {
        ++accepted_kimera;
      }
      event_csv << scheduled->source << ',' << scheduled->belief->ordinal << ','
                << scheduled->belief->from_timestamp_ns << ','
                << scheduled->belief->to_timestamp_ns << ','
                << scheduled->belief->record_timestamp_ns << ','
                << insertion_timestamp_ns << ',' << std::setprecision(17)
                << static_cast<double>(
        scheduled->belief->record_timestamp_ns -
        scheduled->belief->to_timestamp_ns) * kNanosecondsToSeconds << ','
                << kind << '\n';
    };
  auto write_residual = [&](const ScheduledBelief * scheduled, const std::string & kind,
      const gtsam::Values & estimate) {
      const gtsam::Pose3 from_pose = PoseAtEndpoint(
        scheduled->from, scheduled->belief->from_timestamp_ns, endpoints,
        support_accelerations, qc_model, estimate);
      const gtsam::Pose3 to_pose = PoseAtEndpoint(
        scheduled->to, scheduled->belief->to_timestamp_ns, endpoints,
        support_accelerations, qc_model, estimate);
      const gtsam::Vector6 residual = scheduled->belief->measurement.localCoordinates(
        from_pose.between(to_pose));
      residual_csv << scheduled->source << ',' << kind << ','
                   << scheduled->belief->ordinal << ','
                   << scheduled->belief->from_index << ','
                   << scheduled->belief->to_index << ','
                   << std::setprecision(17) << residual.tail<3>().norm() << ','
                   << residual.head<3>().norm() << '\n';
    };
  size_t next_arrival = 0;
  auto process_arrival = [&](const ScheduledBelief * scheduled,
      int64_t insertion_timestamp_ns) {
      if (!options.revision_beliefs_path.empty()) {
        const RelativeBelief & belief = *scheduled->belief;
        if (insertion_timestamp_ns < endpoints.front()) {
          throw std::runtime_error("revision arrived before first support state");
        }
        const auto id = std::make_tuple(scheduled->source,
          belief.from_index, belief.to_index);
        const auto previous = revision_ledger.find(id);
        const bool known = previous != revision_ledger.end();
        const std::string outcome = !known ? "insert" :
          belief.revision == previous->second.latest_revision ? "duplicate" :
          belief.revision < previous->second.latest_revision ? "stale" : "replace";
        if (!known && belief.revision != 0) {
          throw std::runtime_error("first revision for a BeliefID must be zero");
        }
        if (outcome == "duplicate" || outcome == "stale") {
          if (outcome == "duplicate") ++revision_duplicates;
          else ++revision_stale;
          revision_csv << belief.ordinal << ',' << belief.from_index << ','
            << belief.to_index << ',' << belief.revision << ','
            << belief.record_timestamp_ns << ',' << outcome << ','
            << previous->second.factor_index << ',' << previous->second.factor_index
            << ",0,0,0," << backend.ActiveFactorCount() << '\n';
          return;
        }
        const size_t current_index = std::min(
          endpoints.size() - 1,
          static_cast<size_t>(std::upper_bound(
            endpoints.begin(), endpoints.end(), insertion_timestamp_ns) -
            endpoints.begin() - 1));
        const gtsam::Pose3 before = backend.Pose(current_index);
        const size_t endpoint_support_index = scheduled->to.exact ?
          scheduled->to.exact_index : scheduled->to.right_index;
        const gtsam::Pose3 before_endpoint = backend.Pose(endpoint_support_index);
        gtsam::NonlinearFactorGraph graph;
        const std::string kind = AddRelativeFactor(
          *scheduled, endpoints, support_accelerations, qc_model, &graph);
        const gtsam::FactorIndices remove = known ?
          gtsam::FactorIndices{previous->second.factor_index} : gtsam::FactorIndices{};
        const auto update_start = std::chrono::steady_clock::now();
        const gtsam::FactorIndex factor_index = backend.ReplaceFactor(graph, remove);
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - update_start).count();
        const gtsam::Pose3 after = backend.Pose(current_index);
        const gtsam::Pose3 after_endpoint = backend.Pose(endpoint_support_index);
        const double correction = (after.translation() - before.translation()).norm();
        const double endpoint_correction =
          (after_endpoint.translation() - before_endpoint.translation()).norm();
        revision_update_seconds += elapsed;
        ++revision_insertions;
        if (known) ++revision_removals;
        else {
          ++accepted_beliefs;
          if (scheduled->source == "glim") ++accepted_glim;
          else ++accepted_kimera;
          ++factor_kind_counts[kind];
        }
        const auto old_index = known ? previous->second.factor_index : factor_index;
        revision_ledger[id] = {belief.revision, factor_index, scheduled, kind};
        revision_csv << belief.ordinal << ',' << belief.from_index << ','
          << belief.to_index << ',' << belief.revision << ','
          << belief.record_timestamp_ns << ',' << outcome << ','
          << old_index << ',' << factor_index << ',' << std::setprecision(17)
          << elapsed << ',' << correction << ',' << endpoint_correction << ','
          << backend.ActiveFactorCount()
          << '\n';
        maximum_active_values = std::max(maximum_active_values, backend.ActiveValuesCount());
        return;
      }
      gtsam::NonlinearFactorGraph graph;
      const std::string kind = AddRelativeFactor(
        *scheduled, endpoints, support_accelerations, qc_model, &graph);
      backend.Update(graph, gtsam::Values());
      record_factor(scheduled, kind, insertion_timestamp_ns);
      maximum_active_values = std::max(maximum_active_values, backend.Estimate().size());
    };
  std::vector<const ScheduledBelief *> pending_arrivals;
  const auto wall_start = std::chrono::steady_clock::now();
  for (size_t index = 1; index < endpoints.size(); ++index) {
    const int64_t previous_timestamp_ns = endpoints[index - 1];
    const int64_t timestamp_ns = endpoints[index];
    if (options.timeline == "record") {
      while (next_arrival < scheduled_by_arrival.size() &&
        scheduled_by_arrival[next_arrival]->belief->record_timestamp_ns < timestamp_ns)
      {
        const ScheduledBelief * scheduled = scheduled_by_arrival[next_arrival++];
        if (scheduled->activation_index >= index) {
          // The belief is causally available, but an interpolated endpoint can
          // require the next support state, which has not been created yet.
          // Keep the arrival pending and add its unchanged factor as soon as
          // every referenced support key exists.
          pending_arrivals.push_back(scheduled);
        } else {
          process_arrival(scheduled, scheduled->belief->record_timestamp_ns);
        }
      }
    }
    const double delta_t = static_cast<double>(
      timestamp_ns - previous_timestamp_ns) * kNanosecondsToSeconds;
    const double relative_time = static_cast<double>(
      timestamp_ns - initial_timestamp_ns) * kNanosecondsToSeconds;

    const gtsam::Values previous_estimate = backend.Estimate();
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
    support_accelerations.push_back(acceleration_j);
    acceleration_csv << index << ',' << timestamp_ns;
    for (const double value : acceleration_j) {
      acceleration_csv << ',' << std::setprecision(17) << value;
    }
    acceleration_csv << '\n';

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

    std::vector<std::pair<const ScheduledBelief *, std::string>> added;
    if (options.timeline == "physical") {
      for (const ScheduledBelief * scheduled : scheduled_by_support[index]) {
        const std::string kind = AddRelativeFactor(
          *scheduled, endpoints, support_accelerations, qc_model, &graph);
        added.emplace_back(scheduled, kind);
      }
    }

    fgo::solvers::FixedLagSmoother::KeyTimestampMap timestamps{
      {X(index), relative_time}, {V(index), relative_time},
      {B(index), relative_time}, {W(index), relative_time}};
    backend.Update(graph, values, timestamps);
    for (const auto & [scheduled, kind] : added) {
      record_factor(scheduled, kind, timestamp_ns);
    }
    if (options.timeline == "record") {
      auto pending = pending_arrivals.begin();
      while (pending != pending_arrivals.end()) {
        const ScheduledBelief * scheduled = *pending;
        if (scheduled->activation_index <= index) {
          process_arrival(scheduled, timestamp_ns);
          pending = pending_arrivals.erase(pending);
        } else {
          ++pending;
        }
      }
      while (next_arrival < scheduled_by_arrival.size() &&
        scheduled_by_arrival[next_arrival]->belief->record_timestamp_ns == timestamp_ns)
      {
        const ScheduledBelief * scheduled = scheduled_by_arrival[next_arrival++];
        if (scheduled->activation_index > index) {
          pending_arrivals.push_back(scheduled);
        } else {
          process_arrival(scheduled, scheduled->belief->record_timestamp_ns);
        }
      }
    }
    const gtsam::Values estimate = backend.Estimate();
    maximum_active_values = std::max(maximum_active_values, estimate.size());
    const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
    if (options.solver == "isam2") {
      WriteTrajectoryRow(online_trajectory, timestamp_ns, pose);
      write_state(online_state_csv, index, timestamp_ns, estimate);
    } else {
      poses.push_back(pose);
      WriteTrajectoryRow(trajectory, timestamp_ns, pose);
      write_state(state_csv, index, timestamp_ns, estimate);
      for (const auto & [scheduled, kind] : added) {
        write_residual(scheduled, kind, estimate);
      }
    }
  }
  if (options.timeline == "record") {
    if (!pending_arrivals.empty()) {
      throw std::runtime_error("relative belief has unavailable support states");
    }
    while (next_arrival < scheduled_by_arrival.size()) {
      const ScheduledBelief * scheduled = scheduled_by_arrival[next_arrival++];
      if (scheduled->activation_index >= endpoints.size()) {
        throw std::runtime_error("relative belief has unavailable support states");
      }
      process_arrival(scheduled, scheduled->belief->record_timestamp_ns);
    }
  }
  if (options.solver == "isam2") {
    const gtsam::Values final_estimate = backend.Estimate();
    poses.clear();
    poses.reserve(endpoints.size());
    for (size_t index = 0; index < endpoints.size(); ++index) {
      const gtsam::Pose3 pose = final_estimate.at<gtsam::Pose3>(X(index));
      poses.push_back(pose);
      WriteTrajectoryRow(trajectory, endpoints[index], pose);
      write_state(state_csv, index, endpoints[index], final_estimate);
    }
    if (!options.revision_beliefs_path.empty()) {
      for (const auto & [id, entry] : revision_ledger) {
        write_residual(entry.latest, entry.kind, final_estimate);
      }
    } else {
      for (const auto & [scheduled, kind] : accepted) {
        write_residual(scheduled, kind, final_estimate);
      }
    }
  }
  const auto wall_end = std::chrono::steady_clock::now();
  const double wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();
  const double path_length = PathLength(poses);

  std::ofstream summary(options.output_directory + "/summary.json");
  summary << "{\n"
          << "  \"mode\": \"grandtour_all_right_" << options.mode << "\",\n"
          << "  \"imu_support\": \"right_interval_end\",\n"
          << "  \"motion_prior\": \"WNOJ\",\n"
          << "  \"timeline\": \"" << options.timeline << "\",\n"
          << "  \"solver\": \"" << options.solver << "\",\n"
          << "  \"smoother\": \""
          << (options.solver == "isam2" ? "ISAM2" : "IncrementalFixedLagSmoother")
          << "\",\n"
          << "  \"lag_seconds\": "
          << (options.solver == "isam2" ? "null" : JsonNumber(kSmootherLagSeconds))
          << ",\n"
          << "  \"support_states\": " << endpoints.size() << ",\n"
          << "  \"glim_beliefs_loaded\": " << glim_beliefs.size() << ",\n"
          << "  \"kimera_beliefs_loaded\": " << kimera_beliefs.size() << ",\n"
          << "  \"beliefs_accepted\": " << accepted_beliefs << ",\n"
          << "  \"revision_messages_loaded\": " << revision_beliefs.size() << ",\n"
          << "  \"revision_insertions\": " << revision_insertions << ",\n"
          << "  \"revision_removals\": " << revision_removals << ",\n"
          << "  \"revision_duplicates\": " << revision_duplicates << ",\n"
          << "  \"revision_stale\": " << revision_stale << ",\n"
          << "  \"revision_update_seconds\": " << JsonNumber(revision_update_seconds) << ",\n"
          << "  \"glim_beliefs_accepted\": " << accepted_glim << ",\n"
          << "  \"kimera_beliefs_accepted\": " << accepted_kimera << ",\n"
          << "  \"beliefs_skipped_no_support\": " << skipped_no_support << ",\n"
          << "  \"factor_kinds\": {\"exact\": " << factor_kind_counts["exact"]
          << ", \"single_first\": " << factor_kind_counts["single_first"]
          << ", \"single_second\": " << factor_kind_counts["single_second"]
          << ", \"same_support\": " << factor_kind_counts["same_support"]
          << ", \"double\": " << factor_kind_counts["double"] << "},\n"
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

  std::cout << "GrandTour CT replay (" << options.mode << ", "
            << options.timeline << ", " << options.solver << ") completed: "
            << accepted_beliefs << " beliefs (GLIM " << accepted_glim
            << ", Kimera " << accepted_kimera << "), " << endpoints.size()
            << " support states, path "
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
