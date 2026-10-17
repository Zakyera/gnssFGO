#include <tbb/global_control.h>
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
#include <boost/make_shared.hpp>
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
#include <deque>
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
#include "offline_process/CausalSupportClock.h"
#include "offline_process/SourceModelDiscrepancy.h"
#include "SourceWnoaModel.h"
#include "solver/IncrementalFixedLagSmoother.h"
#include "SnapshotAudit.h"
#include "FullPublicationRefinement.h"
#include "FailureProbe.h"
#include "ExactLinearizationMemo.h"

namespace
{

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::A;
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
  int64_t available_ns{};
  gtsam::Vector3 acceleration{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_rate{gtsam::Vector3::Zero()};
};

struct RelativeBelief
{
  size_t ordinal{};
  size_t revision{};
  std::string source_agent;
  std::string session_id{"default"};
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
  std::string graph_snapshots_out;
  std::string event_stream;
  std::string mode{"glim"};
  std::string timeline{"physical"};
  std::string solver{"fixed_lag"};
  std::string receiver_model{"imu_wnoj"};
  bool audit_use_imu{true};
  bool audit_use_wnoj{true};
  double audit_qc_scale{1.0};
  size_t audit_startup_samples{0};
  std::string support_policy{"glim_endpoints"};
  int64_t support_period_ns{100000000};
  int64_t receiver_duration_ns{0};
  size_t receiver_startup_samples{59};
  fgo::offline::SourceModelNoiseMap source_model_noise;
  std::string kimera_channel{"full"};
  std::string kimera_covariance{"full"};
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
    } else if (option == "--graph-snapshots-out") {
      options.graph_snapshots_out = value;
    } else if (option == "--event-stream") {
      options.event_stream = value;
    } else if (option == "--mode") {
      options.mode = value;
    } else if (option == "--timeline") {
      options.timeline = value;
    } else if (option == "--solver") {
      options.solver = value;
    } else if (option == "--receiver-model") {
      options.receiver_model = value;
    } else if (option == "--audit-use-imu") {
      if (value != "true" && value != "false") {
        throw std::runtime_error("--audit-use-imu must be true or false");
      }
      options.audit_use_imu = value == "true";
    } else if (option == "--audit-use-wnoj") {
      if (value != "true" && value != "false") {
        throw std::runtime_error("--audit-use-wnoj must be true or false");
      }
      options.audit_use_wnoj = value == "true";
    } else if (option == "--audit-qc-scale") {
      options.audit_qc_scale = std::stod(value);
    } else if (option == "--audit-startup-samples") {
      options.audit_startup_samples = std::stoull(value);
    } else if (option == "--support-policy") {
      options.support_policy = value;
    } else if (option == "--support-period-ns") {
      options.support_period_ns = std::stoll(value);
    } else if (option == "--receiver-duration-ns") {
      options.receiver_duration_ns = std::stoll(value);
    } else if (option == "--receiver-startup-samples") {
      options.receiver_startup_samples = std::stoull(value);
    } else if (option == "--source-model-noise") {
      const auto fields = SplitCsvLine(value);
      if (fields.size() != 3 || fields[0].empty()) {
        throw std::runtime_error(
                "--source-model-noise requires SOURCE,ROTATION_SIGMA_RAD,TRANSLATION_SIGMA_M");
      }
      auto parse_sigma = [](const std::string & field) {
          size_t consumed = 0;
          const double sigma = std::stod(field, &consumed);
          if (consumed != field.size()) throw std::runtime_error("invalid source model sigma");
          return sigma;
        };
      const fgo::offline::SourceModelNoise noise{
        parse_sigma(fields[1]), parse_sigma(fields[2])};
      fgo::offline::ValidateSourceModelNoise(noise);
      if (!options.source_model_noise.emplace(fields[0], noise).second) {
        throw std::runtime_error("duplicate --source-model-noise source: " + fields[0]);
      }
    } else if (option == "--kimera-channel") {
      if (value != "full" && value != "rotation" && value != "rho") {
        throw std::runtime_error("--kimera-channel must be full, rotation, or rho");
      }
      options.kimera_channel = value;
    } else if (option == "--kimera-covariance") {
      if (value != "full" && value != "blockdiag") {
        throw std::runtime_error("--kimera-covariance must be full or blockdiag");
      }
      options.kimera_covariance = value;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (!options.event_stream.empty()) {
    if (!options.imu_path.empty() || !options.revision_beliefs_path.empty() ||
      !options.glim_beliefs_path.empty() || !options.kimera_beliefs_path.empty()) {
      throw std::runtime_error("stream mode cannot preload sensor CSVs");
    }
    options.revision_beliefs_path = "@stream";
  }
  if (options.receiver_model != "imu_wnoj" && options.receiver_model != "source_wnoa") {
    throw std::runtime_error("--receiver-model must be imu_wnoj or source_wnoa");
  }
  if (options.receiver_model == "source_wnoa" &&
      (options.mode != "sources" || options.support_policy != "uniform_imu" ||
       !options.audit_use_imu || !options.audit_use_wnoj || options.audit_qc_scale != 1.0 ||
       options.audit_startup_samples != 0 || !options.source_model_noise.empty() ||
       options.kimera_channel != "full" || options.kimera_covariance != "full")) {
    throw std::runtime_error("source_wnoa requires sources/uniform_imu and no legacy model/weight ablations");
  }
  if ((options.imu_path.empty() && options.event_stream.empty()) ||
    (options.glim_beliefs_path.empty() && options.support_policy != "uniform_imu") ||
    options.output_directory.empty() ||
    (options.mode != "imu" && options.mode != "glim" &&
    options.mode != "kimera" && options.mode != "joint" && options.mode != "sources") ||
    (options.support_policy != "glim_endpoints" && options.support_policy != "uniform_imu") ||
    (options.support_policy == "uniform_imu" &&
      (options.timeline != "record" || options.solver != "isam2" ||
       (options.receiver_duration_ns <= 0 && options.event_stream.empty()) || options.support_period_ns <= 0 ||
       options.receiver_startup_samples < 20 || options.audit_startup_samples != 0 ||
       !options.audit_use_imu || !options.audit_use_wnoj || options.audit_qc_scale != 1.0)) ||
    (options.mode == "sources" && (options.support_policy != "uniform_imu" ||
      options.revision_beliefs_path.empty())) ||
    (options.kimera_covariance == "blockdiag" && options.kimera_channel != "full") ||
    (!options.event_stream.empty() && (options.mode != "sources" ||
      options.support_policy != "uniform_imu")) ||
    (options.timeline != "physical" && options.timeline != "record") ||
    (options.solver != "fixed_lag" && options.solver != "isam2") ||
    !std::isfinite(options.audit_qc_scale) || options.audit_qc_scale <= 0.0 ||
    (!options.audit_use_imu && !options.audit_use_wnoj) ||
    ((!options.audit_use_imu || !options.audit_use_wnoj ||
      options.audit_qc_scale != 1.0 || options.audit_startup_samples != 0) &&
      (options.mode != "glim" || options.timeline != "record" || options.solver != "isam2")) ||
    (!options.revision_beliefs_path.empty() &&
    (options.solver != "isam2" || options.timeline != "record" ||
    (options.mode != "glim" && options.mode != "kimera" && options.mode != "joint" &&
      options.mode != "sources"))) ||
    (!options.graph_snapshots_out.empty() &&
    (options.solver != "isam2" || options.timeline != "record" ||
    options.revision_beliefs_path.empty())) ||
    (options.timeline == "record" && options.solver != "isam2") ||
    ((options.mode == "kimera" || options.mode == "joint") &&
    options.kimera_beliefs_path.empty()))
  {
    throw std::runtime_error(
            "usage: grandtour_ct_replay --imu IMU.csv --glim-beliefs GLIM.csv "
            "[--kimera-beliefs KIMERA.csv] --mode imu|glim|kimera|joint "
            "[--timeline physical|record] [--solver fixed_lag|isam2] "
            "[--receiver-model imu_wnoj|source_wnoa] "
            "[--revision-beliefs REVISION.csv] --out DIRECTORY "
            "[--graph-snapshots-out DIRECTORY] "
            "[--audit-use-imu true|false] [--audit-use-wnoj true|false] "
            "[--audit-qc-scale POSITIVE] [--audit-startup-samples COUNT] "
            "[--support-policy glim_endpoints|uniform_imu] "
            "[--support-period-ns 100000000] [--receiver-startup-samples 59] "
            "[--receiver-duration-ns NS] [--event-stream FILE_OR_FIFO] "
            "[--source-model-noise SOURCE,ROT_SIGMA_RAD,TRANS_SIGMA_M] "
            "[--kimera-channel full|rotation|rho] "
            "[--kimera-covariance full|blockdiag]; mode sources accepts arbitrary source_agent "
            "and optional session_id in --revision-beliefs without GLIM/Kimera files");
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
    sample.available_ns = row.count("record_timestamp_ns") ?
      std::stoll(Field(row, "record_timestamp_ns")) : sample.timestamp_ns;
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
    if (row.count("source_agent") != 0) {
      belief.source_agent = Field(row, "source_agent");
    }
    if (row.count("session_id") != 0) belief.session_id = Field(row, "session_id");
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
    if (!tangent.allFinite() || belief.session_id.empty() ||
      belief.to_timestamp_ns <= belief.from_timestamp_ns ||
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

// Transport adapter only: no estimator mathematics. A journal can be a regular
// file, named pipe, or stdin. Arrival order is explicit and never resorted.
// I,arrival_ns,physical_ns,ax,ay,az,gx,gy,gz
// B,arrival_ns,source,session,ordinal,revision,i,j,ti_ns,tj_ns,mu[6],cov[36]
// E,arrival_ns   (explicit orderly end; an unmarked EOF is a transport failure)
struct StreamEvent
{
  char kind{};
  int64_t arrival_ns{};
  ImuSample imu;
  RelativeBelief belief;
};

class EventStream
{
public:
  explicit EventStream(const std::string & path)
  {
    if (path == "-") input_ = &std::cin;
    else {
      file_.open(path);
      if (!file_) throw std::runtime_error("cannot open event stream");
      input_ = &file_;
    }
  }
  StreamEvent Next()
  {
    std::string line;
    if (!std::getline(*input_, line)) throw std::runtime_error("stream disconnected without E marker");
    const auto f = SplitCsvLine(line);
    if (f.size() < 2) throw std::runtime_error("short stream event");
    StreamEvent e;
    e.kind = f[0].size() == 1 ? f[0][0] : '?';
    e.arrival_ns = std::stoll(f[1]);
    if (e.arrival_ns < last_ns_) throw std::runtime_error("stream availability moved backwards");
    last_ns_ = e.arrival_ns;
    ++count_;
    if (e.kind == 'E' && f.size() == 2) return e;
    if (e.kind == 'I' && f.size() == 9) {
      e.imu.timestamp_ns = std::stoll(f[2]);
      e.imu.available_ns = e.arrival_ns;
      for (int i = 0; i < 3; ++i) {
        e.imu.acceleration(i) = std::stod(f[3+i]);
        e.imu.angular_rate(i) = std::stod(f[6+i]);
      }
      if (!e.imu.acceleration.allFinite() || !e.imu.angular_rate.allFinite()) {
        throw std::runtime_error("nonfinite streamed IMU");
      }
      return e;
    }
    if (e.kind != 'B' || f.size() != 52) throw std::runtime_error("invalid stream event schema");
    auto & b = e.belief;
    b.record_timestamp_ns = e.arrival_ns;
    b.source_agent = f[2]; b.session_id = f[3];
    for (const auto & id : {b.source_agent, b.session_id}) {
      if (id.empty() || id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos) {
        throw std::runtime_error("invalid stream source/session");
      }
    }
    b.ordinal = std::stoull(f[4]); b.revision = std::stoull(f[5]);
    b.from_index = std::stoull(f[6]); b.to_index = std::stoull(f[7]);
    b.from_timestamp_ns = std::stoll(f[8]); b.to_timestamp_ns = std::stoll(f[9]);
    gtsam::Vector6 mu;
    for (int i = 0; i < 6; ++i) mu(i) = std::stod(f[10+i]);
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) {
      b.covariance(i,j) = std::stod(f[16+6*i+j]);
    }
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(b.covariance);
    if (!mu.allFinite() || !b.covariance.allFinite() ||
      (b.covariance-b.covariance.transpose()).cwiseAbs().maxCoeff() > 1e-12 ||
      eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0 ||
      b.to_timestamp_ns <= b.from_timestamp_ns || e.arrival_ns < b.to_timestamp_ns) {
      throw std::runtime_error("invalid streamed belief");
    }
    b.measurement = gtsam::Pose3::Expmap(mu);
    return e;
  }
  size_t count() const { return count_; }
private:
  std::ifstream file_;
  std::istream * input_{};
  int64_t last_ns_{std::numeric_limits<int64_t>::min()};
  size_t count_{};
};

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

fgo::offline::WnoaEndpoint FullEndpoint(
  const EndpointLocation & location, int64_t timestamp_ns,
  const std::vector<int64_t> & timestamps, const gtsam::SharedNoiseModel & qc)
{
  if (location.exact) return fgo::offline::WnoaEndpoint::Exact(location.exact_index);
  return fgo::offline::WnoaEndpoint::Between(location.left_index, location.right_index,
    (timestamps.at(location.right_index) - timestamps.at(location.left_index)) * kNanosecondsToSeconds,
    (timestamp_ns - timestamps.at(location.left_index)) * kNanosecondsToSeconds, qc);
}

gtsam::Pose3 PoseAtEndpoint(
  const EndpointLocation & location,
  int64_t timestamp_ns,
  const std::vector<int64_t> & support_timestamps,
  const std::vector<gtsam::Vector6> & support_accelerations,
  const gtsam::SharedNoiseModel & qc_model,
  const gtsam::Values & estimate, bool source_wnoa = false)
{
  if (source_wnoa) return FullEndpoint(location, timestamp_ns, support_timestamps, qc_model).pose(estimate);
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

// Diagnostic-only marginal of the existing relative SE(3) factor. The
// underlying interpolators and full residual/Jacobians are unchanged; only
// rows [phi] or [rho] are selected before the Gaussian noise model is applied.
class SelectedRelativeRowsFactor final : public gtsam::NoiseModelFactor
{
public:
  SelectedRelativeRowsFactor(
    const boost::shared_ptr<gtsam::NoiseModelFactor> & full,
    const gtsam::Matrix3 & covariance, size_t first_row)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(covariance, false),
      full->keys()), full_(full), first_row_(first_row)
  {
    if (first_row_ != 0 && first_row_ != 3) {
      throw std::invalid_argument("selected relative rows must be phi or rho");
    }
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::make_shared<SelectedRelativeRowsFactor>(*this);
  }

  const boost::shared_ptr<gtsam::NoiseModelFactor> & fullFactor() const
  {
    return full_;
  }

  gtsam::Vector unwhitenedError(
    const gtsam::Values & values,
    boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override
  {
    if (jacobians) {
      std::vector<gtsam::Matrix> full_jacobians(keys_.size());
      const gtsam::Vector full_error = full_->unwhitenedError(values, full_jacobians);
      jacobians->resize(full_jacobians.size());
      for (size_t i = 0; i < full_jacobians.size(); ++i) {
        (*jacobians)[i] = full_jacobians[i].middleRows(first_row_, 3);
      }
      return full_error.segment(first_row_, 3);
    }
    return full_->unwhitenedError(values).segment(first_row_, 3);
  }

private:
  boost::shared_ptr<gtsam::NoiseModelFactor> full_;
  size_t first_row_;
};

bool IsKimeraChannelSource(const std::string & source)
{
  // ETH-1's real source registry uses kimera_front; legacy CSV modes use kimera.
  return source == "kimera_front" || source == "kimera";
}

gtsam::Matrix6 FactorCovariance(
  const std::string & source, const gtsam::Matrix6 & source_covariance,
  const std::string & kimera_covariance)
{
  gtsam::Matrix6 factor_covariance = source_covariance;
  if (IsKimeraChannelSource(source) && kimera_covariance == "blockdiag") {
    // Diagnostic only: retain each dense marginal and remove their cross block.
    factor_covariance.block<3, 3>(0, 3).setZero();
    factor_covariance.block<3, 3>(3, 0).setZero();
  }
  return factor_covariance;
}

void ValidateSelectedRowsFactor(
  const gtsam::NonlinearFactorGraph & graph, const gtsam::Values & values,
  const gtsam::Matrix6 & source_covariance, const std::string & channel)
{
  const auto selected = boost::dynamic_pointer_cast<SelectedRelativeRowsFactor>(graph.at(0));
  if (!selected) throw std::runtime_error("Kimera subspace factor is missing");
  const size_t first_row = channel == "rotation" ? 0 : 3;
  const auto full = selected->fullFactor();
  std::vector<gtsam::Matrix> full_jacobians(full->keys().size());
  std::vector<gtsam::Matrix> selected_jacobians(selected->keys().size());
  const gtsam::Vector full_error = full->unwhitenedError(values, full_jacobians);
  const gtsam::Vector selected_error = selected->unwhitenedError(values, selected_jacobians);
  if ((selected_error - full_error.segment(first_row, 3)).cwiseAbs().maxCoeff() > 1e-12) {
    throw std::runtime_error("Kimera subspace residual does not match full-factor rows");
  }
  for (size_t i = 0; i < full_jacobians.size(); ++i) {
    if ((selected_jacobians[i] - full_jacobians[i].middleRows(first_row, 3))
      .cwiseAbs().maxCoeff() > 1e-12) {
      throw std::runtime_error("Kimera subspace Jacobian does not match full-factor rows");
    }
  }
  const auto gaussian = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
    selected->noiseModel());
  if (!gaussian) {
    throw std::runtime_error("Kimera subspace noise model is not Gaussian");
  }
  const double covariance_difference = (gaussian->covariance() -
    source_covariance.block<3, 3>(first_row, first_row)).cwiseAbs().maxCoeff();
  if (covariance_difference > 1e-12) {
    std::ostringstream diagnostic;
    diagnostic << std::setprecision(17)
      << "Kimera subspace covariance differs from marginal block by "
      << covariance_difference << " (actual " << gaussian->covariance()
      << "; expected " << source_covariance.block<3, 3>(first_row, first_row) << ')';
    throw std::runtime_error(diagnostic.str());
  }
}

std::string AddRelativeFactor(
  const ScheduledBelief & scheduled,
  const std::vector<int64_t> & support_timestamps,
  const std::vector<gtsam::Vector6> & support_accelerations,
  const gtsam::SharedNoiseModel & qc_model,
  const gtsam::Matrix6 & covariance,
  const std::string & kimera_channel,
  const std::string & kimera_covariance,
  gtsam::NonlinearFactorGraph * graph, bool source_wnoa = false)
{
  if (graph == nullptr) {
    throw std::invalid_argument("relative factor graph is null");
  }
  const bool preserve_dense_blocks = IsKimeraChannelSource(scheduled.source) &&
    kimera_covariance == "blockdiag";
  const auto noise = gtsam::noiseModel::Gaussian::Covariance(
    covariance, !preserve_dense_blocks);
  const EndpointLocation & first = scheduled.from;
  const EndpointLocation & second = scheduled.to;
  auto finish = [&](const std::string & kind) {
      if (IsKimeraChannelSource(scheduled.source) && kimera_channel != "full") {
        const size_t first_row = kimera_channel == "rotation" ? 0 : 3;
        const gtsam::Matrix3 marginal = covariance.block<3, 3>(first_row, first_row);
        const auto full = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph->at(0));
        if (!full || graph->size() != 1) {
          throw std::runtime_error("Kimera subspace requires one full relative factor");
        }
        graph->at(0) = boost::make_shared<SelectedRelativeRowsFactor>(full, marginal, first_row);
      }
      return kind;
    };
  if (source_wnoa) {
    graph->emplace_shared<e2_memo::MemoFactor<fgo::offline::WnoaRelativeFactor,1>>(
      FullEndpoint(first, scheduled.belief->from_timestamp_ns, support_timestamps, qc_model),
      FullEndpoint(second, scheduled.belief->to_timestamp_ns, support_timestamps, qc_model),
      scheduled.belief->measurement, gtsam::noiseModel::Gaussian::Covariance(covariance, false));
    return first.exact && second.exact ? "exact" : first.exact ? "single_second" :
      second.exact ? "single_first" :
      first.left_index == second.left_index && first.right_index == second.right_index ?
      "same_support" : "double";
  }
  if (first.exact && second.exact) {
    graph->emplace_shared<fgo::factor::BetweenPose3Factor>(
      X(first.exact_index), X(second.exact_index),
      scheduled.belief->measurement, noise);
    return finish("exact");
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
    return finish("single_first");
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
    return finish("single_second");
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
    return finish("same_support");
  }

  graph->emplace_shared<fgo::factor::GPInterpolatedDoublePose3BetweenFactor>(
    X(first.left_index), V(first.left_index), W(first.left_index),
    X(first.right_index), V(first.right_index), W(first.right_index),
    X(second.left_index), V(second.left_index), W(second.left_index),
    X(second.right_index), V(second.right_index), W(second.right_index),
    scheduled.belief->measurement, first_interpolator,
    second_interpolator, noise);
  return finish("double");
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

  template<class Factor> size_t ActiveFactorTypeCount() const
  {
    if (!isam2_) throw std::runtime_error("factor type audit requires iSAM2");
    size_t count = 0;
    for (const auto & factor : isam2_->getFactorsUnsafe())
      if (factor && dynamic_cast<const Factor *>(factor.get())) ++count;
    return count;
  }

  gtsam::KeyVector FactorKeys(gtsam::FactorIndex index) const
  {
    if (!isam2_) throw std::runtime_error("graph snapshot requires iSAM2");
    const auto & factors = isam2_->getFactorsUnsafe();
    if (index >= factors.size() || !factors.at(index)) {
      throw std::runtime_error("graph snapshot found a missing active factor");
    }
    return factors.at(index)->keys();
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

  const gtsam::NonlinearFactorGraph& AuditGraph() const {
    if (!isam2_) throw std::runtime_error("audit requires isam2");
    return isam2_->getFactorsUnsafe();
  }
  const gtsam::ISAM2& AuditSolver() const { return *isam2_; }

  void RefinePublication(size_t index,int64_t physical,const std::string& phase,
                         const std::string& out) {
    if(!std::getenv("WNOJ_STRICT_PUBLICATION"))return;
    if(!isam2_)throw std::runtime_error("full refinement requires iSAM2");
    const auto start=std::chrono::steady_clock::now();
    const auto result=numerical_audit::Refine(isam2_);
    e2_memo::Flush(out,index,physical,phase);
    publication_valid_=result.status=="stationary" && std::isfinite(result.gradient_after) && result.gradient_after<=1e-5;
    unsetenv("AUDIT_ROUNDOFF_PUBLICATION");
    for(const auto& step:result.steps) if(step.roundoff)setenv("AUDIT_ROUNDOFF_PUBLICATION","1",1);
    if(!publication_valid_)setenv("AUDIT_FAILED_PUBLICATION","1",1);
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    const auto path=std::filesystem::path(out)/"publication_convergence.csv";
    const bool exists=std::filesystem::exists(path);
    std::ofstream log(path,std::ios::app);log<<std::setprecision(17);
    if(!exists)log<<"phase,index,physical_ns,attempted,accepted,status,error_before,error_after,gradient_before,gradient_after,seconds\n";
    log<<phase<<','<<index<<','<<physical<<','<<result.attempted<<','<<result.accepted<<','<<result.status
       <<','<<result.error_before<<','<<result.error_after<<','<<result.gradient_before<<','<<result.gradient_after<<','<<seconds<<'\n';
    const auto step_path=std::filesystem::path(out)/"publication_steps.csv";
    const bool steps_exist=std::filesystem::exists(step_path);
    std::ofstream steps(step_path,std::ios::app);steps<<std::setprecision(17);
    if(!steps_exist)steps<<"phase,index,iteration,relinearized,error_before,error_after,gradient_after,accepted,roundoff,objective_allowance,state_step_norm,stable_cost_delta,arithmetic_bound,predicted_delta,predicted_bound,recovery,consensus,qr_disagreement,local_prediction\n";
    for(const auto& s:result.steps)steps<<phase<<','<<index<<','<<s.iteration<<','<<s.relinearized<<','
      <<s.before<<','<<s.after<<','<<s.gradient<<','<<s.accepted<<','<<s.roundoff<<','<<s.allowance<<','<<s.state_step_norm<<','<<s.cost.delta<<','<<s.cost.arithmetic_bound<<','<<s.cost.predicted_delta<<','<<s.cost.predicted_bound<<','<<s.recovery<<','<<s.consensus<<','<<s.qr_disagreement<<','<<s.local_prediction<<'\n';
  }

  void RequireValidPublication() const {
    if(!publication_valid_) {
      const char* out=std::getenv("CACHE_FAILURE_PROBE_DIR");
      if(out)failure_probe::Run(out,isam2_->getFactorsUnsafe(),isam2_->calculateEstimate());
    }
    if(!publication_valid_)throw std::runtime_error("STOP: frozen publication validity criterion failed; diagnostic snapshot retained");
  }
private:
  bool publication_valid_{true};
  std::string solver_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  std::unique_ptr<fgo::solvers::IncrementalFixedLagSmoother> fixed_lag_;
};

int Run(int argc, char ** argv)
{
  const Options options = ParseOptions(argc, argv);
  const bool source_wnoa = options.receiver_model == "source_wnoa";
  if (!source_wnoa || options.solver != "isam2" || options.timeline != "record")
    throw std::runtime_error("isolated WNOA diagnostic requires source_wnoa/isam2/record");
  if(std::getenv("WNOJ_STRICT_PUBLICATION") && (!source_wnoa || options.solver!="isam2"))
    throw std::runtime_error("full publication refinement requires source_wnoa iSAM2");
  const bool use_imu = !source_wnoa && options.audit_use_imu;
  const bool streaming = !options.event_stream.empty();
  if (streaming && std::filesystem::exists(options.output_directory)) {
    throw std::runtime_error("stream receiver requires a new output directory");
  }
  std::vector<ImuSample> imu = streaming ? std::vector<ImuSample>() : LoadImu(options.imu_path);
  std::unique_ptr<EventStream> event_stream;
  if (streaming) event_stream = std::make_unique<EventStream>(options.event_stream);
  std::deque<RelativeBelief> streamed_beliefs;
  const std::vector<RelativeBelief> glim_beliefs = options.glim_beliefs_path.empty() ?
    std::vector<RelativeBelief>() : LoadBeliefs(options.glim_beliefs_path);
  const std::vector<RelativeBelief> revision_beliefs =
    (options.revision_beliefs_path.empty() || streaming) ? std::vector<RelativeBelief>() :
    LoadBeliefs(options.revision_beliefs_path);
  const std::vector<RelativeBelief> kimera_beliefs =
    options.kimera_beliefs_path.empty() ? std::vector<RelativeBelief>() :
    LoadBeliefs(options.kimera_beliefs_path);

  const bool independent_clock = options.support_policy == "uniform_imu";
  std::vector<int64_t> endpoints, support_available;
  fgo::offline::CausalSupportClock clock(options.receiver_startup_samples, options.support_period_ns);
  auto receive_imu = [&](const ImuSample & sample) {
      const auto ready = clock.ObserveImu(sample.timestamp_ns, sample.available_ns);
      imu.push_back(sample);
      for (const auto & support : ready) {
        if (options.receiver_duration_ns <= 0 ||
          support.physical_ns - imu.front().timestamp_ns <= options.receiver_duration_ns) {
          endpoints.push_back(support.physical_ns);
          support_available.push_back(support.available_ns);
        }
      }
    };
  if (streaming) {
    while (endpoints.empty()) {
      const auto event = event_stream->Next();
      if (event.kind == 'E') throw std::runtime_error("stream ended before IMU initialization");
      if (event.kind == 'I') receive_imu(event.imu);
      else streamed_beliefs.push_back(event.belief);
    }
  } else if (independent_clock) {
    // The CSV adapter materializes the clock's output for endpoint lookup.
    // Optimizer keys are still created ONLY at each emitted availability time.
    // Live event-stream mode feeds this same clock incrementally.
    const int64_t stop_ns = imu.front().timestamp_ns + options.receiver_duration_ns;
    for (const auto & sample : imu) {
      for (const auto & ready : clock.ObserveImu(sample.timestamp_ns, sample.available_ns)) {
        if (ready.physical_ns <= stop_ns) {
          endpoints.push_back(ready.physical_ns);
          support_available.push_back(ready.available_ns);
        }
      }
      if (sample.timestamp_ns >= stop_ns) break;
    }
  } else {
    std::set<int64_t> endpoint_set;
    for (const RelativeBelief & belief : glim_beliefs) {
      endpoint_set.insert(belief.from_timestamp_ns);
      endpoint_set.insert(belief.to_timestamp_ns);
    }
    endpoints.assign(endpoint_set.begin(), endpoint_set.end());
    support_available = endpoints;
  }
  if (endpoints.size() < (streaming ? 1U : 2U) || endpoints.front() < imu.front().timestamp_ns ||
    endpoints.back() > imu.back().timestamp_ns) {
    throw std::runtime_error("IMU does not bracket GrandTour support grid");
  }

  std::deque<ScheduledBelief> scheduled_beliefs;
  size_t skipped_no_support = 0;
  std::vector<std::pair<const RelativeBelief *, std::string>> unsupported;
  auto schedule = [&](const RelativeBelief & belief, const std::string & source) {
      if (independent_clock && belief.record_timestamp_ns < belief.to_timestamp_ns) {
        throw std::runtime_error("belief availability precedes its physical endpoint");
      }
      try {
        auto locate = [&](int64_t stamp) -> EndpointLocation {
            if (!streaming) return LocateEndpoint(endpoints, stamp);
            if (stamp < endpoints.front()) throw std::out_of_range("before initialization");
            const int64_t offset = stamp - endpoints.front();
            const size_t left = offset / options.support_period_ns;
            if (offset % options.support_period_ns == 0) return {true, left, left, left};
            return {false, 0, left, left + 1};
          };
        const EndpointLocation from = locate(belief.from_timestamp_ns);
        const EndpointLocation to = locate(belief.to_timestamp_ns);
        const size_t activation_index = std::max(
          from.exact ? from.exact_index : from.right_index,
          to.exact ? to.exact_index : to.right_index);
        scheduled_beliefs.push_back({&belief, source, from, to, activation_index});
      } catch (const std::out_of_range &) {
        ++skipped_no_support;
        unsupported.emplace_back(&belief, source);
      }
    };
  if (options.mode == "sources") {
    for (const auto & belief : revision_beliefs) {
      if (belief.source_agent.empty() ||
        belief.source_agent.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos ||
        belief.session_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos) {
        throw std::runtime_error("source/session must be nonempty safe identifiers");
      }
      const std::string source = belief.source_agent == "g" ? "glim" :
        belief.source_agent == "k" ? "kimera" : belief.source_agent;
      schedule(belief, source);
    }
  } else if (options.mode == "joint" && !options.revision_beliefs_path.empty()) {
    for (const RelativeBelief & belief : revision_beliefs) {
      if (belief.source_agent == "g" || belief.source_agent == "glim") {
        schedule(belief, "glim");
      } else if (belief.source_agent == "k" || belief.source_agent == "kimera") {
        schedule(belief, "kimera");
      } else {
        throw std::runtime_error("joint revision row has invalid source_agent");
      }
    }
  } else if (options.mode == "glim" || options.mode == "joint") {
    const auto & selected = options.revision_beliefs_path.empty() ?
      glim_beliefs : revision_beliefs;
    for (const RelativeBelief & belief : selected) {
      schedule(belief, "glim");
    }
  }
  if (options.mode == "kimera" || (options.mode == "joint" && options.revision_beliefs_path.empty())) {
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
  gtsam::Vector3 gravity = gtsam::Vector3::Zero();
  gtsam::Vector3 initial_accelerometer_bias = gtsam::Vector3::Zero();
  gtsam::Vector3 initial_gyroscope_bias = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias initial_bias;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preintegration_parameters;
  if (!source_wnoa) {
  const size_t available_startup_samples = independent_clock ?
    options.receiver_startup_samples : static_cast<size_t>(
    std::lower_bound(imu.begin(), imu.end(), initial_timestamp_ns,
      [](const ImuSample & sample, int64_t timestamp) {
        return sample.timestamp_ns < timestamp;
      }) - imu.begin());
  const size_t first_startup_sample = options.audit_startup_samples == 0 ||
      options.audit_startup_samples >= available_startup_samples ? 0 :
      available_startup_samples - options.audit_startup_samples;
  for (size_t imu_index = first_startup_sample;
    imu_index < available_startup_samples; ++imu_index) {
    const ImuSample & sample = imu[imu_index];
    mean_acceleration += sample.acceleration;
    mean_angular_rate += sample.angular_rate;
    ++startup_samples;
  }
  if (startup_samples < 20) {
    throw std::runtime_error("insufficient causal startup IMU samples");
  }
  mean_acceleration /= static_cast<double>(startup_samples);
  mean_angular_rate /= static_cast<double>(startup_samples);
  gravity = -9.81 * mean_acceleration.normalized();
  initial_accelerometer_bias = mean_acceleration + gravity;
  initial_gyroscope_bias = mean_angular_rate;
  initial_bias = gtsam::imuBias::ConstantBias(
    initial_accelerometer_bias, initial_gyroscope_bias);

  preintegration_parameters =
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
  }

  // Isolated diagnostic build only: unchanged persistent receiver, fixed Step B profile.
  const std::string audit_profile = std::getenv("WNOJ_INCREMENTAL_PROFILE") ?
    std::getenv("WNOJ_INCREMENTAL_PROFILE") : "baseline";
  if (audit_profile != "nominal")
    throw std::runtime_error("unknown diagnostic WNOJ profile");
  if (audit_profile == "nominal" && (!source_wnoa || options.audit_qc_scale != 1.0))
    throw std::runtime_error("nominal profile requires unchanged source_wnoa settings");
  gtsam::Vector6 audit_qc = gtsam::Vector6::Constant(10000.0 * options.audit_qc_scale);
  if (audit_profile == "nominal") audit_qc << .00375,.00375,.00375,.015,.015,.015;
  const gtsam::SharedNoiseModel qc_model = gtsam::noiseModel::Diagonal::Variances(audit_qc);

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
  std::ofstream clock_csv(options.output_directory + "/support_clock.csv");
  clock_csv << "state_index,physical_timestamp_ns,available_timestamp_ns\n";
  for (size_t i = 0; i < endpoints.size(); ++i) {
    clock_csv << i << ',' << endpoints[i] << ',' << support_available[i] << '\n';
  }
  std::ofstream unsupported_csv(options.output_directory + "/unsupported_beliefs.csv");
  unsupported_csv << "source,session,ordinal,revision,from_timestamp_ns,to_timestamp_ns,reason\n";
  for (const auto & item : unsupported) {
    const auto & b = *item.first;
    unsupported_csv << item.second << ',' << b.session_id << ',' << b.ordinal << ','
      << b.revision << ',' << b.from_timestamp_ns << ',' << b.to_timestamp_ns << ','
      << (b.from_timestamp_ns < endpoints.front() ? "before_initialization" : "after_support_horizon") << '\n';
  }
  std::ofstream dispatch_csv(options.output_directory + "/causal_dispatch.csv");
  dispatch_csv << "source,session,from_index,to_index,revision,arrival_ns,insertion_ns,required_support,created_support,outcome\n";
  std::ofstream trajectory(options.output_directory + "/trajectory.tum");
  std::ofstream state_csv(options.output_directory + "/states.csv");
  std::ofstream residual_csv(options.output_directory + "/belief_residuals.csv");
  std::ofstream source_model_csv(options.output_directory + "/source_model_covariance.csv");
  source_model_csv << "source,session,from_index,to_index,revision,rotation_sigma_rad,translation_sigma_m";
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      source_model_csv << ",reported_" << row << column;
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      source_model_csv << ",effective_" << row << column;
    }
  }
  source_model_csv << '\n';
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
  std::ofstream model_covariance_csv(
    options.output_directory + "/model_covariance.csv");
  std::ofstream online_trajectory;
  std::ofstream online_state_csv;
  std::ofstream graph_pose_csv;
  std::ofstream graph_factor_csv;
  if (options.solver == "isam2") {
    online_trajectory.open(options.output_directory + "/online_trajectory.tum");
    online_state_csv.open(options.output_directory + "/online_states.csv");
  }
  if (!options.graph_snapshots_out.empty()) {
    std::filesystem::create_directories(options.graph_snapshots_out);
    graph_pose_csv.open(options.graph_snapshots_out + "/pose_snapshots.csv");
    graph_factor_csv.open(options.graph_snapshots_out + "/factor_snapshots.csv");
  }
  if (!trajectory || !state_csv || !residual_csv || !source_model_csv || !event_csv || !acceleration_csv ||
    !model_covariance_csv ||
    (!options.revision_beliefs_path.empty() && !revision_csv) ||
    (options.solver == "isam2" && (!online_trajectory || !online_state_csv)) ||
    (!options.graph_snapshots_out.empty() && (!graph_pose_csv || !graph_factor_csv)))
  {
    throw std::runtime_error("cannot create GrandTour output files");
  }
  const std::string state_header = source_wnoa ?
    "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n" :
    "state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  state_csv << state_header;
  if (options.solver == "isam2") {
    online_state_csv << state_header;
  }
  if (!options.graph_snapshots_out.empty()) {
    graph_pose_csv << "snapshot_index,snapshot_timestamp_ns,state_index,state_timestamp_ns,"
      "px,py,pz,qx,qy,qz,qw\n";
    graph_factor_csv << "snapshot_index,snapshot_timestamp_ns,source,from_index,to_index,"
      "revision,factor_index,from_timestamp_ns,to_timestamp_ns,keys\n";
  }
  residual_csv <<
    "source,factor_kind,ordinal,from_index,to_index,translation_residual,rotation_residual,"
    "mahalanobis_squared,r0,r1,r2,r3,r4,r5\n";
  event_csv <<
    "source,ordinal,from_timestamp_ns,to_timestamp_ns,record_timestamp_ns,"
    "insertion_timestamp_ns,delay_seconds,factor_kind\n";
  acceleration_csv << "state_index,timestamp_ns,alpha_x,alpha_y,alpha_z,a_x,a_y,a_z\n";
  model_covariance_csv << "state_index,delta_t_seconds,imu_rotation_trace_rad2,"
    "imu_position_trace_m2,motion_rotation_pose_trace_rad2,"
    "motion_translation_pose_trace_m2\n";

  gtsam::NonlinearFactorGraph initial_graph;
  gtsam::Values initial_values;
  const gtsam::Pose3 initial_pose;
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::Vector3 initial_omega = source_wnoa ? gtsam::Vector3::Zero().eval() :
    CorrectedAngularRate(imu, initial_timestamp_ns, initial_bias);
  // Source-only initial pose fixes the 6D gauge, not an IMU gravity direction.
  const gtsam::Vector6 pose_sigmas = source_wnoa ? gtsam::Vector6::Constant(1e-5).eval() :
    (gtsam::Vector6() << 0.174533, 0.174533, 0.00174533, 1e-5, 1e-5, 1e-5).finished();
  // Broad, finite, zero-mean derivative priors. Engineering constants, not GT
  // fitted: V 10 m/s; W 3 rad/s. No acceleration state. No source data reused.
  const gtsam::Vector3 velocity_sigmas = gtsam::Vector3::Constant(source_wnoa ? 10.0 : 0.001);
  const gtsam::Vector6 bias_sigmas =
    (gtsam::Vector6() << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished();
  const gtsam::Vector3 omega_sigmas = gtsam::Vector3::Constant(source_wnoa ? 3.0 : 0.1);
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(0), initial_pose, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas));
  initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    V(0), initial_velocity, gtsam::noiseModel::Diagonal::Sigmas(velocity_sigmas));
  if (use_imu) {
    initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(0), initial_bias, gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));
  }
  if (options.audit_use_wnoj) {
    initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      W(0), initial_omega, gtsam::noiseModel::Diagonal::Sigmas(omega_sigmas));
  }
  initial_values.insert(X(0), initial_pose);
  initial_values.insert(V(0), initial_velocity);
  if (use_imu) initial_values.insert(B(0), initial_bias);
  if (options.audit_use_wnoj) initial_values.insert(W(0), initial_omega);
  fgo::solvers::FixedLagSmoother::KeyTimestampMap initial_timestamps{
    {X(0), 0.0}, {V(0), 0.0}};
  if (use_imu) initial_timestamps.emplace(B(0), 0.0);
  if (options.audit_use_wnoj) initial_timestamps.emplace(W(0), 0.0);
  backend.Update(initial_graph, initial_values, initial_timestamps);

  std::vector<gtsam::Pose3> poses;
  poses.reserve(endpoints.size());
  if (options.solver != "isam2") {
    poses.push_back(initial_pose);
  }
  std::vector<gtsam::Vector6> support_accelerations;
  support_accelerations.reserve(endpoints.size());
  support_accelerations.push_back(source_wnoa ? gtsam::Vector6::Zero().eval() :
    SupportAcceleration(
      imu, initial_timestamp_ns, initial_pose, initial_bias, gravity));
  if (!source_wnoa) {
  acceleration_csv << 0 << ',' << initial_timestamp_ns;
  for (const double value : support_accelerations.back()) {
    acceleration_csv << ',' << std::setprecision(17) << value;
  }
  acceleration_csv << '\n';
  }
  if (options.solver == "isam2") {
    WriteTrajectoryRow(online_trajectory, initial_timestamp_ns, initial_pose);
  } else {
    WriteTrajectoryRow(trajectory, initial_timestamp_ns, initial_pose);
  }

  auto write_state = [&initial_bias, source_wnoa](
    std::ostream & output, size_t index, int64_t timestamp_ns,
    const gtsam::Values & estimate) {
      const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
      const gtsam::Vector3 velocity = estimate.at<gtsam::Vector3>(V(index));
      const auto bias = estimate.exists(B(index)) ?
        estimate.at<gtsam::imuBias::ConstantBias>(B(index)) : initial_bias;
      const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
      output << index << ',' << timestamp_ns << std::setprecision(17)
             << ',' << pose.x() << ',' << pose.y() << ',' << pose.z()
             << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z()
             << ',' << quaternion.w() << ',' << velocity.x() << ',' << velocity.y()
             << ',' << velocity.z();
      if (source_wnoa) {
        for (double value : estimate.at<gtsam::Vector3>(W(index))) output << ',' << value;
        output << '\n';
        return;
      }
      output << ',' << bias.accelerometer().x()
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
  std::map<std::tuple<std::string, std::string, uint64_t, uint64_t>, RevisionLedgerEntry>
    revision_ledger;
  // Visualization-only snapshots of the actual optimized active iSAM2 graph.
  // They never feed back into the estimator or change its event ordering.
  auto write_graph_snapshot = [&](size_t snapshot_index, size_t current_state_index,
      int64_t timestamp_ns, const gtsam::Values & estimate) {
      if (options.graph_snapshots_out.empty()) return;
      const size_t expected_factors = initial_graph.size() +
        (source_wnoa ? 1 : static_cast<size_t>(use_imu) + options.audit_use_wnoj) *
        current_state_index + revision_ledger.size();
      if (backend.ActiveFactorCount() != expected_factors) {
        throw std::runtime_error("graph snapshot active-factor count mismatch");
      }
      for (size_t state_index = 0; state_index <= current_state_index; ++state_index) {
        const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(state_index));
        const gtsam::Quaternion q = pose.rotation().toQuaternion();
        graph_pose_csv << snapshot_index << ',' << timestamp_ns << ','
          << state_index << ',' << endpoints[state_index] << std::setprecision(17)
          << ',' << pose.x() << ',' << pose.y() << ',' << pose.z()
          << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
      }
      for (const auto & [id, entry] : revision_ledger) {
        const RelativeBelief & belief = *entry.latest->belief;
        const gtsam::KeyVector keys = backend.FactorKeys(entry.factor_index);
        graph_factor_csv << snapshot_index << ',' << timestamp_ns << ','
          << std::get<0>(id) << ',' << std::get<2>(id) << ',' << std::get<3>(id)
          << ',' << entry.latest_revision << ',' << entry.factor_index << ','
          << belief.from_timestamp_ns << ',' << belief.to_timestamp_ns << ',';
        for (size_t key_index = 0; key_index < keys.size(); ++key_index) {
          if (key_index) graph_factor_csv << ';';
          const gtsam::Symbol symbol(keys[key_index]);
          graph_factor_csv << symbol.chr() << symbol.index();
        }
        graph_factor_csv << '\n';
      }
    };
  write_graph_snapshot(0, 0, support_available.front(), initial_estimate);
  size_t revision_insertions = 0;
  size_t revision_removals = 0;
  size_t revision_duplicates = 0;
  size_t revision_stale = 0;
  double revision_update_seconds = 0.0;
  double maximum_factor_covariance_abs_difference = 0.0;
  bool selected_rows_validated = false;
  bool kimera_blockdiag_validated = false;
  double maximum_removed_kimera_cross_covariance = 0.0;
  std::map<std::string, size_t> factor_kind_counts;
  size_t maximum_active_values = initial_values.size();
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
        support_accelerations, qc_model, estimate, source_wnoa);
      const gtsam::Pose3 to_pose = PoseAtEndpoint(
        scheduled->to, scheduled->belief->to_timestamp_ns, endpoints,
        support_accelerations, qc_model, estimate, source_wnoa);
      const gtsam::Vector6 residual = scheduled->belief->measurement.localCoordinates(
        from_pose.between(to_pose));
      const gtsam::Matrix6 effective_covariance = fgo::offline::EffectiveSourceCovariance(
        scheduled->belief->covariance, scheduled->source, options.source_model_noise);
      const double mahalanobis_squared = residual.dot(
        effective_covariance.llt().solve(residual));
      residual_csv << scheduled->source << ',' << kind << ','
                   << scheduled->belief->ordinal << ','
                   << scheduled->belief->from_index << ','
                   << scheduled->belief->to_index << ','
                   << std::setprecision(17) << residual.tail<3>().norm() << ','
                   << residual.head<3>().norm() << ',' << mahalanobis_squared;
      for (int component = 0; component < 6; ++component) {
        residual_csv << ',' << residual(component);
      }
      residual_csv << '\n';
    };
  size_t next_arrival = 0;
  size_t created_support = 0;
  size_t queued_arrivals = 0;
  size_t pending_at_end = 0;
  int64_t maximum_wait_ns = 0;
  std::map<std::string, size_t> source_counts;
  auto process_arrival = [&](const ScheduledBelief * scheduled,
      int64_t insertion_timestamp_ns) {
      if (independent_clock &&
        (insertion_timestamp_ns < scheduled->belief->record_timestamp_ns ||
        insertion_timestamp_ns < support_available[scheduled->activation_index] ||
        scheduled->activation_index > created_support)) {
        throw std::runtime_error("causality violation: unavailable measurement/IMU/support");
      }
      maximum_wait_ns = std::max(maximum_wait_ns,
        insertion_timestamp_ns - scheduled->belief->record_timestamp_ns);
      if (!options.revision_beliefs_path.empty()) {
        const RelativeBelief & belief = *scheduled->belief;
        if (insertion_timestamp_ns < endpoints.front()) {
          throw std::runtime_error("revision arrived before first support state");
        }
        const auto id = std::make_tuple(scheduled->source, belief.session_id,
          belief.from_index, belief.to_index);
        const auto previous = revision_ledger.find(id);
        const bool known = previous != revision_ledger.end();
        const std::string outcome = !known ? "insert" :
          belief.revision == previous->second.latest_revision ? "duplicate" :
          belief.revision < previous->second.latest_revision ? "stale" : "replace";
        if (!known && belief.revision != 0 && !independent_clock) {
          throw std::runtime_error("first revision for a BeliefID must be zero");
        }
        if (known && (belief.from_timestamp_ns != previous->second.latest->belief->from_timestamp_ns ||
          belief.to_timestamp_ns != previous->second.latest->belief->to_timestamp_ns)) {
          throw std::runtime_error("revision changed stable-ID endpoint timestamps");
        }
        if (known && outcome == "duplicate" &&
          (belief.measurement.localCoordinates(previous->second.latest->belief->measurement).norm() > 1e-12 ||
          (belief.covariance - previous->second.latest->belief->covariance).cwiseAbs().maxCoeff() > 1e-12)) {
          throw std::runtime_error("conflicting payload for identical revision");
        }
        dispatch_csv << scheduled->source << ',' << belief.session_id << ',' << belief.from_index
          << ',' << belief.to_index << ',' << belief.revision << ',' << belief.record_timestamp_ns
          << ',' << insertion_timestamp_ns << ',' << scheduled->activation_index << ','
          << created_support << ',' << outcome << '\n';
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
        const size_t current_index = independent_clock ? created_support : std::min(
          endpoints.size() - 1,
          static_cast<size_t>(std::upper_bound(
            endpoints.begin(), endpoints.end(), insertion_timestamp_ns) -
            endpoints.begin() - 1));
        const gtsam::Pose3 before = backend.Pose(current_index);
        const size_t endpoint_support_index = scheduled->to.exact ?
          scheduled->to.exact_index : scheduled->to.right_index;
        const gtsam::Pose3 before_endpoint = backend.Pose(endpoint_support_index);
        gtsam::NonlinearFactorGraph graph;
        const gtsam::Matrix6 effective_covariance = fgo::offline::EffectiveSourceCovariance(
          belief.covariance, scheduled->source, options.source_model_noise);
        const gtsam::Matrix6 factor_covariance = FactorCovariance(
          scheduled->source, effective_covariance, options.kimera_covariance);
        const std::string kind = AddRelativeFactor(
          *scheduled, endpoints, support_accelerations, qc_model,
          factor_covariance, options.kimera_channel, options.kimera_covariance, &graph, source_wnoa);
        if (!selected_rows_validated && IsKimeraChannelSource(scheduled->source) &&
          options.kimera_channel != "full") {
          ValidateSelectedRowsFactor(
            graph, backend.Estimate(), effective_covariance, options.kimera_channel);
          selected_rows_validated = true;
        }
        if (graph.size() != 1) {
          throw std::runtime_error("one revision must create exactly one factor");
        }
        const auto * noise_factor = dynamic_cast<const gtsam::NoiseModelFactor *>(
          graph.at(0).get());
        if (!noise_factor) {
          throw std::runtime_error("relative factor has no noise model");
        }
        const auto * gaussian = dynamic_cast<const gtsam::noiseModel::Gaussian *>(
          noise_factor->noiseModel().get());
        if (!gaussian) {
          throw std::runtime_error("relative factor has non-Gaussian noise");
        }
        const bool selected = IsKimeraChannelSource(scheduled->source) &&
          options.kimera_channel != "full";
        const size_t first_row = options.kimera_channel == "rotation" ? 0 : 3;
        const gtsam::Matrix expected_covariance = selected ?
          gtsam::Matrix(factor_covariance.block<3, 3>(first_row, first_row)) :
          gtsam::Matrix(factor_covariance);
        const double covariance_difference =
          (gaussian->covariance() - expected_covariance).cwiseAbs().maxCoeff();
        maximum_factor_covariance_abs_difference = std::max(
          maximum_factor_covariance_abs_difference, covariance_difference);
        if (IsKimeraChannelSource(scheduled->source) &&
          options.kimera_covariance == "blockdiag") {
          if (gaussian->dim() != 6 || covariance_difference > 1e-12) {
            throw std::runtime_error("Kimera block-diagonal factor covariance is invalid");
          }
          kimera_blockdiag_validated = true;
          maximum_removed_kimera_cross_covariance = std::max(
            maximum_removed_kimera_cross_covariance,
            effective_covariance.block<3, 3>(0, 3).cwiseAbs().maxCoeff());
        }
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
          ++source_counts[scheduled->source];
          if (scheduled->source == "glim") ++accepted_glim;
          else if (scheduled->source == "kimera") ++accepted_kimera;
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
      const gtsam::Matrix6 effective_covariance = fgo::offline::EffectiveSourceCovariance(
        scheduled->belief->covariance, scheduled->source, options.source_model_noise);
      const gtsam::Matrix6 factor_covariance = FactorCovariance(
        scheduled->source, effective_covariance, options.kimera_covariance);
      const std::string kind = AddRelativeFactor(
        *scheduled, endpoints, support_accelerations, qc_model,
        factor_covariance, options.kimera_channel, options.kimera_covariance, &graph, source_wnoa);
      backend.Update(graph, gtsam::Values());
      record_factor(scheduled, kind, insertion_timestamp_ns);
      maximum_active_values = std::max(maximum_active_values, backend.Estimate().size());
    };
  std::vector<const ScheduledBelief *> pending_arrivals;
  std::ofstream stream_progress;
  if (streaming) {
    stream_progress.open(options.output_directory + "/stream_progress.csv");
    stream_progress << "event_count,arrival_ns,wall_unix_ns,created_support,pending\n";
  }
  auto stream_belief = [&](const RelativeBelief & belief) {
      const auto previous_size = scheduled_beliefs.size();
      const std::string source = belief.source_agent == "g" ? "glim" :
        belief.source_agent == "k" ? "kimera" : belief.source_agent;
      schedule(belief, source);
      if (scheduled_beliefs.size() == previous_size) {
        unsupported_csv << source << ',' << belief.session_id << ',' << belief.ordinal << ','
          << belief.revision << ',' << belief.from_timestamp_ns << ',' << belief.to_timestamp_ns
          << ",before_initialization\n";
        return;
      }
      const auto * scheduled = &scheduled_beliefs.back();
      if (scheduled->activation_index > created_support) {
        pending_arrivals.push_back(scheduled);
        ++queued_arrivals;
      } else {
        process_arrival(scheduled, belief.record_timestamp_ns);
      }
    };
  for (const auto & belief : streamed_beliefs) stream_belief(belief);
  auto ensure_stream_support = [&](size_t index) {
      while (index >= endpoints.size()) {
        const auto event = event_stream->Next();
        if (event.kind == 'E') return false;
        if (event.kind == 'I') {
          const size_t before = endpoints.size();
          receive_imu(event.imu);
          for (size_t i = before; i < endpoints.size(); ++i) {
            clock_csv << i << ',' << endpoints[i] << ',' << support_available[i] << '\n';
          }
        } else {
          streamed_beliefs.push_back(event.belief);
          stream_belief(streamed_beliefs.back());
        }
        const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        stream_progress << event_stream->count() << ',' << event.arrival_ns << ','
          << wall_ns << ',' << created_support << ',' << pending_arrivals.size() << '\n';
        stream_progress.flush();
      }
      return true;
    };
  const auto wall_start = std::chrono::steady_clock::now();
  for (size_t index = 1;
    streaming ? ensure_stream_support(index) : index < endpoints.size(); ++index) {
    const int64_t previous_timestamp_ns = endpoints[index - 1];
    const int64_t timestamp_ns = endpoints[index];
    const int64_t ready_ns = support_available[index];
    if (options.timeline == "record") {
      while (next_arrival < scheduled_by_arrival.size() &&
        scheduled_by_arrival[next_arrival]->belief->record_timestamp_ns < ready_ns)
      {
        const ScheduledBelief * scheduled = scheduled_by_arrival[next_arrival++];
        if (scheduled->activation_index >= index ||
          (independent_clock && scheduled->belief->record_timestamp_ns < support_available.front())) {
          // The belief is causally available, but an interpolated endpoint can
          // require the next support state, which has not been created yet.
          // Keep the arrival pending and add its unchanged factor as soon as
          // every referenced support key exists.
          pending_arrivals.push_back(scheduled);
          ++queued_arrivals;
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
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    if (source_wnoa) {
      const auto predicted = fgo::offline::PredictSourceWnoa(
        fgo::offline::ReadSourceWnoaState(previous_estimate, index - 1), delta_t);
      fgo::offline::InsertSourceWnoaState(values, index, predicted);
      graph.emplace_shared<e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0>>(
        X(index - 1), V(index - 1), W(index - 1),
        X(index), V(index), W(index), delta_t, qc_model, true, true);
      const double wnoj_pose_variance_rot = audit_qc[0] * std::pow(delta_t, 3.0) / 3.0;
      const double wnoj_pose_variance_trans = audit_qc[3] * std::pow(delta_t, 3.0) / 3.0;
      // Empty IMU covariance columns mean absent, not a zero-noise factor.
      model_covariance_csv << index << ',' << std::setprecision(17) << delta_t
        << ",,," << 3.0 * wnoj_pose_variance_rot << ',' << 3.0 * wnoj_pose_variance_trans << '\n';
    } else {
    const gtsam::Pose3 previous_pose = previous_estimate.at<gtsam::Pose3>(X(index - 1));
    const gtsam::Vector3 previous_velocity =
      previous_estimate.at<gtsam::Vector3>(V(index - 1));
    const auto previous_bias = options.audit_use_imu ?
      previous_estimate.at<gtsam::imuBias::ConstantBias>(B(index - 1)) :
      initial_bias;

    gtsam::PreintegratedCombinedMeasurements preintegration(
      preintegration_parameters, previous_bias);
    IntegrateAllRight(
      imu, previous_timestamp_ns, timestamp_ns, &preintegration);
    const gtsam::Matrix imu_covariance = preintegration.preintMeasCov();
    const double wnoj_pose_variance =
      10000.0 * options.audit_qc_scale * std::pow(delta_t, 5.0) / 20.0;
    model_covariance_csv << index << ',' << std::setprecision(17) << delta_t
      << ',' << imu_covariance.block(0, 0, 3, 3).trace()
      << ',' << imu_covariance.block(3, 3, 3, 3).trace()
      << ',' << 3.0 * wnoj_pose_variance
      << ',' << 3.0 * wnoj_pose_variance << '\n';
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

    if (options.audit_use_imu) {
      graph.emplace_shared<gtsam::CombinedImuFactor>(
        X(index - 1), V(index - 1), X(index), V(index),
        B(index - 1), B(index), preintegration);
    }
    if (options.audit_use_wnoj) {
      graph.emplace_shared<fgo::factor::GPWNOJPrior>(
        X(index - 1), V(index - 1), W(index - 1),
        X(index), V(index), W(index), acceleration_i, acceleration_j,
        delta_t, qc_model, false, true);
    }
    values.insert(X(index), predicted.pose());
    values.insert(V(index), predicted.v());
    if (options.audit_use_imu) values.insert(B(index), previous_bias);
    if (options.audit_use_wnoj) values.insert(W(index), predicted_omega);
    }

    std::vector<std::pair<const ScheduledBelief *, std::string>> added;
    if (options.timeline == "physical") {
      for (const ScheduledBelief * scheduled : scheduled_by_support[index]) {
        const gtsam::Matrix6 effective_covariance = fgo::offline::EffectiveSourceCovariance(
          scheduled->belief->covariance, scheduled->source, options.source_model_noise);
        const gtsam::Matrix6 factor_covariance = FactorCovariance(
          scheduled->source, effective_covariance, options.kimera_covariance);
        const std::string kind = AddRelativeFactor(
          *scheduled, endpoints, support_accelerations, qc_model,
          factor_covariance, options.kimera_channel, options.kimera_covariance, &graph);
        added.emplace_back(scheduled, kind);
      }
    }

    fgo::solvers::FixedLagSmoother::KeyTimestampMap timestamps{
      {X(index), relative_time}, {V(index), relative_time}};
    if (use_imu) timestamps.emplace(B(index), relative_time);
    if (options.audit_use_wnoj) timestamps.emplace(W(index), relative_time);
    backend.Update(graph, values, timestamps);
    created_support = index;
    for (const auto & [scheduled, kind] : added) {
      record_factor(scheduled, kind, timestamp_ns);
    }
    if (options.timeline == "record") {
      auto pending = pending_arrivals.begin();
      while (pending != pending_arrivals.end()) {
        const ScheduledBelief * scheduled = *pending;
        if (scheduled->activation_index <= index) {
          process_arrival(scheduled, ready_ns);
          pending = pending_arrivals.erase(pending);
        } else {
          ++pending;
        }
      }
      while (next_arrival < scheduled_by_arrival.size() &&
        scheduled_by_arrival[next_arrival]->belief->record_timestamp_ns == ready_ns)
      {
        const ScheduledBelief * scheduled = scheduled_by_arrival[next_arrival++];
        if (scheduled->activation_index > index) {
          pending_arrivals.push_back(scheduled);
          ++queued_arrivals;
        } else {
          process_arrival(scheduled, scheduled->belief->record_timestamp_ns);
        }
      }
    }
    backend.RefinePublication(index,timestamp_ns,"support",options.output_directory);
    const gtsam::Values estimate = backend.Estimate();
    if (source_wnoa && snapshot_audit::Selected(index)) {
      const auto dir = std::filesystem::path(options.output_directory) / ("snapshot_" + std::to_string(index));
      std::filesystem::create_directories(dir);
      std::map<size_t,std::string> sources;
      std::ofstream ledger(dir / "ledger.csv");
      ledger << "slot,source,session,from_index,to_index,revision,from_ns,to_ns,arrival_ns\n";
      for (const auto& [id,entry]:revision_ledger) {
        const auto& b=*entry.latest->belief;
        sources[entry.factor_index]=entry.latest->source;
        ledger << entry.factor_index << ',' << std::get<0>(id) << ',' << std::get<1>(id)
          << ',' << b.from_index << ',' << b.to_index << ',' << b.revision << ','
          << b.from_timestamp_ns << ',' << b.to_timestamp_ns << ',' << b.record_timestamp_ns << '\n';
      }
      ledger.close();
      snapshot_audit::Run(dir,backend.AuditGraph(),estimate,endpoints,index,ready_ns,sources,backend.AuditSolver());
    }
    backend.RequireValidPublication();
    write_graph_snapshot(index, index, ready_ns, estimate);
    maximum_active_values = std::max(maximum_active_values, estimate.size());
    const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(X(index));
    if (options.solver == "isam2") {
      WriteTrajectoryRow(online_trajectory, timestamp_ns, pose);
      write_state(online_state_csv, index, timestamp_ns, estimate);
      if (streaming) {
        online_trajectory.flush();
        online_state_csv.flush();
        dispatch_csv.flush();
        clock_csv.flush();
        graph_pose_csv.flush();
        graph_factor_csv.flush();
      }
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
    if (streaming) {
      pending_at_end = pending_arrivals.size();
      for (const auto * pending : pending_arrivals) {
        const auto & b = *pending->belief;
        unsupported_csv << pending->source << ',' << b.session_id << ',' << b.ordinal << ','
          << b.revision << ',' << b.from_timestamp_ns << ',' << b.to_timestamp_ns
          << ",missing_right_support_at_end\n";
      }
      pending_arrivals.clear();
    }
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
    if (streaming && !options.graph_snapshots_out.empty() && !streamed_beliefs.empty()) {
      write_graph_snapshot(endpoints.size(), endpoints.size() - 1,
        std::max(support_available.back(), streamed_beliefs.back().record_timestamp_ns),
        backend.Estimate());
    } else if (!options.graph_snapshots_out.empty() && !scheduled_by_arrival.empty()) {
      const int64_t last_arrival_ns = scheduled_by_arrival.back()->belief->record_timestamp_ns;
      if (last_arrival_ns > endpoints.back()) {
        write_graph_snapshot(endpoints.size(), endpoints.size() - 1,
          last_arrival_ns, backend.Estimate());
      }
    }
  }
  if (options.solver == "isam2") {
    backend.RefinePublication(endpoints.size()-1,endpoints.back(),"final",options.output_directory);
    const gtsam::Values final_estimate = backend.Estimate();
    if (source_wnoa && std::getenv("WNOJ_AUDIT_INDICES")) {
      const auto dir=std::filesystem::path(options.output_directory)/"snapshot_final";
      std::filesystem::create_directories(dir);
      std::map<size_t,std::string> sources;
      std::ofstream ledger(dir/"ledger.csv");
      ledger << "slot,source,session,from_index,to_index,revision,from_ns,to_ns,arrival_ns\n";
      int64_t available=support_available.back();
      for(const auto& [id,entry]:revision_ledger) {
        const auto& b=*entry.latest->belief;
        sources[entry.factor_index]=entry.latest->source;
        available=std::max(available,b.record_timestamp_ns);
        ledger << entry.factor_index << ',' << std::get<0>(id) << ',' << std::get<1>(id)
          << ',' << b.from_index << ',' << b.to_index << ',' << b.revision << ','
          << b.from_timestamp_ns << ',' << b.to_timestamp_ns << ',' << b.record_timestamp_ns << '\n';
      }
      ledger.close();
      snapshot_audit::Run(dir,backend.AuditGraph(),final_estimate,endpoints,endpoints.size()-1,
                          available,sources,backend.AuditSolver());
    }
    if (source_wnoa) {
      if (backend.ActiveFactorCount() != 3 + endpoints.size() - 1 + revision_ledger.size() ||
          final_estimate.size() != 3 * endpoints.size() ||
          backend.ActiveFactorTypeCount<gtsam::CombinedImuFactor>() != 0 ||
          backend.ActiveFactorTypeCount<fgo::factor::GPWNOAPrior>() != endpoints.size() - 1 ||
          backend.ActiveFactorTypeCount<fgo::offline::WnoaRelativeFactor>() != revision_ledger.size())
        throw std::runtime_error("source WNOA final graph accounting mismatch");
      for (size_t i = 0; i < endpoints.size(); ++i) {
        if (final_estimate.exists(B(i)) || final_estimate.exists(A(i)))
          throw std::runtime_error("source WNOA must have neither acceleration nor bias states");
      }
    }
    backend.RequireValidPublication();
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
  if (options.kimera_channel != "full" && !selected_rows_validated) {
    throw std::runtime_error("Kimera channel ablation selected no Kimera factors");
  }
  if (options.kimera_covariance == "blockdiag" &&
    (!kimera_blockdiag_validated || maximum_removed_kimera_cross_covariance == 0.0)) {
    throw std::runtime_error("Kimera block-diagonal diagnostic changed no factors");
  }
  const auto wall_end = std::chrono::steady_clock::now();
  const double wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();
  const double path_length = PathLength(poses);
  fgo::offline::ValidateConfiguredSourceCoverage(options.source_model_noise, source_counts);
  std::ofstream source_csv(options.output_directory + "/source_accounting.csv");
  source_csv << "source,unique_active_beliefs\n";
  for (const auto & item : source_counts) source_csv << item.first << ',' << item.second << '\n';

  auto write_source_covariance = [&](const ScheduledBelief * scheduled) {
      const auto & belief = *scheduled->belief;
      const auto noise = options.source_model_noise.find(scheduled->source);
      const fgo::offline::SourceModelNoise model = noise == options.source_model_noise.end() ?
        fgo::offline::SourceModelNoise{} : noise->second;
      const gtsam::Matrix6 effective = fgo::offline::EffectiveSourceCovariance(
        belief.covariance, scheduled->source, options.source_model_noise);
      source_model_csv << scheduled->source << ',' << belief.session_id << ','
        << belief.from_index << ',' << belief.to_index << ',' << belief.revision << ','
        << std::setprecision(17) << model.rotation_sigma_rad << ','
        << model.translation_sigma_m;
      for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
          source_model_csv << ',' << belief.covariance(row, column);
        }
      }
      for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
          source_model_csv << ',' << effective(row, column);
        }
      }
      source_model_csv << '\n';
    };
  if (!options.revision_beliefs_path.empty()) {
    for (const auto & [id, entry] : revision_ledger) write_source_covariance(entry.latest);
  } else {
    for (const auto & [scheduled, kind] : accepted) write_source_covariance(scheduled);
  }

  std::ofstream summary(options.output_directory + "/summary.json");
  summary << "{\n"
          << "  \"receiver_model\": \"" << options.receiver_model << "\",\n"
          << "  \"support_policy\": \"" << options.support_policy << "\",\n"
          << "  \"kimera_channel\": \"" << options.kimera_channel << "\",\n"
          << "  \"kimera_covariance\": \"" << options.kimera_covariance << "\",\n"
          << "  \"selected_rows_validated\": "
          << (selected_rows_validated ? "true" : "false") << ",\n"
          << "  \"kimera_blockdiag_validated\": "
          << (kimera_blockdiag_validated ? "true" : "false") << ",\n"
          << "  \"maximum_removed_kimera_cross_covariance\": "
          << JsonNumber(maximum_removed_kimera_cross_covariance) << ",\n"
          << "  \"streaming\": " << (streaming ? "true" : "false") << ",\n"
          << "  \"pending_at_end\": " << pending_at_end << ",\n"
          << "  \"support_period_ns\": " << options.support_period_ns << ",\n"
          << "  \"initial_timestamp_ns\": " << initial_timestamp_ns << ",\n"
          << "  \"imu_availability\": \"" << (streaming ? "event-stream adapter receipt timestamp" :
             "CSV record_timestamp_ns; physical timestamp fallback") << "\",\n"
          << "  \"queued_arrival_events\": " << queued_arrivals << ",\n"
          << "  \"maximum_support_wait_seconds\": " << JsonNumber(maximum_wait_ns * 1e-9) << ",\n"
          << "  \"mode\": \"" << (source_wnoa ? "source_wnoa_" : "grandtour_all_right_")
          << options.mode << "\",\n"
          << "  \"imu_support\": \"" << (source_wnoa ? "timestamp_clock_only" : "right_interval_end") << "\",\n"
          << "  \"motion_prior\": \""
          << (source_wnoa ? "WNOA_estimated_velocity" : options.audit_use_wnoj ? "WNOJ" : "none") << "\",\n"
          << "  \"audit_use_imu\": "
          << (use_imu ? "true" : "false") << ",\n"
          << "  \"audit_qc_variance\": "
          << (audit_profile == "nominal" ? "null" : JsonNumber(10000.0 * options.audit_qc_scale)) << ",\n"
          << "  \"audit_startup_sample_limit\": "
          << options.audit_startup_samples << ",\n"
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
          << "  \"revision_messages_loaded\": " << (streaming ? streamed_beliefs.size() : revision_beliefs.size()) << ",\n"
          << "  \"revision_insertions\": " << revision_insertions << ",\n"
          << "  \"revision_removals\": " << revision_removals << ",\n"
          << "  \"revision_duplicates\": " << revision_duplicates << ",\n"
          << "  \"revision_stale\": " << revision_stale << ",\n"
          << "  \"maximum_factor_covariance_abs_difference\": "
          << JsonNumber(maximum_factor_covariance_abs_difference) << ",\n"
          << "  \"source_model_noise\": {";
  bool first_source_model = true;
  for (const auto & [source, noise] : options.source_model_noise) {
    if (!first_source_model) summary << ", ";
    first_source_model = false;
    summary << "\"" << source << "\": ["
            << JsonNumber(noise.rotation_sigma_rad) << ", "
            << JsonNumber(noise.translation_sigma_m) << "]";
  }
  summary << "},\n"
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
          << "  \"wall_seconds\": " << JsonNumber(wall_seconds) << ",\n";
  if (source_wnoa) {
    if (audit_profile == "nominal") summary << "  \"experimental_qc_diagonal\": [0.00375,0.00375,0.00375,0.015,0.015,0.015],\n";
    summary << "  \"gravity\": null,\n  \"initial_accelerometer_bias\": null,\n"
      << "  \"initial_gyroscope_bias\": null,\n"
      << "  \"source_wnoa_initial_sigmas\": {\"pose_rad_m\": 0.00001, \"velocity_m_s\": 10, "
      << "\"omega_rad_s\": 3},\n"
      << "  \"imu_factor_count\": 0,\n  \"bias_variable_count\": 0,\n"
      << "  \"wnoa_factor_count\": " << endpoints.size() - 1 << ",\n"
      << "  \"acceleration_variable_count\": " << 0 << ",\n"
      << "  \"initial_prior_count\": 3,\n"
      << "  \"relative_factor_count\": "
      << backend.ActiveFactorTypeCount<fgo::offline::WnoaRelativeFactor>() << ",\n"
      << "  \"active_factor_count\": " << backend.ActiveFactorCount() << "\n}\n";
  } else {
    summary << "  \"gravity\": [" << JsonNumber(gravity.x()) << ", "
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
  }

  std::cout << "GrandTour CT replay (" << options.mode << ", "
            << options.timeline << ", " << options.solver << ") completed: "
            << accepted_beliefs << " beliefs";
  if (options.mode == "sources") {
    std::cout << " (per-source counts: source_accounting.csv)";
  } else {
    std::cout << " (GLIM " << accepted_glim << ", Kimera " << accepted_kimera << ")";
  }
  std::cout << ", " << endpoints.size()
            << " support states, path "
            << path_length << " m, wall " << wall_seconds << " s\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism, 1);
  try {
    return Run(argc, argv);
  } catch (const std::exception & exception) {
    std::cerr << "grandtour_ct_replay: " << exception.what() << '\n';
    return 1;
  }
}
