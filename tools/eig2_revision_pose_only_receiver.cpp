// Diagnostic, disconnected two-chain receiver. No IMU, WNOJ, or GNSS factors.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using gtsam::symbol_shorthand::X;
using Factor = gtsam::BetweenFactor<gtsam::Pose3>;

std::vector<std::string> split(const std::string& line, char delim) {
  std::stringstream stream(line);
  std::vector<std::string> result;
  std::string field;
  while (std::getline(stream, field, delim)) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    result.push_back(field);
  }
  return result;
}

struct Edge {
  size_t i{}, j{}, revision{};
  int64_t record{};
  gtsam::Pose3 mean;
  gtsam::Matrix6 covariance;
};

std::vector<Edge> read_edges(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read edge CSV");
  std::string line;
  std::getline(input, line);
  const auto header = split(line, ',');
  std::map<std::string, size_t> columns;
  for (size_t index = 0; index < header.size(); ++index) columns[header[index]] = index;
  std::vector<Edge> edges;
  while (std::getline(input, line)) {
    const auto fields = split(line, ',');
    if (fields.size() != header.size()) throw std::runtime_error("edge CSV width");
    auto at = [&](const std::string& key) { return fields.at(columns.at(key)); };
    Edge edge;
    edge.i = std::stoull(at("from_index"));
    edge.j = std::stoull(at("to_index"));
    edge.record = std::stoll(at("record_timestamp_ns"));
    if (columns.count("revision")) edge.revision = std::stoull(at("revision"));
    gtsam::Vector6 tangent;
    for (size_t k = 0; k < 6; ++k) {
      tangent(k) = std::stod(at("relative_mu_" + std::to_string(k)));
      for (size_t l = 0; l < 6; ++l) {
        edge.covariance(k, l) = std::stod(at("covariance_" +
          std::to_string(k) + "_" + std::to_string(l)));
      }
    }
    edge.mean = gtsam::Pose3::Expmap(tangent);
    if (edge.j != edge.i + 2 || !edge.covariance.allFinite() ||
        Eigen::SelfAdjointEigenSolver<gtsam::Matrix6>(edge.covariance)
          .eigenvalues().minCoeff() <= 0.0) {
      throw std::runtime_error("invalid 0.20 s edge");
    }
    edges.push_back(std::move(edge));
  }
  std::stable_sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
    return std::tie(a.record, a.i, a.revision) <
           std::tie(b.record, b.i, b.revision);
  });
  return edges;
}

int run(int argc, char** argv) {
  if (argc != 4) throw std::runtime_error("usage: receiver SOURCE.tum EDGES.csv OUT.tum");
  std::ifstream source(argv[1]);
  if (!source) throw std::runtime_error("cannot open source TUM");
  std::vector<std::string> timestamps;
  std::vector<gtsam::Pose3> source_poses;
  std::string line;
  while (std::getline(source, line)) {
    const auto parts = split(line, ' ');
    if (parts.size() != 8) throw std::runtime_error("source TUM width");
    timestamps.push_back(parts[0]);
    const Eigen::Quaterniond quaternion(std::stod(parts[7]), std::stod(parts[4]),
      std::stod(parts[5]), std::stod(parts[6]));
    source_poses.emplace_back(gtsam::Rot3(quaternion),
      gtsam::Point3(std::stod(parts[1]), std::stod(parts[2]), std::stod(parts[3])));
  }
  auto edges = read_edges(argv[2]);
  gtsam::ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = 0.01;
  params.findUnusedFactorSlots = true;
  params.factorization = gtsam::ISAM2Params::QR;
  gtsam::ISAM2 isam(params);
  gtsam::NonlinearFactorGraph initial;
  gtsam::Values values;
  for (size_t root = 0; root < 2; ++root) {
    initial.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(root), source_poses.at(root),
      gtsam::noiseModel::Isotropic::Sigma(6, 1e-8));
    values.insert(X(root), source_poses.at(root));
  }
  isam.update(initial, values);
  std::map<std::pair<size_t, size_t>, std::pair<size_t, gtsam::FactorIndex>> ledger;
  size_t inserted = 0, replaced = 0, duplicate = 0, stale = 0;
  for (const Edge& edge : edges) {
    const auto id = std::make_pair(edge.i, edge.j);
    const auto previous = ledger.find(id);
    if (previous != ledger.end()) {
      if (edge.revision == previous->second.first) { ++duplicate; continue; }
      if (edge.revision < previous->second.first) { ++stale; continue; }
    }
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<Factor>(X(edge.i), X(edge.j), edge.mean,
      gtsam::noiseModel::Gaussian::Covariance(edge.covariance));
    gtsam::Values new_values;
    if (!isam.valueExists(X(edge.j))) {
      new_values.insert(X(edge.j),
        isam.calculateEstimate<gtsam::Pose3>(X(edge.i)).compose(edge.mean));
    }
    const gtsam::FactorIndices remove = previous == ledger.end() ?
      gtsam::FactorIndices{} :
      gtsam::FactorIndices{previous->second.second};
    const auto result = isam.update(graph, new_values, remove,
      boost::none, boost::none, boost::none, true);
    if (result.newFactorsIndices.size() != 1) throw std::runtime_error("factor index");
    ledger[id] = {edge.revision, result.newFactorsIndices.front()};
    if (remove.empty()) ++inserted;
    else ++replaced;
  }
  if (source_poses.size() != inserted + 2) throw std::runtime_error("pose/edge count");
  size_t active = 0;
  for (const auto& factor : isam.getFactorsUnsafe()) if (factor) ++active;
  if (active != inserted + 2) throw std::runtime_error("more than one factor per ID");
  std::ofstream output(argv[3]);
  output << std::setprecision(17);
  for (size_t i = 0; i < source_poses.size(); ++i) {
    const auto pose = isam.calculateEstimate<gtsam::Pose3>(X(i));
    const auto q = pose.rotation().toQuaternion();
    output << timestamps[i] << ' ' << pose.x() << ' ' << pose.y() << ' '
           << pose.z() << ' ' << q.x() << ' ' << q.y() << ' '
           << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "ids=" << inserted << " replacements=" << replaced
            << " duplicate=" << duplicate << " stale=" << stale
            << " active_factors=" << active << '\n';
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(argc, argv); }
  catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
