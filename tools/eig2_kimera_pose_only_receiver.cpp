// Diagnostic Kimera-only relative graph. No IMU, WNOJ, or GNSS factors.
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
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using gtsam::symbol_shorthand::X;
using Factor = gtsam::BetweenFactor<gtsam::Pose3>;

std::vector<std::string> split(const std::string& line, char delimiter) {
  std::stringstream stream(line);
  std::vector<std::string> result;
  std::string field;
  while (std::getline(stream, field, delimiter)) {
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
  if (!input) throw std::runtime_error("cannot open belief CSV");
  std::string line;
  std::getline(input, line);
  auto header = split(line, ',');
  std::map<std::string, size_t> columns;
  for (size_t k = 0; k < header.size(); ++k) columns[header[k]] = k;
  std::vector<Edge> result;
  while (std::getline(input, line)) {
    auto fields = split(line, ',');
    if (fields.size() != header.size()) throw std::runtime_error("belief CSV width");
    auto at = [&](const std::string& key) { return fields.at(columns.at(key)); };
    Edge edge;
    edge.i = std::stoull(at("from_index"));
    edge.j = std::stoull(at("to_index"));
    edge.record = std::stoll(at("record_timestamp_ns"));
    if (columns.count("revision")) edge.revision = std::stoull(at("revision"));
    gtsam::Vector6 tangent;
    for (size_t r = 0; r < 6; ++r) {
      tangent(r) = std::stod(at("relative_mu_" + std::to_string(r)));
      for (size_t c = 0; c < 6; ++c) {
        edge.covariance(r, c) = std::stod(at("covariance_" +
          std::to_string(r) + "_" + std::to_string(c)));
      }
    }
    if (!edge.covariance.allFinite() ||
        Eigen::SelfAdjointEigenSolver<gtsam::Matrix6>(edge.covariance)
          .eigenvalues().minCoeff() <= 0) throw std::runtime_error("bad covariance");
    edge.mean = gtsam::Pose3::Expmap(tangent);
    result.push_back(std::move(edge));
  }
  std::stable_sort(result.begin(), result.end(), [](const Edge& a, const Edge& b) {
    return std::tie(a.record, a.i, a.revision) < std::tie(b.record, b.i, b.revision);
  });
  return result;
}

int run(int argc, char** argv) {
  if (argc != 4) throw std::runtime_error("usage: pose_only SOURCE.tum BELIEFS.csv OUT.tum");
  std::ifstream source(argv[1]);
  if (!source) throw std::runtime_error("cannot open source TUM");
  std::string line;
  std::vector<std::string> stamps;
  std::vector<gtsam::Pose3> source_poses;
  while (std::getline(source, line)) {
    const auto f = split(line, ' ');
    if (f.size() != 8) throw std::runtime_error("source TUM width");
    stamps.push_back(f[0]);
    const Eigen::Quaterniond q(std::stod(f[7]), std::stod(f[4]),
      std::stod(f[5]), std::stod(f[6]));
    source_poses.emplace_back(gtsam::Rot3(q),
      gtsam::Point3(std::stod(f[1]), std::stod(f[2]), std::stod(f[3])));
  }
  const auto edges = read_edges(argv[2]);
  std::set<size_t> represented, nonroots;
  std::map<size_t, size_t> parent;
  for (const auto& edge : edges) {
    if (edge.i >= source_poses.size() || edge.j >= source_poses.size() ||
        edge.i >= edge.j) throw std::runtime_error("invalid source index");
    represented.insert(edge.i);
    represented.insert(edge.j);
    if (parent.count(edge.j) && parent.at(edge.j) != edge.i)
      throw std::runtime_error("multiple parents for pose");
    parent[edge.j] = edge.i;
    nonroots.insert(edge.j);
  }
  std::set<size_t> roots;
  for (const auto index : represented) if (!nonroots.count(index)) roots.insert(index);
  if (roots.empty()) throw std::runtime_error("no graph root");
  gtsam::ISAM2Params parameters;
  parameters.relinearizeSkip = 1;
  parameters.relinearizeThreshold = 0.01;
  parameters.findUnusedFactorSlots = true;
  parameters.factorization = gtsam::ISAM2Params::QR;
  gtsam::ISAM2 isam(parameters);
  gtsam::NonlinearFactorGraph initial;
  gtsam::Values values;
  for (const auto root : roots) {
    initial.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(root), source_poses[root], gtsam::noiseModel::Isotropic::Sigma(6, 1e-8));
    values.insert(X(root), source_poses[root]);
  }
  isam.update(initial, values);
  std::map<std::pair<size_t, size_t>, std::pair<size_t, gtsam::FactorIndex>> ledger;
  size_t inserted = 0, replaced = 0, duplicate = 0, stale = 0;
  for (const auto& edge : edges) {
    const auto id = std::make_pair(edge.i, edge.j);
    const auto previous = ledger.find(id);
    if (previous != ledger.end()) {
      if (edge.revision == previous->second.first) { ++duplicate; continue; }
      if (edge.revision < previous->second.first) { ++stale; continue; }
    }
    if (!isam.valueExists(X(edge.i)))
      throw std::runtime_error("source graph not topologically ordered at arrival");
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<Factor>(X(edge.i), X(edge.j), edge.mean,
      gtsam::noiseModel::Gaussian::Covariance(edge.covariance));
    gtsam::Values new_values;
    if (!isam.valueExists(X(edge.j))) {
      new_values.insert(X(edge.j),
        isam.calculateEstimate<gtsam::Pose3>(X(edge.i)).compose(edge.mean));
    }
    const gtsam::FactorIndices remove = previous == ledger.end() ?
      gtsam::FactorIndices{} : gtsam::FactorIndices{previous->second.second};
    auto updated = isam.update(graph, new_values, remove,
      boost::none, boost::none, boost::none, true);
    if (updated.newFactorsIndices.size() != 1) throw std::runtime_error("factor index");
    ledger[id] = {edge.revision, updated.newFactorsIndices.front()};
    if (remove.empty()) ++inserted;
    else ++replaced;
  }
  if (ledger.size() + roots.size() != represented.size())
    throw std::runtime_error("graph is not a forest");
  std::ofstream output(argv[3]);
  output << std::setprecision(17);
  for (const auto index : represented) {
    const auto p = isam.calculateEstimate<gtsam::Pose3>(X(index));
    const auto q = p.rotation().toQuaternion();
    output << stamps[index] << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
           << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "roots=" << roots.size() << " represented=" << represented.size()
            << " ids=" << inserted << " replacements=" << replaced
            << " duplicate=" << duplicate << " stale=" << stale << '\n';
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
