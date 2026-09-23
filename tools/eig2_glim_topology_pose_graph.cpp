// Isolated offline topology reconstruction; not linked into any estimator.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

using gtsam::Pose3;

std::vector<std::string> Split(const std::string& line, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(field);
  }
  return fields;
}

struct Edge {
  size_t from;
  size_t to;
  Pose3 transform;
};

int main(int argc, char** argv) {
  if (argc != 4 && argc != 6) {
    std::cerr << "usage: eig2_glim_topology_pose_graph NATIVE.tum EDGES.csv OUTPUT.tum [REMOVE_FROM REMOVE_TO]\n";
    return 2;
  }
  const bool remove = argc == 6;
  const size_t remove_from = remove ? std::stoul(argv[4]) : 0;
  const size_t remove_to = remove ? std::stoul(argv[5]) : 0;
  std::ifstream native_file(argv[1]);
  std::vector<std::string> timestamps;
  std::vector<Pose3> native;
  std::string line;
  while (std::getline(native_file, line)) {
    const auto row = Split(line, ' ');
    if (row.size() != 8) throw std::runtime_error("invalid native TUM row");
    timestamps.push_back(row[0]);
    const Eigen::Quaterniond q(std::stod(row[7]), std::stod(row[4]),
                               std::stod(row[5]), std::stod(row[6]));
    native.emplace_back(gtsam::Rot3(q), gtsam::Point3(
        std::stod(row[1]), std::stod(row[2]), std::stod(row[3])));
  }
  if (native.empty()) throw std::runtime_error("empty native trajectory");

  std::ifstream csv_file(argv[2]);
  if (!std::getline(csv_file, line)) throw std::runtime_error("empty edge CSV");
  const auto header = Split(line, ',');
  std::map<std::string, size_t> columns;
  for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  std::vector<Edge> edges;
  bool removed = false;
  while (std::getline(csv_file, line)) {
    const auto row = Split(line, ',');
    if (row.size() != header.size()) throw std::runtime_error("invalid edge CSV row");
    const auto get = [&](const std::string& name) { return row.at(columns.at(name)); };
    const size_t from = std::stoul(get("from_index"));
    const size_t to = std::stoul(get("to_index"));
    if (from >= native.size() || to >= native.size()) throw std::runtime_error("edge index out of range");
    if (remove && from == remove_from && to == remove_to) {
      removed = true;
      continue;
    }
    gtsam::Vector6 tangent;
    for (int k = 0; k < 6; ++k) tangent(k) = std::stod(get("mu_" + std::to_string(k)));
    edges.push_back({from, to, Pose3::Expmap(tangent)});
  }
  if (remove && !removed) throw std::runtime_error("requested removal edge absent");

  std::vector<std::vector<std::pair<size_t, Pose3>>> adjacency(native.size());
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<Pose3>(0, native[0],
      gtsam::noiseModel::Constrained::All(6)));
  for (const auto& edge : edges) {
    adjacency[edge.from].push_back({edge.to, edge.transform});
    adjacency[edge.to].push_back({edge.from, edge.transform.inverse()});
    // Equal unit noise isolates edge topology and mean consistency. This is
    // not a source-covariance or receiver-weighting experiment.
    graph.add(gtsam::BetweenFactor<Pose3>(edge.from, edge.to, edge.transform,
                                         gtsam::noiseModel::Unit::Create(6)));
  }
  gtsam::Values initial;
  initial.insert(0, native[0]);
  std::queue<size_t> frontier;
  frontier.push(0);
  while (!frontier.empty()) {
    const size_t from = frontier.front();
    frontier.pop();
    for (const auto& [to, transform] : adjacency[from]) {
      if (initial.exists(to)) continue;
      initial.insert(to, initial.at<Pose3>(from).compose(transform));
      frontier.push(to);
    }
  }
  if (initial.size() != native.size()) {
    std::cerr << "disconnected nodes=" << native.size()-initial.size() << '\n';
    return 3;
  }
  gtsam::GaussNewtonParams params;
  params.maxIterations = 20;
  params.relativeErrorTol = 1e-10;
  const auto result = gtsam::GaussNewtonOptimizer(graph, initial, params).optimize();
  std::ofstream output(argv[3]);
  output << std::setprecision(17);
  for (size_t i = 0; i < native.size(); ++i) {
    const auto transform = result.at<Pose3>(i);
    const auto p = transform.translation();
    const Eigen::Quaterniond q(transform.rotation().matrix());
    output << timestamps[i] << ' ' << p.x() << ' ' << p.y() << ' ' << p.z()
           << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "nodes=" << native.size() << " edges=" << edges.size()
            << " initial_cost=" << graph.error(initial)
            << " final_cost=" << graph.error(result) << '\n';
}
