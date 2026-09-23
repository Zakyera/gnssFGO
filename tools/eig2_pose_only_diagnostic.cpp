// Isolated pose-only sanity check. Never linked into the production replay.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using gtsam::Pose3;

std::vector<std::string> Split(const std::string & line, char delim) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, delim)) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(field);
  }
  return fields;
}

struct Edge {
  size_t from, to;
  Pose3 exported;
  gtsam::Matrix6 covariance;
};

int main(int argc, char ** argv) {
  if (argc != 5) {
    std::cerr << "usage: eig2_pose_only_diagnostic NATIVE.tum BELIEFS.csv native|exported OUTPUT.tum\n";
    return 2;
  }
  const std::string mode = argv[3];
  if (mode != "native" && mode != "exported") return 2;
  std::ifstream tum(argv[1]);
  std::vector<std::string> timestamps;
  std::vector<Pose3> native;
  std::string line;
  while (std::getline(tum, line)) {
    auto f = Split(line, ' ');
    if (f.size() != 8) throw std::runtime_error("invalid TUM row");
    timestamps.push_back(f[0]);
    const Eigen::Quaterniond q(std::stod(f[7]), std::stod(f[4]), std::stod(f[5]), std::stod(f[6]));
    native.emplace_back(gtsam::Rot3(q), gtsam::Point3(std::stod(f[1]), std::stod(f[2]), std::stod(f[3])));
  }
  std::ifstream csv(argv[2]);
  std::getline(csv, line);
  const auto header = Split(line, ',');
  std::map<std::string, size_t> columns;
  for (size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  std::vector<Edge> edges;
  while (std::getline(csv, line)) {
    const auto f = Split(line, ',');
    if (f.size() != header.size()) throw std::runtime_error("invalid belief CSV row");
    auto get = [&](const std::string & name) { return f.at(columns.at(name)); };
    gtsam::Vector6 mu;
    for (int i = 0; i < 6; ++i) mu(i) = std::stod(get("relative_mu_" + std::to_string(i)));
    gtsam::Matrix6 covariance;
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j)
      covariance(i, j) = std::stod(get("covariance_" + std::to_string(i) + "_" + std::to_string(j)));
    edges.push_back({static_cast<size_t>(std::stoul(get("from_index"))), static_cast<size_t>(std::stoul(get("to_index"))), Pose3::Expmap(mu), covariance});
  }
  if (native.size() != 3042 || edges.size() != 3040) throw std::runtime_error("unexpected EIG-2 source size");

  gtsam::NonlinearFactorGraph graph;
  // Exactly one constrained gauge per disconnected component; neither bridge
  // between components nor an arbitrary finite-strength trajectory prior.
  graph.add(gtsam::PriorFactor<Pose3>(0, native[0], gtsam::noiseModel::Constrained::All(6)));
  graph.add(gtsam::PriorFactor<Pose3>(1, native[1], gtsam::noiseModel::Constrained::All(6)));
  gtsam::Values initial;
  initial.insert(0, native[0]);
  initial.insert(1, native[1]);
  for (const auto & edge : edges) {
    if (!initial.exists(edge.from) || initial.exists(edge.to)) throw std::runtime_error("not a two-chain forest");
    const Pose3 measurement = mode == "native" ? native[edge.from].between(native[edge.to]) : edge.exported;
    // The native-derived case tests geometry alone with unit noise. Its factor
    // covariances are not native-derived, so importing ultra-small exported
    // covariances here would only make this convention test ill-conditioned.
    const auto noise = mode == "native" ? gtsam::noiseModel::Unit::Create(6)
                                         : gtsam::noiseModel::Gaussian::Covariance(edge.covariance);
    graph.add(gtsam::BetweenFactor<Pose3>(edge.from, edge.to, measurement, noise));
    initial.insert(edge.to, mode == "native" ? native[edge.to]
                                            : initial.at<Pose3>(edge.from).compose(measurement));
  }
  gtsam::GaussNewtonParams params;
  params.maxIterations = 5;
  const auto result = gtsam::GaussNewtonOptimizer(graph, initial, params).optimize();
  double maximum_initial_to_final_translation = 0, maximum_initial_to_final_rotation = 0;
  for (size_t i = 0; i < native.size(); ++i) {
    const auto delta = Pose3::Logmap(initial.at<Pose3>(i).between(result.at<Pose3>(i)));
    maximum_initial_to_final_translation = std::max(maximum_initial_to_final_translation, delta.tail<3>().norm());
    maximum_initial_to_final_rotation = std::max(maximum_initial_to_final_rotation, delta.head<3>().norm());
  }
  std::ofstream output(argv[4]);
  output << std::setprecision(17);
  for (size_t i = 0; i < native.size(); ++i) {
    const auto p = result.at<Pose3>(i).translation();
    const Eigen::Quaterniond q(result.at<Pose3>(i).rotation().matrix());
    output << timestamps[i] << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "mode=" << mode << " variables=" << result.size() << " edges=" << edges.size()
            << " gauges=2 initial_error=" << graph.error(initial) << " final_error=" << graph.error(result)
            << " max_initial_to_final_translation=" << maximum_initial_to_final_translation
            << " max_initial_to_final_rotation_rad=" << maximum_initial_to_final_rotation << '\n';
}
