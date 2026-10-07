// Isolated CBS-OFF GLIM pose-only graph audit; no IMU, GP, Kimera, or GNSS.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

std::vector<std::string> split(const std::string & line) {
  std::vector<std::string> result;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    result.push_back(field);
  }
  return result;
}

int main(int argc, char ** argv) {
  if (argc != 4) {
    std::cerr << "usage: spx3_glim_pose_graph_audit BELIEFS.csv NATIVE_SUPPORT.tum OUTPUT.tum\n";
    return 2;
  }
  std::ifstream native(argv[2]);
  if (!native) throw std::runtime_error("cannot read native support TUM");
  gtsam::Values initial;
  std::vector<double> stamps;
  for (size_t index = 0;; ++index) {
    double stamp, x, y, z, qx, qy, qz, qw;
    if (!(native >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) break;
    stamps.push_back(stamp);
    initial.insert(gtsam::Symbol('x', index),
      gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz), gtsam::Point3(x,y,z)));
  }
  if (stamps.size() < 3) throw std::runtime_error("too few native support poses");

  gtsam::NonlinearFactorGraph graph;
  const auto tight = gtsam::noiseModel::Isotropic::Sigma(6, 1e-8);
  // i->i+2 edges form TWO disconnected trees; each needs one gauge anchor.
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', 0), initial.at<gtsam::Pose3>(gtsam::Symbol('x', 0)), tight);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', 1), initial.at<gtsam::Pose3>(gtsam::Symbol('x', 1)), tight);

  std::ifstream beliefs(argv[1]);
  std::string line;
  if (!std::getline(beliefs, line)) throw std::runtime_error("empty belief CSV");
  const auto header = split(line);
  std::unordered_map<std::string, size_t> columns;
  for (size_t i=0; i<header.size(); ++i) columns.emplace(header[i], i);
  size_t edges=0;
  while (std::getline(beliefs, line)) {
    if (line.empty()) continue;
    const auto fields=split(line);
    auto get=[&](const std::string & key) { return fields.at(columns.at(key)); };
    const size_t from=std::stoull(get("from_index"));
    const size_t to=std::stoull(get("to_index"));
    if (to != from+2 || to >= stamps.size()) throw std::runtime_error("not the expected i->i+2 forest");
    gtsam::Vector6 mu;
    gtsam::Matrix6 covariance;
    for (size_t i=0; i<6; ++i) {
      mu(i)=std::stod(get("relative_mu_"+std::to_string(i)));
      for (size_t j=0; j<6; ++j) {
        covariance(i,j)=std::stod(get("covariance_"+std::to_string(i)+"_"+std::to_string(j)));
      }
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', from), gtsam::Symbol('x', to),
      gtsam::Pose3::Expmap(mu), gtsam::noiseModel::Gaussian::Covariance(covariance));
    ++edges;
  }
  if (edges+2 != stamps.size()) throw std::runtime_error("edge/support count mismatch");
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  const auto optimized=gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  std::ofstream output(argv[3]);
  if (!output) throw std::runtime_error("cannot open output TUM");
  output << std::setprecision(17);
  for (size_t index=0; index<stamps.size(); ++index) {
    const auto pose=optimized.at<gtsam::Pose3>(gtsam::Symbol('x', index));
    const auto q=pose.rotation().toQuaternion();
    output << stamps[index] << ' ' << pose.x() << ' ' << pose.y() << ' '
      << pose.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::cout << "poses=" << stamps.size() << " edges=" << edges
    << " native_initial_cost=" << graph.error(initial)
    << " optimized_cost=" << graph.error(optimized) << '\n';
}
