#include "RealGraphAudit.h"
#include <glim_ros/cbs_bridge.hpp>
#include <glim_ros/cbs_local_graph_linearizer.hpp>
#include <glim_ros/dcreg_edge_metadata.hpp>

#include <aria_viz/visualizer_rerun.h>
#include <cbs/utils/relative_pose_covariance.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JointMarginal.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/timing.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cxxabi.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <typeinfo>
#include <unordered_set>
#include <utility>

namespace glim {
namespace {

using gtsam::symbol_shorthand::X;

constexpr double kMinQuaternionNorm = 1e-9;
constexpr size_t kNsecPerSec = 1000000000ull;

uint64_t stampToNSec(const ros::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * kNsecPerSec +
         static_cast<uint64_t>(stamp.nsec);
}

uint64_t stampSecToNSec(const double stamp_sec) {
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return 0ull;
  }
  return static_cast<uint64_t>(std::llround(stamp_sec * 1.0e9));
}

std::string getDockerGatewayIp() {
  std::ifstream route_file("/proc/net/route");
  std::string line;
  std::getline(route_file, line);
  while (std::getline(route_file, line)) {
    std::istringstream iss(line);
    std::string iface;
    std::string destination;
    std::string gateway;
    unsigned int flags = 0u;
    if (!(iss >> iface >> destination >> gateway >> std::hex >> flags)) {
      continue;
    }
    if (destination != "00000000" || gateway.size() != 8u) {
      continue;
    }

    const unsigned long raw_gateway = std::stoul(gateway, nullptr, 16);
    std::ostringstream ip;
    ip << (raw_gateway & 0xfful) << "." << ((raw_gateway >> 8u) & 0xfful)
       << "." << ((raw_gateway >> 16u) & 0xfful) << "."
       << ((raw_gateway >> 24u) & 0xfful);
    return ip.str();
  }
  return "";
}

std::string defaultRerunHost() {
  const char* env_host = std::getenv("CBSMS_RERUN_HOST");
  if (env_host != nullptr && std::string(env_host).empty() == false) {
    return env_host;
  }

  const std::string gateway_ip = getDockerGatewayIp();
  if (!gateway_ip.empty()) {
    return "rerun+http://" + gateway_ip + ":9876/proxy";
  }
  return "rerun+http://host.docker.internal:9876/proxy";
}

std::string makeRerunRecordingId(const std::string& prefix) {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return prefix + "_" + buffer;
}

void sanitizeHealthParams(cbs::health_aware::Params* params) {
  if (!params) {
    return;
  }
  if (params->alpha_max < params->alpha_min) {
    std::swap(params->alpha_min, params->alpha_max);
  }
  params->warmup_samples = std::max(1, params->warmup_samples);
  params->relative_trust_warmup_samples =
      std::max(1, params->relative_trust_warmup_samples);
  params->relative_trust_min_weight =
      std::max(0.0, params->relative_trust_min_weight);
  params->relative_trust_max_weight =
      std::max(params->relative_trust_min_weight,
               params->relative_trust_max_weight);
  params->relative_trust_deadband =
      std::max(0.0, params->relative_trust_deadband);
}

void readHealthParamGroup(ros::NodeHandle& private_nh,
                          const std::string& prefix,
                          cbs::health_aware::Params* params) {
  if (!params) {
    return;
  }
  private_nh.param<double>(prefix + "_alpha_min",
                           params->alpha_min,
                           params->alpha_min);
  private_nh.param<double>(prefix + "_alpha_max",
                           params->alpha_max,
                           params->alpha_max);
  private_nh.param<double>(prefix + "_alpha_cons_max",
                           params->alpha_cons_max,
                           params->alpha_cons_max);
  private_nh.param<double>(prefix + "_chi2_threshold",
                           params->chi2_threshold,
                           params->chi2_threshold);
  private_nh.param<double>(prefix + "_beta_receiver_cov",
                           params->beta_receiver_cov,
                           params->beta_receiver_cov);
  private_nh.param<bool>(prefix + "_relative_trust_enable",
                         params->relative_trust_enable,
                         params->relative_trust_enable);
  private_nh.param<bool>(prefix + "_relative_trust_calibrated_enable",
                         params->relative_trust_calibrated_enable,
                         params->relative_trust_calibrated_enable);
  private_nh.param<double>(prefix + "_relative_trust_kappa",
                           params->relative_trust_kappa,
                           params->relative_trust_kappa);
  private_nh.param<double>(prefix + "_relative_trust_min_weight",
                           params->relative_trust_min_weight,
                           params->relative_trust_min_weight);
  private_nh.param<double>(prefix + "_relative_trust_max_weight",
                           params->relative_trust_max_weight,
                           params->relative_trust_max_weight);
  private_nh.param<double>(prefix + "_relative_trust_deadband",
                           params->relative_trust_deadband,
                           params->relative_trust_deadband);
  private_nh.param<int>(prefix + "_relative_trust_warmup_samples",
                        params->relative_trust_warmup_samples,
                        params->relative_trust_warmup_samples);
  private_nh.param<double>(prefix + "_w_logdet",
                           params->w_logdet,
                           params->w_logdet);
  private_nh.param<double>(prefix + "_w_lambda",
                           params->w_lambda,
                           params->w_lambda);
  private_nh.param<double>(prefix + "_w_growth",
                           params->w_growth,
                           params->w_growth);
  private_nh.param<double>(prefix + "_kappa",
                           params->kappa,
                           params->kappa);
  private_nh.param<int>(prefix + "_warmup_samples",
                        params->warmup_samples,
                        params->warmup_samples);
  private_nh.param<double>(prefix + "_rot_sigma0_rad",
                           params->rot_sigma0_rad,
                           params->rot_sigma0_rad);
  private_nh.param<double>(prefix + "_trans_sigma0_m",
                           params->trans_sigma0_m,
                           params->trans_sigma0_m);
  private_nh.param<double>(prefix + "_floor_rot_sigma_rad",
                           params->floor_rot_sigma_rad,
                           params->floor_rot_sigma_rad);
  private_nh.param<double>(prefix + "_floor_trans_sigma_m",
                           params->floor_trans_sigma_m,
                           params->floor_trans_sigma_m);
  private_nh.param<double>(prefix + "_eps", params->eps, params->eps);
  sanitizeHealthParams(params);
}

bool finitePose(const gtsam::Pose3& pose) {
  return pose.matrix().allFinite();
}

const char* modeToken(GlimCbsBridge::Mode mode) {
  switch (mode) {
    case GlimCbsBridge::Mode::ObserveOnly:
      return "observe_only";
    case GlimCbsBridge::Mode::InjectPersistent:
      return "inject_persistent";
    case GlimCbsBridge::Mode::TemporaryLinear:
      return "temporary_linear";
    case GlimCbsBridge::Mode::ActiveWindowTemporary:
      return "active_window_temporary";
  }
  return "unknown";
}

const char* outgoingMarginalSourceToken(
    GlimCbsBridge::OutgoingMarginalSource source) {
  switch (source) {
    case GlimCbsBridge::OutgoingMarginalSource::Direct:
      return "direct";
    case GlimCbsBridge::OutgoingMarginalSource::SidecarBatch:
      return "sidecar_batch";
  }
  return "unknown";
}

cbs::BPSAM::CbsOdomFactorMode bpsamModeForGlimMode(
    GlimCbsBridge::Mode mode) {
  switch (mode) {
    case GlimCbsBridge::Mode::InjectPersistent:
      return cbs::BPSAM::CbsOdomFactorMode::Persistent;
    case GlimCbsBridge::Mode::TemporaryLinear:
      return cbs::BPSAM::CbsOdomFactorMode::TemporaryLinear;
    case GlimCbsBridge::Mode::ActiveWindowTemporary:
      return cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
    case GlimCbsBridge::Mode::ObserveOnly:
      return cbs::BPSAM::CbsOdomFactorMode::Legacy;
  }
  return cbs::BPSAM::CbsOdomFactorMode::Legacy;
}

bool factorMatchesPoseEdge(const gtsam::NonlinearFactor::shared_ptr& factor,
                           gtsam::Key from_key,
                           gtsam::Key to_key) {
  return factor && factor->keys().size() == 2 &&
         factor->keys().front() == from_key &&
         factor->keys().back() == to_key;
}

std::string agentToken(cbs::AgentId agent_id) {
  const char agent_char = static_cast<char>(agent_id);
  if (std::isprint(static_cast<unsigned char>(agent_char))) {
    return std::string(1, agent_char);
  }
  return std::to_string(static_cast<int>(agent_id));
}

std::string directionToken(cbs::AgentId sender_agent,
                           cbs::AgentId receiver_agent) {
  std::string sender = agentToken(sender_agent);
  std::string receiver = agentToken(receiver_agent);
  std::transform(sender.begin(), sender.end(), sender.begin(), ::toupper);
  std::transform(receiver.begin(), receiver.end(), receiver.begin(), ::toupper);
  return sender + "2" + receiver;
}

std::string poseKeyToken(gtsam::Key key) {
  try {
    const gtsam::Symbol symbol(key);
    if (std::isprint(static_cast<unsigned char>(symbol.chr()))) {
      return std::string(1, symbol.chr()) + std::to_string(symbol.index());
    }
  } catch (...) {
  }
  return std::to_string(key);
}

std::string csvSafeToken(std::string token) {
  for (char& ch : token) {
    if (ch == ',') {
      ch = ';';
    } else if (ch == '\n' || ch == '\r') {
      ch = ' ';
    }
  }
  return token;
}

std::string demangledTypeName(const std::type_info& type_info) {
  int status = 0;
  char* demangled =
      abi::__cxa_demangle(type_info.name(), nullptr, nullptr, &status);
  std::string name =
      status == 0 && demangled != nullptr ? demangled : type_info.name();
  std::free(demangled);
  return csvSafeToken(name);
}

std::string factorKeysToken(const gtsam::NonlinearFactor& factor) {
  std::ostringstream stream;
  bool first = true;
  for (const gtsam::Key key : factor.keys()) {
    if (!first) {
      stream << "|";
    }
    stream << poseKeyToken(key);
    first = false;
  }
  return stream.str();
}

void logHealthAwareSenderRow(
    cbs::AgentId sender_agent,
    cbs::AgentId receiver_agent,
    gtsam::Key from_pose_key,
    gtsam::Key to_pose_key,
    const cbs::health_aware::SenderHealthResult& result) {
  ROS_INFO_STREAM("CBS_HEALTH_AWARE_SENDER_ROW,"
                  << directionToken(sender_agent, receiver_agent) << ","
                  << agentToken(sender_agent) << ","
                  << agentToken(receiver_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << ","
                  << (result.enabled ? 1 : 0) << ","
                  << result.status << ","
                  << result.raw_rel_trace << ","
                  << result.abs_trace << ","
                  << result.u << ","
                  << result.u0 << ","
                  << result.g_det << ","
                  << result.g_tr << ","
                  << result.alpha_health << ","
                  << result.floor_covariance.trace() << ","
                  << result.final_trace);
}

void logHealthAwareNisRow(
    cbs::AgentId receiver_agent,
    cbs::AgentId source_agent,
    gtsam::Key from_pose_key,
    gtsam::Key to_pose_key,
    const cbs::health_aware::NisResult& result) {
  ROS_INFO_STREAM("CBS_HEALTH_AWARE_NIS_ROW,"
                  << directionToken(source_agent, receiver_agent) << ","
                  << agentToken(receiver_agent) << ","
                  << agentToken(source_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << ","
                  << (result.enabled ? 1 : 0) << ","
                  << result.status << ","
                  << result.residual_norm << ","
                  << result.rot_norm << ","
                  << result.trans_norm << ","
                  << result.sender_trace << ","
                  << result.receiver_trace << ","
                  << result.s_trace << ","
                  << result.nu << ","
                  << result.chi2 << ","
                  << result.alpha_cons << ","
                  << result.final_trace << ","
                  << result.raw_alpha_cons << ","
                  << (result.relative_trust_enabled ? 1 : 0) << ","
                  << result.relative_trust_status << ","
                  << result.sender_uncertainty << ","
                  << result.receiver_uncertainty << ","
                  << result.trust_balance << ","
                  << result.trust_weight << ","
                  << (result.relative_trust_calibrated ? 1 : 0) << ","
                  << result.sender_uncertainty_baseline << ","
                  << result.receiver_uncertainty_baseline << ","
                  << result.sender_uncertainty_growth << ","
                  << result.receiver_uncertainty_growth);
}

void logReceiverCovarianceRow(cbs::AgentId receiver_agent,
                              cbs::AgentId source_agent,
                              gtsam::Key from_pose_key,
                              gtsam::Key to_pose_key,
                              bool from_in_new_values,
                              bool to_in_new_values,
                              bool from_in_linearization,
                              bool to_in_linearization,
                              bool joint_info_attempted,
                              bool joint_info_ok,
                              bool conditional_covariance_ok,
                              const std::string& status) {
  ROS_INFO_STREAM("GLIM_CBS_RECEIVER_COVARIANCE_ROW,"
                  << directionToken(source_agent, receiver_agent) << ","
                  << agentToken(receiver_agent) << ","
                  << agentToken(source_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << ","
                  << (from_in_new_values ? 1 : 0) << ","
                  << (to_in_new_values ? 1 : 0) << ","
                  << (from_in_linearization ? 1 : 0) << ","
                  << (to_in_linearization ? 1 : 0) << ","
                  << (joint_info_attempted ? 1 : 0) << ","
                  << (joint_info_ok ? 1 : 0) << ","
                  << (conditional_covariance_ok ? 1 : 0) << ","
                  << status);
}

double covarianceNis(const gtsam::Matrix6& covariance,
                     const gtsam::Vector6& residual) {
  if (!covariance.allFinite() || !residual.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const gtsam::Matrix6 symmetric_covariance =
      0.5 * (covariance + covariance.transpose());
  Eigen::LDLT<gtsam::Matrix6> ldlt(symmetric_covariance);
  if (ldlt.info() != Eigen::Success) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const gtsam::Vector6 solved = ldlt.solve(residual);
  if (ldlt.info() != Eigen::Success || !solved.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return residual.dot(solved);
}

bool valuesContainFactorKeys(const gtsam::Values& values,
                             const gtsam::NonlinearFactor& factor) {
  for (const gtsam::Key key : factor.keys()) {
    if (!values.exists(key)) {
      return false;
    }
  }
  return true;
}

bool factorTouchesKey(const gtsam::NonlinearFactor& factor, gtsam::Key key) {
  const auto keys = factor.keys();
  return std::find(keys.begin(), keys.end(), key) != keys.end();
}

double factorErrorOrNan(const gtsam::NonlinearFactor& factor,
                        const gtsam::Values& values) {
  if (!valuesContainFactorKeys(values, factor)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    const double error = factor.error(values);
    return std::isfinite(error) ? error
                                : std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception&) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double factorPoseHessianFrobeniusOrNan(const gtsam::NonlinearFactor& factor,
                                       const gtsam::Values& values,
                                       gtsam::Key key_a,
                                       gtsam::Key key_b) {
  if (!valuesContainFactorKeys(values, factor)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    const gtsam::GaussianFactor::shared_ptr linearized =
        factor.linearize(values);
    if (!linearized) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto blocks = linearized->hessianBlockDiagonal();
    double squared_norm_sum = 0.0;
    bool found = false;
    const auto add_block = [&](gtsam::Key key) {
      const auto it = blocks.find(key);
      if (it != blocks.end() && it->second.allFinite()) {
        squared_norm_sum += it->second.squaredNorm();
        found = true;
      }
    };
    add_block(key_a);
    if (key_b != key_a) {
      add_block(key_b);
    }
    return found ? std::sqrt(squared_norm_sum)
                 : std::numeric_limits<double>::quiet_NaN();
  } catch (const std::exception&) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

struct FactorInfluenceSummary {
  size_t count = 0u;
  size_t error_count = 0u;
  double error_sum = 0.0;
  size_t hessian_count = 0u;
  double hessian_frobenius_sum = 0.0;

  void add(double error, double hessian_frobenius) {
    ++count;
    if (std::isfinite(error)) {
      ++error_count;
      error_sum += error;
    }
    if (std::isfinite(hessian_frobenius)) {
      ++hessian_count;
      hessian_frobenius_sum += hessian_frobenius;
    }
  }
};

std::string receiverLocalFactorCategory(
    const gtsam::NonlinearFactor& factor) {
  const std::string type_name = demangledTypeName(typeid(factor));
  if (type_name.find("LinearContainerFactor") != std::string::npos) {
    return "marginal_prior";
  }
  if (type_name.find("IntegratedVGICP") != std::string::npos ||
      type_name.find("GICP") != std::string::npos) {
    return "lidar";
  }
  if (type_name.find("ImuFactor") != std::string::npos) {
    return "imu";
  }
  if (type_name.find("DampingFactor") != std::string::npos) {
    return "damping";
  }
  if (type_name.find("PriorFactor") != std::string::npos) {
    return "prior";
  }
  if (type_name.find("BetweenFactor") != std::string::npos) {
    bool has_pose = false;
    bool has_bias = false;
    for (const gtsam::Key key : factor.keys()) {
      const char chr = static_cast<char>(gtsam::Symbol(key).chr());
      has_pose = has_pose || chr == 'x';
      has_bias = has_bias || chr == 'b';
    }
    if (has_bias) {
      return "bias_between";
    }
    if (has_pose) {
      return "pose_between_local";
    }
    return "between_other";
  }
  return "other";
}

void logOdomPreinjectionResidualRow(cbs::AgentId receiver_agent,
                                    cbs::AgentId source_agent,
                                    gtsam::Key from_pose_key,
                                    gtsam::Key to_pose_key,
                                    const gtsam::Pose3& measured_from_to,
                                    const gtsam::Matrix6& covariance,
                                    const gtsam::Pose3& receiver_from_pose,
                                    const gtsam::Pose3& receiver_to_pose,
                                    const std::string& receiver_pose_source,
                                    const std::string& action) {
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  const gtsam::Pose3 receiver_from_to =
      receiver_from_pose.between(receiver_to_pose);
  const gtsam::Vector6 residual = gtsam::traits<gtsam::Pose3>::Logmap(
      measured_from_to.inverse().compose(receiver_from_to));
  const double yaw_error =
      std::remainder(receiver_from_to.rotation().yaw() -
                         measured_from_to.rotation().yaw(),
                     2.0 * 3.14159265358979323846);
  const double nis = covarianceNis(covariance, residual);
  const double whitened_norm =
      std::isfinite(nis) && nis >= 0.0
          ? std::sqrt(nis)
          : std::numeric_limits<double>::quiet_NaN();

  ROS_INFO_STREAM("CBS_ODOM_PREINJECTION_RESIDUAL_ROW,"
                  << directionToken(source_agent, receiver_agent) << ","
                  << agentToken(receiver_agent) << ","
                  << agentToken(source_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << ","
                  << action << "," << receiver_pose_source << ","
                  << residual.norm() << "," << residual.head(3).norm() << ","
                  << residual.tail(3).norm() << "," << yaw_error << ","
                  << yaw_error * kRadToDeg << "," << covariance.trace() << ","
                  << whitened_norm << "," << nis);
}

void logOdomFactorCovarianceRow(cbs::AgentId receiver_agent,
                                cbs::AgentId source_agent,
                                gtsam::Key from_pose_key,
                                gtsam::Key to_pose_key,
                                const gtsam::Matrix6& covariance,
                                const std::string& action) {
  const auto diag = covariance.diagonal();
  const double rot_trace = diag.segment(0, 3).sum();
  const double trans_trace = diag.segment(3, 3).sum();
  const double asym_frobenius = (covariance - covariance.transpose()).norm();
  const double cov_frobenius = covariance.norm();
  const double asym_relative =
      cov_frobenius > 0.0 ? asym_frobenius / cov_frobenius : asym_frobenius;

  double min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
  double condition = std::numeric_limits<double>::quiet_NaN();
  if (covariance.allFinite()) {
    const gtsam::Matrix6 symmetric_covariance =
        0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> solver(
        symmetric_covariance, Eigen::EigenvaluesOnly);
    if (solver.info() == Eigen::Success && solver.eigenvalues().allFinite()) {
      min_eigenvalue = solver.eigenvalues().minCoeff();
      max_eigenvalue = solver.eigenvalues().maxCoeff();
      const double min_abs_eigenvalue =
          solver.eigenvalues().cwiseAbs().minCoeff();
      if (min_abs_eigenvalue > 0.0) {
        condition =
            solver.eigenvalues().cwiseAbs().maxCoeff() / min_abs_eigenvalue;
      }
    }
  }

  ROS_INFO_STREAM("CBS_ODOM_FACTOR_COVARIANCE_ROW,"
                  << directionToken(source_agent, receiver_agent) << ","
                  << agentToken(receiver_agent) << ","
                  << agentToken(source_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << "," << action << ","
                  << covariance.trace() << "," << rot_trace << ","
                  << trans_trace << "," << diag(0) << "," << diag(1) << ","
                  << diag(2) << "," << diag(3) << "," << diag(4) << ","
                  << diag(5) << "," << min_eigenvalue << ","
                  << max_eigenvalue << "," << condition << ","
                  << asym_frobenius << "," << asym_relative);
  ROS_INFO_STREAM("CBS_ODOM_FACTOR_MATRIX_ROW,"
                  << directionToken(source_agent, receiver_agent) << ","
                  << poseKeyToken(from_pose_key) << ","
                  << poseKeyToken(to_pose_key) << "," << action << ","
                  << cbs::relative_pose_covariance::matrixToken(covariance));
}

}  // namespace

class GlimCbsBridge::RerunImpl {
 public:
  RerunImpl(const std::string& recording_id, const std::string& host)
      : visualizer(aria::viz::VisualizerRerun::Params(
            "cbsms", recording_id, host)) {}

  aria::viz::VisualizerRerun visualizer;
};

GlimCbsBridge::GlimCbsBridge(ros::NodeHandle& private_nh) : nh_(private_nh) {
  configure(private_nh);
  if (!enabled_) {
    return;
  }

  gtsam::ISAM2Params isam2_params;
  isam2_params.findUnusedFactorSlots = true;
  isam2_params.relinearizeSkip = 1;
  isam2_params.setRelinearizeThreshold(0.1);

  cbs::BPSAM::Params bpsam_params;
  bpsam_params.robot_id = self_agent_id_;
  bpsam_params.sam_params_ = isam2_params;
  bpsam_params.reject_first_message = false;
  bpsam_params.cbs_odom_factor_mode = bpsamModeForGlimMode(mode_);
  bpsam_params.use_temporary_cbs_linear_factors =
      mode_ == Mode::TemporaryLinear;
  bpsam_params.outgoing_odom_covariance_mode =
      outgoing_odom_covariance_mode_;
  bpsam_params.relative_covariance_matrix_dump_enable =
      relative_covariance_matrix_dump_enable_;
  bpsam_params.relative_covariance_matrix_dump_limit =
      relative_covariance_matrix_dump_limit_;
  bpsam_params.relative_covariance_matrix_dump_trace_max =
      relative_covariance_matrix_dump_trace_max_;
  bpsam_params.temporary_linear_already_applied_gate_enable = true;
  bpsam_params.gbp_update_params.enable_soft_reset = true;
  bpsam_params.gbp_update_params.d_reset = 0.1;
  bpsam_params.health_aware_params = health_receiver_params_;
  bpsam_params.health_aware_receiver_params = health_receiver_params_;
  bpsam_params.has_health_aware_receiver_params = true;
  const bool sidecar_required =
      outgoing_marginal_source_ == OutgoingMarginalSource::SidecarBatch ||
      mode_ == Mode::ActiveWindowTemporary;
  if (sidecar_required) {
    sidecar_ = std::make_unique<cbs::IncrementalFixedLagBpsamSmoother>(
        smoother_lag_sec_, bpsam_params);
  }

  if (receive_external_beliefs_) {
    belief_sub_ = nh_.subscribe<liorf::pose_odom_belief_array>(
        belief_in_topic_,
        50,
        &GlimCbsBridge::beliefCallback,
        this,
        ros::TransportHints().tcpNoDelay());
  }
  if (publish_beliefs_) {
    belief_pub_ =
        nh_.advertise<liorf::pose_odom_belief_array>(belief_out_topic_, 50);
  }
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic_, 10);
  dcreg_edge_metadata_ = dcreg_edge_metadata::Publisher::create(
      nh_, static_cast<std::uint8_t>(self_agent_id_));
  if (receive_external_beliefs_ && receive_start_delay_sec_ > 0.0) {
    clock_sub_ = nh_.subscribe<rosgraph_msgs::Clock>(
        "/clock",
        10,
        &GlimCbsBridge::clockCallback,
        this,
        ros::TransportHints().tcpNoDelay());
  }

  initRerun();
  registerCallbacks();

  ROS_INFO_STREAM("GLIM CBS bridge enabled. mode="
                  << modeToken(mode_)
                  << ", outgoing_marginal_source="
                  << outgoingMarginalSourceToken(outgoing_marginal_source_)
                  << ", self_agent=" << static_cast<char>(self_agent_id_)
                  << ", peer_agent=" << static_cast<char>(peer_agent_id_)
                  << ", publish_beliefs="
                  << (publish_beliefs_ ? "true" : "false")
                  << ", receive_external_beliefs="
                  << (receive_external_beliefs_ ? "true" : "false")
                  << ", sidecar=" << (sidecar_ ? "enabled" : "disabled")
                  << ", in='" << belief_in_topic_ << "', out='"
                  << belief_out_topic_ << "', odom='" << odometry_topic_
                  << "', sender_mode='" << sender_mode_
                  << "', outgoing_odom_covariance_mode='"
                  << outgoing_odom_covariance_mode_
                  << "', health_aware="
                  << (health_sender_params_.enable ? "enabled" : "disabled")
                  << ", health_sender="
                  << (health_sender_params_.sender_enable ? "on" : "off")
                  << ", health_receiver_nis="
                  << (health_receiver_params_.receiver_nis_enable ? "on" : "off")
                  << ".");
}

GlimCbsBridge::~GlimCbsBridge() {
  shutdown();
}

void GlimCbsBridge::shutdown() {
  if (dcreg_edge_metadata_) {
    dcreg_edge_metadata_->shutdown();
  }
  if (callbacks_registered_) {
    OdometryEstimationCallbacks::on_smoother_update.remove(
        smoother_update_callback_id_);
    OdometryEstimationCallbacks::on_smoother_update_finish.remove(
        smoother_update_finish_callback_id_);
    OdometryEstimationCallbacks::on_timing.remove(odometry_timing_callback_id_);
    callbacks_registered_ = false;
  }
  belief_sub_.shutdown();
  clock_sub_.shutdown();
}

void GlimCbsBridge::configure(ros::NodeHandle& private_nh) {
  private_nh.param<bool>("glim_cbs_bridge_enable", enabled_, false);
  private_nh.param<bool>("glim_cbs_experimental_local_gpu_batching",
                         experimental_local_gpu_batching_, false);
  private_nh.param<bool>("glim_cbs_experimental_native_imu_mean_alignment",
                         experimental_native_imu_mean_alignment_, false);
  if (enabled_ && experimental_native_imu_mean_alignment_)
    native_imu_mean_bridge_ = std::make_unique<CbsNativeImuMeanBridge>(
        IMUIntegrationParams());
  private_nh.param<bool>(
      "glim_cbs_publish_beliefs", publish_beliefs_, enabled_);
  private_nh.param<bool>("glim_cbs_receive_external_beliefs",
                         receive_external_beliefs_,
                         enabled_);
  std::string mode = "observe_only";
  private_nh.param<std::string>("glim_cbs_mode", mode, mode);
  if (mode == "inject" || mode == "persistent" ||
      mode == "inject_persistent") {
    mode_ = Mode::InjectPersistent;
  } else if (mode == "temporary_linear") {
    mode_ = Mode::TemporaryLinear;
  } else if (mode == "active_window_temporary" ||
             mode == "fixed_lag_temporary") {
    mode_ = Mode::ActiveWindowTemporary;
  } else {
    mode_ = Mode::ObserveOnly;
  }
  std::string outgoing_marginal_source = "direct";
  private_nh.param<std::string>("glim_cbs_outgoing_marginal_source",
                                outgoing_marginal_source,
                                outgoing_marginal_source);
  if (outgoing_marginal_source == "sidecar_batch" ||
      outgoing_marginal_source == "batch") {
    outgoing_marginal_source_ = OutgoingMarginalSource::SidecarBatch;
  } else {
    outgoing_marginal_source_ = OutgoingMarginalSource::Direct;
  }

  std::string self_agent = "g";
  std::string peer_agent = "k";
  private_nh.param<std::string>("glim_cbs_self_agent_id",
                                self_agent,
                                self_agent);
  private_nh.param<std::string>("glim_cbs_peer_agent_id",
                                peer_agent,
                                peer_agent);
  self_agent_id_ = parseAgentId(self_agent, self_agent_id_);
  peer_agent_id_ = parseAgentId(peer_agent, peer_agent_id_);

  private_nh.param<std::string>("glim_cbs_odom_belief_in_topic",
                                belief_in_topic_,
                                belief_in_topic_);
  private_nh.param<std::string>("glim_cbs_odom_belief_out_topic",
                                belief_out_topic_,
                                belief_out_topic_);
  private_nh.param<std::string>(
      "glim_cbs_odometry_topic", odometry_topic_, odometry_topic_);
  private_nh.param<std::string>("glim_cbs_frame_id", frame_id_, frame_id_);
  private_nh.param<std::string>(
      "glim_cbs_child_frame_id", child_frame_id_, child_frame_id_);
  private_nh.param<std::string>(
      "glim_cbs_odom_sender_mode", sender_mode_, sender_mode_);
  private_nh.param<std::string>("glim_cbs_odom_receiver_match_mode",
                                receiver_match_mode_,
                                receiver_match_mode_);
  private_nh.param<std::string>("glim_cbs_odom_covariance_mode",
                                outgoing_odom_covariance_mode_,
                                outgoing_odom_covariance_mode_);
  private_nh.param<bool>("glim_dcreg_belief_shadow_enable",
                         dcreg_belief_shadow_enable_,
                         dcreg_belief_shadow_enable_);
  private_nh.param<bool>("glim_cbs_export_endpoint_audit_enable",
                         export_endpoint_audit_enable_,
                         export_endpoint_audit_enable_);
  private_nh.param<bool>("glim_cbs_live_revision_enable",
                         live_revision_enable_, live_revision_enable_);
  private_nh.param<bool>("glim_cbs_relative_covariance_matrix_dump_enable",
                         relative_covariance_matrix_dump_enable_,
                         relative_covariance_matrix_dump_enable_);
  private_nh.param<int>("glim_cbs_relative_covariance_matrix_dump_limit",
                        relative_covariance_matrix_dump_limit_,
                        relative_covariance_matrix_dump_limit_);
  private_nh.param<double>("glim_cbs_relative_covariance_matrix_dump_trace_max",
                           relative_covariance_matrix_dump_trace_max_,
                           relative_covariance_matrix_dump_trace_max_);
  std::string relative_covariance_matrix_dump_edge_filter;
  private_nh.param<std::string>(
      "glim_cbs_relative_covariance_matrix_dump_edge_filter",
      relative_covariance_matrix_dump_edge_filter,
      relative_covariance_matrix_dump_edge_filter);
  if (!relative_covariance_matrix_dump_edge_filter.empty()) {
    relative_covariance_matrix_dump_every_occurrence_ =
        relative_covariance_matrix_dump_edge_filter == "*";
    if (relative_covariance_matrix_dump_every_occurrence_) {
      ROS_INFO_STREAM(
          "GLIM covariance audit will dump every rolling edge publication "
          "until the diagnostic limit is reached");
    }
    std::istringstream edge_stream(
        relative_covariance_matrix_dump_edge_filter);
    std::string edge_token;
    while (!relative_covariance_matrix_dump_every_occurrence_ &&
           std::getline(edge_stream, edge_token, ';')) {
      const size_t separator = edge_token.find(':');
      if (separator == std::string::npos) {
        ROS_WARN_STREAM(
            "Ignoring malformed covariance audit edge filter token: "
            << edge_token);
        continue;
      }
      try {
        size_t from_end = 0u;
        size_t to_end = 0u;
        const size_t from_index = std::stoull(
            edge_token.substr(0u, separator), &from_end);
        const std::string to_token = edge_token.substr(separator + 1u);
        const size_t to_index = std::stoull(to_token, &to_end);
        if (from_end != separator || to_end != to_token.size()) {
          throw std::invalid_argument("trailing characters");
        }
        relative_covariance_matrix_dump_edge_filter_.emplace(from_index,
                                                              to_index);
      } catch (const std::exception& e) {
        ROS_WARN_STREAM(
            "Ignoring malformed covariance audit edge filter token "
            << edge_token << ": " << e.what());
      }
    }
    if (!relative_covariance_matrix_dump_every_occurrence_) {
      ROS_INFO_STREAM(
          "GLIM covariance audit edge filter contains "
          << relative_covariance_matrix_dump_edge_filter_.size()
          << " stable edge(s); every rolling publication of those edges will "
             "be dumped until the diagnostic limit is reached");
    }
  }

  try {
    Config config(GlobalConfig::get_config_path("config_odometry"));
    smoother_lag_sec_ =
        config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  } catch (const std::exception&) {
    smoother_lag_sec_ = 5.0;
  }
  private_nh.param<double>(
      "glim_cbs_smoother_lag_sec", smoother_lag_sec_, smoother_lag_sec_);
  private_nh.param<double>("glim_cbs_belief_timestamp_tolerance_sec",
                           timestamp_tolerance_sec_,
                           timestamp_tolerance_sec_);
  private_nh.param<double>("glim_cbs_belief_receive_start_delay_sec",
                           receive_start_delay_sec_,
                           receive_start_delay_sec_);
  private_nh.param<double>("glim_cbs_odom_unmatched_retry_max_age_sec",
                           unmatched_retry_max_age_sec_,
                           unmatched_retry_max_age_sec_);
  int retry_max_beliefs = static_cast<int>(unmatched_retry_max_beliefs_);
  private_nh.param<int>("glim_cbs_odom_unmatched_retry_max_beliefs",
                        retry_max_beliefs,
                        retry_max_beliefs);
  unmatched_retry_max_beliefs_ =
      retry_max_beliefs > 0 ? static_cast<size_t>(retry_max_beliefs) : 0u;
  private_nh.param<bool>(
      "glim_cbs_wait_for_receiver_covariance_linearization_enable",
      wait_for_receiver_covariance_linearization_enable_,
      wait_for_receiver_covariance_linearization_enable_);
  private_nh.param<int>("glim_cbs_belief_exchange_window_size",
                        belief_exchange_window_size_,
                        belief_exchange_window_size_);
  private_nh.param<bool>("glim_cbs_odom_duration_gate_enable",
                         duration_gate_enable_,
                         duration_gate_enable_);
  private_nh.param<double>("glim_cbs_odom_duration_tolerance_sec",
                           duration_tolerance_sec_,
                           duration_tolerance_sec_);
  private_nh.param<double>("glim_cbs_odom_duration_ratio_min",
                           duration_ratio_min_,
                           duration_ratio_min_);
  private_nh.param<double>("glim_cbs_odom_duration_ratio_max",
                           duration_ratio_max_,
                           duration_ratio_max_);
  private_nh.param<double>(
      "glim_cbs_odom_horizon_sec", horizon_sec_, horizon_sec_);
  private_nh.param<double>("glim_cbs_odom_horizon_tolerance_sec",
                           horizon_tolerance_sec_,
                           horizon_tolerance_sec_);
  int max_horizon_pairs = static_cast<int>(max_horizon_pairs_per_update_);
  private_nh.param<int>("glim_cbs_odom_max_horizon_pairs_per_update",
                        max_horizon_pairs,
                        max_horizon_pairs);
  max_horizon_pairs_per_update_ =
      max_horizon_pairs > 0 ? static_cast<size_t>(max_horizon_pairs) : 0u;
  int outgoing_publish_stride = static_cast<int>(outgoing_publish_stride_);
  private_nh.param<int>("glim_cbs_odom_publish_stride",
                        outgoing_publish_stride,
                        outgoing_publish_stride);
  outgoing_publish_stride_ =
      outgoing_publish_stride > 0
          ? static_cast<size_t>(outgoing_publish_stride)
          : 1u;
  private_nh.param<double>("glim_cbs_k2g_odom_factor_covariance_scale",
                           covariance_scale_,
                           covariance_scale_);
  private_nh.param<double>(
      "glim_cbs_odom_relax_factor", relax_factor_, relax_factor_);
  bool active_factor_diagnostics_enable = false;
  int active_factor_diagnostics_stride = 20;
  int active_factor_diagnostics_max_updates = 40;
  private_nh.param<bool>("glim_cbs_active_factor_diagnostics_enable",
                         active_factor_diagnostics_enable, false);
  private_nh.param<int>("glim_cbs_active_factor_diagnostics_stride",
                        active_factor_diagnostics_stride, 20);
  private_nh.param<int>("glim_cbs_active_factor_diagnostics_max_updates",
                        active_factor_diagnostics_max_updates, 40);
  active_factor_diagnostics_budget_.configure(
      active_factor_diagnostics_enable, active_factor_diagnostics_stride,
      active_factor_diagnostics_max_updates);
  ROS_INFO_STREAM("GLIM CBS active-factor diagnostics: enabled="
                  << active_factor_diagnostics_enable << ", stride="
                  << std::max(1, active_factor_diagnostics_stride)
                  << ", max_updates="
                  << std::max(0, active_factor_diagnostics_max_updates));
  private_nh.param<bool>("glim_cbs_active_factor_detail_log_enable",
                         active_factor_detail_log_enable_,
                         active_factor_detail_log_enable_);
  int active_factor_detail_max_rows =
      static_cast<int>(active_factor_detail_max_rows_per_update_);
  private_nh.param<int>("glim_cbs_active_factor_detail_max_rows_per_update",
                        active_factor_detail_max_rows,
                        active_factor_detail_max_rows);
  active_factor_detail_max_rows_per_update_ =
      active_factor_detail_max_rows > 0
          ? static_cast<size_t>(active_factor_detail_max_rows)
          : 0u;
  int receiver_local_competition_audit_max_samples =
      static_cast<int>(receiver_local_competition_audit_max_samples_);
  private_nh.param<int>(
      "glim_cbs_receiver_local_competition_audit_max_samples",
      receiver_local_competition_audit_max_samples,
      receiver_local_competition_audit_max_samples);
  receiver_local_competition_audit_max_samples_ =
      receiver_local_competition_audit_max_samples > 0
          ? static_cast<size_t>(
                receiver_local_competition_audit_max_samples)
          : 0u;
  int receiver_local_competition_audit_stride =
      static_cast<int>(receiver_local_competition_audit_stride_);
  private_nh.param<int>(
      "glim_cbs_receiver_local_competition_audit_stride",
      receiver_local_competition_audit_stride,
      receiver_local_competition_audit_stride);
  receiver_local_competition_audit_stride_ =
      receiver_local_competition_audit_stride > 0
          ? static_cast<size_t>(receiver_local_competition_audit_stride)
          : 1u;
  cbs::health_aware::Params common_health_params;
  private_nh.param<bool>("glim_cbs_health_aware_enable",
                         common_health_params.enable,
                         common_health_params.enable);
  private_nh.param<bool>("glim_cbs_health_sender_enable",
                         common_health_params.sender_enable,
                         common_health_params.sender_enable);
  private_nh.param<bool>("glim_cbs_health_receiver_nis_enable",
                         common_health_params.receiver_nis_enable,
                         common_health_params.receiver_nis_enable);
  readHealthParamGroup(private_nh, "glim_cbs_health", &common_health_params);
  health_sender_params_ = common_health_params;
  health_receiver_params_ = common_health_params;
  readHealthParamGroup(
      private_nh, "glim_cbs_sender_health", &health_sender_params_);
  readHealthParamGroup(
      private_nh, "glim_cbs_receiver_health", &health_receiver_params_);
  int max_trajectory_len = static_cast<int>(max_trajectory_len_);
  private_nh.param<int>(
      "glim_cbs_rerun_max_trajectory_len", max_trajectory_len, max_trajectory_len);
  max_trajectory_len_ = std::max(2, max_trajectory_len);

  private_nh.param<bool>(
      "glim_cbs_rerun_visualizer_enable", rerun_enable_, false);
  private_nh.param<bool>(
      "glim_cbs_rerun_geometry_enable", rerun_geometry_enable_, true);
  private_nh.param<double>("glim_cbs_rerun_uncertainty_scale",
                           rerun_uncertainty_scale_,
                           rerun_uncertainty_scale_);
  if (!std::isfinite(rerun_uncertainty_scale_) ||
      rerun_uncertainty_scale_ <= 0.0) {
    rerun_uncertainty_scale_ = 1.25;
  }
  private_nh.param<std::string>(
      "glim_cbs_rerun_recording_id", rerun_recording_id_, "");
  private_nh.param<std::string>(
      "glim_cbs_rerun_host", rerun_host_, rerun_host_);

  if (!std::isfinite(smoother_lag_sec_) || smoother_lag_sec_ <= 0.0) {
    smoother_lag_sec_ = 5.0;
  }
  if (!std::isfinite(timestamp_tolerance_sec_) ||
      timestamp_tolerance_sec_ <= 0.0) {
    timestamp_tolerance_sec_ = 0.12;
  }
  if (!std::isfinite(receive_start_delay_sec_) ||
      receive_start_delay_sec_ < 0.0) {
    receive_start_delay_sec_ = 0.0;
  }
  if (!std::isfinite(unmatched_retry_max_age_sec_) ||
      unmatched_retry_max_age_sec_ < 0.0) {
    unmatched_retry_max_age_sec_ = 5.0;
  }
  if (!std::isfinite(covariance_scale_) || covariance_scale_ <= 0.0) {
    covariance_scale_ = 1.0;
  }
  if (belief_exchange_window_size_ < 2) {
    belief_exchange_window_size_ = 2;
  }
  if (!std::isfinite(duration_tolerance_sec_) ||
      duration_tolerance_sec_ < 0.0) {
    duration_tolerance_sec_ = 0.06;
  }
  if (!std::isfinite(duration_ratio_min_) || duration_ratio_min_ < 0.0) {
    duration_ratio_min_ = 0.0;
  }
  if (!std::isfinite(duration_ratio_max_) || duration_ratio_max_ < 0.0) {
    duration_ratio_max_ = 0.0;
  }
  if (duration_ratio_min_ > 0.0 && duration_ratio_max_ > 0.0 &&
      duration_ratio_min_ > duration_ratio_max_) {
    std::swap(duration_ratio_min_, duration_ratio_max_);
  }
  if (!std::isfinite(horizon_sec_) || horizon_sec_ <= 0.0) {
    horizon_sec_ = 0.20;
  }
  if (!std::isfinite(horizon_tolerance_sec_) || horizon_tolerance_sec_ < 0.0) {
    horizon_tolerance_sec_ = 0.06;
  }
  if (sender_mode_ != "adjacent_window" && sender_mode_ != "latest_edge" &&
      sender_mode_ != "new_edge_once" && sender_mode_ != "time_horizon_window") {
    ROS_WARN_STREAM("Invalid glim_cbs_odom_sender_mode='"
                    << sender_mode_
                    << "'; falling back to adjacent_window.");
    sender_mode_ = "adjacent_window";
  }
  if (receiver_match_mode_ != "nearest_stamp" &&
      receiver_match_mode_ != "duration_aware_edge") {
    ROS_WARN_STREAM("Invalid glim_cbs_odom_receiver_match_mode='"
                    << receiver_match_mode_
                    << "'; falling back to nearest_stamp.");
    receiver_match_mode_ = "nearest_stamp";
  }
  if (sender_mode_ == "time_horizon_window" &&
      outgoing_marginal_source_ == OutgoingMarginalSource::SidecarBatch) {
    ROS_WARN_STREAM("GLIM CBS time_horizon_window requires direct outgoing "
                    "marginals; falling back from sidecar_batch to direct.");
    outgoing_marginal_source_ = OutgoingMarginalSource::Direct;
  }
}

void GlimCbsBridge::registerCallbacks() {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  smoother_update_callback_id_ =
      OdometryEstimationCallbacks::on_smoother_update.add(
          std::bind(&GlimCbsBridge::onSmootherUpdate,
                    this,
                    _1,
                    _2,
                    _3,
                    _4,
                    _5));
  smoother_update_finish_callback_id_ =
      OdometryEstimationCallbacks::on_smoother_update_finish.add(
          std::bind(&GlimCbsBridge::onSmootherUpdateFinish, this, _1));
  odometry_timing_callback_id_ =
      OdometryEstimationCallbacks::on_timing.add(
          std::bind(&GlimCbsBridge::onOdometryTiming, this, _1));
  callbacks_registered_ = true;
}

void GlimCbsBridge::beliefCallback(
    const liorf::pose_odom_belief_arrayConstPtr& msg) {
  if (!msg || !enabled_) {
    return;
  }

  if (receive_start_delay_sec_ > 0.0 && !receive_gate_reference_set_) {
    const double stamp_sec = msg->header.stamp.toSec();
    if (std::isfinite(stamp_sec) && stamp_sec > 0.0) {
      receive_gate_reference_stamp_sec_ = stamp_sec;
      receive_gate_reference_set_ = true;
      ROS_INFO_STREAM(std::fixed << std::setprecision(9)
                      << "GLIM CBS receive gate reference stamp="
                      << receive_gate_reference_stamp_sec_
                      << ", accepting beliefs after "
                      << receive_gate_reference_stamp_sec_ +
                             receive_start_delay_sec_);
    }
  }

  std::deque<IncomingBelief> accepted;
  size_t dropped_by_gate = 0u;
  for (const auto& belief_msg : msg->beliefs) {
    IncomingBelief belief;
    belief.source_agent = static_cast<cbs::AgentId>(belief_msg.source_agent);
    if (belief.source_agent == self_agent_id_) {
      continue;
    }
    belief.from_pose_index = static_cast<size_t>(belief_msg.from_pose_index);
    belief.to_pose_index = static_cast<size_t>(belief_msg.to_pose_index);
    belief.revision = belief_msg.revision;
    belief.from_stamp_sec = belief_msg.from_stamp_sec;
    belief.to_stamp_sec = belief_msg.to_stamp_sec > 0.0
                               ? belief_msg.to_stamp_sec
                               : belief_msg.header.stamp.toSec();
    belief.relax_factor = belief_msg.relax_factor;
    for (size_t i = 0; i < belief.relative_mu.size(); ++i) {
      belief.relative_mu[i] = belief_msg.relative_mu[i];
    }
    for (size_t i = 0; i < belief.covariance.size(); ++i) {
      belief.covariance[i] = belief_msg.covariance[i];
    }

    const double gate_stamp = belief.to_stamp_sec > 0.0 ? belief.to_stamp_sec
                                                        : belief.from_stamp_sec;
    if (!finiteArray(belief.relative_mu) || !finiteArray(belief.covariance) ||
        (receive_start_delay_sec_ > 0.0 &&
         (!receive_gate_reference_set_ ||
          gate_stamp < receive_gate_reference_stamp_sec_ +
                           receive_start_delay_sec_))) {
      ++dropped_by_gate;
      continue;
    }
    accepted.push_back(belief);
  }

  if (!accepted.empty()) {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    for (const auto& belief : accepted) {
      if (live_revision_enable_) {
        const auto previous = std::find_if(
            incoming_.begin(), incoming_.end(),
            [&belief](const IncomingBelief& candidate) {
              return candidate.source_agent == belief.source_agent &&
                     candidate.from_pose_index == belief.from_pose_index &&
                     candidate.to_pose_index == belief.to_pose_index;
            });
        if (previous != incoming_.end()) {
          if (belief.revision > previous->revision) {
            *previous = belief;
          }
          continue;
        }
      }
      incoming_.push_back(belief);
    }
  }
  received_total_.fetch_add(accepted.size(), std::memory_order_relaxed);
  dropped_total_.fetch_add(dropped_by_gate, std::memory_order_relaxed);
}

void GlimCbsBridge::clockCallback(const rosgraph_msgs::ClockConstPtr& msg) {
  if (!msg || receive_start_delay_sec_ <= 0.0 ||
      receive_gate_reference_set_) {
    return;
  }
  const double stamp_sec = msg->clock.toSec();
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
    return;
  }
  receive_gate_reference_stamp_sec_ = stamp_sec;
  receive_gate_reference_set_ = true;
  clock_sub_.shutdown();
  ROS_INFO_STREAM(std::fixed << std::setprecision(9)
                  << "GLIM CBS receive gate reference stamp="
                  << receive_gate_reference_stamp_sec_
                  << " from /clock, accepting beliefs after "
                  << receive_gate_reference_stamp_sec_ +
                         receive_start_delay_sec_);
}

void GlimCbsBridge::onSmootherUpdate(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps,
    gtsam::FactorIndices& factors_to_remove) {
  if (!enabled_) {
    return;
  }

  pending_pose_merge_diagnostic_ = PoseMergeDiagnostic();

  const bool timing = timing_enabled();
  const auto total_start = TimingClock::now();
  const auto update_timestamps_start = TimingClock::now();
  updatePoseTimestamps(new_stamps);
  if (native_imu_mean_bridge_)
    native_imu_mean_bridge_->pruneBefore(
        latestPoseStamp() - std::max(1.0, 3.0 * smoother.smootherLag()));
  const double update_timestamps_ms =
      timing ? timing_elapsed_ms(update_timestamps_start) : 0.0;

  const auto remove_active_window_start = TimingClock::now();
  const size_t active_window_removed =
      removeExpiredActiveWindowFactors(
          smoother, new_values, new_stamps, factors_to_remove);
  const double remove_active_window_ms =
      timing ? timing_elapsed_ms(remove_active_window_start) : 0.0;

  const auto sidecar_start = TimingClock::now();
  updateSidecar(new_factors, new_values, new_stamps);
  const double sidecar_update_ms =
      timing ? timing_elapsed_ms(sidecar_start) : 0.0;

  const auto consume_start = TimingClock::now();
  std::vector<MatchedBelief> matched_beliefs = consumeDeferredMatchedBeliefs();
  std::vector<MatchedBelief> newly_matched_beliefs = consumeIncoming();
  matched_beliefs.insert(matched_beliefs.end(),
                         newly_matched_beliefs.begin(),
                         newly_matched_beliefs.end());
  if (live_revision_enable_ && mode_ == Mode::InjectPersistent) {
    std::map<SourceBeliefId, MatchedBelief> newest_by_source;
    for (const auto& matched : matched_beliefs) {
      const SourceBeliefId id{
          static_cast<int>(matched.incoming.source_agent),
          matched.incoming.from_pose_index,
          matched.incoming.to_pose_index};
      auto it = newest_by_source.find(id);
      if (it == newest_by_source.end() ||
          matched.incoming.revision > it->second.incoming.revision) {
        newest_by_source.insert_or_assign(id, matched);
      }
    }
    matched_beliefs.clear();
    for (const auto& item : newest_by_source) {
      matched_beliefs.push_back(item.second);
    }
  }
  const double consume_incoming_ms =
      timing ? timing_elapsed_ms(consume_start) : 0.0;
  size_t injected = 0u;
  size_t duplicates = 0u;
  size_t rejected = 0u;
  size_t deferred = 0u;

  const auto inject_loop_start = TimingClock::now();
  if (mode_ == Mode::ActiveWindowTemporary) {
    std::vector<MatchedBelief> selected_beliefs;
    std::map<ReceiverEdgeId, size_t> selected_by_receiver_edge;
    for (const auto& matched : matched_beliefs) {
      const EdgeId edge_id = edgeId(matched);
      if (injected_edges_.find(edge_id) != injected_edges_.end()) {
        ++duplicates;
        drawRerunBeliefDebug(matched, smoother, new_values);
        continue;
      }

      const ReceiverEdgeId receiver_edge_id = receiverEdgeId(matched);
      if (active_window_external_factors_.find(receiver_edge_id) !=
          active_window_external_factors_.end()) {
        ++duplicates;
        drawRerunBeliefDebug(matched, smoother, new_values);
        ROS_INFO_STREAM(
            "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,"
            "skipped_receiver_edge_already_active,"
            << poseToken(self_agent_id_, matched.local_from_index) << ","
            << poseToken(self_agent_id_, matched.local_to_index) << ","
            << poseToken(matched.incoming.source_agent,
                         matched.incoming.from_pose_index)
            << "->"
            << poseToken(matched.incoming.source_agent,
                         matched.incoming.to_pose_index));
        continue;
      }

      auto selected_it = selected_by_receiver_edge.find(receiver_edge_id);
      if (selected_it != selected_by_receiver_edge.end()) {
        ++duplicates;
        drawRerunBeliefDebug(matched, smoother, new_values);
        ROS_INFO_STREAM(
            "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,"
            "skipped_receiver_edge_already_pending,"
            << poseToken(self_agent_id_, matched.local_from_index) << ","
            << poseToken(self_agent_id_, matched.local_to_index) << ","
            << poseToken(matched.incoming.source_agent,
                         matched.incoming.from_pose_index)
            << "->"
            << poseToken(matched.incoming.source_agent,
                         matched.incoming.to_pose_index));
        continue;
      }
      selected_by_receiver_edge.emplace(receiver_edge_id,
                                        selected_beliefs.size());
      selected_beliefs.push_back(matched);
    }

    for (const auto& matched : selected_beliefs) {
      if (shouldDeferForReceiverCovariance(matched, smoother)) {
        if (shouldRetryMatched(matched)) {
          enqueueDeferredMatchedBelief(matched);
          ++deferred;
          ROS_INFO_STREAM(
              "GLIM_CBS_ODOM_INJECT_ROW,"
              << poseToken(matched.incoming.source_agent,
                           matched.incoming.from_pose_index)
              << "->"
              << poseToken(matched.incoming.source_agent,
                           matched.incoming.to_pose_index)
              << "," << poseToken(self_agent_id_, matched.local_from_index)
              << "->" << poseToken(self_agent_id_, matched.local_to_index)
              << "," << matched.from_abs_dt << "," << matched.to_abs_dt
              << ",nan,deferred_wait_linearization");
        } else {
          ++rejected;
        }
        continue;
      }
      const bool accepted = appendActiveWindowTemporaryFactor(
          matched, smoother, new_values, &new_factors, factors_to_remove);
      if (accepted) {
        injected_edges_.insert(edgeId(matched));
        ++injected;
        drawRerunBeliefDebug(matched, smoother, new_values);
      } else {
        ++rejected;
      }
    }
  } else {
    for (const auto& matched : matched_beliefs) {
      const bool duplicate_or_observe_skip = !shouldInject(matched);
      if (duplicate_or_observe_skip) {
        if (mode_ != Mode::ObserveOnly) {
          ++duplicates;
        }
        drawRerunBeliefDebug(matched, smoother, new_values);
        continue;
      }
      if (shouldDeferForReceiverCovariance(matched, smoother)) {
        if (shouldRetryMatched(matched)) {
          enqueueDeferredMatchedBelief(matched);
          ++deferred;
          ROS_INFO_STREAM(
              "GLIM_CBS_ODOM_INJECT_ROW,"
              << poseToken(matched.incoming.source_agent,
                           matched.incoming.from_pose_index)
              << "->"
              << poseToken(matched.incoming.source_agent,
                           matched.incoming.to_pose_index)
              << "," << poseToken(self_agent_id_, matched.local_from_index)
              << "->" << poseToken(self_agent_id_, matched.local_to_index)
              << "," << matched.from_abs_dt << "," << matched.to_abs_dt
              << ",nan,deferred_wait_linearization");
        } else {
          ++rejected;
        }
        continue;
      }
      bool accepted = false;
      if (mode_ == Mode::InjectPersistent) {
        accepted =
            appendExternalFactor(matched, smoother, new_values, &new_factors,
                                 &factors_to_remove);
      } else if (mode_ == Mode::TemporaryLinear) {
        accepted = queueTemporaryFactor(matched, smoother, new_values);
      }
      if (accepted) {
        ++injected;
        drawRerunBeliefDebug(matched, smoother, new_values);
      } else {
        ++rejected;
      }
    }
  }
  if (!factors_to_remove.empty()) {
    std::sort(factors_to_remove.begin(), factors_to_remove.end());
    factors_to_remove.erase(
        std::unique(factors_to_remove.begin(), factors_to_remove.end()),
        factors_to_remove.end());
  }
  const double inject_loop_ms =
      timing ? timing_elapsed_ms(inject_loop_start) : 0.0;

  injected_total_.fetch_add(injected, std::memory_order_relaxed);
  duplicate_total_.fetch_add(duplicates, std::memory_order_relaxed);
  rejected_total_.fetch_add(rejected, std::memory_order_relaxed);
  size_t pending_count = 0u;
  double pending_oldest_age_sec = std::numeric_limits<double>::quiet_NaN();
  {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    pending_count = incoming_.size();
    if (!incoming_.empty()) {
      const double latest_stamp = latestPoseStamp();
      const double pending_stamp = incoming_.front().to_stamp_sec > 0.0
                                       ? incoming_.front().to_stamp_sec
                                       : incoming_.front().from_stamp_sec;
      if (std::isfinite(latest_stamp) && std::isfinite(pending_stamp)) {
        pending_oldest_age_sec = latest_stamp - pending_stamp;
      }
    }
  }
  rerun_last_cbs_pending_count_ = pending_count;
  rerun_last_cbs_pending_deferred_count_ = deferred_matched_beliefs_.size();
  rerun_last_cbs_pending_oldest_age_sec_ = pending_oldest_age_sec;

  ROS_INFO_STREAM_THROTTLE(
      1.0,
      "GLIM CBS incoming flow: matched="
          << matched_beliefs.size() << " injected=" << injected
          << " duplicate=" << duplicates << " rejected=" << rejected
          << " deferred=" << deferred << " pending=" << pending_count
          << " pending_deferred=" << deferred_matched_beliefs_.size());
  if (timing) {
    ROS_INFO_STREAM(
        "GLIM_CBS_TIMING_ROW,on_smoother_update,"
        << (has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u) << ","
          << matched_beliefs.size() << "," << injected << "," << duplicates
          << "," << rejected << "," << pending_count << ","
          << pending_oldest_age_sec << "," << update_timestamps_ms << ","
          << consume_incoming_ms << "," << inject_loop_ms << ","
          << sidecar_update_ms << "," << remove_active_window_ms << ","
          << active_window_removed << ","
          << active_window_external_factors_.size() +
                 pending_active_window_external_factors_.size()
          << ",0,"
          << timing_elapsed_ms(total_start));
  }
}

void GlimCbsBridge::onSmootherUpdateFinish(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (!enabled_) {
    return;
  }
  const bool timing = timing_enabled();
  const auto total_start = TimingClock::now();
  commitPendingActiveWindowFactorRemovals();
  registerPendingActiveWindowFactors(smoother);
  const auto temporary_start = TimingClock::now();
  const size_t temporary_applied = applyTemporaryFactors(smoother);
  const double temporary_ms = timing ? timing_elapsed_ms(temporary_start) : 0.0;
  const auto publish_odom_start = TimingClock::now();
  publishOdometryAndRerun(smoother);
  const double publish_odom_ms =
      timing ? timing_elapsed_ms(publish_odom_start) : 0.0;
  const auto active_diagnostics_start = TimingClock::now();
  logActiveWindowFactorDiagnostics(smoother);
  if (timing) {
    ROS_INFO_STREAM("GLIM_CBS_DIAGNOSTIC_TIMING_ROW,"
                    << (has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u)
                    << "," << timing_elapsed_ms(active_diagnostics_start)
                    << "," << active_factor_diagnostics_budget_.samples()
                    << "," << active_factor_diagnostics_budget_.seen());
  }
  const ros::Time stamp =
      has_last_pose_key_
          ? ros::Time(pose_timestamps_sec_.at(poseIndex(last_pose_key_)))
          : ros::Time::now();
  finalizePoseMergeDiagnostic(smoother);
  const auto outgoing_start = TimingClock::now();
  publishOutgoingBeliefs(stamp, smoother);
  if (export_endpoint_audit_enable_ || live_revision_enable_) {
    const size_t update_index =
        has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u;
    const double record_stamp = ros::Time::now().toSec();
    liorf::pose_odom_belief_array revision_msg;
    revision_msg.header.stamp = stamp;
    revision_msg.header.frame_id = frame_id_;
    for (auto it = revision_audit_beliefs_.begin();
         it != revision_audit_beliefs_.end();) {
      const auto [from_index, to_index] = it->first;
      const gtsam::Key from_key = poseKey(from_index);
      const gtsam::Key to_key = poseKey(to_index);
      if (!smoother.getLinearizationPoint().exists(from_key) ||
          !smoother.getLinearizationPoint().exists(to_key)) {
        ROS_INFO_STREAM("GLIM_CBS_REVISION_RETIRED_ROW,"
                        << static_cast<char>(self_agent_id_) << ","
                        << from_index << "," << to_index << ","
                        << it->second.revision << "," << update_index);
        it = revision_audit_beliefs_.erase(it);
        continue;
      }
      if (it->second.last_update_index != update_index) {
        try {
          const auto from_pose = smoother.calculateEstimate<gtsam::Pose3>(from_key);
          const auto to_pose = smoother.calculateEstimate<gtsam::Pose3>(to_key);
          const auto mean = from_pose.between(to_pose);
          const auto joint = smoother.jointMarginalInformation(
              gtsam::KeyVector{from_key, to_key});
          const auto result = cbs::relative_pose_covariance::compute(
              joint, from_key, to_key, from_pose, to_pose, mean,
              outgoing_odom_covariance_mode_);
          const gtsam::Matrix6 covariance = result.covariance;
          const bool changed =
              gtsam::Pose3::Logmap(it->second.mean.between(mean)).norm() > 1e-12 ||
              (it->second.covariance - covariance).cwiseAbs().maxCoeff() > 1e-12;
          if (changed) {
            ++it->second.revision;
            if (export_endpoint_audit_enable_) {
              ROS_INFO_STREAM(
                "GLIM_CBS_REVISION_BELIEF_ROW,"
                << std::setprecision(17) << static_cast<char>(self_agent_id_)
                << "," << from_index << "," << to_index << ","
                << it->second.revision << "," << record_stamp << ","
                << update_index << "," << pose_timestamps_sec_[from_index]
                << "," << pose_timestamps_sec_[to_index] << ","
                << cbs::relative_pose_covariance::matrixToken(
                       gtsam::Pose3::Logmap(mean)) << ","
                << cbs::relative_pose_covariance::matrixToken(covariance));
            }
            if (live_revision_enable_) {
              liorf::pose_odom_belief revised;
              revised.header = revision_msg.header;
              revised.source_agent = static_cast<uint8_t>(self_agent_id_);
              revised.from_pose_index = static_cast<uint32_t>(from_index);
              revised.to_pose_index = static_cast<uint32_t>(to_index);
              revised.revision = static_cast<uint32_t>(it->second.revision);
              revised.from_stamp_sec = pose_timestamps_sec_.at(from_index);
              revised.to_stamp_sec = pose_timestamps_sec_.at(to_index);
              revised.relax_factor = relax_factor_;
              std::array<double, 6> mean_values;
              std::array<double, 36> covariance_values;
              vector6ToArray(gtsam::Pose3::Logmap(mean), &mean_values);
              matrix6ToArray(covariance, &covariance_values);
              for (size_t i = 0; i < mean_values.size(); ++i) {
                revised.relative_mu[i] = mean_values[i];
              }
              for (size_t i = 0; i < covariance_values.size(); ++i) {
                revised.covariance[i] = covariance_values[i];
              }
              revision_msg.beliefs.push_back(revised);
            }
            it->second.mean = mean;
            it->second.covariance = covariance;
          }
          it->second.last_update_index = update_index;
        } catch (const std::exception& e) {
          ROS_WARN_STREAM("GLIM_CBS_REVISION_ERROR_ROW,"
                          << from_index << "," << to_index << ","
                          << update_index << "," << e.what());
        }
      }
      ++it;
    }
    if (live_revision_enable_ && !revision_msg.beliefs.empty()) {
      belief_pub_.publish(revision_msg);
      ROS_INFO_STREAM("GLIM_CBS_LIVE_REVISION_ROW,published,"
                      << revision_msg.beliefs.size() << ","
                      << update_index);
    }
    for (const gtsam::Key key : smoother.getLinearizationPoint().keys()) {
      if (!isPoseKey(key)) {
        continue;
      }
      const size_t index = poseIndex(key);
      if (index >= pose_timestamps_sec_.size()) {
        continue;
      }
      const auto pose = smoother.calculateEstimate<gtsam::Pose3>(key);
      const auto q = pose.rotation().toQuaternion();
      ROS_INFO_STREAM(
          "GLIM_CBS_ACTIVE_POSE_ROW,"
          << std::setprecision(17) << static_cast<char>(self_agent_id_)
          << "," << update_index << "," << record_stamp << ","
          << index << "," << pose_timestamps_sec_[index] << ","
          << pose.translation().x() << "," << pose.translation().y() << ","
          << pose.translation().z() << "," << q.x() << "," << q.y()
          << "," << q.z() << "," << q.w());
    }
  }
  const double outgoing_ms = timing ? timing_elapsed_ms(outgoing_start) : 0.0;
  rerun_last_cbs_temporary_ms_ = temporary_ms;
  rerun_last_cbs_publish_odometry_ms_ = publish_odom_ms;
  rerun_last_cbs_publish_outgoing_ms_ = outgoing_ms;
  rerun_last_cbs_total_ms_ = timing ? timing_elapsed_ms(total_start) : 0.0;
  const auto scalar_start = TimingClock::now();
  drawRerunScalars(stamp);
  const double scalar_ms = timing ? timing_elapsed_ms(scalar_start) : 0.0;
  if (timing) {
    ROS_INFO_STREAM(
        "GLIM_CBS_TIMING_ROW,on_smoother_update_finish,"
        << (has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u)
        << ",0," << temporary_applied << ",0,0,0,"
        << std::numeric_limits<double>::quiet_NaN()
        << ",0,0,0," << temporary_ms << "," << publish_odom_ms << "," << outgoing_ms << ","
        << scalar_ms << ",0," << timing_elapsed_ms(total_start));
  }
}

void GlimCbsBridge::cacheNativeImu(double stamp, const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& gyro) {
  if (native_imu_mean_bridge_) native_imu_mean_bridge_->append(stamp, acc, gyro);
}

void GlimCbsBridge::updatePoseTimestamps(
    const std::map<std::uint64_t, double>& new_stamps) {
  for (const auto& [key, stamp] : new_stamps) {
    if (!isPoseKey(key)) {
      continue;
    }
    const size_t index = poseIndex(key);
    if (pose_timestamps_sec_.size() <= index) {
      pose_timestamps_sec_.resize(
          index + 1u, std::numeric_limits<double>::quiet_NaN());
    }
    pose_timestamps_sec_[index] = stamp;
    last_pose_key_ = key;
    has_last_pose_key_ = true;
  }
}

bool GlimCbsBridge::findClosestPoseByStamp(double stamp_sec,
                                           size_t* index,
                                           double* abs_dt) const {
  if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0 ||
      pose_timestamps_sec_.empty()) {
    return false;
  }

  double best_dt = std::numeric_limits<double>::infinity();
  size_t best_index = 0u;
  for (size_t i = 0; i < pose_timestamps_sec_.size(); ++i) {
    const double candidate = pose_timestamps_sec_[i];
    if (!std::isfinite(candidate) || candidate <= 0.0) {
      continue;
    }
    const double dt = std::abs(candidate - stamp_sec);
    if (dt < best_dt) {
      best_dt = dt;
      best_index = i;
    }
  }
  if (!std::isfinite(best_dt) || best_dt > timestamp_tolerance_sec_) {
    return false;
  }
  if (index) {
    *index = best_index;
  }
  if (abs_dt) {
    *abs_dt = best_dt;
  }
  return true;
}

bool GlimCbsBridge::findReceiverEdgeForIncoming(
    const IncomingBelief& incoming,
    size_t* from_index,
    size_t* to_index,
    double* from_abs_dt,
    double* to_abs_dt) const {
  if (receiver_match_mode_ != "duration_aware_edge") {
    size_t nearest_from = 0u;
    size_t nearest_to = 0u;
    double nearest_from_dt = std::numeric_limits<double>::quiet_NaN();
    double nearest_to_dt = std::numeric_limits<double>::quiet_NaN();
    const bool from_matched = findClosestPoseByStamp(
        incoming.from_stamp_sec, &nearest_from, &nearest_from_dt);
    const bool to_matched = findClosestPoseByStamp(
        incoming.to_stamp_sec, &nearest_to, &nearest_to_dt);
    if (!from_matched || !to_matched || nearest_from == nearest_to ||
        !isLikelyActivePose(nearest_from) || !isLikelyActivePose(nearest_to)) {
      return false;
    }
    if (from_index) {
      *from_index = nearest_from;
    }
    if (to_index) {
      *to_index = nearest_to;
    }
    if (from_abs_dt) {
      *from_abs_dt = nearest_from_dt;
    }
    if (to_abs_dt) {
      *to_abs_dt = nearest_to_dt;
    }
    return true;
  }

  const double sender_dt = incoming.to_stamp_sec - incoming.from_stamp_sec;
  if (!std::isfinite(sender_dt) || sender_dt <= 0.0 ||
      pose_timestamps_sec_.size() < 2u) {
    return false;
  }

  const double latest_stamp = latestPoseStamp();
  if (!std::isfinite(latest_stamp)) {
    return false;
  }
  const double active_cutoff =
      latest_stamp - smoother_lag_sec_ - timestamp_tolerance_sec_;

  std::vector<std::pair<size_t, double>> from_candidates;
  std::vector<std::pair<size_t, double>> to_candidates;
  from_candidates.reserve(4u);
  to_candidates.reserve(4u);
  for (size_t i = 0u; i < pose_timestamps_sec_.size(); ++i) {
    const double stamp = pose_timestamps_sec_[i];
    if (!std::isfinite(stamp) || stamp <= 0.0 || stamp < active_cutoff) {
      continue;
    }
    const double candidate_from_dt = std::abs(stamp - incoming.from_stamp_sec);
    if (std::isfinite(candidate_from_dt) &&
        candidate_from_dt <= timestamp_tolerance_sec_) {
      from_candidates.emplace_back(i, candidate_from_dt);
    }
    const double candidate_to_dt = std::abs(stamp - incoming.to_stamp_sec);
    if (std::isfinite(candidate_to_dt) &&
        candidate_to_dt <= timestamp_tolerance_sec_) {
      to_candidates.emplace_back(i, candidate_to_dt);
    }
  }
  if (from_candidates.empty() || to_candidates.empty()) {
    return false;
  }

  size_t best_from = 0u;
  size_t best_to = 0u;
  double best_from_dt = std::numeric_limits<double>::quiet_NaN();
  double best_to_dt = std::numeric_limits<double>::quiet_NaN();
  double best_duration_error = std::numeric_limits<double>::infinity();
  double best_endpoint_error = std::numeric_limits<double>::infinity();

  for (const auto& [candidate_from, candidate_from_dt] : from_candidates) {
    const double receiver_from_stamp = pose_timestamps_sec_[candidate_from];
    for (const auto& [candidate_to, candidate_to_dt] : to_candidates) {
      if (candidate_to <= candidate_from) {
        continue;
      }
      const double receiver_to_stamp = pose_timestamps_sec_[candidate_to];
      const double receiver_dt = receiver_to_stamp - receiver_from_stamp;
      if (!std::isfinite(receiver_dt) || receiver_dt <= 0.0) {
        continue;
      }

      const double duration_error = std::abs(sender_dt - receiver_dt);
      const double endpoint_error = candidate_from_dt + candidate_to_dt;
      if (duration_error < best_duration_error ||
          (duration_error == best_duration_error &&
           endpoint_error < best_endpoint_error)) {
        best_duration_error = duration_error;
        best_endpoint_error = endpoint_error;
        best_from = candidate_from;
        best_to = candidate_to;
        best_from_dt = candidate_from_dt;
        best_to_dt = candidate_to_dt;
      }
    }
  }

  if (!std::isfinite(best_duration_error)) {
    return false;
  }
  if (from_index) {
    *from_index = best_from;
  }
  if (to_index) {
    *to_index = best_to;
  }
  if (from_abs_dt) {
    *from_abs_dt = best_from_dt;
  }
  if (to_abs_dt) {
    *to_abs_dt = best_to_dt;
  }
  return true;
}

bool GlimCbsBridge::isLikelyActivePose(size_t index) const {
  if (index >= pose_timestamps_sec_.size()) {
    return false;
  }
  const double stamp = pose_timestamps_sec_[index];
  const double latest_stamp = latestPoseStamp();
  if (!std::isfinite(stamp) || !std::isfinite(latest_stamp)) {
    return false;
  }
  return stamp >= latest_stamp - smoother_lag_sec_ - timestamp_tolerance_sec_;
}

bool GlimCbsBridge::shouldRetry(const IncomingBelief& belief) const {
  if (unmatched_retry_max_beliefs_ == 0u || pose_timestamps_sec_.empty()) {
    return false;
  }
  const double latest_stamp = latestPoseStamp();
  if (!std::isfinite(latest_stamp) || latest_stamp <= 0.0) {
    return true;
  }
  const double belief_stamp =
      belief.to_stamp_sec > 0.0 ? belief.to_stamp_sec : belief.from_stamp_sec;
  if (!std::isfinite(belief_stamp) || belief_stamp <= 0.0) {
    return false;
  }
  return latest_stamp - belief_stamp <= unmatched_retry_max_age_sec_;
}

bool GlimCbsBridge::shouldRetryMatched(const MatchedBelief& belief) const {
  if (unmatched_retry_max_beliefs_ == 0u || pose_timestamps_sec_.empty()) {
    return false;
  }
  if (!isLikelyActivePose(belief.local_from_index) ||
      !isLikelyActivePose(belief.local_to_index)) {
    return false;
  }
  return shouldRetry(belief.incoming);
}

double GlimCbsBridge::latestPoseStamp() const {
  for (auto it = pose_timestamps_sec_.rbegin();
       it != pose_timestamps_sec_.rend();
       ++it) {
    if (std::isfinite(*it) && *it > 0.0) {
      return *it;
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

std::vector<GlimCbsBridge::MatchedBelief>
GlimCbsBridge::consumeDeferredMatchedBeliefs() {
  if (deferred_matched_beliefs_.empty()) {
    return {};
  }
  std::vector<MatchedBelief> pending;
  pending.reserve(deferred_matched_beliefs_.size());
  while (!deferred_matched_beliefs_.empty()) {
    pending.push_back(deferred_matched_beliefs_.front());
    deferred_matched_beliefs_.pop_front();
  }
  return pending;
}

bool GlimCbsBridge::shouldDeferForReceiverCovariance(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother)
    const {
  if (!wait_for_receiver_covariance_linearization_enable_) {
    return false;
  }
  if (!health_receiver_params_.enable ||
      !health_receiver_params_.receiver_nis_enable ||
      !health_receiver_params_.relative_trust_enable) {
    return false;
  }
  const auto& linearization_point = smoother.getLinearizationPoint();
  return !linearization_point.exists(matched.local_from_key) ||
         !linearization_point.exists(matched.local_to_key);
}

void GlimCbsBridge::enqueueDeferredMatchedBelief(const MatchedBelief& matched) {
  if (live_revision_enable_) {
    const auto previous = std::find_if(
        deferred_matched_beliefs_.begin(), deferred_matched_beliefs_.end(),
        [&matched](const MatchedBelief& candidate) {
          return candidate.incoming.source_agent ==
                     matched.incoming.source_agent &&
                 candidate.incoming.from_pose_index ==
                     matched.incoming.from_pose_index &&
                 candidate.incoming.to_pose_index ==
                     matched.incoming.to_pose_index;
        });
    if (previous != deferred_matched_beliefs_.end()) {
      if (matched.incoming.revision > previous->incoming.revision) {
        *previous = matched;
      }
      return;
    }
  }
  deferred_matched_beliefs_.push_back(matched);
  while (deferred_matched_beliefs_.size() > unmatched_retry_max_beliefs_) {
    deferred_matched_beliefs_.pop_front();
  }
}

std::vector<GlimCbsBridge::MatchedBelief> GlimCbsBridge::consumeIncoming() {
  std::deque<IncomingBelief> pending;
  {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    pending.swap(incoming_);
  }
  if (pending.empty()) {
    return {};
  }

  std::vector<MatchedBelief> matched;
  std::vector<IncomingBelief> retry;
  size_t dropped = 0u;
  size_t retried = 0u;
  size_t duration_mismatches = 0u;
  for (const auto& incoming : pending) {
    size_t from_index = 0u;
    size_t to_index = 0u;
    double from_abs_dt = std::numeric_limits<double>::quiet_NaN();
    double to_abs_dt = std::numeric_limits<double>::quiet_NaN();
    const bool edge_matched = findReceiverEdgeForIncoming(
        incoming, &from_index, &to_index, &from_abs_dt, &to_abs_dt);
    if (!edge_matched) {
      if (shouldRetry(incoming)) {
        retry.push_back(incoming);
        ++retried;
      } else {
        ++dropped;
      }
      continue;
    }

    const double sender_dt = incoming.to_stamp_sec - incoming.from_stamp_sec;
    const double receiver_dt =
        pose_timestamps_sec_[to_index] - pose_timestamps_sec_[from_index];
    const double duration_error = std::abs(sender_dt - receiver_dt);
    const double duration_ratio =
        receiver_dt > 0.0 ? sender_dt / receiver_dt
                          : std::numeric_limits<double>::quiet_NaN();
    bool duration_ok = true;
    if (duration_gate_enable_) {
      duration_ok = std::isfinite(sender_dt) && std::isfinite(receiver_dt) &&
                    sender_dt > 0.0 && receiver_dt > 0.0 &&
                    duration_error <= duration_tolerance_sec_;
      if (duration_ok && duration_ratio_min_ > 0.0) {
        duration_ok = duration_ratio >= duration_ratio_min_;
      }
      if (duration_ok && duration_ratio_max_ > 0.0) {
        duration_ok = duration_ratio <= duration_ratio_max_;
      }
    }
    if (!duration_ok) {
      ++dropped;
      ++duration_mismatches;
      ROS_INFO_STREAM(
          std::fixed << std::setprecision(9)
                     << "CBS_ODOM_MATCH_ROW_K2G,"
                     << poseToken(incoming.source_agent,
                                  incoming.from_pose_index)
                     << "->"
                     << poseToken(incoming.source_agent,
                                  incoming.to_pose_index)
                     << "," << incoming.from_stamp_sec << ","
                     << incoming.to_stamp_sec << ","
                     << poseToken(self_agent_id_, from_index) << "->"
                     << poseToken(self_agent_id_, to_index) << ","
                     << pose_timestamps_sec_[from_index] << ","
                     << pose_timestamps_sec_[to_index] << "," << from_abs_dt
                     << "," << to_abs_dt << "," << timestamp_tolerance_sec_
                     << ",none,none,duration_mismatch," << sender_dt << ","
                     << receiver_dt << "," << duration_error << ","
                     << duration_ratio);
      continue;
    }

    MatchedBelief item;
    item.incoming = incoming;
    item.local_from_index = from_index;
    item.local_to_index = to_index;
    item.local_from_key = poseKey(from_index);
    item.local_to_key = poseKey(to_index);
    item.from_abs_dt = from_abs_dt;
    item.to_abs_dt = to_abs_dt;
    matched.push_back(item);
  }

  if (!retry.empty()) {
    std::lock_guard<std::mutex> lock(incoming_mutex_);
    for (const auto& belief : retry) {
      if (live_revision_enable_) {
        const auto previous = std::find_if(
            incoming_.begin(), incoming_.end(),
            [&belief](const IncomingBelief& candidate) {
              return candidate.source_agent == belief.source_agent &&
                     candidate.from_pose_index == belief.from_pose_index &&
                     candidate.to_pose_index == belief.to_pose_index;
            });
        if (previous != incoming_.end()) {
          if (belief.revision > previous->revision) {
            *previous = belief;
          }
          continue;
        }
      }
      incoming_.push_back(belief);
    }
    while (incoming_.size() > unmatched_retry_max_beliefs_) {
      incoming_.pop_front();
      ++dropped;
    }
  }

  matched_total_.fetch_add(matched.size(), std::memory_order_relaxed);
  dropped_total_.fetch_add(dropped, std::memory_order_relaxed);
  retried_total_.fetch_add(retried, std::memory_order_relaxed);
  duration_mismatch_total_.fetch_add(duration_mismatches, std::memory_order_relaxed);
  return matched;
}

bool GlimCbsBridge::shouldInject(const MatchedBelief& matched) {
  if (mode_ == Mode::ObserveOnly) {
    return false;
  }
  if (live_revision_enable_ && mode_ == Mode::InjectPersistent) {
    const SourceBeliefId id{
        static_cast<int>(matched.incoming.source_agent),
        matched.incoming.from_pose_index,
        matched.incoming.to_pose_index};
    const auto previous = live_revision_factors_.find(id);
    if (previous != live_revision_factors_.end() &&
        matched.incoming.revision <= previous->second.revision) {
      ROS_INFO_STREAM("GLIM_CBS_LIVE_REVISION_ROW,ignored,"
                      << matched.incoming.from_pose_index << ","
                      << matched.incoming.to_pose_index << ","
                      << matched.incoming.revision << ","
                      << previous->second.revision);
      return false;
    }
    return true;
  }
  const auto edge_id = edgeId(matched);
  const auto [_, inserted] = injected_edges_.insert(edge_id);
  return inserted;
}

GlimCbsBridge::EdgeId GlimCbsBridge::edgeId(const MatchedBelief& matched) const {
  return std::make_tuple(static_cast<int>(matched.incoming.source_agent),
                         matched.incoming.from_pose_index,
                         matched.incoming.to_pose_index,
                         matched.local_from_index,
                         matched.local_to_index);
}

GlimCbsBridge::ReceiverEdgeId GlimCbsBridge::receiverEdgeId(
    const MatchedBelief& matched) const {
  return std::make_tuple(static_cast<int>(matched.incoming.source_agent),
                         matched.local_from_index,
                         matched.local_to_index);
}

size_t GlimCbsBridge::removeExpiredActiveWindowFactors(
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    const std::map<std::uint64_t, double>& new_stamps,
    gtsam::FactorIndices& factors_to_remove) {
  if (mode_ != Mode::ActiveWindowTemporary ||
      active_window_external_factors_.empty()) {
    return 0u;
  }
  pending_active_window_external_factor_removals_.clear();

  double latest_stamp = latestPoseStamp();
  for (const auto& [key, stamp] : new_stamps) {
    (void)key;
    if (std::isfinite(stamp)) {
      latest_stamp = std::isfinite(latest_stamp)
                         ? std::max(latest_stamp, stamp)
                         : stamp;
    }
  }
  if (!std::isfinite(latest_stamp)) {
    return 0u;
  }

  const double oldest_active_stamp = latest_stamp - smoother.smootherLag();
  gtsam::KeySet active_pose_keys;
  const auto& linearization_point = smoother.getLinearizationPoint();
  const gtsam::NonlinearFactorGraph& factors = smoother.getFactors();
  const auto find_factor_index =
      [&](const ActiveWindowExternalFactor& tracked)
          -> std::optional<gtsam::FactorIndex> {
    if (tracked.factor_index && *tracked.factor_index < factors.size()) {
      const auto& candidate = factors[*tracked.factor_index];
      if (candidate == tracked.factor &&
          factorMatchesPoseEdge(
              candidate, tracked.local_from_key, tracked.local_to_key)) {
        return *tracked.factor_index;
      }
    }
    for (gtsam::FactorIndex i = 0; i < factors.size(); ++i) {
      const auto& candidate = factors[i];
      if (candidate == tracked.factor &&
          factorMatchesPoseEdge(
              candidate, tracked.local_from_key, tracked.local_to_key)) {
        return i;
      }
    }
    return std::nullopt;
  };
  for (size_t i = 0u; i < pose_timestamps_sec_.size(); ++i) {
    const double stamp = pose_timestamps_sec_[i];
    if (!std::isfinite(stamp) || stamp < oldest_active_stamp) {
      continue;
    }
    const gtsam::Key key = poseKey(i);
    if (linearization_point.exists(key) || new_values.exists(key)) {
      active_pose_keys.insert(key);
    }
  }
  for (const auto& [raw_key, stamp] : new_stamps) {
    const gtsam::Key key = static_cast<gtsam::Key>(raw_key);
    if (isPoseKey(key) && std::isfinite(stamp) &&
        stamp >= oldest_active_stamp &&
        (linearization_point.exists(key) || new_values.exists(key))) {
      active_pose_keys.insert(key);
    }
  }

  size_t removed = 0u;
  for (auto it = active_window_external_factors_.begin();
       it != active_window_external_factors_.end(); ++it) {
    ActiveWindowExternalFactor& tracked = it->second;
    const std::optional<gtsam::FactorIndex> factor_index =
        find_factor_index(tracked);
    if (factor_index) {
      tracked.factor_index = *factor_index;
    }
    const bool still_active =
        active_pose_keys.find(tracked.local_from_key) != active_pose_keys.end() &&
        active_pose_keys.find(tracked.local_to_key) != active_pose_keys.end();
    if (still_active && factor_index) {
      continue;
    }

    ++removed;
    if (factor_index) {
      factors_to_remove.push_back(*factor_index);
    } else {
      ROS_WARN_STREAM(
          "GLIM CBS active-window factor for "
          << poseKeyToken(tracked.local_from_key) << "->"
          << poseKeyToken(tracked.local_to_key)
          << (still_active ? " lost its graph slot while still active"
                           : " expired without a valid graph slot")
          << "; dropping bookkeeping only");
    }
    pending_active_window_external_factor_removals_.push_back(it->first);
    ROS_INFO_STREAM(
        "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_stage_remove,"
        << poseKeyToken(tracked.local_from_key) << ","
        << poseKeyToken(tracked.local_to_key) << ",slot="
        << (factor_index ? std::to_string(*factor_index) : "none")
        << ",total_active=" << active_window_external_factors_.size());
  }

  return removed;
}

void GlimCbsBridge::commitPendingActiveWindowFactorRemovals() {
  if (pending_active_window_external_factor_removals_.empty()) {
    return;
  }

  for (const ReceiverEdgeId& receiver_edge_id :
       pending_active_window_external_factor_removals_) {
    const auto it = active_window_external_factors_.find(receiver_edge_id);
    if (it == active_window_external_factors_.end()) {
      continue;
    }
    const ActiveWindowExternalFactor tracked = it->second;
    active_window_external_factors_.erase(it);
    ROS_INFO_STREAM(
        "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_remove_committed,"
        << poseKeyToken(tracked.local_from_key) << ","
        << poseKeyToken(tracked.local_to_key) << ",active_updates="
        << tracked.active_update_count << ",active_age_sec="
        << (latestPoseStamp() - tracked.inserted_stamp_sec)
        << ",total_active="
        << active_window_external_factors_.size());
  }
  pending_active_window_external_factor_removals_.clear();
}

void GlimCbsBridge::registerPendingActiveWindowFactors(
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (mode_ != Mode::ActiveWindowTemporary ||
      pending_active_window_external_factors_.empty()) {
    return;
  }

  const gtsam::ISAM2Result& result = smoother.getISAM2Result();
  const gtsam::NonlinearFactorGraph& factors = smoother.getFactors();
  for (auto& pending : pending_active_window_external_factors_) {
    auto& tracked = pending.tracked;
    std::optional<gtsam::FactorIndex> factor_index;
    if (pending.new_factor_index < result.newFactorsIndices.size()) {
      const gtsam::FactorIndex candidate =
          result.newFactorsIndices[pending.new_factor_index];
      if (candidate < factors.size() && factors[candidate] == tracked.factor &&
          factorMatchesPoseEdge(factors[candidate],
                                tracked.local_from_key,
                                tracked.local_to_key)) {
        factor_index = candidate;
      }
    }

    if (!factor_index) {
      for (gtsam::FactorIndex i = 0; i < factors.size(); ++i) {
        if (factors[i] == tracked.factor &&
            factorMatchesPoseEdge(
                factors[i], tracked.local_from_key, tracked.local_to_key)) {
          factor_index = i;
          break;
        }
      }
    }

    if (!factor_index) {
      ROS_FATAL_STREAM(
          "GLIM CBS active-window factor for "
          << poseKeyToken(tracked.local_from_key) << "->"
          << poseKeyToken(tracked.local_to_key)
          << " was inserted but no iSAM2 factor slot could be tracked");
      pending_active_window_external_factors_.clear();
      throw std::runtime_error(
          "GLIM CBS active-window factor inserted without tracked slot");
    }

    tracked.factor_index = factor_index;
    active_window_external_factors_.insert_or_assign(tracked.receiver_edge_id,
                                                     tracked);
    ROS_INFO_STREAM("GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,slot_tracked,"
                    << poseKeyToken(tracked.local_from_key) << ","
                    << poseKeyToken(tracked.local_to_key) << ",slot="
                    << *factor_index << ",total_active="
                    << active_window_external_factors_.size());
  }
  pending_active_window_external_factors_.clear();
}

void GlimCbsBridge::logActiveWindowFactorDiagnostics(
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (mode_ != Mode::ActiveWindowTemporary ||
      active_window_external_factors_.empty()) {
    return;
  }

  // Lifecycle accounting must not depend on whether costly diagnostics run.
  for (auto& item : active_window_external_factors_) {
    ++item.second.active_update_count;
  }
  // Gate before calculateEstimate/error/linearize. The old detail-log flag
  // controlled only printing, leaving all GPU factor queries unconditional.
  if (!active_factor_diagnostics_budget_.shouldSample()) {
    return;
  }

  gtsam::Values estimate;
  try {
    estimate = smoother.calculateEstimate();
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("GLIM CBS active-window diagnostics skipped: "
                    << e.what());
    return;
  }

  const gtsam::NonlinearFactorGraph& factors = smoother.getFactors();
  std::unordered_set<const gtsam::NonlinearFactor*> external_factor_ptrs;
  external_factor_ptrs.reserve(active_window_external_factors_.size());
  for (const auto& item : active_window_external_factors_) {
    if (item.second.factor) {
      external_factor_ptrs.insert(item.second.factor.get());
    }
  }

  const double latest_stamp = latestPoseStamp();
  size_t active_factor_detail_rows = 0u;
  for (auto& [receiver_edge_id, tracked] : active_window_external_factors_) {
    (void)receiver_edge_id;

    double cbs_error = std::numeric_limits<double>::quiet_NaN();
    double cbs_hessian_frobenius = std::numeric_limits<double>::quiet_NaN();
    if (tracked.factor) {
      cbs_error = factorErrorOrNan(*tracked.factor, estimate);
      cbs_hessian_frobenius = factorPoseHessianFrobeniusOrNan(
          *tracked.factor,
          estimate,
          tracked.local_from_key,
          tracked.local_to_key);
    }

    double residual_norm = std::numeric_limits<double>::quiet_NaN();
    double rot_norm = std::numeric_limits<double>::quiet_NaN();
    double trans_norm = std::numeric_limits<double>::quiet_NaN();
    double nis = std::numeric_limits<double>::quiet_NaN();
    double whitened_norm = std::numeric_limits<double>::quiet_NaN();
    try {
      const gtsam::Pose3 receiver_from =
          estimate.at<gtsam::Pose3>(tracked.local_from_key);
      const gtsam::Pose3 receiver_to =
          estimate.at<gtsam::Pose3>(tracked.local_to_key);
      const gtsam::Pose3 receiver_from_to =
          receiver_from.between(receiver_to);
      const gtsam::Vector6 residual =
          gtsam::traits<gtsam::Pose3>::Logmap(
              tracked.measured_from_to.inverse().compose(receiver_from_to));
      residual_norm = residual.norm();
      rot_norm = residual.head(3).norm();
      trans_norm = residual.tail(3).norm();
      nis = covarianceNis(tracked.covariance, residual);
      if (std::isfinite(nis) && nis >= 0.0) {
        whitened_norm = std::sqrt(nis);
      }
    } catch (const std::exception&) {
    }

    FactorInfluenceSummary same_edge;
    FactorInfluenceSummary unary;
    FactorInfluenceSummary other_local;
    FactorInfluenceSummary other_external;
    const double active_age_sec =
        std::isfinite(latest_stamp) && std::isfinite(tracked.inserted_stamp_sec)
            ? latest_stamp - tracked.inserted_stamp_sec
            : std::numeric_limits<double>::quiet_NaN();
    const auto maybe_log_detail =
        [&](const gtsam::NonlinearFactor& factor,
            const std::string& category,
            std::optional<gtsam::FactorIndex> graph_factor_index,
            bool is_external,
            bool has_from,
            bool has_to,
            double error,
            double hessian_frobenius) {
          if (!active_factor_detail_log_enable_ ||
              active_factor_detail_max_rows_per_update_ == 0u ||
              active_factor_detail_rows >=
                  active_factor_detail_max_rows_per_update_) {
            return;
          }
          ++active_factor_detail_rows;
          ROS_INFO_STREAM(
              "GLIM_CBS_ACTIVE_FACTOR_DETAIL_ROW,"
              << directionToken(tracked.source_agent, self_agent_id_) << ","
              << agentToken(self_agent_id_) << ","
              << agentToken(tracked.source_agent) << ","
              << poseKeyToken(tracked.local_from_key) << ","
              << poseKeyToken(tracked.local_to_key) << ","
              << poseToken(tracked.source_agent, tracked.source_from_index)
              << "->"
              << poseToken(tracked.source_agent, tracked.source_to_index)
              << "," << tracked.active_update_count << ","
              << active_age_sec << ","
              << (tracked.factor_index ? std::to_string(*tracked.factor_index)
                                       : "none")
              << ","
              << (graph_factor_index ? std::to_string(*graph_factor_index)
                                     : "none")
              << "," << category << "," << (is_external ? 1 : 0) << ","
              << (has_from ? 1 : 0) << "," << (has_to ? 1 : 0) << ","
              << factor.keys().size() << "," << factorKeysToken(factor) << ","
              << demangledTypeName(typeid(factor)) << "," << error << ","
              << hessian_frobenius);
        };

    if (tracked.factor) {
      maybe_log_detail(
          *tracked.factor,
          "cbs_active",
          tracked.factor_index,
          true,
          true,
          true,
          cbs_error,
          cbs_hessian_frobenius);
    }

    for (gtsam::FactorIndex factor_index = 0; factor_index < factors.size();
         ++factor_index) {
      const auto& factor = factors[factor_index];
      if (!factor) {
        continue;
      }
      const bool has_from = factorTouchesKey(*factor, tracked.local_from_key);
      const bool has_to = factorTouchesKey(*factor, tracked.local_to_key);
      if (!has_from && !has_to) {
        continue;
      }
      if (tracked.factor && factor.get() == tracked.factor.get()) {
        continue;
      }
      const double error = factorErrorOrNan(*factor, estimate);
      const double hessian_frobenius = factorPoseHessianFrobeniusOrNan(
          *factor, estimate, tracked.local_from_key, tracked.local_to_key);
      if (external_factor_ptrs.find(factor.get()) !=
          external_factor_ptrs.end()) {
        other_external.add(error, hessian_frobenius);
        maybe_log_detail(*factor,
                         "other_external",
                         factor_index,
                         true,
                         has_from,
                         has_to,
                         error,
                         hessian_frobenius);
      } else if (has_from && has_to) {
        same_edge.add(error, hessian_frobenius);
        maybe_log_detail(*factor,
                         "same_edge",
                         factor_index,
                         false,
                         has_from,
                         has_to,
                         error,
                         hessian_frobenius);
      } else if (factor->keys().size() == 1u) {
        unary.add(error, hessian_frobenius);
        maybe_log_detail(*factor,
                         "unary",
                         factor_index,
                         false,
                         has_from,
                         has_to,
                         error,
                         hessian_frobenius);
      } else {
        other_local.add(error, hessian_frobenius);
        maybe_log_detail(*factor,
                         "other_local",
                         factor_index,
                         false,
                         has_from,
                         has_to,
                         error,
                         hessian_frobenius);
      }
    }

    ROS_INFO_STREAM(
        "GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW,"
        << directionToken(tracked.source_agent, self_agent_id_) << ","
        << agentToken(self_agent_id_) << "," << agentToken(tracked.source_agent)
        << "," << poseKeyToken(tracked.local_from_key) << ","
        << poseKeyToken(tracked.local_to_key) << ","
        << poseToken(tracked.source_agent, tracked.source_from_index) << "->"
        << poseToken(tracked.source_agent, tracked.source_to_index) << ","
        << (tracked.factor_index ? std::to_string(*tracked.factor_index)
                                 : "none")
        << "," << tracked.active_update_count << "," << active_age_sec << ","
        << tracked.from_abs_dt << "," << tracked.to_abs_dt << ","
        << tracked.covariance.trace() << ","
        << tracked.covariance.diagonal().segment(0, 3).sum() << ","
        << tracked.covariance.diagonal().segment(3, 3).sum() << ","
        << cbs_error << "," << residual_norm << "," << rot_norm << ","
        << trans_norm << "," << whitened_norm << "," << nis << ","
        << cbs_hessian_frobenius << "," << same_edge.count << ","
        << same_edge.error_count << "," << same_edge.error_sum << ","
        << same_edge.hessian_count << ","
        << same_edge.hessian_frobenius_sum << "," << unary.count << ","
        << unary.error_count << "," << unary.error_sum << ","
        << unary.hessian_count << "," << unary.hessian_frobenius_sum << ","
        << other_local.count << "," << other_local.error_count << ","
        << other_local.error_sum << "," << other_local.hessian_count << ","
        << other_local.hessian_frobenius_sum << ","
        << other_external.count << "," << other_external.error_count << ","
        << other_external.error_sum << ","
        << other_external.hessian_count << ","
        << other_external.hessian_frobenius_sum);
  }
}

bool GlimCbsBridge::buildExternalFactor(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    gtsam::NonlinearFactor::shared_ptr* factor,
    double* covariance_trace,
    const std::string& action,
    gtsam::Pose3* measured_out,
    gtsam::Matrix6* covariance_out) {
  if (!factor) {
    return false;
  }
  try {
    gtsam::Matrix6 covariance = matrix6FromArray(matched.incoming.covariance);
    gtsam::Pose3 measured =
        gtsam::Pose3::Expmap(vector6FromArray(matched.incoming.relative_mu));
    if (experimental_native_imu_mean_alignment_) {
      const double from_stamp = pose_timestamps_sec_.at(matched.local_from_index);
      const double to_stamp = pose_timestamps_sec_.at(matched.local_to_index);
      // Scan poses are stamped at scan START, whereas native GLIM has already
      // received IMU through the scan. A camera endpoint may therefore be later
      // than the newest scan pose and still be causally available. Bound the
      // nominal bridge by RECEIVED IMU, never by a future optimized pose or by
      // an extrapolation of the last sample. between() also verifies coverage.
      const double causal_limit = native_imu_mean_bridge_->availableUntil();
      auto bridge = [&](double from, double to) -> gtsam::Pose3 {
        if (from == to) return gtsam::Pose3();
        const double first = std::min(from, to);
        const auto& prior = smoother.getLinearizationPoint();
        for (size_t i = pose_timestamps_sec_.size(); i-- > 0;) {
          const double stamp = pose_timestamps_sec_[i];
          if (!std::isfinite(stamp) || stamp > first) continue;
          const gtsam::Symbol x('x', i), v('v', i), b('b', i);
          if (!prior.exists(x) || !prior.exists(v) || !prior.exists(b)) continue;
          return native_imu_mean_bridge_->between(stamp,
              gtsam::NavState(smoother.calculateEstimate<gtsam::Pose3>(x),
                             smoother.calculateEstimate<gtsam::Vector3>(v)),
              smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(b),
              from, to, causal_limit);
        }
        throw std::runtime_error("GLIM mean bridge: no active native anchor");
      };
      const auto left = bridge(from_stamp, matched.incoming.from_stamp_sec);
      const auto right = bridge(matched.incoming.to_stamp_sec, to_stamp);
      const auto raw = measured;
      measured = left.compose(raw).compose(right);
      ROS_INFO_STREAM(std::setprecision(17) << "GLIM_CBS_MEAN_ALIGNMENT_ROW,"
          << matched.incoming.from_pose_index << "," << matched.incoming.to_pose_index
          << "," << matched.local_from_index << "," << matched.local_to_index
          << "," << matched.incoming.from_stamp_sec << ","
          << matched.incoming.to_stamp_sec << "," << from_stamp << ","
          << to_stamp << "," << causal_limit << ","
          << cbs::relative_pose_covariance::matrixToken(gtsam::Pose3::Logmap(raw))
          << "," << cbs::relative_pose_covariance::matrixToken(gtsam::Pose3::Logmap(measured))
          << "," << cbs::relative_pose_covariance::matrixToken(covariance));
    }
    if (health_receiver_params_.enable &&
        health_receiver_params_.receiver_nis_enable) {
      cbs::health_aware::NisResult nis_result;
      nis_result.enabled = true;
      nis_result.status = "no_receiver_pose";
      nis_result.final_covariance = cbs::health_aware::symmetrize6(covariance);
      nis_result.sender_trace = nis_result.final_covariance.trace();
      nis_result.final_trace = nis_result.sender_trace;
      const bool from_in_new_values = new_values.exists(matched.local_from_key);
      const bool to_in_new_values = new_values.exists(matched.local_to_key);
      const auto& linearization_point = smoother.getLinearizationPoint();
      const bool from_in_linearization =
          linearization_point.exists(matched.local_from_key);
      const bool to_in_linearization =
          linearization_point.exists(matched.local_to_key);
      bool joint_info_attempted = false;
      bool joint_info_ok = false;
      bool conditional_covariance_ok = false;
      std::string receiver_covariance_status = "not_attempted";
      try {
        const gtsam::Pose3 receiver_from =
            lookupPose(matched.local_from_key, &smoother, &new_values);
        const gtsam::Pose3 receiver_to =
            lookupPose(matched.local_to_key, &smoother, &new_values);
        const gtsam::Pose3 receiver_from_to = receiver_from.between(receiver_to);
        std::optional<gtsam::Matrix> receiver_relative_covariance;
        if (from_in_linearization && to_in_linearization) {
          joint_info_attempted = true;
          try {
            const gtsam::JointMarginal joint_info =
                smoother.jointMarginalInformation(
                    gtsam::KeyVector{matched.local_from_key,
                                     matched.local_to_key});
            joint_info_ok = true;
            const auto conditional_covariance =
                cbs::health_aware::conditionalCovarianceFromJointInformation(
                    joint_info, matched.local_to_key);
            if (conditional_covariance) {
              receiver_relative_covariance = *conditional_covariance;
              conditional_covariance_ok = true;
              receiver_covariance_status = "ok";
            } else {
              receiver_covariance_status = "conditional_cov_invalid";
            }
          } catch (const std::exception& e) {
            receiver_covariance_status = "joint_marginal_exception";
            ROS_DEBUG_STREAM("GLIM CBS receiver covariance unavailable for "
                             "health-aware NIS: "
                             << e.what());
          } catch (...) {
            receiver_covariance_status = "joint_marginal_exception";
          }
        } else if (!from_in_linearization && !to_in_linearization) {
          receiver_covariance_status = "missing_both_linearization";
        } else if (!from_in_linearization) {
          receiver_covariance_status = "missing_from_linearization";
        } else {
          receiver_covariance_status = "missing_to_linearization";
        }
        const gtsam::Vector6 residual =
            cbs::health_aware::computeRelativePoseResidual(measured,
                                                           receiver_from_to);
        nis_result = cbs::health_aware::computeNisInflation(
            residual,
            covariance,
            receiver_relative_covariance,
            health_receiver_params_,
            &relative_trust_states_by_source_[matched.incoming.source_agent]);
        covariance = nis_result.final_covariance;
        logReceiverCovarianceRow(self_agent_id_,
                                 matched.incoming.source_agent,
                                 matched.local_from_key,
                                 matched.local_to_key,
                                 from_in_new_values,
                                 to_in_new_values,
                                 from_in_linearization,
                                 to_in_linearization,
                                 joint_info_attempted,
                                 joint_info_ok,
                                 conditional_covariance_ok,
                                 receiver_covariance_status);
      } catch (const std::exception& e) {
        logReceiverCovarianceRow(self_agent_id_,
                                 matched.incoming.source_agent,
                                 matched.local_from_key,
                                 matched.local_to_key,
                                 from_in_new_values,
                                 to_in_new_values,
                                 from_in_linearization,
                                 to_in_linearization,
                                 joint_info_attempted,
                                 joint_info_ok,
                                 conditional_covariance_ok,
                                 "pose_lookup_exception");
        ROS_DEBUG_STREAM("GLIM CBS receiver pose unavailable for "
                         "health-aware NIS: "
                         << e.what());
        nis_result.status = "no_receiver_pose";
      }
      last_receiver_nis_nu_ = nis_result.nu;
      last_receiver_nis_alpha_ = nis_result.alpha_cons;
      logHealthAwareNisRow(self_agent_id_,
                           matched.incoming.source_agent,
                           matched.local_from_key,
                           matched.local_to_key,
                           nis_result);
    }
    covariance *= covariance_scale_;

    logOdomFactorCovarianceRow(self_agent_id_,
                               matched.incoming.source_agent,
                               matched.local_from_key,
                               matched.local_to_key,
                               covariance,
                               action);
    try {
      const gtsam::Pose3 receiver_from =
          lookupPose(matched.local_from_key, &smoother, &new_values);
      const gtsam::Pose3 receiver_to =
          lookupPose(matched.local_to_key, &smoother, &new_values);
      logOdomPreinjectionResidualRow(self_agent_id_,
                                     matched.incoming.source_agent,
                                     matched.local_from_key,
                                     matched.local_to_key,
                                     measured,
                                     covariance,
                                     receiver_from,
                                     receiver_to,
                                     "pre_graph_update",
                                     action);
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("GLIM CBS pre-injection residual unavailable: "
                       << e.what());
    }

    const gtsam::SharedNoiseModel noise =
        gtsam::noiseModel::Gaussian::Covariance(covariance);
    *factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        matched.local_from_key, matched.local_to_key, measured, noise);
    if (covariance_trace) {
      *covariance_trace = covariance.trace();
    }
    if (measured_out) {
      *measured_out = measured;
    }
    if (covariance_out) {
      *covariance_out = covariance;
    }
    return true;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("GLIM CBS failed to build external odometry factor: "
                    << e.what());
    return false;
  }
}

bool GlimCbsBridge::appendExternalFactor(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    gtsam::NonlinearFactorGraph* new_factors,
    gtsam::FactorIndices* factors_to_remove) {
  if (!new_factors || !factors_to_remove) {
    return false;
  }
  gtsam::NonlinearFactor::shared_ptr factor;
  double covariance_trace = std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 measured_from_to;
  gtsam::Matrix6 inserted_covariance = gtsam::Matrix6::Identity();
  if (!buildExternalFactor(
          matched,
          smoother,
          new_values,
          &factor,
          &covariance_trace,
          "persistent_odom_applied",
          &measured_from_to,
          &inserted_covariance) ||
      !factor) {
    return false;
  }
  ROS_INFO_STREAM(
      "GLIM_CBS_SOURCE_TRANSPORT_ROW,"
      << static_cast<char>(matched.incoming.source_agent) << ","
      << matched.incoming.from_pose_index << ","
      << matched.incoming.to_pose_index << ","
      << matched.incoming.revision << ","
      << cbs::relative_pose_covariance::matrixToken(
             matrix6FromArray(matched.incoming.covariance)) << ","
      << cbs::relative_pose_covariance::matrixToken(inserted_covariance));
  try {
    logReceiverLocalCompetition(
        matched, smoother, new_values, *new_factors, inserted_covariance,
        factor);
    if (live_revision_enable_) {
      const SourceBeliefId id{
          static_cast<int>(matched.incoming.source_agent),
          matched.incoming.from_pose_index,
          matched.incoming.to_pose_index};
      const auto previous = live_revision_factors_.find(id);
      if (previous != live_revision_factors_.end()) {
        const auto& graph = smoother.getFactors();
        bool found = false;
        for (gtsam::FactorIndex i = 0; i < graph.size(); ++i) {
          if (graph[i] == previous->second.factor) {
            factors_to_remove->push_back(i);
            found = true;
            break;
          }
        }
        ROS_INFO_STREAM("GLIM_CBS_LIVE_REVISION_ROW,replacement,"
                        << matched.incoming.from_pose_index << ","
                        << matched.incoming.to_pose_index << ","
                        << previous->second.revision << ","
                        << matched.incoming.revision << ","
                        << (found ? "old_factor_removed" : "old_factor_retired"));
      } else {
        ROS_INFO_STREAM("GLIM_CBS_LIVE_REVISION_ROW,first_insert,"
                        << matched.incoming.from_pose_index << ","
                        << matched.incoming.to_pose_index << ","
                        << matched.incoming.revision);
      }
      live_revision_factors_.insert_or_assign(
          id, LiveRevisionFactor{matched.incoming.revision, factor});
    }
    new_factors->push_back(factor);
    if (receiver_local_competition_audit_max_samples_ > 0u &&
        receiver_local_competition_audit_count_ <
            receiver_local_competition_audit_max_samples_) {
      persistent_external_factors_.push_back(factor);
    }
    capturePoseMergeDiagnostic(
        matched, smoother, new_values, measured_from_to);
    ROS_INFO_STREAM(
        "GLIM_CBS_ODOM_INJECT_ROW,"
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.from_pose_index)
        << "->"
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.to_pose_index)
        << "," << poseToken(self_agent_id_, matched.local_from_index) << "->"
        << poseToken(self_agent_id_, matched.local_to_index) << ","
        << matched.from_abs_dt << "," << matched.to_abs_dt << ","
        << covariance_trace << ",injected_persistent");
    return true;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("GLIM CBS failed to append external odometry factor: "
                    << e.what());
    return false;
  }
}

void GlimCbsBridge::logReceiverLocalCompetition(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    const gtsam::NonlinearFactorGraph& pending_factors,
    const gtsam::Matrix6& inserted_covariance,
    const gtsam::NonlinearFactor::shared_ptr& incoming_factor) {
  if (receiver_local_competition_audit_max_samples_ == 0u ||
      receiver_local_competition_audit_count_ >=
          receiver_local_competition_audit_max_samples_) {
    return;
  }

  const EdgeId edge_id = edgeId(matched);
  if (!receiver_local_competition_audit_seen_edges_.insert(edge_id).second) {
    return;
  }
  const size_t seen_index = receiver_local_competition_audit_seen_count_++;
  if (seen_index % std::max<size_t>(
                       1u, receiver_local_competition_audit_stride_) !=
      0u) {
    return;
  }

  const size_t sample_index = receiver_local_competition_audit_count_;
  try {
    gtsam::Values audit_values = smoother.calculateEstimate();
    for (const auto& key_value : new_values) {
      if (!audit_values.exists(key_value.key)) {
        audit_values.insert(key_value.key, key_value.value);
      }
    }

    // Passive, capped diagnostic: evaluate the ACTUAL incoming factor and all
    // local quantities at this one Values. Never insert it into local_graph.
    const auto* incoming = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(
        incoming_factor.get());
    if (!incoming) {
      throw std::runtime_error("incoming audit factor is not Pose3 BetweenFactor");
    }
    const auto emit_matrix = [&](const char* label, const gtsam::Matrix& value) {
      ROS_INFO_STREAM("CBS_RECEIVER_INCOMING_MATRIX_ROW,"
          << directionToken(matched.incoming.source_agent, self_agent_id_)
          << "," << poseKeyToken(matched.local_from_key) << ","
          << poseKeyToken(matched.local_to_key) << "," << sample_index << ","
          << label << "," << value.rows() << "," << value.cols() << ","
          << cbs::relative_pose_covariance::matrixToken(value));
    };
    const auto from = audit_values.at<gtsam::Pose3>(matched.local_from_key);
    const auto to = audit_values.at<gtsam::Pose3>(matched.local_to_key);
    gtsam::Matrix H_from, H_to;
    const gtsam::Vector residual = incoming->evaluateError(from, to, H_from, H_to);
    const gtsam::Vector whitened = incoming->whitenedError(audit_values);
    std::vector<gtsam::Matrix> jacobians{H_from, H_to};
    gtsam::Vector whitened_system = residual;
    incoming->noiseModel()->WhitenSystem(jacobians, whitened_system);
    gtsam::Matrix J(6, 12);
    J << jacobians[0], jacobians[1];
    gtsam::GaussianFactorGraph linear;
    linear.push_back(incoming->linearize(audit_values));
    gtsam::Ordering order;
    order.push_back(matched.local_from_key);
    order.push_back(matched.local_to_key);
    const auto [hessian, eta] = linear.hessian(order);
    emit_matrix("from_pose", from.matrix());
    emit_matrix("to_pose", to.matrix());
    emit_matrix("measurement", incoming->measured().matrix());
    emit_matrix("covariance", inserted_covariance);
    emit_matrix("residual", residual);
    emit_matrix("whitened_residual", whitened);
    emit_matrix("whitened_system_residual", whitened_system);
    emit_matrix("H_from", H_from);
    emit_matrix("H_to", H_to);
    emit_matrix("J_whitened", J);
    emit_matrix("hessian", hessian);
    emit_matrix("gradient", -eta);
    gtsam::Matrix cost(1, 1);
    cost(0, 0) = incoming->error(audit_values);
    emit_matrix("cost", cost);

    std::unordered_set<const gtsam::NonlinearFactor*> external_factors;
    external_factors.reserve(persistent_external_factors_.size() +
                             active_window_external_factors_.size());
    for (const auto& factor : persistent_external_factors_) {
      if (factor) {
        external_factors.insert(factor.get());
      }
    }
    for (const auto& [receiver_edge, tracked] :
         active_window_external_factors_) {
      (void)receiver_edge;
      if (tracked.factor) {
        external_factors.insert(tracked.factor.get());
      }
    }

    struct LocalFactorGroup {
      size_t count = 0u;
      size_t touching_count = 0u;
      double error_sum = 0.0;
      size_t error_count = 0u;
      double touching_hessian_frobenius_sum = 0.0;
      size_t touching_hessian_count = 0u;
    };
    using LocalFactorGroupKey = std::pair<std::string, std::string>;
    std::map<LocalFactorGroupKey, LocalFactorGroup> groups;
    gtsam::NonlinearFactorGraph local_graph;
    const auto append_local_factors =
        [&](const gtsam::NonlinearFactorGraph& graph) {
          for (const auto& factor : graph) {
            if (!factor || external_factors.count(factor.get()) > 0u ||
                !valuesContainFactorKeys(audit_values, *factor)) {
              continue;
            }
            local_graph.push_back(factor);
            const std::string category = receiverLocalFactorCategory(*factor);
            const std::string exact_type = demangledTypeName(typeid(*factor));
            LocalFactorGroup& group = groups[{category, exact_type}];
            ++group.count;
            const double error = factorErrorOrNan(*factor, audit_values);
            if (std::isfinite(error)) {
              ++group.error_count;
              group.error_sum += error;
            }
            const bool touches_from =
                factorTouchesKey(*factor, matched.local_from_key);
            const bool touches_to =
                factorTouchesKey(*factor, matched.local_to_key);
            if (touches_from || touches_to) {
              ++group.touching_count;
              const double hessian_frobenius =
                  factorPoseHessianFrobeniusOrNan(*factor,
                                                  audit_values,
                                                  matched.local_from_key,
                                                  matched.local_to_key);
              if (std::isfinite(hessian_frobenius)) {
                ++group.touching_hessian_count;
                group.touching_hessian_frobenius_sum += hessian_frobenius;
              }
              ROS_INFO_STREAM(
                  "CBS_RECEIVER_LOCAL_FACTOR_ROW,"
                  << directionToken(matched.incoming.source_agent,
                                    self_agent_id_)
                  << "," << agentToken(self_agent_id_) << ","
                  << agentToken(matched.incoming.source_agent) << ","
                  << poseToken(matched.incoming.source_agent,
                               matched.incoming.from_pose_index)
                  << ","
                  << poseToken(matched.incoming.source_agent,
                               matched.incoming.to_pose_index)
                  << "," << poseKeyToken(matched.local_from_key) << ","
                  << poseKeyToken(matched.local_to_key) << ","
                  << sample_index << "," << category << ","
                  << exact_type << "," << factorKeysToken(*factor) << ","
                  << (touches_from ? 1 : 0) << ","
                  << (touches_to ? 1 : 0) << "," << error << ","
                  << hessian_frobenius);
            }
          }
        };
    append_local_factors(smoother.getFactors());
    append_local_factors(pending_factors);

    if (!audit_values.exists(matched.local_from_key) ||
        !audit_values.exists(matched.local_to_key)) {
      throw std::runtime_error("receiver endpoint value unavailable");
    }
    if (local_graph.empty()) {
      throw std::runtime_error("receiver local graph is empty");
    }

    gtsam::Marginals marginals(
        local_graph, audit_values, gtsam::Marginals::CHOLESKY);
    const gtsam::JointMarginal joint = marginals.jointMarginalInformation(
        gtsam::KeyVector{matched.local_from_key, matched.local_to_key});
    const gtsam::Pose3 receiver_from =
        audit_values.at<gtsam::Pose3>(matched.local_from_key);
    const gtsam::Pose3 receiver_to =
        audit_values.at<gtsam::Pose3>(matched.local_to_key);
    const gtsam::Pose3 receiver_from_to = receiver_from.between(receiver_to);
    const auto local = cbs::relative_pose_covariance::compute(
        joint,
        matched.local_from_key,
        matched.local_to_key,
        receiver_from,
        receiver_to,
        receiver_from_to,
        "schur_relative_between");

    const gtsam::Matrix6 inserted_symmetric =
        0.5 * (inserted_covariance + inserted_covariance.transpose());
    Eigen::LDLT<gtsam::Matrix6> inserted_ldlt(inserted_symmetric);
    if (inserted_ldlt.info() != Eigen::Success ||
        !inserted_ldlt.isPositive()) {
      throw std::runtime_error("inserted covariance is not SPD");
    }
    const gtsam::Matrix6 inserted_information = inserted_ldlt.solve(
        gtsam::Matrix6::Identity());
    if (!inserted_information.allFinite()) {
      throw std::runtime_error("inserted information solve failed");
    }

    const gtsam::Matrix6 local_information =
        0.5 * (local.schur_relative_information +
               local.schur_relative_information.transpose());
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> local_eigensolver(
        local_information);
    if (local_eigensolver.info() != Eigen::Success ||
        local_eigensolver.eigenvalues().minCoeff() <= 0.0) {
      throw std::runtime_error("receiver local information is not SPD");
    }
    const gtsam::Matrix6 local_inverse_root =
        local_eigensolver.eigenvectors() *
        local_eigensolver.eigenvalues()
            .cwiseInverse()
            .cwiseSqrt()
            .asDiagonal() *
        local_eigensolver.eigenvectors().transpose();
    const gtsam::Matrix6 comparison =
        local_inverse_root * inserted_information * local_inverse_root;
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> comparison_eigensolver(
        0.5 * (comparison + comparison.transpose()));
    if (comparison_eigensolver.info() != Eigen::Success) {
      throw std::runtime_error("generalized information eigensolve failed");
    }
    const double trace_ratio = inserted_information.trace() /
                               local_information.trace();

    ROS_INFO_STREAM(
        "CBS_RECEIVER_LOCAL_COMPETITION_ROW,"
        << directionToken(matched.incoming.source_agent, self_agent_id_) << ","
        << agentToken(self_agent_id_) << ","
        << agentToken(matched.incoming.source_agent) << ","
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.from_pose_index)
        << ","
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.to_pose_index)
        << "," << poseKeyToken(matched.local_from_key) << ","
        << poseKeyToken(matched.local_to_key) << "," << sample_index << ","
        << seen_index << "," << local_graph.size() << ","
        << inserted_information.trace() << "," << local_information.trace()
        << "," << trace_ratio << ","
        << cbs::relative_pose_covariance::eigenvaluesToken(
               comparison_eigensolver.eigenvalues())
        << ","
        << cbs::relative_pose_covariance::matrixToken(inserted_covariance)
        << ","
        << cbs::relative_pose_covariance::matrixToken(inserted_information)
        << ","
        << cbs::relative_pose_covariance::matrixToken(
               local.schur_relative_covariance)
        << ","
        << cbs::relative_pose_covariance::matrixToken(local_information)
        << ",ok");

    for (const auto& [group_key, group] : groups) {
      ROS_INFO_STREAM(
          "CBS_RECEIVER_LOCAL_FACTOR_GROUP_ROW,"
          << directionToken(matched.incoming.source_agent, self_agent_id_)
          << "," << agentToken(self_agent_id_) << ","
          << agentToken(matched.incoming.source_agent) << ","
          << poseToken(matched.incoming.source_agent,
                       matched.incoming.from_pose_index)
          << ","
          << poseToken(matched.incoming.source_agent,
                       matched.incoming.to_pose_index)
          << "," << poseKeyToken(matched.local_from_key) << ","
          << poseKeyToken(matched.local_to_key) << "," << sample_index << ","
          << group_key.first << "," << group_key.second << ","
          << group.count << "," << group.touching_count << ","
          << group.error_count << "," << group.error_sum << ","
          << group.touching_hessian_count << ","
          << group.touching_hessian_frobenius_sum);
    }
    ++receiver_local_competition_audit_count_;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM(
        "CBS_RECEIVER_LOCAL_COMPETITION_STATUS_ROW,"
        << directionToken(matched.incoming.source_agent, self_agent_id_) << ","
        << agentToken(self_agent_id_) << ","
        << agentToken(matched.incoming.source_agent) << ","
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.from_pose_index)
        << ","
        << poseToken(matched.incoming.source_agent,
                     matched.incoming.to_pose_index)
        << "," << poseKeyToken(matched.local_from_key) << ","
        << poseKeyToken(matched.local_to_key) << "," << sample_index << ","
        << seen_index << ",unavailable," << csvSafeToken(e.what()));
  }
}

bool GlimCbsBridge::appendActiveWindowTemporaryFactor(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    gtsam::NonlinearFactorGraph* new_factors,
    gtsam::FactorIndices& factors_to_remove) {
  (void)factors_to_remove;
  if (!new_factors) {
    return false;
  }
  gtsam::NonlinearFactor::shared_ptr factor;
  double covariance_trace = std::numeric_limits<double>::quiet_NaN();
  gtsam::Pose3 measured_from_to;
  gtsam::Matrix6 covariance = gtsam::Matrix6::Identity();
  if (!buildExternalFactor(
          matched,
          smoother,
          new_values,
          &factor,
          &covariance_trace,
          "active_window_temporary_graph_queued",
          &measured_from_to,
          &covariance) ||
      !factor) {
    return false;
  }
  const ReceiverEdgeId receiver_edge_id = receiverEdgeId(matched);
  auto active_it = active_window_external_factors_.find(receiver_edge_id);
  if (active_it != active_window_external_factors_.end()) {
    ROS_INFO_STREAM(
        "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,"
        "skipped_receiver_edge_already_active,"
        << poseKeyToken(matched.local_from_key) << ","
        << poseKeyToken(matched.local_to_key));
    return false;
  }

  // Same no-op-by-default diagnostic as the persistent path, before insertion.
  logReceiverLocalCompetition(
      matched, smoother, new_values, *new_factors, covariance, factor);
  const size_t new_factor_index = new_factors->size();
  new_factors->push_back(factor);
  capturePoseMergeDiagnostic(
      matched, smoother, new_values, measured_from_to);

  ActiveWindowExternalFactor tracked;
  tracked.edge_id = edgeId(matched);
  tracked.receiver_edge_id = receiver_edge_id;
  tracked.source_agent = matched.incoming.source_agent;
  tracked.source_from_index = matched.incoming.from_pose_index;
  tracked.source_to_index = matched.incoming.to_pose_index;
  tracked.local_from_key = matched.local_from_key;
  tracked.local_to_key = matched.local_to_key;
  tracked.from_abs_dt = matched.from_abs_dt;
  tracked.to_abs_dt = matched.to_abs_dt;
  tracked.inserted_stamp_sec = latestPoseStamp();
  const size_t local_to_index = poseIndex(matched.local_to_key);
  if (local_to_index < pose_timestamps_sec_.size() &&
      std::isfinite(pose_timestamps_sec_[local_to_index])) {
    tracked.inserted_stamp_sec = pose_timestamps_sec_[local_to_index];
  }
  tracked.measured_from_to = measured_from_to;
  tracked.covariance = covariance;
  tracked.factor = factor;
  pending_active_window_external_factors_.push_back(
      PendingActiveWindowExternalFactor{tracked, new_factor_index});
  ROS_INFO_STREAM(
      "GLIM_CBS_ODOM_INJECT_ROW,"
      << poseToken(matched.incoming.source_agent,
                   matched.incoming.from_pose_index)
      << "->"
      << poseToken(matched.incoming.source_agent,
                   matched.incoming.to_pose_index)
      << "," << poseToken(self_agent_id_, matched.local_from_index) << "->"
      << poseToken(self_agent_id_, matched.local_to_index) << ","
      << matched.from_abs_dt << "," << matched.to_abs_dt << ","
      << covariance_trace << ",active_window_temporary_graph_queued");
  ROS_INFO_STREAM("GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,inserted,"
                  << poseKeyToken(tracked.local_from_key) << ","
                  << poseKeyToken(tracked.local_to_key) << ",total_active="
                  << active_window_external_factors_.size() +
                         pending_active_window_external_factors_.size());
  return true;
}

bool GlimCbsBridge::queueTemporaryFactor(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values) {
  gtsam::NonlinearFactor::shared_ptr factor;
  double covariance_trace = std::numeric_limits<double>::quiet_NaN();
  if (!buildExternalFactor(
          matched,
          smoother,
          new_values,
          &factor,
          &covariance_trace,
          "temporary_linear_queued") ||
      !factor) {
    return false;
  }
  pending_temporary_factors_.push_back(factor);
  ROS_INFO_STREAM(
      "GLIM_CBS_ODOM_INJECT_ROW,"
      << poseToken(matched.incoming.source_agent,
                   matched.incoming.from_pose_index)
      << "->"
      << poseToken(matched.incoming.source_agent,
                   matched.incoming.to_pose_index)
      << "," << poseToken(self_agent_id_, matched.local_from_index) << "->"
      << poseToken(self_agent_id_, matched.local_to_index) << ","
      << matched.from_abs_dt << "," << matched.to_abs_dt << ","
      << covariance_trace << ",temporary_linear_queued");
  return true;
}

size_t GlimCbsBridge::applyTemporaryFactors(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (mode_ == Mode::ActiveWindowTemporary) {
    (void)smoother;
    return 0u;
  }

  if (mode_ != Mode::TemporaryLinear || pending_temporary_factors_.empty()) {
    return 0u;
  }
  const size_t count = pending_temporary_factors_.size();
  try {
    smoother.updateWithTemporaryFactors(pending_temporary_factors_);
    ROS_INFO_STREAM("GLIM CBS applied " << count
                    << " temporary linear odometry factors");
    pending_temporary_factors_.resize(0);
    return count;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("GLIM CBS temporary linear factor update failed: "
                    << e.what());
    pending_temporary_factors_.resize(0);
    rejected_total_.fetch_add(count, std::memory_order_relaxed);
    return 0u;
  }
}

void GlimCbsBridge::updateSidecar(
    const gtsam::NonlinearFactorGraph& new_factors,
    const gtsam::Values& new_values,
    const std::map<std::uint64_t, double>& new_stamps) {
  if (!sidecar_) {
    return;
  }
  try {
    sidecar_->update(new_factors, new_values, new_stamps);
  } catch (const std::exception& e) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "GLIM CBS sidecar smoother update failed: " << e.what());
  }
}

void GlimCbsBridge::publishOutgoingBeliefs(
    const ros::Time& stamp,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (!publish_beliefs_ || !belief_pub_ ||
      belief_pub_.getNumSubscribers() == 0 ||
      pose_timestamps_sec_.size() < 2u) {
    return;
  }
  const size_t publish_count = outgoing_publish_count_++;
  if (publish_count % outgoing_publish_stride_ != 0u) {
    return;
  }
  if (outgoing_marginal_source_ == OutgoingMarginalSource::SidecarBatch &&
      !sidecar_) {
    return;
  }

  const bool timing = timing_enabled();
  const auto total_start = TimingClock::now();
  const auto request_start = TimingClock::now();
  gtsam::KeySet request_keys;
  const auto value_exists = [&](gtsam::Key key) {
    if (outgoing_marginal_source_ == OutgoingMarginalSource::SidecarBatch) {
      return sidecar_ && sidecar_->valueExists(key);
    }
    return smoother.getLinearizationPoint().exists(key);
  };
  const size_t last_index = pose_timestamps_sec_.size() - 1u;
  const size_t first_index =
      last_index > static_cast<size_t>(belief_exchange_window_size_)
          ? last_index - static_cast<size_t>(belief_exchange_window_size_)
          : 0u;
  std::vector<gtsam::Key> available_pose_keys;
  for (size_t i = first_index; i <= last_index; ++i) {
    if (!std::isfinite(pose_timestamps_sec_[i]) ||
        pose_timestamps_sec_[i] <= 0.0) {
      continue;
    }
    const gtsam::Key key = poseKey(i);
    if (sender_mode_ == "adjacent_window") {
      request_keys.insert(key);
    }
    if (value_exists(key)) {
      available_pose_keys.push_back(key);
    }
  }

  bool latest_pair_valid = false;
  gtsam::Key latest_from_key = 0u;
  gtsam::Key latest_to_key = 0u;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> explicit_pairs;
  if (sender_mode_ == "latest_edge" || sender_mode_ == "new_edge_once") {
    if (available_pose_keys.size() < 2u) {
      return;
    }
    latest_from_key = available_pose_keys[available_pose_keys.size() - 2u];
    latest_to_key = available_pose_keys.back();
    latest_pair_valid = true;
    if (sender_mode_ == "new_edge_once" && last_outgoing_pair_valid_ &&
        latest_from_key == last_outgoing_from_key_ &&
        latest_to_key == last_outgoing_to_key_) {
      return;
    }
    request_keys.insert(latest_from_key);
    request_keys.insert(latest_to_key);
  } else if (sender_mode_ == "time_horizon_window") {
    explicit_pairs = buildTimeHorizonPairs(available_pose_keys);
  }
  const double request_build_ms =
      timing ? timing_elapsed_ms(request_start) : 0.0;
  if (request_keys.size() < 2u && explicit_pairs.empty()) {
    return;
  }

  std::vector<cbs::BPSAM::CbsOdometryBelief> outgoing;
  double set_marginalization_graph_ms = 0.0;
  double get_odometry_beliefs_ms = 0.0;
  try {
    const cbs::IncrementalFixedLagBpsamSmoother* local_covariance_smoother =
        nullptr;
    if (mode_ == Mode::ActiveWindowTemporary && sidecar_) {
      const auto set_marginalization_graph_start = TimingClock::now();
      const size_t e1_sample=e1_real::selectedCall();
      if (e1_sample) {
        sidecar_->setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL,
          [&](const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& values) {
            std::vector<e1_real::Pair> pairs;
            for(const auto& p:explicit_pairs) pairs.push_back({p.first,p.second,
              smoother.calculateEstimate<gtsam::Pose3>(p.first),
              smoother.calculateEstimate<gtsam::Pose3>(p.second)});
            auto linear=e1_real::compare(e1_sample,stamp.toSec(),graph,values,pairs);
            for(const auto& p:pairs) {
              if(!p.pa.equals(smoother.calculateEstimate<gtsam::Pose3>(p.a),0) ||
                 !p.pb.equals(smoother.calculateEstimate<gtsam::Pose3>(p.b),0)) {
                std::ofstream(std::string(std::getenv("GLIM_E1_REAL_AUDIT_ROOT"))+"/FAILED.txt") << "main means changed";
                throw std::runtime_error("E1 main means changed");
              }
            }
            std::ofstream(std::string(std::getenv("GLIM_E1_REAL_AUDIT_ROOT"))+"/sample_"+
                          std::to_string(e1_sample)+"/means_unchanged.marker") << "exact main endpoint poses preserved\n";
            return linear;
          });
      } else if (experimental_local_gpu_batching_) {
        sidecar_->setMarginalizationGraph(
            cbs::BPSAM::MarginalizationType::LOCAL,
            linearizeCbsLocalGraphBatched);
      } else {
        sidecar_->setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      }
      set_marginalization_graph_ms =
          timing ? timing_elapsed_ms(set_marginalization_graph_start) : 0.0;
      local_covariance_smoother = sidecar_.get();
    }
    if (!explicit_pairs.empty()) {
      const auto get_odometry_beliefs_start = TimingClock::now();
      outgoing = getOdometryBeliefsDirectForPairs(
          explicit_pairs, smoother, local_covariance_smoother);
      get_odometry_beliefs_ms =
          timing ? timing_elapsed_ms(get_odometry_beliefs_start) : 0.0;
    } else if (outgoing_marginal_source_ == OutgoingMarginalSource::SidecarBatch &&
               mode_ != Mode::ActiveWindowTemporary) {
      if (!local_covariance_smoother) {
        const auto set_marginalization_graph_start = TimingClock::now();
        sidecar_->setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
        set_marginalization_graph_ms =
            timing ? timing_elapsed_ms(set_marginalization_graph_start) : 0.0;
      }
      const auto get_odometry_beliefs_start = TimingClock::now();
      outgoing = sidecar_->getOdometryBeliefs(request_keys, peer_agent_id_);
      get_odometry_beliefs_ms =
          timing ? timing_elapsed_ms(get_odometry_beliefs_start) : 0.0;
    } else {
      const auto get_odometry_beliefs_start = TimingClock::now();
      outgoing = getOdometryBeliefsDirect(
          request_keys, smoother, local_covariance_smoother);
      get_odometry_beliefs_ms =
          timing ? timing_elapsed_ms(get_odometry_beliefs_start) : 0.0;
    }
  } catch (const std::exception& e) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "GLIM CBS failed to create outgoing beliefs: " << e.what());
    return;
  }

  const auto message_start = TimingClock::now();
  liorf::pose_odom_belief_array msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.beliefs.reserve(outgoing.size());
  for (const auto& odom : outgoing) {
    const size_t from_index = poseIndex(odom.from_pose_key);
    const size_t to_index = poseIndex(odom.to_pose_key);
    if (from_index >= pose_timestamps_sec_.size() ||
        to_index >= pose_timestamps_sec_.size() ||
        pose_timestamps_sec_[from_index] <= 0.0 ||
        pose_timestamps_sec_[to_index] <= 0.0) {
      continue;
    }

    liorf::pose_odom_belief belief_msg;
    belief_msg.header = msg.header;
    belief_msg.source_agent = static_cast<uint8_t>(self_agent_id_);
    belief_msg.from_pose_index = static_cast<uint32_t>(from_index);
    belief_msg.to_pose_index = static_cast<uint32_t>(to_index);
    belief_msg.revision = 0u;
    belief_msg.from_stamp_sec = pose_timestamps_sec_[from_index];
    belief_msg.to_stamp_sec = pose_timestamps_sec_[to_index];
    belief_msg.relax_factor =
        relax_factor_ > 0.0 ? relax_factor_ : odom.relax_factor;
    gtsam::Matrix covariance_to_publish = odom.covariance;
    if (health_sender_params_.enable && health_sender_params_.sender_enable) {
      std::optional<gtsam::Matrix> absolute_covariance;
      if (odom.to_pose_covariance_valid) {
        absolute_covariance = odom.to_pose_covariance;
      }
      const auto health_result = cbs::health_aware::applySenderHealth(
          odom.covariance,
          absolute_covariance,
          &sender_health_state_,
          health_sender_params_);
      covariance_to_publish = health_result.covariance;
      last_sender_health_alpha_ = health_result.alpha_health;
      logHealthAwareSenderRow(self_agent_id_,
                              peer_agent_id_,
                              odom.from_pose_key,
                              odom.to_pose_key,
                              health_result);
    }
    std::array<double, 6> relative_mu;
    std::array<double, 36> covariance;
    vector6ToArray(gtsam::Pose3::Logmap(odom.measured_from_to), &relative_mu);
    matrix6ToArray(covariance_to_publish, &covariance);
    if (dcreg_belief_shadow_enable_) {
      const gtsam::Point3 measured_translation =
          odom.measured_from_to.translation();
      const gtsam::Quaternion measured_quaternion =
          odom.measured_from_to.rotation().toQuaternion();
      ROS_INFO_STREAM(
          "GLIM_DCREG_BELIEF_SHADOW_ROW,"
          << std::setprecision(17)
          << stamp.toSec() << "," << from_index << "," << to_index << ","
          << pose_timestamps_sec_[from_index] << ","
          << pose_timestamps_sec_[to_index] << ","
          << cbs::relative_pose_covariance::matrixToken(
                 gtsam::Pose3::Logmap(odom.measured_from_to))
          << "," << measured_translation.x()
          << "," << measured_translation.y()
          << "," << measured_translation.z()
          << "," << measured_quaternion.x()
          << "," << measured_quaternion.y()
          << "," << measured_quaternion.z()
          << "," << measured_quaternion.w()
          << "," << cbs::relative_pose_covariance::matrixToken(
                 covariance_to_publish)
          << "," << last_sender_health_alpha_
          << "," << outgoing_odom_covariance_mode_);
    }
    for (size_t i = 0; i < relative_mu.size(); ++i) {
      belief_msg.relative_mu[i] = relative_mu[i];
    }
    for (size_t i = 0; i < covariance.size(); ++i) {
      belief_msg.covariance[i] = covariance[i];
    }
    if (export_endpoint_audit_enable_ || live_revision_enable_) {
      if (export_endpoint_audit_enable_) {
        ROS_INFO_STREAM(
          "GLIM_CBS_EXPORT_BELIEF_ROW,"
          << std::setprecision(17) << static_cast<char>(self_agent_id_)
          << "," << from_index << "," << to_index << ",0,"
          << ros::Time::now().toSec() << "," << stamp.toSec() << ","
          << belief_msg.from_stamp_sec << "," << belief_msg.to_stamp_sec
          << "," << cbs::relative_pose_covariance::matrixToken(
                         gtsam::Pose3::Logmap(odom.measured_from_to))
          << "," << cbs::relative_pose_covariance::matrixToken(
                         covariance_to_publish));
      }
      const auto edge_id = std::make_pair(from_index, to_index);
      if (revision_audit_beliefs_.count(edge_id) == 0u) {
        RevisionAuditBelief tracked;
        tracked.revision = 0u;
        tracked.last_update_index =
            has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u;
        tracked.mean = odom.measured_from_to;
        tracked.covariance = covariance_to_publish;
        revision_audit_beliefs_.emplace(edge_id, std::move(tracked));
      }
    }
    msg.beliefs.push_back(belief_msg);
  }
  const double message_build_ms =
      timing ? timing_elapsed_ms(message_start) : 0.0;

  if (latest_pair_valid && !msg.beliefs.empty()) {
    last_outgoing_pair_valid_ = true;
    last_outgoing_from_key_ = latest_from_key;
    last_outgoing_to_key_ = latest_to_key;
  }

  const auto publish_start = TimingClock::now();
  if (!msg.beliefs.empty()) {
    belief_pub_.publish(msg);
    outgoing_total_.fetch_add(msg.beliefs.size(), std::memory_order_relaxed);
    if (dcreg_edge_metadata_) {
      dcreg_edge_metadata_->enqueueBeliefArray(msg, pose_timestamps_sec_);
    }
  }
  const double publish_ms = timing ? timing_elapsed_ms(publish_start) : 0.0;
  if (timing) {
    ROS_INFO_STREAM(
        "GLIM_CBS_TIMING_ROW,publish_outgoing,"
        << (has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u) << ",0,0,0,0,0,"
        << std::numeric_limits<double>::quiet_NaN()
        << ",0,0,0,0,0,"
        << timing_elapsed_ms(total_start) << ",0,"
        << msg.beliefs.size() << "," << timing_elapsed_ms(total_start)
        << "," << request_build_ms << "," << set_marginalization_graph_ms
	        << "," << get_odometry_beliefs_ms << "," << message_build_ms
	        << "," << publish_ms << ","
	        << (explicit_pairs.empty() ? request_keys.size()
	                                   : explicit_pairs.size() * 2u));
  }
}

std::vector<std::pair<gtsam::Key, gtsam::Key>>
GlimCbsBridge::buildTimeHorizonPairs(
    const std::vector<gtsam::Key>& pose_keys) const {
  std::vector<std::pair<gtsam::Key, gtsam::Key>> pairs;
  if (pose_keys.size() < 2u || max_horizon_pairs_per_update_ == 0u) {
    return pairs;
  }

  for (size_t to_i = 1u; to_i < pose_keys.size(); ++to_i) {
    const size_t to_index = poseIndex(pose_keys[to_i]);
    if (to_index >= pose_timestamps_sec_.size()) {
      continue;
    }
    const double to_stamp = pose_timestamps_sec_[to_index];
    if (!std::isfinite(to_stamp) || to_stamp <= 0.0) {
      continue;
    }

    size_t best_from_i = pose_keys.size();
    double best_error = std::numeric_limits<double>::infinity();
    for (size_t from_i = 0u; from_i < to_i; ++from_i) {
      const size_t from_index = poseIndex(pose_keys[from_i]);
      if (from_index >= pose_timestamps_sec_.size()) {
        continue;
      }
      const double from_stamp = pose_timestamps_sec_[from_index];
      const double dt = to_stamp - from_stamp;
      if (!std::isfinite(from_stamp) || from_stamp <= 0.0 ||
          !std::isfinite(dt) || dt <= 0.0) {
        continue;
      }
      const double error = std::abs(dt - horizon_sec_);
      if (error < best_error) {
        best_error = error;
        best_from_i = from_i;
      }
    }

    if (best_from_i != pose_keys.size() && best_error <= horizon_tolerance_sec_) {
      pairs.emplace_back(pose_keys[best_from_i], pose_keys[to_i]);
    }
  }

  if (pairs.size() > max_horizon_pairs_per_update_) {
    pairs.erase(pairs.begin(),
                pairs.end() -
                    static_cast<std::ptrdiff_t>(max_horizon_pairs_per_update_));
  }
  return pairs;
}

std::vector<cbs::BPSAM::CbsOdometryBelief>
GlimCbsBridge::getOdometryBeliefsDirect(
    const gtsam::KeySet& request_keys,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const cbs::IncrementalFixedLagBpsamSmoother* covariance_smoother) const {
  std::vector<gtsam::Key> pose_keys;
  pose_keys.reserve(request_keys.size());
  for (const gtsam::Key key : request_keys) {
    if (isPoseKey(key) && smoother.getLinearizationPoint().exists(key)) {
      pose_keys.push_back(key);
    }
  }
  std::sort(pose_keys.begin(), pose_keys.end(), [](gtsam::Key a, gtsam::Key b) {
    return poseIndex(a) < poseIndex(b);
  });

  std::vector<std::pair<gtsam::Key, gtsam::Key>> pose_pairs;
  pose_pairs.reserve(pose_keys.size() > 0u ? pose_keys.size() - 1u : 0u);
  for (size_t i = 1u; i < pose_keys.size(); ++i) {
    pose_pairs.emplace_back(pose_keys[i - 1u], pose_keys[i]);
  }
  return getOdometryBeliefsDirectForPairs(
      pose_pairs, smoother, covariance_smoother);
}

std::vector<cbs::BPSAM::CbsOdometryBelief>
GlimCbsBridge::getOdometryBeliefsDirectForPairs(
    const std::vector<std::pair<gtsam::Key, gtsam::Key>>& pose_pairs,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const cbs::IncrementalFixedLagBpsamSmoother* covariance_smoother) const {
  const auto total_start = TimingClock::now();
  std::vector<cbs::BPSAM::CbsOdometryBelief> odometry_beliefs;
  odometry_beliefs.reserve(pose_pairs.size());
  if (pose_pairs.empty()) {
    ROS_INFO_STREAM("CBS_ODOM_MARGINAL_PAIR_COUNT_ROW,"
                    << directionToken(self_agent_id_, peer_agent_id_) << ","
                    << agentToken(self_agent_id_) << ","
                    << agentToken(peer_agent_id_) << ",0,0,0");
    return odometry_beliefs;
  }

  size_t n_marginal_pose_pairs = 0u;
  double estimate_pose_pairs_ms = 0.0;
  double joint_marginal_information_total_ms = 0.0;
  double joint_marginal_information_max_ms = 0.0;
  double covariance_ms = 0.0;

  for (const auto& [from_key, to_key] : pose_pairs) {
    if (!isPoseKey(from_key) || !isPoseKey(to_key) ||
        !smoother.getLinearizationPoint().exists(from_key) ||
        !smoother.getLinearizationPoint().exists(to_key)) {
      continue;
    }
    try {
      const auto estimate_pose_pair_start = TimingClock::now();
      const gtsam::Pose3 from_pose =
          smoother.calculateEstimate<gtsam::Pose3>(from_key);
      const gtsam::Pose3 to_pose =
          smoother.calculateEstimate<gtsam::Pose3>(to_key);
      const gtsam::Pose3 measured_from_to = from_pose.between(to_pose);
      if (export_endpoint_audit_enable_) {
        const size_t from_index = poseIndex(from_key);
        const size_t to_index = poseIndex(to_key);
        const size_t current_index =
            pose_timestamps_sec_.empty() ? 0u : pose_timestamps_sec_.size() - 1u;
        const double current_stamp =
            current_index < pose_timestamps_sec_.size()
                ? pose_timestamps_sec_[current_index]
                : 0.0;
        const auto from_quaternion = from_pose.rotation().toQuaternion();
        const auto to_quaternion = to_pose.rotation().toQuaternion();
        ROS_INFO_STREAM(
            "GLIM_CBS_EXPORT_ENDPOINT_ROW,"
            << std::setprecision(17)
            << static_cast<char>(self_agent_id_) << "," << current_index << ","
            << current_stamp << "," << from_index << "," << to_index << ","
            << pose_timestamps_sec_[from_index] << ","
            << pose_timestamps_sec_[to_index] << ","
            << from_pose.translation().x() << ","
            << from_pose.translation().y() << ","
            << from_pose.translation().z() << "," << from_quaternion.x()
            << "," << from_quaternion.y() << "," << from_quaternion.z()
            << "," << from_quaternion.w() << ","
            << to_pose.translation().x() << "," << to_pose.translation().y()
            << "," << to_pose.translation().z() << "," << to_quaternion.x()
            << "," << to_quaternion.y() << "," << to_quaternion.z() << ","
            << to_quaternion.w() << ","
            << cbs::relative_pose_covariance::matrixToken(
                   gtsam::Pose3::Logmap(measured_from_to)));
      }
      estimate_pose_pairs_ms += timing_elapsed_ms(estimate_pose_pair_start);

      const bool use_local_covariance =
          covariance_smoother && covariance_smoother->valueExists(from_key) &&
          covariance_smoother->valueExists(to_key);

      const auto joint_info_start = TimingClock::now();
      const gtsam::JointMarginal joint_info =
          use_local_covariance
              ? covariance_smoother->jointMarginalInformation(
                    gtsam::KeyVector{from_key, to_key})
              : smoother.jointMarginalInformation(
                    gtsam::KeyVector{from_key, to_key});
      const double joint_info_ms = timing_elapsed_ms(joint_info_start);
      joint_marginal_information_total_ms += joint_info_ms;
      joint_marginal_information_max_ms =
          std::max(joint_marginal_information_max_ms, joint_info_ms);
      ++n_marginal_pose_pairs;

      const auto covariance_start = TimingClock::now();
      const auto covariance_result = cbs::relative_pose_covariance::compute(
          joint_info,
          from_key,
          to_key,
          from_pose,
          to_pose,
          measured_from_to,
          outgoing_odom_covariance_mode_);
      const gtsam::Matrix from_pose_covariance =
          use_local_covariance ? covariance_smoother->marginalCovariance(from_key)
                               : smoother.marginalCovariance(from_key);
      const gtsam::Matrix to_pose_covariance =
          use_local_covariance ? covariance_smoother->marginalCovariance(to_key)
                               : smoother.marginalCovariance(to_key);
      const double from_pose_trace = from_pose_covariance.trace();
      const double to_pose_trace = to_pose_covariance.trace();
      covariance_ms += timing_elapsed_ms(covariance_start);

      cbs::BPSAM::CbsOdometryBelief odom;
      odom.source_agent = self_agent_id_;
      odom.from_pose_key = from_key;
      odom.to_pose_key = to_key;
      odom.sender_from_pose_key = from_key;
      odom.sender_to_pose_key = to_key;
      odom.measured_from_to = measured_from_to;
      odom.covariance = covariance_result.covariance;
      odom.from_pose_trace = from_pose_trace;
      odom.to_pose_trace = to_pose_trace;
      if (to_pose_covariance.rows() == 6 && to_pose_covariance.cols() == 6 &&
          to_pose_covariance.allFinite()) {
        odom.to_pose_covariance = to_pose_covariance;
        odom.to_pose_covariance_valid = true;
      }
      odometry_beliefs.push_back(odom);

      ROS_INFO_STREAM("CBS_ODOM_OUTGOING_ROW,"
                      << directionToken(self_agent_id_, peer_agent_id_) << ","
                      << agentToken(self_agent_id_) << ","
                      << agentToken(peer_agent_id_) << ","
                      << poseToken(self_agent_id_, poseIndex(from_key)) << ","
                      << poseToken(self_agent_id_, poseIndex(to_key)) << ","
                      << from_pose_trace << "," << to_pose_trace << ","
                      << odom.covariance.trace() << ",sent_direct");
      ROS_INFO_STREAM("CBS_ODOM_RELATIVE_COVARIANCE_ROW,"
                      << directionToken(self_agent_id_, peer_agent_id_) << ","
                      << agentToken(self_agent_id_) << ","
                      << agentToken(peer_agent_id_) << ","
                      << poseToken(self_agent_id_, poseIndex(from_key)) << ","
                      << poseToken(self_agent_id_, poseIndex(to_key)) << ","
                      << cbs::relative_pose_covariance::modeToken(
                             covariance_result.mode)
                      << ","
                      << covariance_result.traceConditionalOld() << ","
                      << covariance_result.traceSchurRelativeNew() << ","
                      << covariance_result.ratioNewOverOld() << ","
                      << covariance_result.eig_old_min << ","
                      << covariance_result.eig_old_max << ","
                      << covariance_result.eig_new_min << ","
                      << covariance_result.eig_new_max << ","
                      << covariance_result.rotation_trace_old << ","
                      << covariance_result.translation_trace_old << ","
                      << covariance_result.rotation_trace_new << ","
                      << covariance_result.translation_trace_new << ","
                      << covariance_result.h_to_rank << ","
                      << covariance_result.h_to_condition_estimate << ","
                      << covariance_result.schur_vs_direct_difference_norm
                      << ","
                      << (covariance_result.used_jitter ? 1 : 0) << ","
                      << covariance_result.jitter_added << ","
                      << cbs::relative_pose_covariance::eigenvaluesToken(
                             covariance_result.lambda_rel_eigenvalues)
                      << ","
                      << cbs::relative_pose_covariance::eigenvaluesToken(
                             covariance_result.sigma_rel_eigenvalues)
                      << ","
                      << covariance_result.status);
      ROS_INFO_STREAM("CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW,"
                      << directionToken(self_agent_id_, peer_agent_id_) << ","
                      << agentToken(self_agent_id_) << ","
                      << agentToken(peer_agent_id_) << ","
                      << poseToken(self_agent_id_, poseIndex(from_key)) << ","
                      << poseToken(self_agent_id_, poseIndex(to_key)) << ","
                      << covariance_result.joint_sigma_ff_trace << ","
                      << covariance_result.joint_sigma_tt_trace << ","
                      << covariance_result.propagated_from_trace << ","
                      << covariance_result.propagated_to_trace << ","
                      << covariance_result.propagated_cross_trace << ","
                      << covariance_result.propagated_sum_trace);
      const bool matrix_dump_trace_ok =
          relative_covariance_matrix_dump_trace_max_ <= 0.0 ||
          covariance_result.traceSchurRelativeNew() <=
              relative_covariance_matrix_dump_trace_max_;
      const auto edge_indices = std::make_pair(
          poseIndex(from_key), poseIndex(to_key));
      const bool matrix_dump_edge_ok =
          relative_covariance_matrix_dump_every_occurrence_ ||
          relative_covariance_matrix_dump_edge_filter_.empty() ||
          relative_covariance_matrix_dump_edge_filter_.count(edge_indices) !=
              0u;
      const bool matrix_dump_edge_already_seen =
          !relative_covariance_matrix_dump_every_occurrence_ &&
          relative_covariance_matrix_dump_edge_filter_.empty() &&
          relative_covariance_matrix_dumped_edges_.count(
              std::make_pair(from_key, to_key)) != 0u;
      if (relative_covariance_matrix_dump_enable_ &&
          matrix_dump_trace_ok &&
          matrix_dump_edge_ok && !matrix_dump_edge_already_seen &&
          (relative_covariance_matrix_dump_limit_ < 0 ||
           relative_covariance_matrix_dump_count_ <
               static_cast<size_t>(
                   relative_covariance_matrix_dump_limit_))) {
        const size_t sample_index = relative_covariance_matrix_dump_count_++;
        if (!relative_covariance_matrix_dump_every_occurrence_ &&
            relative_covariance_matrix_dump_edge_filter_.empty()) {
          relative_covariance_matrix_dumped_edges_.insert(
              std::make_pair(from_key, to_key));
        }
        for (const auto& [label, matrix] :
             cbs::relative_pose_covariance::matrixDumpRows(
                 covariance_result)) {
          ROS_INFO_STREAM("CBS_ODOM_RELATIVE_COVARIANCE_MATRIX_ROW,"
                          << directionToken(self_agent_id_, peer_agent_id_)
                          << "," << agentToken(self_agent_id_) << ","
                          << agentToken(peer_agent_id_) << ","
                          << poseToken(self_agent_id_, poseIndex(from_key))
                          << ","
                          << poseToken(self_agent_id_, poseIndex(to_key))
                          << "," << sample_index << "," << label << ","
                          << matrix.rows() << "," << matrix.cols() << ","
                          << cbs::relative_pose_covariance::matrixToken(
                                 matrix));
        }
      }
    } catch (const std::exception& e) {
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "GLIM CBS direct outgoing odometry skipped "
              << poseToken(self_agent_id_, poseIndex(from_key)) << "->"
              << poseToken(self_agent_id_, poseIndex(to_key)) << ": "
              << e.what());
    }
  }

  ROS_INFO_STREAM("CBS_ODOM_MARGINAL_PAIR_COUNT_ROW,"
                  << directionToken(self_agent_id_, peer_agent_id_) << ","
                  << agentToken(self_agent_id_) << ","
                  << agentToken(peer_agent_id_) << ","
                  << (pose_pairs.size() * 2u) << "," << n_marginal_pose_pairs << ","
                  << odometry_beliefs.size());
  ROS_INFO_STREAM("CBS_ODOM_BELIEF_TIMING_ROW,"
                  << directionToken(self_agent_id_, peer_agent_id_) << ","
                  << agentToken(self_agent_id_) << ","
                  << agentToken(peer_agent_id_) << ","
                  << (pose_pairs.size() * 2u) << "," << (pose_pairs.size() * 2u) << ","
                  << odometry_beliefs.size() << ",0,"
                  << estimate_pose_pairs_ms << ","
                  << joint_marginal_information_total_ms << ","
                  << n_marginal_pose_pairs << ","
                  << joint_marginal_information_max_ms << ","
                  << covariance_ms << "," << timing_elapsed_ms(total_start));
  return odometry_beliefs;
}

void GlimCbsBridge::publishOdometryAndRerun(
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (!has_last_pose_key_) {
    return;
  }
  const size_t index = poseIndex(last_pose_key_);
  if (index >= pose_timestamps_sec_.size()) {
    return;
  }
  const ros::Time stamp(pose_timestamps_sec_[index]);
  try {
    const bool timing = timing_enabled();
    const auto total_start = TimingClock::now();
    const auto estimate_start = TimingClock::now();
    const gtsam::Pose3 pose =
        smoother.calculateEstimate<gtsam::Pose3>(last_pose_key_);
    const double estimate_ms = timing ? timing_elapsed_ms(estimate_start) : 0.0;
    if (!finitePose(pose)) {
      return;
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = pose.translation().z();
    const auto quat = pose.rotation().toQuaternion();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    gtsam::Matrix covariance;
    bool covariance_ok = false;
    double marginal_covariance_ms = 0.0;
    try {
      const auto marginal_covariance_start = TimingClock::now();
      covariance = smoother.marginalCovariance(last_pose_key_);
      marginal_covariance_ms =
          timing ? timing_elapsed_ms(marginal_covariance_start) : 0.0;
      writePoseCovarianceToOdometry(covariance, &odom);
      covariance_ok = true;
    } catch (const std::exception& e) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "GLIM CBS failed to compute pose covariance: " << e.what());
    }
    const auto odom_publish_start = TimingClock::now();
    odom_pub_.publish(odom);
    const double odom_publish_ms =
        timing ? timing_elapsed_ms(odom_publish_start) : 0.0;

    const auto rerun_start = TimingClock::now();
    if (rerun_ && rerun_geometry_enable_) {
      rerun_->visualizer.setTimeNSec(stampToNSec(stamp));
      rerun_->visualizer.drawTf("glim/base_link", pose, 0.5f);
      rerun_trajectory_.push_back(pose);
      if (rerun_trajectory_.size() > max_trajectory_len_) {
        const size_t extra = rerun_trajectory_.size() - max_trajectory_len_;
        rerun_trajectory_.erase(rerun_trajectory_.begin(),
                                rerun_trajectory_.begin() + extra);
      }
      if (rerun_trajectory_.size() > 1u) {
        rerun_->visualizer.drawTrajectory(
            "glim/trajectory",
            rerun_trajectory_,
            Eigen::Vector4f(245.f, 180.f, 20.f, 220.f),
            1.5f);
      }
      if (covariance.rows() >= 6 && covariance.cols() >= 6) {
        const Eigen::Matrix3d position_covariance =
            covariance.block<3, 3>(3, 3);
        if (position_covariance.allFinite()) {
          rerun_->visualizer.drawUncertainty(
              "glim/current_pose/uncertainty",
              pose,
              position_covariance,
              Eigen::Vector4f(245.f, 180.f, 20.f, 180.f),
              static_cast<float>(rerun_uncertainty_scale_));
          rerun_->visualizer.drawScalar(
              "glim/current_pose/uncertainty_frobenius_norm",
              position_covariance.norm());
        }
      }
    }
    const double rerun_ms = timing ? timing_elapsed_ms(rerun_start) : 0.0;
    if (timing) {
      ROS_INFO_STREAM(
          "GLIM_CBS_TIMING_ROW,publish_odometry,"
          << poseIndex(last_pose_key_) << ",0,0,0,0,0,"
          << std::numeric_limits<double>::quiet_NaN()
          << ",0,0,0,0," << timing_elapsed_ms(total_start)
          << ",0,0,0," << timing_elapsed_ms(total_start)
          << "," << estimate_ms << "," << marginal_covariance_ms
          << "," << odom_publish_ms << "," << rerun_ms << ","
          << (covariance_ok ? 1 : 0));
    }
  } catch (const std::exception& e) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "GLIM CBS failed to publish current odometry: " << e.what());
  }
}

gtsam::Pose3 GlimCbsBridge::lookupPose(
    gtsam::Key key,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback* smoother,
    const gtsam::Values* new_values) const {
  if (new_values && new_values->exists(key)) {
    return new_values->at<gtsam::Pose3>(key);
  }
  if (!smoother) {
    throw std::runtime_error("No smoother provided for pose lookup");
  }
  return smoother->calculateEstimate<gtsam::Pose3>(key);
}

void GlimCbsBridge::initRerun() {
  if (!rerun_enable_) {
    return;
  }
  if (rerun_recording_id_.empty()) {
    ros::param::param<std::string>(
        "/cbsms/rerun_recording_id", rerun_recording_id_, "");
  }
  if (rerun_recording_id_.empty()) {
    rerun_recording_id_ = makeRerunRecordingId("glim_cbs");
  }
  if (rerun_host_.empty() || rerun_host_ == "auto") {
    rerun_host_ = defaultRerunHost();
  }
  rerun_ = std::make_unique<RerunImpl>(rerun_recording_id_, rerun_host_);
}

void GlimCbsBridge::onOdometryTiming(const OdometryTimingStatus& timing) {
  if (!rerun_) {
    return;
  }

  const uint64_t stamp_nsec = stampSecToNSec(timing.stamp_sec);
  if (stamp_nsec > 0ull) {
    rerun_->visualizer.setTimeNSec(stamp_nsec);
  }
  rerun_->visualizer.drawScalar("glim/timing/frame_index",
                                static_cast<double>(timing.frame_index));
  rerun_->visualizer.drawScalar("glim/timing/point_count",
                                static_cast<double>(timing.point_count));
  rerun_->visualizer.drawScalar(
      "glim/timing/imu_integrated_count",
      static_cast<double>(timing.imu_integrated_count));
  rerun_->visualizer.drawScalar("glim/timing/new_factor_count",
                                static_cast<double>(timing.new_factor_count));
  rerun_->visualizer.drawScalar(
      "glim/timing/active_frame_count",
      static_cast<double>(timing.active_frame_count));
  rerun_->visualizer.drawScalar(
      "glim/timing/marginalized_frame_count",
      static_cast<double>(timing.marginalized_frame_count));
  rerun_->visualizer.drawScalar("glim/timing/state_lookup_ms",
                                timing.state_lookup_ms);
  rerun_->visualizer.drawScalar("glim/timing/inter_scan_imu_ms",
                                timing.inter_scan_imu_ms);
  rerun_->visualizer.drawScalar("glim/timing/imu_factor_ms",
                                timing.imu_factor_ms);
  rerun_->visualizer.drawScalar("glim/timing/intra_scan_imu_ms",
                                timing.intra_scan_imu_ms);
  rerun_->visualizer.drawScalar("glim/timing/deskew_ms", timing.deskew_ms);
  rerun_->visualizer.drawScalar("glim/timing/point_covariance_ms",
                                timing.point_covariance_ms);
  rerun_->visualizer.drawScalar("glim/timing/cpu_frame_ms",
                                timing.cpu_frame_ms);
  rerun_->visualizer.drawScalar("glim/timing/create_frame_ms",
                                timing.create_frame_ms);
  rerun_->visualizer.drawScalar("glim/timing/create_factors_ms",
                                timing.create_factors_ms);
  rerun_->visualizer.drawScalar("glim/timing/pre_smoother_callback_ms",
                                timing.pre_smoother_callback_ms);
  rerun_->visualizer.drawScalar("glim/timing/smoother_update_ms",
                                timing.smoother_update_ms);
  rerun_->visualizer.drawScalar("glim/timing/post_smoother_callback_ms",
                                timing.post_smoother_callback_ms);
  rerun_->visualizer.drawScalar("glim/timing/marginalization_ms",
                                timing.marginalization_ms);
  rerun_->visualizer.drawScalar("glim/timing/update_frames_ms",
                                timing.update_frames_ms);
  rerun_->visualizer.drawScalar("glim/timing/imu_validation_ms",
                                timing.imu_validation_ms);
  rerun_->visualizer.drawScalar("glim/timing/update_callbacks_ms",
                                timing.update_callbacks_ms);
  rerun_->visualizer.drawScalar("glim/timing/total_ms", timing.total_ms);
}

void GlimCbsBridge::drawRerunScalars(const ros::Time& stamp) {
  if (!rerun_) {
    return;
  }
  rerun_->visualizer.setTimeNSec(stampToNSec(stamp));
  const size_t received_total =
      received_total_.load(std::memory_order_relaxed);
  const size_t matched_total = matched_total_.load(std::memory_order_relaxed);
  const size_t injected_total = injected_total_.load(std::memory_order_relaxed);
  const size_t duplicate_total =
      duplicate_total_.load(std::memory_order_relaxed);
  const size_t rejected_total = rejected_total_.load(std::memory_order_relaxed);
  const size_t dropped_total = dropped_total_.load(std::memory_order_relaxed);
  const size_t retried_total = retried_total_.load(std::memory_order_relaxed);
  const size_t duration_mismatch_total =
      duration_mismatch_total_.load(std::memory_order_relaxed);
  const size_t outgoing_total = outgoing_total_.load(std::memory_order_relaxed);
  auto drawDelta = [this](const std::string& path,
                          const size_t current,
                          size_t* last) {
    if (!last) {
      return;
    }
    const size_t delta = current >= *last ? current - *last : current;
    rerun_->visualizer.drawScalar(path, static_cast<double>(delta));
    *last = current;
  };

  rerun_->visualizer.drawScalar(
      "glim/cbs/received_total", static_cast<double>(received_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/matched_total", static_cast<double>(matched_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/injected_total", static_cast<double>(injected_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/duplicate_total", static_cast<double>(duplicate_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/rejected_total", static_cast<double>(rejected_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/dropped_total", static_cast<double>(dropped_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/retried_total", static_cast<double>(retried_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/duration_mismatch_total",
      static_cast<double>(duration_mismatch_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/outgoing_total", static_cast<double>(outgoing_total));
  rerun_->visualizer.drawScalar(
      "glim/cbs/beliefs/published_total",
      static_cast<double>(outgoing_total));
  if (std::isfinite(last_sender_health_alpha_)) {
    rerun_->visualizer.drawScalar("metrics/cbs/glim_to_kimera/health_alpha",
                                  last_sender_health_alpha_);
  }
  if (std::isfinite(last_receiver_nis_nu_)) {
    rerun_->visualizer.drawScalar("metrics/cbs/kimera_to_glim/nis",
                                  last_receiver_nis_nu_);
  }
  if (std::isfinite(last_receiver_nis_alpha_)) {
    rerun_->visualizer.drawScalar("metrics/cbs/kimera_to_glim/alpha_cons",
                                  last_receiver_nis_alpha_);
  }

  drawDelta("glim/cbs/beliefs/received_per_update",
            received_total,
            &rerun_last_received_total_);
  drawDelta("glim/cbs/beliefs/matched_per_update",
            matched_total,
            &rerun_last_matched_total_);
  drawDelta("glim/cbs/beliefs/added_to_factor_graph_per_update",
            injected_total,
            &rerun_last_injected_total_);
  drawDelta("glim/cbs/beliefs/duplicate_per_update",
            duplicate_total,
            &rerun_last_duplicate_total_);
  drawDelta("glim/cbs/beliefs/rejected_per_update",
            rejected_total,
            &rerun_last_rejected_total_);
  drawDelta("glim/cbs/beliefs/dropped_by_receive_gate_per_update",
            dropped_total,
            &rerun_last_dropped_total_);
  drawDelta("glim/cbs/beliefs/retried_per_update",
            retried_total,
            &rerun_last_retried_total_);
  drawDelta("glim/cbs/beliefs/duration_mismatch_per_update",
            duration_mismatch_total,
            &rerun_last_duration_mismatch_total_);
  drawDelta("glim/cbs/beliefs/published_per_update",
            outgoing_total,
            &rerun_last_outgoing_total_);

  rerun_->visualizer.drawScalar("glim/cbs/timing/temporary_factor_ms",
                                rerun_last_cbs_temporary_ms_);
  rerun_->visualizer.drawScalar("glim/cbs/timing/publish_odometry_ms",
                                rerun_last_cbs_publish_odometry_ms_);
  rerun_->visualizer.drawScalar("glim/cbs/timing/publish_outgoing_ms",
                                rerun_last_cbs_publish_outgoing_ms_);
  rerun_->visualizer.drawScalar("glim/cbs/timing/update_finish_total_ms",
                                rerun_last_cbs_total_ms_);
  rerun_->visualizer.drawScalar(
      "glim/cbs/incoming/pending",
      static_cast<double>(rerun_last_cbs_pending_count_));
  rerun_->visualizer.drawScalar(
      "glim/cbs/incoming/pending_deferred",
      static_cast<double>(rerun_last_cbs_pending_deferred_count_));
  if (std::isfinite(rerun_last_cbs_pending_oldest_age_sec_)) {
    rerun_->visualizer.drawScalar(
        "glim/cbs/incoming/pending_oldest_age_sec",
        rerun_last_cbs_pending_oldest_age_sec_);
  }
}

void GlimCbsBridge::drawRerunBeliefDebug(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values) {
  if (!rerun_ || !rerun_geometry_enable_) {
    return;
  }
  try {
    const gtsam::Pose3 local_from =
        lookupPose(matched.local_from_key, &smoother, &new_values);
    const gtsam::Pose3 local_to =
        lookupPose(matched.local_to_key, &smoother, &new_values);
    const gtsam::Pose3 external_to = local_from.compose(gtsam::Pose3::Expmap(
        vector6FromArray(matched.incoming.relative_mu)));
    std::vector<std::pair<gtsam::Point3, gtsam::Point3>> local_edge;
    local_edge.emplace_back(local_from.translation(), local_to.translation());
    std::vector<std::pair<gtsam::Point3, gtsam::Point3>> external_edge;
    external_edge.emplace_back(local_from.translation(),
                               external_to.translation());
    const ros::Time stamp(pose_timestamps_sec_.at(matched.local_to_index));
    rerun_->visualizer.setTimeNSec(stampToNSec(stamp));
    rerun_->visualizer.drawLines(
        "glim/cbs/k2g_debug/local_edge",
        local_edge,
        {Eigen::Vector4f(245.f, 180.f, 20.f, 220.f)},
        1.25f);
    rerun_->visualizer.drawLines(
        "glim/cbs/k2g_debug/kimera_belief_edge",
        external_edge,
        {Eigen::Vector4f(40.f, 220.f, 80.f, 220.f)},
        1.25f);
  } catch (const std::exception&) {
  }
}

void GlimCbsBridge::capturePoseMergeDiagnostic(
    const MatchedBelief& matched,
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    const gtsam::Values& new_values,
    const gtsam::Pose3& external_relative) {
  try {
    PoseMergeDiagnostic diagnostic;
    diagnostic.valid = true;
    diagnostic.optimization_pose_index =
        has_last_pose_key_ ? poseIndex(last_pose_key_)
                           : matched.local_to_index;
    diagnostic.from_pose_key = matched.local_from_key;
    diagnostic.to_pose_key = matched.local_to_key;
    const gtsam::Pose3 from_pose =
        lookupPose(matched.local_from_key, &smoother, &new_values);
    const gtsam::Pose3 to_pose =
        lookupPose(matched.local_to_key, &smoother, &new_values);
    diagnostic.local_relative_before = from_pose.between(to_pose);
    diagnostic.external_relative = external_relative;

    if (!pending_pose_merge_diagnostic_.valid ||
        poseIndex(diagnostic.to_pose_key) >=
            poseIndex(pending_pose_merge_diagnostic_.to_pose_key)) {
      pending_pose_merge_diagnostic_ = std::move(diagnostic);
    }
  } catch (const std::exception& e) {
    ROS_DEBUG_STREAM("GLIM CBS pose-merge pre-update diagnostic unavailable: "
                     << e.what());
  }
}

void GlimCbsBridge::finalizePoseMergeDiagnostic(
    const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  if (!pending_pose_merge_diagnostic_.valid) {
    pose_merge_diagnostic_ = PoseMergeDiagnostic();
    return;
  }

  try {
    const gtsam::Key from_key = pending_pose_merge_diagnostic_.from_pose_key;
    const gtsam::Key to_key = pending_pose_merge_diagnostic_.to_pose_key;
    const gtsam::Pose3 from_pose =
        smoother.calculateEstimate<gtsam::Pose3>(from_key);
    const gtsam::Pose3 to_pose =
        smoother.calculateEstimate<gtsam::Pose3>(to_key);
    pending_pose_merge_diagnostic_.final_relative_after =
        from_pose.between(to_pose);
    pose_merge_diagnostic_ = pending_pose_merge_diagnostic_;

    const gtsam::Vector6 local_to_external = gtsam::Pose3::Logmap(
        pose_merge_diagnostic_.local_relative_before.between(
            pose_merge_diagnostic_.external_relative));
    const gtsam::Vector6 final_to_external = gtsam::Pose3::Logmap(
        pose_merge_diagnostic_.final_relative_after.between(
            pose_merge_diagnostic_.external_relative));
    ROS_INFO_STREAM(
        "GLIM_CBS_POSE_MERGE_ROW,"
        << pose_merge_diagnostic_.optimization_pose_index << ","
        << gtsam::DefaultKeyFormatter(from_key) << "->"
        << gtsam::DefaultKeyFormatter(to_key) << ","
        << local_to_external.head<3>().norm() << ","
        << local_to_external.tail<3>().norm() << ","
        << final_to_external.head<3>().norm() << ","
        << final_to_external.tail<3>().norm());

  } catch (const std::exception& e) {
    ROS_DEBUG_STREAM("GLIM CBS pose-merge post-update diagnostic unavailable: "
                     << e.what());
    pose_merge_diagnostic_ = PoseMergeDiagnostic();
  }
}

cbs::AgentId GlimCbsBridge::parseAgentId(const std::string& value,
                                          cbs::AgentId fallback) {
  if (value.empty()) {
    return fallback;
  }
  return static_cast<cbs::AgentId>(value.front());
}

gtsam::Matrix6 GlimCbsBridge::matrix6FromArray(
    const std::array<double, 36>& values) {
  gtsam::Matrix6 matrix;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      matrix(row, col) = values[static_cast<size_t>(row * 6 + col)];
    }
  }
  return matrix;
}

gtsam::Vector6 GlimCbsBridge::vector6FromArray(
    const std::array<double, 6>& values) {
  gtsam::Vector6 vector;
  for (int i = 0; i < 6; ++i) {
    vector(i) = values[static_cast<size_t>(i)];
  }
  return vector;
}

void GlimCbsBridge::matrix6ToArray(const gtsam::Matrix& matrix,
                                   std::array<double, 36>* values) {
  if (!values) {
    return;
  }
  values->fill(0.0);
  const int rows = std::min<int>(6, matrix.rows());
  const int cols = std::min<int>(6, matrix.cols());
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      (*values)[static_cast<size_t>(row * 6 + col)] = matrix(row, col);
    }
  }
}

void GlimCbsBridge::vector6ToArray(const gtsam::Vector6& vector,
                                   std::array<double, 6>* values) {
  if (!values) {
    return;
  }
  for (int i = 0; i < 6; ++i) {
    (*values)[static_cast<size_t>(i)] = vector(i);
  }
}

bool GlimCbsBridge::finiteArray(const std::array<double, 6>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool GlimCbsBridge::finiteArray(const std::array<double, 36>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool GlimCbsBridge::isPoseKey(gtsam::Key key) {
  return gtsam::Symbol(key).chr() == 'x';
}

size_t GlimCbsBridge::poseIndex(gtsam::Key key) {
  return static_cast<size_t>(gtsam::Symbol(key).index());
}

gtsam::Key GlimCbsBridge::poseKey(size_t index) {
  return X(static_cast<int>(index));
}

std::string GlimCbsBridge::poseToken(cbs::AgentId agent, size_t index) {
  std::ostringstream stream;
  stream << static_cast<char>(agent) << index;
  return stream.str();
}

void GlimCbsBridge::writePoseCovarianceToOdometry(
    const gtsam::Matrix& covariance,
    nav_msgs::Odometry* odometry) {
  if (!odometry || covariance.rows() < 6 || covariance.cols() < 6) {
    return;
  }
  static const int remapping[6] = {3, 4, 5, 0, 1, 2};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      odometry->pose.covariance[remapping[row] * 6 + remapping[col]] =
          covariance(row, col);
    }
  }
}

}  // namespace glim
