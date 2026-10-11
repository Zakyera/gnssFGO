#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <liorf/pose_odom_belief_array.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <cbs/utils/health_aware_relative_cbs.h>
#include <glim_ros/cbs_native_imu_mean_bridge.hpp>

namespace gtsam_points {
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

struct OdometryTimingStatus;
namespace dcreg_edge_metadata {
class Publisher;
}

class GlimCbsBridge {
 public:
  explicit GlimCbsBridge(ros::NodeHandle& private_nh);
  ~GlimCbsBridge();

  bool enabled() const { return enabled_; }
  void shutdown();
  void cacheNativeImu(double stamp, const Eigen::Vector3d& acc,
                      const Eigen::Vector3d& gyro);

 public:
  enum class Mode {
    ObserveOnly,
    InjectPersistent,
    TemporaryLinear,
    ActiveWindowTemporary,
  };

  enum class OutgoingMarginalSource {
    Direct,
    SidecarBatch,
  };

  struct PoseMergeDiagnostic {
    bool valid = false;
    size_t optimization_pose_index = 0u;
    gtsam::Key from_pose_key = 0u;
    gtsam::Key to_pose_key = 0u;
    gtsam::Pose3 local_relative_before;
    gtsam::Pose3 external_relative;
    gtsam::Pose3 final_relative_after;
  };

  PoseMergeDiagnostic poseMergeDiagnostic() const {
    return pose_merge_diagnostic_;
  }

 private:
  struct IncomingBelief {
    cbs::AgentId source_agent = 0;
    size_t from_pose_index = 0u;
    size_t to_pose_index = 0u;
    uint32_t revision = 0u;
    double from_stamp_sec = 0.0;
    double to_stamp_sec = 0.0;
    double relax_factor = 0.0;
    std::array<double, 6> relative_mu{};
    std::array<double, 36> covariance{};
  };

  struct MatchedBelief {
    IncomingBelief incoming;
    size_t local_from_index = 0u;
    size_t local_to_index = 0u;
    gtsam::Key local_from_key = 0u;
    gtsam::Key local_to_key = 0u;
    double from_abs_dt = 0.0;
    double to_abs_dt = 0.0;
  };

  using EdgeId = std::tuple<int, size_t, size_t, size_t, size_t>;
  using SourceBeliefId = std::tuple<int, size_t, size_t>;
  using ReceiverEdgeId = std::tuple<int, size_t, size_t>;

  struct ActiveWindowExternalFactor {
    EdgeId edge_id;
    ReceiverEdgeId receiver_edge_id;
    cbs::AgentId source_agent = 0;
    size_t source_from_index = 0u;
    size_t source_to_index = 0u;
    gtsam::Key local_from_key = 0u;
    gtsam::Key local_to_key = 0u;
    double from_abs_dt = 0.0;
    double to_abs_dt = 0.0;
    double inserted_stamp_sec = std::numeric_limits<double>::quiet_NaN();
    size_t active_update_count = 0u;
    gtsam::Pose3 measured_from_to;
    gtsam::Matrix6 covariance = gtsam::Matrix6::Identity();
    gtsam::NonlinearFactor::shared_ptr factor;
    std::optional<gtsam::FactorIndex> factor_index;
  };

  struct PendingActiveWindowExternalFactor {
    ActiveWindowExternalFactor tracked;
    size_t new_factor_index = 0u;
  };

  void configure(ros::NodeHandle& private_nh);
  void registerCallbacks();

  void beliefCallback(const liorf::pose_odom_belief_arrayConstPtr& msg);
  void clockCallback(const rosgraph_msgs::ClockConstPtr& msg);

  void onSmootherUpdate(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph& new_factors,
      gtsam::Values& new_values,
      std::map<std::uint64_t, double>& new_stamps,
      gtsam::FactorIndices& factors_to_remove);
  void onSmootherUpdateFinish(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);
  void onOdometryTiming(const OdometryTimingStatus& timing);

  void updatePoseTimestamps(const std::map<std::uint64_t, double>& new_stamps);
  bool findClosestPoseByStamp(double stamp_sec,
                              size_t* index,
                              double* abs_dt) const;
  bool findReceiverEdgeForIncoming(const IncomingBelief& incoming,
                                   size_t* from_index,
                                   size_t* to_index,
                                   double* from_abs_dt,
                                   double* to_abs_dt) const;
  bool isLikelyActivePose(size_t index) const;
  bool shouldRetry(const IncomingBelief& belief) const;
  bool shouldRetryMatched(const MatchedBelief& belief) const;
  double latestPoseStamp() const;

  std::vector<MatchedBelief> consumeIncoming();
  std::vector<MatchedBelief> consumeDeferredMatchedBeliefs();
  EdgeId edgeId(const MatchedBelief& matched) const;
  ReceiverEdgeId receiverEdgeId(const MatchedBelief& matched) const;
  bool shouldInject(const MatchedBelief& matched);
  bool shouldDeferForReceiverCovariance(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother)
      const;
  void enqueueDeferredMatchedBelief(const MatchedBelief& matched);
  size_t removeExpiredActiveWindowFactors(
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      const std::map<std::uint64_t, double>& new_stamps,
      gtsam::FactorIndices& factors_to_remove);
  void registerPendingActiveWindowFactors(
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);
  void commitPendingActiveWindowFactorRemovals();
  bool buildExternalFactor(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactor::shared_ptr* factor,
      double* covariance_trace,
      const std::string& action,
      gtsam::Pose3* measured_out = nullptr,
      gtsam::Matrix6* covariance_out = nullptr);
  bool appendExternalFactor(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactorGraph* new_factors,
      gtsam::FactorIndices* factors_to_remove);
  void logReceiverLocalCompetition(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      const gtsam::NonlinearFactorGraph& pending_factors,
      const gtsam::Matrix6& inserted_covariance,
      const gtsam::NonlinearFactor::shared_ptr& incoming_factor);
  bool appendActiveWindowTemporaryFactor(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      gtsam::NonlinearFactorGraph* new_factors,
      gtsam::FactorIndices& factors_to_remove);
  bool queueTemporaryFactor(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values);
  size_t applyTemporaryFactors(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);
  void updateSidecar(const gtsam::NonlinearFactorGraph& new_factors,
                     const gtsam::Values& new_values,
                     const std::map<std::uint64_t, double>& new_stamps);
  void publishOutgoingBeliefs(
      const ros::Time& stamp,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);
  std::vector<std::pair<gtsam::Key, gtsam::Key>> buildTimeHorizonPairs(
      const std::vector<gtsam::Key>& pose_keys) const;
  std::vector<cbs::BPSAM::CbsOdometryBelief> getOdometryBeliefsDirect(
      const gtsam::KeySet& request_keys,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const cbs::IncrementalFixedLagBpsamSmoother* covariance_smoother =
          nullptr) const;
  std::vector<cbs::BPSAM::CbsOdometryBelief> getOdometryBeliefsDirectForPairs(
      const std::vector<std::pair<gtsam::Key, gtsam::Key>>& pose_pairs,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const cbs::IncrementalFixedLagBpsamSmoother* covariance_smoother =
          nullptr) const;
  void publishOdometryAndRerun(
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);
  void logActiveWindowFactorDiagnostics(
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);

  gtsam::Pose3 lookupPose(
      gtsam::Key key,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback* smoother,
      const gtsam::Values* new_values) const;

  static cbs::AgentId parseAgentId(const std::string& value,
                                   cbs::AgentId fallback);
  static gtsam::Matrix6 matrix6FromArray(const std::array<double, 36>& values);
  static gtsam::Vector6 vector6FromArray(const std::array<double, 6>& values);
  static void matrix6ToArray(const gtsam::Matrix& matrix,
                             std::array<double, 36>* values);
  static void vector6ToArray(const gtsam::Vector6& vector,
                             std::array<double, 6>* values);
  static bool finiteArray(const std::array<double, 6>& values);
  static bool finiteArray(const std::array<double, 36>& values);
  static bool isPoseKey(gtsam::Key key);
  static size_t poseIndex(gtsam::Key key);
  static gtsam::Key poseKey(size_t index);
  static std::string poseToken(cbs::AgentId agent, size_t index);
  static void writePoseCovarianceToOdometry(const gtsam::Matrix& covariance,
                                            nav_msgs::Odometry* odometry);

  void initRerun();
  void drawRerunScalars(const ros::Time& stamp);
  void drawRerunBeliefDebug(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values);
  void capturePoseMergeDiagnostic(
      const MatchedBelief& matched,
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      const gtsam::Values& new_values,
      const gtsam::Pose3& external_relative);
  void finalizePoseMergeDiagnostic(
      const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);

 private:
  bool enabled_ = false;
  bool experimental_native_imu_mean_alignment_ = false;
  std::unique_ptr<CbsNativeImuMeanBridge> native_imu_mean_bridge_;
  bool publish_beliefs_ = false;
  bool receive_external_beliefs_ = false;
  Mode mode_ = Mode::ObserveOnly;
  OutgoingMarginalSource outgoing_marginal_source_ =
      OutgoingMarginalSource::Direct;
  bool callbacks_registered_ = false;
  int smoother_update_callback_id_ = -1;
  int smoother_update_finish_callback_id_ = -1;
  int odometry_timing_callback_id_ = -1;

  ros::NodeHandle nh_;
  ros::Subscriber belief_sub_;
  ros::Subscriber clock_sub_;
  ros::Publisher belief_pub_;
  ros::Publisher odom_pub_;
  std::unique_ptr<dcreg_edge_metadata::Publisher> dcreg_edge_metadata_;

  cbs::AgentId self_agent_id_ = static_cast<cbs::AgentId>('g');
  cbs::AgentId peer_agent_id_ = static_cast<cbs::AgentId>('k');
  std::string belief_in_topic_ = "/kimera/cbs/odom_belief_out";
  std::string belief_out_topic_ = "/kimera/cbs/odom_belief_in";
  std::string odometry_topic_ = "/glim/cbs/odometry";
  std::string frame_id_ = "map";
  std::string child_frame_id_ = "glim_imu";
  std::string sender_mode_ = "adjacent_window";
  std::string receiver_match_mode_ = "nearest_stamp";
  std::string outgoing_odom_covariance_mode_ = "schur_relative_between";
  bool dcreg_belief_shadow_enable_ = false;
  bool export_endpoint_audit_enable_ = false;
  bool live_revision_enable_ = false;
  struct RevisionAuditBelief {
    size_t revision = 0u;
    size_t last_update_index = 0u;
    gtsam::Pose3 mean;
    gtsam::Matrix6 covariance = gtsam::Matrix6::Zero();
  };
  std::map<std::pair<size_t, size_t>, RevisionAuditBelief>
      revision_audit_beliefs_;
  bool relative_covariance_matrix_dump_enable_ = false;
  int relative_covariance_matrix_dump_limit_ = 3;
  double relative_covariance_matrix_dump_trace_max_ = 0.0;
  mutable size_t relative_covariance_matrix_dump_count_ = 0u;
  mutable std::set<std::pair<gtsam::Key, gtsam::Key>>
      relative_covariance_matrix_dumped_edges_;
  std::set<std::pair<size_t, size_t>>
      relative_covariance_matrix_dump_edge_filter_;
  bool relative_covariance_matrix_dump_every_occurrence_ = false;

  double smoother_lag_sec_ = 5.0;
  double timestamp_tolerance_sec_ = 0.12;
  double receive_start_delay_sec_ = 0.0;
  double receive_gate_reference_stamp_sec_ = 0.0;
  bool receive_gate_reference_set_ = false;
  double unmatched_retry_max_age_sec_ = 5.0;
  size_t unmatched_retry_max_beliefs_ = 500u;
  bool wait_for_receiver_covariance_linearization_enable_ = false;
  int belief_exchange_window_size_ = 25;
  bool duration_gate_enable_ = false;
  double duration_tolerance_sec_ = 0.06;
  double duration_ratio_min_ = 0.5;
  double duration_ratio_max_ = 2.0;
  double horizon_sec_ = 0.20;
  double horizon_tolerance_sec_ = 0.06;
  size_t max_horizon_pairs_per_update_ = 25u;
  size_t outgoing_publish_stride_ = 1u;
  size_t outgoing_publish_count_ = 0u;
  double covariance_scale_ = 1.0;
  double relax_factor_ = 0.0;
  bool active_factor_detail_log_enable_ = false;
  size_t active_factor_detail_max_rows_per_update_ = 1000u;
  size_t receiver_local_competition_audit_max_samples_ = 0u;
  size_t receiver_local_competition_audit_stride_ = 1u;
  size_t receiver_local_competition_audit_count_ = 0u;
  size_t receiver_local_competition_audit_seen_count_ = 0u;
  size_t max_trajectory_len_ = 2000u;
  cbs::health_aware::Params health_sender_params_;
  cbs::health_aware::Params health_receiver_params_;
  cbs::health_aware::SenderHealthState sender_health_state_;
  std::map<cbs::AgentId, cbs::health_aware::RelativeTrustCalibrationState>
      relative_trust_states_by_source_;
  double last_sender_health_alpha_ = std::numeric_limits<double>::quiet_NaN();
  double last_receiver_nis_nu_ = std::numeric_limits<double>::quiet_NaN();
  double last_receiver_nis_alpha_ = std::numeric_limits<double>::quiet_NaN();

  std::unique_ptr<cbs::IncrementalFixedLagBpsamSmoother> sidecar_;
  std::vector<double> pose_timestamps_sec_;
  gtsam::Key last_pose_key_ = 0u;
  bool has_last_pose_key_ = false;
  bool last_outgoing_pair_valid_ = false;
  gtsam::Key last_outgoing_from_key_ = 0u;
  gtsam::Key last_outgoing_to_key_ = 0u;

  mutable std::mutex incoming_mutex_;
  std::deque<IncomingBelief> incoming_;
  std::deque<MatchedBelief> deferred_matched_beliefs_;
  std::set<EdgeId> injected_edges_;
  struct LiveRevisionFactor {
    uint32_t revision = 0u;
    gtsam::NonlinearFactor::shared_ptr factor;
  };
  std::map<SourceBeliefId, LiveRevisionFactor> live_revision_factors_;
  std::set<EdgeId> receiver_local_competition_audit_seen_edges_;
  std::vector<gtsam::NonlinearFactor::shared_ptr>
      persistent_external_factors_;
  std::map<ReceiverEdgeId, ActiveWindowExternalFactor>
      active_window_external_factors_;
  std::vector<PendingActiveWindowExternalFactor>
      pending_active_window_external_factors_;
  std::vector<ReceiverEdgeId> pending_active_window_external_factor_removals_;
  gtsam::NonlinearFactorGraph pending_temporary_factors_;
  PoseMergeDiagnostic pending_pose_merge_diagnostic_;
  PoseMergeDiagnostic pose_merge_diagnostic_;

  std::atomic<size_t> received_total_{0u};
  std::atomic<size_t> dropped_total_{0u};
  std::atomic<size_t> matched_total_{0u};
  std::atomic<size_t> injected_total_{0u};
  std::atomic<size_t> duplicate_total_{0u};
  std::atomic<size_t> rejected_total_{0u};
  std::atomic<size_t> retried_total_{0u};
  std::atomic<size_t> duration_mismatch_total_{0u};
  std::atomic<size_t> outgoing_total_{0u};

  bool rerun_enable_ = false;
  bool rerun_geometry_enable_ = true;
  std::string rerun_recording_id_;
  std::string rerun_host_ = "auto";
  double rerun_uncertainty_scale_ = 1.25;
  std::vector<gtsam::Pose3> rerun_trajectory_;
  size_t rerun_last_received_total_ = 0u;
  size_t rerun_last_dropped_total_ = 0u;
  size_t rerun_last_matched_total_ = 0u;
  size_t rerun_last_injected_total_ = 0u;
  size_t rerun_last_duplicate_total_ = 0u;
  size_t rerun_last_rejected_total_ = 0u;
  size_t rerun_last_retried_total_ = 0u;
  size_t rerun_last_duration_mismatch_total_ = 0u;
  size_t rerun_last_outgoing_total_ = 0u;
  double rerun_last_cbs_temporary_ms_ = 0.0;
  double rerun_last_cbs_publish_odometry_ms_ = 0.0;
  double rerun_last_cbs_publish_outgoing_ms_ = 0.0;
  double rerun_last_cbs_total_ms_ = 0.0;
  size_t rerun_last_cbs_pending_count_ = 0u;
  size_t rerun_last_cbs_pending_deferred_count_ = 0u;
  double rerun_last_cbs_pending_oldest_age_sec_ =
      std::numeric_limits<double>::quiet_NaN();

  class RerunImpl;
  std::unique_ptr<RerunImpl> rerun_;
};

}  // namespace glim
