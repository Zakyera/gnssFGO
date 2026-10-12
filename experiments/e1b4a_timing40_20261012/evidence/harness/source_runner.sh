#!/usr/bin/env bash
set -euo pipefail

project_root="${CBSMS_PROJECT_ROOT:-/workspace/cbs_gtsam4.3}"
dataset_dir="${CBSMS_DATASET_DIR:-${project_root}/src/datasets/GrandTour}"
mission="${CBSMS_MISSION:-2024-10-01-11-29-55}"
duration_sec="${CBSMS_BAG_DURATION_SEC:-45.0}"
bag_start_sec="${CBSMS_BAG_START_SEC:-0.0}"
bag_rate="${CBSMS_BAG_RATE:-0.75}"
post_playback_drain_sec="${CBSMS_POST_PLAYBACK_DRAIN_SEC:-25}"
ros_master_port="${CBSMS_ROS_MASTER_PORT:-11368}"
cbs_enable="${CBSMS_CBS_ENABLE:-true}"
cbs_live_revision_enable="${CBSMS_CBS_LIVE_REVISION_ENABLE:-false}"
cbs_protocol="${CBSMS_CBS_PROTOCOL:-}"
if [[ -n "${cbs_protocol}" ]]; then
  case "${cbs_protocol}" in
    classic)
      cbs_live_revision_enable=false
      ;;
    live_revision)
      cbs_live_revision_enable=true
      ;;
    *)
      echo "CBSMS_CBS_PROTOCOL must be classic or live_revision." >&2
      exit 2
      ;;
  esac
  if [[ -n "${CBSMS_CBS_LIVE_REVISION_ENABLE+x}" &&
        "${CBSMS_CBS_LIVE_REVISION_ENABLE}" != "${cbs_live_revision_enable}" ]]; then
    echo "CBSMS_CBS_PROTOCOL conflicts with CBSMS_CBS_LIVE_REVISION_ENABLE." >&2
    exit 2
  fi
elif [[ "${cbs_live_revision_enable}" == true ]]; then
  cbs_protocol=live_revision
else
  cbs_protocol=classic
fi
cbs_direction="${CBSMS_CBS_DIRECTION:-bidirectional}"
presentation_enable="${CBSMS_PRESENTATION_ENABLE:-true}"
kimera_mode="${CBSMS_KIMERA_MODE:-mono}"
run_name="${CBSMS_RUN_NAME:-grandtour_${mission}_${duration_sec}s_$(date +%Y%m%d_%H%M%S)}"
run_root="${CBSMS_RUN_ROOT:-${project_root}/runs/grandtour_eth1_glim_kimera_cbs}"
run_dir="${run_root}/${run_name}"
experiment_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launch_file="${experiment_dir}/active_window.launch"
[[ "${cbs_enable}" == true && "${cbs_direction}" == bidirectional && "${duration_sec}" == 40.0 && "${bag_start_sec}" == 0.0 ]] || { echo "Prepared 40-second ON E1b profiling capture only." >&2; exit 2; }
glim_config_source="${CBSMS_GLIM_CONFIG_SOURCE:-${project_root}/clean_glim_ws/src/glim/config_grandtour_eth1_gpu_hesai_cpt7}"
kimera_profile_source="${CBSMS_KIMERA_PROFILE_SOURCE:-}"
kimera_calibration_overlay="${CBSMS_KIMERA_CALIBRATION_OVERLAY:-${project_root}/src/cbsms/config/grandtour_kimera/eth1_mono_shared_cpt7}"
reference_bag="${CBSMS_REFERENCE_BAG:-${dataset_dir}/${mission}_ap20_prism_position.bag}"
recording_id="${CBSMS_RECORDING_ID:-${run_name}}"
rerun_host="${CBSMS_RERUN_HOST:-rerun+http://172.17.0.1:9876/proxy}"
glim_factor_graph_stride="${CBSMS_GLIM_FACTOR_GRAPH_STRIDE:-1}"
kimera_factor_graph_stride="${CBSMS_KIMERA_FACTOR_GRAPH_STRIDE:-5}"
max_horizon_pairs_per_update="${CBSMS_MAX_HORIZON_PAIRS_PER_UPDATE:-25}"
odom_publish_stride="${CBSMS_ODOM_PUBLISH_STRIDE:-1}"
cbs_belief_receive_start_delay_sec="${CBSMS_CBS_BELIEF_RECEIVE_START_DELAY_SEC:-0.0}"
kimera_receiver_audit_max_samples="${CBSMS_KIMERA_RECEIVER_AUDIT_MAX_SAMPLES:-0}"
kimera_receiver_audit_stride="${CBSMS_KIMERA_RECEIVER_AUDIT_STRIDE:-1}"
glim_receiver_audit_max_samples="${CBSMS_GLIM_RECEIVER_AUDIT_MAX_SAMPLES:-0}"
glim_receiver_audit_stride="${CBSMS_GLIM_RECEIVER_AUDIT_STRIDE:-1}"
min_estimator_coverage_ratio="${CBSMS_MIN_ESTIMATOR_COVERAGE_RATIO:-0.90}"
cbs_odom_horizon_sec="${CBSMS_CBS_ODOM_HORIZON_SEC:-0.20}"
export_endpoint_audit_enable="${CBSMS_EXPORT_ENDPOINT_AUDIT_ENABLE:-false}"
diagnostic_node_output="${CBSMS_DIAGNOSTIC_NODE_OUTPUT:-screen}"
kimera_bad_update_audit_dir="${CBSMS_KIMERA_BAD_UPDATE_AUDIT_DIR:-}"
seed_clock_secs="${CBSMS_SEED_CLOCK_SECS:-1727775007}"

if [[ "${cbs_enable}" != true && "${cbs_enable}" != false ]]; then
  echo "CBSMS_CBS_ENABLE must be true or false." >&2
  exit 2
fi
if [[ "${presentation_enable}" != true && "${presentation_enable}" != false ]]; then
  echo "CBSMS_PRESENTATION_ENABLE must be true or false." >&2
  exit 2
fi
if [[ "${diagnostic_node_output}" != screen && "${diagnostic_node_output}" != log ]]; then
  echo "CBSMS_DIAGNOSTIC_NODE_OUTPUT must be screen or log." >&2
  exit 2
fi
if ! awk -v ratio="${min_estimator_coverage_ratio}" \
  'BEGIN { exit !(ratio > 0.0 && ratio <= 1.0) }'; then
  echo "CBSMS_MIN_ESTIMATOR_COVERAGE_RATIO must be in (0, 1]." >&2
  exit 2
fi
if ! awk -v horizon="${cbs_odom_horizon_sec}" \
  'BEGIN { exit !(horizon > 0.0) }'; then
  echo "CBSMS_CBS_ODOM_HORIZON_SEC must be positive." >&2
  exit 2
fi

case "${kimera_mode}" in
  mono)
    # This is the exact frozen 50-state/numOptimize=1 profile used by the
    # official Newer College campaign. Only GrandTour's author-provided
    # camera/IMU calibration files are overlaid below.
    kimera_profile_source="${kimera_profile_source:-${project_root}/runs/newer_college_maths_easy_kimera_cam4_lag50_20260820/profiles/cam4_lag50_numopt1}"
    kimera_sensor_pair=alphasense_front_left+cpt7_imu
    kimera_body_frame=cpt7_imu
    kimera_source_frame=cpt7_imu
    kimera_input_imu_topic=/boxi/cpt7/imu
    external_frame_transform=native_shared_cpt7
    ;;
  stereo)
    kimera_profile_source="${kimera_profile_source:-${project_root}/src/Kimera-VIO/params/GrandTourEth1StereoPosZStereoGuess}"
    kimera_sensor_pair=alphasense_front_stereo+alphasense_imu
    kimera_body_frame=alphasense_imu
    kimera_source_frame=alphasense_imu
    kimera_input_imu_topic=/boxi/alphasense/imu
    external_frame_transform=author_tf_model
    ;;
  stereo_shared_cpt7)
    kimera_profile_source="${kimera_profile_source:-${project_root}/src/Kimera-VIO/params/GrandTourEth1StereoSharedCpt7}"
    kimera_sensor_pair=alphasense_front_stereo+cpt7_imu
    kimera_body_frame=cpt7_imu
    kimera_source_frame=cpt7_imu
    kimera_input_imu_topic=/boxi/cpt7/imu
    external_frame_transform=native_shared_cpt7
    ;;
  *)
    echo "CBSMS_KIMERA_MODE must be mono, stereo, or stereo_shared_cpt7." >&2
    exit 2
    ;;
esac

bags=(
  "${dataset_dir}/${mission}_hesai.bag"
  "${dataset_dir}/${mission}_cpt7_imu.bag"
  "${dataset_dir}/${mission}_alphasense.bag"
)
if [[ "${kimera_mode}" == stereo ]]; then
  bags+=("${dataset_dir}/${mission}_alphasense_imu.bag")
fi
required_inputs=("${launch_file}" "${glim_config_source}"
  "${kimera_profile_source}" "${bags[@]}")
if [[ "${kimera_mode}" == mono ]]; then
  required_inputs+=("${kimera_calibration_overlay}/ImuParams.yaml"
    "${kimera_calibration_overlay}/LeftCameraParams.yaml"
    "${kimera_calibration_overlay}/README.md")
fi
for required in "${required_inputs[@]}"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required input: ${required}" >&2
    exit 3
  fi
done
if [[ "${presentation_enable}" == true && ! -f "${reference_bag}" ]]; then
  echo "Missing GrandTour prism reference: ${reference_bag}" >&2
  exit 3
fi
if [[ -e "${run_dir}" ]]; then
  echo "Refusing to overwrite run directory: ${run_dir}" >&2
  exit 3
fi

mkdir -p "${run_dir}/recordings" "${run_dir}/kimera_logs" \
  "${run_dir}/evaluation"
cp "${launch_file}" "${run_dir}/grandtour_launch_used.launch"
cp "${project_root}/src/glim_ros1/launch/newer_college_2021_glim_kimera_cbs_on.launch" \
  "${run_dir}/newer_college_reference_launch.launch"
cp "$0" "${run_dir}/experiment_runner_used.sh"
cp -a "${glim_config_source}" "${run_dir}/glim_config"
cp -a "${kimera_profile_source}" "${run_dir}/kimera_profile"
if [[ "${kimera_mode}" == mono ]]; then
  cp "${kimera_calibration_overlay}/ImuParams.yaml" \
    "${run_dir}/kimera_profile/ImuParams.yaml"
  cp "${kimera_calibration_overlay}/LeftCameraParams.yaml" \
    "${run_dir}/kimera_profile/LeftCameraParams.yaml"
  cp "${kimera_calibration_overlay}/README.md" \
    "${run_dir}/kimera_profile/PROFILE.md"
fi
printf '%s\n' "${recording_id}" >"${run_dir}/rerun_recording_id.txt"

export ROS_MASTER_URI="http://127.0.0.1:${ros_master_port}"
export ROS_IP=127.0.0.1

pids=()
recorders=()
estimator_pid=''
bag_pid=''
visualizer_pid=''
cleanup() {
  set +e
  for process_pid in "${pids[@]}"; do
    if [[ -n "${process_pid}" ]] && kill -0 "${process_pid}" 2>/dev/null; then
      kill -INT "${process_pid}" 2>/dev/null
    fi
  done
  sleep 2
  for process_pid in "${pids[@]}"; do
    if [[ -n "${process_pid}" ]] && kill -0 "${process_pid}" 2>/dev/null; then
      kill -TERM "${process_pid}" 2>/dev/null
    fi
  done
  for process_pid in "${pids[@]}"; do
    [[ -n "${process_pid}" ]] && wait "${process_pid}" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
source "${project_root}/devel/setup.bash"
export LD_LIBRARY_PATH="${run_root}/private_build:${project_root}/build/kimera_vio:${LD_LIBRARY_PATH}"

ground_truth_tum="${run_dir}/evaluation/ground_truth_prism.tum"
if [[ "${presentation_enable}" == true ]]; then
  python3 "${project_root}/src/cbsms/tools/export_grandtour_prism_tum.py" \
    --bag "${reference_bag}" --output "${ground_truth_tum}" \
    >"${run_dir}/evaluation/ground_truth_export.log"
  cp "${project_root}/src/cbsms/config/newer_college_cbs_ekf.yaml" \
    "${run_dir}/grandtour_cbs_ekf.yaml"
fi


if [[ "${CBSMS_USE_EXISTING_ROS_MASTER:-false}" == true ]]; then
  # Live downstream receivers may already be subscribed before source launch.
  # Do not own/kill that caller-managed master; estimator/CBS settings are unchanged.
  rosparam get /rosversion >/dev/null
elif [[ "${CBSMS_USE_EXISTING_ROS_MASTER:-false}" == false ]]; then
  roscore -p "${ros_master_port}" >"${run_dir}/roscore.log" 2>&1 &
  pids+=("$!")
else
  echo "CBSMS_USE_EXISTING_ROS_MASTER must be true or false." >&2
  exit 2
fi
for _ in $(seq 1 100); do
  rosparam get /rosversion >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true
rosparam set /cbsms/rerun_recording_id "${recording_id}"

local_belief_publish=true
if [[ "${presentation_enable}" == true ]]; then
  ekf_enable=true
  factor_graph_enable=true
else
  ekf_enable=false
  factor_graph_enable=false
fi
factor_graph_enable="${CBSMS_FACTOR_GRAPH_ENABLE:-${factor_graph_enable}}"
if [[ "${factor_graph_enable}" != true && "${factor_graph_enable}" != false ]]; then
  echo "CBSMS_FACTOR_GRAPH_ENABLE must be true or false." >&2
  exit 2
fi

glim_receive_external_beliefs=false
kimera_receive_external_beliefs=false
glim_cbs_mode=observe_only
if [[ "${cbs_enable}" == true ]]; then
  case "${cbs_direction}" in
    bidirectional)
      glim_receive_external_beliefs=true
      kimera_receive_external_beliefs=true
      glim_cbs_mode=active_window_temporary
      ;;
    g2k_only)
      kimera_receive_external_beliefs=true
      ;;
    k2g_only)
      glim_receive_external_beliefs=true
      glim_cbs_mode=active_window_temporary
      ;;
    *)
      echo "CBSMS_CBS_DIRECTION must be bidirectional, g2k_only, or k2g_only." >&2
      exit 2
      ;;
  esac
fi

roslaunch "${launch_file}" \
  glim_config_path:="${run_dir}/glim_config" \
  glim_dump_path:="${run_dir}/glim_dump" \
  kimera_mode:="${kimera_mode}" \
  kimera_params_folder:="${run_dir}/kimera_profile" \
  kimera_input_imu_topic:="${kimera_input_imu_topic}" \
  kimera_body_frame_id:="${kimera_body_frame}" \
  kimera_log_output_path:="${run_dir}/kimera_logs" \
  cbs_enable:="${cbs_enable}" \
  cbs_live_revision_enable:="${cbs_live_revision_enable}" \
  glim_publish_cbs_beliefs:="${local_belief_publish}" \
  kimera_publish_cbs_beliefs:="${local_belief_publish}" \
  glim_receive_cbs_beliefs:="${glim_receive_external_beliefs}" \
  kimera_receive_cbs_beliefs:="${kimera_receive_external_beliefs}" \
  glim_cbs_mode:="${glim_cbs_mode}" \
  kimera_cbs_backend_enable:=true \
  kimera_bad_update_audit_dir:="${kimera_bad_update_audit_dir}" \
  enable_ekf:="${ekf_enable}" \
  rerun_host:="${rerun_host}" \
  rerun_recording_id:="${recording_id}" \
  glim_factor_graph_inspector_enable:="${factor_graph_enable}" \
  glim_factor_graph_inspector_stride:="${glim_factor_graph_stride}" \
  kimera_factor_graph_inspector_enable:="${factor_graph_enable}" \
  kimera_factor_graph_inspector_stride:="${kimera_factor_graph_stride}" \
  cbs_odom_max_horizon_pairs_per_update:="${max_horizon_pairs_per_update}" \
  cbs_odom_publish_stride:="${odom_publish_stride}" \
  cbs_odom_horizon_sec:="${cbs_odom_horizon_sec}" \
  glim_export_endpoint_audit_enable:="${export_endpoint_audit_enable}" \
  diagnostic_node_output:="${diagnostic_node_output}" \
  cbs_belief_receive_start_delay_sec:="${cbs_belief_receive_start_delay_sec}" \
  kimera_receiver_local_competition_audit_max_samples:="${kimera_receiver_audit_max_samples}" \
  kimera_receiver_local_competition_audit_stride:="${kimera_receiver_audit_stride}" \
  glim_receiver_local_competition_audit_max_samples:="${glim_receiver_audit_max_samples}" \
  glim_receiver_local_competition_audit_stride:="${glim_receiver_audit_stride}" \
  >"${run_dir}/estimators.log" 2>&1 &
estimator_pid=$!
pids+=("${estimator_pid}")

if [[ "${presentation_enable}" == true ]]; then
  roslaunch cbsms grandtour_eth1_combined_rerun_visualizer.launch \
    ground_truth_path:="${ground_truth_tum}" \
    bag_duration:="${duration_sec}" \
    kimera_source_frame:="${kimera_source_frame}" \
    rerun_host:="${rerun_host}" \
    rerun_recording_id:="${recording_id}" \
    cbs_metrics_enable:=true \
    ekf_enable:=true \
    >"${run_dir}/rerun_visualizer.log" 2>&1 &
  visualizer_pid=$!
  pids+=("${visualizer_pid}")
fi

# Both online providers wait for a valid simulated clock before constructing
# all subscribers.  Seed a timestamp just before the mission begins.
rostopic pub -1 /clock rosgraph_msgs/Clock \
  "{clock: {secs: ${seed_clock_secs}, nsecs: 0}}" >/dev/null

ready=false
for _ in $(seq 1 400); do
  if ! kill -0 "${estimator_pid}" 2>/dev/null; then
    wait "${estimator_pid}"
    echo "Estimator launch exited during startup." >&2
    exit 5
  fi
  glim_info=$(rosnode info /glim_rosnode 2>/dev/null || true)
  kimera_info=$(rosnode info /kimera_vio_ros/kimera_vio_ros_node 2>/dev/null || true)
  presentation_ready=true
  if [[ "${presentation_enable}" == true ]]; then
    ekf_info=$(rosnode info /cbs_ekf/ekf_localization_node 2>/dev/null || true)
    adapter_info=$(rosnode info /cbs_ekf/belief_to_twist 2>/dev/null || true)
    visualizer_info=$(rosnode info /grandtour_eth1_combined_rerun_visualizer 2>/dev/null || true)
    if ! grep -q '/cbs_ekf/glim/twist' <<<"${ekf_info}" || \
       ! grep -q '/kimera/cbs/odom_belief_in' <<<"${adapter_info}" || \
       ! grep -q '/grandtour/ekf/prism_odometry' <<<"${visualizer_info}"; then
      presentation_ready=false
    fi
  fi
  kimera_sensor_ready=false
  if [[ "${kimera_mode}" == mono ]] && \
     grep -q '/grandtour/front_left/image_raw' <<<"${kimera_info}" && \
     grep -q "${kimera_input_imu_topic}" <<<"${kimera_info}"; then
    kimera_sensor_ready=true
  elif [[ "${kimera_mode}" != mono ]] && \
       grep -q '/grandtour/front_left/image_raw' <<<"${kimera_info}" && \
       grep -q '/grandtour/front_right/image_raw' <<<"${kimera_info}" && \
       grep -q "${kimera_input_imu_topic}" <<<"${kimera_info}"; then
    kimera_sensor_ready=true
  fi
  if grep -q '/boxi/hesai/points' <<<"${glim_info}" && \
     grep -q '/boxi/cpt7/imu' <<<"${glim_info}" && \
     [[ "${kimera_sensor_ready}" == true ]] && \
     [[ "${presentation_ready}" == true ]]; then
    ready=true
    break
  fi
  sleep 0.2
done
if [[ "${ready}" != true ]]; then
  echo "Startup readiness failed." >&2
  exit 5
fi

rosparam get /glim_rosnode >"${run_dir}/resolved_glim_params.yaml"
rosparam get /kimera_vio_ros/kimera_vio_ros_node \
  >"${run_dir}/resolved_kimera_params.yaml"
if [[ "${presentation_enable}" == true ]]; then
  rosparam get /cbs_ekf/ekf_localization_node \
    >"${run_dir}/resolved_ekf_params.yaml"
  rosparam get /cbs_ekf/belief_to_twist \
    >"${run_dir}/resolved_ekf_adapter_params.yaml"
fi

python3 "${experiment_dir}/verify_effective.py" "${run_dir}"

rostopic echo -p /glim/cbs/odometry >"${run_dir}/glim_odometry.csv" &
recorders+=("$!"); pids+=("$!")
rostopic echo -p /kimera_vio_ros/odometry >"${run_dir}/kimera_odometry.csv" &
recorders+=("$!"); pids+=("$!")
if [[ "${presentation_enable}" == true ]]; then
  rostopic echo -p /cbs_ekf/odometry/filtered \
    >"${run_dir}/ekf_odometry.csv" &
  recorders+=("$!"); pids+=("$!")
  rostopic echo -p /cbs_ekf/bridge_diagnostics \
    >"${run_dir}/ekf_bridge_diagnostics.csv" &
  recorders+=("$!"); pids+=("$!")
fi
rosbag record --buffsize=256 -O "${run_dir}/recordings/cbs.bag" \
  /kimera/cbs/odom_belief_in /kimera/cbs/odom_belief_out \
  /glim/cbs/odometry /kimera_vio_ros/odometry \
  /cbs_ekf/glim/twist /cbs_ekf/kimera/twist \
  /cbs_ekf/odometry/filtered /cbs_ekf/bridge_diagnostics \
  >"${run_dir}/cbs_recorder.log" 2>&1 &
recorders+=("$!"); pids+=("$!")

sleep 2
play_topics=(/boxi/hesai/points /boxi/cpt7/imu
  /boxi/alphasense/front_left/image_raw/compressed)
if [[ "${kimera_mode}" != mono ]]; then
  play_topics+=(/boxi/alphasense/front_right/image_raw/compressed)
fi
if [[ "${kimera_mode}" == stereo ]]; then
  play_topics+=(/boxi/alphasense/imu)
fi
rosbag play --quiet --clock --delay=2.0 --rate="${bag_rate}" \
  --start="${bag_start_sec}" --duration="${duration_sec}" "${bags[@]}" \
  --topics "${play_topics[@]}" \
  >"${run_dir}/rosbag.log" 2>&1 &
bag_pid=$!
pids+=("${bag_pid}")

while kill -0 "${bag_pid}" 2>/dev/null; do
  if ! kill -0 "${estimator_pid}" 2>/dev/null; then
    echo "Estimator launch exited during playback." >&2
    exit 5
  fi
  sleep 0.5
done
wait "${bag_pid}"
bag_pid=''
sleep "${post_playback_drain_sec}"

for process_pid in "${recorders[@]}"; do
  kill -INT "${process_pid}" 2>/dev/null || true
done
for process_pid in "${recorders[@]}"; do
  wait "${process_pid}" 2>/dev/null || true
done
recorders=()

kill -INT "${estimator_pid}" 2>/dev/null || true
wait "${estimator_pid}" 2>/dev/null || true
estimator_pid=''
sleep 2

if [[ -d "${run_dir}/glim_dump" ]]; then
  for trajectory in odom_imu traj_imu odom_lidar traj_lidar; do
    if [[ -f "${run_dir}/glim_dump/${trajectory}.txt" ]]; then
      cp "${run_dir}/glim_dump/${trajectory}.txt" \
        "${run_dir}/${trajectory}.txt"
    fi
  done
fi

glim_csv_lines=$(wc -l <"${run_dir}/glim_odometry.csv")
if (( glim_csv_lines > 1 )); then
  glim_rows=$((glim_csv_lines - 1))
  glim_rows_source=glim_cbs_odometry
elif [[ -s "${run_dir}/odom_imu.txt" ]]; then
  # With CBS disabled GLIM deliberately does not advertise /glim/cbs/odometry.
  # Its native trajectory dump is still the authoritative estimator output.
  glim_rows=$(wc -l <"${run_dir}/odom_imu.txt")
  glim_rows_source=glim_native_dump
else
  glim_rows=0
  glim_rows_source=missing
fi
kimera_rows=$(($(wc -l <"${run_dir}/kimera_odometry.csv") - 1))
python3 "${experiment_dir}/coverage_checks.py" "${run_dir}" "${duration_sec}" "${run_root}/${run_name%_source}_receiver/events.csv"
kimera_duration_sec=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["native_kimera"]["span_s"])' "${run_dir}/source_coverage.json")
ekf_rows=0
if [[ "${presentation_enable}" == true && -f "${run_dir}/ekf_odometry.csv" ]]; then
  ekf_rows=$(($(wc -l <"${run_dir}/ekf_odometry.csv") - 1))
fi
g2k_messages=$(rosbag info --yaml "${run_dir}/recordings/cbs.bag" | \
  awk '/topic: \/kimera\/cbs\/odom_belief_in/{found=1} found && /messages:/{print $2; exit}')
g2k_messages="${g2k_messages:-0}"
diagnostic_logs=("${run_dir}/estimators.log")
if [[ "${diagnostic_node_output}" == log ]]; then
  mapfile -d '' node_logs < <(find "${run_dir}/ros_logs" -type f \
    -name '*stdout.log' -print0 2>/dev/null)
  diagnostic_logs+=("${node_logs[@]}")
fi
g2k_injected=$(grep -h \
  'CBS_BPSAM_ODOM_ADD_ROW_G2K.*sender_odom_queued' \
  "${diagnostic_logs[@]}" | wc -l || true)
k2g_injected=$(grep -h \
  'GLIM_CBS_ODOM_INJECT_ROW.*active_window_temporary_graph_queued' \
  "${diagnostic_logs[@]}" | wc -l || true)
kimera_last_index=$(tail -n 1 "${run_dir}/kimera_odometry.csv" | cut -d, -f2)
kimera_last_index="${kimera_last_index:-0}"
g2k_applied_max_target=0
g2k_applied_tail_gap=0
if [[ "${cbs_enable}" == true ]]; then
  g2k_applied_max_target=$(grep -h \
    'CBS_ODOM_FACTOR_COVARIANCE_ROW,G2K.*active_window_temporary_odom_applied' \
    "${diagnostic_logs[@]}" 2>/dev/null | \
    sed -E 's/.*G2K,k,g,x[0-9]+,x([0-9]+),.*/\1/' | \
    sort -n | tail -n 1 || true)
  g2k_applied_max_target="${g2k_applied_max_target:-0}"
  g2k_applied_tail_gap=$((kimera_last_index - g2k_applied_max_target))
fi

min_glim_rows=350
min_kimera_rows=150
if (( glim_rows < min_glim_rows || kimera_rows < min_kimera_rows )); then
  echo "Insufficient estimator output: GLIM=${glim_rows}, Kimera=${kimera_rows}." >&2
  exit 6
fi
if ! awk -v observed="${kimera_duration_sec}" -v requested="${duration_sec}" \
  -v ratio="${min_estimator_coverage_ratio}" \
  'BEGIN { exit !(observed >= requested * ratio) }'; then
  echo "Insufficient Kimera temporal coverage: ${kimera_duration_sec}s of ${duration_sec}s requested (minimum ratio ${min_estimator_coverage_ratio})." >&2
  exit 6
fi
if [[ "${presentation_enable}" == true ]] && (( ekf_rows < 10 )); then
  echo "Insufficient EKF output: rows=${ekf_rows}." >&2
  exit 6
fi
if [[ "${cbs_enable}" == true ]]; then
  case "${cbs_direction}" in
    bidirectional)
      if (( g2k_messages < 1 || g2k_injected < 1 || k2g_injected < 1 )); then
        echo "Bidirectional CBS expected but not proven: G2K messages=${g2k_messages}, G2K injected=${g2k_injected}, K2G injected=${k2g_injected}." >&2
        exit 7
      fi
      ;;
    g2k_only)
      if (( g2k_messages < 1 || g2k_injected < 1 || k2g_injected != 0 )); then
        echo "G2K-only CBS expected but not proven: G2K messages=${g2k_messages}, G2K injected=${g2k_injected}, K2G injected=${k2g_injected}." >&2
        exit 7
      fi
      ;;
    k2g_only)
      if (( g2k_injected != 0 || k2g_injected < 1 )); then
        echo "K2G-only CBS expected but not proven: G2K injected=${g2k_injected}, K2G injected=${k2g_injected}." >&2
        exit 7
      fi
      ;;
  esac
fi
if [[ "${kimera_receive_external_beliefs}" == true ]] && (( g2k_applied_tail_gap > 15 )); then
  echo "CBS G2K did not cover the receiver tail: last Kimera x${kimera_last_index}, latest applied x${g2k_applied_max_target}." >&2
  exit 7
fi
if [[ "${cbs_enable}" == false ]] && \
   (( g2k_injected != 0 || k2g_injected != 0 )); then
  echo "CBS OFF unexpectedly injected external factors: G2K=${g2k_injected}, K2G=${k2g_injected}." >&2
  exit 7
fi

mode=cbs_off
direction=none
if [[ "${cbs_enable}" == true ]]; then
  mode=cbs_on
  direction="${cbs_direction}"
fi
printf '%s\n' \
  "dataset=GrandTour" \
  "mission=${mission}" \
  "mode=${mode}" \
  "direction=${direction}" \
  "kimera_mode=${kimera_mode}" \
  "bag_start_sec=${bag_start_sec}" \
  "seed_clock_secs=${seed_clock_secs}" \
  "sensor_window_sec=${duration_sec}" \
  "bag_rate=${bag_rate}" \
  "glim_factor_graph_stride=${glim_factor_graph_stride}" \
  "kimera_factor_graph_stride=${kimera_factor_graph_stride}" \
  "max_horizon_pairs_per_update=${max_horizon_pairs_per_update}" \
  "odom_publish_stride=${odom_publish_stride}" \
  "cbs_belief_receive_start_delay_sec=${cbs_belief_receive_start_delay_sec}" \
  "glim_publish_cbs_beliefs=${local_belief_publish}" \
  "kimera_publish_cbs_beliefs=${local_belief_publish}" \
  "glim_receive_cbs_beliefs=${glim_receive_external_beliefs}" \
  "kimera_receive_cbs_beliefs=${kimera_receive_external_beliefs}" \
  "glim_cbs_mode=${glim_cbs_mode}" \
  "kimera_cbs_odom_factor_mode=active_window_temporary" \
  "experiment=gnssfgo_e1b4a_timing40_20261012" \
  "glim_sensor_pair=hesai+cpt7_imu" \
  "kimera_sensor_pair=${kimera_sensor_pair}" \
  "glim_body_frame=cpt7_imu" \
  "kimera_body_frame=${kimera_body_frame}" \
  "kimera_calibration_overlay=${kimera_calibration_overlay}" \
  "external_frame_transform=${external_frame_transform}" \
  "cbs_horizon_sec=${cbs_odom_horizon_sec}" \
  "kimera_belief_timestamp_tolerance_sec=0.05" \
  "kimera_min_intra_keyframe_time_sec=0.19" \
  "kimera_receiver_audit_max_samples=${kimera_receiver_audit_max_samples}" \
  "kimera_receiver_audit_stride=${kimera_receiver_audit_stride}" \
  "glim_receiver_audit_max_samples=${glim_receiver_audit_max_samples}" \
  "glim_receiver_audit_stride=${glim_receiver_audit_stride}" \
  "cbs_covariance=schur_relative_between" \
  "cbs_protocol=${cbs_protocol}" \
  "diagnostic_node_output=${diagnostic_node_output}" \
  "cbs_factor_mode=active_window_temporary" \
  "glim_rows=${glim_rows}" \
  "glim_rows_source=${glim_rows_source}" \
  "kimera_rows=${kimera_rows}" \
  "kimera_duration_sec=${kimera_duration_sec}" \
  "min_estimator_coverage_ratio=${min_estimator_coverage_ratio}" \
  "ekf_rows=${ekf_rows}" \
  "presentation_enable=${presentation_enable}" \
  "rerun_recording_id=${recording_id}" \
  "g2k_messages=${g2k_messages}" \
  "g2k_injected=${g2k_injected}" \
  "k2g_injected=${k2g_injected}" \
  "g2k_applied_max_target=${g2k_applied_max_target}" \
  "g2k_applied_tail_gap=${g2k_applied_tail_gap}" \
  >"${run_dir}/run_metadata.txt"

if [[ "${presentation_enable}" == true ]]; then
  score_tool="${project_root}/src/cbsms/tools/score_grandtour_prism.py"
  python3 "${score_tool}" \
    --estimate "${run_dir}/odom_imu.txt" --estimate-format tum \
    --source-frame cpt7_imu --reference-bag "${reference_bag}" \
    --output "${run_dir}/evaluation/glim_metrics_prism.json" \
    --aligned-csv "${run_dir}/evaluation/glim_prism_aligned.csv" \
    >"${run_dir}/evaluation/glim_scorer_stdout.json"
  python3 "${score_tool}" \
    --estimate "${run_dir}/kimera_odometry.csv" --estimate-format kimera_csv \
    --source-frame "${kimera_source_frame}" --reference-bag "${reference_bag}" \
    --output "${run_dir}/evaluation/kimera_metrics_prism.json" \
    --aligned-csv "${run_dir}/evaluation/kimera_prism_aligned.csv" \
    >"${run_dir}/evaluation/kimera_scorer_stdout.json"
  python3 "${score_tool}" \
    --estimate "${run_dir}/ekf_odometry.csv" --estimate-format kimera_csv \
    --source-frame cpt7_imu --reference-bag "${reference_bag}" \
    --output "${run_dir}/evaluation/ekf_metrics_prism.json" \
    --aligned-csv "${run_dir}/evaluation/ekf_prism_aligned.csv" \
    >"${run_dir}/evaluation/ekf_scorer_stdout.json"
fi

touch "${run_dir}/complete.marker"
echo "GrandTour ETH-1 run complete: ${run_dir}"
echo "GLIM rows=${glim_rows}, Kimera rows=${kimera_rows}, EKF rows=${ekf_rows}, G2K messages=${g2k_messages}, G2K injected=${g2k_injected}, K2G injected=${k2g_injected}"
