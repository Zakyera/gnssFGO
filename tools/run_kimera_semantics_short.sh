#!/usr/bin/env bash
# Bounded CBS-OFF mechanism diagnostic in the existing Noetic container.
set -euo pipefail
workspace=/workspace/cbs_gtsam4.3
run_dir="${1:?run directory required}"
profile="${2:?source profile required}"
port="${3:-11461}"
[[ ! -e "${run_dir}/estimators.log" ]] || exit 2
mkdir -p "${run_dir}/recordings" "${run_dir}/kimera_logs"
export ROS_MASTER_URI="http://127.0.0.1:${port}" ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"
export KIMERA_FIXED_LAG_SEMANTICS_AUDIT=1
unset KIMERA_CBS_REVISION_AUDIT
pids=()
cleanup() {
  set +e
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null; done
  sleep 3
  for pid in "${pids[@]}"; do kill -TERM "${pid}" 2>/dev/null; wait "${pid}" 2>/dev/null; done
}
trap cleanup EXIT INT TERM
roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 &
pids+=("$!")
for attempt in $(seq 1 100); do
  rosparam get /rosversion >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true
roslaunch kimera_vio_ros grandtour_eth1_kimera_stereo.launch \
  params_folder:="${profile}" imu_topic:=/boxi/cpt7/imu \
  base_link_frame_id:=cpt7_imu log_output:=false \
  log_output_path:="${run_dir}/kimera_logs" \
  rerun_visualizer_enable:=false rerun_factor_graph_inspector_enable:=false \
  cbs_belief_bridge_enable:=true cbs_belief_publish_enable:=true \
  cbs_belief_receive_enable:=false cbs_backend_enable:=true \
  cbs_odom_sender_mode:=time_horizon_window cbs_odom_horizon_sec:=0.20 \
  cbs_odom_max_horizon_pairs_per_update:=1 cbs_active_factor_diagnostic_enable:=true \
  >"${run_dir}/estimators.log" 2>&1 &
estimator_pid=$!
pids+=("${estimator_pid}")
rostopic pub -1 /clock rosgraph_msgs/Clock '{clock: {secs: 1730638811, nsecs: 0}}' >/dev/null
ready=false
for attempt in $(seq 1 400); do
  kill -0 "${estimator_pid}" 2>/dev/null || exit 3
  info=$(rosnode info /kimera_vio_ros/kimera_vio_ros_node 2>/dev/null || true)
  if grep -q '/grandtour/front_right/image_raw' <<<"${info}" &&
     grep -q '/boxi/cpt7/imu' <<<"${info}"; then ready=true; break; fi
  sleep 0.2
done
[[ "${ready}" == true ]] || exit 4
rosparam get /kimera_vio_ros >"${run_dir}/resolved_kimera_params.yaml"
rosbag record --buffsize=256 -O "${run_dir}/recordings/kimera.bag" \
  /kimera_vio_ros/odometry /kimera/cbs/odom_belief_out \
  >"${run_dir}/recorder.log" 2>&1 &
recorder_pid=$!
pids+=("${recorder_pid}")
sleep 2
dataset="${workspace}/src/datasets/GrandTour/EIG-2"
mission=2024-11-03-13-59-54
# Retain the same three bag start bounds. No LiDAR topic or GLIM node is consumed.
rosbag play --quiet --clock --delay=2.0 --rate=0.10 --start=5.7 --duration=30.0 \
  "${dataset}/${mission}_hesai.bag" "${dataset}/${mission}_cpt7_imu.bag" \
  "${dataset}/${mission}_alphasense.bag" \
  --topics /boxi/cpt7/imu /boxi/alphasense/front_left/image_raw/compressed \
  /boxi/alphasense/front_right/image_raw/compressed >"${run_dir}/rosbag.log" 2>&1 &
bag_pid=$!
pids+=("${bag_pid}")
wait "${bag_pid}"
sleep 20
kill -INT "${recorder_pid}"
wait "${recorder_pid}" || true
printf 'completed 30 s Kimera-only playback\n' >"${run_dir}/complete.marker"
