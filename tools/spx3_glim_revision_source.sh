#!/usr/bin/env bash
set -euo pipefail

# CBS-OFF SPX-3 GLIM-only diagnostic reproduction of the matched source run.
# The endpoint audit only logs active poses and full mean/covariance revisions.
workspace=/workspace/cbs_gtsam4.3
source_run="${workspace}/runs/classic_cbs_restoration_20260925/spx3_off_native"
run_dir="${CBS_SPX3_REVISION_RUN_DIR:-${workspace}/runs/_spx3_glim_revision_preservation_20260926}"
dataset="${workspace}/src/datasets/GrandTour/SPX-3"
mission=2024-11-02-17-43-10
bag_start_sec="${CBS_SPX3_BAG_START_SEC:-32.0}"
bag_duration_sec="${CBS_SPX3_BAG_DURATION_SEC:-134.0}"
bag_rate="${CBS_SPX3_BAG_RATE:-0.30}"
port=11433
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP=127.0.0.1
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"

if [[ -e "${run_dir}/estimators.log" ]]; then
  echo "Refusing to overwrite existing diagnostic run" >&2
  exit 2
fi
mkdir -p "${run_dir}/recordings"
pids=()
cleanup() {
  set +e
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do
    kill -TERM "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

roscore -p "${port}" >"${run_dir}/roscore.log" 2>&1 &
pids+=("$!")
for attempt in $(seq 1 100); do
  rosparam get /rosversion >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam set /use_sim_time true

roslaunch glim_ros grandtour_eth1_glim_kimera_cbs.launch \
  glim_config_path:="${source_run}/glim_config" \
  kimera_params_folder:="${source_run}/kimera_profile" \
  kimera_enable:=false enable_ekf:=false cbs_enable:=false \
  glim_publish_cbs_beliefs:=true glim_receive_cbs_beliefs:=false \
  glim_cbs_mode:=observe_only \
  glim_dump_path:="${run_dir}/dump" \
  glim_factor_graph_inspector_enable:=false \
  kimera_factor_graph_inspector_enable:=false \
  cbs_odom_horizon_sec:=0.20 \
  cbs_odom_max_horizon_pairs_per_update:=1 \
  cbs_odom_publish_stride:=1 \
  glim_export_endpoint_audit_enable:=true \
  >"${run_dir}/estimators.log" 2>&1 &
estimator_pid=$!
pids+=("${estimator_pid}")

rostopic pub -1 /clock rosgraph_msgs/Clock \
  '{clock: {secs: 1730565871, nsecs: 0}}' >/dev/null
ready=false
for attempt in $(seq 1 400); do
  if ! kill -0 "${estimator_pid}" 2>/dev/null; then
    echo "GLIM launch exited during startup" >&2
    exit 3
  fi
  info=$(rosnode info /glim_rosnode 2>/dev/null || true)
  if grep -q '/boxi/hesai/points' <<<"${info}" && \
     grep -q '/boxi/cpt7/imu' <<<"${info}"; then
    ready=true
    break
  fi
  sleep 0.2
done
[[ "${ready}" == true ]] || { echo "GLIM not ready" >&2; exit 4; }
rosparam get /glim_rosnode >"${run_dir}/resolved_glim_params.yaml"

rosbag record --buffsize=256 -O "${run_dir}/recordings/cbs.bag" \
  /kimera/cbs/odom_belief_in /glim/cbs/odometry \
  >"${run_dir}/recorder.log" 2>&1 &
pids+=("$!")
sleep 2

rosbag play --quiet --clock --delay=2.0 --rate="${bag_rate}" \
  --start="${bag_start_sec}" --duration="${bag_duration_sec}" \
  "${dataset}/${mission}_hesai.bag" \
  "${dataset}/${mission}_cpt7_imu.bag" \
  --topics /boxi/hesai/points /boxi/cpt7/imu \
  >"${run_dir}/rosbag.log" 2>&1 &
bag_pid=$!
pids+=("${bag_pid}")
wait "${bag_pid}"
sleep 25
printf 'playback_completed\n' >"${run_dir}/completion.txt"
