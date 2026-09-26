#!/usr/bin/env bash
set -euo pipefail

products="${1:?usage: score PRODUCTS_DIRECTORY}"
workspace=/workspace/cbs_gtsam4.3
dataset="${workspace}/src/datasets/GrandTour/SPX-3"
mission=2024-11-02-17-43-10
prism="${dataset}/${mission}_ap20_prism_position.bag"
ie_tc="${dataset}/${mission}_cpt7_ie_tc.bag"
evaluation="${products}/evaluation"
mkdir -p "${evaluation}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"
start=$(awk 'NR==1 {print $1}' "${products}/full_r0/trajectory.tum")
end=$(awk 'END {print $1}' "${products}/full_r0/trajectory.tum")

for case in r0 r1 r2; do
  cp "${products}/full_${case}/trajectory.tum" "${products}/score_full_${case}.tum"
done
for case in r0 r1; do
  cp "${products}/full_${case}/online_trajectory.tum" \
    "${products}/score_online_${case}.tum"
done
cp "${products}/pose_r0/native_support.tum" "${products}/score_native.tum"
for case in r0 r1 r2; do
  for parity in 0 1; do
    awk -v parity="${parity}" '((NR - 1) % 2) == parity' \
      "${products}/pose_${case}/graph.tum" \
      > "${products}/score_pose_${case}_${parity}.tum"
  done
done

for tum in "${products}"/score_*.tum; do
  name=$(basename "${tum}" .tum)
  if [[ -s "${evaluation}/${name}_prism.json" &&
        -s "${evaluation}/${name}_rpe.json" ]]; then
    continue
  fi
  /usr/bin/python3 "${workspace}/src/cbsms/tools/score_grandtour_prism.py" \
    --estimate "${tum}" --estimate-format tum --source-frame cpt7_imu \
    --reference-bag "${prism}" \
    --output "${evaluation}/${name}_prism.json" \
    --aligned-csv "${evaluation}/${name}_prism_aligned.csv" \
    > "${evaluation}/${name}_prism_stdout.json"
  /usr/bin/python3 "${workspace}/src/cbsms/tools/score_grandtour_relative_pose.py" \
    --estimate "${tum}" --estimate-format tum \
    --reference-bag "${ie_tc}" --start-time-sec "${start}" \
    --end-time-sec "${end}" --delta-sec 1.0 \
    --output "${evaluation}/${name}_rpe.json" \
    > "${evaluation}/${name}_rpe_stdout.json"
done
