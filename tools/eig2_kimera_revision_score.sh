#!/usr/bin/env bash
set -euo pipefail

# Run inside the existing Noetic container, after common_eval_tum is generated.
run_dir="${1:?usage: score RUN_DIRECTORY}"
workspace=/workspace/cbs_gtsam4.3
dataset="${workspace}/src/datasets/GrandTour/EIG-2"
mission=2024-11-03-13-59-54
prism="${dataset}/${mission}_ap20_prism_position.bag"
ie_tc="${dataset}/${mission}_cpt7_ie_tc.bag"
products="${run_dir}/revision_products"
evaluation="${products}/evaluation"
mkdir -p "${evaluation}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"
start=$(awk 'NR==1 {print $1}' "${products}/common_eval_tum/native_online.tum")
end=$(awk 'END {print $1}' "${products}/common_eval_tum/native_online.tum")

for trajectory in "${products}"/common_eval_tum/*.tum; do
  name=$(basename "${trajectory}" .tum)
  if [[ -s "${evaluation}/${name}_prism.json" &&
        -s "${evaluation}/${name}_rpe.json" ]]; then
    continue
  fi
  /usr/bin/python3 "${workspace}/src/cbsms/tools/score_grandtour_prism.py" \
    --estimate "${trajectory}" --estimate-format tum --source-frame cpt7_imu \
    --reference-bag "${prism}" --output "${evaluation}/${name}_prism.json" \
    --aligned-csv "${evaluation}/${name}_prism_aligned.csv" \
    > "${evaluation}/${name}_prism_stdout.json"
  /usr/bin/python3 "${workspace}/src/cbsms/tools/score_grandtour_relative_pose.py" \
    --estimate "${trajectory}" --estimate-format tum \
    --reference-bag "${ie_tc}" --start-time-sec "${start}" \
    --end-time-sec "${end}" --delta-sec 1.0 \
    --output "${evaluation}/${name}_rpe.json" \
    > "${evaluation}/${name}_rpe_stdout.json"
done
