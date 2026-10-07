#!/usr/bin/env bash
set -euo pipefail

# Score already-produced A/B/C/D trajectories with the unchanged SPX-3 evaluators.
products="${1:?usage: score PRODUCTS_DIRECTORY}"
workspace=/workspace/cbs_gtsam4.3
dataset="${workspace}/src/datasets/GrandTour/SPX-3"
mission=2024-11-02-17-43-10
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"
mkdir -p "${products}/evaluation"

reference="${products}/A/trajectory.tum"
for case in A B C D; do
  for product in trajectory online_trajectory; do
    target="${products}/${case}/${product}.tum"
    [[ -s "${target}" ]] || { echo "Missing ${target}" >&2; exit 2; }
    if ! cmp -s <(awk '{print $1}' "${reference}") \
               <(awk '{print $1}' "${target}"); then
      echo "Graph timestamp grid differs: ${target}" >&2
      exit 3
    fi
  done
done

for mode in off on; do
  run="${workspace}/runs/spx3_full_system_revision_diagnostics_20260927/spx3_${mode}_classic_revisions_nodelog"
  for estimator in glim kimera; do
    python3 "${workspace}/src/cbsms/tools/export_odometry_csv_tum.py" \
      --input "${run}/${estimator}_odometry.csv" \
      --output "${products}/native_${mode}_${estimator}.tum" \
      --source-to-target 0 0 0 0 0 0 1
  done
done

start=$(awk 'NR==1 {print $1}' "${reference}")
end=$(awk 'END {print $1}' "${reference}")
for case in A B C D; do
  for product in trajectory online_trajectory; do
    name="${case}_${product}"
    tum="${products}/${case}/${product}.tum"
    python3 "${workspace}/src/cbsms/tools/score_grandtour_prism.py" \
      --estimate "${tum}" --estimate-format tum --source-frame cpt7_imu \
      --reference-bag "${dataset}/${mission}_ap20_prism_position.bag" \
      --output "${products}/evaluation/${name}_prism.json" \
      --aligned-csv "${products}/evaluation/${name}_prism_aligned.csv" \
      > "${products}/evaluation/${name}_prism_stdout.json"
    python3 "${workspace}/src/cbsms/tools/score_grandtour_relative_pose.py" \
      --estimate "${tum}" --estimate-format tum \
      --reference-bag "${dataset}/${mission}_cpt7_ie_tc.bag" \
      --start-time-sec "${start}" --end-time-sec "${end}" --delta-sec 1.0 \
      --output "${products}/evaluation/${name}_rpe.json" \
      > "${products}/evaluation/${name}_rpe_stdout.json"
  done
done
for mode in off on; do
  for estimator in glim kimera; do
    name="native_${mode}_${estimator}"
    tum="${products}/${name}.tum"
    python3 "${workspace}/src/cbsms/tools/score_grandtour_prism.py" \
      --estimate "${tum}" --estimate-format tum --source-frame cpt7_imu \
      --reference-bag "${dataset}/${mission}_ap20_prism_position.bag" \
      --output "${products}/evaluation/${name}_prism.json" \
      --aligned-csv "${products}/evaluation/${name}_prism_aligned.csv" \
      > "${products}/evaluation/${name}_prism_stdout.json"
    python3 "${workspace}/src/cbsms/tools/score_grandtour_relative_pose.py" \
      --estimate "${tum}" --estimate-format tum \
      --reference-bag "${dataset}/${mission}_cpt7_ie_tc.bag" \
      --start-time-sec "${start}" --end-time-sec "${end}" --delta-sec 1.0 \
      --output "${products}/evaluation/${name}_rpe.json" \
      > "${products}/evaluation/${name}_rpe_stdout.json"
  done
done
