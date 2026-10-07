#!/usr/bin/env bash
set -eo pipefail

# The same frozen full-history gnssFGO model is used in A/B/C/D. Only the
# diagnostic source products and inclusion of Kimera differ.
products="${1:?usage: replay PRODUCTS_DIRECTORY}"
source /opt/ros/humble/setup.bash
set -u
replay=/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay
imu=/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv

for case in A B C D; do
  if [[ "${case}" == A || "${case}" == B ]]; then
    source_dir="${products}/off"
  else
    source_dir="${products}/on"
  fi
  if [[ "${case}" == A || "${case}" == C ]]; then
    "${replay}" --imu "${imu}" \
      --glim-beliefs "${source_dir}/glim/first.csv" \
      --revision-beliefs "${source_dir}/glim/revisions.csv" \
      --mode glim --timeline record --solver isam2 \
      --out "${products}/${case}"
  else
    "${replay}" --imu "${imu}" \
      --glim-beliefs "${source_dir}/glim/first.csv" \
      --kimera-beliefs "${source_dir}/kimera/first.csv" \
      --revision-beliefs "${source_dir}/joint_revisions.csv" \
      --mode joint --timeline record --solver isam2 \
      --out "${products}/${case}"
  fi
done
