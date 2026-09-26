#!/usr/bin/env bash
set -eo pipefail

# Run only the frozen SPX-3 GLIM receiver: same IMU, WNOJ, priors, and iSAM2.
products="${1:?usage: replay PRODUCTS_DIRECTORY}"
source /opt/ros/humble/setup.bash
set -u
replay=/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay
imu=/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv

"${replay}" --imu "${imu}" \
  --glim-beliefs "${products}/first.csv" \
  --mode glim --timeline record --solver isam2 \
  --out "${products}/full_r0"

"${replay}" --imu "${imu}" \
  --glim-beliefs "${products}/first.csv" \
  --revision-beliefs "${products}/revisions.csv" \
  --mode glim --timeline record --solver isam2 \
  --out "${products}/full_r1"

"${replay}" --imu "${imu}" \
  --glim-beliefs "${products}/last_revision.csv" \
  --mode glim --timeline physical --solver isam2 \
  --out "${products}/full_r2"
