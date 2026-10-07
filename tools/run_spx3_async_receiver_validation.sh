#!/usr/bin/env bash
set -euo pipefail
# Offline receiver only. Reuse frozen source recordings; never launch estimators.
target_root=${1:?usage: run_spx3_async_receiver_validation.sh NEW_OUTPUT_ROOT}
test ! -e "$target_root"
mkdir -p "$target_root"
replay_binary=/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay
capture=/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/spx3_graph_inspection_20260928/spx3_classic_on_graphs_first60_start32
receiver_inputs="$capture/evaluation/receiver_inputs"
imu_input=/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv
sha256sum "$replay_binary" "$imu_input" "$receiver_inputs/glim/first.csv" \
  "$receiver_inputs/kimera/first.csv" "$receiver_inputs/joint_revisions.csv" > "$target_root/input_hashes.txt"
common=(--imu "$imu_input" --timeline record --solver isam2 --revision-beliefs "$receiver_inputs/joint_revisions.csv")
"$replay_binary" "${common[@]}" --glim-beliefs "$receiver_inputs/glim/first.csv" \
  --kimera-beliefs "$receiver_inputs/kimera/first.csv" --mode joint \
  --out "$target_root/legacy" > "$target_root/legacy.log" 2>&1 &
legacy_pid=$!
"$replay_binary" "${common[@]}" --mode sources --support-policy uniform_imu \
  --receiver-duration-ns 60000000000 --graph-snapshots-out "$target_root/uniform_graph" \
  --out "$target_root/uniform" > "$target_root/uniform.log" 2>&1
wait "$legacy_pid"
"$replay_binary" "${common[@]}" --mode sources --support-policy uniform_imu \
  --receiver-duration-ns 60000000000 --out "$target_root/repeat" > "$target_root/repeat.log" 2>&1
for product in trajectory.tum states.csv online_trajectory.tum online_states.csv; do
  cmp "$target_root/legacy/$product" "$capture/evaluation/gnssfgo_joint/$product"
  cmp "$target_root/uniform/$product" "$target_root/repeat/$product"
done
for product in support_clock.csv causal_dispatch.csv support_accelerations.csv; do
  cmp "$target_root/uniform/$product" "$target_root/repeat/$product"
done
sha256sum -c "$target_root/input_hashes.txt" > "$target_root/hash_validation.txt"
echo 'Legacy byte identity, independent repeat byte identity, and input hashes PASS.'
