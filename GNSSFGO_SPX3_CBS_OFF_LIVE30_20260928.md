# SPX-3 CBS-OFF live 30-second run — 2026-09-28

## Outcome and scope

Completed one **native GLIM + native Kimera + live gnssFGO** CBS-OFF run on GrandTour SPX-3, mission `2024-11-02-17-43-10`, bag seconds `[32, 62]`, played at `0.10×`. The native estimators and receiver ran concurrently. This is a bounded integration test, not the full-sequence performance comparison and not an EKF run. No GNSS factors were used.

The successful run is [`gnssfgo_spx3_cbs_off_live30_verified_20260928`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928). Earlier directories with similar names were **aborted transport-debug attempts** and are not included in the results below.

## Frozen setup

- Native GLIM: Hesai LiDAR + CPT7 IMU, same SPX-3 source preset and 0.20 s horizon as the classic protocol.
- Native Kimera: front Alphasense stereo + shared CPT7 IMU, 25-state window and classic CBS protocol.
- CBS **OFF**: both sources published diagnostic beliefs, but neither applied external source-to-source beliefs (`g2k_injected=0`, `k2g_injected=0`). The source runner reported 290 G→K messages, all observe-only.
- Receiver: source-independent 100 ms support grid; ALL_RIGHT CPT7; Combined IMU; WNOJ `Qc=10000`; full-history iSAM2; 59 IMU startup samples; full source 6×6 covariance, scale 1; stable-ID revision replacement. No gating, robust loss, health weighting, or GNSS.
- Exact configuration: [`grandtour_spx3_cbs_off_live30_20260928.json`](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_off_live30_20260928.json), [`run_metadata.txt`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/run_metadata.txt), and [`manifest.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/manifest.json).

The live adapter follows ROS `/boxi/cpt7/imu` and tails the native source revision logs. Its transport-only fixes for this run were duplicate identical IMU sample handling, sub-millisecond ROS `/clock` callback ordering, a run-local `ROS_LOG_DIR`, and canonicalizing the ROS `latest` log symlink so one GLIM log is not counted twice. These did not change source estimation or receiver factor mathematics.

## Source/receiver accounting

| Item | Result |
| --- | ---: |
| CPT7 IMU samples transported | 3,000 |
| Native GLIM odometry rows | 292 |
| Native Kimera odometry rows | 149 |
| GLIM revision messages transported | 12,730 |
| Kimera revision messages transported | 3,400 |
| Distinct active GLIM receiver factors | 290 |
| Distinct active Kimera receiver factors | 146 |
| Receiver factor insertions / replacements | 16,080 / 15,644 |
| Fused support states | 295 |
| Maximum adapter queue / queue at end | 77 / 0 |
| Unsupported source messages | 50 early Kimera revisions, all `before_initialization` |
| Receiver wall time | 346.4 s, including 0.1× playback and source shutdown |

See [`transport_complete.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/transport_complete.json), [`source_accounting.csv`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/receiver/source_accounting.csv), and [`summary.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/receiver/summary.json). The `glim_beliefs_loaded=0` and `kimera_beliefs_loaded=0` fields in the summary are legacy file-input counters; the live source use is the 436 accepted beliefs and the separate source-accounting file.

## Short-window trajectory scores

Existing GrandTour scorers were used on the same nominal interval, `1730565903.589`–`1730565932.672` UTC seconds. Prism ATE uses the known CPT7-to-prism lever arm and rigid SE(3) Kabsch alignment without scale. One-second RPE uses the retained IE-TC reference. RPE has the same 281 valid pairs for every row. ATE is scored on each trajectory's own sampled timestamps within the same bounds, so its matched pose counts differ slightly; these are **pilot diagnostics**, not a strict identical-sample paired ATE comparison.

| Trajectory | ATE RMSE (m) | 1 s t-RPE RMSE (m) | 1 s r-RPE RMSE (deg) | Prism-matched poses | Full prism-frame path (m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Native GLIM | 0.006044 | 0.006078 | 0.218076 | 284 | 23.145 |
| Native Kimera ROS odometry | 0.536418 | 0.301291 | 0.803676 | 142 | 22.059 |
| Live gnssFGO, online/as-played | 0.072977 | 0.051061 | 3.485086 | 286 | 23.621 |
| Live gnssFGO, final retrospectively revised | 0.027501 | 0.016605 | 0.417489 | 286 | 23.041 |

The fused final graph improves considerably over its online/as-played output, but both are less accurate than native GLIM on this short CBS-OFF segment. No settings were changed after observing these results.

Source trajectories: [`odom_imu.txt`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/odom_imu.txt), [`kimera_odometry.csv`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/kimera_odometry.csv). Fused trajectories: [`online_trajectory.tum`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/receiver/online_trajectory.tum), [`trajectory.tum`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/receiver/trajectory.tum). Scorer JSON files are under [`evaluation`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/evaluation).

## Validation

- Native runner exited 0 and wrote `complete.marker`; receiver and transport exited 0 and wrote `complete.marker`/`transport_complete.json`.
- All 16,130 revision messages were journaled; accepted source IDs were 290 GLIM + 146 Kimera. No source-to-source CBS factors were injected.
- Replaying the captured [`events.csv`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver/events.csv) through the same receiver binary produced byte-identical final/online trajectory, final/online state, and source-accounting files. Final trajectory SHA-256: `6a1e61414a0e9a937aa8ef870ae6413db3f6959cceb43ef28b7ac4ba78c8c9f4`.
- Focused receiver protocol/stream tests: 5 passed in the existing gnssFGO container. Host Python lacked NumPy, so host execution was not used.
- Repository HEAD during run: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. The working tree already contains ongoing experimental changes; this report does not claim a clean tree or a committed run.

## Exact launch sequence

With a Noetic ROS master on `http://127.0.0.1:11368` and `/use_sim_time=true`:

```bash
python3 /home/yeranis/repos/gnssFGO/tools/launch_grandtour_live_receiver.py \
  --config /home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_off_live30_20260928.json \
  --source-run /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source \
  --out /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/receiver \
  --start
```

After `transport_ready.marker`, the source runner was started in `cbsms_ws_gpu` with the saved SPX-3 source preset and these run-specific overrides: `CBSMS_RUN_ROOT=/workspace/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928`, `CBSMS_RUN_NAME=spx3_source`, `CBSMS_CBS_ENABLE=false`, `CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour/SPX-3`, and `CBSMS_BAG_DURATION_SEC=30.0`. The saved source preset supplies bag offset 32 s, playback 0.1×, and run-local ROS logging. The exact launched source script is retained as [`experiment_runner_used.sh`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/experiment_runner_used.sh). After source completion, `receiver/STOP` triggered a final transport drain and receiver shutdown.
