# SPX-3 classic CBS-ON live 30-second run and Rerun recording

## Result

Completed a **live, concurrent** GrandTour SPX-3 run: native GLIM (Hesai + CPT7), native Kimera (front stereo + CPT7), classic bidirectional CBS exchange, and the source-independent gnssFGO receiver. Mission `2024-11-02-17-43-10`, bag seconds 32–62, playback 0.10×. No GNSS or EKF fusion was used.

The validated, saved [three-factor-graph and trajectory Rerun recording](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/visualization/SPX3_CLASSIC_ON_LIVE30_THREE_GRAPHS_V2.rrd) was verified by `rerun rrd verify`. It is also open in the desktop Rerun viewer. Reopen with:

```bash
rerun --new --memory-limit 8GB /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/visualization/SPX3_CLASSIC_ON_LIVE30_THREE_GRAPHS_V2.rrd
```

The top row shows the recorded **actual fixed-lag GLIM and Kimera graph inspectors** and gnssFGO's complete growing optimized iSAM2 graph. The source panels include their injected external CBS factor classes. In the receiver panel, blue edges are Combined IMU, purple WNOJ, green GLIM, and orange Kimera; its source factor keys and optimized pose coordinates come from the saved graph snapshots. The bottom-left panel shows timestamp-revealed AP20 prism ground truth, original/native GLIM, original/native Kimera ROS odometry, and the **final retrospectively revised** fused trajectory. The online/as-played fused trajectory is separately saved, not misrepresented as the final path known in real time. The factor-node vertical offsets are visualization layout, not information weight.

## Matched setup and source accounting

The exact [run metadata](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/spx3_source/run_metadata.txt) differs from the completed 30-second OFF run only in CBS mode/direction, receiving/injection flags, and resulting exchange counts. Both used the same mission/window, Hesai, front stereo, CPT7, saved source profiles, 0.20 s horizon, classic protocol, one pair per update, `schur_relative_between` covariance, and logging-only source graph inspectors. Presentation EKF was disabled. The run-specific receiver [configuration](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_on_live30_20260928.json) preserves ALL_RIGHT, Combined IMU, WNOJ `Qc=10000`, full-history iSAM2, 100 ms independent support grid, 59 startup IMU samples, native 6×6 source covariance at scale 1, and stable-ID factor replacement. Receiver graph snapshots were enabled solely for the requested visualization.

| Check | CBS-ON result |
| --- | ---: |
| GLIM/Kimera native odometry rows | 292 / 149 |
| Classic G→K / K→G injected factors | 144 / 145 |
| CPT7 IMU samples transported | 3,000 |
| GLIM/Kimera revision messages transported | 12,909 / 3,400 |
| Distinct active receiver GLIM/Kimera factors | 290 / 146 |
| Receiver replacements | 15,823 |
| Fused support states | 295 |
| Optimized graph snapshots | 296 |
| Active source factors in final graph snapshot | 436 |
| Unsupported early Kimera revision messages | 50, all before receiver initialization |
| Pending receiver events at end | 0 |

The source and receiver each exited successfully and wrote completion markers. [Transport completion](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/transport_complete.json), [receiver summary](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/receiver/summary.json), [source accounting](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/receiver/source_accounting.csv), and [viewer validation](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/visualization/SPX3_CLASSIC_ON_LIVE30_THREE_GRAPHS_V2_validation.json) retain the detailed counts and input SHA-256 hashes. The saved source inspector RRD was compacted and verified before merge; the merged RRD contains explicit GLIM `cbs_k2g_pose_between` and Kimera `cbs_g2k_pose_between` entities as well as all four trajectory trails. Merged RRD SHA-256: `37d9e0a86dd53622cb9275f90553ac006a1ffc429d0861df23f10216c241959b`.

## Short-window trajectory evaluation

The following is a **bounded diagnostic**, scored with the existing GrandTour AP20-prism no-scale SE(3) Kabsch evaluator and 1 s IE-TC RPE evaluator over the same nominal time bounds, `1730565903.589`–`1730565932.672`, as the live OFF test. Every row has 281 valid RPE pairs. ATE uses each trajectory's own sample timestamps within those bounds (native Kimera has half-rate poses), so the rows do not share an identical ATE sample mask. Do not generalize this 30-second section to a full sequence.

| Case | ATE RMSE (m) | 1 s t-RPE RMSE (m) | 1 s r-RPE RMSE (deg) |
| --- | ---: | ---: | ---: |
| CBS-OFF native GLIM | 0.006044 | 0.006078 | 0.218076 |
| CBS-ON native GLIM | 0.016316 | 0.025499 | 0.555751 |
| CBS-OFF native Kimera | 0.536418 | 0.301291 | 0.803676 |
| CBS-ON native Kimera | 0.075552 | 0.041145 | 0.500344 |
| CBS-OFF gnssFGO online/as-played | 0.072977 | 0.051061 | 3.485086 |
| CBS-ON gnssFGO online/as-played | 0.060593 | 0.073187 | 3.572475 |
| CBS-OFF gnssFGO final revised | 0.027501 | 0.016605 | 0.417489 |
| CBS-ON gnssFGO final revised | 0.025744 | 0.014103 | 0.633418 |

On this segment, CBS-ON helps native Kimera markedly but degrades native GLIM. Final fused ATE and translational RPE improve slightly versus OFF; final fused rotational RPE worsens. No covariance, WNOJ, IMU, prior, or CBS setting was changed in response.

Score JSONs are under the ON [evaluation directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/spx3_source/evaluation); the matched OFF score JSONs remain under the [OFF evaluation directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live30_verified_20260928/spx3_source/evaluation).

## Saved trajectories and graph inputs

- Native GLIM: [odom_imu.txt](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/spx3_source/odom_imu.txt)
- Native Kimera ROS odometry: [kimera_odometry.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/spx3_source/kimera_odometry.csv)
- gnssFGO online/as-played: [online_trajectory.tum](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/receiver/online_trajectory.tum)
- gnssFGO final revised: [trajectory.tum](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/receiver/trajectory.tum)
- Source inspector recording: [source_graphs.rrd](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/visualization/source_graphs.rrd)
- Receiver optimized graph snapshots: [graph directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live30_20260928/receiver/graph)

The combined Rerun recording is the validated `V2` file above. Earlier intermediate viewer files in the same directory were incomplete during adapter development and are not result artifacts.
