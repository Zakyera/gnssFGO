# GrandTour EIG-2: native sources and causal gnssFGO graph, CBS-OFF/ON

Date: 2026-09-23. This extends the ETH1 experiment to the locally retained EIG-2 source runs; it does **not** change GLIM, Kimera, graph factors, covariances, IMU handling, or scoring. The saved GLIM/Kimera source runs were completed on 2026-09-21. The gnssFGO graphs here are **offline causal record-arrival replays of exported beliefs**, not live estimators inside the source processes. No GNSS factors are used.

## Input identity and settings

- GrandTour EIG-2 mission `2024-11-03-13-59-54`; 305.0 s sensor window beginning 5.7 s into the bag, 0.10× bag playback. Source settings: Hesai + CPT7 IMU for GLIM, Alphasense front stereo + the same CPT7 IMU for Kimera, 0.20 s CBS belief horizon, graph-inspector strides 5/5. The copied GLIM configuration and Kimera profile trees compare byte-for-byte with the ETH1 CBS-ON presentation run; the sequence, window, and playback rate differ. These profiles use the source estimators' retained IMU settings; this report does not relabel them as an experimental `ALL_RIGHT` source replay. gnssFGO's own right-interval-end IMU support is recorded in its summary.
- Existing OFF run: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_off_full305_start5p7_rate010_20260921`.
- Existing bidirectional ON run: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_on_bidirectional_full305_start5p7_rate010_20260921`.
- OFF injected zero CBS source factors. ON recorded 1,517 GLIM→Kimera and 1,519 Kimera→GLIM factors in the source runs. Source trajectories have 3,042 GLIM and 1,523 Kimera rows each. Both source Rerun recordings contain their factor-graph inspectors.
- Beliefs were exported with the unchanged `export_cbs_unique_relative_beliefs.py` from each saved `recordings/cbs.bag` into `/home/yeranis/repos/V4RL/cbs_eig2_gnssfgo_20260923/{off,on}/beliefs/`. Each export has 3,040 unique GLIM and 1,521 unique Kimera edges. The ON/OFF raw IMU CSV is byte-identical, SHA-256 `c4d7b68674849c4be3b3b83198050c564ff310fa2167a42aa73217321b933fca`.
- Ordered belief CSV SHA-256 (OFF GLIM, OFF Kimera, ON GLIM, ON Kimera): `4ba70e11298ba0f8337fb9806b7986f3ecbc2eb4eee817b16743059abfa43782`, `acbc1f09fcb8426cd24410e638a6dea7046afe98ccfb5aac9188c65ee21ab707`, `b3cef39b4ca21be8101a0cbf349424ed7ff96530bf2da716997486f32761ff99`, `93852da2420c64aacdecedeaf6a0cfb3a72e97bc5c524629a50e2076eb4a8b21`.

## Graph execution

Unchanged executable: `/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay`, SHA-256 `0a5247085378bc54434def0890d2e143f102dfc323c285de65a91fb3f78f35b8`. Both modes used `--timeline record --solver isam2`, the same raw IMU CSV, and the respective mode's exported GLIM/Kimera belief CSVs. `--mode glim` is a graph-framework control; `--mode joint` is GLIM+Kimera fusion. For example, in the ROS Humble container:

```bash
source /opt/ros/humble/setup.bash
grandtour_ct_replay --imu INPUT_IMU_CSV --glim-beliefs MODE/glim_beliefs.csv --kimera-beliefs MODE/kimera_beliefs.csv --mode joint --timeline record --solver isam2 --out OUTPUT/joint
```

The concrete `INPUT_IMU_CSV` was `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/grandtour_eig2_causal_sync_20260921/on/input/imu/imu.csv`; the actual full executable path is above. Graph outputs are in `/home/yeranis/results/gnssfgo_grandtour_eig2_20260923/{off,on}/{glim,joint}/`. Each joint graph loaded 3,040 GLIM and 1,521 Kimera beliefs; 3,040 GLIM plus 1,518 Kimera factors were accepted, and three Kimera edges were skipped for absent graph support. There were 3,042 support states. OFF and ON joint wall times were 502.9 s and 402.7 s, respectively. Both GLIM-only graphs accepted all 3,040 GLIM factors. There was no covariance scaling, gating, robust loss, or post-result adjustment.

## Evaluation

ATE is AP20 prism-position RMSE after the established CPT7→prism lever arm and no-scale SE(3) Kabsch alignment. The prism provides 193.6 s of matched elapsed coverage, **not** the entire 304.1 s graph/source trajectory; this limitation applies to every row. Translational and rotational RPE are 1-second relative-pose RMSE against the retained CPT7 Inertial Explorer tightly coupled reference, 3,030 valid pairs over the specified 304 s interval. Native source trajectories and the gnssFGO graph are distinct estimators; do not interpret the latter as native GLIM performance.

| CBS | Trajectory | Prism ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---:|---:|---:|---:|
| OFF | native GLIM | 0.045264 | 0.018241 | 0.608075 | 186.368 |
| OFF | native Kimera | 1.309657 | 0.048621 | 0.888068 | 180.880 |
| OFF | gnssFGO GLIM-only | 0.405834 | 0.019592 | 0.634771 | 185.877 |
| OFF | gnssFGO GLIM+Kimera | 0.306152 | 0.014673 | 0.632098 | 185.129 |
| ON | native GLIM | 0.042980 | 0.034541 | 0.639596 | 192.238 |
| ON | native Kimera | 0.479174 | 0.047135 | 0.868122 | 185.217 |
| ON | gnssFGO GLIM-only | 0.482159 | 0.034096 | 0.677750 | 185.189 |
| ON | gnssFGO GLIM+Kimera | 0.387641 | 0.029204 | 0.675979 | 184.986 |

Adding Kimera helps **within the gnssFGO graph** in each CBS mode: OFF ATE 0.406→0.306 m and t-RPE 0.01959→0.01467 m; ON ATE 0.482→0.388 m and t-RPE 0.03410→0.02920 m. Nevertheless, CBS-ON fusion is worse than CBS-OFF fusion on EIG-2 (ATE 0.388 versus 0.306 m; t-RPE 0.02920 versus 0.01467 m). Most importantly, native GLIM remains far more accurate in prism ATE (about 0.043–0.045 m) than either graph fusion. The graph's relative-belief reconstruction must not be visually or numerically conflated with native GLIM's saved optimized trajectory. This result does not establish a general CBS-ON benefit across sequences.

Evaluator JSON and aligned CSVs are at `/home/yeranis/repos/V4RL/cbs_eig2_gnssfgo_20260923/{off,on}/metrics/`. The unchanged scorers are `score_grandtour_prism.py` and `score_grandtour_relative_pose.py` in `/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/`, run inside the existing ROS Noetic container with `source /opt/ros/noetic/setup.bash`. The prism bag is `2024-11-03-13-59-54_ap20_prism_position.bag`; RPE uses `2024-11-03-13-59-54_cpt7_ie_tc.bag`.

## Rerun presentations

The two verified, time-progressive presentations are:

- [CBS-OFF Rerun](/home/yeranis/repos/V4RL/cbs_eig2_gnssfgo_20260923/off/GrandTour_EIG2_CBS_OFF_GNSSFGO_ANIMATED.rrd)
- [CBS-ON Rerun](/home/yeranis/repos/V4RL/cbs_eig2_gnssfgo_20260923/on/GrandTour_EIG2_CBS_ON_GNSSFGO_ANIMATED.rrd)

Each contains the saved source GLIM/Kimera factor-graph inspectors, their external-factor count plots, native GLIM and Kimera paths, gnssFGO fused path, prism GT, and saved GLIM submaps. Trajectory trails and current-position markers reveal along the common timeline; completed submaps reveal at their saved completion time. This is not a live individual-scan cloud. The common-start position shift in the *display only* is explicitly noted in the Rerun panel; all metrics above use the original unshifted scorer alignments. ON has 91 displayed submaps/227,162 sampled points; OFF has 89/222,340. The saved map→IMU-odometry position checks are below 0.072 mm. Both combined RRDs passed `rerun rrd verify`; validation JSONs sit beside them. Their SHA-256 values are ON `8e13407eb41bd31fcfe2b033a1934f134cc6b5947ee56c170f3d47913e9af142`, OFF `490f147969bc35cb034345ea4087a13e292562cec6f6b1742ba73210dd830cee`.

The only source change for this extension generalizes the existing ETH1 **visualization helper** to derive the GrandTour sequence label from the source run directory. It changes no estimator, factor, or evaluator. Both source profile trees and saved source RRDs remain untouched.
