# SPX-3 CBS-OFF gnssFGO control and exact configuration

Date: 2026-09-25. This is the **CBS-OFF** half of a prospective matched OFF/ON comparison. It reuses the completed native SPX-3 OFF run and the existing GrandTour CT replay; no GLIM/Kimera/EKF/gnssFGO source was changed, no GNSS factor was added, and no CBS-ON gnssFGO graph was run here. The graph is an **offline causal record-arrival replay** of saved relative beliefs, not a live node alongside GLIM and Kimera.

## Input identity

| Item | Frozen input / configuration |
|---|---|
| Mission | GrandTour SPX-3 `2024-11-02-17-43-10`; native bag offset 32 s, sensor duration 134 s, playback 0.30× |
| Native source run | `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/classic_cbs_restoration_20260925/spx3_off_native/` (`complete.marker` present); Hesai+CPT7 GLIM and front stereo+CPT7 Kimera, same saved profiles as the matched ON control |
| Belief bag | `recordings/cbs.bag`, SHA-256 `b2fdf827f512f2b8a957b737a57fcde9091b2df71a21f603ad83745b61a12492`; topics `/kimera/cbs/odom_belief_in` (GLIM source `g`) and `/kimera/cbs/odom_belief_out` (Kimera source `k`) |
| Export policy | Existing `export_cbs_unique_relative_beliefs.py`: latest valid 0.20±0.06 s horizon belief per array, first exact-edge occurrence retained; **all** connected components retained. 1,330 GLIM and 663 Kimera unique edges; Kimera has three duplicate publications. No source-state trajectory is substituted for a belief. |
| Exported GLIM CSV | [glim_beliefs.csv](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/input/glim_beliefs.csv), SHA-256 `73daba74eb7728d7b7e05a1b38ef0c92c88680f7dd58b658b89927613e39fb47` |
| Exported Kimera CSV | [kimera_beliefs.csv](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/input/kimera_beliefs.csv), SHA-256 `bff07249be46a67a2d172189d6fa338f4e204b4172984a50b831f5d67558875` |
| IMU CSV | Existing SPX-3 raw CPT7 CSV `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv`, SHA-256 `c041516c61056977f5b249227e6ab981cd1603e8dcefd19b53523e7f7ff3a7d9`; its retained ON copy is byte-identical. This is the same mission/window and brackets all graph support times. |
| Replay executable | `/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay`, SHA-256 `b283fb0d9be9b471e85788a94acc9b308acc2ee53e51219846c3f1f7d80a174e`; existing image `haomingac/gnssfgo:latest`, local image ID `sha256:dff7473681dea4401bf3b6490faffbb52a3f73bbb57a79e090b5d126f411acb9`, with its installed GTSAM runtime |

## gnssFGO graph configuration

These are actual settings of the existing [GrandTourCsvReplay.cpp](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp), not generic gnssFGO defaults.

| Parameter | Value / semantics |
|---|---|
| Cases | `--mode glim` graph control; `--mode joint` GLIM+Kimera graph. The former loads but does not insert Kimera beliefs. |
| Availability | `--timeline record`: relative factors are inserted **only after** `record_timestamp_ns` and required support states exist. Physical endpoints remain the source `from/to_timestamp_ns`; no pretending the belief was known at its endpoint. |
| Solver | `--solver isam2`: full-history iSAM2, **no fixed lag**. QR factorization, relinearization threshold 0.01, skip 1, unused factor slots enabled. This run is not the alternative 3 s fixed-lag option. |
| Continuous-time model | gnssFGO `GPWNOJPrior` and `GPWNOJInterpolator` (**WNOJ**, not WNOA) with diagonal GP acceleration/jerk prior variance `10000` in all six coordinates. Support states `X,V,B,W` are placed on the ordered GLIM endpoint grid; non-grid Kimera endpoints use the existing single/double/same-interval GP-interpolated relative-pose factors. |
| IMU | Existing `CombinedImuFactor`/`PreintegratedCombinedMeasurements`. CPT7 `ALL_RIGHT`: sample stamped `t_k` owns `[t_(k-1),t_k]`, with exact partial overlaps. Gyro/accelerometer continuous densities `1.113913052107e-5` and `5.928679563445e-4`; gyro/accelerometer bias densities `0.000266` and `0.0043`. Integration covariance `1e-8 I`, bias integration `1e-5 I`, zero Coriolis. No values were tuned for this run. |
| Initialization | Identity pose, zero velocity; mean of 59 causally earlier IMU samples initializes gravity and biases. Initial pose sigma `[0.174533,0.174533,0.00174533,1e-5,1e-5,1e-5]`, velocity sigma `0.001`, bias sigma `[0.1,0.1,0.1,0.01,0.01,0.01]`, angular-rate sigma `0.1`. Actual estimated gravity and bias vectors are in each output `summary.json`. |
| Relative beliefs | Existing exact or interpolated Pose3-between factors use each exported native full 6×6 covariance as loaded. No manual covariance scaling, robust loss, gating, GNSS factors, or extra source-correlation model. The absence of a cross-estimator correlation model remains an approximation, not proof of independence. |

Executable command, with `MODE=glim` and then `MODE=joint` and the output directory changed accordingly, inside `haomingac/gnssfgo:latest` after sourcing `/opt/ros/humble/setup.bash`:

```bash
/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay \
  --imu /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv \
  --glim-beliefs /home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/input/glim_beliefs.csv \
  --kimera-beliefs /home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/input/kimera_beliefs.csv \
  --mode MODE --timeline record --solver isam2 \
  --out /home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/MODE
```

The first attempt used a generic Humble image plus an incompatible GTSAM library and exited 139 after opening zero-byte output files. It performed no graph computation and is **not** a result. Both completed cases below ran with the existing `haomingac/gnssfgo:latest` image and the same executable/input hashes; the failed empty files were overwritten by the successful control.

## Completed CBS-OFF controls

The unchanged `score_grandtour_prism.py` used the SPX-3 AP20 prism bag, CPT7→prism lever arm and SE(3) no-scale Kabsch alignment. The unchanged `score_grandtour_relative_pose.py` used the SPX-3 CPT7 IE-TC reference, a 1 s interval and 0.1 s sampling. Prism-matched GT coverage is 108.795 s of the 133.099 s graph trajectory. The RPE interval has 1,321 valid pairs in each graph case.

| Product | Factors accepted | Prism ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full CPT7 graph path (m) |
|---|---:|---:|---:|---:|---:|
| Native GLIM (context, **not** gnssFGO) | — | 0.007190 | — | — | — |
| Native Kimera (context, **not** gnssFGO) | — | 0.668695 | — | — | — |
| [gnssFGO GLIM-only](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/glim/summary.json) | 1,330 G, 0 K | 0.142027 | 0.012023 | 0.311471 | 84.243568 |
| [gnssFGO joint](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/joint/summary.json) | 1,330 G, 659 K | 0.136001 | 0.013237 | 0.294058 | 84.069022 |

The joint graph loaded 663 Kimera edges and skipped four outside the GLIM support grid. It used 1,332 support states and 5,328 graph values; wall times were 13.58 s (GLIM) and 117.74 s (joint). No source covariance was tuned after seeing these results. The graph GLIM-only trajectory is **not** native GLIM odometry; its ATE must not be labeled as native GLIM performance. The prism scorer's *prism-frame* full paths are 88.305 m and 88.105 m, respectively, different from the graph's CPT7-body path values above because of the lever arm.

Output trajectories and metrics are under [CBS-OFF graph artifacts](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/). Final `trajectory.tum` SHA-256: GLIM `cfbf6e330688494f0acf37ce810966d0a162c9015c61872c880e2da626a98233`; joint `1808fba804e5034a7af4e586692947e39daade58a4c9ee9f8245313026b37d59`.

## Fair future CBS-ON comparison contract

For the next authorized ON run, export beliefs from the **matched** `/runs/classic_cbs_restoration_20260925/spx3_classic_on_native/recordings/cbs.bag` with the same exporter, then use the exact same IMU CSV, graph executable/image, `record` timeline, full iSAM2/WNOJ configuration, no GNSS factor, and the same two evaluators/windows. Save OFF and ON native source metrics **separately** from gnssFGO GLIM-only/joint results. Do not change factors, covariance, or graph parameters to make the ON result look better. The current classic source run is operationally beneficial but not yet event-for-event certified against the archived September binary; see the restoration audit.
