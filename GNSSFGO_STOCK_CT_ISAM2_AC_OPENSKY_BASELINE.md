# gnssFGO stock CT fixed-lag baseline on AC_Opensky

Date: 2026-09-21

## Executive result

The unchanged Aachen loosely coupled configuration runs successfully from beginning to end on the official 210 s `AC_Opensky` ROS 2 bag. It produces 2,099 optimized states spanning 209.7998 s and completes 2,098 incremental fixed-lag optimization cycles.

The selected continuous-time model is **WNOJ**, not WNOA. The solver is the repository's iSAM2-backed `IncrementalFixedLagSmoother` with a 3 s lag. The graph fuses the repository's reference-PVA loose-coupling integrator, raw IMU propagation/preintegration, and the stock LIO-SAM relative-pose path.

Two complete runs were made without changing estimator source or configuration:

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| Optimized states | 2,099 | 2,099 |
| Optimization cycles | 2,098 | 2,098 |
| Optimized-state span | 209.7998 s | 209.7998 s |
| 2D position RMSE | 0.052341 m | 0.057447 m |
| 3D position RMSE | 0.057931 m | 0.062748 m |
| 3D position median | 0.034073 m | 0.036711 m |
| 3D position p95 | 0.119571 m | 0.125628 m |
| 3D position maximum | 0.192670 m | 0.274169 m |
| 3D velocity RMSE | 0.265404 m/s | 0.266994 m/s |
| Reconstructed ECEF path length | 1,930.125 m | 1,930.120 m |
| Mean optimization time | 3.772 ms | 3.765 ms |
| p95 optimization time | 5.011 ms | 5.040 ms |
| Maximum optimization time | 6.597 ms | 7.880 ms |

This is a working stock-configuration baseline for subsequent engineering. It is not byte deterministic because LIO processing and graph construction run concurrently. The two runs nevertheless have identical optimized timestamps; their median position separation is 5.6 mm and p95 separation is 5.1 cm.

## Scope

This run did not:

- connect the new same-support relative factor to a production integrator;
- add a GLIM, Kimera, CBS, GrandTour, or EKF measurement;
- alter WNOJ/WNOA parameters, factor noise, robust losses, or fixed-lag settings;
- change any estimator source or configuration.

The previously added `GPInterpolatedSameIntervalPose3BetweenFactor` remains an isolated header/test primitive and is not instantiated by the online estimator.

## Provenance

### Source

- Repository: `/home/yeranis/repos/gnssFGO`
- Branch: `ct-isam2-relative-factor`
- Replay HEAD: `45da07a639a319ba2b5a1d4d9a4be8a1e4bbc5f5`
- Official upstream tip merged into the branch: `16030fb6a24263a4b927d2ec5d1da266340ddb9b`
- Docker image: `haomingac/gnssfgo:latest`
- Image ID/digest: `sha256:dff7473681dea4401bf3b6490faffbb52a3f73bbb57a79e090b5d126f411acb9`

The working branch is not a pristine checkout of `upstream/ros2`: it retains the user's earlier fork commits, including prior GP-interpolator/LIO fixes, and adds the isolated same-support factor test. This report therefore means **stock Aachen configuration on the exact merged working branch above**, not an upstream-only benchmark. No working-branch change was made for this replay.

### Dataset

- Source: official gnssFGO Sciebo share linked by the repository
- Sequence: `ros2_db3/AC_Opensky`
- Local path: `/home/yeranis/datasets/gnssFGO/AC_Opensky`
- Bag duration: approximately 210 s
- Bag database SHA-256: `6b4d4ffb74e95f4c9c94caf47cc82719ecc50b2d32f9e77f4952fbf50cb417c7`
- Metadata SHA-256: `100af6bce9ff464a1ebb21fb28a8cb80081405cf87213254ca4ead93f6054ce8`

`AC_Opensky` begins at the same approximate point as applying the stock Aachen launch's 60 s start offset to the full AC bag. It includes IMU, VLP-16 point clouds, NovAtel/Ublox data, and the reference PVA required by the stock configuration.

## Exact estimator configuration

The following repository files were used unchanged:

| File | SHA-256 |
|---|---|
| `online_fgo/config/aachen_lc/common.yaml` | `a1122f2a5eadffb55c77a64bbd7330d6bf9f7b3a655380a9f21bf20acf035135` |
| `online_fgo/config/aachen_lc/integrator.yaml` | `534d13d1053bccf7c6849a624d4d40a3b93d492c1f0d967df1e52adb839451ce` |
| `online_fgo/config/aachen_lc/optimizer.yaml` | `218128e4f5643ab5c93f5838416049634ec2cf026aed73f05576a168111b23d7` |
| `online_fgo/config/aachen_lc/sensor_parameters.yaml` | `64eec9e56ff7c89d376abf794d8689e10d965ccab162accf3f5feadda116fbd4` |
| `LIOSAM/config/params.yaml` | `523866118704148d9e1c110b5d7bb6dddb31253f01b87935b2637622b7d03b19` |

Relevant active settings:

```yaml
use_sim_time: true
optFrequency: 10
stateFrequency: 10
addIMUFactor: true
addGPPriorFactor: true
addGPInterpolatedFactor: true
gpType: WNOJ
Integrators: [IRTPVALCIntegrator, LIOIntegrator]
smootherType: IncrementalFixedLag
smootherLag: 3.0
QcGPWNOJInterpolatorFull: [10000, 10000, 10000, 10000, 10000, 10000]
QcGPWNOJMotionPriorFull: [10000, 10000, 10000, 10000, 10000, 10000]
```

The fixed-lag implementation finds keys older than `current_timestamp - smootherLag`, orders them for elimination, calls iSAM2 `marginalizeLeaves`, and erases their timestamp entries. Thus the successful 209.8 s run does not retain the full trajectory in the active optimizer.

## Execution

The GUI-dependent `aachen_lc_all.launch.py` starts its children through `xterm` and also launches Mapviz/RViz. In the headless container, the same non-visual components were started explicitly:

```bash
ros2 launch irt_gnss_preprocessing gnss_preprocessor.launch.py
ros2 launch online_fgo aachen_lc.launch.py
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
ros2 run lio_sam lio_sam_imageProjection --ros-args \
  --params-file /workspace/fork/install/lio_sam/share/lio_sam/config/params.yaml
ros2 run lio_sam lio_sam_featureExtraction --ros-args \
  --params-file /workspace/fork/install/lio_sam/share/lio_sam/config/params.yaml
ros2 bag play /data/AC_Opensky --clock
```

The orchestration script is retained outside the repository at:

`/home/yeranis/results/gnssfgo_stock_ct_isam2_ac_opensky_20260921/run_stock_ac_opensky.sh`

Recorded output topics were:

```text
/deutschland/stateOptimized
/deutschland/statePredicted
/deutschland/GNSSFGO/error
/deutschland/GNSSFGO/elapsedTime
/deutschland/user_estimation
/lio_sam/feature/cloud_info
```

## Completeness and LIO activity

Both runs cover the full sequence and contain the same 2,099 optimized timestamps.

| Diagnostic | Run 1 | Run 2 |
|---|---:|---:|
| `/stateOptimized` | 2,099 | 2,099 |
| `/statePredicted` | 20,945 | 20,945 |
| `/GNSSFGO/error` | 2,098 | 2,098 |
| `/GNSSFGO/elapsedTime` | 2,098 | 2,098 |
| LIO feature-cloud messages | 1,229 | 1,228 |
| LIO ordinary-between integration cycles | 225 | 218 |
| LIO double-between integration cycles | 355 | 376 |
| Successful scan-to-map convergence messages | 615 | 634 |

The LIO path is therefore active. The common `LIOSAM: no odom available` log line occurs on graph cycles where a fresh odometry result is not ready; it does not mean LIO was absent from the complete run.

## Position and runtime statistics

All 2,098 stock error messages in both runs report reference mode `RTKFIX`.

| Statistic | Run 1 3D position | Run 2 3D position |
|---|---:|---:|
| Mean | 0.045653 m | 0.048919 m |
| RMSE | 0.057931 m | 0.062748 m |
| Median | 0.034073 m | 0.036711 m |
| p95 | 0.119571 m | 0.125628 m |
| Maximum | 0.192670 m | 0.274169 m |

The `Error2GT` publisher compares the optimized antenna pose against the interpolated reference PVA. That same reference-PVA stream is also used by `IRTPVALCIntegrator` as a graph measurement. These centimeter-level numbers are therefore stock **reference-PVA consistency errors**, not accuracy against an independent survey/mocap ground truth. They are appropriate for reproducing the repository scenario but must not be presented as independent-GT accuracy.

The factor graph builds 3 factors on the median cycle, 4 at p95, and at most 7 in these runs. Mean graph construction time is about 0.213 ms; mean optimization time is about 3.77 ms, comfortably inside the 100 ms state period on this host.

## Stock yaw diagnostic caveat

The stock `Error2GT.yaw_error` output is not credible on this slice:

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| Yaw RMSE | 100.294 deg | 100.349 deg |
| Yaw median | 86.269 deg | 86.186 deg |
| Yaw maximum | 179.730 deg | 179.702 deg |

No attitude mathematics or evaluator was changed to address this. The position/runtime baseline is valid as recorded; the stock yaw field requires a separate frame/convention audit before it can be used as an accuracy claim.

## Repeatability

The ROS graph is multi-threaded and LiDAR odometry availability varies slightly with scheduling. Comparing the two optimized state streams at all 2,099 identical physical timestamps gives:

| Run-1 versus run-2 difference | Median | RMS | p95 | Maximum |
|---|---:|---:|---:|---:|
| Position | 0.00558 m | 0.02291 m | 0.05099 m | 0.18850 m |
| Velocity | 0.00615 m/s | 0.02837 m/s | 0.06735 m/s | 0.18054 m/s |
| Orientation | 0.1194 deg | 0.2739 deg | 0.5704 deg | 1.3991 deg |

Consequently, a future custom-factor result should not be judged from one online run when the change is smaller than this scheduling envelope. The cleanest later comparison is either repeated runs with distributions or a deterministic saved LIO-relative-measurement stream.

## Plot

The retained comparison plot is:

`/home/yeranis/results/gnssfgo_stock_ct_isam2_ac_opensky_20260921/position_error_repeat.png`

It shows that the two runs follow the same error structure, with localized scheduling-dependent differences.

## Validation

- Clean Release rebuild of `online_fgo`: **PASS**.
- Focused `test_ct_relative_factor`: **PASS** (1 CTest target, 3/3 GoogleTests).
- Full 210 s replay, run 1: **PASS**, process exit 0.
- Full 210 s replay, run 2: **PASS**, process exit 0.
- `git diff --check`: **PASS**.
- Repository source was clean before this report was added.

The aggregate package test command is not a clean upstream baseline. It still reports the repository's pre-existing copyright, cpplint, and uncrustify failures across legacy/generated/third-party files. The focused executable tests pass; no attempt was made to rewrite unrelated files.

## Retained artifacts

Root:

`/home/yeranis/results/gnssfgo_stock_ct_isam2_ac_opensky_20260921`

Important files:

- `full_run_1/output/output_0.db3` — SHA-256 `61b364a5ea80350ce7b9b4d40a72d16f9f68ae46c28c7f4beafb75c9106b4a71`
- `full_run_2/output/output_0.db3` — SHA-256 `b6151b5ddf5f341c132e0991e43de166fadc671d8f6ba0b6698d79f2d5290c8b`
- `full_run_1/analysis/summary.json`
- `full_run_2/analysis/summary.json`
- `repeatability.json`
- `position_error_repeat.png`
- per-run optimized-state, error, and elapsed-time CSV files
- complete node logs and bag metadata

## Decision and next bounded step

The maintained gnssFGO environment, CT WNOJ model, LIO relative-pose path, and incremental fixed-lag iSAM2 solver are operational on an official retained sequence. This is now a usable baseline.

Before connecting a new relative-belief source, the next bounded engineering step should be to add and test deterministic runtime dispatch for the four endpoint cases already identified: synchronized, single-interpolated, distinct-support double-interpolated, and same-support double-interpolated. The stock baseline and its scheduling envelope should remain frozen while that dispatch is tested.
