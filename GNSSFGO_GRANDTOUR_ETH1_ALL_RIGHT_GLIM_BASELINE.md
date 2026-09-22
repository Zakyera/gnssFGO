# gnssFGO continuous-time GrandTour ETH1 GLIM baseline

Date: 2026-09-22

## Result

The first 40 seconds of the GrandTour ETH1 mission now run deterministically
through gnssFGO's iSAM2-backed `IncrementalFixedLagSmoother`. The graph uses
the stock WNOJ motion prior, a 3 s lag, CPT7 Combined-IMU factors with the
validated ALL_RIGHT sample convention, and the 389 native full-covariance GLIM
relative beliefs at their physical endpoint times.

| Quantity | Result |
|---|---:|
| Support states | 391 |
| GLIM beliefs accepted / loaded | 389 / 389 |
| Physical duration | 38.999880 s |
| Active smoother values, maximum | 124 |
| Pose path length | 14.320746 m |
| Prism-lever-arm path length | 14.294099 m |
| AP20 prism ATE RMSE | 0.070180 m |
| 1 s translational RPE RMSE | 0.047558 m |
| 1 s rotational RPE RMSE | 1.815599 deg |
| Graph runtime, run 1 | 0.613 s |
| Graph runtime, validation rerun | 0.761 s |

The prism is only available for one 14.100256 s segment in this 40 s window.
The AP20 result therefore uses 142 matched poses and must not be described as
full-window position ground truth. The 1 s RPE uses the retained CPT7 IE-TC
6-DoF reference across the complete 40 s window.

## Dataset and immutable inputs

- Mission: GrandTour `2024-10-01-11-29-55` (ETH1).
- IMU CSV: `/home/yeranis/cbs_imu_host_experiment_20260914/imu.csv`
  (`925d6cd8c10000434a9f92dd0ba4565798a5a2ef51cf249c400147a32a11a708`).
- GLIM beliefs:
  `/home/yeranis/cbs_cpt7_common_convention_40s_pilot_20260921/all_right/glim_beliefs.csv`
  (`59ee8ead374f8aee87b464e4834f83ca0c1590ec3c175667752c97d46b104316`).
- AP20 prism bag:
  `/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_ap20_prism_position.bag`
  (`837d2097f365b683159f4de95b51dff5dd70adc0137d423ccef25d43ceaa44c2`).
- CPT7 IE-TC bag:
  `/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag`
  (`28815a4a300624766e3ffe0e1cc0f150ac60e09e395381ca25c3e6d7e04e18e3`).

The source beliefs remain in right-local Pose3 tangent order `[phi,rho]`. The
full dense 6x6 covariance is passed unchanged to `BetweenPose3Factor`.

## Graph and IMU semantics

The executable is
`online_fgo/src/offline_process/GrandTourCsvReplay.cpp` and is built as
`grandtour_ct_replay`.

Each unique GLIM endpoint creates one support state containing Pose3, velocity,
IMU bias, and body angular rate. Consecutive supports receive:

1. one `CombinedImuFactor`;
2. one stock gnssFGO `GPWNOJPrior`;
3. any GLIM relative factor ending at the new support.

The smoother is `IncrementalFixedLagSmoother` with a 3 s lag. Its maximum 124
active values correspond to approximately 31 four-variable support states, so
the graph does not retain the full trajectory in the active optimization.

ALL_RIGHT means that the CPT7 sample stamped `t_k` is integrated over
`[t_(k-1),t_k]`. The first and last sample intervals are clipped to their exact
fractional overlap with each support interval. The pure overlap implementation
is in `online_fgo/include/offline_process/GrandTourImuSupport.h`.

The initialization uses the 68 causally preceding IMU samples and reproduces
the validated ETH1 values:

```text
gravity = [0.1085926571, 0.1475208421, 9.8082896183]
b_a     = [-0.0002687238, -0.0003650556, -0.0242716306]
b_g     = [0.0000139402, -0.0001024612, -0.0000837158]
```

The continuous noise densities match the validated GrandTour 15D host. The
stock gnssFGO integration covariance, bias-integration covariance, WNOJ
`Qc=[10000,...,10000]`, and 3 s lag were retained; none was tuned to ETH1.

## Important scope boundary

This is an offline physical-time graph baseline. The CSV `record_timestamp_ns`
is validated and retained, but this executable currently inserts a GLIM factor
when its physical endpoint support is processed. It does not yet simulate
delayed online availability or historical replay. This is intentional for the
first gnssFGO adapter check and must not be confused with the earlier causal
EKF replay.

The graph does not yet consume Kimera. Native Kimera endpoints do not coincide
with the GLIM support grid; 180 of the 182 retained Kimera beliefs have both
endpoints in distinct support intervals, while the first and last lack full
bracketing coverage. The next implementation step is therefore the tested
double-interpolated relative factor, not timestamp rounding or the earlier EKF
temporal-alignment approximation.

## Accuracy diagnostics

The AP20 prism evaluation uses the existing CPT7-to-prism lever arm and SE(3)
Kabsch alignment without scale:

| AP20-overlap statistic | Value |
|---|---:|
| ATE mean | 0.057803 m |
| ATE median | 0.051048 m |
| ATE RMSE | 0.070180 m |
| ATE maximum | 0.186846 m |
| Matched estimate / reference path | 4.104411 / 3.617858 m |

The posterior GLIM factor residuals are small:

| Residual | Mean | Median | p95 | Maximum |
|---|---:|---:|---:|---:|
| Translation | 0.006333 m | 0.004841 m | 0.016474 m | 0.029662 m |
| Rotation | 0.000234 rad | 0.000178 rad | 0.000596 rad | 0.001109 rad |

Small posterior factor residuals establish that the factors are being consumed;
they are not an independent accuracy metric.

## Determinism and validation

Three runs after the final overlap-helper refactor produced byte-identical
scientific outputs:

| Artifact | SHA-256 |
|---|---|
| `trajectory.tum` | `097c9e1c5a8189c8c22ad8f64340c26cdce5b0f0f9289f1d97b2e58b9bcf928d` |
| `states.csv` | `4eab2528d76741c3548de770ac7b80137371f3cc72a2c817f99bd51d37a99692` |
| `belief_residuals.csv` | `bca20d66773526c76211391d98a3a0209de617847ad0efe3fb2caaafe1fdd493` |

- Clean Release build: pass.
- Existing CT relative-factor tests: 3/3 pass.
- New ALL_RIGHT exact/full, fractional-boundary, split-conservation, and
  invalid-coverage tests: 4/4 pass.
- 389/389 relative beliefs accepted.
- Dense source covariance symmetry/positive-definiteness checked at load.
- `git diff --check`: pass.

Retained artifacts are in
`/home/yeranis/results/gnssfgo_grandtour_eth1_40s_20260922`.

## Exact replay command

Inside the maintained gnssFGO ROS 2 container/workspace:

```bash
grandtour_ct_replay \
  --imu /input/imu/imu.csv \
  --beliefs /input/right/glim_beliefs.csv \
  --out /results/run_1
```

This establishes the deterministic GrandTour/gnssFGO adapter baseline needed
before adding asynchronous Kimera relative factors.
