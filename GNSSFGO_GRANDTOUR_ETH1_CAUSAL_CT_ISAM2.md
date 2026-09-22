# GrandTour ETH1 causal continuous-time iSAM2 pilot

Date: 2026-09-22

## Result in one paragraph

The existing 40 s GrandTour ALL_RIGHT prototype now supports real measurement
availability in a direct, full-history GTSAM `ISAM2` graph. A GLIM or Kimera
belief is added only at its `record_timestamp_ns`, but its factor continues to
connect the continuous-time states at the belief's physical from/to
timestamps. No GNSS factor is used. After all measurements have arrived, the
causal and noncausal physical-time solutions agree to sub-millimetre state
difference and effectively identical ATE/RPE. The online trajectory is also
retained separately from the post-run smoothed trajectory. The requested
pure-IMU numerical control is not stable: without any external pose belief the
current inertial/WNOJ graph runs away and becomes rank deficient. No artificial
prior was added to hide that result.

## Scope and immutable inputs

This work uses gnssFGO as the factor-graph framework. It does not use the CBS
EKF and it does not add GNSS factors.

| Input | Path | SHA-256 |
|---|---|---|
| CPT7 IMU | `/home/yeranis/cbs_imu_host_experiment_20260914/imu.csv` | `925d6cd8c10000434a9f92dd0ba4565798a5a2ef51cf249c400147a32a11a708` |
| ALL_RIGHT GLIM beliefs | `/home/yeranis/cbs_cpt7_common_convention_40s_pilot_20260921/all_right/glim_beliefs.csv` | `59ee8ead374f8aee87b464e4834f83ca0c1590ec3c175667752c97d46b104316` |
| ALL_RIGHT Kimera beliefs | `/home/yeranis/cbs_cpt7_common_convention_40s_pilot_20260921/all_right/kimera_beliefs.csv` | `e831dd552ca784c90b90351a716a2b948768637d13fdd061f5fa64d02b551122` |

The 391-state support grid and the ALL_RIGHT IMU convention are unchanged.
Every GLIM belief is exact on that grid. Kimera endpoints retain the native
continuous-time WNOJ interpolation already validated in the physical-time
prototype.

## Causal event semantics

The runner remains
`online_fgo/src/offline_process/GrandTourCsvReplay.cpp`. Its new switches are:

```text
--mode imu|glim|kimera|joint
--timeline physical|record
--solver fixed_lag|isam2
```

`--timeline record --solver isam2` has the following semantics:

1. IMU support states are created in increasing physical time.
2. Relative beliefs are ordered by `record_timestamp_ns`, then source and
   source ordinal for deterministic ties.
3. A belief is not added before its record timestamp.
4. Once it arrives, the new factor connects the already-retained historical
   state keys at its physical endpoints. Interpolated Kimera endpoints connect
   the adjacent historical WNOJ support states.
5. Direct `gtsam::ISAM2` retains the full 40 s graph; there is no marginalizing
   fixed-lag boundary in this pilot.

The factor is not retimestamped to its arrival and no measurement value or
covariance is changed. The graph therefore performs historical correction
through ordinary iSAM2 re-elimination, not through an EKF rewind/replay.

Full iSAM2 uses QR factorization. This is a numerical factorization choice and
does not change the factor-graph objective. The original fixed-lag mode and
defaults remain available and unchanged.

## Measurement availability

| Source | Accepted | Minimum delay | Median | Mean | p95 | Maximum |
|---|---:|---:|---:|---:|---:|---:|
| GLIM | 389 | 100.577 ms | 112.914 ms | 126.081 ms | 156.166 ms | 199.503 ms |
| Kimera | 180 | 24.715 ms | 31.776 ms | 31.677 ms | 35.216 ms | 40.273 ms |

Delay is `record_timestamp_ns - to_timestamp_ns`. Two of the 182 loaded Kimera
beliefs remain outside the GLIM support range, exactly as in the preceding
prototype. No usable belief was lost because of delayed arrival.

## Causal versus physical-time equivalence

The `physical` control inserts a factor as soon as its right support is
available. The `record` run waits for the real record timestamp. Both retain
the full graph, and the table below compares their final estimates after every
accepted factor has been incorporated.

| Mode | max position | max velocity | max attitude | max accel-bias | max gyro-bias |
|---|---:|---:|---:|---:|---:|
| GLIM | 0.598 mm | 0.0936 mm/s | 0.00709 deg | 0.001202 m/s^2 | 0.000304 rad/s |
| Kimera | 0.287 mm | 0.0416 mm/s | 0.00536 deg | 0.000785 m/s^2 | 0.0000322 rad/s |
| Joint | 0.313 mm | 0.1204 mm/s | 0.00285 deg | 0.000104 m/s^2 | 0.000000800 rad/s |

This is numerical agreement, not byte identity. Two small effects are retained
and explicitly recorded instead of being hidden:

- iSAM2 linearization/re-elimination order differs when factors arrive later;
- the current WNOJ support acceleration constants are formed from the estimate
  available when a support state is created. Consequently physical and causal
  schedules produce slightly different constants (maximum component change
  0.0552 in the joint run).

The resulting differences are far below the measurement accuracy in this
pilot. A future strict graph-identity test would have to freeze the support
acceleration construction independently of measurement arrival. That was not
changed here.

## Accuracy results

The AP20 prism provides 14.100 s of matched position coverage in this window.
ATE uses the existing SE(3) Kabsch/no-scale prism evaluator. The 1 s RPE uses
the existing CPT7 IE-TC 6-DoF reference over the complete 39.000 s estimate.

### Final full-graph estimate after all arrivals

| Timeline | Case | ATE RMSE | t-RPE RMSE | r-RPE RMSE | full path |
|---|---|---:|---:|---:|---:|
| Physical | IMU only | **failed** | **failed** | **failed** | **failed** |
| Physical | IMU + GLIM | 0.055894 m | 0.031212 m | 0.521767 deg | 13.968566 m |
| Physical | IMU + Kimera | 0.011499 m | 0.009529 m | 0.304884 deg | 13.607565 m |
| Physical | IMU + GLIM + Kimera | 0.041114 m | 0.021355 m | 0.508215 deg | 13.805758 m |
| Causal record-time | IMU only | **failed** | **failed** | **failed** | **failed** |
| Causal record-time | IMU + GLIM | 0.055896 m | 0.031210 m | 0.521789 deg | 13.968763 m |
| Causal record-time | IMU + Kimera | 0.011503 m | 0.009527 m | 0.304882 deg | 13.607518 m |
| Causal record-time | IMU + GLIM + Kimera | 0.041108 m | 0.021350 m | 0.508217 deg | 13.805791 m |

The final physical and causal accuracy is effectively the same. This verifies
the primary same-information check for the retained full-history graph.

### Estimate available online at each support time

These rows are the forward estimates that existed before future delayed
beliefs arrived. They must not be called smoothed trajectories.

| Causal case | ATE RMSE | t-RPE RMSE | r-RPE RMSE | full path |
|---|---:|---:|---:|---:|
| IMU + GLIM | 0.062849 m | 0.050367 m | 1.856717 deg | 14.393812 m |
| IMU + Kimera | 0.021358 m | 0.019732 m | 0.314936 deg | 13.792378 m |
| IMU + GLIM + Kimera | 0.046813 m | 0.036697 m | 1.213297 deg | 13.942726 m |

The gap between online and final rows is expected: full iSAM2 later updates
historical states when delayed factors arrive.

## IMU-only control

The IMU-only graph is not a valid 40 s navigation solution with the current
initialization and no absolute/relative pose information. Direct full iSAM2
became indeterminate near `x175`; immediately beforehand position, velocity
and biases had already grown to nonphysical magnitudes. The existing 3 s
fixed-lag backend also became indeterminate near `x145`. QR delayed but did not
remove the failure. This is a useful negative numerical control and not a
reason to add an unrequested stabilizing prior.

## Is full continuous-time iSAM2 strictly required?

For this 40 s window, no: all accepted delays are below 0.2 s, so a correctly
implemented fixed-lag smoother with a lag safely longer than the delay could
retain every referenced support state. Continuous-time interpolation is still
required for native Kimera endpoints that fall between GLIM support states.

Full iSAM2 was used here because it is the simplest unambiguous validation of
the supervisor's requested gnssFGO + continuous-iSAM2 scenario: no historical
state is marginalized before a delayed factor arrives. For longer sequences,
a bounded fixed-lag version is the practical next engineering step after the
full-history reference is trusted.

## Outputs and reproducibility

Artifacts are retained under:

```text
/home/yeranis/results/gnssfgo_grandtour_eth1_ct_causal_20260922/
```

Each successful iSAM2 directory contains:

- `online_trajectory.tum` / `online_states.csv`: causal forward estimates;
- `trajectory.tum` / `states.csv`: final post-arrival full-graph estimates;
- `measurement_events.csv`: physical and arrival timestamps;
- `support_accelerations.csv`: the WNOJ constants used to build the graph;
- `belief_residuals.csv`, ATE/RPE JSON, and evaluator CSVs.

Two independent causal executions produced byte-identical trajectory, state,
event, acceleration and residual files in all three observable cases. Final
trajectory hashes are:

| Case | SHA-256 |
|---|---|
| GLIM | `9d754f75f1ac168e8e2fd6afec67d8989a1d950eb798d3ffa1f7477b6acb2750` |
| Kimera | `84125ebb28eae7687879c76f9152f990fc3cbf7a377f1a9ee6cf297f95bbb32b` |
| Joint | `86d3470bf66de818c90186f65136225a8e3ae7d91a8c85411afae6b7feaf3482` |

Backward compatibility was checked before the new experiments: the default
fixed-lag physical-time GLIM trajectory, states and residual CSV are
byte-identical to the committed prototype (`097c9e...f928d`,
`4eab25...a99692`, `874910...ec17f`).

Validation:

- complete Release package build: pass;
- focused factor and ALL_RIGHT tests: 10/10 pass;
- modified source focused cpplint/uncrustify: pass;
- package-wide CTest: 5/10 pass; the five failures are pre-existing repository
  lint debt (copyright/cpplint/flake8/lint-cmake/uncrustify across legacy and
  generated files), not runtime or scientific-test failures;
- deterministic causal replay: pass for GLIM, Kimera and joint.

## Reproduction command pattern

```bash
grandtour_ct_replay \
  --imu imu.csv \
  --glim-beliefs glim_beliefs.csv \
  --kimera-beliefs kimera_beliefs.csv \
  --mode joint \
  --timeline record \
  --solver isam2 \
  --out OUTPUT_DIRECTORY
```

## Conclusion

The causal continuous-time factor insertion is working as intended on the
40 s ETH1 pilot. Delayed GLIM/Kimera factors update their historical physical
endpoints through full iSAM2, and the final same-information result agrees
with physical-time insertion at sub-millimetre scale. This establishes the
causal graph mechanism without GNSS. It does not yet establish bounded-memory
operation or long-sequence performance.
