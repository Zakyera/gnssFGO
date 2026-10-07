# Source driven WNOJ receiver implementation and synthetic validation

The opt-in `source_wnoj` receiver is implemented and passes the bounded synthetic validation described below. It uses GLIM/Kimera relative beliefs as motion measurements and gnssFGO's existing full WNOJ model as a motion prior. Acceleration is an estimated graph variable, not an IMU input.

No GLIM, Kimera, EKF, GrandTour replay, CBS-OFF/ON comparison, or real-data fusion experiment was run for this task. The production receiver binary and default configuration were not replaced. This is an implementation milestone, not evidence that CBS-ON accuracy improves.

## Receiver model and reuse

The existing executable gains an explicit selector:

```text
--receiver-model imu_wnoj       existing default
--receiver-model source_wnoj    opt-in experimental receiver
```

The new mode requires the existing `sources`, `record`, `isam2`, `uniform_imu` route. It rejects legacy motion ablations, source uncertainty additions, covariance block changes and Kimera row ablations. No new source transport or optimizer/replay framework was introduced.

| Component | Existing default | New opt-in mode |
|---|---|---|
| Support variables | X, V, W, B | X, V, W, A |
| IMU factor | Combined IMU | Absent |
| WNOJ factor | Existing GPWNOJPrior with fixed IMU-derived accelerations | Existing GPWNOJPriorFull with estimated A at both endpoints |
| Relative interpolation | Existing GPWNOJInterpolator | Existing GPWNOJInterpolatorFull |
| Bias and gravity initialization | Existing raw-IMU initialization | Absent |
| Initial guesses | Preintegrated raw IMU | Zero-process-noise full-WNOJ transition from the latest optimized state |
| Source revisions | Stable-ID factor replacement | Same ledger and replacement mechanism |

`X` is Pose3, `V` is world linear velocity, `W` is body angular velocity, and `A` is the existing full-WNOJ six-dimensional tangent acceleration, ordered angular then translational. In particular, the translational part is not raw specific force or automatically world Cartesian acceleration. Its meaning is the one used by the existing prior/interpolator and their body-twist conversion.

The objective is the sum of source relative-factor errors, full-WNOJ errors and initial priors. With N supports and M distinct source beliefs, the graph contains N−1 WNOJ factors, M relative factors and four initial priors. Revisions replace an existing relative factor rather than adding a concurrent copy.

The new code explicitly audits the final actual factor types, factor count, A keys and absence of B keys. In the two-source synthetic example: 25 supports, 24 WNOJ factors, 20 source factors, four priors, zero IMU factors and zero bias variables.

## Timing and initialization

The unchanged causal clock still consumes IMU physical/availability timestamps. Defaults remain 100 ms supports, anchored at the 59th timestamp. This preserves the receiver grid and arrival chronology for a later matched experiment. It does **not** average those 59 sensor values, integrate acceleration/gyro, infer gravity, initialize bias, or constrain angular velocity from the IMU in the new mode.

The transport continues carrying and validating finite IMU payloads, but only their timestamps enter the new receiver's computation. Timestamp changes can affect the clock; changing valid acceleration/gyro values cannot affect the graph. The synthetic integration tests verify this including all startup samples.

Relative factors are inserted only after both their actual arrival and required support availability. Endpoints remain at the source's physical timestamps. Delayed revisions, pending right support, source/session identities and full-history iSAM2 behavior are unchanged.

Startup means are identity pose and zero V/W/A. New-mode prior standard deviations were chosen explicitly before any real-data scoring:

| Prior | Standard deviation | Meaning |
|---|---|---|
| X0 | 1e−5 per rotational/translational coordinate | Fix the arbitrary global SE(3) gauge; not a measured gravity direction |
| V0 | 10 m/s per coordinate | Broad finite initial velocity prior |
| W0 | 3 rad/s per coordinate | Broad finite initial angular-rate prior |
| A0 | 10 rad/s² angular, 10 m/s² translational | Broad finite tangent-acceleration prior |

These are engineering assumptions, not calibrated sensor uncertainties or sequence-tuned values. They regularize startup and can matter before enough source motion arrives. No source belief is reused to create an additional tight prior. The old mode retains its original priors and operation path.

Qc stays at `diag(10000,10000,10000,10000,10000,10000)`. The existing 18D WNOJ covariance is used unchanged; its pose block is `Qc * dt^5 / 20`. No Qc sweep, source covariance scaling or GT-based selection took place. WNOJ permits acceleration to change under the jerk prior; it does not force a constant acceleration throughout the trajectory.

## Relative factor implementation

[SourceWnojModel.h](/home/yeranis/repos/gnssFGO/online_fgo/include/offline_process/SourceWnojModel.h) supplies the small receiver-specific adapter around the existing full-WNOJ classes.

The relative residual is unchanged:

```text
r = Local(Z, T(t_i)^-1 T(t_j))
```

Source measurement noise remains its full dense 6×6 Gaussian covariance, with scale 1. No gating, robust kernel, covariance inflation, health weighting or CI is added. The existing GP conditional-mean interpolation convention is retained; no additional interpolation covariance is inserted into the source R matrix in this task.

An exact endpoint reads X. An interpolated endpoint reads X/V/W/A at its two supports. The new factor takes the union of endpoint keys and **sums** Jacobian contributions for shared keys. Tested key counts are 2 for exact/exact; 8 for exact/shared interpolation or two endpoints in one interval; 12 for adjacent interpolated intervals; and 16 for disjoint interpolated intervals. This avoids duplicating optimizer variables when intervals overlap.

Prediction is only an initial guess. It is derived by setting the existing full-WNOJ residual to zero, including its SE(3) right-Jacobian and tangent-acceleration conversion. The synthetic test verifies the resulting prior residual is below 1e−12. Optimization subsequently estimates X/V/W/A; it does not hold those predictions fixed.

## Derivative finding

The stock `GPWNOJPriorFull` analytic derivative branch differed from independent finite differences by a maximum absolute element of **0.452831** at the tested nontrivial state. This is a concrete implementation warning for that branch, not a claim about all stock configurations or the old receiver.

The new mode selects the library's already-existing `useAutoDiff=true` branch. Despite that name, it uses numerical derivatives. No library mathematics or derivative implementation was edited.

Measured differences against independent finite differences using a different perturbation size:

| Used path | Maximum absolute Jacobian difference |
|---|---:|
| Full WNOJ prior | 1.9186e−10 |
| Full WNOJ pose interpolator | 4.55188e−10 |
| New relative factor across all tested endpoint patterns | Below 2e−6 test tolerance |

This is a correctness-first implementation. Numerical derivatives may affect throughput; there is no real-time capacity claim yet.

## Synthetic validation results

Artifacts are in [gnssfgo_source_wnoj_20261005](/home/yeranis/results/gnssfgo_source_wnoj_20261005).

The optimized isolated build uses C++20, `-O3 -DNDEBUG`, the retained local ROS Humble image and copied existing GTSAM/gnssFGO shared libraries. No downloads or package installations were needed.

- 21 focused C++ tests passed: 3 new full-WNOJ tests, 6 existing relative-factor tests, 4 IMU-support tests, 4 causal-clock tests and 4 source-model-covariance tests.
- 5 transport protocol tests passed, including opt-in model validation and arbitrary source combinations.
- Dense covariance preservation, residual convention, exact/partial endpoints, shared-key Jacobian accumulation, factor copying and acceleration-variable sensitivity passed.
- Synthetic stationary, constant-velocity and constant-acceleration optimization passed. Their full joint covariance matrices were finite and positive definite; repeated optimization gave exactly equal joint covariance matrices.
- A 3 s synthetic input stream, with 2.4 s of initialized support history, tested two source IDs, delayed arrivals, a later revision and a final source gap. Maximum x-position error against its analytic synthetic trajectory was 0, 2.3730e−5 m and 4.9248e−5 m respectively. These are synthetic correctness results, not GrandTour ATE.
- Repeated journals produced byte-identical final and as-played trajectories/states, source covariance outputs, residuals, support clock and model diagnostics.
- Mutating **all** finite accel/gyro values, including startup values, left those same outputs byte-identical in the new mode.
- The existing asynchronous test exercised seven arbitrary source IDs, nine active belief IDs, replacement, duplicate/stale handling, session restart, five endpoint patterns, a source dropout and future-suffix causality. It passed in both modes.
- Incremental pipe processing and CSV processing of the same monotonic-IMU event journal gave byte-identical trajectory/state/clock outputs in both modes.
- Invalid covariance, contradictory revisions, changed endpoints, pre-arrival measurements, reversed arrival clocks, premature disconnect and unsupported end-of-stream endpoints were handled by the existing rejection/accounting checks. These are not a proof of rollback after an arbitrary internal iSAM2 exception; no such general guarantee was added.
- The preserved production binary, the newly built default mode and explicit `imu_wnoj` mode produced byte-identical numerical output files on all three synthetic motion cases. Wall-clock timing fields are not compared.

Evidence: [unit log](/home/yeranis/results/gnssfgo_source_wnoj_20261005/logs/unit_tests.log), [existing regression commands](/home/yeranis/results/gnssfgo_source_wnoj_20261005/logs/commands_regression.json), [receiver validation](/home/yeranis/results/gnssfgo_source_wnoj_20261005/final_receiver_checks/validation.json), [asynchronous validation](/home/yeranis/results/gnssfgo_source_wnoj_20261005/final_async_source_wnoj/validation.json), [stream equivalence](/home/yeranis/results/gnssfgo_source_wnoj_20261005/final_stream_source_wnoj/validation.json).

## Sanitizer scope and build caveats

Focused ASan/UBSan tests passed for the new wrapper, full prior/interpolator calls and synthetic optimization, with leak detection enabled. Prebuilt GTSAM and gnssFGO shared libraries were not rebuilt with sanitizer instrumentation; this is not a fully instrumented dependency stack or a full repository test campaign.

The first sanitizer attempt exposed an Eigen allocation-ABI mismatch: sanitized headers used the hand-aligned allocator while the prebuilt nonsanitized GTSAM library returned ordinary aligned glibc allocations. The focused sanitizer build now explicitly uses `EIGEN_MALLOC_ALREADY_ALIGNED=1` to match those libraries. No sanitizer check was suppressed and no estimator source was altered to resolve this build incompatibility. The initial failure is retained in `logs/sanitizer_tests_initial_abi_failure.log`; the successful run is [sanitizer_tests.log](/home/yeranis/results/gnssfgo_source_wnoj_20261005/logs/sanitizer_tests.log).

Existing empty virtual functions in `GPInterpolatorBase.h` still emit return-value warnings. The tested path uses the full implementation's overridden methods, not those empty base methods. No broad upstream cleanup was attempted.

## Files and reproducibility

Receiver integration is in [GrandTourCsvReplay.cpp](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp). CMake registers the new [C++ tests](/home/yeranis/repos/gnssFGO/online_fgo/test/test_source_wnoj.cpp). The [integration test](/home/yeranis/repos/gnssFGO/tools/test_source_wnoj_receiver.py) generates its own synthetic inputs; it has no dataset dependency.

The live launcher accepts an optional `receiver_model` field and defaults to `imu_wnoj`; its frozen JSON profile was not changed. Setting a field is not evidence of live validation. The isolated binary was not installed into the production workspace. Existing Rerun scripts that assume an IMU factor must not be used to label this new graph without checking their mode handling.

Exact compilation and test invocations are retained in `logs/commands_unit.json`, `commands_receiver.json`, `commands_regression.json`, `commands_sanitizer.json`, and `commands_integration.json`. They use these mounts:

```text
/audit   = /home/yeranis/results/gnssfgo_source_wnoj_20261005
/source  = /home/yeranis/repos/gnssFGO                            read only
/runtime = /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime
                                                                  read only
```

The isolated container has no network and a read-only root filesystem. Its local image is `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`. The additional existing `gtsam_unstable` headers were copied from the stopped build container, not downloaded. `LD_LIBRARY_PATH=/runtime/lib` supplies the frozen libraries during these tests.

The pre-edit working tree is saved in `baseline/`; [implementation.patch](/home/yeranis/results/gnssfgo_source_wnoj_20261005/implementation.patch) isolates this task from older uncommitted changes. [manifest.json](/home/yeranis/results/gnssfgo_source_wnoj_20261005/manifest.json) records code and protected-file hashes.

Production binary SHA-256, unchanged:

```text
0f175e4db81acb3624442ed7b3b232201e1b876676f2c148d0a747f009e706fa
```

Isolated experimental binary `/home/yeranis/results/gnssfgo_source_wnoj_20261005/build/grandtour_source_wnoj` SHA-256:

```text
51f05fcc6f90f04dfd7d239ddd100982030b4cf20bfdd40bd5ce43b93eca69c5
```

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created. The repository was already dirty and remains dirty, with unrelated user/research work preserved. `git diff --check` passes. The stock full/legacy WNOJ headers, causal support clock, ALL_RIGHT helper, ROS1 transport and frozen live configuration match the pre-edit snapshot byte-for-byte.

## Meaning and stopping point

The new receiver removes direct raw-IMU reuse from the fuser, including hidden IMU-derived acceleration and initialization. It does **not** remove the common IMU information or CBS feedback correlations already embedded in GLIM and Kimera beliefs. It therefore does not establish statistical independence, calibrated source covariance, or an expected CBS-ON improvement.

The bounded implementation is ready for a separately authorized matched frozen-journal experiment. No real-data accuracy, long-duration stability or live throughput result has been obtained for this mode. Existing experimental results remain untouched.

**SOURCE_WNOJ_SYNTHETIC_VALIDATED**
