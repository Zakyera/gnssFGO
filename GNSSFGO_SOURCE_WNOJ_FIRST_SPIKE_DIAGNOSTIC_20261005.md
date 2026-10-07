# ETH1 source WNOJ first spike diagnosis

The bounded diagnostic identifies **both incomplete incremental optimization and poor motion constraints at the current source-only operating point**. Re-optimizing exactly the same graph removes most of its objective error, but does not remove excessive velocity and acceleration. This does not prove that central raw IMU is necessary. It also does not validate the current no-central-IMU model as a replacement.

The next engineering work should address convergence and validate WNOJ motion uncertainty, separately. No source covariance, Qc, initialization, production solver setting or estimator implementation was changed in this task. No setting was selected using ground truth.

## Scope and protected baseline

This continues [the frozen ETH1 40 second comparison](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_SOURCE_WNOJ_40S_COMPARISON_20261005.md). Only the first **eight seconds of its saved receiver journals** were replayed, independently OFF and ON. GLIM and Kimera were not rerun. This is offline causal receiver diagnosis, not a live experiment or a new full-sequence comparison.

Artifacts: [/home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005](/home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005).

The replay preserves the existing `source_wnoj` X/V/W/A state, full WNOJ prior/interpolator, independent 100 ms supports, four startup priors, Qc diagonal 10000, full-history iSAM2, stable-ID source-factor replacement, complete native source covariance and scale one. No central raw IMU value is used in this mode; its timestamps continue to advance the common support clock. Classic source CBS and the original source settings remain unchanged.

All journal rows retain their original order, physical timestamps, arrival times and payloads. The prefix begins at `1727775008003475000 ns` and ends inclusively at `1727775016003475000 ns`; later arrivals are excluded. Each mode contains 800 IMU clock rows and 2572 belief-version messages, with 75 initialized supports. Each ends with 93 active source IDs, comprising 71 GLIM and 22 Kimera. Equal counts do not imply equal OFF/ON payloads.

Input SHA-256:

```text
OFF 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed
ON  a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

## Snapshot selection and comparison

The selection rule was the first saved as-played speed greater than 5 m/s, plus a snapshot two supports before and ten supports after that crossing. This is a diagnostic trigger, not a measurement gate, a calibrated motion limit, or a claim to have found the earliest infinitesimal error. It was selected before running the batch controls and without GT scoring.

The first crossings are OFF support 38 at stream offset 4.38 s and ON support 58 at 6.38 s. The six snapshots are OFF supports 36/38/48 and ON supports 56/58/68. They are intentionally not identical OFF/ON timestamps; this diagnoses each run's first excursion and must not be read as an accuracy ranking between modes.

At each snapshot, immediately before the usual online output:

1. Copy the actual current iSAM2 graph and estimate, including the currently active version of every source factor, all WNOJ factors and the original priors. Do not reconstruct factors from a final trajectory.
2. Run batch Levenberg–Marquardt on this exact information set, initialized at the incremental estimate. No future source revision or later support is available to the batch graph.
3. Restart LM from its result to check numerical stability. Do not feed either solution back into replay.
4. Record all support poses, V/W/A, factor errors, source IDs/revisions, information coverage and linearized numerical diagnostics.

LM uses the existing GTSAM implementation, Ceres-style damping defaults, multifrontal QR, maximum 500 iterations, relative/absolute objective-decrease tolerances `1e-11`, then a restart capped at 100 iterations. Damping is an optimizer device, not an added factor or altered probabilistic prior. These settings are diagnostic only.

The original factor slots, keys, dense covariance matrices and residuals evaluated at the fixed initial estimate are serialized and checked identical before/after optimization and between original/copied graphs. The receiving replay continues from its untouched iSAM2 instance.

## Same graph results

Objective means the actual graph cost `0.5 * sum(r^T R^-1 r)`, including priors and WNOJ. It is not ATE or a calibrated NIS statistic. Batch below includes the convergence restart.

| Mode | Support | Offset s | Incremental objective | Batch objective | Emitted speed m/s | Batch speed at same newest state m/s |
|---|---:|---:|---:|---:|---:|---:|
| OFF | 36 | 4.18 | 6265.656 | 2.638 | 3.319 | 2.473 |
| OFF | 38 | 4.38 | 29200.329 | 10.680 | 6.121 | 4.524 |
| OFF | 48 | 5.38 | 625079.997 | 230.452 | 19.459 | 10.039 |
| ON | 56 | 6.18 | 375.466 | 43.962 | 4.424 | 4.137 |
| ON | 58 | 6.38 | 4201.287 | 72.388 | 7.785 | 4.783 |
| ON | 68 | 7.38 | 8220.271 | 161.239 | 6.523 | 5.672 |

The batch solutions have column-normalized gradient infinity norms between `9.14e-8` and `1.42e-6`. Restarting changes any support position by at most `1.26e-8 m` and velocity by at most `3.62e-7 m/s`. This is strong evidence of local numerical convergence, not proof of the unique global optimum.

The large cost reduction demonstrates that the saved incremental states were not close to these lower-cost solutions at publication. The remaining excessive speeds demonstrate that convergence alone is not sufficient under the unchanged objective.

### Which factors carried the optimization error

At the first selected crossing in each mode, the largest incremental cost is the Kimera factor group. Re-optimization sharply reduces it without removing or reweighting those factors.

| Snapshot | Factor family | Incremental objective contribution | Batch contribution |
|---|---|---:|---:|
| OFF support 38 | GLIM | 0.414678 | 0.408222 |
| OFF support 38 | Kimera | 28751.740982 | 0.058945 |
| OFF support 38 | WNOJ | 448.173094 | 10.212956 |
| OFF support 38 | Initial priors | 0.000165 | 0.000165 |
| ON support 58 | GLIM | 7.269406 | 7.274724 |
| ON support 58 | Kimera | 3302.579991 | 13.150229 |
| ON support 58 | WNOJ | 891.437160 | 51.962544 |
| ON support 58 | Initial priors | 0.000086 | 0.000085 |

This locates the residual reduction; it does not establish that Kimera's measurements are wrong, or that Kimera covariance should change. Source correctness versus GT was not tested here.

## Extra incremental updates without new information

For the two first-crossing snapshots only, copy the actual iSAM2 object and perform up to 20 additional empty updates with the existing `force_relinearize=true` option. No new factor, value, covariance or prior is supplied. The production instance remains untouched.

| Snapshot | Original objective | After one extra update | After 20 extra updates | Batch objective |
|---|---:|---:|---:|---:|
| OFF support 38 | 29200.328918 | 10.696820 | 10.681985 | 10.680288 |
| ON support 58 | 4201.286643 | 72.414291 | 72.421065 | 72.387581 |

One extra update removes most of this optimization error. Its newest speeds are still 4.522 m/s OFF and 4.781 m/s ON. Twenty updates do not eliminate the remaining motion problem. These iterations are not guaranteed monotonic; the ON cost rises slightly after the first extra update.

`calculateBestEstimate()` on a separate copy gives exactly the original objective at iteration zero, so merely requesting full back-substitution does not remove the initial discrepancy. Later it makes small differences. Also, `force_relinearize` checks the existing delta threshold; it does not force every variable to a completely fresh linearization regardless of threshold.

This supports testing a bounded convergence check before publication. It does **not** certify “one extra iteration” as a universal production fix or establish why every preceding incremental update stopped where it did.

## Prediction tail and already covered states

At all six snapshots, the newest Kimera endpoint precedes the current physical output time by approximately 28.1 ms. The newest GLIM endpoint precedes it by approximately 208.4–208.6 ms. Those are endpoint-to-output ages, not necessarily identical to transport delays.

| Snapshot | Batch newest speed m/s | Maximum batch speed at supports no later than the newest source endpoint m/s | Newest translational tangent acceleration norm m/s² |
|---|---:|---:|---:|
| OFF 36 | 2.473 | 1.165 | 24.525 |
| OFF 38 | 4.524 | 1.897 | 43.214 |
| OFF 48 | 10.039 | 3.853 | 93.759 |
| ON 56 | 4.137 | 1.296 | 36.595 |
| ON 58 | 4.783 | 1.584 | 43.717 |
| ON 68 | 5.672 | 2.097 | 51.959 |

The largest speeds are at the newest state, but the problem is not exclusively beyond all available measurements. The “covered” column is a timestamp-coverage diagnostic: it does not claim every derivative is directly measured or equally well constrained, nor that both sources cover every listed state. Delaying publication may help, but cannot alone be assumed to repair the model.

## What the frozen WNOJ uncertainty permits

The receiver constructs Qc using `Diagonal::Variances`, not standard deviations. Its existing [calcQ3](/home/yeranis/repos/gnssFGO/online_fgo/include/utils/GPUtils.h:93) implements the full correlated WNOJ interval covariance. With scalar coordinate spectral density `q=10000` and `dt=0.1 s`, the translational diagonal blocks are:

| Local process block | Variance expression | Standard deviation per coordinate |
|---|---|---:|
| Pose translation increment | `q*dt^5/20` | 0.07071 m |
| Velocity increment | `q*dt^3/3` | 1.82574 m/s |
| Acceleration increment | `q*dt` | 31.62278 m/s² |

These are marginal blocks of a correlated process covariance, not three independent penalties or measured robot motions. The rotational blocks have corresponding angular units. The translational A variable follows the full model's tangent-acceleration convention; it is not raw specific force and should not be equated blindly with Cartesian world acceleration.

Actual QR-based linearized marginal covariance at the batch solutions confirms substantial derivative uncertainty:

| First-crossing snapshot | Newest world velocity standard deviations x/y/z m/s | Newest translational tangent acceleration standard deviations m/s² |
|---|---|---|
| OFF 38 | 2.162 / 2.205 / 2.187 | 34.578 / 34.653 / 34.648 |
| ON 58 | 2.193 / 2.150 / 2.219 | 34.595 / 34.557 / 34.706 |

The last five X/V/W/A marginal blocks are finite and positive definite; maximum symmetry error is `1.43e-14`. Their existence does not make the uncertainties calibrated. The column-normalized information matrices have very small positive eigenvalues (about `3.63e-10` to `6.10e-10`), indicating a wide spread of local information scales. This is not an exact unobservability or gauge-rank proof.

**Inference:** the unchanged Qc and source constraints permit substantial derivative freedom after central IMU removal. This makes motion-prior calibration a justified next target. No Qc intervention was performed here, so the experiment does not isolate Qc as the sole cause or establish a correct replacement value.

## Actual state Jacobian checks

At OFF support 38 and ON support 58, every active factor was checked both before and after batch optimization, using independent central differences with step `2e-6` and the same manifold retraction. The production full-WNOJ numerical derivative path uses `1e-5`. The checks cover 80 factors OFF and 130 ON, including all source factors, WNOJ and the four priors.

Maximum absolute Jacobian differences were `1.234e-6` OFF and `4.795e-6` ON. Maximum entry differences normalized by `1+abs(numerical_entry)` were `1.231e-6` and `1.983e-6`. These checks reveal no large derivative mismatch at the selected states. They do not prove all implementations, manifolds or future operating points correct, and are not a replacement for the preceding synthetic tests.

## Recommended next step

Keep the existing IMU-based baseline unchanged and the source-only model experimental.

1. In the isolated source-only branch, test an objective-based, bounded convergence check before emitting an estimate. The same-snapshot result gives a concrete reason to do this; do not blindly assume a fixed iteration count always suffices. Do not change the source factors.
2. Separately test the sensitivity of the converged source-only graph to physically justified WNOJ jerk uncertainty. Distinguish translation from rotation and keep startup priors fixed for that test. Use motion/uncertainty plausibility and source residuals before GT accuracy, and freeze any candidate identically for OFF and ON before cross-sequence validation.
3. Only after the supported trajectory is healthy, decide the allowed publication latency and how to label extrapolated versus measurement-supported outputs.

The current evidence neither requires reintroducing raw IMU nor guarantees a successful no-IMU receiver. It also cannot guarantee that CBS-ON should outperform CBS-OFF. Removing central IMU does not remove correlations already embedded in the source beliefs.

## Validation and reproduction

- Production source, configuration, binaries, original journals and earlier artifacts are unchanged: 234 protected files pass their SHA-256 checks.
- Both diagnostic copies reproduce the unmodified experimental binary byte-for-byte in 12 numerical/accounting output files. Revision-event rows match exactly after excluding wall-clock update duration.
- Their as-played state rows are byte-identical to the corresponding prefix of the original 40 s run.
- Repeated first-crossing snapshots reproduce factor probes, ledgers, batch states, polished states and errors byte-for-byte.
- Every batch graph is a copy of actual active factors; no factor or covariance changes, no future information, no feedback of batch results.
- The optimized diagnostic builds succeeded. Stock empty-base-method compiler warnings remain; no library cleanup was undertaken. This task does not claim a new complete test-suite or sanitizer run.
- No GT file or scorer was used. No ATE-based parameter selection, source rerun, long-sequence rerun, new live experiment, or production change was performed.

Preparation, execution, analysis and validation are retained in `prepare.py`, `run.py`, `analyze.py`, `analyze_deep.py`, `verify.py`, their command records, `manifest.json`, `analysis.json`, `deep_analysis.json` and `verification.json` under the artifact directory. C++ changes exist only in its `build/DiagnosticReplay.cpp` and `build/SnapshotAudit.h`; the repository receiver remains untouched.

Containers use the existing image `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`, no network, a read-only root filesystem and UID/GID 1000. Mounts are the new artifact directory read/write at `/audit`, the repository read-only at `/source`, the retained runtime at `/runtime`, and the frozen experimental build at `/baseline` for regression checks. `LD_LIBRARY_PATH=/runtime/lib`. Nothing was installed or downloaded.

Receiver command, independently for `mode=off/on`:

```bash
WNOJ_AUDIT_INDICES=INDICES /audit/build/diagnostic_replay \
  --receiver-model source_wnoj --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59 \
  --event-stream /audit/inputs/MODE_events.csv --out /audit/MODE
```

Use indices `36,38,48` OFF and `56,58,68` ON. For the additional snapshot validation, `WNOJ_AUDIT_DEEP=1`, select only `38` or `58`, use `diagnostic_replay_deep` and a fresh output directory. The current diagnostic source also reproduces the primary run with the deep flag absent. Exact compilation flags and commands are in `commands_build*.json`; execution records are in `commands_primary.json`, `commands_deep.json` and `commands_regression.json`.

Diagnostic binary hashes:

```text
primary 1c470f651587dbeb928352e1766bafa519430e945df36e22b3c174a1acba93eb
deep    b20abde854a24070db47692fd9ae68951af72c4047f00094a9dc72b230fb4db1
```

Repository HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. The pre-existing dirty worktree was preserved; this task adds this report, not a production implementation change. `git diff --check` passes.

**Conclusion:** two distinct problems are demonstrated at these snapshots: incomplete incremental convergence and an unchanged objective that still accepts unhealthy motion after local convergence. The no-central-IMU approach remains possible in principle, but is not yet validated for deployment.
