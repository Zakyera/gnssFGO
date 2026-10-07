# ETH1 WNOA: frozen prediction and uncertainty audit

Date: 2026-10-07. This completes the approved Step 2 following
[numerical convergence recovery](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_NUMERICAL_CONVERGENCE_RECOVERY_20261006.md).

## Conclusion

**The tested WNOA interpolation and prediction covariance are numerically
consistent, but reliable current-time prediction and calibrated uncertainty
are not yet certified.** The outstanding sensitivity is in estimated motion,
not a detected failure to propagate covariance or connect interpolation endpoints.

For the retained approximately 120 s ETH1 run, the largest next-publication
revision of the *same physical pose* is 9.35 cm OFF and 14.02 cm ON. The largest
same-time velocity revision is 0.555 m/s OFF and 1.170 m/s ON. Some revisions are
large compared with the previous graph's nominal marginal uncertainty. These
are changes of estimates, **not ground-truth errors**, and their normalized
sizes are **not NIS/NEES or chi-square consistency tests**.

The largest ON position-revision event already changes velocity substantially
at the last source-connected support. Extending that provisional motion into
the source-free prediction interval increases the visible position correction.
This does not identify whether new beliefs, historical revisions, source error,
correlations, or a model mismatch caused the original derivative change.

No GLIM/Kimera run, receiver replay, graph optimization, parameter tuning,
production integration, or GT scoring was performed. No covariance, factor,
arrival policy, or saved estimator output was changed.

## 1. Inputs and frozen model

Saved outputs:

- [OFF](/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/off_wnoa)
- [ON](/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/on_wnoa)

They contain 1193 publication snapshots each, including final flush, with
1193 support states through 119.78 s after the retained journal start. The
recorded IMU-timestamp span is 119.850027 s. Four initial publications have no
source factors; the first source-informed support publication is at 1.08 s.
There are 1188 source-informed next-publication comparisons per mode. One
late current timestamp never obtains source endpoint coverage before cutoff:
first-coverage statistics have 1187 comparisons rather than inventing coverage.

Original journal root:
`/home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/inputs`.

| File | SHA-256 |
|---|---|
| `off_events.csv` | `44d89960cc69941ec13e2e6299120ef3eb8398e16cffa2149d521af8be948590` |
| `on_events.csv` | `8ba7ea3a306de88c763635380ddc6f359cd5219f688a46d253779cfe271715fe` |

Unchanged model: Pose3 X, world velocity V, body angular rate W; no independent
acceleration, central IMU factor, bias state, or gravity initialization. Sources
still use their own IMU processing. The independent 100 ms support clock uses
raw IMU timestamps only, with the frozen 59-timestamp startup offset. WNOA Qc
diagonal is `[.00375,.00375,.00375,.015,.015,.015]`; initial priors are identity
X with sigma 1e-5, zero V with sigma 10, zero W with sigma 3. Full native source
covariances, scale 1, classic source CBS, causal availability, stable-ID revision
replacement, and full-history graph semantics remain unchanged. No gating,
robust kernel, GNSS, health weighting, or covariance scaling is introduced.

## 2. Definitions and coverage

At each saved publication:

- **Current**: newest support state's physical timestamp, not computer wall time.
- **F, exact source frontier**: latest physical endpoint of an already inserted
  source factor, considering both sources—not a GLIM master clock.
- **Covered support**: latest support at or before F.
- **Connected boundary**: right interpolation support of F, or F itself if exact.
  Source factors can touch this support even though it lies after F.
- **Pure prediction suffix**: supports after that connected boundary. We
  explicitly verify no source factor touches them in reconstructed graphs.

Each next-publication comparison queries the old state's *unchanged physical
timestamp* in the next saved graph. It does not compare two moving current
states at different times. Exact-frontier pose queries use the stock WNOA
interpolator. First-coverage and final retrospective comparisons use the same
old support timestamp; neither later estimate is assumed to be truth.

Publication availability is distinct from physical support time. All 2,091,228
retained ledger entries checked satisfy arrival <= that publication's recorded
availability. The largest arrival after its physical support timestamp is
36.229 ms; this is not an arrival-before-availability violation. The delay table
below is relative to physical support time, not wall-clock computational latency
and not the latency of each individual historical revision.

| Latest endpoint age | OFF | ON |
|---|---:|---:|
| Median | 128.108 ms | 128.109 ms |
| p95 | 128.714 ms | 128.719 ms |
| Maximum | 208.451 ms | 209.033 ms |
| Publications with a nonempty pure prediction suffix | 609 | 613 |
| Maximum pure prediction duration | 200 ms | 200 ms |

Both sources continue arriving in this window. **No claim about seconds-long
source outages or arbitrary extrapolation is supported by this test.**

## 3. Same-time state revisions over the full saved window

The following are next-publication changes at the old current timestamp,
including both new source IDs and revisions delivered in that publication batch.

| Quantity | OFF median / RMS / p95 / max | ON median / RMS / p95 / max |
|---|---:|---:|
| Position, cm | 0.949 / 1.754 / 3.518 / 9.350 | 1.917 / 2.813 / 5.235 / 14.018 |
| Orientation, degrees | 0.333 / 0.922 / 1.927 / 4.515 | 0.367 / 1.173 / 2.430 / 4.837 |
| Velocity, m/s | 0.0797 / 0.1193 / 0.2260 / 0.5552 | 0.1452 / 0.2143 / 0.4144 / 1.1702 |
| Body angular rate, rad/s | 0.1272 / 0.1655 / 0.2925 / 0.9398 | 0.1404 / 0.1814 / 0.3209 / 0.8589 |

| Same-time position comparison | OFF RMS / max, cm | ON RMS / max, cm |
|---|---:|---:|
| Covered support -> next publication | 0.880 / 6.009 | 1.819 / 8.815 |
| Exact source frontier -> next publication | 1.047 / 7.156 | 1.725 / 8.566 |
| Connected boundary -> next publication | 1.173 / 7.666 | 1.823 / 8.984 |
| Current -> first later source endpoint coverage | 2.030 / 9.844 | 2.861 / 14.018 |
| Current -> final retrospective estimate | 2.166 / 10.575 | 3.453 / 12.833 |

Maximum published speed is 1.057 m/s OFF and 1.462 m/s ON; maximum published
body angular-rate norm is 0.867 rad/s OFF and 1.046 rad/s ON. These are finite
observed magnitudes, not independently validated velocity truth or predefined
physical acceptance thresholds. A lower revision or smoother path is not
necessarily a more accurate path.

## 4. Covariance reconstruction and growth

All state comparisons above cover the full publication history. Covariance is
recovered for a bounded selection of **99 OFF + 94 ON graphs**: every 20th
source-informed publication, first/final, five largest delay/correction/rate
events per selection criterion, and selected original full-payload audit points.
The pre-analysis [protocol](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/PROTOCOL.md)
and [selection reasons](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/selection.json)
are retained. This is a stress sample, not a population-calibration experiment;
even the regular 2 s sample can be phase-locked to source cadence.

For each selected graph, restore Values, factor slots, active stable-ID versions,
source means/full covariances, original priors, and stock WNOA factors from the
frozen records. **No optimization occurs.** QR marginalization uses the complete
graph and cross-covariance, not independent diagonal blocks. Stored 12D state
marginals have ordering `[local phi, local rho, world V, body W]`; position
statistics are rotated into world coordinates. Endpoint uncertainty uses
`H P_joint H^T`, including both supports and their cross blocks.

For each WNOA-only suffix step, reuse the *actual existing factor Jacobians*:

```text
F = -H_j^{-1} H_i
Q_step = H_j^{-1} Q_factor H_j^{-T}
P_next = F P_previous F^T + Q_step
```

This local-Gaussian propagation matches direct current-state graph
marginalization to maximum relative matrix error **1.123e-14**. All exported
queried joint marginals are finite and positive definite. Accumulated process
covariance is PSD; the empty-suffix case is correctly zero. This is a numerical
test of the chosen model, not proof that its uncertainty matches physical errors.

In every selected nonempty suffix, position, orientation, velocity and angular-rate
variance traces grow from the connected boundary to the current support. Across
these 32 OFF / 26 ON selected suffixes, median fractions of current variance
trace supplied by newly accumulated process noise are:

| Component | OFF | ON |
|---|---:|---:|
| Position | 4.78% | 4.87% |
| Orientation | 19.67% | 23.60% |
| Velocity | 58.68% | 61.69% |
| Angular rate | 65.11% | 69.00% |

The complement is propagated joint boundary uncertainty, including all cross
terms. These are **variance decompositions, not proportions of a correction's
cause**. Position uncertainty also accumulates over the longer relative-only
trajectory; it does not simply collapse toward zero.

For reference, the regular covariance sample's median `sqrt(trace P)` is:

| Current component | OFF | ON |
|---|---:|---:|
| Position | 3.147 cm | 3.069 cm |
| Orientation | 0.002702 rad | 0.002649 rad |
| Velocity | 0.06296 m/s | 0.06385 m/s |
| Angular rate | 0.02968 rad/s | 0.02963 rad/s |

`sqrt(trace P)` is a three-coordinate aggregate, not a per-axis standard
deviation, 95% radius, or confidence guarantee. Full sampled distributions,
directional scales and covariance-normalized revision magnitudes are retained
in [summary.json](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/summary.json).

## 5. Largest correction: what changes before prediction?

CBS-ON publication **201 -> 202**, at 20.68 s after journal start, has the largest
next-publication position change. Its old current timestamp is
`1727775028683475000` ns, F is `1727775028555285500` ns, and its pure prediction
suffix is 100 ms. The next snapshot has **2 new source IDs and 72 previously
present IDs whose revision has changed**.
They were not separated into causal ablations in this task.

| Location, same physical timestamp before/after | Position change | Velocity change |
|---|---:|---:|
| Covered support 199 | 0.653 cm | 0.174 m/s |
| Connected boundary 200 | 4.317 cm | 0.807 m/s |
| Current prediction 201 | 14.018 cm | 1.170 m/s |

At current support 201, the previous directional position standard deviation
along the eventual change is **0.916 cm**, versus a 14.018 cm change. The analogous
velocity scale is **0.0513 m/s**, versus 1.170 m/s. The speed estimate at that
same timestamp changes from **1.462 to 0.371 m/s**. At connected boundary 200,
the velocity directional scale is 0.0334 m/s versus its 0.807 m/s change.

Only 6.10% of the current position variance at this event is new process noise
from its final 100 ms; most is inherited. The large change is therefore not
located solely in an otherwise well-determined state propagated too far into
the future. The source-connected motion estimate is itself provisional.
This does **not** establish a unique cause or justify a covariance multiplier.

The largest OFF position event, publication **817 -> 818** at 82.28 s, also has
2 new IDs plus 72 previously present IDs with changed revisions and a 100 ms
prediction suffix. Its connected
boundary/current position changes are 7.666/9.350 cm; velocity changes are
0.176/0.171 m/s. There is no universal decomposition in which all corrections
come only from extrapolation.

Normalized revision distances use the earlier local marginal. They are not
innovation or estimation-error statistics: factor replacement changes the
historical objective, new information need not be independent, the later
solution is not truth, and the nonlinear approximation changes. Large ratios
warn against interpreting the reported marginal as a bound on future revisions;
they do not on their own quantify statistical overconfidence or its cause.

## 6. Interpolation and hidden between-support motion

Sampled the last five 100 ms intervals of every saved snapshot at 10 ms, plus
the entire final history: 157,124 pose rows, representing 14,284 intervals with
overlapping local-history queries. Online local intervals must not be summed
and called a trajectory length.

| Final retrospective interpolation | OFF | ON |
|---|---:|---:|
| Support-to-support polygon length | 72.0591 m | 73.3374 m |
| Same history sampled at 10 ms | 72.1727 m | 73.9582 m |
| Maximum per-interval arc-length excess over chord | 3.440 mm | 17.910 mm |
| Maximum sampled perpendicular departure from endpoint chord | 4.506 mm | 14.092 mm |
| Maximum projected overshoot beyond endpoint chord | 1.673 mm | 4.370 mm |
| Maximum sampled finite-difference speed | 1.090 m/s | 1.559 m/s |

There is genuine between-support curvature/occasional small overshoot, especially
ON. A support-only drawing hides it. Curvature is not automatically an artifact
or error: the pose and derivative estimates jointly determine the GP curve.
No truth-based judgment about these excursions is made here.

The largest ON final arc-length excess occurs at interval 198, near the same
20 s region: chord 2.891 cm, sampled arc 4.682 cm over 100 ms. These are final
retrospective values, not information that was available at the original time.

Read-only one-sided endpoint differentiation was checked at 1930 endpoint
locations with steps 10 us and 1 us. At 1 us, maximum velocity discrepancy from
the saved support V is **1.042e-5 m/s**, and angular-rate discrepancy from W is
**1.363e-5 rad/s**. Discrepancies shrink approximately tenfold with the tenfold
step reduction. Exact endpoint limits and fractional constant-velocity fixtures
also pass. No endpoint velocity-convention mismatch was detected.

### Remaining covariance approximations

The actual adapter is
[SourceWnoaModel.h](/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/SourceWnoaModel.h).
`WnoaEndpoint::pose` calls the stock
[GPWNOAInterpolator](/home/yeranis/repos/gnssFGO/online_fgo/include/model/gp_interpolator/GPWNOAInterpolator.h:200).
`WnoaRelativeFactor::unwhitenedError` retains native source R and uses that
conditional interpolation mean. It does **not** add a separate conditional GP
bridge covariance or account for residual correlations between multiple
interpolated measurements. Queried endpoint `H P_joint H^T` is uncertainty of
this mean under uncertain supports, not the complete continuous-process posterior.

Using the same local-chart Q/Phi definitions in
[GPUtils.h](/home/yeranis/repos/gnssFGO/online_fgo/include/utils/GPUtils.h:39), the
prior conditional bridge covariance with both boundary states fixed is:

```text
C = Q(tau) Phi(dt-tau)^T
Q_bridge(tau) = Q(tau) - C Q(dt)^-1 C^T
```

For dt=100 ms, its maximum position-chart per-axis standard deviation is
0.2795 mm and rotation-chart value 0.00013975 rad (at mid-interval). This is a
local prior bridge calculation, **not** an evaluated replacement R, nonlinear
posterior calibration, or explanation of the observed 14 cm revision. Two
relative endpoints can have correlated interpolation errors; simply adding
two independent endpoint variances would not generally be justified.

Shared source IMU, overlapping source beliefs, and classic CBS feedback also
remain possible information correlations. Removing the central IMU factor
does not make the two sources or successive factors independent. None of
these approximations was changed or experimentally isolated here.

## 7. Figures

Red curves show same-time next-publication revisions. Blue points show the
earlier nominal marginal aggregate at regular samples, not confidence bands.
Both OFF/ON plots use matching vertical scales.

![Prediction revisions and uncertainty](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/prediction_and_uncertainty.png)

![Largest same-time correction events](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/largest_revision_events.png)

## 8. Validation, provenance and reproduction

- All 193 reconstructed graphs match saved factor slots, key lists, dimensions
  and costs; maximum normalized per-factor cost discrepancy is 1.216e-10.
- Sixteen selected snapshots also retain complete dense payload probes:
  covariance discrepancy **zero**, maximum unwhitened residual discrepancy
  **1.221e-13**. No covariance was reconstructed from a diagonal approximation.
- All 1158 exported SPD matrices pass independent NumPy checks; minimum
  eigenvalue 5.425e-9, maximum symmetry error 1.728e-19. Process-only covariance
  is separately checked PSD, including zero-duration cases.
- Five covariance export files and three interpolation export files repeat
  byte-identically. Focused ASan/UBSan synthetic tests and six real graphs
  (first, largest-position-revision, final in each mode) pass. Sanitized exports
  match their Release subsets exactly. Frozen external libraries themselves
  are not sanitizer-instrumented; this is not the complete ROS test suite.
- Independent NumPy subtraction/solve reproduces the ON201 velocity revision
  distance, 22.795087211343, without treating it as NEES.
- All **11,923 protected input/source/previous-result files** retain their hashes.
  Existing dirty worktree changes are preserved. No production file is changed;
  only this new report and the new diagnostic artifact directory are added.
- Initial host analysis checks exposed two analysis-script representation
  issues: nanosecond timestamps coerced through a floating Pandas row, and exact
  float equality selecting no 1-us derivative rows. They were corrected to
  integer timestamp parsing and an explicitly counted tolerant step selection.
  No C++ graph result, estimator output, threshold, or model was changed to
  resolve those host-script checks. The original bounds remain satisfied.
- The existing base interpolator emits compiler warnings for unused non-returning
  virtual stubs; the called WNOA pose/derivative paths pass the focused checks.
  No library modification was made.

Artifacts and reproducible scripts:
[gnssfgo_eth1_wnoa_prediction_audit_20261007](/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007).
`prepare.py` records all-publication changes and selected factor payloads;
`audit.cpp` reconstructs/linearizes but never optimizes; `velocity_check.cpp`
checks endpoint derivatives; `analyze.py` validates and produces statistics/plots.

The existing frozen Docker image is
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`.
Its network and root filesystem writes are disabled; prior artifacts, runtime
and repository are read-only mounts. Only the new artifact directory is writable.
Full compiler argv and wall times are in `commands_*.json`. Release covariance
recovery took 46.62 s; repeat 41.61 s; saved-state interpolation 6.55 s. These are
analysis costs, not fusion throughput measurements.

Commands used, from the new artifact directory:

```bash
/home/yeranis/.local/share/pipx/venvs/evo/bin/python prepare.py
bash run.sh build
bash run.sh primary
bash run.sh means
bash run.sh sanitizer
bash run.sh repeat
bash run.sh velocity
bash run.sh means_repeat
/home/yeranis/.local/share/pipx/venvs/evo/bin/python analyze.py
```

Output creation for C++ exports is exclusive. Reproduction requires a fresh
artifact root and corresponding mount/output paths, not overwriting these files.
The initial unprivileged Docker call was denied at the socket; the authorized
scoped container invocation then completed. No dependencies were installed.

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created.
`git diff --check` passes. No diagnostic process remains running.

## 9. What this completes—and what it does not

We can keep WNOA as a **numerically coherent experimental motion model** for
these short observed prediction intervals. We should not claim its current-pose
uncertainty is calibrated or that an accurate final historical trajectory makes
the original predictions accurate. There is no new native GLIM/Kimera/fusion
ATE comparison here, and no conclusion that CBS-ON is more or less accurate.

If further mechanism work is authorized, the bounded next target is the saved
ON201 -> 202 change: distinguish the effect of its two new source IDs from its
72 historical revisions on boundary velocity. That would test the source of
the measured sensitivity without another full sequence, model switch, or
covariance sweep. It has **not** been run in this task.

**NUMERICALLY_CONSISTENT_PREDICTION_RELIABILITY_NOT_CERTIFIED**
