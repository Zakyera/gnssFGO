# S4d — shared-native covariance provider, bounded live integration

**Result: bounded provider integration passed, with an observed overall GLIM
processing-time reduction on this matched 40-second test.** The candidate remains
private/default-off. This is not yet a long-run, real-time or covariance-accuracy
certification.

## Scope and implementation

This is the separately authorized candidate-only ETH1 first-40-second integration test
following S4c. The reference is the retained unchanged B0 first40s capture, not
the instrumented dual-provider S3 run. There is one candidate sensor run at 0.10×;
no longer run, tuning, estimator-profile edit or installed-library replacement.

Private selector: `GLIM_SHARED_NATIVE_PROVIDER=shared_native`. Unset, empty or
`independent_local` keeps the original provider. Unknown values are rejected.
The candidate is restricted to classic `active_window_temporary`, direct
outgoing means, `schur_relative_between`, and source `live_revision=false`.

The candidate does not allocate the independent nonlinear LOCAL sidecar. It
admits native factors before incoming CBS factors are appended, captures their
MAIN Gaussian cache blocks before retirement, and maintains its own native-only
Gaussian marginal history. The S4c rank certificate and unchanged eigensolver
fallback are reused. Endpoint joint covariance is transported from stored
linearization charts to the actual MAIN endpoint estimates; the existing Schur
relative-pose helper produces the outgoing full 6×6 covariance. MAIN continues
to supply the ordinary relative mean.

The publication adapter compares its extraction with the backend's result at
1e-10 relative tolerance, rejects jitter/nonfinite results, and commits output
only after success. Unavailable history, wrong charts, reset, retirement or
endpoint errors never fall back to MAIN covariance. The controller stops the
capture on provider/history failure.

### Important scope boundary: later revisions

The pre-existing normal first-publication route used LOCAL covariance, while
later diagnostic revision exports used MAIN covariance. This test replaces
only the former. The revision-export code is asserted text-identical to S3.
Consequently downstream gnssFGO still consumes the existing revised belief
policy; this is not a separate downstream covariance-policy experiment.

The new covariance means **native information evaluated at MAIN's recorded
linearization points**, with a separately maintained native-only prior. Explicit
CBS factor blocks and MAIN's mixed marginal prior are excluded. MAIN poses,
correspondences and nonlinear history can nevertheless be influenced by CBS.
Do not describe this as statistically independent CBS-OFF information or claim
equality with the old independent LOCAL covariance.

## Frozen setup

ETH1 mission `2024-10-01-11-29-55`, bag offset 0, duration 40 s, rate 0.10×;
Hesai/CPT7 and Alphasense front stereo/shared CPT7. GLIM lag 5 s, Kimera 25 states. The current
Kimera experimental IMU-support override stays unset; no old ALL_RIGHT pilot
override is silently introduced. Classic temporary active-window exchange,
native-IMU mean alignment, 0.20 s horizon, one pair/update, covariance scale 1,
no health weighting or new robustification. Rerun inspectors are disabled.

The concurrent receiver is the unchanged E2-cached source-driven WNOA graph:
no central IMU/bias/acceleration factors, independent 100 ms support clock,
full-history iSAM2, Qc=[0.00375,0.00375,0.00375,0.015,0.015,0.015], priors
pose 1e-5 / velocity 10 / angular-rate 3, stable-ID revision replacement. The raw CPT7
stream drives its clock, not central IMU measurement factors.

## Pre-capture validation

- 47/47 Release tests: retained lifecycle/history tests plus explicit selector,
  adapter/helper equality, zero added native evaluations, no MAIN mutation and
  transactional unavailable-endpoint failure.
- 18/18 existing regression tests; default-off MAIN control byte-identical
  to the original 659,210-byte control output.
- 47/47 focused ASan/UBSan tests pass after non-PIE relinking, leak checking on.
  The original PIE test failed before test output with `AddressSanitizer:DEADLYSIGNAL`;
  its artifacts are retained. Same sanitizer objects, no relaxed checks.
  Prebuilt GTSAM/CUDA dependencies are not sanitizer-instrumented.
- No-data startup passed: LOCAL disabled, provider enabled, zero sensor messages,
  expected mapped private libraries, normal owned shutdown.
- Effective ROS parameters and Kimera flags match B0 except output namespaces.
  Frozen receiver identity, input provenance, 715 protected paths and empty
  estimator/controller state verified before capture.

Build attempt `release_1` stopped at a configuration-check compile error:
the existing covariance-mode member is a string, not an enum. Corrected the
guard without changing the formula; successful build is `release_2`. No sensor
run was launched from either failed build/test admission.

## Capture, timing and validation results

All 391 scans completed in both captures. The candidate's controller finished
in 445.426 s versus B0's 445.373 s: playback deliberately paces 40 sensor seconds
at 0.10×, so total experiment wall time is not a useful unpaced-speed metric.

| Complete GLIM scan processing | B0 independent LOCAL | Shared-native candidate | Reduction |
| --- | ---: | ---: | ---: |
| Mean | 183.525 ms | 140.044 ms | 23.7% |
| Median | 206.942 ms | 167.421 ms | 19.1% |
| p95 | 437.376 ms | 312.422 ms | 28.6% |
| Maximum | 750.035 ms | 541.559 ms | 27.8% |
| Sum over 391 scans | 71.758 s | 54.757 s | 23.7% |

The predeclared p95 < 800 ms first-window budget at 0.10× passes. Improvement
is not uniform: early-window median rises from 29.962 to 34.703 ms, although its
p95 falls from 214.441 to 198.576 ms. Late-window median falls from 256.272 to
180.669 ms, and p95 from 469.574 to 352.022 ms. Windows are relative to the first
processed scan; the retained `summary.json` includes all bins and sample counts.

| Profiled component, inclusive totals | B0 | Candidate |
| --- | ---: | ---: |
| Independent LOCAL update | 3.415 s, 392 rows | Absent, zero rows |
| Ordinary covariance/publication scope | 4.164 s, 389 rows | 5.083 s, 389 rows |
| MAIN update scope | 50.719 s | 37.443 s |
| Revision-export scope | 8.346 s | 7.878 s |
| MAIN synchronous GPU linearizations | 185,378 | 169,890 |
| LOCAL cached GPU linearizations | 155,556 | 0 |
| Ordinary covariance cached GPU linearizations | 196,620 | 0 |

These inclusive scopes must not be added to the complete-scan sum. Candidate
history capture/commit is included in MAIN; it was **not separately timed** in
this capture. This is a measurement limitation relative to the protocol's
desired separate history-cost reporting. No additional timing run was made.

All 389 actual candidate queries were measured: mean 12.913 ms, median 9.926 ms,
p95 33.259 ms, maximum 46.312 ms; total 5.023 s. This includes the small repeated
Schur extraction used to retain existing diagnostics. The full covariance scope's
tail is slower than B0's (p95 33.409 versus 14.210 ms), even though the independent
LOCAL update has disappeared. Thus the new mechanism is not uniformly cheaper
at every substep.

**Attribution:** this is an observed whole-GLIM improvement, not proof that all
17.001 s saved came directly from deleting LOCAL. MAIN performed fewer GPU
linearizations and was faster too. The new covariance definition and live
arrival/optimization interleaving can change the coupled workload. This single
matched-setting pair is not an identical-input or repeated statistical timing
experiment. MAIN remains the largest measured component.

### Provider and source validation

- All 389 ordinary covariance results match all 389 GLIM ROS-wire belief
  matrices exactly, element for element at full saved precision.
- All are finite, symmetric positive definite: minimum eigenvalue across the
  389 matrices is 3.720375792e-10; maximum symmetry error is zero. This checks
  numerical validity, not calibration against actual estimation error.
- Zero independent LOCAL profile rows; zero GPU-factor linearizations or cache
  accesses in ordinary covariance scopes; no provider failures or resets.
- At the last queried epoch, 781: 5,299 admitted native factors = 4,491 absorbed
  into native-only history + 808 active; zero explicit removals. This is the
  last-publication checkpoint, not a claim that every shutdown callback was logged.
- Native incoming mean/covariance transport and diagnostic checks pass in both
  directions. K→G: 181 constructed; G→K: 180 constructed; no pre-stage rejection.
- Temporary-factor lifecycle checks pass: GLIM 156 removed/25 remaining;
  Kimera 156 removed/24 remaining; no rollback. This verifies logged factor
  lifecycle, not absence of every possible indirect CBS influence.

### Concurrent receiver and retained trajectories

There were 21,970 captured versions = 570 active IDs + 21,375 replacements +
25 versions before initialization. Those 25 all belong to the same earliest
Kimera interval and have the explicit unchanged `before_initialization` outcome;
they are not lost transport. The receiver consumed 3,986 clock IMU samples,
published 393 poses, had zero duplicates and ended with queue zero (maximum 46).
It still had no central IMU, bias or acceleration factors.

Maximum publication normalized gradient: 9.921024397e-6, below the frozen 1e-5
threshold. Live output age: median 0.050008 s, p95 0.090005 s, max 0.104784 s
in simulation time. All nine saved receiver repeat files are byte-identical,
including online and final state/trajectory files, dispatch and measurement events.

| Retained candidate output | Rows | Physical span | Path length |
| --- | ---: | ---: | ---: |
| Original/native GLIM | 391 | 38.999880 s | 14.011266 m |
| Original/native Kimera ROS odometry | 184 | 39.599778 s | 13.785152 m |
| Fused online/as-played | 393 | 39.200000 s | 15.275147 m |
| Fused final retrospective | 393 | 39.200000 s | 13.983070 m |

The existing Kimera startup gap is 3.199979 s; afterwards its maximum gap is
0.200001 s. The GLIM maximum gap is 0.100990 s. This is a timing/integration
experiment, not an ATE comparison; no partial-GT score is presented as whole-run
accuracy. Native outputs are not replaced with GLIM-only/Kimera-only receiver
diagnostic trajectories. The retrospective output is not labelled online.

## Decision and preserved state

Accept **S4d bounded private integration**. The supervisor's shared-equation
route can supply actual ordinary outgoing covariance without a second nonlinear
LOCAL optimizer in this test. Overall scan processing time also improved in
this capture. Do not promote global defaults, claim calibrated uncertainty,
claim complete downstream revision-policy replacement, or extrapolate to the
160-second tail. Stop here for the next decision.

Integrity checks pass: 680 baseline files, 19 S1 evidence pins, 716 S3 protected
paths, the 7/8/10 S4a/S4b/S4c identity sets, and all six source repository
HEAD/clean-status checks. Installed libraries, source profiles and receiver code
remain untouched. The S4c backend/rank code and S3 lifecycle hooks are byte-identical.
Provider, numerical, native transport and lifecycle evidence are archived under
`evidence/`; detailed timing is in `summary.json`.

Private bridge SHA-256:
`0114b435785ae85f58a69f1156abd1a2a320dc7138ca0b2dd4bfd56715d4ebea`.
Private gtsam_points SHA-256:
`42fb1e8a931ebc86a06ae41a2eaf6ff39b9efbe5a1a4fd9737ee06edb2274228`.

## Reproducibility and recovery

Authoring scripts: this directory. Private build root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_provider_20261012`.
Capture root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s4_provider_20261012`.
Use the saved run root's `prepared/run_case.py profile40 on` only for provenance;
it deliberately refuses duplicate launches. No installed defaults are changed.

Parent commit: `c22a7aad39320f737ef0b368eb56191765dcee29`.
Branch: `engineering/shared-native-provider40-20261012`.
The completion commit containing this report and both updated plans is the final
local HEAD (resolve with `git log -1 --format=%H -- experiments/shared_linearization_s4_provider_20261012/REPORT.md`).
No push is authorized in this task. Both containers were checked after validation;
no GLIM, Kimera or receiver process remained running.
