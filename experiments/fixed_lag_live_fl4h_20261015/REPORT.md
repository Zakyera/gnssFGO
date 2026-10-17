# FL4h — what happened inside Kimera in the ETH1 120 s CBS-ON run

Date: 2026-10-15. **Saved-data investigation only. No source or fusion run, rebuild, estimator edit, covariance change, or tuning.**

## Finding and confidence

The severe degradation is already present **inside Kimera's source smoother**, before its output reaches gnssFGO. It is not explained by a corrupt GLIM payload, missing endpoint alignment, a growing arrival delay, or merely the ROS trajectory export. The recorded GLIM constraints remain centimetre-level against the full-window reference, while Kimera's updates develop large deformations of the very relative edges being constrained. Later, existing recovery guards roll back updates and discard otherwise accurate GLIM constraints.

The code exposes a specific numerical vulnerability: one Gauss–Newton update per keyframe, with marginal-prior-connected poses excluded from relinearization even when CBS requests a full solve. Recorded marginal priors sometimes touch **25 of the 26 currently retained pose states**. This is a well-defined next hypothesis, **not a proven unique root cause or a reason to remove GTSAM's fixed-variable protection**. The saved logs do not include the complete original-chart objective and values needed to distinguish an inadequate nonlinear step from an already inconsistent objective/prior.

## 1. Exact evidence and evaluation scope

Capture root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared`.

Inputs: both `pilot120_{off,on}_source/estimators.log`, native `kimera_odometry.csv`, actual ON `recordings/cbs.bag`, effective parameters/copied profiles, and Kimera's `ros_logs/b72a69ce-c816-11f1-a6aa-267c7c544b29/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log`. The existing IE-TC reference decoder/interpolator is reused for `/boxi/inertial_explorer/tc/odometry` in mission `2024-10-01-11-29-55_cpt7_ie_tc.bag`.

The primary original-trajectory result remains the [FL4g common-mask table](../fixed_lag_live_fl4g_20261015/REPORT.md): native Kimera ATE **0.458070 OFF → 3.745897 m ON** over the same 115.9 s common support. No 11-second subset replaces that evaluation. Here, local relative transformations are compared at their exact physical endpoints, in the shared CPT7 body frame. They are **not** 1-second RPE or new native-trajectory scores. IE-TC shares CPT7 with the estimators; its errors are not independent or covariance-free ground truth.

Times below are seconds from the frozen bag-clock origin `1727775007`, not wall elapsed time. Kimera's first/last saved poses are at approximately 1.155/120.755 s on that axis, spanning 119.60 s. The timeline includes the entire saved run.

## 2. Payload, timing, and alignment checks

- All **580** G→K aligned candidates match an actual GLIM wire belief: sender indices, physical timestamps, raw six-coordinate mean, and all **36 covariance entries**. The transported and constructed covariance entries are exactly unchanged; raw-mean comparison passes `1e-12`. No pre-stage rejection occurs in this capture.
- Every candidate at backend update `u` connects Kimera's **`x(u−2) → x(u−1)`**. This is the previous physical edge, not the new edge currently being published. Median source-endpoint-to-processing-epoch age is **180.224 ms**, range **179.234–182.656 ms**. This is a physical-epoch age, not a measured wall-clock network latency. There is no progressive endpoint-age growth here.
- Receiver endpoints are about **19.776 ms earlier** than the selected GLIM endpoints. The enabled native-IMU bridge performs the recorded temporal mean alignment. Its translation change is median **2.583 mm**, p95 **6.054 mm**, maximum **11.126 mm**.
- Across all 580 aligned GLIM relative measurements, translation discrepancy against IE-TC is median **7.231 mm**, RMS **9.810 mm**, maximum **32.090 mm**. It stays around 1 cm RMS late in the run; it does not jump to the metre-scale discrepancies seen inside Kimera.

This rules out a gross payload/endpoint/alignment failure in these measurements. It does **not** prove that GLIM's small systematic errors, correlations, or covariance calibration are harmless.

## 3. When Kimera deteriorates

The following are RMS translation errors of **native Kimera's consecutive approximately 0.20 s increments** against the same physical reference intervals:

| Bag-clock section | OFF | ON |
|---|---:|---:|
| Before 75 s | 4.73 mm | 6.26 mm |
| 75–90 s | 23.32 mm | 35.53 mm |
| 90–105 s | 8.77 mm | 123.22 mm |
| 105 s through the saved end | 22.66 mm | 174.26 mm |

Both modes encounter difficult visual tracking, but their later behavior differs sharply. There are **38 OFF versus 39 ON** recorded stereo-tracking-failure warnings. ON's first is associated with backend281 / 60.355 s; its first new-edge translation discrepancy above 5 cm is backend368 / 77.755 s. The first above 10 cm is backend438 / 91.755 s. OFF has a similar weak section around 75–90 s, but its local errors then recover substantially. Warning count alone cannot establish feature quality or explain ON's much larger degradation.

Two decisive individual updates:

| Backend update / clock time | Target edge | Aligned GLIM error vs reference | Kimera edge error before | Kimera edge error after | What happened |
|---|---|---:|---:|---:|---|
| 438 / 91.755 s | 436→437 | **6.45 mm** | 16.08 mm | **432.48 mm** | Factor retained; bad deformation already occurs in the graph update |
| 549 / 113.955 s | 547→548 | **5.60 mm** | 76.98 mm | **5.927 m** | Initial trial rejected; this is the final result after retry without the new factor |

For update438, the directly logged residual **to the GLIM measurement** changes from 22.24 mm to 428.40 mm. For update549, the first trial has a 9.538 m residual to GLIM, is rolled back, and the final retry still has a 5.933 m residual. These are different stages; the rejected 9.538 m trial is not the published result.

An independent SE(3) reconstruction from saved endpoint poses reproduces all pre-update residual norms within `6.7e-13`; for the **562 retained candidates**, it reproduces BPSAM's pre-marginalization post-update norms within `6.7e-13`. Thus update438's bad relative motion is already created during optimization, **not by an immediate pose jump in the subsequent `marginalizeLeaves()` call**. The previously accumulated marginal prior can still influence that optimization.

## 4. What the solver is doing

The frozen Kimera `BackendParams.yaml` has `numOptimize: 1`, `useDogLeg: 0`, `relinearizeThreshold: 0.01`, `relinearizeSkip: 1`, `nr_states: 25`, `wildfire_threshold: 0.001`.

- [VioBackend.cpp:5021](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:5021) performs the main smoother update. The extra-iteration loop at [5083](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:5083) does not execute with `numOptimize=1`.
- CBS graph changes request extra re-elimination, `force_relinearize=true`, and `forceFullSolve=true` at [bpsam.cpp:4236](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:4236). **This is not a nonlinear convergence certificate.**
- [ISAM2.cpp:738](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/nonlinear/ISAM2.cpp:738) wraps the marginal Gaussian and puts its keys in `fixedVariables_`. [ISAM2-impl.h:381](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/nonlinear/ISAM2-impl.h:381) explicitly removes those keys from relinearization candidates even for a forced full solve. “Fixed” means a fixed **linearization point**, not an immovable estimated pose.
- The retained factor inventory confirms the broad separator: before update444, the marginal prior contains `x418…x442` plus two other variables, while the existing pose window is `x418…x443`. Similar 25-pose priors recur later. The requested 25-state lag uses the smoother's strict cutoff, hence 26 retained pose epochs at this boundary.

The saved pre-update family diagnostics also show a severe internal inconsistency:

| Update / clock time | Existing IMU factors | Recorded IMU cost | Existing external CBS cost | Smart-stereo cost |
|---|---:|---:|---:|---:|
| 124 / 28.955 s | 25 | 2.92e3 | 2.82e4 | 248.46 |
| 384 / 80.955 s | 25 | 1.19e8 | 3.55e4 | 57.01 |
| 444 / 92.955 s | 25 | 6.80e8 | 1.90e5 | 27.63 |
| 508 / 105.755 s | 25 | 1.48e10 | 4.53e6 | 27.95 |
| 558 / 115.755 s | 25 | 1.55e11 | 3.80e6 | 26.19 |

These are the existing factors' recorded errors at the current estimate, **not factor weights, a same-graph before/after total cost, or evidence that a particular family caused the first jump**. Smart factors can return small/zero contribution when invalid, so a low visual cost does not prove good visual support. The marginal prior's logged zero cost is also not evidence of no prior influence: [LinearContainerFactor.cpp:78](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/nonlinear/LinearContainerFactor.cpp:78) returns zero when no explicit linearization point is attached, as in this iSAM marginal representation.

Consequently the evidence supports **source-side nonlinear graph conflict/poor updates with a strongly frozen marginal separator**, but does not yet separate finite-step/linearization error from an objective whose retained prior/constraints are inconsistent. Increasing iteration count, changing the lag, or disabling fixed-variable protection is not justified as an automatic fix.

## 5. Existing recovery cascade and a reporting correction

The source mode is Gaussian with scale1 and health/Huber disabled, **but it is not guard-free**. BPSAM has pre-existing residual-based transactional recovery:

- A new factor's post-step residual above `max(1.25*pre + 0.25, 0.5)` causes rollback and a retry **without that new CBS factor** ([bpsam.cpp:4590](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:4590)).
- An older external factor above the existing mixed-residual threshold2 triggers rollback, removal of that factor, and retry ([4640](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:4640)). These thresholds mix the existing SE(3) coordinates; they are not calibrated NIS tests.
- The first rollback is update482 / **100.555 s**, after deterioration was already visible. It removes older edge475→476 and discards newly staged480→481. Thus recovery cannot explain the initial onset, but it creates a later path that removes corrective measurements when the receiver disagrees with them.

The validated nested-transaction checker reconciles **580 constructions − 18 undone admissions = 562 retained admissions**, followed by **533 normal window removals + 11 guarded old-factor removals**, leaving18 active. There are22 rollback transactions:11 staged-regression and11 active-regression; some are nested, so this is not22 independently dropped new factors. All restored slots, counts, and receiver edges reconcile; no duplicate live slot or wrong-slot removal was found in these logs.

**Correction to FL4g:** its580 G→K “injected” count was a candidate/construction count, not successful retained admission. The report now notes this distinction. Runtime completion and downstream deterministic replay did not certify Kimera's nonlinear stability. The18 dropped candidates and11 old-factor removals are source-side behavior; they do not contradict conservation of the separate downstream revision journal.

## 6. Covariance and original-output semantics

Incoming aligned GLIM formal uncertainty is small: median RMS-per-axis standard deviation **1.108 mm** in translation and **0.004525°** in rotation. Against the relative reference, median error norms are7.231mm and0.152592°. The full six-dimensional reference-residual Mahalanobis statistic has median2053.6. This statistic omits reference uncertainty/correlation and must **not** be promoted to a calibrated innovation test or a GT-derived covariance scale. It nevertheless warns against equating a low native GLIM ATE with perfectly consistent, infinitesimally uncertain short-edge factors. No covariance was changed.

Kimera's original ROS output composes each new increment from the current smoother poses ([VioBackend.cpp:5460](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:5460)); it does not rewrite older published increments when the next delayed GLIM factor arrives. The saved native increment agrees with the current-state consecutive relative pose to a maximum translation discrepancy of **7.49e-11 m**. Thus the export itself is behaving as implemented. There is both real relative deformation and accumulating state/output separation, not merely a plotting alignment issue. The recorded state/output position gap reaches50.49m ON versus1.87m OFF; that gap alone is not an ATE, but the relative-edge checks independently establish non-rigid error.

## 7. Bounded next diagnostic; not executed

The useful next target is **the source update around438 (approximately92 s), before recovery begins**, not another fusion experiment or covariance sweep. If authorized, use a single normal-start source diagnostic ending just beyond that event and capture the actual in-process graph, all X/V/b values, fixed keys/deltas and original marginal-prior chart immediately before/after the update. Compare the production step with a carefully constructed same-objective reference step and separate visual, native IMU, external, and prior residual/gradient contributions. Any reference solve must preserve the marginal prior's original chart; evaluating the zero-return container as a nonlinear prior would be invalid. A reproduced run must identify its own event if it does not match the historical one.

The current retained logs lack that full graph/velocity/bias/prior checkpoint. The older `kimera_bad_update_audit` helper is hard-coded to unrelated EIG-2 updates1082–1094 and was disabled here. It cannot retrospectively answer this question. No diagnostic capture or helper modification has been launched in this task.

## Artifacts and non-regression

Derived read-only products are in `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4h/`: `mechanism_summary.json`, `off_update_trace.csv`, `on_update_trace.csv`, **`factor_details_v2.json`**, `stage_checks.json`, `lifecycle_validation.json`, and `failure_timeline.png`. The earlier exploratory `factor_details.json` is retained; v2 correctly aggregates multiple typed prior groups. The plotted three families are unchanged by that reader correction.

![Saved source-failure timeline](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4h/failure_timeline.png)

Analysis scripts are retained beside this report. `analyze.py`/`details.py` run with the existing Noetic container Python and existing reference decoder; defaults point to the original `/workspace/cbs_gtsam4.3/runs/...` capture. The container does not mount the gnssFGO repository, so the executed copies live in `audit_fl4h`. The host lifecycle script reuses the previously validated nested-recovery contract; the plot script uses the existing evo environment. No new dependency was installed.

Repeated saved-data parsing reproduces the numeric summaries exactly. Wire/covariance/cutoff assertions pass; the lifecycle checker and independent stage-residual reconstruction pass. Runtime-loaded Kimera, GTSAM and private CBS-library hashes still match the frozen manifest (`libkimera_vio` `8ca57e0b…`, GTSAM `0307b153…`, private CBS `82f85c2a…`). No source/binary/profile edit, new estimator experiment, or newly claimed source test-suite validation. The only changes are analysis tools, this report, the ledger, and the explicit earlier-report accounting clarification.

**Status: failure localized to Kimera's source graph update and subsequent recovery; unique underlying numerical/model cause not yet proven.**
