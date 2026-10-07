# ETH-1 classic CBS: covariance evolution and receiver factor interaction

Date: 2026-09-30. Frozen-recording audit only. **No source rerun, estimator change, weighting change, or new sequence experiment.**

## Scope and principal result

The audited run is the first 120 s of GrandTour ETH-1 mission `2024-10-01-11-29-55`, with concurrently captured classic CBS-OFF and bidirectional classic CBS-ON GLIM/Kimera/gnssFGO. The receiver was given zero additional source-model uncertainty. Its exact event journals—not completed trajectory files—are the inputs to this *diagnostic* replay. The original experiment and settings are documented in [the live-run report](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md).

The accepted GLIM and Kimera physical intervals match exactly across OFF and ON. Their exported covariances remain finite, symmetric and positive definite, but **CBS-ON does not apply a common uncertainty scale**. Kimera's later revisions contract sharply, especially in rotation; its translation covariance can also *increase* relative to OFF for entire 10 s sections. CBS-ON changes the source means and local reference discrepancies at the same time. Therefore the worse fused ON ATE cannot scientifically be assigned to covariance contraction alone, and this ETH-1 result does not establish a universal WNOJ/IMU retuning rule.

The result is deliberately limited to ETH-1; no other sequence was executed here.

## Frozen source and receiver

- Source: Hesai GLIM and front AlphaSense stereo Kimera, both using the saved ETH-1 CPT7 profile; classic CBS, one pair/update, 0.20 s source horizon, persistent external factors and full Schur-relative-between 6×6 covariance. OFF published but did not ingest CBS; ON exchanged beliefs bidirectionally. No experimental `live_revision` source protocol.
- Receiver: independent 100 ms support clock; 59 startup IMU samples; ALL_RIGHT CPT7; Combined IMU, WNOJ `Qc=10000`, original priors, full-history iSAM2, causal arrival and stable-ID revision replacement, no GNSS, no gates/kernels/health weighting and `source_model_noise={}`. Every accepted source factor interpolates both endpoints.
- Run directory: [ETH-1 frozen capture](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/). [Analysis scripts, exact-journal outputs, summaries and plot](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/).
- The original receiver used source-revision **diagnostic log availability**, including log transport latency, not a direct ROS belief subscription. This audit preserves that exact recorded chronology.

Existing outcomes, copied from the retained evaluation (not rescored here):

| Actual trajectory | OFF ATE m | ON ATE m | OFF 1 s t-RPE m | ON 1 s t-RPE m |
|---|---:|---:|---:|---:|
| Native GLIM | 0.038007 | 0.028978 | 0.027251 | 0.033387 |
| Native Kimera ROS odometry | 0.541016 | 0.365236 | 0.045950 | 0.030284 |
| gnssFGO online/as-played | 0.051511 | 0.122472 | 0.031413 | 0.054332 |
| gnssFGO final retrospective | 0.048359 | 0.107042 | 0.015597 | 0.018430 |

These are different outputs. Native source ATE measures each source's original path, whereas the receiver optimizes their *relative beliefs* alongside IMU/WNOJ. Prism ATE has only 654 fused matches over 64.9 aggregate seconds in five reference segments, not continuous full-window coverage. Native Kimera has 315 prism matches and must not be compared as though it has the fused trajectory's mask. The retained 1 s RPE uses IE-TC.

## Input and factor accounting

The [pinned-input manifest](/home/yeranis/results/gnssfgo_eth1_covariance_audit_preparation_20260930/input_manifest.json) and [post-audit integrity check](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/frozen_input_integrity.json) agree byte-for-byte on all **76** saved input/reference/configuration files. The physical IMU sample rows are identical OFF/ON. Each receiver accepted 1,770 source IDs: 1,189 GLIM plus 581 Kimera, with 25 early snapshots skipped for unavailable support and zero pending events at end. OFF journal: 70,711 revision messages; ON: 70,748. There were no duplicate or stale receiver revisions. Revisions *replace* factors, not add an independent new factor per message.

Matching by `(source, from_timestamp_ns, to_timestamp_ns)` yields **1,189/1,189 GLIM** and **581/581 Kimera** unique exact physical interval matches, zero unmatched/ambiguous intervals; source indices also match for every pair. No nearest-neighbor timestamp substitution was used.

The receiver summary's maximum difference between its installed Gaussian factor covariance and the effective source matrix is below `7e-20` in both modes. Added source-model uncertainty is empty. The relative-belief tangent vector is GTSAM `Pose3::Logmap(Z)` in `[phi,rho]` order, and the receiver reconstructs it with `Pose3::Expmap`; the *complete dense 6×6 matrix* is passed to the Gaussian factor, including cross terms. See the earlier [tangent/frame audit](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_FROZEN_FACTOR_INFORMATION_AUDIT.md) and current [receiver loader](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:501).

## Covariance by revision age

For every accepted BeliefID, “first” is its first saved publication; `+1 s`/`+2 s` select the latest publication available by that age; “last” is its **last observed** publication, not an asserted final jointly valid source state. One ID contributes once to each age bin. Ratios below pair OFF and ON on the exact physical interval; each number is the median of per-edge ON/OFF block-trace ratios. Translation is the `rho` tangent block, not a new Cartesian measurement.

| Source/channel | First | +1 s | +2 s | Last observed |
|---|---:|---:|---:|---:|
| GLIM rotation | 0.833 | 0.797 | 0.790 | 0.786 |
| GLIM translation | 0.863 | 0.789 | 0.786 | 0.784 |
| Kimera rotation | 0.859 | 0.814 | 0.695 | 0.234 |
| Kimera translation | 1.505 | 1.377 | 1.145 | 0.437 |

The last-observed 10 s endpoint-bin medians vary: GLIM rotation `0.580–0.899`, GLIM translation `0.595–0.887`, Kimera rotation `0.068–0.561`, and Kimera translation **`0.061–5.701`**. In particular, Kimera translation is not simply “smaller under CBS-ON”: one 60–70 s section has median ON/OFF trace ratio `5.701`, while later sections are near `0.06–0.13`. See [the time-resolved plot](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/covariance_ratio_10s.png) and [all per-age/block statistics](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/covariance_age_reference.json).

The final median ratios' descriptive 95% intervals from a 10 s contiguous-block resampling (12 blocks, 2,000 deterministic resamples) are GLIM rotation `[0.726,0.834]`, GLIM translation `[0.699,0.837]`, Kimera rotation `[0.126,0.332]`, and Kimera translation `[0.130,0.700]`. These intervals summarize *within-run temporal dependence*, not variability across independent missions.

Median last-observed traces are GLIM OFF→ON: rotation `1.405e-8→7.838e-9 rad²`, translation `2.171e-6→1.663e-6 m²`; Kimera OFF→ON: rotation `2.000e-8→4.195e-9 rad²`, translation `1.416e-6→6.392e-7 m²`. An additional [all-revision health check](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/all_revision_covariance_health.json) covered **all 141,459 saved belief messages**, including messages not selected in the age tables. Its minimum full-matrix eigenvalue was `2.03e-10`, maximum symmetry error was zero at saved precision, and it found **zero nonpositive eigenvalues**. Minimum rotation trace was `2.42e-9 rad²` (ON GLIM); minimum translation trace was `2.14e-8 m²` (OFF Kimera). Thus no *numerical* collapse to zero occurred in this capture. The matrices can nevertheless represent very small physical uncertainty. Finite 120 s observations cannot prove a lower bound for future runs or source conditions.

The dense matrices are not interchangeable with one scalar trace: final pooled 3×3 generalized ON/OFF covariance eigenvalues range from `0.013` to `16.44` across source/channel/directions. This block-eigenvalue diagnostic is not a standalone full-6D weighting rule; cross-block terms also enter the receiver.

## Do smaller covariances track locally better belief means?

The saved CPT7 IE-TC `enu_origin→cpt7_imu` odometry was read from the existing bag. Each frozen belief was compared at its *own original physical endpoints* by interpolating reference poses with a 50 ms maximum gap, using `Log(Z⁻¹(T_i⁻¹T_j))` and also Cartesian relative-translation difference. All 1,770 accepted intervals were evaluable in both modes. This is a local relative-belief comparison, **not** native path ATE or the 1 s RPE. IE-TC shares CPT7 data, has no supplied covariance here, and source/GT correlations and possible timing systematics are unmodelled; the saved descriptive Mahalanobis discrepancies are not valid NIS/NEES calibration tests.

| Last-observed belief | OFF relative rotation median (deg) | ON (deg) | OFF Cartesian relative translation median (m) | ON (m) |
|---|---:|---:|---:|---:|
| GLIM | 0.1108 | 0.2019 | 0.00525 | 0.01489 |
| Kimera | 0.1091 | 0.1064 | 0.00289 | 0.00379 |

For first-publication Kimera, OFF→ON medians are `0.1070→0.2018 deg` and `0.00417→0.00786 m`; later ON revisions improve its rotation discrepancy substantially, while the last-observed translational discrepancy remains somewhat larger than OFF. GLIM's ON relative error is larger than OFF both at first and last. Its ON source mean itself changes: the matched final six-vector's median OFF/ON coordinate difference is `0.00310 rad` rotation and `0.01524 m` tangent translation. This is a *source-mean change in addition to* a covariance change. The native ON GLIM trajectory can improve ATE on the partial prism mask while its short local exported relative edges are less accurate against IE-TC; those are different evaluations.

## Exact-journal receiver interaction

The pre-existing SPX-3 [read-only factor exporter](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/) was reused as an isolated diagnostic executable. It adds graph export at support 300/600/900 and final drain; its analysis freshly evaluates each saved nonlinear factor's whitened residual/Jacobian at the actual iSAM2 estimate. It does not change a factor, update, prior, source covariance, or production executable. Fresh linearizations are **not** iSAM2's cached Bayes-tree linearizations, and local `JᵀJ`/precision derivatives are not a causal attribution of the optimized ATE or a retuned run.

The isolated executable is the already-built SPX-3 diagnostic binary (`SHA-256 b85c3e9aed77b42fe4a1e9e08bda844eb18f9eddd799b628eeb4fcbbf52338e4`), executed in the established ROS Humble container. It was given the *unchanged* ETH-1 OFF/ON `events.csv` journals with `--mode sources --timeline record --solver isam2 --support-policy uniform_imu --support-period-ns 100000000 --receiver-startup-samples 59`. The [replay identity ledger](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/replay_identity.json) shows **all 11 checked output files in each mode byte-identical** to the original receiver: final and online trajectory, final and online state rows, support clock/acceleration, model covariance diagnostic, belief residuals, source accounting, causal dispatch and unsupported beliefs. OFF took 620 s and ON 766 s without live pacing; these timings are diagnostic replay costs, not a new live-throughput claim. Only after this identity check were the factor exports interpreted.

At each graph snapshot, cost is `0.5 ||whitened residual||²` in the *freshly evaluated* graph. More factors accumulate as the support clock advances; compare OFF against ON **within a row**, not costs at different times as a per-edge change. Every paired row has equal OFF/ON factor counts per family.

| Support checkpoint | Mode | GLIM cost | Kimera cost | Combined IMU cost | WNOJ cost |
|---|---|---:|---:|---:|---:|
| 300 (~30 s) | OFF | 20,605 | 4,009 | 6,174 | 1,147 |
| 300 (~30 s) | ON | 112,989 | 10,518 | 10,470 | 1,706 |
| 600 (~60 s) | OFF | 30,577 | 6,424 | 8,477 | 2,266 |
| 600 (~60 s) | ON | 331,821 | 22,528 | 33,606 | 3,731 |
| 900 (~90 s) | OFF | 35,797 | 7,453 | 9,591 | 2,933 |
| 900 (~90 s) | ON | 416,085 | 26,338 | 40,161 | 4,701 |
| Final | OFF | 43,272 | 8,473 | 11,735 | 3,968 |
| Final | ON | 547,972 | 30,857 | 50,516 | 5,855 |

Four initial priors add final cost `<2` in each mode. The final graph has 1,189 GLIM, 581 Kimera, 1,192 Combined IMU and 1,192 WNOJ factors plus those four priors on 1,193 support states. At final, squared whitened residual *per residual coordinate* is GLIM `12.13→153.62`, Kimera `4.86→17.70`, IMU `1.31→5.65`, WNOJ `0.370→0.546` (OFF→ON). These are **post-fit graph residuals**, not pre-update NIS. They show that ON has more conflict with *several* frozen factor families; GLIM's term rises most. The original receiver's median physical post-fit residuals corroborate this: GLIM translation `0.00426→0.01258 m`, rotation `0.00941→0.01935 deg`; Kimera translation `0.00285→0.00323 m`, rotation `0.00934→0.00626 deg`.

In final fresh linearizations, the shares of the **summed pose-block diagonal of** `JᵀJ` are:

| Family | Pose rotation OFF → ON | Pose translation OFF → ON |
|---|---:|---:|
| GLIM | 87.69% → 75.84% | 0.32% → 0.50% |
| Kimera | 12.19% → 24.07% | 0.91% → 0.48% |
| Combined IMU | 0.12% → 0.10% | 98.00% → 98.25% |
| WNOJ | <0.01% → <0.01% | <0.01% → <0.01% |

The rest is mostly the initial priors. **These diagonal traces are not independent information after eliminating other variables**, nor “percentages of the final trajectory.” Source factors also connect interpolated support velocity/angular-rate states. Kimera's rotation-block share *doubles* from OFF to ON; it has a real role and is not simply ignored because GLIM is accurate. The direct pose-translation diagonal is dominated by Combined IMU, yet changing source precision can still move the optimized translation through graph cross-couplings.

To assess that coupling without modifying the graph, the existing diagnostic solves the fixed linearized normal equations and differentiates their optimum with respect to each family's *log precision*. After removing a common rigid displacement, the RMS support-position derivative is:

| Family | OFF local derivative (m) | ON local derivative (m) |
|---|---:|---:|
| GLIM | 0.0174 | 0.0388 |
| Kimera | 0.0160 | 0.0457 |
| Combined IMU | 0.00279 | 0.00963 |
| WNOJ | 0.000912 | 0.00150 |

Both GLIM and Kimera therefore have material *local* shape leverage under ON. This derivative is a sensitivity of one fixed fresh linearization, **not** the result of actually scaling a covariance, rerunning iSAM2 or scoring a new trajectory. Independent central differences of this fixed linear system agreed with the analytic source derivatives to better than `1.4e-6` relative; normal-equation solve residuals were `<4e-14` and the sum of all-family precision derivatives was `<2e-10` in maximum absolute state coordinate. The complete [per-checkpoint diagnostic JSONs](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/) retain each factor family's residuals, Jacobian curvature and sensitivity.

At final, the uneliminated rotation/translation family-gradient cosine for GLIM versus Combined IMU changes from `-0.826/-0.423` OFF to `-0.905/-0.956` ON. Negative values show their locally linearized gradient directions oppose one another on these coordinate blocks; they are **not** a calibrated conflict probability or unique causal explanation. The physical IMU samples are identical, WNOJ covariance diagnostics match exactly, and IMU preintegration covariance differences are tiny (`≤2.65e-11` in saved trace columns). Some support-frame linear accelerations still differ because they are constructed using the changing estimated orientation (RMS component differences `0.0374, 0.0407, 0.0231 m/s²`), so “same IMU file” does not mean all resulting nonlinear graph factors evaluate identically.

## Interpretation and limits

The simple story “CBS-ON gives both sources universally lower covariance, so the receiver trusts them too much” is **not supported as a complete explanation**. In ETH-1, confidence changes with source, channel, time and revision age; the relative means change; and their local IE-TC discrepancies do not improve uniformly. A globally fixed IMU/WNOJ change would not directly track this structure. This is not a proposed covariance correction or a claim that the source Schur covariance extraction is wrong. Missing source-source/common-information correlations, temporal/reference errors, and nonlinear model interaction remain possible mechanisms requiring separate controlled tests.

No next sequence, fusion tuning, or new estimator experiment was started.

## Validation and provenance

- Analysis-only files were created under `/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/`; the saved source/receiver artifacts were left unchanged, as verified above.
- Both diagnostic replays passed 11/11 byte-identities per mode. All four factor checkpoints in each mode were finite; graph residual/cost/Jacobian consistency and independent local-derivative finite differences passed.
- All 141,459 saved 6×6 covariance messages were finite/positive definite and symmetric at serialized precision; this is a numerical-health check, not a calibration certificate.
- The gnssFGO worktree was already dirty before this audit. Existing edits were preserved; no estimator/receiver production source was changed here. `git diff --check` passes.
- Original gnssFGO HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; source HEAD recorded by the live run: `db9ddcaa47c419ca59987746ff8ddd37410a23d3`. No new commit was made.
