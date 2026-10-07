# SPX-3 source uncertainty and bounded covariance sensitivity

Date: 2026-09-29. Scope: the frozen **120-second headless live** CBS-OFF/ON experiment, followed by **offline diagnostic receiver replays**. This is not a new live experiment and does not adopt new production weights.

**Outcome:** all eight diagnostic replays completed with unchanged factor identities, dispatch, support clock and scoring masks. Modestly favoring GLIM relative to Kimera improves ATE in both modes, but does not eliminate the matched OFF–ON gap; RPE trade-offs remain. Kimera is demonstrably influential, not ignored. No Schur-complement defect or statistically calibrated production multiplier is established.

## Question and controlled protocol

Does source covariance explain why improved native Kimera under classic CBS-ON does not automatically improve fusion? Follow-up to [the four-family interaction audit](GNSSFGO_SPX3_LIVE120_FACTOR_INTERACTION_AUDIT.md).

The predeclared test changes one source's **entire dense 6×6 covariance** by 0.8 or 1.25, separately in OFF and ON: eight runs. Covariance ×0.8 means precision ×1.25; covariance ×1.25 means precision ×0.8. Rotation/translation cross terms remain intact. No per-axis tuning, sweep expansion, or production selection is authorized by these diagnostic results.

Frozen: actual source means, stable IDs/revisions and arrival order; independent 100 ms IMU-driven support grid; 59-sample startup; receiver ALL_RIGHT; Combined IMU; WNOJ/Qc variance 10000; initial priors; full-history iSAM2; stable-ID replacement. Source CBS remains classic, not experimental live_revision. No source rerun, GNSS, gating, robust loss or health weighting. Native GLIM and native Kimera outputs never change in this study.

"Same IMU/WNOJ" means identical measurements, configuration and construction algorithm. Their realized preintegration bias seeds and fixed WNOJ acceleration inputs depend on earlier estimates, so changing source weights can change subsequent realized dynamics factors. These replays measure that complete existing receiver response; they are not an optimization of one otherwise numerically identical frozen final graph.

Artifacts and scripts: [experiment directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_covariance_sensitivity_20260929/). `experiment_manifest.json` records all eight cases and SHA-256 identities before replay. `prepare.py` asserts only the selected covariance fields change; all other event fields/lines retain identity.

## Native and unmodified fusion references

Same physical scoring window: 1730565903.588829994–1730566022.688653946. ATE: existing AP20 prism evaluator, CPT7-to-prism lever arm, SE(3) alignment without scale. RPE: existing IE-TC relative evaluator, 1 s horizon / 0.1 s sampling, both endpoint poses converted to prism. Prism coverage is partial, approximately 94.9 s over five segments, not the full 120 s.

| Original output | OFF ATE m | ON ATE m | OFF t-RPE m | ON t-RPE m | OFF r-RPE ° | ON r-RPE ° |
|---|---:|---:|---:|---:|---:|---:|
| Native GLIM | 0.007046 | 0.015811 | 0.007028 | 0.022587 | 0.275979 | 0.574259 |
| Native Kimera ROS odometry | 0.655087 | 0.170926 | 0.176565 | 0.030378 | 0.614367 | 0.535886 |
| Fused online/as-played | 0.069367 | 0.078541 | 0.027762 | 0.047590 | 1.750184 | 1.911003 |
| Fused final retrospective | 0.047570 | 0.066802 | 0.011439 | 0.012325 | 0.380927 | 0.651236 |

These are the original/native GLIM, original/native Kimera and their fusion—not GLIM-only or Kimera-only receiver graphs. Baseline results are reused from the retained evaluation; scale 1 was already replay-verified byte-identical by the previous audit.

## Exact-interval source/reference audit

For each accepted stable ID, compare its first observed and latest received revision to IE-TC interpolated at its exact two physical endpoint timestamps. Use the existing evaluator's exact position interpolation / quaternion interpolation functions, with its 0.05 s reference-gap limit. No source pairing or temporal realignment is introduced. Only intervals with both endpoints inside the scoring window are included: 1190 GLIM and 589/591 Kimera OFF/ON. Two accepted Kimera intervals per mode cross the mask boundary and are excluded from this diagnostic only, not from filtering.

The source is `Z = Exp(mu)` and reference increment is `T_i^-1 T_j`. Compute `e = Log(Z^-1 (T_i^-1 T_j)) = [phi,rho]` in **CPT7 coordinates**. These are approximately 0.2 s source intervals, not the trajectory table's 1 s prism RPE. `rho` is an SE(3) Log coordinate; separate Cartesian translation discrepancies are retained in the CSV. Installed GTSAM has `GTSAM_POSE3_EXPMAP` enabled. Independent diagnostic calculations match `Pose3::localCoordinates` in 200 synthetic cases to maximum absolute error 1.33e-15; Exp/Log round trips are also checked.

| Mode / source | Revision | rho RMS mm | Rotation RMS ° | Median dense source-normalized squared discrepancy |
|---|---|---:|---:|---:|
| OFF GLIM | first | 3.365 | 0.123623 | 88.48 |
| OFF GLIM | latest | 2.575 | 0.113097 | 87.85 |
| ON GLIM | first | 16.132 | 0.441441 | 792.03 |
| ON GLIM | latest | 13.772 | 0.431038 | 1354.28 |
| OFF Kimera | first | 38.552 | 0.162983 | 273.54 |
| OFF Kimera | latest | 26.020 | 0.152024 | 583.87 |
| ON Kimera | first | 10.209 | 0.178187 | 573.24 |
| ON Kimera | latest | 3.910 | 0.139315 | 1360.90 |

This is not merely a comparison of different Kimera intervals. On the **585 exactly matching OFF/ON Kimera intervals**, latest rho RMS is 26.109→3.928 mm and rotation RMS 0.152543→0.140025°. All 1190 GLIM intervals match OFF/ON exactly. Thus the source-level pattern survives an exact-interval comparison.

Revisions reduce RMS error in all four source/mode combinations, but do not guarantee lower median error or better covariance calibration. For example, ON GLIM latest rho RMS improves versus first while its median rises 6.65→8.60 mm. Kimera covariance contracts enough that source-normalized discrepancy grows even as its absolute disagreement improves.

### What the covariance numbers do—and do not—prove

Dense discrepancy is `eᵀ Sigma^-1 e`. Separate block discrepancies use the 3×3 rotational/translational **marginals**, not Schur conditioning; the block values are not additive because cross terms are nonzero.

| Latest factors | Median trace phi rad² | Median trace rho m² | Median phi discrepancy | Median rho discrepancy |
|---|---:|---:|---:|---:|
| OFF GLIM | 9.4423e-8 | 1.7863e-6 | 68.60 | 6.83 |
| ON GLIM | 4.0610e-8 | 1.6170e-6 | 1197.39 | 140.00 |
| OFF Kimera | 6.0831e-8 | 1.0841e-5 | 398.09 | 19.56 |
| ON Kimera | 8.7370e-9 | 8.3712e-7 | 1225.28 | 29.18 |

These restricted-mask Kimera medians differ slightly from the previous all-accepted-factor table. Matrices are finite, symmetric within 1e-8 and positive definite. A small nonzero covariance is not itself evidence of correct confidence.

**Reference limitation:** all 24,001 IE-TC poses within the 120 s reference interval have zero ROS pose-covariance fields. That does not establish zero physical reference error. Cross-time reference covariance is unavailable; IE-TC also uses CPT7, shared with these estimators. Therefore these numbers are **source-normalized reference disagreement**, not calibrated NEES, independent innovations, a formal chi-square rejection test, or evidence for a specific covariance multiplier. No confidence intervals based on independent edges are claimed.

Directional checks use each marginal's covariance eigenvectors ordered by increasing eigenvalue. Median squared standardized rho projections, smallest/middle/largest uncertainty directions:

| Latest factors | Smallest | Middle | Largest |
|---|---:|---:|---:|
| OFF GLIM | 1.89 | 0.92 | 0.84 |
| ON GLIM | 34.48 | 16.93 | 18.46 |
| OFF Kimera | 4.78 | 2.87 | 3.27 |
| ON Kimera | 5.26 | 5.98 | 5.88 |

The ON GLIM increase is not confined to one covariance eigendirection. These are local uncertainty axes, not a single fixed world axis across the run.

## Dependence and uncertainty provenance

Every adjacent GLIM interval overlaps its predecessor; consecutive Kimera intervals do not overlap but share endpoints and estimator information. Latest-factor lag-one **body-coordinate rho error** correlations are GLIM OFF (0.773, 0.726, 0.497), ON (0.609, 0.584, 0.456); Kimera OFF (0.983, 0.977, 0.876), ON (0.542, 0.715, 0.472). These are descriptive time-series correlations; changing body frames, common reference error and nonstationarity preclude interpreting them as exported cross-covariance estimates.

There are **zero exactly identical GLIM/Kimera interval pairs** in either mode. This audit does not manufacture a cross-source covariance by correlating different physical increments. Cross-source dependence is structurally expected from shared CPT7 and CBS, but its quantitative covariance and contribution to ATE remain unmeasured.

As traced in the prior audit, GLIM later revisions use the current full smoother's endpoint marginal, including received CBS information. Kimera's LOCAL covariance extraction excludes tracked explicit CBS/anchor factors but does not certify removal of information in an inherited marginal prior or the current mean. The receiver keeps full within-belief covariance but receives no cross-edge/source/central-IMU covariance. Stable-ID replacement removes superseded versions; it does not make different IDs statistically independent.

## Bounded nonlinear replay results

All eight completed successfully. Every result uses exactly 954 prism matches and 1181 one-second RPE intervals. The complete prism timestamp/match mask equals the corresponding baseline, not merely its count.

| Mode / covariance change | Online ATE m | Online t-RPE m | Online r-RPE ° | Final ATE m | Final t-RPE m | Final r-RPE ° |
|---|---:|---:|---:|---:|---:|---:|
| OFF unchanged 1.0 | 0.069367 | 0.027762 | 1.750184 | 0.047570 | 0.011439 | 0.380927 |
| OFF GLIM ×0.8 | 0.062406 | 0.026040 | 1.798770 | 0.039727 | 0.010671 | 0.392113 |
| OFF GLIM ×1.25 | 0.077710 | 0.029605 | 1.684621 | 0.056878 | 0.012428 | 0.370135 |
| OFF Kimera ×0.8 | 0.077284 | 0.029736 | 1.774574 | 0.056695 | 0.012295 | 0.382586 |
| OFF Kimera ×1.25 | 0.063013 | 0.025910 | 1.723926 | 0.039934 | 0.010778 | 0.379493 |
| ON unchanged 1.0 | 0.078541 | 0.047590 | 1.911003 | 0.066802 | 0.012325 | 0.651236 |
| ON GLIM ×0.8 | 0.073595 | 0.050124 | 1.953145 | 0.059802 | 0.012568 | 0.680595 |
| ON GLIM ×1.25 | 0.084146 | 0.045301 | 1.872780 | 0.073767 | 0.012216 | 0.621940 |
| ON Kimera ×0.8 | 0.085034 | 0.046736 | 1.926495 | 0.073770 | 0.012373 | 0.651144 |
| ON Kimera ×1.25 | 0.072188 | 0.048358 | 1.888117 | 0.059775 | 0.012351 | 0.650497 |

Remember: **smaller covariance means more influence**. Only one source is scaled in each row; the other stays at 1.0. Online columns are the causal/as-played output of these offline journal replays, not newly co-running live estimators.

Interpretation:

1. **Kimera already has substantial influence.** Increasing its precision by 25% (covariance ×0.8) raises final ATE OFF 0.047570→0.056695 m and ON 0.066802→0.073770 m. Native Kimera's large CBS improvement does not imply that increasing its receiver weight will improve fusion.
2. Reducing Kimera precision by 20% (covariance ×1.25) lowers final ATE OFF to 0.039934 m and ON to 0.059775 m. The matched ON-minus-OFF gap is still **19.84 mm**, compared with **19.23 mm** at scale 1. Thus this tested reweighting improves both graphs but does not recover a CBS-ON advantage. Every tested matched setting still has worse ON final ATE.
3. Increasing GLIM precision also lowers ATE, but ON final t-RPE and r-RPE worsen, and online RPE worsens. Conversely, reducing GLIM precision improves ON RPE while worsening ATE. Global position fit and local relative accuracy are not interchangeable objectives.
4. Kimera covariance ×1.25 improves OFF's three final metrics in this one window. In ON it improves ATE but changes final t-RPE slightly adversely (0.012325→0.012351 m), and online t-RPE worsens (0.047590→0.048358 m). The tiny final RPE differences are descriptive, not statistically significant improvement claims.
5. No tested result approaches native GLIM's ATE. This bounded sweep demonstrates weight sensitivity, not that a scalar multiplier repairs source representation/dependence or makes the receiver source-preserving.

### Agreement with the previous local influence calculation

At all 1195 common support timestamps, compare each final trajectory to its scale-1 baseline. Convert the previous local position derivative to world coordinates and multiply by `log(1/c)`. Remove the best infinitesimal common rigid translation/rotation from both actual and predicted displacements. This measures shape response, not gauge movement or ATE.

| Case | Actual shape RMS mm | Predicted shape RMS mm | Prediction error RMS mm | Direction cosine |
|---|---:|---:|---:|---:|
| OFF GLIM ×0.8 | 7.626 | 8.276 | 0.695 | 0.999528 |
| OFF GLIM ×1.25 | 8.962 | 8.276 | 0.721 | 0.999667 |
| OFF Kimera ×0.8 | 8.764 | 8.097 | 0.712 | 0.999566 |
| OFF Kimera ×1.25 | 7.455 | 8.097 | 0.679 | 0.999591 |
| ON GLIM ×0.8 | 6.975 | 6.998 | 0.213 | 0.999542 |
| ON GLIM ×1.25 | 6.920 | 6.998 | 0.279 | 0.999259 |
| ON Kimera ×0.8 | 6.900 | 7.031 | 0.332 | 0.999043 |
| ON Kimera ×1.25 | 7.001 | 7.031 | 0.130 | 0.999836 |

The small finite/nonlinear differences are expected: the prediction is a derivative of a fixed linearized final problem, while the real test changes all revisions and replays state-dependent factor construction. Their agreement provides positive evidence that the prior influence calculation captures the observed response. It does not prove global optimality or rule out other modeling limitations.

### Source-factor fit is not ground-truth accuracy

For ON, increasing Kimera influence reduces its median final rho residual to its own measurement from 2.579 to 2.450 mm, while final ATE worsens. Decreasing its influence allows that residual to rise to 2.698 mm while ATE improves. Fitting one source more closely is not synonymous with fitting ground truth more closely.

`validation.json` retains each source's unwhitened residual median and total squared whitened residual per dimension for all eight cases. Whitened values use the case's changed covariance, so they must not be interpreted as a common-scale accuracy score. No new per-family IMU/WNOJ Jacobian export is claimed for these eight production-binary runs; the four-family analysis remains the previous retained audit.

### How this connects to the four factor families

The optimizer does not average GLIM and Kimera's ATE values. It minimizes the sum of their relative-factor costs, Combined IMU costs, WNOJ costs and initial priors. At a common linearization, with whitened Jacobians/residuals `A_s,r_s`, `H = sum(A_sᵀ A_s)`, `g = sum(A_sᵀ r_s)` and the coupled step solves `H delta = -g`. Scaling one source covariance by `c` scales that family's local curvature and gradient by `1/c`; the response still depends on all connected variables and factors.

The preceding audit found comparable GLIM/Kimera trajectory-shape leverage, substantial rotational opposition, and much stronger ON GLIM/IMU translational opposition. The IMU's large raw position-diagonal curvature is not a percentage of fused trajectory ownership. This follow-up tests the predicted source-weight sensitivity without changing IMU or WNOJ settings. It does not rerun a WNOJ sweep or estimate missing cross-source covariance.

## Reproduction and validation

`run_cases.py` invokes the unchanged executable for every case:

```bash
/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay \
  --event-stream CASE_JOURNAL --out FRESH_CASE_DIRECTORY \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59
```

Environment: existing `ubuntu-jammy-humble-nvidia`, source `/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`. Four independent diagnostic processes at a time; runtime is not a live-throughput benchmark. `score_cases.py` uses unchanged `score_grandtour_prism.py` and `score_grandtour_relative_pose.py` inside existing `cbsms_ws_gpu`, sourcing `/opt/ros/noetic/setup.bash`.

`export_reference.py`, `belief_reference_audit.py`, `check_se3.cpp` and `validate_se3.py` are isolated diagnostic utilities. The reference analysis was restarted with cached NPZ arrays to avoid repeated decompression; this did not stop or change receiver runs. No package was installed.

Validation completed:

- All eight processes returned zero. Original journal and receiver-binary hashes still match the predeclared manifest; all derived journal hashes still match their recorded values.
- 1195 support states each; unique final factors remain 1190 GLIM + 591 Kimera OFF, 1190 GLIM + 593 Kimera ON. Pending queue zero; 50 unsupported messages per case, identical to that mode's baseline. No duplicate/stale revision counts or factor types change.
- Support clocks, unsupported-event lists and source accounting are byte-identical to their baselines. `causal_dispatch.csv` has no changed columns. Revision events differ only in measured compute time and resulting live/endpoint correction magnitudes; all factor IDs, revision identities, slot replacements and ordering remain identical.
- Initialization, gravity, bias initialization, timing selector, Qc, startup count and solver settings compare exactly. Every saved final-state physical timestamp matches baseline.
- Exact scoring timestamps/masks match; all native reference trajectories/evaluations were reused without rewriting them. The prior audit's scale-1 repeated replay was byte-identical. These eight distinct sensitivity cases were not each repeated a second time.
- Six CT-relative-factor tests, four ALL_RIGHT IMU-support tests and four causal-support-clock tests passed unchanged; XML files retained. Isolated diagnostic Python files compile; GTSAM residual probe passed. No new sanitizer/whole-project clean-build claim is made because production code was not changed.
- Runtime OFF 731–814 s per replay; ON 1018–1058 s, with four concurrent jobs and no live pacing. These measurements are not a capacity claim for real-time operation.
- `git diff --check` passes. Production source and executable hashes remain unchanged. No source estimator, source CBS setting, source covariance exporter or production fusion configuration was edited.

Production binary SHA-256: `e5af1ade5f72ed1e05fb4505613840eb20e7b2b76623384605db6386bd390c2c`. Production replay source SHA-256: `0719b562f8def4dfb0096e16a2a05db5ac48e718575adcdb4fc02df2bace5ffe`. Original event journal identities are retained in the manifest. IE-TC bag SHA-256: `563ad3681eb40ed85bb682926ea1bdafb86b038dfffa7e6413cc4e32b8fb75aa`.

gnssFGO HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit; pre-existing dirty research worktree preserved. This report is the only new gnssFGO repository file; diagnostics are in the separate run directory. No estimator/factor implementation or saved production configuration is modified.

## Decision

**Keep the saved production scales at 1.0.** The authorized sensitivity investigation is complete; no wider sweep or new run was added after observing the results.

What is established: source-wise weighting matters; Kimera is influential; CBS-ON changes both local belief accuracy and reported confidence; modest scalar changes can improve ATE but leave the matched OFF–ON gap and introduce metric trade-offs. What is not established: a Schur bug, calibrated absolute covariance, quantitative cross-source covariance, a universal optimal weight, or proof that ignored correlations alone caused the gap.

Before adopting a weighting method, the justified next work is independent/held-out validation and a more defensible treatment of exported posterior-belief dependence and uncertainty. This report proposes no production change and runs no follow-up experiment.
