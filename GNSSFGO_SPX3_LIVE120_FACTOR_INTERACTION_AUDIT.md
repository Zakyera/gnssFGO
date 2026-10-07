# SPX-3 live120: how GLIM, Kimera, Combined IMU and WNOJ interact

Date: 2026-09-29. **Frozen-data diagnostic only; no tuning or new source experiment.**

## Main finding

The evidence does **not** support the simple explanation that GLIM overwhelms an ineffectual Kimera. Both source families have substantial, approximately equal local influence on trajectory shape. CBS-ON makes Kimera's exported covariance much smaller, and the final graph fits Kimera more closely while accepting substantially larger GLIM residuals. Translational disagreement between GLIM and the IMU terms also increases sharply.

This establishes **changed uncertainty and increased constraint conflict**, not a uniquely identified cause of the ATE loss. It does not establish that the Schur complement is wrong or identify an optimal covariance multiplier. Missing cross-belief/source correlations, uncertainty calibration, source means, motion/interpolation assumptions and arrival-dependent factor construction remain distinguishable hypotheses.

Both instrumented replays reproduced the original final and online trajectories, full saved state rows and receiver diagnostics **byte-for-byte**. No settings were changed in the receiver, and neither GLIM nor Kimera was rerun.

## 1. Which experiment is being audited?

This is the most recent matched **120 s headless live SPX-3 capture**, not the earlier 134 s offline experiment. The original sources and receiver ran concurrently. This audit subsequently replays their exact saved event journals offline; it is not another live run.

- [Original OFF capture](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live120_headless_20260929/).
- [Original ON capture](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live120_headless_20260929/).
- [Frozen configuration](/home/yeranis/repos/gnssFGO/GNSSFGO_GRANDTOUR_CLASSIC_LIVE_BASELINE_20260929.md) and [original results](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_LIVE120_HEADLESS_OFF_ON_20260929.md).
- [This audit's scripts, copied diagnostic source, binary, Jacobians and summaries](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/).

Retained results, not recomputed or replaced here:

| Actual trajectory | OFF ATE m | ON ATE m | OFF 1 s t-RPE m | ON 1 s t-RPE m | OFF / ON 1 s r-RPE deg |
|---|---:|---:|---:|---:|---:|
| Original/native GLIM | 0.007046 | 0.015811 | 0.007028 | 0.022587 | 0.275979 / 0.574259 |
| Original/native Kimera ROS odometry | 0.655087 | 0.170926 | 0.176565 | 0.030378 | 0.614367 / 0.535886 |
| gnssFGO fusion, online/as-played | 0.069367 | 0.078541 | 0.027762 | 0.047590 | 1.750184 / 1.911003 |
| gnssFGO fusion, final retrospective | 0.047570 | 0.066802 | 0.011439 | 0.012325 | 0.380927 / 0.651236 |

All comparisons use the retained common evaluation mask, no-scale alignment, prism ATE and IE-TC 1 s RPE conventions. ATE has partial prism coverage; it is not a claim of full-window GT coverage. Native trajectories are not source-only gnssFGO controls.

Kimera's native ATE improves about 3.83-fold but remains about 10.8 times native GLIM's ON ATE. More importantly, improvement of a complete native path is not an additive contribution to the fuser: the receiver consumes **relative smoother beliefs and covariances**, not a weighted average of the two native absolute paths or their ATEs.

## 2. What iSAM2 actually balances

There are four **factor families**, plus four initial priors, not four individual factors. At the final state:

| Family | OFF factors | ON factors | Residual dimension per factor |
|---|---:|---:|---:|
| GLIM relative belief | 1190 | 1190 | 6 |
| Kimera relative belief | 591 | 593 | 6 |
| Combined IMU | 1194 | 1194 | 15 |
| WNOJ | 1194 | 1194 | 18 |
| Initial pose/velocity/bias/angular-rate priors | 4 | 4 | 18 combined |

Both cases have 1195 support states. Every accepted source factor interpolates both endpoints on the same independent 100 ms IMU-driven grid. There is no GLIM-selected master grid in these runs. Stable-ID replacement means a revision replaces a factor rather than increasing the final count.

For a source factor, the implementation uses the complete native 6×6 matrix:

`e = Log(Z⁻¹ (T(t_i)⁻¹ T(t_j)))`, in `[phi,rho]` order;
`E_source = 1/2 eᵀ Sigma⁻¹ e`.

The total objective is the sum of these source costs, Combined IMU costs, WNOJ costs and priors. At a common linearization, with whitened residual `r_s` and whitened Jacobian `A_s`:

`H = sum_s A_sᵀ A_s`, `g = sum_s A_sᵀ r_s`, `H delta = -g`.

This is why covariance trace is not a fusion weight. The inverse **dense** covariance, Jacobian directions, graph connections, residuals and coupling to velocity/bias/angular rate all matter. Rotation and translation traces must also remain separate because their units differ.

The read-only diagnostic exports actual active factors at support indices 300, 600, 900 and at final drain. It freshly evaluates their residuals/Jacobians at the actual iSAM2 estimate. These are nonlinear-factor diagnostics, **not a dump of iSAM2's possibly older cached linearization or Bayes-tree attribution**. Nothing exported is fed back to the solver.

Implementation references: [factor construction](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:604), [replacement](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:731), [actual dynamics construction](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:1452).

## 3. Are IMU and WNOJ identical OFF versus ON?

The **12,000 raw physical IMU rows are exactly identical**. Initialization, gravity, initial bias, all priors, support timestamps, ALL_RIGHT integration, IMU noise parameters and WNOJ Qc variance 10000 are unchanged.

However, the realized model is not entirely independent of the source estimates:

- IMU preintegration uses the currently estimated previous bias as its linearization bias.
- [SupportAcceleration()](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:536) uses measured acceleration, estimated bias and gravity rotated by the current pose. These acceleration inputs are constants supplied to WNOJ factors/interpolators, not optimized acceleration variables in this receiver.
- All factor residuals/Jacobians change with the optimized state even when measurement data and noise settings are unchanged.

Saved OFF/ON angular-acceleration support inputs match exactly. Linear-acceleration input differences have per-axis RMS `(0.04671, 0.05133, 0.01968)` m/s² and maxima `(0.25320, 0.21579, 0.10697)` m/s². Recorded IMU rotation/position covariance-trace differences are tiny: maxima `1.25e-11` rad² and `1.94e-11` m². WNOJ covariance-trace diagnostics match exactly.

Thus “same IMU/WNOJ settings” is true; “the same two inertial/motion terms play an unchanged role in both optimizations” is not. This is observed state-dependent construction, not evidence by itself of an implementation error.

## 4. The exported uncertainties do not collapse to zero

Latest accepted revisions only, retaining all off-diagonal terms in calculations:

| Source | Median rotation covariance trace OFF → ON, rad² | Median translation covariance trace OFF → ON, m² |
|---|---:|---:|
| GLIM | 9.4423e-8 → 4.0610e-8 | 1.7863e-6 → 1.6170e-6 |
| Kimera | 6.1033e-8 → 8.7645e-9 | 1.0841e-5 → 8.3723e-7 |

Ratios of these medians: GLIM translation changes by about 10%; Kimera translation contracts about **12.9-fold** and rotation about **7.0-fold**. These are distribution summaries, not a 12.9× multiplier of Kimera's global influence. Kimera's ON translation covariance median is smaller than GLIM's.

All accepted covariance matrices are finite and positive definite. Minimum full-matrix eigenvalues are OFF GLIM `4.19e-9`, OFF Kimera `1.22e-9`, ON GLIM `1.94e-9`, ON Kimera `9.71e-10` in the stored mixed-unit tangent coordinates. This rules out zero/singular covariance in these payloads; it does **not** prove correct statistical calibration or a universal lower bound.

Exact OFF/ON interval matches exist for 1190 GLIM IDs and 270 accepted Kimera IDs. Kimera timing/identity coverage is not assumed identical just because its factor counts are similar. Matched-interval covariance ratios and first→last revision evolution are retained in [input_audit.json](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/input_audit.json).

## 5. Which constraints are being sacrificed?

At the final solution, define `chi² = eᵀ Sigma⁻¹ e`. The following is total `chi²` divided by the family's residual-coordinate count, **not by statistically independent degrees of freedom**:

| Family | OFF | ON |
|---|---:|---:|
| GLIM | 4.643 | 54.673 |
| Kimera | 16.820 | 8.057 |
| Combined IMU | 1.110 | 4.252 |
| WNOJ | 0.453 | 1.122 |
| Initial priors | 45.151 | 73.330 |

These are **posterior factor residuals, not pre-update NIS**. They do not justify ordinary nominal chi-square gating, and a value near 1 is not used here as a calibration pass/fail test.

Physical source residuals help interpret the change:

| Source | OFF median rho norm | ON median rho norm | OFF median phi norm | ON median phi norm |
|---|---:|---:|---:|---:|
| GLIM | 1.967 mm | 7.590 mm | 0.02315° | 0.03462° |
| Kimera | 5.540 mm | 2.579 mm | 0.03261° | 0.00810° |

Kimera is not being ignored: the ON graph fits its relative beliefs more closely, while GLIM's disagreement grows. The GLIM translational marginal standardized-residual median grows from `6.25` to `110.00`. The rotational/translational marginal statistics are not additive because the full covariance has cross terms.

This is not just a terminal outlier. The cumulative graph checkpoints give:

| Checkpoint after initialization | OFF GLIM / Kimera / IMU / WNOJ | ON GLIM / Kimera / IMU / WNOJ |
|---|---|---|
| 30 s | 6.192 / 31.932 / 1.404 / 0.655 | 75.387 / 9.217 / 5.108 / 1.266 |
| 60 s | 4.158 / 17.533 / 0.979 / 0.442 | 51.431 / 7.129 / 3.773 / 0.944 |
| 90 s | 3.817 / 12.928 / 0.995 / 0.411 | 62.803 / 7.793 / 4.129 / 0.953 |
| Final drain | 4.643 / 16.820 / 1.110 / 0.453 | 54.673 / 8.057 / 4.252 / 1.122 |

These checkpoint rows summarize all active historical factors at those times, not isolated 30 s windows. Graphs at different times also have different information sets.

## 6. Curvature, opposing gradients and actual coupled influence

### Direct pose curvature is not trajectory attribution

Summing the pose blocks of `A_sᵀ A_s`, divided by their total across all families:

| Family | OFF pose-rotation share | ON pose-rotation share | OFF pose-translation share | ON pose-translation share |
|---|---:|---:|---:|---:|
| GLIM | 28.861% | 29.479% | 0.252% | 0.316% |
| Kimera | 70.256% | 70.076% | 0.273% | 0.517% |
| IMU | 0.877% | 0.442% | 98.703% | 98.398% |
| WNOJ | 0.00538% | 0.00271% | 0.00133% | 0.00132% |
| Priors | 0.000034% | 0.000017% | 0.771% | 0.768% |

**Do not read this as “98% of the fused position comes from IMU.”** These are conditional diagonal-block curvatures, before allowing all connected velocities/biases/poses to move. Marginalizing nuisance variables is nonlinear in these block contributions; diagonal shares do not add up to marginal posterior attribution. Kimera already has major rotational curvature in both graphs.

### Which directions conflict?

Cosines between whole-history family gradient subvectors at the final estimate (negative means opposed within that state-coordinate group):

| Pair / coordinate group | OFF | ON |
|---|---:|---:|
| GLIM–Kimera, pose rotation | -0.864 | -0.856 |
| GLIM–Kimera, pose translation | -0.057 | -0.218 |
| GLIM–IMU, pose translation | -0.175 | **-0.829** |
| GLIM–IMU, velocity | -0.084 | -0.543 |

This is concrete evidence of stronger ON GLIM/inertial conflict. It does not tell us which term is correct relative to ground truth.

### Solve through the complete coupled graph

For a more meaningful influence diagnostic, solve the freshly linearized graph with **all** pose, velocity, bias and angular-rate variables free. Let `delta*` minimize that fixed linear problem. For a hypothetical family precision multiplier `alpha_s`, compute analytically:

`d(delta*) / d log(alpha_s) = -H⁻¹ A_sᵀ (r_s + A_s delta*)`.

This is a sensitivity of the **fixed linearized problem**, not a changed nonlinear receiver run. We do not apply `delta*` or any new weight. Convert position increments to world coordinates and remove their best infinitesimal common rigid translation/rotation; the remaining RMS measures trajectory-shape sensitivity rather than a global gauge movement.

| Hypothetical +1% family precision | OFF predicted shape RMS change | ON predicted shape RMS change |
|---|---:|---:|
| GLIM | 0.371 mm | 0.314 mm |
| Kimera | 0.363 mm | 0.315 mm |
| Combined IMU | 0.0192 mm | 0.0127 mm |
| WNOJ | 0.0135 mm | 0.00787 mm |
| Priors | 0.00334 mm | 0.00369 mm |

GLIM and Kimera have comparable local shape leverage, despite very different raw position-curvature percentages. WNOJ-Qc tuning is not the first lever suggested by this *local* evidence. These norms do not indicate whether a change improves ATE, do not apply to large scale changes, and do not test separate rotation/translation scaling.

The unapplied, jointly relinearized Gauss–Newton step has final shape RMS `0.209 mm OFF`, `0.196 mm ON`. This supplies no evidence of a large immediately remaining linearized shape correction accounting for the 19 mm final ATE difference. It is not proof of global nonlinear convergence or correctness of every analytic Jacobian.

## 7. Is the Schur complement the problem?

The shared [relative covariance helper](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/utils/relative_pose_covariance.h:257) transforms joint endpoint information into anchor/relative coordinates, then forms:

`Lambda_relative = C - Bᵀ A⁻¹ B`, `Sigma_relative = Lambda_relative⁻¹`.

It explicitly retains the two endpoint states' cross-information in this extraction. The receiver does not replace this with `1/trace(Sigma)`: it uses `Gaussian::Covariance(full_matrix)` unchanged. Maximum inserted-factor covariance differences from input are `1.73e-18 OFF`, `4.77e-18 ON`.

There is, however, an important distinction between **within-belief** covariance and **between-belief** dependence:

- [GLIM's later diagnostic revisions](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/src/glim_ros/cbs_bridge.cpp:1503) take means and joint marginal information from the current full smoother. In CBS-ON, that smoother contains received Kimera constraints.
- [Kimera's normal export and revision path](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:3460) selects `LOCAL`, then reuses `getOdometryBeliefs`. [BPSAM](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:2555) uses current estimated endpoint means; its [LOCAL graph filter](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:4770) removes tracked explicit CBS/anchor factors from the covariance query. That is not a certificate that inherited CBS information has been removed from the fixed-lag marginal prior or from the mean.
- Both source estimators and the central IMU graph consume the same CPT7 measurements. Overlapping relative beliefs also share source states and underlying measurements.
- The receiver uses separate factor noise models; it does not receive cross-source, cross-edge or source–central-IMU covariance. Stable-ID replacement prevents duplicate versions of the same belief, but does not remove correlations between different beliefs.

These are identifiable structural dependencies/omissions. **Their magnitude and causal contribution to this ATE gap have not been measured.** They are reasons not to interpret smaller source covariance as proportionally more independent information, not a finding that the Schur algebra is defective.

The previously documented tangent/frame audit supports `[phi,rho]`/right-local/body-frame compatibility. This task inspected the same paths and reran existing CT-factor tests; it did not claim a new complete source covariance-calibration or analytic-Jacobian audit.

## 8. What can be improved next—without choosing a weight from ATE alone?

The justified next target is **source uncertainty and compatibility**, not an arbitrary Qc change or a rule to trust Kimera more because its native ATE improved.

1. Check first/latest relative-belief error against a suitable relative reference on the exact physical intervals, separately for phi/rho and directional covariance eigenvectors. Account for reference uncertainty and shared IMU; final posterior residuals alone are not a calibration test.
2. Examine cross-edge/source error dependence and the differing GLIM full-smoother versus Kimera LOCAL covariance provenance. Establish what information is reused before treating every exported posterior marginal as an independent measurement.
3. If subsequently authorized, do a small **frozen-input, one-parameter-at-a-time** nonlinear sensitivity study, with equal evaluation masks and online/final reporting. Compare actual movement with the local predictions here. Keep mean payloads, revision arrival order, topology and all other settings fixed. A held-out sequence is needed before selecting a production setting.
4. Retain IMU/WNOJ in the investigation: ON GLIM–IMU translational gradients strongly oppose each other, and WNOJ acceleration inputs depend on the preceding estimate. Do not infer from raw IMU position curvature that it should simply be downweighted.

No multipliers, Qc, source extraction policy, correlations, gating, robust loss, source CBS protocol or production settings were changed in this task. No proposed setting is certified as an improvement.

## 9. Reproduction, numerical checks and non-regression

Artifacts: [gnssfgo_spx3_live120_factor_audit_20260929](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/).

`build_diagnostic.py` generates an instrumented **copy** of `GrandTourCsvReplay.cpp` using the existing CMake flags/link command. It adds only const graph access and export calls at three checkpoints/final drain. It never edits production source or overwrites the production binary. `export_snippet.txt` records the exact additions; `build_manifest.json` records compile/link commands and hashes.

For each `MODE=off,on`, the replay command was:

```bash
/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/grandtour_ct_replay_factor_audit \
  --event-stream /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_${MODE}_live120_headless_20260929/receiver/events.csv \
  --out /home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/${MODE} \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 --receiver-startup-samples 59
```

Executed in the existing `ubuntu-jammy-humble-nvidia` container after sourcing `/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`. There was no ROS bag playback. Replays took about 569 s OFF / 685 s ON, run concurrently without live pacing; these are diagnostic runtimes, not a new live throughput benchmark.

Validation:

- [replay_identity.json](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/replay_identity.json): all **11 files per mode** byte-identical, including final/online trajectories and full saved nominal state histories, support clock/accelerations, model covariance diagnostics, residuals, source accounting, dispatch and unsupported events. This is not a full posterior-covariance equality claim; the receiver does not save that matrix.
- Final TUM SHA-256 OFF `5e06cf4c4e4af63de40c95b1cdbeaa905ad41954448359fdbaf73a8fa7cdb20e`; ON `64609f58be978689b0e10fb22349593fa683701b881bdfd096043272d784bc00`.
- Source residual/covariance recomputation agrees with saved Mahalanobis values within `1.37e-12`.
- All eight checkpoint matrices finite; exported whitened residual norms reproduce per-factor costs. Sparse Jacobian assembly sums any repeated column entries.
- Diagonally scaled normal-equation solve relative residual at most `4.53e-14`; family sensitivity solve residual at most `1.72e-12`. Independent central finite differences of the fixed linear problem agree with analytic sensitivities within `1.95e-7` relative error. These algebra-only precision perturbations were not fed to the receiver.
- Uniform scaling of every family's precision leaves the fixed linear optimum invariant: sum of family derivatives has maximum absolute coordinate `1.08e-11`.
- Existing tests run directly: **6 CT-relative-factor + 4 ALL_RIGHT IMU-support + 4 causal-support-clock tests passed**. XML retained. The CTest wrapper could not write its old log directory; direct test execution avoided changing directory permissions. No new sanitizer or whole-project test claim is made.
- Production source SHA remains `0719b562f8def4dfb0096e16a2a05db5ac48e718575adcdb4fc02df2bace5ffe`; production binary remains `e5af1ade5f72ed1e05fb4505613840eb20e7b2b76623384605db6386bd390c2c`.
- gnssFGO HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was made. Pre-existing dirty research worktrees were preserved; `git diff --check` passed. This report is the only new repository file from this task; diagnostic artifacts live outside the repository.

**Conclusion: measurable CBS-ON source/inertial conflict and changed source confidence; no evidence that Kimera is ineffective, no demonstrated Schur-complement bug, and no validated tuning choice yet.**
