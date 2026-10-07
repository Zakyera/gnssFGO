# SPX-3 frozen-factor tangent and information audit

Date: 2026-09-28. Scope: the retained SPX-3 full-history gnssFGO receiver, chiefly **D: classic CBS-ON, revised GLIM + revised Kimera**. B (matched CBS-OFF joint graph) is used only as a frozen residual/covariance comparison. This is not a new source run, fusion configuration, weighting rule, or performance evaluation. The receiver is an **offline causal replay**, not a live co-running gnssFGO process. Its `trajectory.tum` is the final retrospectively revised graph, distinct from the as-played `online_trajectory.tum`.
The previously scored final joint products were B ATE `0.054274 m`, 1 s t-RPE `0.011785 m`, versus D ATE `0.067920 m`, 1 s t-RPE `0.012847 m`; this audit did not rescore them. These receiver diagnostics are not the native GLIM or native Kimera trajectories.

## Result in brief

No end-to-end `[phi,rho]` tangent-order, right-local residual, 6×6 row-order, or body-frame mismatch was found for this SPX-3 run. Both exporters and the receiver use the same GTSAM `Pose3` relative-error convention. Kimera nevertheless has substantial *local graph curvature*: in final D it supplies about 1.57 times GLIM's summed pose-rotation diagonal Hessian trace and 1.05 times GLIM's summed pose-translation trace, despite half as many factors. Its interpolated factors also touch support velocity and angular-rate states, which exact-endpoint GLIM factors do not. These numbers are **not** a percentage of trajectory correction or proof of a weighting defect.

Both sources have large final whitened residuals relative to their exported covariances, especially CBS-ON GLIM's translational tangent channel. The OFF→ON covariance contraction and the receiver's omission of cross-source correlation remain important hypotheses to examine, **not established causes** of the CBS-ON fusion ATE difference. No covariance or model was changed.

## Frozen case and integrity gate

The source run uses the SPX-3 mission `2024-11-02-17-43-10`, bag offset 32 s, duration 134 s, Hesai/CPT7 GLIM and front Alphasense stereo/CPT7 Kimera. It uses the [classic CBS protocol](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md), not experimental source-to-source `live_revision`. Downstream stable-ID revision replacement is receiver-only. The retained [full-system report](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_REVISED_BELIEFS_CBS_OFF_ON_FULL_SYSTEM.md) defines the unchanged receiver: Combined IMU, WNOJ `Qc=10000`, ALL_RIGHT CPT7 support, full-history iSAM2, source covariance scale 1, no GNSS, no gate, no robust loss, no receiver health weighting.

The [frozen ON joint revision CSV](/home/yeranis/results/gnssfgo_spx3_full_system_revisions_20260927/on/joint_revisions.csv) SHA-256 is `54f3ced9f752921aec0470ec1977a7e2c9c6c679d5913d2761450ea5b06c81a1`. The four source CSV hashes still match `source_match.json`. It contains 79,618 version rows for 1,330 GLIM and 663 Kimera IDs. Four Kimera IDs lack the fixed GLIM support grid; the active D graph has 1,330 GLIM and 659 Kimera factors, exactly as before. All GLIM factors are exact-endpoint; all supported Kimera factors interpolate two endpoints.

I compiled a **separate diagnostic binary** with a temporary `CBS_FACTOR_AUDIT` block at the final estimate, then removed that block from the repository source. The exact [instrumentation snippet](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/factor_jacobian_export_snippet.txt) is retained with the artifacts; the original receiver executable remains SHA-256 `b283fb0d9be9b471e85788a94acc9b308acc2ee53e51219846c3f1f7d80a174e`. The diagnostic replays the same frozen inputs and only exports the active factors' final unwhitened Jacobians. The following outputs are byte-identical to frozen D:

| Product | Verified SHA-256 |
|---|---|
| Final `trajectory.tum` | `e96b3f0b20919fa2b5a21697c376d84b49eb2471410f19689ce40cd043977540` |
| As-played `online_trajectory.tum` | `241cf19e18e7fb49d80aab88d44fa8a5c3b1f6a73babc97895af771c52660785` |
| Final `states.csv` | `42c98713906d13426f7e7c8689d81fcbef857614526b2b7e1d66927c4cc778e7` |
| As-played `online_states.csv` | `e40b82235eccfd30c7116a632d6fb5b4c6c3bfa09b52adc5fe35cffe8491cb16` |
| Final `belief_residuals.csv` | `c149a34b8eff14aef437e3b7afcb611d0f5ba14b786fc3cd934a90ee1adedddc` |

The original [D `summary.json`](/home/yeranis/results/gnssfgo_spx3_full_system_revisions_20260927/D/summary.json) reports maximum input-to-inserted-factor covariance difference `1.7e-17`, i.e. logging/roundoff. The audit's 1,989 active covariances have maximum symmetry error zero and minimum eigenvalue `9.98e-10`. Recomputed full-covariance Mahalanobis-square values agree with the receiver residual CSV to `2.73e-12` maximum absolute difference. No source or frozen result file was overwritten.

## Tangent/frame convention, end to end

For each source, `Z = T_i^{-1}T_j`; the exported six-vector is `xi = Pose3::Logmap(Z) = [phi,rho]`, with radians then tangent metres. **`rho` is the translational part of the SE(3) logarithm, not simply the Cartesian translation vector of `Z`.** The 6×6 covariance describes the *right-local measurement error* about `Z`, not additive error in the six CSV mean numbers:

`e(X_i,X_j) = Local(Z, X_i^{-1}X_j) = Log(Z^{-1}(X_i^{-1}X_j))`, with `Z' = Z Exp(delta)`.

- GLIM constructs `relative_mu` using `Pose3::Logmap(odom.measured_from_to)` and writes all covariance entries row-major in [the bridge](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/src/glim_ros/cbs_bridge.cpp:3288). Kimera does the same in [VioBackend](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:3542). Both covariance arrays retain off-diagonal rotation/translation terms.
- The shared [Schur-relative covariance helper](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/utils/relative_pose_covariance.h:293) forms relative Jacobians with a GTSAM `BetweenFactor<Pose3>` at the measured transform, using the full joint endpoint information. It therefore targets the same local residual as the receiver, including endpoint cross-covariance. Kimera's [BPSAM extraction](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:2555) calls that helper; the configured outgoing mode is `schur_relative_between`.
- The saved ON parameters put GLIM's child frame and Kimera's base/external pose frame at `cpt7_imu`: [GLIM parameters](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/spx3_full_system_revision_diagnostics_20260927/spx3_on_classic_revisions_nodelog/resolved_glim_params.yaml:9), [Kimera parameters](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/spx3_full_system_revision_diagnostics_20260927/spx3_on_classic_revisions_nodelog/resolved_kimera_params.yaml:1). The Kimera ROS wrapper's [external-frame adjoint branch](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO-ROS/src/KimeraVioRos.cpp:2358) is therefore inactive here. Different global map gauges cancel in relative transforms.
- The receiver [loads](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:299) `Pose3::Expmap(relative_mu_0..5)` and `covariance_r_c` without reordering. Its [factor construction](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp:466) passes the full matrix unchanged to `Gaussian::Covariance`. Both its [exact](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/odometry/BetweenFactor.h:99) and [GP-interpolated](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/odometry/GPInterpolatedDoublePose3BetweenFactor.h:162) factors use `Local(measured,predicted)`.

Independent numerical checks: a nontrivial right-local GTSAM pose test gave Log/Exp maximum error `4.44e-16`, right-local retract error `2.89e-16`, and **exactly zero** source-versus-receiver `BetweenFactor` residual and both endpoint-Jacobian differences. All six existing `CtRelativeFactor.*` tests passed, including dense covariance preservation and finite-difference Jacobians for common, single, and distinct WNOJ-interpolated support. Final active mean rotation norms are at most `0.290 rad`, far from a Log branch cut.

## Whitened final residuals

For each *latest active revision* in the final graph, the audit uses the entire dense covariance `Sigma = LL^T` and `w=L^{-1}e`; the table's `chi²` is `||w||²=e^T Sigma^{-1}e`. The rotational and translational norm columns are the unwhitened `[phi,rho]` residual blocks. These are **final posterior factor residuals, not pre-update innovation NIS**; ordinary six-degree-of-freedom chi-square calibration rules do not apply directly after joint optimization, revisions, and correlated source exchange.
The per-factor artifact retains all six unwhitened and Cholesky-whitened coordinates; because the 6×6 covariance is dense, individual whitened coordinates should not be read as independent physical rotation/translation channels.

| Frozen final graph | Source | Factors | Median `||e_phi||` rad | Median `||e_rho||` m | Median `chi²` | Mean `chi²` | P95 `chi²` |
|---|---|---:|---:|---:|---:|---:|---:|
| B, CBS-OFF | GLIM | 1330 | 2.174e-4 | 0.001488 | 4.69 | 20.09 | 28.71 |
| B, CBS-OFF | Kimera | 659 | 3.928e-4 | 0.005705 | 14.95 | 69.56 | 336.52 |
| D, classic CBS-ON | GLIM | 1330 | 3.715e-4 | 0.007126 | 102.77 | 293.76 | 1269.21 |
| D, classic CBS-ON | Kimera | 659 | 9.121e-5 | 0.002591 | 24.97 | 40.36 | 136.51 |

In D, marginal-block standardized residual medians are GLIM rotation `9.89`, GLIM rho `100.19`, Kimera rotation `2.65`, Kimera rho `23.14`. These use each 3×3 *marginal* covariance block and are **not additive** because the original 6×6 matrices have nonzero rotation–translation cross terms. The mismatch is not a single outlier: across 10 s bins, median full `chi²` ranges from `9.7` to `495` for GLIM and `4.2` to `44.8` for Kimera.

## Exported covariance and local information

The source covariance contraction from B to D is measurable, not a receiver weight change:

| Source | Median `tr(Sigma_phi)` OFF → ON (rad²) | Median `tr(Sigma_rho)` OFF → ON (m²) | Median dense cross-block Frobenius norm in D |
|---|---:|---:|---:|
| GLIM | `9.645e-8 → 4.091e-8` | `1.743e-6 → 1.590e-6` | `3.506e-8` |
| Kimera | `6.447e-8 → 8.761e-9` | `1.158e-5 → 8.553e-7` | `1.155e-8` |

At the **same final D state**, for each active source factor the diagnostic evaluates the actual unwhitened graph Jacobian `J = de/dx`, including the WNOJ endpoint-interpolation derivatives, then forms `H_source = J^T Sigma^{-1}J`. The table sums diagonal blocks in the same GTSAM state tangent; pose rotation and pose translation are deliberately separate because their units differ. These are Gauss–Newton *local curvature* contributions. Cross-state and rotation–translation blocks remain in the full calculation and are reconstructible from the [raw Jacobian export](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/D_diagnostic_fixed/factor_jacobians.csv); the displayed diagonal traces do not replace the complete Hessian.

| D source | Factors | Median per-factor pose-rotation trace (rad⁻²) | Sum pose-rotation trace (rad⁻²) | Median per-factor pose-translation trace (m⁻²) | Sum pose-translation trace (m⁻²) |
|---|---:|---:|---:|---:|---:|
| GLIM, exact endpoints | 1330 | `6.829e8` | `9.530e11` | `1.399e7` | `2.123e10` |
| Kimera, both endpoints interpolated | 659 | `2.180e9` | `1.496e12` | `2.444e7` | `2.238e10` |
| Kimera / GLIM | — | `3.19×` | `1.57×` | `1.75×` | `1.05×` |

Kimera's interpolation additionally yields median per-factor diagonal traces `5.46e3` on support velocity and `4.90e5` on support angular rate; GLIM exact-endpoint relative factors have zero direct derivatives for those states. This is a structural difference in *where* the same 6D residual enters the graph. On 1,326 support poses touched by both sources, the per-pose Kimera/GLIM curvature ratios are widely dispersed: median rotation `0.603`, P95 `8.47`; median translation `0.153`, P95 `2.88`. Kimera exceeds GLIM on about 50.0% of shared rotation blocks and 48.4% of translation blocks. It is not a uniform global multiplier. No repeated support-key slots occurred in the active factors; the analysis script still combines any repeated slots before computing a Hessian.

The local source cost gradients point in opposite directions on 55.4% of shared pose rotation blocks and 70.7% of shared pose translation blocks (median cosine `-0.117` and `-0.496`, respectively). This is evidence of *local graph tension*, not a causal decomposition of the final trajectory change: the IMU and WNOJ gradients also enter the optimum, and gradients are evaluated only at the final D solution.

## What this audit does and does not establish

1. **Tangent/frame mismatch:** not supported by code tracing, the synthetic source–receiver test, active-factor covariance equality, or the frozen replay hash gate.
2. **Kimera has real receiver leverage:** supported. Its ON source covariance is small, its 659 factors contribute substantial pose curvature, and interpolation couples them to velocity/angular-rate states. This answers why comparing only factor counts or diagonal covariance entries is misleading.
3. **Statistical calibration / root cause of ATE loss:** not established. Final whitened residuals are large, especially for GLIM ON, but they alone cannot distinguish source covariance calibration, source-mean differences, continuous-time interpolation error, competing IMU/WNOJ constraints, or hidden GLIM–Kimera correlation. Classic CBS feedback means the two source streams are not physically independent; Kimera's `LOCAL` extraction excludes tracked explicit CBS factors but does not certify removal of old CBS information from its fixed-lag prior. The receiver presently puts native covariance in the factor and represents the interpolated endpoint through state variables/WNOJ; it does not add a separate conditional GP interpolation covariance to the measurement noise. This is a documented modeling assumption, not a newly proven error.
4. **No tuning decision follows:** `tr(J^T Sigma^{-1}J)` is local curvature, not a posterior attribution percentage. Neither these traces nor one sequence's ATE justifies scaling Kimera or GLIM covariance.

## Reproduction and non-regression

The [artifact directory](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/) holds the diagnostic-only build script and binary, synthetic tangent probe, exact active-factor Jacobians, [per-factor metrics](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/active_factor_diagnostics.csv), [per-support-pose curvature](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/support_pose_information.csv), [10 s summaries](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/ten_second_factor_summary.csv), [OFF/ON residual summary](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/off_on_residual_summary.json), and [machine-readable D summary](/home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/factor_summary.json). The first diagnostic attempt failed at its final Jacobian export because its output vector was not pre-sized for GTSAM; no frozen files were affected. The corrected attempt completed and passed every byte-identity gate above.

The corrected diagnostic run used the same command-line inputs as D, changing only the binary and output directory:

```text
grandtour_ct_replay_factor_audit \
  --imu /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_ekf_work/spx3_all_unique_kimera_20260920/off/input/imu/imu.csv \
  --glim-beliefs /home/yeranis/results/gnssfgo_spx3_full_system_revisions_20260927/on/glim/first.csv \
  --kimera-beliefs /home/yeranis/results/gnssfgo_spx3_full_system_revisions_20260927/on/kimera/first.csv \
  --revision-beliefs /home/yeranis/results/gnssfgo_spx3_full_system_revisions_20260927/on/joint_revisions.csv \
  --mode joint --timeline record --solver isam2 \
  --out /home/yeranis/results/gnssfgo_spx3_factor_information_audit_20260928/D_diagnostic_fixed
```

The diagnostic build uses the frozen CMake flags and a separate output binary. Its temporary guarded block only called `unwhitenedError` after the final estimate; it has been removed from the repository source, and the production path and original binary are untouched. The six existing CT relative-factor tests passed. Python analysis syntax checks passed; `git diff --check` passed. The gnssFGO worktree was already dirty before this audit; no pre-existing edits were removed or committed, no source estimator was rerun, and no CBS or gnssFGO mathematics, source covariance, WNOJ, IMU, or weighting was changed.
