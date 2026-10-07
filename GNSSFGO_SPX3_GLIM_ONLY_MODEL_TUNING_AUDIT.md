# SPX-3 CBS-OFF GLIM-only gnssFGO model audit

Date: 2026-09-25. Scope: the **saved CBS-OFF GLIM beliefs only**. No Kimera factor, CBS-ON run, GNSS factor, source-estimator change, covariance scaling, gating, or final GT-selected tuning was used. Input is the frozen [1,330-edge GLIM CSV](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/input/glim_beliefs.csv), SHA-256 `73daba74eb7728d7b7e05a1b38ef0c92c88680f7dd58b658b89927613e39fb47`, and the frozen CPT7 IMU CSV, SHA-256 `c041516c61056977f5b249227e6ab981cd1603e8dcefd19b53523e7f7ff3a7d9`. Source run: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/classic_cbs_restoration_20260925/spx3_off_native/`. All scored graph/chain products have the same 1,332 physical support timestamps, 1,093 matched AP20 prism poses over 108.795 s, and 1,321 IE-TC 1 s RPE pairs. The frozen original graph is [here](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/glim/).

## Executive finding

The 0.007→0.142 m preservation gap is **already present before gnssFGO dynamics**. Direct composition of the exported GLIM factors, and an independently optimized pose-only graph using precisely those factors, both score **0.158857 m ATE**. The same native GLIM poses put on this support grid score **0.007549 m ATE** (the original native CSV scores `0.007190 m`). The current Combined IMU reduces the belief-only ATE from about `0.159` to `0.142 m`; WNOJ does not create the gap. This is not evidence that the exported beliefs are bad relative measurements in isolation. It proves that this *first-publication, overlapping i→i+2 relative-belief representation does not preserve the published native GLIM trajectory when composed into a new graph*.

## Source preservation and the two-chain rank issue

The export contains exactly the edges `0→2, 1→3, …, 1329→1331`. They form **two disconnected trees** (even and odd pose indices), not one consecutive chain. With 1,332 pose nodes and 1,330 independent edges:

- no prior: two independent SE(3) gauges;
- one initial-pose prior: one remaining six-dimensional gauge, so the requested one-prior-only pose graph has no unique trajectory or meaningful ATE;
- two initial-pose priors, one per component: minimum gauge fixing. Only the native poses at indices 0 and 1 were used as anchors; no later native pose was inserted as a factor.

The read-only [composition script](/home/yeranis/repos/gnssFGO/tools/spx3_glim_chain_audit.py) and independent [GTSAM pose-only optimizer](/home/yeranis/repos/gnssFGO/tools/spx3_glim_pose_graph_audit.cpp) agree to maximum `3.62e-14 m` position and minimum absolute quaternion dot `0.9999999999999998`. The pose-only graph used all native full 6×6 covariances, started from native support poses, and reduced weighted cost `14481.7→7.94e-22`. Its exact fit to the exported factors **still** gives `0.158857 m` ATE. Direct composition and pose-only optimization therefore are not two different performance outcomes on this forest.

Against the native pose pair on each *identical physical interval*, the exported GLIM measurement differs by median/RMS `1.42/1.91 mm` translation and `0.0344/0.0456 deg` rotation. These small per-edge differences accumulate: the composed pose differs from native by `0.353 m` RMS position in the initial two-anchor gauge. The interleaved pose-only CPT7 path is `110.71 m` versus native `84.61 m`, although each separate parity chain has a path near `84.2 m`; the two independently accumulated tracks no longer interleave cleanly. The prism-frame full path is `114.23 m` versus native `88.54 m`. This supports a source-belief-to-native-output inconsistency, not a missing GNSS/Kimera explanation. It does **not** identify which source-side revision/publication event created each difference.

## Controlled graph-component results

All rows below use the same saved GLIM measurements, physical timestamps, record-arrival order, source covariance, full-history iSAM2 solver and evaluation windows. “WNOJ only” removes the **CombinedImuFactor**, but retains the existing runner's IMU-derived support acceleration and IMU-derived initial guesses; it is not a raw-IMU-free model. The current graph has both factor families. Absent-state variables/priors were removed rather than retained unconstrained. Case E from the opt-in diagnostic build reproduced the [frozen original trajectory](/home/yeranis/results/gnssfgo_spx3_classic_candidate_20260925/off/glim/trajectory.tum) **byte for byte**, SHA-256 `cfbf6e330688494f0acf37ce810966d0a162c9015c61872c880e2da626a98233`.

| Case | Relative GLIM | Combined IMU | WNOJ | ATE m | t-RPE m | r-RPE deg | Full prism path m | Median final GLIM residual, trans/rot |
|---|---|---|---|---:|---:|---:|---:|---:|
| Native GLIM, same support grid | native trajectory | — | — | 0.007549 | — | — | 88.63 | — |
| A/B: two-gauge-anchored pose-only graph | 1,330 | no | no | 0.158857 | 0.009149 | 0.341174 | 114.23 | ≈0 / ≈0 |
| D: WNOJ only | 1,330 | no | yes | 0.156705 | 0.008948 | 0.340404 | 88.70 | 0.221 mm / `5.23e-5` rad |
| C: Combined IMU only | 1,330 | yes | no | 0.142048 | 0.012022 | 0.311696 | 88.30 | 1.911 mm / `2.74e-4` rad |
| E: current graph | 1,330 | yes | yes | 0.142027 | 0.012023 | 0.311471 | 88.30 | 1.903 mm / `2.81e-4` rad |

The first large ATE gap is at **source-belief reconstruction**, not the addition of WNOJ or Combined IMU. The Combined IMU pulls the optimized trajectory farther from exact GLIM-factor agreement (median translation residual `0.221→1.903 mm` comparing D→E), but improves ATE `0.1567→0.1420 m` and rotational RPE `0.3404→0.3115 deg`; translational RPE worsens `0.00895→0.01202 m`. Thus “helping” depends on the metric. Removing WNOJ from E changes ATE by `0.000021 m`; after global SE(3) alignment, C versus E position shape RMS is `0.000277 m`. The graph's current shape is governed mainly by the Combined IMU plus GLIM factors, **not** WNOJ.

## WNOJ model and bounded Qc sensitivity

The [runner](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp) uses support variables `X` (Pose3), `V` (world velocity), `B` (accelerometer/gyro bias) and `W` (body angular rate). The [GPWNOJPrior](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/motion/GPWNOJPrior.h) joins `X,V,W` across adjacent support times and receives IMU-derived angular/linear acceleration at the two endpoints as fixed inputs, not optimized acceleration states. The [Qc construction](/home/yeranis/repos/gnssFGO/online_fgo/include/utils/GPUtils.h) uses a diagonal six-axis white-noise-on-jerk spectral density `Qc = 10000 I`. In the local angle/translation coordinates its physical units are approximately `rad²/s⁵` and `m²/s⁵`; larger Qc means a **weaker** motion prior. It was not established or tuned specifically for GrandTour.

Support spacing is `0.098436–0.101319 s`, median `0.100004 s`. At median spacing, the WNOJ *pose block alone* has one-axis standard deviation `sqrt(Qc Δt⁵/20)=0.07072` rad or m. The actual GTSAM combined-IMU preintegration covariance at first/middle/last adjacent supports has one-axis rotation standard deviations `0.000999/0.001001/0.000996 rad` and position standard deviations `0.0000689/0.0000687/0.0000676 m`; the WNOJ pose-block scales are `0.0702/0.0711/0.0694`. Median native GLIM factor block RMS sigmas are about `0.000209 rad` and `0.001016 m` over its roughly 0.2 s edge. These are **local factor block scales, not global Kalman/Schur weights**: the IMU factor also constrains velocity and biases and all full matrices have coupling. The exact per-interval values are in [model_covariance.csv](/home/yeranis/results/gnssfgo_spx3_glim_model_audit_20260925/E_modelcov/model_covariance.csv).

Only Qc was varied in this logarithmic sensitivity check; other model settings were frozen. Every run accepted 1,330 GLIM factors, 0 Kimera factors.

| Qc multiplier | Diagonal Qc | ATE m | t-RPE m | r-RPE deg | Median GLIM trans residual mm |
|---:|---:|---:|---:|---:|---:|
| 0.01× | 100 | 0.142029 | 0.012050 | 0.317307 | 1.910 |
| 0.1× | 1,000 | 0.142094 | 0.012037 | 0.311379 | 1.923 |
| 1× | 10,000 | 0.142027 | 0.012023 | 0.311471 | 1.903 |
| 10× | 100,000 | 0.142065 | 0.012032 | 0.311150 | 1.903 |
| 100× | 1,000,000 | 0.142063 | 0.012038 | 0.311381 | 1.917 |

The current WNOJ is **not overconstraining the output relative to Combined IMU**; even two orders of magnitude stronger/weaker Qc leave ATE near `0.142 m`. This sweep does not validate `10000` as an optimal physical Qc or justify selecting a GT-best value.

## Initialization and timing audit

Actual default initialization: `X₀=I`, `V₀=0`; the 59 IMU samples strictly before the first GLIM support span `0.580 s` and end `4.636 ms` before it. Gravity is `-9.81` times the normalized mean acceleration (`[-0.399013,-0.047078,9.801769] m/s²`); initial accelerometer bias is mean acceleration plus gravity (`[-0.006345,-0.000749,0.155858] m/s²`); initial gyro bias is the mean gyro (`[0.000199,0.004447,0.009181] rad/s`). Initial `W₀` is the bias-corrected raw gyro at the first support. Pose prior sigmas are `[0.174533,0.174533,0.00174533,1e-5,1e-5,1e-5]` in `[phi,rho]`; velocity `0.001` per axis; bias `[0.1,0.1,0.1,0.01,0.01,0.01]` in GTSAM accelerometer-then-gyro order; angular rate `0.1` per axis. These prior widths were **audited but not independently swept**.

Using only the latest 20 or 40 pre-start IMU samples changes the estimated gravity and biases substantially, but the aligned trajectory shape changes less than `0.5 mm` RMS relative to 59 samples; ATE remains `0.142047` and `0.142145 m`, respectively, with t-RPE `0.012018` and `0.012031 m`. In the *unregistered* common initial gauge, position RMS separations are `0.144` and `0.090 m`: initial orientation/global-frame changes are real and rigid alignment masks them. The evidence supports insensitivity of the **scored trajectory shape** to this particular sample-window change, not universal initialization robustness.

The Combined IMU preintegrator uses frozen CPT7 **ALL_RIGHT**: sample stamped `t_k` owns `[t_(k-1),t_k]`, including exact fractional boundary overlaps. This gnssFGO runner has no existing selectable alternative physical support convention, so none was invented or tested. Separately, `CorrectedAngularRate()` and `SupportAcceleration()` use `LastImuAtOrBefore()` for fixed WNOJ endpoint kinematics; at fractional support times that lookup is not the same as choosing the right-support sample. The small WNOJ ablation effect means this cannot explain the present 0.007→0.142 m gap, but it remains an implementation detail to audit before claiming a fully uniform timing convention.

## Answers and freeze decision

1. **Do beliefs alone reproduce native GLIM?** No. Two-anchor pose-only graph ATE `0.158857 m`, versus same-grid native `0.007549 m`. Its factors fit almost exactly; the exported first-publication relative set is not a source-preserving representation of the native output.
2. **First component creating the large gap?** The saved belief representation/composition **before** IMU or WNOJ. Two disconnected `i→i+2` chains require two gauge anchors. Combined IMU partly reduces, not creates, the ATE gap.
3. **WNOJ over/underconstraining?** At `Qc=10000 I` it is weak relative to the Combined IMU in this graph; C and E are nearly identical and the 0.01–100× Qc sweep barely changes ATE. No optimality claim.
4. **Combined IMU helping or hurting?** Mixed: it improves ATE and r-RPE over WNOJ-only but increases t-RPE and final GLIM factor residuals. It is the principal dynamics influence on E.
5. **Initialization/timing important?** The 20/40/59 sample sensitivity changes global gauge but not aligned shape much. Prior widths were inspected, not swept. ALL_RIGHT remains the reference; no alternative runner implementation exists to compare. The WNOJ kinematics' preceding-sample lookup is a caveat.
6. **What to freeze before Kimera?** Keep the existing E configuration only as a **reproducible diagnostic reference** (`trajectory.tum` byte-identical to frozen E). There is **no validated healthy SPX-3 source-preserving gnssFGO baseline yet**, so no newly GT-tuned Qc or Combined-IMU setting should be promoted for Kimera. First establish a GLIM belief publication/revision representation that preserves the desired native trajectory, under a stated causal policy. This audit did not implement one.

## Reproduction, validation, and boundaries

Artifacts: [GLIM-only model audit](/home/yeranis/results/gnssfgo_spx3_glim_model_audit_20260925/). The isolated scripts above implement pose composition and pose-only optimization. Opt-in `--audit-use-imu`, `--audit-use-wnoj`, `--audit-qc-scale`, and `--audit-startup-samples` switches were added only to the offline CSV replay; the default math/path remains unchanged. The saved original executable SHA-256 is `b283fb0d9be9b471e85788a94acc9b308acc2ee53e51219846c3f1f7d80a174e`; it was restored to the working build directory after the audit. The separate audit executable SHA-256 is `723e8d323b49810ed33db3d6efed105485b9e58cd393b59fa3f6f833e7d5e060`. Two default-run diagnostic builds each reproduced the frozen trajectory SHA-256 `cfbf6e330688494f0acf37ce810966d0a162c9015c61872c880e2da626a98233`. `git diff --check` passed. The gnssFGO checkout was already dirty before this task; no claim of a clean worktree or new commit is made. HEAD at reporting: `0a8e49728ba96ba7333659383eea05f7249fd7dd`.

No CBS-ON, Kimera, GNSS, covariance scaling, gating, or robust-kernel experiment was run in this audit.
