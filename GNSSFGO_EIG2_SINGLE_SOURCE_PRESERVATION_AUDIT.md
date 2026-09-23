# EIG-2 single-source preservation audit

Date: 2026-09-23. Scope: retained EIG-2 GLIM runs and beliefs, isolated pose-only diagnostics, and the unchanged gnssFGO GLIM-only replay. No GLIM/Kimera estimator run, fusion change, covariance scaling, or parameter tuning was performed. The CBS-OFF case was resolved first; CBS-ON was then inspected with the same diagnostic procedure.

## Finding first

**The first substantial loss is already present in the exported GLIM relative means, before the gnssFGO IMU or GP factors are introduced.** On each legitimately connected CBS-OFF belief chain, native-derived relative transforms reproduce the scored native GLIM trajectory at numerical precision and score about **0.045 m** prism ATE. Replacing only those transforms with the **actual exported** GLIM beliefs gives **0.813 m** and **0.681 m** ATE on the even and odd chains, respectively, under the same per-chain GT masks. The full receiver subsequently improves those chain scores to about **0.406 m**; it does **not** create the first loss. It still cannot recover the scored 0.045 m native path.

The source code constructs each exported belief from **the smoother's two pose estimates at publication time**, whereas the 0.045 m reference is a shutdown-written `odom_imu.txt` dump. The exact publication-time endpoint poses were not retained (`glim_cbs_export_endpoint_audit_enable: false`), so the amount attributable specifically to later endpoint revision cannot be proven for individual edges. What *is* proven is that the retained exported edge means are not the relative transforms of the scored native trajectory. This is a product/information mismatch, not evidence of a `Pose3` multiplication or CSV adapter bug.

## 1. Native reference, input identity, and evaluation

The run is GrandTour **EIG-2**, mission `2024-11-03-13-59-54`, 305 s sensor window starting 5.7 s into the bag, Hesai LiDAR + `/boxi/cpt7/imu` for GLIM and Alphasense front stereo + the same CPT7 for Kimera. CBS-OFF retained source run:

`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_off_full305_start5p7_rate010_20260921`

The **actual 0.045263659 m native ATE** was computed from that run's `odom_imu.txt`, *not* `traj_imu.txt` or the `/glim/cbs/odometry` CSV. `experiment_runner_used.sh:391-410,543-550` copies the GLIM shutdown dump and invokes `score_grandtour_prism.py` on `odom_imu.txt`. `global_mapping.cpp:603-626` writes `odom_imu.txt` from stored `submaps[*]->odom_frames[*]->T_world_imu`; it writes `traj_imu.txt` separately from globally transformed submap frames. Thus the scored product is a **shutdown dump of stored odometry-frame poses**, not the global-map trajectory. It is not demonstrably a byte-for-byte first-publication trajectory: contemporaneous `/glim/cbs/odometry` has the same 3,042 timestamps but differs from the dump by 0.0108 m RMS position and 0.0888° RMS orientation. Whether individual stored frames were revised later is not directly logged.

The scorer treats the poses as CPT7 IMU/body poses. It applies the retained CPT7-to-AP20-prism transform (`[0.308837802, -0.038312997, -0.234944116]` m and the rotation in `score_grandtour_prism.py:20-28`), matches `/boxi/ap20/prism_position` with the original 0.20 s reference-gap rule, and fits a **single no-scale SE(3) Kabsch alignment**. There are 3,042 estimate poses from `1730638817.279942989` to `1730639121.379048109` (304.099 s). The *identical* native/receiver GT mask contains 1,945 poses in nine segments, with 193.5955 s matched elapsed coverage; its first and last matched estimate times are `1730638817.279942989` and `1730639121.179214001`. Mask-bit comparison between source and receiver aligned CSVs found **zero disagreements** in both CBS modes. The 1 s translational/rotational RPE reference is the retained CPT7 IE-TC trajectory, not prism orientation.

| Retained full trajectory | Prism ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Prism-frame full path (m) |
|---|---:|---:|---:|---:|
| CBS-OFF native GLIM `odom_imu.txt` | 0.045264 | 0.018241 | 0.608075 | 186.368 |
| CBS-OFF gnssFGO GLIM-only | 0.405834 | 0.019592 | 0.634771 | 185.877 |
| CBS-ON native GLIM `odom_imu.txt` | 0.042980 | 0.034541 | 0.639596 | 192.238 |
| CBS-ON gnssFGO GLIM-only | 0.482159 | 0.034096 | 0.677750 | 185.189 |

The graph summary's unlevered body-path length (CBS-OFF 182.812 m) is **not** the same quantity as the scorer's prism-frame 185.877 m path; the table consistently uses the latter.

## 2. Native-derived reconstruction and graph connectivity

The exact 3,040 exported endpoint pairs are `(0,2),(1,3),...,(3039,3041)`: a 0.20 s target horizon over a roughly 0.10 s pose stream. As an endpoint graph, these form **two disconnected 1,521-pose trees**, even indices 0–3040 and odd indices 1–3041. The exporter audit's `connected_component_count: 3040` is a *consecutive-file-row chain-break count*, not graph connectivity: its scanner compares each row only with the immediately previous interleaved row. This diagnostic label is wrong for this edge map, but the exporter explicitly retains all 3,040 edges and does not alter measurements because of it.

For each actual pair, `T_i^-1 T_j` was computed from the *scored* native poses. Deterministic composition with one **native root gauge per component** reproduces every native pose: maximum right-local translation error `5.93e-13 m`, RMS `2.63e-13 m`; maximum orientation error is at numerical precision. An isolated GTSAM pose-only graph with these native-derived `BetweenFactor<Pose3>` means, unit test noise, and one constrained gauge at each true component root has initial cost `2.23e-34`, final cost `2.56e-27`, and maximum optimized pose movement `1.78e-15 m`. No IMU, GP, Kimera, GNSS, or cross-component bridge was included.

**The two native gauges are diagnostic reference values, not information available from the exported beliefs.** A pose-only graph of the beliefs cannot determine the even-to-odd gauge. Accordingly, component ATEs below are scored **separately**; an interleaved two-gauge TUM would produce artificial jumps and is not a valid full-trajectory ATE.

## 3. Actual exported beliefs: first measured degradation

The unchanged exporter `export_cbs_unique_relative_beliefs.py` selected the latest valid approximately 0.20 s GLIM belief per message array, deduplicated exact edge identities, and wrote all six `relative_mu` coordinates and all 36 covariance entries with `.17g` formatting. CBS-OFF input is `.../off/beliefs/glim_beliefs.csv`, SHA-256 `4ba70e11298ba0f8337fb9806b7986f3ecbc2eb4eee817b16743059abfa43782`. The selected belief endpoint times match native TUM endpoints within at most **119 ns** of TUM decimal rounding. There are no duplicate publications in the retained exporter audit. All 3,040 belief record times are later than their physical end times (minimum 99.8 ms; median 141.3 ms; maximum 210.2 ms).

Using **the actual exported means** instead of native-derived means in the same two pose-only trees gives:

| CBS-OFF, same component mask | Native-derived ATE (m) | Actual-export ATE (m) | Native-derived 1 s t/r-RPE (m/deg) | Actual-export 1 s t/r-RPE (m/deg) |
|---|---:|---:|---:|---:|
| Even, 972 prism matches | 0.045285 | **0.812580** | 0.019823 / 0.862477 | 0.029866 / 0.886490 |
| Odd, 973 prism matches | 0.045242 | **0.681022** | 0.020017 / 0.864297 | 0.029960 / 0.889342 |

The component RPE values use the same original 1 s evaluator parameters but a 0.20 s pose subset, so they should be compared **within each row**, not numerically equated with the full 0.10 s native RPE in Section 1. GTSAM's exported-edge pose-only graph fits the exported factors to near zero (translation residual RMS `2.01e-7 m`, final cost `9.15e-15`). Its maximum departure from direct exported-edge composition is only 0.312 mm. Thus the result is not a failed optimizer or a `Pose3` sign convention.

For `d_ij = Log[(Z_ij^export)^-1 (T_i^native)^-1 T_j^native]`, CBS-OFF over 3,040 edges:

| Right-local error | Median | RMS | p95 | Maximum |
|---|---:|---:|---:|---:|
| Translation norm (m) | 0.004636 | 0.006550 | 0.012383 | 0.027187 |
| Rotation norm (deg) | 0.033423 | 0.048192 | 0.090914 | 0.219621 |

The [full edge CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/edge_disagreement.csv), [worst 20](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/worst20.csv), and [time-series plot](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/export_vs_native_edges.png) preserve this check. Worst translation edges include ordinal 830 (`830→832`, 27.19 mm, 0.1414°), 775 (`775→777`, 24.17 mm), and 174 (`174→176`, 23.76 mm). Their exported transforms, shutdown-dump endpoints, and each online endpoint at *its own* publication are in [representative_edges.json](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/representative_edges.json).

**Publication-snapshot limit:** `cbs_bridge.cpp:3210-3224` obtains both endpoint means from `smoother.calculateEstimate` when it publishes the belief; `cbs_bridge.cpp:3043-3076` sends `Pose3::Logmap` and the dense covariance. The saved `cbs.bag` contains the belief arrays and `/glim/cbs/odometry`, but **not** the historical `from_pose` estimate *at that later belief publication*. The optional `GLIM_CBS_EXPORT_ENDPOINT_ROW` log was disabled in this run, and no re-publication of a selected exact edge is recorded. Therefore the exact `from_pose`/`to_pose` at publication for worst edges cannot honestly be reconstructed; the report does **not** label their discrepancy an exporter implementation bug. The difference between the scored shutdown dump and the online pose topic (Section 1), together with the publication-snapshot code path, supports online-versus-later data-product mismatch, but does not quantify later revisions edge by edge.

## 4. The actual gnssFGO receiver graph

This is the **custom** `online_fgo/src/offline_process/GrandTourCsvReplay.cpp` runner, not a claim that every component of a complete GNSS-enabled gnssFGO system was exercised. The retained command used `--mode glim --timeline record --solver isam2`, the EIG-2 raw IMU CSV, and the actual GLIM belief CSV; no GNSS or Kimera measurement was loaded in this case. `GrandTourCsvReplay.cpp:260-305` reads the six-coordinate mean and full 6×6 covariance without diagonalization/scaling; `:435-444` constructs `fgo::factor::BetweenPose3Factor` with `Gaussian::Covariance`. The `record` timeline waits for `record_timestamp_ns`, then inserts the factor on the **historical endpoint keys** into full iSAM2. It is not a fixed-lag graph or a current-pose pseudo-measurement.

| Factor family in CBS-OFF GLIM-only graph | Count | Connected variables | Noise/model | Timing |
|---|---:|---|---|---|
| Initial pose prior | 1 | `X(0)` | diagonal sigma `[0.174533,0.174533,0.00174533,1e-5,1e-5,1e-5]` in Pose3 tangent | first support |
| Initial velocity, bias, angular-rate priors | 3 | `V(0),B(0),W(0)` | diagonal sigmas in `GrandTourCsvReplay.cpp:723-733` | first support |
| `CombinedImuFactor` | 3,041 | adjacent `X,V,B` states | retained CPT7 preintegration and bias random walk | between support states |
| `GPWNOJPrior` | 3,041 | adjacent `X,V,W` states | WNOJ, `Qc=diag(10000)` | between support states |
| Direct GLIM relative `BetweenPose3Factor` | 3,040 | `X(i),X(j)` | exported dense 6×6 Gaussian | available at record time, attached to exact endpoint states |
| Interpolated GLIM factors | **0** | — | — | all GLIM endpoints coincide with the 3,042 support timestamps |
| GNSS, Kimera, other external/robust factors | **0** | — | — | — |

Total: 9,126 factors, 3,042 support poses, four state types per support (`X,V,B,W`), 12,168 final active values. Bias evolution is inside `CombinedImuFactor`; there is no additional standalone bias-evolution factor. There is no separate velocity measurement factor beyond the initial prior and the two dynamic factor families. All 3,040 GLIM beliefs were accepted; zero lacked support, zero were gated, and no robust kernel/weight was applied. The saved summary records no per-update optimizer iteration count or IMU/GP residual breakdown, so these cannot be asserted retrospectively.

## 5. Nearest valid receiver add-back and schedule control

The only mathematically valid isolated pose-only model is the two-chain graph with two explicit diagnostic gauges. The production runner always inserts **Combined IMU and WNOJ together** when it creates each support state; it exposes no independent factor-family switch. Its IMU-only control, attempted unchanged on the same EIG-2 support grid, terminated with an `IndeterminantLinearSystemException` near `x218` and has **no valid score**. We did not alter priors, noise, or solver to force a number. Separate IMU-versus-WNOJ or GP-interpolation ATE claims would be invented; the latter has **zero** GLIM factors here.

The valid add-back is therefore exported-belief pose-only → full existing GLIM-only graph (IMU + WNOJ + one initial state prior together). The full graph *reduces*, rather than creates, the exported-only ATE gap:

| CBS-OFF, identical per-chain prism masks | Native-derived pose-only ATE | Actual-export pose-only ATE | Full graph ATE | Full-graph 1 s t/r-RPE (m/deg) |
|---|---:|---:|---:|---:|
| Even | 0.045285 | 0.812580 | **0.406114** | 0.021460 / 0.881083 |
| Odd | 0.045242 | 0.681022 | **0.405554** | 0.021500 / 0.884599 |

The original all-row full graph is 0.405834 m ATE. Its final GLIM unwhitened factor residuals have median/RMS translation `0.00731/0.01019 m` and rotation `0.0158/0.0245°`; the median/full-covariance whitened residual norm is **8.01** (p95 18.79). By comparison, the exported edges relative to the scored shutdown trajectory have median whitened norm **13.51** (p95 38.14). These norms are objective/conflict diagnostics, **not valid independent chi-square NIS tests** against a later trajectory that helped generate the beliefs. [factor_residuals.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/factor_residuals.csv) and [factor_summary.json](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/off/factor_summary.json) contain all factors.

One unchanged, valid schedule control used the same GLIM, IMU, factors, iSAM2 settings, and support states with `--timeline physical` instead of record arrival. It scored **0.405780 m** ATE and **0.019599 m / 0.634788°** 1 s RPE, versus the saved record-arrival **0.405834 m** and **0.019592 m / 0.634771°**. Position separation between their *unaligned graph poses* is 0.000626 m RMS, 0.002176 m maximum. This rules out the approximately 0.10–0.21 s factor insertion delay as the principal explanation for the 0.36 m native-versus-graph ATE gap. The small nonzero output difference is consistent with different incremental linearization paths; no solver-equivalence claim is made.

## 6. CPT7 and timestamp conventions in the retained EIG-2 paths

`rosbag info` of the actual EIG-2 IMU bag verifies 36,286 `sensor_msgs/Imu` records on `/boxi/cpt7/imu`. The retained GrandTour conversion documented in `CBS_CPT7_TIMESTAMP_PROVENANCE_TASK1.md` maps OEM7 **RAWIMUSX** (`%RAWIMUSXA`, message ID 1462) delta-angle/delta-velocity counts to 100 Hz ROS gyro/acceleration rates. The ROS header and offline bag time come from the RAWIMUSX body GNSS Week/Week Seconds converted to UTC/POSIX, **not live ROS receipt time**. Public NovAtel documentation does not conclusively specify start/centre/end increment support (`CBS_CPT7_RAWIMUSX_SUPPORT_TASK2.md`); the following are software *engineering* conventions, not a proven hardware time-tag fact.

- GLIM: EIG-2 retained launch/config has no recorded `GLIM_IMU_SAMPLE_SUPPORT` override. `glim/common/imu_integration.cpp:17-26,56-103` defaults `current/right`, using the following sample at each positive integration segment and fractional endpoints. This is the actual source path for the GLIM-only result.
- Kimera: EIG-2 profile uses the same `/boxi/cpt7/imu`, identity IMU/body extrinsic, `imu_time_shift: 0.0`, and `do_imu_rate_time_alignment: 0`. No `KIMERA_IMU_SAMPLE_SUPPORT` override is recorded; `DataProviderModule.cpp:126-156` therefore uses its existing **interpolated-border** packet behavior, not the pilot's experimental right-ZOH switch. Kimera is **not** a receiver measurement in the GLIM-only audit.
- gnssFGO: `GrandTourCsvReplay.cpp:323-345,871-900` integrates with `IntegrateAllRight`: sample stamped `t_k` is held over `[t_(k-1),t_k]`, including exact fractional boundary overlaps. It uses the same extracted raw IMU CSV in ON/OFF (SHA-256 `c4d7b68674849c4be3b3b83198050c564ff310fa2167a42aa73217321b933fca`). `SupportAcceleration` and `CorrectedAngularRate` at `:351-377` choose the sample **at or before** a support timestamp for WNOJ endpoint derivatives; 3,040/3,042 GLIM support times are not exact raw IMU stamps. This is a concrete distinction from right-interval preintegration, but its effect is **not isolated** here and cannot explain the already-observed source-to-export gap.
- The exported GLIM physical endpoints are carried as integer nanoseconds; graph association is exact for all 3,040 factors. There is no snapping beyond CSV's sub-microsecond source timestamp conversion and **no GP interpolation** in this GLIM-only EIG-2 case. `record_timestamp_ns` controls causal factor availability, not its physical endpoint keys.

## 7. Covariance adapter

The retained GLIM mode is `glim_cbs_odom_covariance_mode: schur_relative_between` (`resolved_glim_params.yaml:41`). In `cbs_bridge.cpp:3210-3282`, the published mean is `from_pose.between(to_pose)` and the covariance comes from the smoother's two-pose **joint marginal information**. `relative_pose_covariance.h:258-420` uses the actual GTSAM BetweenFactor endpoint Jacobians, changes variables to the relative right-local tangent, Schur-eliminates the remaining absolute-pose coordinate, and inverts the 6×6 relative information. This is an approximate **marginal covariance of that relative transform** at the publication linearization point—not an absolute marginal, not merely the destination conditional block, and not a covariance of the later shutdown-dump trajectory. Its tangent order is **rotation then translation, `[phi,rho]`**, radians and metres squared with cross terms.

The ROS message carries all 36 row-major covariance entries. The exporter writes all 36 with `.17g`; `GrandTourCsvReplay.cpp:260-305` reads those named entries into the same matrix positions, tests finite/symmetry/positive eigenvalues, and `:435-444` passes the dense matrix to `Gaussian::Covariance`. No diagonalization, unit conversion, scaling, robust kernel, or receiver floor appears in this path. The source SPD solver *can* add jitter on failure, but all 3,040 CBS-OFF `G2K` covariance log rows ended `ok` rather than `ok_jitter`. Observed CBS-OFF CSV matrices have maximum symmetry error zero, minimum eigenvalue `1.799e-10`, and nonzero off-diagonal terms up to `1.561e-6`; median rotation/translation block traces are `8.550e-9 rad²` and `2.993e-6 m²`.

The isolated [synthetic adapter test](/home/yeranis/repos/gnssFGO/tools/eig2_covariance_adapter_test.cpp) uses the **actual** `fgo::factor::BetweenPose3Factor`, a nonidentity SE(3) measurement, and dense rotation/translation-coupled covariance. For `T_j=Z Exp(delta)`, it obtains `||residual-delta||=3.73e-17` and error `2.66e-15` between GTSAM's squared whitened norm and `deltaᵀΣ⁻¹delta`. This rules out a basic tangent-order, covariance-versus-sigma, or dense-matrix-whitening adapter error. It does **not** establish that GLIM's estimated covariance is statistically calibrated or independent of gnssFGO's IMU. GLIM's posterior already uses the same CPT7 stream; gnssFGO treats its exported edge covariance as an independent factor in a graph with that CPT7 IMU again. Cross-edge/source-to-IMU correlations are not represented.

## 8. CBS-ON repeat

Only after the CBS-OFF boundary was established, the same diagnostic was run on the frozen bidirectional CBS-ON source. Its GLIM belief endpoint map and 3,040-row count are **identical** to OFF, but its mean and covariance values are not. The source run reports 1,519 Kimera→GLIM injections (and 1,517 GLIM→Kimera). Consequently `gnssFGO GLIM-only / CBS-ON` does **not** mean the GLIM source is Kimera-free; it only means no separate Kimera factor was added in gnssFGO.

| CBS-ON diagnostic | Even component | Odd component |
|---|---:|---:|
| Native-derived pose-only ATE (m) | 0.044152 | 0.041776 |
| Actual exported-belief pose-only ATE (m) | **0.785449** | **0.635750** |
| Full GLIM-only receiver ATE (m), same mask | 0.481946 | 0.482368 |

For the same even/odd 1 s IE-TC masks, ON exported-belief pose-only t-RPE is `0.054895/0.037531 m` and r-RPE `0.926422/0.905614°`; its full receiver has t-RPE `0.035244/0.034951 m` and r-RPE `0.910567/0.899766°`. As with OFF, the receiver improves the exported-only translation RPE on each chain but does not restore native ATE.

CBS-ON versus OFF exported means differ by median/RMS `0.01113/0.02712 m` in relative translation and `0.0836/0.2068°` in relative rotation. ON covariance trace medians are about **0.898×** OFF for translation and **0.924×** for rotation, with individual edges varying widely; its median traces are `2.430e-6 m²` and `6.888e-9 rad²`. ON export-versus-its-own-native edge mismatch is larger than OFF: translation RMS **0.01805 m** versus **0.00655 m**; rotation RMS **0.1299°** versus **0.0482°**. ON's final receiver GLIM factor residual is also larger (translation median/RMS `0.01257/0.02561 m`; whitened median **15.28** versus OFF **8.01**). The [ON summary](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/on/factor_summary.json), [ON edge plot](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/on/export_vs_native_edges.png), and [ON per-edge CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/on/factor_residuals.csv) retain these values.

Interestingly, ON exported-belief *pose-only* chains score slightly better than OFF chains, but its full receiver scores worse (0.482 versus 0.406 m). Thus ON's poorer full-graph result cannot be attributed merely to larger pose-only exported ATE. Greater conflict with the fixed receiver dynamics and tighter, potentially correlated source covariances are **suspicious mechanisms**, not causally isolated findings. No common-information correction or weight change was attempted.

## Pipeline verdict and classifications

| Stage | CBS-OFF prism ATE (m) | Change and interpretation |
|---|---:|---|
| Native scored `odom_imu.txt` | **0.045264 full** | Source reference, 1,945 common prism matches. |
| Native-derived 0.20 s relative reconstruction | **0.045285 even / 0.045242 odd** | Same native trajectory at component masks; exact algebraic reconstruction. |
| Pose-only GTSAM, native-derived edges | **0.045285 / 0.045242** | GTSAM convention and two legitimate gauge priors preserve it. |
| Pose-only GTSAM, **actual exported** GLIM edges | **0.812580 / 0.681022** | **First substantial degradation.** Different publication-time relative means and reduced two-chain belief product. Cross-component gauge is unobservable here. |
| + complete existing receiver (initial state prior, Combined IMU, WNOJ) | **0.406114 / 0.405554** on the same masks; **0.405834 full** | Receiver reduces the exported-only error but does not restore native path. No GLIM interpolation. |
| Same receiver, endpoint-time rather than record-time insertion | **0.405780 full** | Timing order changes ATE by only 0.000055 m; not the main cause. |

- **Confirmed implementation bug, noncausal:** the exporter's `connected_component_count` labels interleaved file-row breaks as 3,040 components although the endpoint graph has two. It does not drop or modify edges in this export. No confirmed bug was found in SE(3) mean serialization, factor convention, dense covariance loading, or GT-mask matching.
- **Confirmed modeling/data-product mismatch:** the receiver consumes sparse 0.20 s relative posterior beliefs as if they were independent constraints alongside the same raw IMU, while the compared 0.045 m native result is a different, shutdown-saved odometry trajectory. The beliefs omit the cross-component gauge, source graph's other measurements and correlations, and inter-belief/common-IMU correlations. Their retained means do not reconstruct that scored trajectory.
- **Expected information loss / online-versus-later difference:** two interleaved belief trees cannot alone encode the full 0.10 s trajectory; source code proves their means are publication-snapshot smoother differences. Directly proving how much of each worst edge arose from a later endpoint revision is prevented by the disabled endpoint audit.
- **Suspicious, not proven primary cause:** very small exported covariance relative to later-trajectory edge differences, high post-fit whitened factor residuals, ON shared Kimera/IMU information, and the receiver's preceding-sample WNOJ support derivative beside right-interval IMU preintegration. No independent calibration or ablation here licenses a tuning conclusion.
- **Ruled out as first loss:** wrong endpoint pair direction, `Pose3` Exp/Log ordering, missing 6×6 cross terms in CSV loading, GT frame/mask mismatch, GP interpolation on GLIM factors, and record-vs-physical graph insertion time. The full receiver moves ATE **toward**, not away from, the native reference compared with export-only reconstruction.

## Reproducibility and non-regression

Diagnostic-only sources are `tools/eig2_single_source_preservation.py`, `tools/eig2_pose_only_diagnostic.cpp`, `tools/eig2_factor_diagnostics.py`, and `tools/eig2_covariance_adapter_test.cpp`. Outputs are under `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_single_source_audit_20260923/{off,on}/`; original source runs, belief CSVs, and receiver outputs were read-only. The pose-only executable was compiled against the locally installed GTSAM 4.3 in `ubuntu-jammy-humble-nvidia`. Existing `cbsms_ws_gpu` ROS Noetic and unchanged prism/IE-TC evaluators scored the diagnostic TUMs. The only additional unchanged receiver run was the CBS-OFF physical-timeline control; the IMU-only attempt failed and produced no usable result. No estimator-behavior source was edited.

Frozen input checks: OFF/ON GLIM belief CSV SHA-256 `4ba70e11298ba0f8337fb9806b7986f3ecbc2eb4eee817b16743059abfa43782` / `b3cef39b4ca21be8101a0cbf349424ed7ff96530bf2da716997486f32761ff99`; OFF/ON native `odom_imu.txt` `88bfcf1bedbd9b94eceeeea8e9421b74d667ae28a8f7a4b4e426159e2f94deec` / `9e9ef5b1df9b94a42eba4887539b23d88d857bbb2d66453e9a2777901cd6029b`. OFF/ON retained graph trajectory SHA-256 `e764d305f3749a22073a269078e0149335e3adbdb57c17d73813c77ab23e5fd2` / `e3b2a1ca96283764a3d0d5ed9530400c009a84bccfebc73d6e01766f8e9bc6f9`. These match the retained files after the audit. No source estimator or receiver production file changed.

Audit base commit: `c51e74f7fdcc6052452c692219ac670f05ed1479`. The diagnostic sources and this report are committed together; `git log -1 --format=%H` identifies that audit commit without putting a circular self-hash in the report. **Stop here; no tuning or estimator change is authorized by this audit.**
