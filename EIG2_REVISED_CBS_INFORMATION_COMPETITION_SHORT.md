# EIG-2 revised-CBS information competition: bounded 40 s experiment

## Scope and result

This is the requested **short EIG-2 experiment**, not a 305 s rerun and not an EKF experiment. It tests current-smoother relative beliefs with monotonically increasing versions and factor replacement in both directions. The run uses mission `2024-11-03-13-59-54`, bag offset 5.7 s, 40.0 s of sensors at 0.10× playback, Hesai + CPT7 for GLIM, and Alphasense front stereo + the same CPT7 for Kimera. Both estimators use the retained EIG-2 profiles copied from the preceding full-run directory. Their OFF and ON profile directories compare identically; the intended difference is bidirectional CBS and its opt-in revision semantics. The raw source bags are the mission's `_hesai.bag`, `_cpt7_imu.bag`, and `_alphasense.bag` under `src/datasets/GrandTour/EIG-2/`.

**Main observation:** on this interval, CBS-ON worsened both optimizers against ground truth. GLIM's prism-aligned ATE went from **0.0114 to 0.0202 m**; Kimera's *optimizer-state* ATE went from **0.0373 to 0.0557 m**. This does not imply that revision replacement failed. It means that correctly replaced, very confident external relative beliefs can still disagree with the native graph and with the reference.

The source-to-receiver covariance matrices are exactly preserved in the logged transport checks. The 6-DoF *total native-graph* Schur comparison is available for 13 GLIM→Kimera and 20 Kimera→GLIM sample events. A mathematically defensible **per-factor-family** Schur decomposition was **not** recorded; consequently this report does not invent numerical visual-vs-IMU-vs-prior ratios, nor a gnssFGO Combined-IMU/WNOJ ratio. That is an explicit limit of this audit, not a claim that covariance alone is optimization weight.

The first ON preflight in `on_40s/` was stopped when the unreduced pending revision queue grew rapidly. It is **invalid** as a scientific run. The only ON result below is `on_40s_queue_fixed/`, after opt-in newest-per-BeliefID coalescing in the pending/retry queues. That run has a completion marker, full trajectory coverage, and no reported numerical estimator failure. No source covariance, robust kernel, lag, IMU model, or weighting was changed. The live-revision flag remains off by default.

## A. Source relative-belief uncertainty

The sent message carries source, `(from_index,to_index)`, physical endpoint stamps, record stamp, revision, the current relative mean, and all **36** entries of the same-snapshot covariance. Tangent order is GTSAM Pose3 `[phi,rho]`; angle variance is in rad², translation variance in m², and cross entries in rad·m. The statistics in this section are over **all ON published versions**, rather than treating a small covariance as evidence of accuracy. Full distributions (minimum/p05/median/p95/maximum, full eigenvalues, block eigenvalues, log-determinant, condition number and cross-block norm) are in [`belief_audit.json`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/belief_audit/belief_audit.json); complete per-message dense matrices are in the adjacent source CSVs. [`principal_directions.json`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/belief_audit/principal_directions.json) retains representative eigenvectors and dense cross blocks.

| Sender | BeliefIDs; published versions | Median rotation-block trace (rad²); p05–p95 | Median translation-block trace (m²); p05–p95 | Median eigenvalue range, rotation (rad²) | Median eigenvalue range, translation (m²) | Median cross-block Frobenius (rad·m) |
|---|---:|---:|---:|---:|---:|---:|
| GLIM | 390; 17,768 | 5.870e-9; 2.593e-9–1.050e-8 | 1.124e-6; 2.044e-7–1.575e-6 | 8.716e-10–3.366e-9 | 2.786e-7–4.583e-7 | 1.425e-8 |
| Kimera | 196; 4,600 | 1.475e-8; 4.345e-9–3.284e-8 | 6.913e-7; 2.269e-7–2.978e-6 | 3.239e-9–7.378e-9 | 1.127e-7–4.285e-7 | 2.037e-8 |

The median per-axis standard-deviation proxy `sqrt(trace(block)/3)` is about **0.00253° / 0.612 mm** for GLIM rotation/translation and **0.00402° / 0.480 mm** for Kimera. These are *claimed relative-belief* uncertainties, not measured errors. In the marginal block comparison GLIM generally claims more angular certainty, while Kimera's median translational trace is smaller. The beliefs are not all on identical intervals, and the dense cross terms mean a block-only comparison is not a complete 6D ranking. The median full covariance condition numbers are 530 (GLIM) and 146 (Kimera), in mixed coordinates; the median full log determinants are −105.85 and −104.60. Every exported matrix was finite, symmetric within 1e-9 and strictly positive definite under the source audit.

Per-BeliefID version age changes those claims materially:

| Sender | Revision age | Median rotation trace rad² | Median translation trace m² | Median full logdet |
|---|---|---:|---:|---:|
| GLIM | first / 25% / 50% / 75% / last | 7.389e-9 / 5.868e-9 / 5.457e-9 / 5.267e-9 / 5.019e-9 | 2.083e-6 / 1.151e-6 / 1.143e-6 / 1.100e-6 / 9.800e-7 | −103.51 / −105.69 / −105.85 / −105.98 / −106.42 |
| Kimera | first / 25% / 50% / 75% / last | 3.008e-8 / 2.077e-8 / 1.375e-8 / 8.967e-9 / 3.442e-9 | 2.803e-6 / 8.888e-7 / 7.273e-7 / 5.690e-7 / 2.213e-7 | −98.21 / −103.06 / −104.66 / −106.16 / −111.31 |

The stage labels select a version by fractional position *within each ID's version history*, then take a median across IDs. They are not wall-clock ages. The physical-endpoint-to-record lag over **all** revisions has median 2.30 s for GLIM and 2.40 s for Kimera; first-publication median is approximately zero. This is smoother revision age, **not** measured ROS transport latency. GLIM sent 17,378 post-first revision messages. Kimera sent 4,404 post-first messages, of which one duplicated an already seen revision; no older/out-of-order revision was seen in the recorded source products.

### Covariance transport and replacement

The dense transport audit [`transport_audit.json`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/transport_audit.json) matches source rows by `(source,from,to,revision)` and checks the serialized receiver/factor matrices. The logged maximum absolute source→receiver and receiver→factor differences were **0.0** for all 4,549 Kimera→GLIM receiver records and 4,504 GLIM→Kimera transport records. The further 4,500 GLIM→Kimera BPSAM source→used checks also have maximum difference **0.0**. No unmatched transport record was found. These observations rule out an added diagonalization, scale, or floor on those logged paths; they do not establish statistical consistency or independence.

The GLIM receiver logged 194 first inserts and 4,355 replacements. Kimera accepted 192 first revisions and 4,308 newer revisions into its receive path. Matching, active-window retirement and queue coalescing explain why receiver counts differ from sent versions; **counts are not independent measurements**. The persistent factor paths remove a tracked prior version before inserting the newer one, rather than accumulating versions. The source-ID map and same-receiver-edge slot in the implementation prevent a second active factor for that identity. A global maximum-active-CBS-factor counter was not recorded, so an exact peak count is not claimed.

## B. Receiver factor competition

For a sampled incoming factor we use its full `Omega_CBS = Sigma_CBS^{-1}`. The receiver's native graph immediately before insertion is marginalized to the *same endpoint relative-Pose3 tangent*, giving `Lambda_local = Sigma_local^{-1}`. Existing explicit CBS factors are excluded from this native reference. The six generalized eigenvalues of `Lambda_local^{-1/2} Omega_CBS Lambda_local^{-1/2}` are dimensionless and are much more informative than either factor counts or a mixed-unit trace. Greater than one means CBS is stronger in that generalized direction. All matrices are in [`competition_matrices.npz`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/receiver_competition/competition_matrices.npz); sampled event rows and local-factor composition are in the adjacent CSV files.

| Receiver ← external | Samples | Median external/native trace ratio (mixed-unit contextual summary) | Median six generalized information ratios, ascending | Pre-update disagreement/NIS | Observed postsolve external gap |
|---|---:|---:|---|---|---|
| Kimera ← GLIM | 13 | 4.12 (p05–p95 2.79–8.92) | 0.348, 0.578, 1.225, 2.128, 6.391, 8.777 | sampled median raw factor Mahalanobis² 32,783; independence-approximation innovation NIS 4,831 | median relative translation gap 14.06→11.89 mm; rotation gap 0.00702→0.00417 rad over 192 optimizer merges |
| GLIM ← Kimera | 20 | 0.273 (p05–p95 0.090–0.345) | 0.132, 0.167, 0.363, 0.766, 2.015, 3.983 | all 4,548 logged pre-update residuals: median raw factor Mahalanobis² 29,958 | median relative translation gap 23.64→24.70 mm; rotation gap 0.01207→0.01059 rad over 194 optimizer merges |

The two trace ratios are supplied for continuity with the audit log, **not** as physical scalar weights: rotational and translational coordinate units differ. The generalized ratios demonstrate anisotropy: GLIM is stronger than Kimera's native graph in roughly four of six median generalized directions, but weaker in two; Kimera is generally weaker than GLIM's native graph yet stronger in one or two directions. At one representative G2K event (sample 6), block-restricted generalized ratios are rotation **1.65–5.16** and translation **0.49–1.71**; at K2G sample 10 they are rotation **0.181–0.569** and translation **0.585–2.57**. Block restriction ignores rotation/translation coupling; the full six eigenvalues above remain the proper comparison.

The archives retain the external factor's full `Sigma` and the native graph's relative Schur information, but **not** each raw endpoint Jacobian `J_f` or all six raw K2G residual coordinates at the sampled linearization. Therefore this report does not claim to have reconstructed every requested full-state `H_f=J_fᵀ Sigma_f⁻¹ J_f` and `g_f=J_fᵀ Sigma_f⁻¹ r_f`. The measured 6D Schur ratios answer the well-defined total-native-versus-external relative-information question; raw normal-equation family attribution remains a separate unmeasured quantity.

The Kimera native graph in the 13 sampled comparisons contains median 1,268 SmartStereo factors, 25 IMU factors, 25 bias-between factors and one marginalization prior. The GLIM native graph in the 20 samples contains median 706 LiDAR factors, 50 IMU factors, 50 bias-between factors and one marginalization prior. These are **composition counts only**. A family-specific endpoint-relative Schur information cannot be recovered by assigning the total Schur matrix proportionally to counts or raw Hessian norms: `Schur(sum_F H_F) != sum_F Schur(H_F)`, and standalone families can be gauge-singular. Neither the family-specific full Hessians/gradients nor a declared leave-one-family-out attribution was retained. Thus the experiment establishes the CBS-vs-*total-native-graph* directional competition, not numerical CBS-vs-visual-only, CBS-vs-LiDAR-only, or CBS-vs-IMU-only ratios.

Raw Mahalanobis² values are large relative to a nominal 6D chi-square reference; they are **diagnostics**, not gates. They are not innovation NIS unless the native receiver covariance and its correlation with the external source are handled. In particular, the G2K independence-approximation innovation NIS in the sample is still high. Very small `Sigma` plus a centimetre-scale disagreement can generate a large gradient, but the final movement is constrained by the entire nonlinear graph. Postsolve gap change is an observed outcome, not a literal linear weighted average or an isolated update-vector norm. It can even increase when other factors move the same poses.

Eight selected event budgets are in [`competition_finalize/summary.json`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/competition_finalize/summary.json), including dense covariance eigenvalues, cross-block norm, native and external information spectra, full six directional ratios, residual and matched postsolve external gap. A compact subset illustrates the range:

| Direction; sample; receiver edge | External/native directional ratio min…max | Pre-update translation / rotation residual | Raw Mahalanobis² | Postsolve translation gap |
|---|---:|---:|---:|---:|
| G→K; 0; x4→x5 | 0.042…72.6 | 5.98 mm / 0.00147 rad | 2,160 | 6.07 mm |
| G→K; 6; x94→x95 | 0.275…7.13 | 9.86 mm / 0.00837 rad | 32,783 | 13.53 mm |
| G→K; 12; x184→x185 | 0.690…8.41 | 30.17 mm / 0.00699 rad | 32,358 | 21.43 mm |
| K→G; 0; x0→x2 | 0.00275…82.6 | 2.61 mm / 0.000410 rad | 446 | 0.416 mm |
| K→G; 10; x192→x194 | 0.128…4.50 | 26.20 mm / 0.00517 rad | 4,746 | 16.00 mm |
| K→G; 19; x357→x359 | 0.172…2.31 | 7.34 mm / 0.01689 rad | 35,525 | 20.12 mm |

These event rows are selected by start/middle/end and low/high trace-ratio, **not** by ground-truth performance. Full plots against physical receiver time are [G→K diagnostics](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/competition_finalize/g2k_competition.png) and [K→G diagnostics](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/competition_finalize/k2g_competition.png). Source rotational and translational covariance traces are plotted separately; the G→K pre-update residual/NIS series is sampled, while K→G has all 4,548 recorded pre-update residuals. The last panel is *remaining source disagreement after optimization*, not a standalone correction-vector norm.

## C. Bounded gnssFGO graph with the same ON beliefs

The existing GrandTour WNOJ/Combined-IMU iSAM2 replay was run on the same bounded raw CPT7 IMU and **exact ON source-belief CSV rows**, with record-time causal availability and physical relative-factor endpoints. The joint graph uses source-keyed revision replacement. Of 390 GLIM and 196 Kimera source IDs, 390 GLIM and **193 Kimera** have supported endpoints on this GLIM-derived graph grid. Three early Kimera IDs and their 75 versions have no support and were skipped; they must not be represented as accepted factors. There were 22,368 source revision rows, 22,293 factor insertions and 21,710 removals/replacements, with zero replay duplicates or stale rows. The single-source controls use the same source rows and graph settings on this 40 s window; they are diagnostics of solution response, **not** a new estimator-policy sweep.

The factor-construction audit verifies `Sigma_source CSV = Sigma_Gaussian factor` to maximum elementwise difference **3.39e-21** for the joint run. The six residual coordinates at the final graph solution were then checked independently against the dense source covariance in [`fgo_factor_audit.json`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/evaluation/fgo_factor_audit.json).

| Joint-graph source | Accepted final factors | Median native rotation/translation covariance trace | Median blocks of `Sigma^{-1}`: rotation / translation | Median postsolve rotation / translation residual | Median postsolve Mahalanobis² |
|---|---:|---:|---:|---:|---:|
| GLIM | 390 | 5.019e-9 rad² / 9.800e-7 m² | 3.094e9 rad⁻² / 1.188e7 m⁻² | 2.118e-4 rad / 8.07 mm | 344.4 |
| Kimera | 193 | 3.426e-9 rad² / 2.229e-7 m² | 2.908e9 rad⁻² / 4.579e7 m⁻² | 8.411e-5 rad / 3.34 mm | 169.0 |

The inverse-covariance block traces preserve the dense cross covariance; they are **native relative-factor precision blocks**, not projected information after CT interpolation and elimination of GP/IMU nuisance variables. The GLIM and Kimera factors also span different physical intervals. A direct ratio of those numbers would not answer which factor dominates each common trajectory direction.

| Bounded gnssFGO graph | Prism ATE RMSE | 1 s translation / rotation RPE RMSE | Path length | Accepted source IDs |
|---|---:|---:|---:|---:|
| GLIM only | 0.09795 m | 0.02818 m / 0.78093° | 24.423 m | 390 G |
| Kimera only | 0.04268 m | 0.01688 m / 0.66196° | 24.610 m | 193 K |
| Joint | 0.06700 m | 0.02331 m / 0.77942° | 24.494 m | 390 G + 193 K |

At identical graph-support timestamps, the joint solution's median position gap is **0.0451 m** from GLIM-only and **0.1120 m** from Kimera-only; median orientation gap is **0.0863°** and **0.4805°**, respectively. This indicates which *solution* it resembles on this run, but does not decompose normal-equation information into GLIM/Kimera/Combined-IMU/WNOJ contributions. There is no retained per-factor Jacobian/Hessian/gradient or family-wise projected Schur audit for the gnssFGO graph. In particular, this report **cannot** give defensible numerical Combined-IMU or WNOJ competition ratios or a per-translation-axis source dominance claim. The condition is documented rather than replaced by a covariance-only assertion. The recorded source noise-model equality and final residuals are useful facts; the exact CT directional attribution remains unmeasured.

## D. CBS-OFF versus CBS-ON native estimator accuracy

Prism ATE uses the existing GrandTour AP20 prism scorer, calibrated CPT7-to-prism lever arm and rigid Kabsch alignment **without scale**. The 1 s relative error uses the existing CPT7 Inertial Explorer tightly coupled odometry evaluator on the fixed common `[1730638818,1730638855]` interval (361 valid pairs). Ground truth was used only here, not in either estimator or the selection of CBS factors. Kimera's `optimizer` line is the online backend `state_` snapshot at each frame, timestamped by the exact ROS odometry row. `increments` is the separate increment-composed published-odometry path; it is **not** substituted for the optimizer trajectory. The Kimera backend diagnostic logs print position/quaternion with about six significant digits, which limits sub-10-micrometre comparisons, not the reported centimetre-scale result.

| Native estimator product | CBS-OFF ATE | CBS-ON ATE | OFF→ON 1 s translation RPE | OFF→ON 1 s rotation RPE | OFF→ON path length |
|---|---:|---:|---:|---:|---:|
| GLIM optimizer | 0.01140 m | 0.02016 m | 0.01413→0.02529 m | 0.60875→0.77572° | 25.185→25.600 m |
| Kimera optimizer `state_` | 0.03728 m | 0.05574 m | 0.02141→0.04262 m | 0.84795→1.20719° | 25.023→25.647 m |
| Kimera increment-composed output | 0.04293 m | 0.07123 m | 0.01904→0.03571 m | 0.80559→0.78007° | 24.948→25.163 m |

Both runs have 392 GLIM and 198 Kimera online trajectory rows, roughly 39.1 and 39.6 s respectively. OFF has zero external CBS injections; ON metadata records 4,500 G→K and 4,549 K→G accepted/injected revision events. The 40 s ON optimizer-state result therefore **does not show an accuracy benefit in either estimator**. The Kimera published path's slight rotation-RPE improvement does not reverse its ATE/translation degradation and must not be mistaken for the backend optimizer-state response.

## Provenance, validation, and limitations

- Source live versioning is opt-in in `src/glim_ros1/src/glim_ros/cbs_bridge.cpp`, `src/Kimera-VIO/src/backend/VioBackend.cpp`, the ROS belief adapter/message, and `src/cbs/src/bpsam/bpsam.cpp`. Newer versions replace the same tracked source edge; equal/older versions are ignored by receiver checks. Pending/retry queues keep the newest available version for that ID in the opt-in mode. The default non-revision path is unchanged.
- The frozen OFF/ON run directories are [`off_40s`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/off_40s) and [`on_40s_queue_fixed`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed). Both contain `complete.marker`, captured launch/config, metadata, source bag, trajectory and scorer JSON. The stopped `on_40s/` preflight must **not** be used for comparisons.
- Relevant ROS1 targets, including GLIM and Kimera, compiled before the corrected ON run. The existing gnssFGO `grandtour_ct_replay` target was rebuilt successfully with diagnostic source-agent routing, factor-covariance equality check, and postsolve residual-vector output; no GP, IMU or measurement-factor mathematics was changed. Its joint re-run for richer diagnostics differs from the prior joint trajectory by at most **1.21e-8 m** in position; the GLIM control trajectory is byte-identical. This is numerically negligible, but the joint replay is not claimed byte-identical.
- The ON source-belief audit found 390/196 IDs, strictly positive-definite covariances, one Kimera duplicate and no stale published versions; receiver transport tests have zero unmatched rows and zero logged covariance alteration. gnssFGO's 3 unsupported Kimera IDs are accounted for explicitly.
- [`competition_samples.csv`](../V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/receiver_competition/competition_samples.csv), matrix archive, source revision CSVs and factor residual CSVs retain the numerical evidence. Analysis scripts are `tools/eig2_short_belief_audit.py`, `eig2_short_transport_audit.py`, `eig2_short_kimera_paths.py`, `eig2_short_competition_finalize.py`, `eig2_short_fgo_inputs.py`, and `eig2_short_fgo_factor_audit.py` in this repository, plus the existing prism/RPE scorers in `cbsms/tools/`.
- Existing source trees contained numerous unrelated pre-existing uncommitted changes and generated run artifacts. This task did not clean or overwrite them. No covariance or other estimator parameter sweep, full-sequence rerun, CBS-ON policy selection by ATE, GNSS factor, EKF change, CI, health weighting or fusion retuning was performed.
- The diagnostic Python tools pass syntax compilation; `git diff --check` passes in the edited gnssFGO, GLIM, Kimera and CBS nested repositories. The bounded live replay and targeted gnssFGO build are the exercised integration checks. A complete repository-wide test or sanitizer campaign was not run, and the previously dirty nested worktrees remain dirty; neither is represented here as passed/clean.

## Direct answers to the eight questions

1. **GLIM's claim:** median ON relative covariance block traces are 5.870e-9 rad² and 1.124e-6 m² across all versions, with dense cross terms retained. This corresponds to sub-millimetre/approximately 0.0025° per-axis proxies, not measured accuracy.
2. **Kimera's claim:** median traces are 1.475e-8 rad² and 6.913e-7 m². Its median translation claim is tighter, angular claim looser than GLIM's over these non-identical source intervals.
3. **Revision evolution:** both generally become more confident; Kimera's median rotation and translation traces fall about 8.7× and 12.7× from first to last per ID, GLIM's about 1.47× and 2.13×. This is a statement about the reported covariance, not improving truth error.
4. **GLIM inside Kimera:** against Kimera's *total native graph*, the 13 sampled factors have median six generalized information ratios 0.348–8.777; GLIM is strong in several directions, especially rotation in the selected block-restricted example. Its strength specifically against SmartStereo versus IMU versus prior is **not separately quantified** by these logs.
5. **Kimera inside GLIM:** against GLIM's *total native graph*, the 20 sampled factors have median ratios 0.132–3.983. It is weaker in most measured directions but can exceed native information in some directions. Separate LiDAR/IMU/prior ratios are **not separately quantified**.
6. **gnssFGO versus IMU/WNOJ:** all accepted belief-factor covariances are preserved to 3.39e-21 and final residual/Mahalanobis distributions are measured. The requested common-tangent projected Combined-IMU/WNOJ information was **not logged**; no defensible numerical relative-strength ratio can be asserted.
7. **Directional dominance:** no source dominates every measured native receiver direction. In gnssFGO the joint *trajectory* is empirically closer to GLIM-only, but direction-wise dominance after CT interpolation/Schur elimination is **unresolved**, not inferred from ATE or covariance traces.
8. **Accuracy on this interval:** **no**. CBS-ON worsens both GLIM and Kimera optimizer ATE and 1 s translation/rotation RPE on this bounded 40 s EIG-2 test. This conclusion is specific to this interval and fixed parameters; no corrective method is proposed here.

**Stop condition:** bounded experiment and evidence report complete; the family-specific information questions above remain explicitly unresolved. No longer EIG-2 run or fusion/weight change was initiated.
