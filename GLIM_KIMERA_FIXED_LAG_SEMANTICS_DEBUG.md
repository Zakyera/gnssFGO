# GLIM / Kimera fixed-lag semantics: bounded EIG-2 audit

Date: 2026-09-24. This is a source-semantics investigation, **not a fusion experiment or belief-policy selection**. No 305 s sequence, CBS-ON, receiver/fusion, weighting, or topology experiment was run.

## Main findings

1. **H1 is true for the consecutive edges in this configuration, by construction.** Kimera's normal first outgoing CBS mean and its increment-composed native ROS odometry use the same relative transform from the same post-optimization snapshot. K0 reproducing native odometry identifies that output product; it is not independent evidence that first publications are more accurate.
2. **H2 is partly supported, but “pure gauge/reanchoring” is too strong.** A fitted common SE(3) motion accounts for 74.46% of the startup pose-change objective and 85.35% in a retained high-change section. Genuine deformation remains. In the latter section residual translation RMS is 7.81 cm, with a 79.69 cm maximum; this is not a rigid window alone.
3. **H3 is not supported as the origin of the state/increment split.** That split is measurable at update 2, before the first marginalization at update 26. A 30 s control with no marginalization still develops a comparable split. Marginalization changes subsequent estimates, but neither its immediate operation nor its prior has been shown to cause the large late-sequence behavior specifically.
4. **Local revision quality is not uniform.** Startup Kimera revisions improve local translation RMS; revisions in the retained high-change section worsen it from 3.55 to 5.93 cm. Thus the previous result cannot be explained solely by mixing individually improved edges from different snapshots.
5. **GLIM publishes a different object:** its current optimized pose, not a product of newest relative increments. Numerically its published pose equals its logged newest smoother state; its first relative edges do not equal increments between successive published poses. Preserving changes to a GLIM smoother-derived product requires revision information, but asynchronous last-valid edges still do not automatically reconstruct one coherent final snapshot.

## Scope, data and reproducibility

Dataset: GrandTour EIG-2 `2024-11-03-13-59-54`, CBS-OFF, Alphasense front stereo and CPT7 for Kimera; existing Hesai/IMU GLIM audit. Retained source data:

- [Kimera full audit](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_kimera_full_revision_20260924/full305): `estimators.log`, `kimera_odometry.csv`, `revision_products/{first,revisions,last_revision,finalized}.csv`.
- [GLIM full audit](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924): full active-pose logs, first-export endpoint logs, revision products and recorded `/glim/cbs/odometry`.
- [New bounded audit artifacts](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_glim_kimera_fixed_lag_semantics_20260924): `baseline30`, `longlag30`, `startup_analysis`, `late_recorded_analysis`, and `reference`.

Two short sections are analyzed, both anchored to Kimera initialization `t0 = 1730638816.7634923`:

| Section | Physical bounds, seconds after t0 | Kimera snapshots/transitions | GLIM snapshots/transitions | Selection reason |
|---|---:|---:|---:|---|
| Startup | 0–25 | 125 / 124 | 244 / 243 | Includes first marginalization |
| Retained high-change | 215–240 | 125 / 124 | 250 / 249 | Contains the largest already-recorded shared-endpoint first-publication change: 1.48081 m at about 228.198 s |

The second section is deliberately a mechanism stress section, **not a representative random sample or a performance-selected policy**. No new late-sequence playback was performed. At every included transition the fit uses all poses common to both logged active windows.

The retained Kimera edge-endpoint union missed state 4 at update 4 and state 3 at update 28. It would not satisfy the requested complete-window test. One 30 s Kimera-only reproduction therefore adds a flag-gated, full-precision logger for every active pose and the increment output. Its 2,923 common logged poses match the retained source **exactly**; output transform maximum elementwise difference is **0**. Missing poses are supplied by this verified reproduction, not carried forward or interpolated. The late retained section has no missing active states.

A second, authorized 30 s Kimera-only control changes only the diagnostic copy of `BackendParams.yaml` from `nr_states: 25` to `200`. It has no retirement in the 30 s playback. Both complete markers exist; both stop at output index 147. Quantitative control comparisons use the identical first 25 s, updates 0–124. The production profile remains at 25. Incoming CBS is disabled in both, and no GLIM node is launched. Source IMU settings are inherited unchanged from the retained run, including the unset `KIMERA_IMU_SAMPLE_SUPPORT` override; this task does **not** switch support conventions based on a receiver setting.

## A. Output/belief relationship

### Exact executed code

- [VioBackend::updateStates, line 4811](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:4811) calls `calculateMainBackendEstimate()`, fetches current and previous poses from that same `state_`, calculates `W_Pose_B_lkf.between(W_Pose_B_kf)` at line 4836, stores the current smoother pose at line 4840, and composes the relative increment into `W_Pose_B_lkf_from_increments_` at line 4846.
- [Backend output, line 1555](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:1555) selects that increment-composed pose unless `no_incremental_pose` is true. The retained run uses the normal increment output.
- [Optimization ordering, line 4486](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:4486): `updateStates(cur_id)` precedes `refreshCbsOutgoingBeliefs(cur_id)` at line 4536, without another intervening optimizer update.
- [refreshCbsOutgoingBeliefs, line 2839](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:2839) requests the pair under the existing 0.20 s horizon/one-pair setting; `getOdometryBeliefs` is called at line 2952. The mean is exported as `Pose3::Logmap(odom.measured_from_to)` at line 3020.
- [BPSAM source mean, line 2548](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/src/bpsam/bpsam.cpp:2548) calculates both endpoint estimates and `from_pose.between(to_pose)` from the same optimizer. Covariance queries do not alter this mean.

Let `X_i^u` be smoother pose i after backend update u, and `I_k` be native increment-composed output k. For an emitted consecutive pair:

```text
D_k = (X_(k-1)^k)^-1 X_k^k
I_k = I_(k-1) D_k
Z_(k-1,k)^(revision 0) = D_k
therefore I_(k-1)^-1 I_k = Z_(k-1,k)^(revision 0).
```

The equality is about the **mean**, not a claim that a pose-only chain reproduces Kimera's full joint covariance or information. It is scoped to consecutive edges: a different horizon that skips poses would not automatically have this identity. The recorded missing CBS edge 3→4 remains missing; no edge was invented. The small component 0–3 and main component starting at 4 need their actual root gauges when reconstructing a trajectory.

For each real edge the numerical error is `Log(Z0^-1 (I_i^-1 I_j))`, reported as rho norm and phi norm:

| Section | Edges | Translation RMS / max (m) | Rotation RMS / max (deg) |
|---|---:|---:|---:|
| Startup | 123 | 1.3331e-12 / 3.2878e-12 | 1.4306e-11 / 1.4942e-10 |
| Retained high-change | 125 | 1.4280e-10 / 2.4972e-10 | 7.0158e-9 / 7.6429e-8 |

The literal six-coordinate radian/metre norm has RMS/max `1.3563e-12 / 3.2888e-12` at startup and `1.8811e-10 / 1.3448e-9` late. Separate-unit errors above are more interpretable. CSV serialization and Exp/Log/quaternion reconstruction account for these negligible errors. Per-edge values are retained in each analysis directory's `kimera_increment_vs_first.csv`.

The product distinction can also be written:

```text
X_k^k I_k^-1 = X_(k-1)^k I_(k-1)^-1.
```

The native accumulator does not retrospectively apply the revision of the previous absolute smoother pose. Consequently, an evolving absolute window and an increment-composed output can separate even before any state is marginalized.

## B. Active-window rigid-motion decomposition

For each consecutive backend snapshot, minimize exactly the requested objective over common poses:

```text
G = argmin Σ_i ||Log((G X_i^u)^-1 X_i^(u+1))||².
```

The implementation uses SE(3) Exp/Log, a rotation-average/translation-average initialization and converged nonlinear least squares. The objective uses **radians and metres with unit weights**, as written in the request; it is not covariance weighted. A known rigid-transform synthetic test recovers G and zero residual to better than `1e-10`. Raw/residual movement statistics pool common-pose transitions; G statistics contain one value per backend transition. CSVs preserve every transition and common pose.

Important interpretation: the translation part of a left transform G depends on the world origin. A rotation about a distant window can produce a large `G.t` without moving the window by that distance. We additionally record the common transform's displacement of the old window centroid. Also, the joint objective can trade rotational residual against translation: fitting it does not guarantee that each channel's separate RMS decreases. Neither a fitted common displacement nor an objective reduction proves a physically unobservable gauge mode in this anchored visual-inertial graph.

### Kimera — median / RMS / p95 / maximum

| Section | Quantity | Median | RMS | p95 | Max |
|---|---|---:|---:|---:|---:|
| Startup | Raw pose translation, m | .001599 | .004034 | .008357 | .030755 |
| Startup | Raw pose rotation, deg | .040196 | .175379 | .507478 | .786581 |
| Startup | G translation, m | .001993 | .002998 | .006399 | .010368 |
| Startup | G rotation, deg | .036086 | .181324 | .500464 | .831353 |
| Startup | Residual translation, m | .001129 | .002425 | .005150 | .016846 |
| Startup | Residual rotation, deg | .030284 | .046854 | .092353 | .161042 |
| High-change | Raw pose translation, m | .099615 | .224527 | .435502 | 1.480810 |
| High-change | Raw pose rotation, deg | .292075 | .594461 | 1.204991 | 3.039642 |
| High-change | G translation, m | .721518 | 1.628915 | 3.794034 | 6.727293 |
| High-change | G rotation, deg | .835378 | 2.081452 | 4.561995 | 9.513933 |
| High-change | Residual translation, m | .021913 | .078142 | .153560 | .796859 |
| High-change | Residual rotation, deg | .619015 | 2.063389 | 4.484378 | 9.934485 |

Fitted objective reduction is **74.46% startup / 85.35% high-change**. Centroid displacement RMS is `.002660 / .202155 m` respectively, rather than interpreting late G translation RMS 1.629 m as a physical window jump. Late residual rotation grows because the specified joint fit rotates the spatial window to reduce much larger positional deformation; this is another reason not to call it a pure rotational gauge shift. Late absolute state/increment gap has translation median/RMS/max `2.78271/2.71674/3.71907 m`, rotation `2.17534/2.31368/3.73854 deg`.

For a genuinely common displacement `X_i' = G X_i`, all short relative transforms cancel G exactly. More generally, writing `X_i' = G X_i Exp(e_i)` gives:

```text
Z_ij' = Exp(-e_i) Z_ij Exp(e_j).
```

Thus the shared part cancels, while differential deformation changes relative beliefs. H2 explains why absolute-state changes can greatly exceed edge changes, **not** why every edge revision must be correct.

## C. Exact first marginalization and onset

[Key timestamp insertion, line 4365](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:4365) uses the keyframe index as the smoother's timestamp, not seconds. With lag 25, [findKeysBefore, line 107](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:107) first finds old keys at **update 26**, physical timestamp **1730638822.1634438**, **5.39995146 s** after initialization. The extra startup interval is real; the source has a longer gap at keyframe 4.

The diagnostic records removal of `x0,v0,b0`, one new marginal factor, five deleted factors. The new prior is factor slot 5929 on `b1|v1|x1`, error 0 at creation. The wrapper first performs its nonlinear incremental update, then calls `marginalizeLeaves` at lines 346–349. GTSAM creates `LinearContainerFactor`s and fixes participating linearization variables at [ISAM2.cpp:739](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/nonlinear/ISAM2.cpp:739); [ISAM2.h:185](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/nonlinear/ISAM2.h:185) documents this linearized-prior behavior. Fixed linearization variables are not frozen nominal estimates.

The immediate marginalization operation changes surviving nominal positions by **0 m** and rotations by at most **8.20e-16 deg** over all 99 nonempty marginalizations in the analyzed startup window. At update 26 the preceding optimizer stage moves common poses by up to `.016626 m / .605326 deg`; the subsequent marginalization itself does not move them. Subsequent updates can respond to the new prior and changed graph.

| Update | Time since t0 (s) | State/increment translation gap (m) | Rotation gap (deg) |
|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 |
| 2 | .399996 | .000959 | .355965 |
| 24 | 4.999955 | .041610 | 1.594740 |
| 25 | 5.199953 | .034605 | 1.620145 |
| **26: first marginalization** | **5.399951** | **.032504** | **1.944788** |
| 27 | 5.599950 | .023803 | 1.559723 |
| 28 | 5.799948 | .012579 | 1.509241 |
| 124 | 24.999786 | .156362 | 1.248619 |

There is no clean onset of the split at marginalization. Common rotation RMS before update 26 is `.317502 deg`, versus `.125398 deg` afterward; residual translation RMS rises `.001250→.002508 m`, but motion/initialization also changes over this section. The control below is needed rather than treating that temporal association as causation.

![Marginalization onset and no-retirement control](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_glim_kimera_fixed_lag_semantics_20260924/startup_analysis/kimera_marginalization_onset.png)

Dashed line: first production marginalization. Top two rows use each run's **entire actual common active population**; the enlarged-horizon population grows, so matched-population statistics are also given below. Third row: state/increment gap. Bottom row: retrospective first-to-last jointly valid edge change, plotted at the edge endpoint, **not at revision arrival time**. This last curve uses the retained native-lag revisions only; its ~4.8 s future maturity must not be mistaken for an instantaneous marginalization effect.

## D. Local correctness of revisions against IE-TC

Exact reference bag: [2024-11-03-13-59-54_cpt7_ie_tc.bag](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag), 72,401 loaded reference rows.

Reference: retained `/boxi/inertial_explorer/tc/odometry`, ENU world, CPT7 IMU body. The existing reference loader is reused. Positions are interpolated linearly and rotations by SLERP only for reference sampling, rejecting gaps above 50 ms. This is a **direct CPT7-body relative interval test**, not AP20 ATE, no global alignment, no scale, no prism lever-arm shift, and no new source trajectory. It should not be conflated with the older prism-frame 1 s RPE tables.

For every retained edge use exactly the same physical `(ti,tj)` at all five ages. Select the recorded revision nearest 0/25/50/75/100% of first-to-last **availability age**, never invent an interpolated belief. Kimera typically selects revisions 0/6/12/18/24, ~0/1.2/2.4/3.6/4.8 s. Later Kimera revision record times are the previous audit's conservative odometry-publication availability proxy. Last means **last jointly active**, not separately retired endpoints. All selected IDs are fully matured in the retained full run.

Translation error is `||t(Z)-t(Tref_i^-1 Tref_j)||`; rotation error is the geodesic angle of `Rref^T RZ`. The companion CSV also records the full SE(3)-Log rho error. Edge endpoints are selected from startup 0–20 s and high-change 215–235 s, leaving enough support to observe their revisions. Sensor schedules differ, so Kimera and GLIM rows below are not claimed to share exactly the same cross-source endpoints; **within each source every age uses identical endpoints and IDs**.

| Source / section | Age | Translation median / RMS (m) | Rotation median / RMS (deg) |
|---|---:|---:|---:|
| Kimera startup, 98 edges | 0% | .003196 / .004201 | .122739 / .141815 |
| | 25% | .002681 / .003468 | .117296 / .140175 |
| | 50% | .002778 / .003512 | .116460 / .140615 |
| | 75% | .002937 / .003536 | .117568 / .140614 |
| | Last | .002815 / .003572 | .117537 / .140764 |
| Kimera high-change, 100 edges | 0% | .015229 / .035549 | .079479 / .093074 |
| | 25% | .026742 / .041446 | .077779 / .093935 |
| | 50% | .040524 / .049571 | .079029 / .092434 |
| | 75% | .051572 / .056202 | .077487 / .091760 |
| | Last | .060151 / .059322 | .075514 / .091575 |
| GLIM startup, 192 edges | 0% | .006844 / .009260 | .106349 / .126967 |
| | 25% | .005318 / .006418 | .095101 / .120958 |
| | 50% | .005157 / .006357 | .093724 / .120272 |
| | 75% | .005140 / .006354 | .093515 / .120144 |
| | Last | .005153 / .006353 | .093515 / .120112 |
| GLIM high-change, 200 edges | 0% | .009546 / .011716 | .096383 / .120340 |
| | 25% | .006577 / .007940 | .091863 / .111289 |
| | 50% | .006563 / .007953 | .091551 / .111141 |
| | 75% | .006570 / .007957 | .091534 / .111032 |
| | Last | .006542 / .007953 | .091523 / .111006 |

Kimera startup translation RMS improves about 15%; late translation RMS worsens about **67%**, while late rotational RMS improves only slightly. GLIM revisions improve local error in both inspected sections. This answers the requested distinction: **locally worse revised Kimera translations demonstrably occur**; chain construction is not the only possible explanation.

A chain without loops is always algebraically integrable, so “inconsistent” must be used carefully. Mixed-snapshot edges need not telescope to any particular native smoother trajectory, but that does not mean an unsatisfiable pose-only graph. This audit does not use ATE to choose a source's preferred revision policy.

## E. Bounded no-marginalization control

Before update 26, all **351** compared active poses match exactly across lag 25 and lag 200. At update 26 the surviving means still agree within `1e-9 m/deg`; the first above-tolerance difference is update **27**, `1.207e-6 m / 8.001e-5 deg` maximum across common poses. This timing is consistent with a subsequent update using the different retained graph/prior, not an instantaneous pose jump when eliminating leaves.

| First 25 s diagnostic, RMS | Lag 25 | Lag 200, entire population | Lag 200, same recent pose IDs as lag 25 |
|---|---:|---:|---:|
| Raw pose translation, m | .004034 | .004413 | .007076 |
| Raw pose rotation, deg | .175379 | .108661 | .175224 |
| G translation, m | .002998 | .002737 | .005981 |
| G rotation, deg | .181324 | .184176 | .181653 |
| Residual translation, m | .002425 | .002773 | .002452 |
| Residual rotation, deg | .046854 | .055029 | .054320 |
| State/increment translation gap, m | .094649 | .102332 | .102332 |
| State/increment rotation gap, deg | 1.114879 | 1.127177 | 1.127177 |

The last column only restricts **analysis IDs**; it does not remove anything from the enlarged-horizon estimator. Its apparent population retirement field in the CSV describes this analysis mask, not real marginalization. The real control logs zero nonempty marginalization events and retains every pose. Complete median/p95/max statistics for both populations are in `startup_analysis/summary.json`.

Avoiding marginalization does not remove or reduce the startup state/increment split. The structural output definition and ordinary reoptimization are already sufficient. **We cannot isolate a pathological marginal prior as the cause of the late 215–240 s deformation from this startup-only control.** Changing lag also changes which historical nonlinear visual factors/states survive and the ordering constraints; it is not a prior-only intervention. No production-lag recommendation or longer ablation follows from these results.

## F. GLIM comparison

### Numerical decomposition, identical physical sections

| Section | Quantity | Median | RMS | p95 | Max |
|---|---|---:|---:|---:|---:|
| Startup | Raw translation, m | .000040 | .000945 | .001665 | .014775 |
| Startup | Raw rotation, deg | .000325 | .007855 | .014913 | .096840 |
| Startup | G translation, m | .000221 | .000625 | .001362 | .003069 |
| Startup | G rotation, deg | .005583 | .008723 | .016988 | .032292 |
| Startup | Residual translation, m | .000131 | .000872 | .001356 | .014864 |
| Startup | Residual rotation, deg | .005869 | .010440 | .020074 | .103869 |
| High-change | Raw translation, m | .000004 | .000929 | .001625 | .014170 |
| High-change | Raw rotation, deg | .000018 | .006235 | .010776 | .081528 |
| High-change | G translation, m | .006484 | .008900 | .015925 | .033076 |
| High-change | G rotation, deg | .007961 | .009927 | .017715 | .030985 |
| High-change | Residual translation, m | .000164 | .000845 | .001254 | .012961 |
| High-change | Residual rotation, deg | .008105 | .011235 | .019487 | .081656 |

Common-motion objective reduction is **12.94% startup / 14.14% late**, and centroid displacement RMS `.000342 / .000253 m`. GLIM's movement in these windows is much smaller and less dominated by one common rigid displacement. Separate rotational residual increases are again permitted by the mixed metre/radian objective, not evidence of an optimizer failure.

| First→last jointly valid edge change | Translation median / RMS / p95 / max (m) | Rotation median / RMS / p95 / max (deg) |
|---|---|---|
| Kimera startup | .003000 / .003692 / .005967 / .009738 | .011131 / .024148 / .026789 / .170894 |
| Kimera late | .039323 / .044998 / .078432 / .093306 | .016318 / .021433 / .040237 / .049126 |
| GLIM startup | .004338 / .005800 / .011114 / .018363 | .030141 / .044222 / .086183 / .156007 |
| GLIM late | .005140 / .006784 / .012504 / .017854 | .037784 / .046950 / .080713 / .127565 |

### Why the output products differ

[GlimCbsBridge::publishOdometryAndRerun, line 3536](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/src/glim_ros/cbs_bridge.cpp:3536) calls `smoother.calculateEstimate(last_pose_key_)` at line 3551 and copies that pose into ROS odometry at line 3561. There is **no Kimera-style increment accumulator**. The [GLIM odometry implementation](/home/yeranis/repos/V4RL/cbs_gtsam4.3/clean_glim_ws/src/glim/src/glim/odometry/odometry_estimation_imu.cpp:391) also overwrites active frame poses from the smoother, while retirement/export occurs around lines 334–364. A matured dump and a historical online ROS trajectory are distinct products and must not be silently equated.

Numerically, latest logged smoother pose versus recorded `/glim/cbs/odometry` is **zero translation and rotation difference** for all 244 startup and 250 late samples. Conversely, the relative transform between historical published endpoint poses differs from the first same-snapshot CBS mean:

| Section | Compared GLIM edges | Translation RMS / max (m) | Rotation RMS / max (deg) |
|---|---:|---:|---:|
| Startup | 242 | .006253 / .019307 | .049481 / .157810 |
| Late | 248 | .006700 / .017281 | .047315 / .110110 |

GLIM's 0.20 s edges commonly skip one 0.10 s pose; the comparisons use their exact two endpoint IDs, not an invented consecutive topology. GLIM's first retirement is update 51, timestamp `1730638822.478157`, consistent with the approximately five-second physical lag. Its smoother also performs a nonlinear update and then leaf marginalization: [IncrementalFixedLagSmootherExt::update](/home/yeranis/repos/V4RL/cbs_gtsam4.3/clean_glim_ws/src/gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.cpp:43).

For GLIM, `O_j=X_j^j`, but a first relative belief is `(X_i^j)^-1 X_j^j`, generally not `(X_i^i)^-1 X_j^j`. Chaining first means discards revisions of the shared historical endpoint, with no native increment-output identity to rescue it. If the downstream object is a revised/matured smoother product, retaining revised information is necessary; **last-valid replacements are not a theorem of exact final-pose preservation**, because adjacent edges may still mature in different snapshots. For Kimera, first means already define its intentionally increment-composed online product; replacing them changes that product. Neither fact proves which downstream fusion policy is statistically best.

## Validation, commands and limits

Analysis implementation: [fixed_lag_semantics_debug.py](/home/yeranis/repos/gnssFGO/tools/fixed_lag_semantics_debug.py). It validates known rigid motion, SE(3) consistency, common-snapshot agreement, reference coverage and GLIM timestamp matching. Per-pose movement, per-update G/residual, state/increment gaps, every tested revision error and source-output comparisons are retained as CSV/JSON. The figure was visually inspected. Python syntax and shell syntax checks passed. This task does not claim a full estimator test suite or sanitizer run for a read-only logger.

The only new estimator-source edit is the read-only `KIMERA_FIXED_LAG_SEMANTICS_AUDIT=1` block in `VioBackend.cpp:4849`. It runs after the existing increment update, logs `state_` and the output at precision 17, and is disabled by default. Existing dirty Kimera changes were preserved; no dynamics, covariance, graph, output or default horizon was edited. `catkin build kimera_vio kimera_vio_ros --no-deps -j8` completed in the existing Noetic `cbsms_ws_gpu` container after sourcing `/opt/ros/noetic/setup.bash` and `/workspace/cbs_gtsam4.3/devel/setup.bash`.

The bounded launcher is [run_kimera_semantics_short.sh](/home/yeranis/repos/gnssFGO/tools/run_kimera_semantics_short.sh), copied unchanged to `/tmp/run_kimera_semantics_short.sh` in that container. Exact invocations:

```bash
docker exec --user cbsms cbsms_ws_gpu bash /tmp/run_kimera_semantics_short.sh \
 /workspace/cbs_gtsam4.3/runs/_glim_kimera_fixed_lag_semantics_20260924/baseline30 \
 /workspace/cbs_gtsam4.3/runs/_eig2_kimera_full_revision_20260924/full305/kimera_profile 11461
docker exec --user cbsms cbsms_ws_gpu bash /tmp/run_kimera_semantics_short.sh \
 /workspace/cbs_gtsam4.3/runs/_glim_kimera_fixed_lag_semantics_20260924/longlag30 \
 /workspace/cbs_gtsam4.3/runs/_glim_kimera_fixed_lag_semantics_20260924/long_lag_profile 11462
```

Both use `rosbag play --clock --rate=0.10 --start=5.7 --duration=30.0` with only CPT7 and stereo image topics. The retained three bag files establish the same playback origin; **no LiDAR topic is played**. `cbs_belief_receive_enable=false`, `cbs_backend_enable=true`, horizon `.20`, one outgoing pair per update. The existing marginalization diagnostic is enabled; detailed shadow/reference-optimizer diagnostics are not enabled.

The reference export script [fixed_lag_semantics_reference.py](/home/yeranis/repos/gnssFGO/tools/fixed_lag_semantics_reference.py) runs in Noetic using the existing IE-TC loader. Numerical analysis runs in the existing Humble container. Its parameters are the two retained run directories above, `--reference ART/reference/ie_tc.npz --glim-online ART/reference/glim_online.npz`, `--seconds 25 --edge-seconds 20`. Startup adds `--baseline-log ART/baseline30/estimators.log --ablation-log ART/longlag30/estimators.log`; late analysis instead uses `--start-sec 215`. `ART` denotes the new bounded artifact root given above.

Frozen hashes were rechecked and match the prior reports:

| Retained product | SHA-256 |
|---|---|
| Kimera native ROS CSV | `369277944988416ed6728bdb37799afcfa34858a51e30718ace891e273b62e90` |
| Kimera first | `37e5ec3071d6cc3be00cae48e935a7d540130dcf6d332c5eeec1e7a8b651bd81` |
| Kimera revisions | `13c297ddcba5ccea221258e7aa024844489d163886bc46552f2e91f84804ab47` |
| Kimera last | `11bf763133c4c2dc22118266aee95dd47653c1cc739b5c571a653742e80427b7` |
| Kimera finalized | `4a94d8512f743efd23cc89f7cc19dc68dacc3a16089142d394d1d0f6519f38c5` |
| GLIM first | `49154b46701244dcf830b6944835721774db7750408458000fad0fbc1cf56a48` |
| GLIM revisions | `a7b600211b6f80f665040ad57a8723473864875563fdc6bd437ece6c82949d80` |

Repository base: gnssFGO `0a8e49728ba96ba7333659383eea05f7249fd7dd`. The new report and diagnostic tools are isolated from receiver code. The nested Kimera checkout already contained uncommitted prior changes; this report does not claim a clean nested worktree or silently commit those changes.

**Stop:** H1 is established algebraically; H2 is partial rather than pure rigid motion; H3 is not established as the cause and is contradicted as the onset explanation. No first/latest belief policy is selected, no fusion run is authorized by these findings, and no production method is changed.
