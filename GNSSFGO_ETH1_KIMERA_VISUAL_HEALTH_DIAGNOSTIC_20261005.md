# ETH1 Kimera visual-health / outgoing-belief diagnostic

Date: 2026-10-05. Completed matched, source-only, first-120-second ETH1 captures: CBS OFF followed by classic bidirectional CBS ON, both at 0.10×. No gnssFGO, EKF, weighting experiment, or live visualization was run.

## Finding

**There is a real weak-stereo-support signal, but it is not simply a shortage of detected/tracked 2D features and it does not identify every bad outgoing belief.**

At intervals with non-VALID stereo motion, the median number of tracked 2D landmarks remains 280 OFF / 284 ON. However, the median number of cross-keyframe stereo candidates falls to 3 OFF / 3.5 ON, with only 2 returned inliers in either case. First-publication translation discrepancies are larger in these intervals. New feature detection later replenishes the frame, so final feature/depth counts alone obscure the shortage that affected the motion estimate.

Nevertheless, non-VALID stereo status identifies only 14/59 OFF and 12/59 ON intervals in each mode's largest translation-error decile. Some large errors occur with VALID stereo, including one ON interval with 169 candidates and 158 inliers. Rotation discrepancies are particularly poorly explained by this status. These observations do **not** validate a covariance multiplier, a gate, or a universal Kimera degeneracy detector.

In this fresh capture CBS ON reduces the largest translation spikes, but increases the median first-publication translation discrepancy and first-publication rotational discrepancy. Later smoother revisions give a different result and must not be confused with the beliefs originally sent through classic CBS.

## 1. Frozen scope and configuration

The [classic runbook](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md) and repository instructions were followed. The existing ETH1 source recipe and complete saved profile trees were reused, not reconstructed from generic defaults.

| Setting | Both new captures |
|---|---|
| Mission | GrandTour ETH1 `2024-10-01-11-29-55` |
| Bag interval / rate | Offset 0.0 s, duration 120.0 s, 0.10× |
| GLIM | Hesai `/boxi/hesai/points`, CPT7 `/boxi/cpt7/imu`, existing 5 s smoother profile |
| Kimera | Alphasense front stereo + shared CPT7, existing 25-state profile |
| Output body frame | `cpt7_imu` for both sources |
| Native Kimera IMU path | Recovered classic `current`; `KIMERA_IMU_SAMPLE_SUPPORT` unset |
| CBS | `classic`, `live_revision=false`, persistent external factors |
| Publication | 0.20 s nominal horizon, stride 1, one horizon pair per update |
| Receive start delay | 0 s |
| Covariance | Unchanged dense 6×6 `schur_relative_between`; both injection scales 1.0 |
| Adaptive health / weighting | Disabled |
| Matched difference | OFF publishes without receiving; ON exchanges beliefs bidirectionally during playback |
| Diagnostics | Passive frontend health and existing log-only smoother revisions enabled in both |
| Fusion / inspectors | No fusion receiver; presentation and factor-graph inspectors disabled |

The earlier experimental ALL_RIGHT Kimera selector was **not** substituted for the recovered classic source configuration. This task does not change any timing convention. Metadata contains a generic `eth1_mono_shared_cpt7` calibration-overlay path, but the effective mode, copied stereo profile, flags and frame IDs are stereo; that generic metadata field is not the selected camera mode.

Source recipe: [run_mode.sh](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/run_mode.sh), sourcing [the frozen ETH1 environment](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/source.env). The diagnostic uses a separate ROS master on port 11378 and a fresh output directory.

Artifacts are under:

`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/`

The `off_source/` and `on_source/` directories retain `recordings/cbs.bag`, `estimators.log`, native `glim_odometry.csv` and `kimera_odometry.csv`, copied profiles, resolved ROS parameters, launch logs and `complete.marker`.

## 2. What was instrumented, and why

Only default-off, read-only frontend logging was added. No estimation branch, rejection threshold, factor, covariance, bias, calibration, sensor selection or smoother setting was changed.

- [VisualHealthAudit.h](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/include/kimera-vio/frontend/VisualHealthAudit.h:14): enabled only by `KIMERA_VISUAL_HEALTH_AUDIT=1`; counts valid landmark IDs, depth status, landmark ages and 4×4 image-bin occupancy from const frame arrays.
- [StereoVisionImuFrontend.cpp](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp:314): logs immediately after tracking; after mono rejection/current stereo reconstruction; after geometric rejection but before feature replenishment; after replenishment; and at final keyframe output.
- [Tracker.cpp](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/frontend/Tracker.cpp:664): logs the fresh stereo candidate/inlier vectors before outlier removal. Equivalent mono logging is at line 342. Cached debug counters were not used because an early solver return can leave them stale.

The source diff against the retained pre-instrumentation copies consists only of includes and logging calls. The new header contains const reads and local accumulators. The library was rebuilt and its runtime hash verified for both captures. The logger remains available but **off by default**; no shared launcher was changed to enable it.

### Status semantics—not an estimator failure flag

The existing rotation-assisted stereo solver returns INVALID if its maximum coherent set has fewer than two members ([Tracker.cpp](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/frontend/Tracker.cpp:564)). It returns FEW_MATCHES below the saved five-inlier threshold. An empty returned list on INVALID means no certified stereo solution, not zero detected image features.

This solver uses an IMU-derived rotation. Its failures are therefore not automatically camera-only failures. Also, non-VALID stereo does **not** mean the backend update is IMU-only: [getSmartStereoMeasurements](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp:506) preserves usable left observations and encodes unavailable right measurements as NaN. The saved backend has `addBetweenStereoFactors: 0`; stereo motion status affects the pose initial-guess path, not a blanket removal of all visual factors.

## 3. Measurement identity, reference and cohort

Primary: the **actual first outgoing Kimera CBS belief**, matched to health from the same fresh run and same physical endpoints. Secondary: the last observed log-only smoother revision for that edge.

For both captures, all 583 first-publication log records were checked against `/kimera/cbs/odom_belief_out` in the recorded ROS bag:

- Maximum mean-coordinate difference: `1.1102230246251565e-16`, within the ROS bridge's existing Pose3 Exp/Log round-trip numerical noise.
- Maximum covariance-entry difference across all 36 entries: **0**.
- Outgoing ROS revision fields are all zero; later diagnostic revisions were not sent through the experimental live-revision protocol.

There are 582 unique logged edge IDs. Initial edge `0→1` has two different records marked first/revision zero; it is explicitly excluded rather than silently overwritten. The remaining **581 unique intervals match exactly between OFF and ON**, with no unmatched interval. Their duration is approximately 0.20 s (median 0.1999989 s).

Both endpoint timestamps must match the original integer frontend timestamps within 1 µs, allowing existing double-seconds serialization. Maximum observed endpoint rounding is 383 ns. Reference evaluation then uses the original integer stamps.

Reference: retained CPT7 IE-TC `/boxi/inertial_explorer/tc/odometry`, body `cpt7_imu`, from [ie_tc.npz](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/ie_tc.npz). The unchanged [reference helper](/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/covariance_age_reference.py:27) uses linear position interpolation, quaternion SLERP and a 50 ms endpoint-bracket gate.

For measured `Z = Exp([phi,rho])` and reference `Z_ref = T_ref(a)^-1 T_ref(b)`, translation discrepancy is `||t(Z)-t(Z_ref)||` and rotation discrepancy is the angle of `R(Z)^-1 R(Z_ref)`. There is no trajectory alignment or fitted scale. These are **~0.20 s relative-belief discrepancies, not ATE or 1 s RPE**. IE-TC shares CPT7 inputs and is not independent ground truth; no NIS/NEES or covariance-calibration claim follows.

## 4. Visual support and location in time

All 1,196 tracked-frame records per run retain hundreds of valid 2D landmarks: OFF minimum/median/maximum **249 / 291 / 316**, ON **237 / 291 / 317**. This rules out a general disappearance of tracked 2D points in this window, not geometric degeneracy or inaccurate tracking.

Statistics below use the 581 scored endpoint intervals. Non-VALID combines FEW_MATCHES and INVALID.

| Quantity | OFF VALID | OFF non-VALID | ON VALID | ON non-VALID |
|---|---:|---:|---:|---:|
| Intervals | 543 | 38 | 535 | 46 |
| Tracked 2D landmarks, median | 291 | 280 | 289 | 284 |
| Surviving landmarks after mono rejection, median | 205 | 172.5 | 198 | 183 |
| Current valid depths before stereo rejection, median | 110 | 4 | 110 | 4.5 |
| Cross-keyframe stereo candidates, median | 105 | 3 | 106 | 3.5 |
| Returned stereo inliers, median | 91 | 2 | 94 | 2 |
| Final valid depths after replenishment, median | 175 | 68 | 174 | 63 |
| Effective occupied image bins (out of 16), median | 9.891 | 6.814 | 9.931 | 6.630 |
| First-belief translation discrepancy, median (mm) | 3.863 | 9.888 | 7.691 | 13.064 |

OFF has 23 FEW_MATCHES + 15 INVALID; ON has 28 + 18. Of these warnings, OFF has 37/38 after 70 s, ON has 34/46 in 100–120 s. This is not evidence that ON eliminates frontend difficulty: this fresh ON run has **more**, not fewer, non-VALID stereo intervals.

The image-bin metric is `(sum n)^2 / sum(n^2)`, computed on all valid tracked left-image landmarks before rejection. It describes concentration, not an observability eigenvalue. Landmark age is in keyframes, not seconds.

### How predictive is the warning?

Each mode's largest translation-error decile contains 59 intervals. This is a retrospective descriptive label, not a selected operating threshold.

| Warning criterion | OFF largest-error intervals identified | ON identified | OFF / ON precision |
|---|---:|---:|---:|
| Non-VALID at the same endpoint | 14/59 (23.7%) | 12/59 (20.3%) | 36.8% / 26.1% |
| Non-VALID at endpoint or in preceding 1 s | 31/59 (52.5%) | 25/59 (42.4%) | 34.4% / 22.3% |

The second row uses past-only context, but is **not** a tested gate or a proposed tuned timeout. It flags 90 OFF and 112 ON intervals, including many outside the largest-error decile. Same-endpoint warnings identify only 4/59 OFF and 8/59 ON of the largest rotational discrepancies.

Descriptive Spearman association with first-publication translation discrepancy:

| Signal | OFF | ON |
|---|---:|---:|
| Tracked 2D count | −0.210 | −0.083 |
| Stereo candidate count | −0.667 | −0.372 |
| Stereo inlier count | −0.643 | −0.372 |
| Stereo inlier fraction | −0.149 | −0.218 |
| Effective image-bin coverage | −0.457 | −0.164 |

Thus absolute usable stereo support is more informative here than raw track count or inlier percentage. A high fraction can be misleading when only a few candidates remain. Samples are temporally correlated; these are associations, not independent-sample significance tests or causal proof.

### Concrete counterexamples to a simple failure gate

| Mode / elapsed time | Stereo status | Tracked 2D | Candidates / inliers | First translation discrepancy |
|---|---|---:|---:|---:|
| OFF 112.299 s | VALID | 275 | 13 / 7 | 145.685 mm |
| OFF 85.300 s | VALID | 271 | 41 / 38 | 97.280 mm |
| ON 106.299 s | FEW_MATCHES | 290 | 3 / 2 | 48.206 mm |
| ON 46.500 s | VALID | 298 | 169 / 158 | 43.381 mm |

The largest OFF rotational discrepancy, 0.7962° at 18.500 s, also has VALID stereo (54 candidates, 36 inliers, 295 tracked landmarks). The largest ON rotational discrepancy is 0.9576° at 21.100 s with VALID stereo (67 candidates, 59 inliers). Feature/stereo counts alone cannot certify these beliefs as accurate.

## 5. Fresh OFF/ON relative-belief comparison

Same 581 physical intervals; these tables do not substitute for native-trajectory ATE or fused performance.

| First publication | OFF median | ON median | OFF RMS | ON RMS | OFF p95 | ON p95 | OFF max | ON max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Translation (mm) | 4.170 | 7.924 | 12.789 | 10.837 | 22.234 | 19.525 | 145.685 | 48.206 |
| Rotation (deg) | 0.1070 | 0.1990 | 0.1758 | 0.2595 | 0.3443 | 0.4541 | 0.7962 | 0.9576 |

ON has smaller first-publication discrepancy on 26.2% of translation intervals and 13.1% of rotation intervals. Translation tail/RMS improvement therefore coexists with a worse median; there is no uniform improvement of the short outgoing transformations.

| Last observed revision (retrospective) | OFF median | ON median | OFF RMS | ON RMS | OFF p95 | ON p95 |
|---|---:|---:|---:|---:|---:|---:|
| Translation (mm) | 2.893 | 3.764 | 5.055 | 4.818 | 11.791 | 7.996 |
| Rotation (deg) | 0.1091 | 0.1077 | 0.1760 | 0.1663 | 0.3407 | 0.3365 |

Later revisions reduce the large transient translation discrepancies and substantially change ON rotation. These revisions were recorded for diagnosis, not retroactively available at first publication. The last 25 edges in each capture remain active at stopping time: their last observed revisions are **right-censored**, not guaranteed final lifetime versions.

## 6. Time-synchronized plots and scene context

[Health and relative-belief error timeline](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/health_vs_belief_error.png)

![Fresh ETH1 health and relative-belief discrepancies](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/health_vs_belief_error.png)

[Camera context and matched diagnostics](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/camera_context.png)

![Selected camera context with OFF and ON diagnostics](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/camera_context.png)

All six camera images match the selected integer endpoint stamps exactly. They are recorded front-left images, not feature overlays. Late examples show a plaza/walkway, broad paving and bright sky; early examples include nearby pedestrians and buildings. This supplies location/appearance context but does not prove that glare, texture, dynamic objects, calibration or blur caused the stereo failures. Selection deliberately includes large errors, weak-stereo cases and early context; it is not a representative scene-performance sample.

## 7. Validation, reproducibility and limits

| Check | OFF | ON |
|---|---:|---:|
| Completion marker | Present | Present |
| Native GLIM poses | 1,191 | 1,191 |
| Native Kimera poses | 584 | 584 |
| Native Kimera coverage | 119.599328 s | 119.599328 s |
| Actual Kimera outgoing beliefs | 583 | 583 |
| Actual GLIM outgoing messages in recorded Kimera input topic | 1,189 | 1,189 |
| Applied G→K / K→G CBS factors | 0 / 0 | 580 / 580 |
| Fresh tracked-frame rows | 1,196 | 1,196 |
| Fresh final-status rows | 583 | 583 |
| Fresh mono + stereo match rows | 1,166 | 1,166 |
| All stage rows | 3,528 | 3,528 |
| Log-only revision rows | 14,250 | 14,250 |
| Identical copied profiles to frozen ETH1 | Yes | Yes |
| Runtime binaries unchanged during capture | Yes | Yes |

The per-frame invariants check valid counts against array size, depth against valid tracks, grid totals, inliers against candidates, and replenishment against array-size increase. Every included first belief was produced at its endpoint backend update. Both timestamps, source index bindings and raw ROS payloads were validated. No fatal or queue-drop warning was found in estimator logs. The final process check found no running estimator/fuser—only the container's idle process.

**Historical comparison:** new OFF native Kimera CSV is byte-identical to the frozen September-30 OFF CSV (SHA-256 `ef7f5ef2f516f80aa9a48068aea710b02b6b267f9b74ba6ace25f3b2350a7c80`). New ON is **not** byte-identical to the old ON CSV. We have not isolated that ON difference or certified event-for-event repeatability of the coupled source system. Passive logging and removing receiver load can affect execution timing even without changing mathematics. No timing cause is claimed as established. Accordingly, all ON health/error conclusions use **new ON health with new ON beliefs**, never old ON errors with new counters. This diagnostic does not overwrite or re-score the old fused result.

The Kimera library build and capture/data assertions passed. Both capture validators were run again after analysis and passed. Repeating the read-only analysis produced byte-identical `summary.json`, `joined_intervals.json` and `health_vs_belief_error.png`; this verifies analysis repeatability, not estimator repeatability. A full unit-test/sanitizer suite was not run for this logging-only experiment, and estimator determinism was not claimed from repeated ON captures. No extra estimator rerun was performed after seeing these findings. Existing dirty repositories were preserved; no clean-worktree claim or commit was made. The report repository HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

### Provenance

Input bag SHA-256s, profile manifests, runtime/source identities and dirty-diff fingerprints are retained in [provenance/](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/provenance). In particular:

- [Manifest](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/provenance/manifest.json)
- [Raw bag SHA-256s](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/provenance/raw_bags.sha256)
- [OFF validation](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/validation_off.json) / [ON validation](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/validation_on.json)
- [Complete final statistics](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/summary.json) / [per-interval evidence](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/analysis/joined_intervals.json)
- [Analysis protocol](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/ANALYSIS_PROTOCOL.md)

Use the unsuffixed final `summary.json` / `joined_intervals.json`; suffixed files are retained intermediate progress snapshots.

Source HEADs: Kimera-VIO `e1148f64077fdf2a427256fb5779973fc8c4a0f3`; Kimera-VIO-ROS `c28b0ed9d2a279dc2b3b70828d6d3b1d2054a5bb`; cbsms `db9ddcaa47c419ca59987746ff8ddd37410a23d3`; cbs `eb72f0cf617bd0ea8f01333d26573e23aec4699d`; glim_ros1 `8ce5c09df95e5876599c9723c34a8488bea3af3d`. These HEADs alone do not describe the dirty source trees; consult the saved manifest and logging-only before/after copies. Runtime `libkimera_vio.so` SHA-256: `8bd97e0501a71a8486e89d4c7282def23e9f41d2f8e94f2be3758312d06642f1`.

## 8. Commands used

Container: `cbsms_ws_gpu`, image `cbsms:noetic-cuda-glim`; host V4RL tree mounted at `/workspace`.

Build inside the container (then the rebuilt library was installed to the existing devel runtime location, preserving the prior library in `provenance/libkimera_vio.before.so`):

```bash
source /opt/ros/noetic/setup.bash
source /workspace/cbs_gtsam4.3/devel/setup.bash
cmake --build /workspace/cbs_gtsam4.3/build/kimera_vio --target kimera_vio -- -j2
```

Sequential captures, already completed; do not rerun into these frozen output directories:

```bash
docker exec cbsms_ws_gpu bash /workspace/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/run_mode.sh off
docker exec cbsms_ws_gpu bash /workspace/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005/run_mode.sh on
```

Host analysis using the existing scientific Python environment:

```bash
cd /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_kimera_visual_health_20261005
/home/yeranis/.local/share/pipx/venvs/evo/bin/python validate_capture.py off
/home/yeranis/.local/share/pipx/venvs/evo/bin/python validate_capture.py on
/home/yeranis/.local/share/pipx/venvs/evo/bin/python analyze_health.py --mode both
/home/yeranis/.local/share/pipx/venvs/evo/bin/python camera_context.py
```

## Conclusion

Kimera exposes a useful **usable-stereo-support warning**, especially for some late translation-error bursts. Raw feature count is insufficient, and neither VALID status nor a high inlier fraction guarantees an accurate outgoing relative transform. The warning also leaves major rotational discrepancies unexplained. This is evidence for a diagnostic signal—not yet a scientifically validated health-to-covariance mapping. No covariance, fusion or CBS weighting change was implemented.
