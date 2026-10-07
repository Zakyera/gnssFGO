# GrandTour classic-CBS live gnssFGO baseline — freeze for the next sequence

Established 2026-09-29 from the [matched SPX-3 120 s live CBS-OFF/ON experiment](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_LIVE120_HEADLESS_OFF_ON_20260929.md). This is an **experimental configuration and reproducibility contract**, not an assertion that the current fusion is optimal. The SPX-3 native Kimera result improved under CBS-ON while its live fused result worsened; preserve this baseline when testing a new sequence so a setting change is not confused with a sequence effect. Do not tune source/fuser covariance, CBS policy, WNOJ, IMU, or graph topology in a purported matched cross-sequence comparison.

Read the [native classic-CBS runbook](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md) and [live receiver runbook](/home/yeranis/repos/gnssFGO/GNSSFGO_GRANDTOUR_LIVE_RECEIVER_RUNBOOK.md) before launching. The saved [SPX-3 OFF receiver JSON](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_off_live120_headless_20260929.json), [ON receiver JSON](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_on_live120_headless_20260929.json), [OFF run](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live120_headless_20260929/), and [ON run](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live120_headless_20260929/) are the **reference artifacts**. Never overwrite them.

## Freeze these shared algorithmic rules

| Layer | Rule for a matched GrandTour comparison |
|---|---|
| Native source CBS | `CBSMS_CBS_PROTOCOL=classic`; `CBSMS_CBS_ENABLE=false` for OFF, `true` plus bidirectional receiving for ON. Both publish in both modes. Never substitute experimental source-to-source `live_revision`. |
| Native belief | Nominal 0.20 s `time_horizon_window`, one pair/update and odometry stride 1 in the validated SPX-3 profile; persistent external relative-pose factors; `schur_relative_between` full dense 6×6 source covariance, scales 1, no source health weighting. Confirm the corresponding effective values in each **new sequence's** saved source profile. |
| Downstream beliefs | Export full current-smoother GLIM and Kimera means/covariances with stable source IDs and availability order. gnssFGO removes the previous factor version and inserts the newly observed version; this **receiver revision replacement** is separate from source-to-source classic CBS. No duplicated belief consumption. |
| Receiver support/timing | Independent central CPT7 IMU-driven 100 ms support clock, anchored after the existing 59-sample initialization; exact fractional ALL_RIGHT CPT7 integration. Beliefs keep their physical endpoints and causal observed arrival times. No GLIM-selected support grid, source pairing, or endpoint snapping. Review the 30 ms IMU-gap guard for a changed IMU. |
| Receiver graph | Existing Combined IMU + WNOJ `Qc` variance `10000`, existing priors/initialization, full-history iSAM2, interpolated relative factors, full source covariances at scale 1. No GNSS factors, NIS gating, robust kernel, adaptive weighting, or covariance tuning. Full history has unbounded long-run memory; do not claim arbitrary-duration or 1× capacity. |
| Primary outputs | Original/native GLIM odometry; original/native Kimera ROS odometry; gnssFGO **online/as-played** fused trajectory. Keep gnssFGO's retrospectively revised final trajectory separate and labeled noncausal. Receiver GLIM-only/Kimera-only controls are not native source trajectories. |
| Evaluation | The same GT source, physical overlap, alignment, lever arm, and 1 s RPE reference for every OFF/ON output; report native versus receiver sampling/coverage. Do not select source belief policies by ATE alone. |

For the validated shared-CPT7 SPX-3 source profile, native Kimera used its saved `KIMERA_IMU_SAMPLE_SUPPORT=current` packet path (also selected when unset), whereas the **gnssFGO receiver** used ALL_RIGHT. Do not silently set Kimera to the earlier experimental RIGHT path to make the labels match. A different GrandTour sensor/IMU profile requires its own effective-path verification.

## Freeze this operational live-capture protocol

1. Use a fresh, isolated ROS master and simulated clock for each mode. Start and preflight the receiver **before** launching sources/bag playback; wait for `transport_ready.marker`. `CBSMS_USE_EXISTING_ROS_MASTER=true` and the receiver's port must agree. OFF and ON must use different fresh run directories and source sessions.
2. Preserve the diagnostic exports feeding the current live adapter: `CBSMS_EXPORT_ENDPOINT_AUDIT_ENABLE=true`, `KIMERA_CBS_REVISION_AUDIT=1`, `CBSMS_DIAGNOSTIC_NODE_OUTPUT=log`, and `ROS_LOG_DIR` inside the fresh source run. The receiver currently tails completed GLIM/Kimera diagnostic lines. It does not subscribe to native belief ROS messages or send anything back to the sources; observed log-line time is the receiver availability time.
3. For this **validated 120 s headless operating point**, set `CBSMS_PRESENTATION_ENABLE=false`, `CBSMS_FACTOR_GRAPH_ENABLE=false`, and omit receiver `--graph-snapshots`. This removes the synchronous source graph inspector from estimator callbacks while retaining native odometry and downstream full-revision diagnostics. The prior inspector-enabled 120 s captures stalled GLIM around 80–89 s; a 0.05× diagnostic did not resolve it. Do not enable graph rendering in the critical path and still call it the same operating point.
4. Run matched OFF and ON with the **same** bag interval, playback rate, source profile trees, receiver model, logging load, evaluation mask and software binaries. In SPX-3 the complete profiles were byte-identical between modes; the receiver JSONs differed only in labels/session. CBS mode is the intended experimental difference.
5. End only after the source runner has actually completed and its logs have drained. Send the receiver's `STOP` marker, then require its `complete.marker`, final summary and `pending_at_end=0`. Confirm native GLIM and Kimera both cover the requested window, both source receipt counters advance during playback, online fused poses are published throughout, and all inserts/replacements/skips are accounted for. A source `complete.marker` **alone** did not detect the earlier GLIM tail stall.
6. Score the saved native and fused paths after capture using the existing GrandTour evaluators. Render saved trajectories after playback. This headless setting does **not** save true historical GLIM/Kimera active-graph snapshots; do not reconstruct them from trajectories and label them real. The SPX-3 Rerun files show time-growing trajectories and a completed GLIM submap, not an individual live scan stream or live source factor graphs.

The successful SPX-3 point used `0.10×` playback and required about 1,245–1,250 s receiver wall time for the 120 s bag window. It proves concurrent processing at **that** load, not 0.3×, 0.5×, or real-time 1×. For another sequence, begin with a bounded validated window and measure native-source progress, receiver event/progress latency, queue growth, output age, memory, and end drain. Playback rate is a **sequence/load test setting**, not a universal CBS/fusion parameter; hold it equal between modes.

## SPX-3 reference values — do not copy as universal defaults

| SPX-3-specific item | Saved 120 s value |
|---|---|
| Mission and bag window | `2024-11-02-17-43-10`, GrandTour `SPX-3`, offset 32 s, duration 120 s |
| Input sensors | GLIM Hesai `/boxi/hesai/points` + `/boxi/cpt7/imu`; Kimera front Alphasense stereo + shared CPT7; common output body `cpt7_imu` |
| Source lag/profile | GLIM saved 5 s smoother; Kimera saved front-stereo 25-state profile and its full camera/IMU calibration/flags |
| Source trees | September-27 `spx3_off_classic_revisions_nodelog/{glim_config,kimera_profile}` under `/workspace/cbs_gtsam4.3/runs/spx3_full_system_revision_diagnostics_20260927/`; copied OFF and ON trees matched those files byte-for-byte |
| Other source values | `CBSMS_KIMERA_MODE=stereo_shared_cpt7`, `CBSMS_CBS_BELIEF_RECEIVE_START_DELAY_SEC=0`, 0.20 s horizon, one pair/update, stride 1 |
| Playback/clock | `CBSMS_BAG_RATE=0.10`, seed clock `1727775007`, 25 s post-playback drain |
| Receiver registry | IDs `glim_hesai` and `kimera_front`, each with a run-unique session; physical beliefs in `cpt7_imu`; GLIM `ros_logs/*/glim_rosnode-*-stdout.log`, Kimera `estimators.log` |
| Receiver transport limits | Support 100,000,000 ns; startup 59 samples; max IMU gap 30,000,000 ns; max queue 200,000; drain timeout 300 wall seconds |
| GT/scoring | AP20 prism ATE with CPT7→prism lever arm and SE(3) no-scale alignment; IE-TC 1 s relative-pose error; shared matched window `1730565903.588829994`–`1730566022.688653946` |

These are SPX-3 reference values. For ETH1, EIG-2, or another GrandTour sequence, explicitly choose and save the correct mission/bags, LiDAR/camera topics, extrinsics/intrinsics, IMU/clock frame, Kimera/GLIM profiles and lag, temporal window, playback rate and GT coverage. If a different sequence cannot provide the same calibrated central body frame or a reliable common evaluation, stop and report it; do not silently reuse SPX-3 calibration or a receiver JSON with mislabeled source IDs.

## Minimum future-run record

Save the exact effective source environment, copied profile manifests/hashes, resolved ROS parameters and Kimera flags, bag identities, receiver JSON, executable/git hashes and dirty state, source/receiver completion markers, native source coverage, injected CBS factor counts, receiver source accounting/revision ledger, online/final TUM hashes, common GT interval and ATE/RPE/path metrics. State explicitly if graph snapshots were disabled. Record every unsupported factor and any tail gap; do not call zero pending events proof that every published belief was accepted.

This document **does not authorize** starting another sequence, adding a source, enabling GNSS, changing CBS, or tuning fusion. It is the frozen configuration reference to apply when a future experiment is authorized.
