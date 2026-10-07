# SPX-3 matched 120 s live CBS-OFF/ON — headless source and receiver

Date: 2026-09-29. This is the successful, separate headless rerun after the [earlier inspector-stalled attempt](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_LIVE120_OFF_ON_ATTEMPT_20260929.md). The result is a **live parallel receiver**, not a post-run re-injection of completed source trajectories. The receiver started before playback and processed diagnostic source revisions while GLIM and Kimera were running. gnssFGO is downstream only: it does not feed factors back into GLIM or Kimera. In CBS-ON, GLIM and Kimera exchanged classic CBS beliefs bidirectionally during playback.

## Frozen setup and computational issue

GrandTour SPX-3 mission `2024-11-02-17-43-10`, bag seconds `[32,152]`, 120 s at `0.10×`, seed clock `1727775007`. GLIM used Hesai + CPT7 IMU, its saved 5 s smoother profile; Kimera used front stereo + the same CPT7 and saved 25-state profile. Both used `cpt7_imu`, 0.20 s CBS horizon, one pair/update, odometry stride 1, dense `schur_relative_between` relative covariance, and the named `classic` source protocol. OFF disabled source-to-source reception; ON enabled bidirectional reception with zero start delay. Source-to-source `live_revision` was **not** enabled. Full smoother revisions were logged strictly for the downstream gnssFGO receiver.

The matched receiver JSONs are [OFF](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_off_live120_headless_20260929.json) and [ON](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_cbs_on_live120_headless_20260929.json). Their only differences are the sequence/session labels. The receiver used its independent 100 ms IMU support grid, ALL_RIGHT CPT7 integration, Combined IMU, WNOJ `Qc=10000`, full-history iSAM2, stable-ID source-factor replacement, source covariance scale 1, and no GNSS, gating, health weighting, or robust loss. Receiver executable SHA-256: `e5af1ade5f72ed1e05fb4505613840eb20e7b2b76623384605db6386bd390c2c`.

The old runs had complete IMU/receiver support but GLIM stopped at ~80–89 s. Their final `GLIM_FACTOR_GRAPH_INSPECTOR_ROW` was followed by no `on_smoother_update_finish` for that update, consistent with the inspector callback blocking. The 0.05× diagnostic did not remove the stall. For this rerun only the visualization work on estimator threads was disabled: `CBSMS_PRESENTATION_ENABLE=false`, `CBSMS_FACTOR_GRAPH_ENABLE=false`, and receiver `--graph-snapshots` was omitted. The required diagnostic full-revision export remained enabled (`CBSMS_EXPORT_ENDPOINT_AUDIT_ENABLE=true`, `KIMERA_CBS_REVISION_AUDIT=1`). With the inspector disabled, **both GLIM runs reached the full 120 s and completed**. This strongly implicates synchronous inspector logging as the earlier operational bottleneck; it is not proof of one precise internal blocking call. No GLIM, Kimera, CBS, or gnssFGO estimator mathematics was changed.

The copied GLIM and Kimera config trees compare byte-for-byte equal between OFF and ON, and to the frozen September-27 copied profiles by recursive file comparison. Resolved parameters confirm `glim_cbs_live_revision_enable=false` and both dedicated graph inspectors off; ON has `glim_cbs_receive_external_beliefs=true`. The [canonical classic-CBS runbook](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md) remains the setting source for future sequences.

## Execution and live-accounting evidence

For each mode, a fresh ROS master at port 11368 was started with `/use_sim_time=true`; the existing receiver launcher was prechecked and started **before** the source runner. Its `transport_ready.marker` was present before source playback. The source used `run_grandtour_eth1_glim_kimera.sh` with the explicit values above and `CBSMS_USE_EXISTING_ROS_MASTER=true`. After the source runner exited and its `complete.marker` appeared, the receiver was ended using its documented `STOP` marker. It drained and wrote `complete.marker`; the temporary ROS master was stopped after that. The exact source settings, copied profiles, resolved ROS parameters, original bag, ROS logs, receiver journal, online/final trajectories, and complete transport summaries are retained under:

- [CBS-OFF run](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_off_live120_headless_20260929/)
- [Classic CBS-ON run](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_cbs_on_live120_headless_20260929/)

| Check | OFF | ON |
|---|---:|---:|
| Native GLIM poses | 1,192 | 1,192 |
| Native Kimera poses | 597 | 598 |
| Source G→K injected | 0 | 591 |
| Source K→G injected | 0 | 593 |
| Raw CPT7 samples received by fuser | 12,000 | 12,000 |
| Fused support poses, online and final | 1,195 | 1,195 |
| Revision messages received | 70,676 | 71,130 |
| Unique source-belief identities accepted | 1,781 | 1,783 |
| Revision insertions / removals | 70,626 / 68,845 | 71,080 / 69,297 |
| Revisions skipped outside available support | 50 | 50 |
| Pending events at completion | 0 | 0 |
| Maximum transport queue | 576 | 574 |
| Total receiver wall time | 1,250 s | 1,245 s |

All four fused TUM files use the identical 1,195-timestamp support grid, from `1730565903.454194069` to `1730566022.854193926`. Native GLIM and Kimera both cover the requested window. ON's source metadata reports one last G→K receiver-tail edge not applied before the run boundary (`g2k_applied_tail_gap=1`); it is **not** silently counted as an applied factor.

The receiver's saved receipt journal and progress records show concurrent, advancing factor handling. As a conservative receipt-to-next-progress wall-time bound, matching each belief receipt event to the next receiver progress record gave:

| Mode/source | Median | p95 | Maximum |
|---|---:|---:|---:|
| OFF GLIM revisions | 0.122 s | 0.830 s | 8.983 s |
| OFF Kimera revisions | 0.075 s | 0.515 s | 8.006 s |
| ON GLIM revisions | 0.161 s | 2.789 s | 8.764 s |
| ON Kimera revisions | 0.102 s | 2.498 s | 8.674 s |

These are **wall-clock** bounds at 0.10× playback, not measurement-to-arrival latency or a 1×/0.3× throughput guarantee. A progress event can represent pending scheduling before the final factor optimization. All queues drained by completion. The native estimators and receiver ran concurrently; no GUI/Rerun rendering was in their critical path.

## Matched evaluation

The [saved scoring script](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_live120_headless_evaluation_20260929/score.sh) uses the existing `score_grandtour_prism.py` and 1 s `score_grandtour_relative_pose.py`, with the **same** absolute window `1730565903.588829994`–`1730566022.688653946` for all outputs. It uses SE(3) Kabsch alignment without scale and the existing CPT7→AP20 prism lever arm. The prism GT covers ~94.8 s in five matched segments of this ~119.1 s estimate window; the IE-TC relative reference supports 1,181 valid 1 s pairs for every case. ATE is prism-based, RPE is IE-TC-based. Native source trajectories are the original GLIM `odom_imu.txt` and Kimera ROS odometry export, **not** their gnssFGO-only reconstructions. Fused online means the receiver's as-played estimate; fused final incorporates later revisions retrospectively and is noncausal as a time-series product.

| Mode | Trajectory | ATE RMSE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) | Prism-matched path (m) |
|---|---|---:|---:|---:|---:|---:|
| OFF | Native GLIM | 0.007046 | 0.007028 | 0.275979 | 78.589 | 71.795 |
| OFF | Native Kimera | 0.655087 | 0.176565 | 0.614367 | 77.070 | 69.628 |
| OFF | **Live fused** | **0.069367** | **0.027762** | **1.750184** | **79.123** | **72.239** |
| OFF | Fused final revised | 0.047570 | 0.011439 | 0.380927 | 78.283 | 71.645 |
| ON | Native GLIM | 0.015811 | 0.022587 | 0.574259 | 79.132 | 72.269 |
| ON | Native Kimera | 0.170926 | 0.030378 | 0.535886 | 78.576 | 71.721 |
| ON | **Live fused** | **0.078541** | **0.047590** | **1.911003** | **81.866** | **74.801** |
| ON | Fused final revised | 0.066802 | 0.012325 | 0.651236 | 78.465 | 71.831 |

The native Kimera ATE improves sharply under CBS-ON (0.6551→0.1709 m), while native GLIM becomes less accurate (0.0070→0.0158 m) but remains strong. The **live** fused ATE worsens from 0.0694 to 0.0785 m; its 1 s translational and rotational RPE also worsen. The **retrospectively revised** fusion worsens from 0.0476 to 0.0668 m ATE. Thus the source-side Kimera benefit does not translate into a fused benefit in this matched window. The table alone does not identify why; the source belief/covariance competition and receiver dynamics would require a separate frozen-factor diagnosis. No weight, covariance, timing, or CBS settings were adjusted after seeing these scores.

## Rerun and visualization limitation

The verified, post-run recordings are [OFF trajectories + completed GLIM submap](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_live120_headless_evaluation_20260929/visualization/SPX3_CLASSIC_OFF_LIVE120_HEADLESS.rrd) and [ON trajectories + completed GLIM submap](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_live120_headless_evaluation_20260929/visualization/SPX3_CLASSIC_ON_LIVE120_HEADLESS.rrd). The blue GT comes from a retained full-134-second prism export for this same mission; **the numerical scoring reads the original reference bag directly**. The green native GLIM, orange native Kimera, and red **online** fusion paths grow with the physical timeline; a separate panel compares online fusion with its purple retrospectively revised final path. Both `.rrd` files pass `rerun rrd verify`. The GLIM dump contains just one completed submap for this window, so its gray cloud appears only at the end; it is not represented as a continuously arriving point-cloud scan stream.

There are **no actual 120 s GLIM/Kimera active factor-graph snapshots** in these headless runs. Disabling the synchronous inspector was the operational change needed to avoid the earlier source stall, and trajectories cannot reconstruct those historical active graph states. The older 60 s graph recording is a different run and must not be relabeled as this result. The [post-run visualization script](/home/yeranis/repos/gnssFGO/tools/spx3_live120_headless_trajectory_rerun.py) explicitly labels this limitation. This is the remaining visualization gap, not a failure of live fusion/trajectory capture.

Open either recording with `rerun --new --memory-limit 8GB /absolute/path/to/the-linked-file.rrd`; the ON recording was also launched in the local desktop viewer after validation.

## Reproducibility and boundaries

No estimator implementation or frozen source profile was modified for this experiment. New task files are the two run-specific receiver configs, the visualization-only Rerun script, this report, and run/evaluation artifacts. Existing dirty worktree changes were preserved. At reporting time, gnssFGO HEAD was `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; cbsms HEAD was `db9ddcaa47c419ca59987746ff8ddd37410a23d3`. The run was at 0.10× and does **not** establish 1× live capacity or recommend 0.3× without a separate matched load test.

**VALID_LIVE_120S_COMPARISON; CBS_ON_FUSION_WORSE_ON_THIS_WINDOW**
