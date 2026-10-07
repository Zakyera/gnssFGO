# ETH-1 live classic-CBS comparison, zero added source uncertainty

## Scope and provenance

This is a matched **first 120 s** GrandTour ETH-1 experiment, mission
`2024-10-01-11-29-55`, starting at bag offset 0 and played at `0.10×`.
It is not a full-sequence result. GLIM, Kimera, and the gnssFGO receiver ran
concurrently. The receiver consumed observed, causally available revision logs
while the source estimators ran; it did not ingest completed trajectory files.
The native GLIM and native Kimera trajectories below are their original source
outputs, not gnssFGO single-source reconstructions.

The run follows the [classic CBS runbook](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md)
and [live baseline](/home/yeranis/repos/gnssFGO/GNSSFGO_GRANDTOUR_CLASSIC_LIVE_BASELINE_20260929.md).
The complete source and receiver artifacts are in
[`gnssfgo_eth1_live120_zero_20260930`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/).
Source repository HEAD: `db9ddcaa47c419ca59987746ff8ddd37410a23d3`;
gnssFGO repository HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
These repositories contain pre-existing uncommitted work. No estimator or fusion
source was edited for this experiment; only run-specific configuration,
evaluation artifacts, and this report were created.

## Frozen settings actually used

- Sources: ETH-1 Hesai LiDAR plus CPT7 IMU in GLIM; calibrated front AlphaSense
  stereo plus the same CPT7 in Kimera; common `cpt7_imu` belief frame. Both modes
  used copied ETH-1 saved profile trees from the retained 2026-09-07 successful
  source run. OFF and ON source profile hashes match: GLIM config
  `ab620ebba9fd3cc1ad73e9fe1df8b3e74da9d884447a93b37409798b47c84602`,
  Kimera profile `7e8dbf681d7fec8a609a9595df06ff9af73467c0ded12b32ce966ed345da25ee`.
  Effective source parameters are retained as `resolved_glim_params.yaml`,
  `resolved_kimera_params.yaml`, and `run_metadata.txt` in each source directory.
- Source CBS: `classic`, 0.20 s horizon, one pair/update, odometry stride 1,
  persistent external factors, full `schur_relative_between` 6×6 covariance,
  source covariance scale 1. OFF publishes but does not receive CBS beliefs;
  ON receives bidirectionally. Experimental source `live_revision` is disabled.
  Kimera's historical `current` IMU packet path was left unchanged (the
  `KIMERA_IMU_SAMPLE_SUPPORT` override is unset).
- Receiver: source-independent 100 ms support clock; 59 IMU startup samples;
  exact fractional `ALL_RIGHT` CPT7 integration; Combined IMU, WNOJ variance
  `Qc=10000`, existing priors, full-history iSAM2, interpolated relative
  factors, and stable-ID factor revision replacement. No GNSS, gating, robust
  kernel, health weighting, covariance scale, or **additional source-model
  uncertainty**. Both receiver summaries contain `source_model_noise: {}`.
  The executable SHA-256 is
  `2f188a661e87a63092271b2b325bed0fa3ab0471951f596c86a7a45f8ebf4530`
  in both modes.
- Headless load: no Rerun or source factor-graph inspectors during playback.
  Full revision diagnostics remained enabled. The current adapter observes
  completed diagnostic log lines, so its arrival time includes log transport
  latency; it is not a direct ROS belief subscription.

The exact source environment and mode-specific receiver registries are
[`source.env`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/source.env),
[`receiver_off.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/receiver_off.json),
and [`receiver_on.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/receiver_on.json).
The existing source runner was invoked inside `cbsms_ws_gpu` after setting
`CBSMS_RUN_NAME=off_source`/`on_source`, `CBSMS_CBS_ENABLE=false`/`true`, and
sourcing `source.env`:

```bash
/workspace/cbs_gtsam4.3/src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh
```

For each mode, the receiver was checked and started before its source run with
`tools/launch_grandtour_live_receiver.py --config RECEIVER_JSON --source-run SOURCE_RUN --out RECEIVER_RUN --check` then `--start`, following the linked live runbook. A fresh caller-owned ROS master/simulated clock was used, and the receiver was explicitly stopped and drained after source completion. The exact evaluator invocation is saved in
[`score.sh`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/score.sh).

## Results

The common requested scoring bounds were GPS time
`1727775008.775065899`–`1727775127.674791098` (~118.90 s). Prism ATE used
the existing CPT7-to-AP20 lever arm and SE(3) alignment without scale. The
1 s translational and rotational RPE used the existing IE-TC reference.

| Mode | Actual trajectory | ATE RMSE (m) | 1 s t-RPE RMSE (m) | 1 s r-RPE RMSE (deg) | Full path (m) | Prism matches |
|---|---|---:|---:|---:|---:|---:|
| CBS-OFF | Native GLIM | 0.038007 | 0.027251 | 0.351298 | 72.533 | 655 |
| CBS-OFF | Native Kimera ROS odometry | 0.541016 | 0.045950 | 0.541482 | 69.828 | 315 |
| CBS-OFF | gnssFGO **online/as-played** | 0.051511 | 0.031413 | 0.575945 | 72.336 | 654 |
| CBS-ON | Native GLIM | 0.028978 | 0.033387 | 0.429674 | 73.217 | 655 |
| CBS-ON | Native Kimera ROS odometry | 0.365236 | 0.030284 | 0.571374 | 71.429 | 315 |
| CBS-ON | gnssFGO **online/as-played** | 0.122472 | 0.054332 | 0.942747 | 76.687 | 654 |

The final retrospectively revised gnssFGO graph is **not** its online output:

| Mode | Fused product | ATE RMSE (m) | 1 s t-RPE RMSE (m) | 1 s r-RPE RMSE (deg) | Full path (m) | Prism matches |
|---|---|---:|---:|---:|---:|---:|
| CBS-OFF | Final revised | 0.048359 | 0.015597 | 0.414167 | 71.824 | 654 |
| CBS-ON | Final revised | 0.107042 | 0.018430 | 0.515790 | 71.928 | 654 |

The AP20 prism reference is **not continuous over the full 120 s**. Fused
OFF/ON each have exactly the same 654 matched timestamps spanning 64.9 s of
aggregate matched elapsed coverage in five reference segments (reference
path 47.790 m). Native Kimera is sampled more sparsely (315 prism matches);
its ATE must not be read as if sampled identically to the 654-point fused
product. The RPE reference has 1,179 valid pairs for GLIM and fused, 1,151
for Kimera. The OFF/ON fused support timestamps and matched prism timestamps
are identical. All raw JSON and aligned CSV outputs are under
[`evaluation`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/evaluation/).

## Completion and source accounting

Both source runs have completion markers and cover the requested sensor window:
1,191 GLIM rows and 584 Kimera rows each. OFF injected zero external CBS
factors. ON injected 580 GLIM-to-Kimera and 581 Kimera-to-GLIM factors.
The ON metadata reports one GLIM-to-Kimera applied tail gap; this is retained
as a coverage caveat, not silently rounded away.

Both receiver runs have completion markers after explicit STOP. Each received
11,986 central IMU samples, created 1,193 support states, accepted 1,770 unique
relative-belief IDs (1,189 GLIM + 581 Kimera), and ended with zero pending
beliefs. OFF saw 70,711 revision snapshots, inserted 70,686 and removed
68,916 factor versions; ON saw 70,748, inserted 70,723 and removed 68,953.
Each skipped 25 early snapshots lacking support. Maximum absolute difference
between exported and inserted source covariance entries was
`6.78e-20` OFF and `4.07e-20` ON. No source-model covariance was added.

The complete IMU event rows are not byte-identical because the **arrival-time**
column records independent ROS `/clock` callback jitter. Removing that column,
all 11,986 ordered physical IMU timestamps and six measured motion values are
byte-identical between OFF and ON (SHA-256
`bfb168ddb13aa238b5fb4045a36d76fc09c09260116bda660a1ddddc3625e08d`).
The support **physical** timestamps also match exactly; some recorded
availability timestamps differ by microseconds. This is a live causal
experiment, so observed availability and source revisions can legitimately
differ between modes.

## Interpretation and stop

On this bounded ETH-1 run, classic CBS-ON improves native GLIM ATE and native
Kimera ATE, but **worsens gnssFGO fusion**: online ATE rises from 0.0515 to
0.1225 m, and final revised ATE from 0.0484 to 0.1070 m. The online t-RPE and
r-RPE also worsen. This establishes a reproducible comparison at the frozen
zero-addition operating point; it does **not** identify the factor-level cause
or justify a covariance change on ETH-1. No added-uncertainty variant was run.
