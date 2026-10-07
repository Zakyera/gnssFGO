# ETH-1 frozen-journal GLIM-only and Kimera-only receiver controls

## Question and scope

Does the frozen gnssFGO receiver preserve each source's CBS-ON improvement when
only that source contributes relative-pose factors? This is the same first
120 s ETH-1 (`2024-10-01-11-29-55`) experiment as the [original live OFF/ON
run](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md).
The four controls here are **offline replays of its recorded, causal arrival
journals**, not new native GLIM/Kimera runs and not a new co-running live
receiver. No estimator, receiver mathematics, covariance, or setting was
changed.

The previous [factor-interaction audit](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_COVARIANCE_AND_FACTOR_INTERACTION_AUDIT.md)
motivated isolating the sources before attributing the joint loss to any
particular covariance mechanism. Original source runs and all 76 checked
frozen inputs remain unchanged (`verify_frozen_inputs.py`: 76/76 equal).

## Exact control construction

For each OFF or ON mode, retain every raw IMU (`I`) event, its recorded
availability, and the end (`E`) event from the original receiver journal.
Retain byte-for-byte only GLIM `B` revision events for the GLIM-only control,
or only Kimera `B` revision events for the Kimera-only control. The relative
means, full 6×6 covariances, stable IDs, revisions, physical endpoints,
availability timestamps, and within-source order are untouched. The OFF and
ON journals are independent recorded live arrivals; their physical IMU
samples match, but their availability timestamps need not be byte-identical.

| Frozen mode | IMU events | GLIM revision events | Kimera revision events | Unique accepted factors |
|---|---:|---:|---:|---:|
| OFF, GLIM-only | 11,986 | 56,461 | 0 | 1,189 GLIM |
| OFF, Kimera-only | 11,986 | 0 | 14,250 | 581 Kimera |
| ON, GLIM-only | 11,986 | 56,498 | 0 | 1,189 GLIM |
| ON, Kimera-only | 11,986 | 0 | 14,250 | 581 Kimera |

The frozen receiver executable is
`/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay`,
SHA-256 `2f188a661e87a63092271b2b325bed0fa3ab0471951f596c86a7a45f8ebf4530`.
Inside the existing `ubuntu-jammy-humble-nvidia` container, after sourcing
`/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`, each
filtered journal was replayed with:

```bash
grandtour_ct_replay --event-stream INPUT.csv --out OUTPUT_DIR \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59
```

The graph retains the same full-history iSAM2, 100 ms source-independent
support, 59-sample initialization, ALL_RIGHT CPT7 Combined IMU, WNOJ
`Qc=10000`, priors, interpolation, source covariance scale 1, and stable-ID
revision replacement. There is no GNSS, added source-model uncertainty,
gating, health weighting, or robust loss. All controls report
`source_model_noise: {}`, 1,193 support states, zero pending factors, and the
same mode-specific support clock as the original joint receiver. The 25
early Kimera revisions lacking support are skipped in both Kimera-only
controls, as in the original joint receiver; GLIM-only skips none.

The source-only **ON** beliefs were produced by native estimators with classic
bidirectional CBS active. In particular, “Kimera-only” here means that the
downstream gnssFGO graph receives only Kimera factors; it does **not** mean
that Kimera's source estimate is independent of GLIM's CBS information.

## Evaluation and results

Scoring uses precisely the original requested GPS window
`1727775008.775065899`–`1727775127.674791098`, CPT7-to-AP20 lever-arm
correction, no-scale SE(3) alignment for prism ATE, and IE-TC 1 s relative
pose RPE. All controls and the original joint receiver share **exactly the
same 654 matched prism timestamps**, 64.9 s aggregate coverage in five
segments, 47.790 m matched reference path, and 1,179 RPE pairs. The prism
does not cover the full 120 s continuously. The *full path* column is the
estimate over the requested window, not the 47.790 m prism path.

### Receiver online/as-played trajectory

These trajectories follow what the causal receiver knew as the recorded
arrivals occurred. “Joint” is the already-frozen original run, not a fifth
experiment performed here.

| Mode | gnssFGO factors | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---:|---:|---:|---:|
| OFF | GLIM-only | 0.049909 | 0.044001 | 1.104008 | 73.334 |
| OFF | Kimera-only | 0.269584 | 0.147258 | 0.337717 | 87.336 |
| OFF | Joint GLIM+Kimera | 0.051511 | 0.031413 | 0.575945 | 72.336 |
| ON | GLIM-only | 0.068069 | 0.065721 | 1.070211 | 80.059 |
| ON | Kimera-only | 0.194242 | 0.045085 | 0.675815 | 74.730 |
| ON | Joint GLIM+Kimera | 0.122472 | 0.054332 | 0.942747 | 76.687 |

### Receiver final retrospectively revised trajectory

This is the graph after all recorded revisions have arrived; it is **not**
the online trajectory.

| Mode | gnssFGO factors | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---:|---:|---:|---:|
| OFF | GLIM-only | 0.035142 | 0.021627 | 0.388291 | 72.158 |
| OFF | Kimera-only | 0.210687 | 0.022181 | 0.262138 | 71.235 |
| OFF | Joint GLIM+Kimera | 0.048359 | 0.015597 | 0.414167 | 71.824 |
| ON | GLIM-only | 0.036467 | 0.021920 | 0.641766 | 72.011 |
| ON | Kimera-only | 0.197527 | 0.019497 | 0.339661 | 71.925 |
| ON | Joint GLIM+Kimera | 0.107042 | 0.018430 | 0.515790 | 71.928 |

For context, the **native estimator outputs** from the original live run
are different products, not the single-source gnssFGO graphs:

| Native source | OFF ATE / t-RPE / r-RPE | ON ATE / t-RPE / r-RPE | Prism matches |
|---|---|---|---:|
| GLIM | 0.038007 m / 0.027251 m / 0.351298° | 0.028978 m / 0.033387 m / 0.429674° | 655 |
| Kimera ROS odometry | 0.541016 m / 0.045950 m / 0.541482° | 0.365236 m / 0.030284 m / 0.571374° | 315 |

The native ATE masks differ from the receiver mask, especially for Kimera;
native-vs-receiver ATE is therefore **not** an identical-sample preservation
metric. OFF-vs-ON within each receiver-control row *is* identical-mask.

## What the controls establish

1. **The receiver does respond to Kimera's CBS-ON change.** Kimera-only
   online ATE improves from 0.269584 to 0.194242 m (−27.9%) and 1 s
   t-RPE from 0.147258 to 0.045085 m. Final ATE improves more modestly,
   0.210687 to 0.197527 m (−6.2%). However r-RPE gets worse in both
   products (online 0.337717° to 0.675815°; final 0.262138° to
   0.339661°). Thus the Kimera-only response is not uniformly beneficial.
2. **The receiver's GLIM-only control does not preserve the native GLIM ATE
   gain under CBS-ON.** Native GLIM ATE improves 0.038007 to 0.028978 m,
   whereas GLIM-only receiver ATE worsens online 0.049909 to 0.068069 m
   (+36.4%) and final 0.035142 to 0.036467 m (+3.8%). These are separate
   products; the within-receiver OFF/ON comparison is the controlled one.
3. **The joint ON graph has an additional interaction loss.** ON joint ATE
   0.122472 m online / 0.107042 m final is worse than ON GLIM-only
   0.068069 / 0.036467 m. This cannot be described solely as the receiver
   failing to preserve GLIM's native improvement. The source-only and joint
   graphs also differ at common physical support times: ON GLIM-only vs
   joint position RMS is 0.198 m over all 1,193 as-played support poses
   (OFF: 0.099 m). This is a graph-to-graph difference, not ATE.
4. **Do not overstate the loss:** the joint *final* graphs improve 1 s
   translational RPE over their corresponding GLIM-only final controls
   (OFF 0.021627 to 0.015597 m; ON 0.021920 to 0.018430 m), despite worse
   ATE. The mechanisms and merits are metric-dependent.

These controls rule out a simple story that the receiver entirely ignores
the sources and only follows IMU/WNOJ. They do **not** identify a unique
cause for the ON joint loss, establish whether source cross-correlation or
calibration is responsible, or justify altering any covariance. A native
absolute-pose ATE improvement does not guarantee that its exported relative
factors are independently calibrated to improve a second optimizer.

## Validation, artifacts, and repository state

The [artifact directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_source_preservation_controls_20260930/)
contains the four filtered input journals and input hashes
([`input_manifest.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_source_preservation_controls_20260930/input_manifest.json)),
full receiver trajectories and summaries, the evaluator outputs and
[`control_metrics.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_source_preservation_controls_20260930/control_metrics.json),
[`validation.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_source_preservation_controls_20260930/validation.json),
and common-support graph comparisons
([`path_differences.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_source_preservation_controls_20260930/path_differences.json)).
The scripts in that directory build the subsets, score with the original
evaluators, and verify immutable inputs and matching masks. Each selected
belief line and all non-belief lines are byte-identical to the original
mode-specific journal; all original journal SHA-256 values match the
manifest. Each control has the expected single source ID and accepted
factor count. Original frozen-input verification passed for 76 files.

gnssFGO HEAD is `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
There were pre-existing uncommitted changes in that repository; this task
did not edit production code or the original experiment artifacts. No
source run, new joint run, weighting change, or tuning was performed.
