# ETH1 live WNOA / no-central-IMU validation — 2026-10-07

## Status

The matched 30-second native CBS-OFF and classic CBS-ON live checks are complete
and validated. The full 346-second CBS-OFF attempt was **stopped deliberately
after a frozen live-throughput gate failed** on 2026-10-07. Approximately 227
seconds were captured; the receiver drained the partial capture successfully.
The full CBS-ON case was **not launched**. There is no completed full-sequence
OFF/ON result. No settings were changed and no automatic retry was made.

346-second comparison status: **BLOCKED — LIVE_THROUGHPUT_GATE_FAILED**. This
is not a solver crash or a demonstrated mathematical failure of WNOA. The
scheduled full-comparison follow-up is paused; the separately authorized
120-second check below is complete.

**Subsequent authorized result:** the separate **120-second CBS-ON** check at
0.1x in `bounded120_on_*` directories **completed and passed validation**,
including throughput, convergence, source-payload conservation and exact
captured-journal replay. Its common-mask accuracy results are below. This
does not resume or certify the failed 346-second comparison. The original
automatic full-comparison continuation remains paused.

Run root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/`

## Frozen configuration

- ETH1 mission `2024-10-01-11-29-55`, bag offset 0, playback 0.1x.
- Native GLIM: Hesai/CPT7, exact retained ETH1 configuration, 5-second lag.
- Native Kimera: front AlphaSense stereo/shared CPT7, retained 25-state profile.
  The recovered classic source IMU path is `current` (override unset), not
  silently changed to the experimental source RIGHT option.
- Source CBS: classic, persistent factors, 0.20 s horizon, one pair per update,
  publish stride 1, delay 0, `schur_relative_between`, covariance scales 1,
  no health weighting. `live_revision` is disabled for source-to-source CBS.
- Receiver: full-history iSAM2; independent 100 ms support clock; full source
  mean/covariance revisions replace prior versions of each stable BeliefID.
  Availability is when a complete log row is observed in the simulated online
  stream; endpoints keep their physical timestamps and are interpolated.
- WNOA state: pose, world velocity, body angular velocity. Qc diagonal
  `[0.00375,0.00375,0.00375,0.015,0.015,0.015]`. Initial identity pose sigma
  `1e-5`, zero velocity sigma `10`, zero angular rate sigma `3`.
- **No central IMU factors**, acceleration variables, bias variables, or gravity
  initialization. The existing 59-sample timestamp startup supplies the clock
  only; native GLIM/Kimera still use their IMUs.
- Source covariance scale 1, no additions, robust loss, gating, GNSS or tuning.
- Mandatory strict nominal numerical publication checks; diagnostic full-graph
  snapshots and independent LM audit optimizations disabled. Rerun/source graph
  inspectors disabled for throughput. True source graph recordings cannot be
  reconstructed later from trajectory lines alone.

This is live co-running bag playback: both native sources and the receiver
execute together. It is not proof of 1x real-time capacity. Final revised
fusion is retrospective and is kept separate from online/as-played fusion.

## Implementation and preflight

Reused the exact tested isolated executable:
`/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay`

SHA-256: `fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f`.
The retained `gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib`
libraries are selected explicitly. No C++ estimator implementation, Qc,
source profile or production default was modified.

Small transport changes:

1. `tools/live_receiver_protocol.py` accepts explicit `source_wnoa`, requires
   the validated strict/nominal flags, and validates its environment settings.
2. `tools/launch_grandtour_live_receiver.py` passes that explicit environment;
   default `imu_wnoj` behavior remains unchanged.
3. `tools/grandtour_live_receiver_ros1.py` reads pose fields by CSV header rather
   than assuming the IMU model's 18-column bias-state output. WNOA has 15 columns.
4. Existing streaming test CLI selectors accept WNOA; transport tests include
   IMU/WNOJ/WNOA pose schemas, malformed lengths and nonfinite values.

Validation: six Python transport tests passed; five negative streaming cases
passed. An 8-second retained ETH1 journal produced byte-identical file/pipe
outputs, and its online state/trajectory prefix remained byte-identical to the
previously validated diagnostic-enabled WNOA run. Disabling deep audit exports
therefore did not change that tested trajectory. No new C++ build/sanitizer
claim is made: the previously tested executable is unchanged.

Two unsuccessful startup attempts are retained, not overwritten:

- `smoke_off`: process orchestration used `setsid` without `--wait`; it detached
  and falsely appeared finished before sensor playback. Corrected process
  ownership/waiting; no measurements reached the receiver.
- `smoke_off_attempt2`: receiver reached initialization, but the old pose
  publisher rejected WNOA's CSV schema. Stopped early, fixed only the adapter,
  added schema tests, and reran in a fresh directory.

Successful source/receiver directories are `smoke_off_attempt3_*` and
`smoke_on_*`. `protected_manifest.json` and `protected_manifest_v2.json` retain
the before/after adapter hashes; estimator binary, libraries and profiles agree.

## Completed short-run validation

| Check | CBS-OFF | Classic CBS-ON |
|---|---:|---:|
| Native GLIM poses | 291 | 291 |
| Native Kimera ROS-odometry poses | 134 | 134 |
| Receiver support states | 293 | 293 |
| Captured changed belief versions | 15,866 | 15,884 |
| Active source BeliefIDs | 420 | 420 |
| Stable-ID replacements | 15,421 | 15,439 |
| Pending receiver events at end | 0 | 0 |
| Median output age, simulated seconds | 0.060003 | 0.060003 |
| p95 output age, simulated seconds | 0.100001 | 0.100004 |
| Maximum output age, simulated seconds | 0.100005 | 0.103625 |
| Maximum publication gradient | 9.76518e-6 | 9.99503e-6 |
| G→K injected source factors | 0 | 130 |
| K→G injected source factors | 0 | 131 |

All source revision payloads and their per-ID order were conserved exactly.
Each run has 289 active GLIM IDs and 131 active Kimera IDs. Twenty-five versions
of one additional Kimera ID precede the unchanged receiver initialization time
and are explicitly recorded as `before_initialization`; these are not lost
transport messages. Source timestamps do not determine arrival timestamps.

All publications were certified stationary under the existing normalized
gradient tolerance `1e-5`. Each completed live journal was replayed once;
online/final states and trajectories, dispatch, model covariance and measurement
products are byte-identical. No central IMU, bias or acceleration factors/keys
were created. The complete resolved native parameter maps and profile files
match the retained corresponding ETH1 OFF/ON baseline, except output paths and
recording IDs. ON's final G→K applied-index tail gap is one, recorded explicitly.

See `smoke_off_validation.json`, `smoke_on_validation.json`, `preflight/`, and
the per-case `*_control/monitor.jsonl` for detailed evidence.

## Short-window metrics — not the full ETH1 conclusion

Existing evaluators were used unchanged: no-scale SE(3) prism ATE with CPT7/AP20
lever arm, and 1-second IE-TC relative pose error. Original-rate products are
retained in `smoke_evaluation/`. Their prism association coverage differs by
native output rate, so the table below uses `smoke_common_evaluation/` instead.

The common scoring grid uses existing linear position/SLERP interpolation,
0.1 s spacing and maximum 0.5 s estimate gap, intersected across all eight
curves. It has 261 common times, spanning approximately 26 seconds. Only 113
of those times have valid prism matches, covering **11.2 seconds**. All eight
curves use exactly the same valid prism timestamp mask. Resampling is solely
for offline scoring and does not change online publications or native outputs.

| Mode | Evaluated output | ATE m | 1 s t-RPE m | 1 s r-RPE deg |
|---|---|---:|---:|---:|
| OFF | Original/native GLIM | 0.044764 | 0.031596 | 0.473337 |
| OFF | Original/native Kimera | 0.029065 | 0.025955 | 0.615447 |
| OFF | Fused online | 0.025550 | 0.029105 | 1.397913 |
| OFF | Fused final revised | 0.021076 | 0.015427 | 0.431749 |
| ON | Original/native GLIM | 0.040086 | 0.041577 | 0.579600 |
| ON | Original/native Kimera | 0.043489 | 0.039061 | 0.727400 |
| ON | Fused online | 0.052280 | 0.061273 | 1.833606 |
| ON | Fused final revised | 0.045374 | 0.030127 | 0.575818 |

These short, partial-GT values are descriptive only. No covariance, motion
prior, initial condition or source setting was changed after observing them.
They are not a universal CBS-ON performance claim.

## Full OFF throughput failure and controlled stop

The pre-existing `validate_case.py` requires both p95 simulated output age
below 1.0 second and maximum observed writer queue below 1,000 events. Its
comment calls these operational smoke gates, but the unchanged validator
applies them to full cases too. No threshold was added or relaxed in response
to this run.

The first saved monitor sample violating the queue criterion was at
`1727775226573519000` ns, **218.570044 s after the first captured IMU timestamp**
(`1727775008003475000` ns). Its queue was 1,074 and output age 1.590044 s.
The monitor records wall time `1791357400342277027` ns. At the next scheduled
inspection this was a definite acceptance blocker: a maximum-over-run queue
criterion cannot recover later by draining. Both native source receipt counts
were still advancing, and the receiver still emitted stationary solutions.

| Partial OFF evidence | Value |
|---|---:|
| First observed queue-limit breach | 218.570044 s from first IMU |
| First violating queue | 1,074 events |
| Maximum queue observed by controller | 1,517 events |
| Adapter internal maximum queue | 1,569 events |
| Median / p95 / maximum output age during capture | 0.090022 / 1.170043 / 2.330045 s |
| Completed receiver support states | 2,264 |
| Failed publication convergence checks | 0 |
| Maximum recorded normalized publication gradient | 9.955635988889554e-6 (limit 1e-5) |
| Partial receiver support duration | 226.3 s |
| Captured changed belief versions after drain | 135,941 |
| Active relative BeliefIDs / replacements | 3,374 / 132,542 |
| Pending / duplicate / stale revisions after drain | 0 / 0 / 0 |
| Explicit unsupported startup revision rows | 25 |
| Central IMU / bias / acceleration factors or variables | 0 / 0 / 0 |

On detecting the failed gate, SIGINT was sent only to the verified owned host
controller PID 112569 (exact command was `run_case.py full off`). Its cleanup
stopped the owned source group and ROS master and sent the receiver STOP marker.
The receiver drained and exited successfully. Subsequent process checks found
no experiment processes remaining in either container. Containers themselves
were left running; no artifacts were deleted or overwritten.

The controller's `failed.txt` contains an empty KeyboardInterrupt description;
the explicit stop reason is retained in
`full_off_control/throughput_failure.json`. The source runner then reported
`Insufficient Kimera temporal coverage: 226.698709248s of 346s requested`.
That message is a consequence of the deliberate early stop, **not evidence
that Kimera independently crashed or stopped at that time**. The receiver's
`complete.marker` certifies only drainage of this shortened stream, not
successful completion of the intended full experiment.

All protected binary/runtime/profile/transport hashes still matched
`protected_manifest_v2.json`. No estimator, Qc, prior, covariance, timing,
publication tolerance, playback rate or source settings were changed. No full
validator pass, exact-repeat validation, common-mask full metrics or full ON
capture is claimed. The saved partial trajectories are not substituted for a
346-second matched comparison. End-drain accounting above comes from the
receiver summary; full independent source-payload conservation was not rerun
for this aborted case.

Evidence: `full_off_control/monitor.jsonl`, `validate_case.py`,
`full_off_receiver/transport_status.json`,
`full_off_receiver/receiver/publication_convergence.csv`,
`full_off_receiver/receiver/summary.json`, and `full_off_control/source.log`.
These establish loss of the frozen throughput operating point as the graph
grew, but do not yet isolate which solver operation causes that growth. No
new profiling campaign or corrective modification was started.

## Subsequent bounded 120-second CBS-ON check

User request: "Ok let’s try also 120sec of cbs on and check".
Final status: **120_SECOND_CBS_ON_LIVE_CHECK_PASSED**.

This is the same ETH1 interval start, source profiles, classic CBS protocol,
revision transport, WNOA executable/runtime, Qc, priors, support clock and
0.1x playback. Only duration (120 s), output labels and source sessions change.
The frozen throughput/convergence criteria remain unchanged. There is no
central IMU factor, covariance tuning, source change or heavy visualization.

Run-root additions are `bounded120_on.json`, `bounded120_on_PROTOCOL.md`, and
scoped copies `run_bounded120_case.py`, `score_bounded120_case.py`,
`score_bounded120_common.py`. The launch copy changes only phase/mode selection
and duration; the scoring copies select this ON case and its four trajectories
instead of a two-mode/eight-trajectory comparison. Existing scripts and
artifacts remain unchanged. Use the original `validate_case.py bounded120_on`
for independent payload accounting and exact replay after completion.

Primary outputs remain original/native GLIM, original/native Kimera ROS
odometry, fused online and fused final retrospective. The stopped 227-second
OFF final graph is **not** a fair 120-second final-graph comparator because it
contains later information; do not present it as a matched OFF baseline.

### Bounded capture validation

The source runner and receiver both completed normally. The effective ROS
parameter maps match the retained corresponding ETH1 ON baseline except
output paths and recording IDs; GLIM and Kimera profile trees match byte for
byte. The protected binary, runtime libraries, transport code and source
profile manifest remained unchanged. The original validator passed without
changing its thresholds or logic.

| Check | 120-second CBS-ON result |
|---|---:|
| Native GLIM poses / duration | 1,191 / 118.999748 s |
| Native Kimera ROS-odometry poses / duration | 584 / 119.599328 s |
| Receiver support states / duration | 1,193 / 119.2 s |
| Captured IMU samples (clock only in receiver) | 11,986 |
| Changed belief versions conserved exactly, in per-ID order | 70,762 |
| Active GLIM / Kimera BeliefIDs | 1,189 / 581 |
| Revision replacements | 68,967 |
| Unsupported startup rows | 25 versions of one pre-initialization Kimera ID |
| Pending / duplicate / stale revisions at end | 0 / 0 / 0 |
| Source G→K messages / injected factors | 1,189 / 580 |
| Source K→G injected factors | 581 |
| Source G→K applied-index tail gap | 1 |
| Median / p95 / maximum simulated output age | 0.070013 / 0.120019 / 0.150025 s |
| Maximum queue observed by controller / internal adapter high-water | 16 / 49 events |
| Stationary publication checks | 1,193 / 1,193 |
| Maximum normalized publication gradient | 9.998128466044178e-6 (limit 1e-5) |
| Maximum factor-vs-source covariance element difference | 6.776263578034403e-20 |
| Central IMU factors / bias variables / acceleration variables | 0 / 0 / 0 |
| WNOA factors / relative factors / initial priors | 1,192 / 1,770 / 3 |
| Controller wall time including startup/drain | 1,246.514 s |
| Exact captured-journal replay wall time | 341.052 s |

The independent validator reparsed the completed source logs and verified
all changed mean/covariance payloads and per-ID order against the captured
journal. It verified causal arrival/insertion/support ordering, stable-ID
replacement accounting, the frozen Qc/priors and covariance scale, and zero
pending events. The 25 unsupported startup versions are explicitly recorded,
not silently dropped or described as accepted.

Replaying that same journal produced byte-identical `online_trajectory.tum`,
`online_states.csv`, `trajectory.tum`, `states.csv`, `support_clock.csv`,
`source_model_covariance.csv`, `model_covariance.csv`, `causal_dispatch.csv`
and `measurement_events.csv`. These model-covariance diagnostic files are
**not** a complete posterior-covariance export. This proves reproducibility
of the recorded event ordering, not deterministic arrival ordering across
independent native estimator reruns.

Online trajectory SHA-256:
`b9b28773240c344793b5ae837e8f84e3aa5b609761358b4d9b44bcd9aa378480`.
Final trajectory SHA-256:
`5b7ace6038d4a07ddd2a62224da398ab5129baa8e56a1123b6153ed291c85283`.
All nine hashes and detailed validation are in
`bounded120_on_validation.json`; source exchange counters are retained in
`bounded120_on_source/run_metadata.txt`.

### Bounded accuracy results and coverage

The unchanged prism and IE-TC evaluators were first run on the original-rate
outputs (`bounded120_evaluation/`), then on a common valid timestamp grid
(`bounded120_common_evaluation/`). Only the latter is used for the primary
table. This uses the existing no-scale SE(3) alignment and CPT7→AP20 lever arm;
RPE uses the existing 1-second IE-TC relative-pose convention.

The intersection contains 1,161 samples at 0.1 s spacing from
`1727775011.5750432` to `1727775127.5750432`, spanning 116.0 s. Initial estimate
gaps exceeding the unchanged 0.5 s interpolation limit are excluded for all
four trajectories. All four have the same **626 valid prism matches covering
62.100 s** and 1,151 valid 1-second RPE pairs. Thus the ATE numbers are not
ground-truth validation of every second in the 120-second capture.
Common-grid timestamp SHA-256:
`0f35c93786afb955023f7116aa2a280318c3f5cedc596ca37049b78090fec79e`.

| CBS-ON output | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Full original body path m |
|---|---:|---:|---:|---:|
| Original/native GLIM | 0.032901 | 0.035228 | 0.451883 | 73.425268 |
| Original/native Kimera ROS odometry | 0.382791 | 0.033533 | 0.588966 | 71.522188 |
| Fused online/as-played | 0.117749 | 0.052404 | 1.793091 | 79.111703 |
| Fused final retrospectively revised | 0.116862 | 0.027224 | 0.430579 | 73.386298 |

Full body path above is the sum of Euclidean position increments in the
original TUM output, before scoring resampling/lever-arm conversion. The
prism-frame path diagnostics from common-grid scoring are separate:

| Output | Evaluation prism path m | GT-overlap estimate prism path m |
|---|---:|---:|
| Native GLIM | 73.160265 | 48.388053 |
| Native Kimera | 71.338332 | 47.307703 |
| Fused online | 78.323516 | 50.027913 |
| Fused final | 72.844679 | 48.292137 |

The matched GT prism path is 47.766353 m for every row. Full original body
paths and these evaluation/prism paths differ in timestamp coverage,
resampling and lever arm; do not treat them as the same quantity.

Interpretation: this 120-second operating point passed the frozen live
validation criteria at 0.1x. Native GLIM has the lowest ATE. Final fusion has
lower 1-second translational and rotational RPE than either native output,
but its ATE is worse than native GLIM. The online fused path has materially
higher RPE and accumulated length than its retrospective final path; a
successful final smoother result is not evidence of equally accurate live
publications. These are measured outcomes, not a newly isolated root cause.
No matched 120-second OFF final graph was run in this task, so no CBS-ON
versus CBS-OFF improvement claim is made. No settings were changed after
observing these results, and no follow-up experiment was started.

Exact executed commands (run-root paths are absolute):

```bash
python3 /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/run_bounded120_case.py bounded120 on
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/validate_case.py bounded120_on
docker exec cbsms_ws_gpu bash -lc 'source /opt/ros/noetic/setup.bash && python3 /workspace/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/score_bounded120_case.py bounded120'
docker exec cbsms_ws_gpu bash -lc 'source /opt/ros/noetic/setup.bash && python3 /workspace/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/score_bounded120_common.py bounded120'
```

After capture, validation and scoring, both containers had no experiment
processes remaining. Prior artifacts were preserved, the full-comparison
automation stayed paused, and no estimator C++ change/build or new sanitizer
run was performed. Launch/scoring copies passed Python syntax checks; their
diffs are limited to the duration/phase/mode and four-output selection
described above. HEAD remains the value recorded below; no commit was made.

## Matched first-120-second OFF/ON as-published comparison

At the user's request for OFF/ON results, the retained first 120 seconds of
`full_off` were compared with `bounded120_on`, without rerunning any estimator.
This is a matched **live/as-published trajectory** comparison, not a comparison
of final retrospective graphs. The stopped OFF final graph incorporates about
227 seconds of information and is deliberately excluded.

Both native source curves here are exported from their recorded ROS odometry:
`glim_odometry.csv` and `kimera_odometry.csv`. Receiver output is
`online_trajectory.tum`. In particular, the native GLIM row is **not** the
shutdown `odom_imu.txt` dump used in the earlier ON-only table. Comparing that
dump with same-timestamp ROS publications showed maximum position differences
of 0.036760 m for OFF and 0.068598 m for ON over their respective retained
runs. The two outputs must not be silently interchanged. This explains why
ON GLIM ATE below is 0.038209 m instead of the earlier dump-based 0.032901 m;
no estimator settings were changed.

All six curves use the same 1,161 physical timestamps as the preceding ON
evaluation: 116 seconds of common trajectory, 626 prism matches covering
62.100 seconds, and 1,151 valid 1-second IE-TC RPE pairs. Evaluators, no-scale
alignment, lever arm, interpolation limits and reference bags are unchanged.
Path below is the CPT7 body path on this common grid, not the original-rate
full-output path in the preceding table.

| Original/native or fused online output | Mode | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Common-grid body path m |
|---|---|---:|---:|---:|---:|
| GLIM ROS odometry | OFF | 0.028295 | 0.029414 | 0.358208 | 72.736668 |
| GLIM ROS odometry | ON | 0.038209 | 0.041163 | 0.468800 | 75.479675 |
| Kimera ROS odometry | OFF | 0.541048 | 0.045950 | 0.541480 | 69.720803 |
| Kimera ROS odometry | ON | 0.382791 | 0.033533 | 0.588966 | 71.266913 |
| gnssFGO online/as-played | OFF | 0.050385 | 0.031364 | 1.261076 | 74.452353 |
| gnssFGO online/as-played | ON | 0.117749 | 0.052404 | 1.793091 | 77.561919 |

Within this window, CBS improves native Kimera ATE by about 29% and its
translation RPE by about 27%, while its rotation RPE increases. Native GLIM
worsens on all three metrics. Online fusion also worsens on all three;
its ON ATE is about 2.34 times OFF. This observation does not by itself
isolate covariance, correlation, source means or another mechanism as the
cause. No tuning or subsequent experiment was performed.

Prefix validity checks: identical receiver executable SHA-256 and identical
receiver model settings (config differences are run paths/session labels).
Effective source parameters differ only in expected CBS receive/injection
switches and run paths/recording IDs. For the first 120 seconds:

| Check | OFF | ON |
|---|---:|---:|
| Maximum observed queue | 3 | 16 |
| Median output age s | 0.070012 | 0.070013 |
| p95 output age s | 0.120019 | 0.120019 |
| Maximum output age s | 0.130023 | 0.150025 |
| Stationary publication checks / total | 1194 / 1194 | 1193 / 1193 |
| Maximum normalized publication gradient | 9.904712e-6 | 9.998128e-6 |

OFF's later throughput failure began at 218.570044 seconds, outside this
comparison. These checks support scoring its retained early live output;
they do **not** retroactively declare the full OFF run validated or provide
a deterministic prefix replay that was not performed. ON retains its full
120-second conservation and deterministic-replay validation above.

Reproduction script: `score_retained120_online_comparison.py` in the run root;
it refuses to overwrite its output directory. Results, source hashes,
commands, common-grid trajectories and individual evaluator outputs are in
`retained120_online_off_on_evaluation/`, with `comparison.json` as the summary.

```bash
docker exec cbsms_ws_gpu bash -lc 'source /opt/ros/noetic/setup.bash && python3 /workspace/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/score_retained120_online_comparison.py'
```

## Reproduction and pending work

The run root retains `source.env`, phase-specific receiver JSONs, `prepare.py`,
`preflight_stream.py`, `run_case.py`, `validate_case.py`, `score_cases.py` and
`score_common.py`. `run_case.py` refuses existing output directories. Full
launches require both successful smoke validation files. Execute the host
controller using the existing containers; validation uses the evo Python
interpreter; scoring runs inside `cbsms_ws_gpu` after sourcing ROS Noetic.

Stopped full OFF controller command (provenance only; do not rerun into the
existing output directory):

```bash
python3 /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007/run_case.py full off
```

Full ON was not launched because full OFF did not pass the frozen operational
criteria. No full-sequence metrics or completion claim is available. A new
run, throughput policy or implementation change needs explicit authorization.

The first scheduled follow-up attempt was rejected by permission review.
After the user explicitly authorized it, a task-attached follow-up was created
on 2026-10-07: `complete-frozen-eth1-live-wnoa-comparison`, checking every five
minutes. Its explicit stop-on-failure instruction was followed: the automation
was changed to **PAUSED** after this throughput failure. It will not launch ON
or retry the capture automatically.

Repository HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
No commit made. The repository already contained extensive unrelated modified
and untracked research work, preserved; this is not a clean-worktree claim.

## Frozen 120-second mechanism audit — approved Steps 1–3

This follow-up supplies the matched OFF final graph that was intentionally
missing from the preceding comparison. Historical statements above describe
what had been completed at that time; the results below supersede the earlier
limitation concerning final-graph comparability, not the full-run failure.

Artifacts: [/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007](/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007).
`PROTOCOL.json` records the scope and selections before the audit. No native
GLIM/Kimera run, parameter change, covariance swap, weighting sweep or new
sequence was executed. These are offline reconstructions of already captured
live journals, not another live co-running experiment. Only the unchanged
frozen WNOA executable was used to reproduce graph evolution.

### 1. Matched information cutoff and identity gate

Both journals were cut at arrival time `1727775127854656555` ns: the existing
ON end marker, **119.851181555 s** after the first IMU timestamp. Every retained
event preserves its recorded order. OFF no longer receives its extra future
information. ON's journal is unchanged. The 11,986 physical IMU rows are
identical; here they supply the clock, not central inertial factors.

| Conservation/identity check | OFF | ON |
|---|---:|---:|
| Belief versions arriving by cutoff | 70,729 | 70,762 |
| Unique active GLIM / Kimera IDs | 1,189 / 581 | 1,189 / 581 |
| Replacements, not additional simultaneous factors | 68,934 | 68,967 |
| Explicit unsupported startup versions | 25 | 25 |
| Independent 100 ms support states | 1,193 | 1,193 |
| Recorded online states/trajectory/clock reproduced exactly | yes | yes |
| Final trajectory matches original captured final | not comparable: old OFF included future data | byte-identical |
| Maximum normalized publication gradient | 9.904712e-6 | 9.998128e-6 |

The 1,770 accepted physical intervals and their source indices match exactly
between modes. No duplicates, stale replacements or pending events remain.
The cutoff accounts for every version as an insertion, replacement or explicit
startup rejection. Causal availability checks pass. It is a matched time window
and topology, **not** an assertion that OFF and ON contain identical beliefs
or identical revision arrival histories.

Executable SHA-256:
`fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f`.
Clipped-journal hashes: OFF
`61211dae64debc0513a1f614ec4d3281951692830824d7ea0cd98d836e2a4b42`, ON
`4536c09a55938e6939519afbaf4e1acde5ef1b3dabf263b870e3550a10c56798`.
Original and protected-file hashes are in `manifest.json`.

### 2. Primary comparison: original sources and fusion

Same 1,161 common physical timestamps, 116-second span, **626 prism matches
covering 62.100 seconds**, and 1,151 one-second IE-TC RPE pairs as above.
The existing no-scale SE(3) evaluator, CPT7→prism lever arm and masks are
unchanged. Original GLIM and Kimera are their recorded ROS odometry, not
receiver source-only controls or GLIM's shutdown dump. The six earlier
native/online scores are reused after identity verification; only the two
matched final graphs are newly scored. `metrics.json` retains all eight rows.

| Output | Mode | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Common-grid body path m |
|---|---|---:|---:|---:|---:|
| Original GLIM ROS odometry | OFF | 0.028295 | 0.029414 | 0.358208 | 72.736668 |
| Original GLIM ROS odometry | ON | 0.038209 | 0.041163 | 0.468800 | 75.479675 |
| Original Kimera ROS odometry | OFF | 0.541048 | 0.045950 | 0.541480 | 69.720803 |
| Original Kimera ROS odometry | ON | 0.382791 | 0.033533 | 0.588966 | 71.266913 |
| Fusion online/as-played | OFF | 0.050385 | 0.031364 | 1.261076 | 74.452353 |
| Fusion online/as-played | ON | 0.117749 | 0.052404 | 1.793091 | 77.561919 |
| Fusion final retrospective | OFF | 0.045295 | 0.016425 | 0.311870 | 71.741407 |
| Fusion final retrospective | ON | 0.116862 | 0.027224 | 0.430579 | 72.906873 |

The final ON loss persists: about **2.58× ATE, 1.66× translation RPE and
1.38× rotation RPE** versus matched final OFF. Waiting for later revisions
greatly improves online RPE in both modes but does not eliminate the ON ATE
loss. Therefore prediction roughness alone cannot explain that loss. Conversely,
final fusion RPE is better than either original source in its respective mode;
the no-central-IMU model is not simply failing on every metric.

![Matched online and final metrics](/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007/matched_fusion_metrics.png)

### 3. Same-time prediction/revision localization

Every publication was saved. For each adjacent pair of snapshots we compare
the **same old physical state**, not consecutive positions along a moving
trajectory. Three locations are distinguished: the last support at/before the
latest source endpoint, its upper bracketing support, and the old current
support. Beyond the upper bracket, the graph has only WNOA prediction factors.
Statistics below cover 1,188 transitions with source information present.

| Old current state changed by next publication | OFF | ON |
|---|---:|---:|
| Position correction median / RMS / p95 / max, cm | 0.917 / 1.745 / 3.404 / 9.182 | 1.972 / 2.843 / 5.297 / 12.985 |
| Velocity correction median / RMS / p95 / max, m/s | 0.0811 / 0.1186 / 0.2266 / 0.5494 | 0.1348 / 0.2000 / 0.3890 / 1.0039 |
| Online→final same-time position change RMS, cm | 2.102 | 3.554 |
| Latest endpoint age median / maximum, s | 0.12811 / 0.20844 | 0.12811 / 0.20844 |

These are **estimate revisions, not errors against GT**. Nearly identical
endpoint ages do not support a longer ON prediction horizon as the explanation.
The previously recorded transport checks also exclude a queue backlog in this
window. This does not absolve prediction quality: ON's inferred velocity is
more strongly revised despite similar availability delays.

The largest ON position correction is snapshot **497→498**, revising physical
time `1727775058283475000` ns (50.28 s from the first IMU timestamp). The old
latest source endpoint is `1727775058155114700` ns. The batch contains one new
GLIM ID, one new Kimera ID, **48 GLIM revisions and 24 Kimera revisions**.
Position changes are 1.397 cm at the covered support, 3.989 cm at the upper
bracket, and 12.985 cm at the old current support. Upper-bracket velocity
changes by 0.8305 m/s; old-current velocity by 0.9587 m/s. This localizes a
large correction to the extrapolated tail and its uncertain boundary velocity,
not merely an absolute translation of the entire trajectory.

This batch does not isolate an individual culpable factor. For example, among
its 24 Kimera revisions, 16 improve relative translation agreement with IE-TC;
their median discrepancy falls from 6.281 to 3.971 mm. A large correction can
be a correction *of a poor prediction*, not evidence that the arriving revision
is itself worse. OFF's largest event, 817→818 at 82.28 s, also contains two new
IDs and 72 revisions. Full details are in `changes.csv`, `publications.csv`,
`prediction_summary.json` and `largest_event_beliefs.csv`.

![Same-time state revisions](/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007/same_time_revisions.png)

### 4. What information did the sources actually send?

All 1,770 accepted intervals are compared to the existing CPT7 IE-TC relative
reference on **identical physical endpoints**. The existing SE(3)
`Log(Z^-1 T_reference)` residual is used in `[phi,rho]` ordering; rho is a
tangent translation, not a new Cartesian measurement model. Reference
interpolation permits at most 50 ms endpoint gaps. The retained reference
cache came from `2024-10-01-11-29-55_cpt7_ie_tc.bag`, topic
`/boxi/inertial_explorer/tc/odometry`, child frame `cpt7_imu`.

The following uses each edge's latest version available at the common cutoff.
It is **not** a claim of finalization for edges near the end of the window.

| Source | Mode | Median rho discrepancy mm | Median rotation discrepancy deg | Median sqrt(trace Sigma_rho), mm | Median sqrt(trace Sigma_phi), deg |
|---|---|---:|---:|---:|---:|
| GLIM | OFF | 5.461 | 0.10967 | 1.471 | 0.006831 |
| GLIM | ON | 15.332 | 0.20811 | 1.303 | 0.005078 |
| Kimera | OFF | 2.894 | 0.10915 | 1.190 | 0.008103 |
| Kimera | ON | 4.025 | 0.10848 | 0.846 | 0.003754 |

Only 18.3% of matched GLIM intervals and 37.0% of Kimera intervals improve
rho discrepancy under ON. Kimera rotation improves on 54.4% of intervals.
This is not a contradiction with improved native Kimera ATE: native odometry's
long trajectory and current-smoother short relative beliefs are different
quantities. Global drift, local relative errors and their temporal correlation
cannot be inferred from one another through a single ATE number.

The same translation trend appears at matched publication ages, excluding
edges too young to reach that age in either mode:

| Source and mode | First-version median rho mm | At age 1 s | At age 2 s | At cutoff |
|---|---:|---:|---:|---:|
| GLIM OFF | 7.393 | 5.535 | 5.502 | 5.461 |
| GLIM ON | 15.498 | 15.282 | 15.281 | 15.332 |
| Kimera OFF | 4.170 | 3.035 | 2.979 | 2.894 |
| Kimera ON | 8.246 | 4.321 | 4.395 | 4.025 |

Revisions improve Kimera substantially relative to its first publication;
this audit does **not** justify disabling revision replacement.

Median **paired** ON/OFF covariance-trace ratios at cutoff are GLIM
rotation/rho **0.817 / 0.812**, and Kimera **0.236 / 0.480**. However, Kimera's
rho ratio has mean **1.273**, p95 **7.741** and maximum **14.342**. Thus the
covariance change is not a uniform shrinkage or one universal scale. The full
dense matrices, correlations and directions matter. All compared matrices are
finite, symmetric and positive definite; nothing was scaled or diagonalized
for receiver use. `belief_summary.json` includes generalized covariance
eigenvalue summaries and all four age selections; `belief_comparison.csv`
contains the underlying observations.

IE-TC shares CPT7 information with the sources and has no available joint
relative-reference covariance in this analysis. These are descriptive reference
discrepancies, **not calibrated NIS/NEES or proof of true covariance coverage**.
They nevertheless refute the premise that native Kimera's ATE improvement
necessarily meant more accurate short translation factors were entering this
receiver in this window.

### 5. Interaction of GLIM, Kimera and WNOA

Actual active factor slots, keys, means, full covariances and optimized X/V/W
states were reconstructed at 18 frozen snapshots: both modes at regular
approximately 30-second checkpoints, their largest correction before/after
indices, and the common final graph. The pre-existing `SourceWnoaModel`,
`WnoaRelativeFactor` and `GPWNOAPrior` implementations are reused, not rederived.
Whitened residuals and Jacobians are exported without changing any estimate.

There are three measurement/motion families here plus startup priors;
**there is no central IMU family**. Final per-factor mean squared whitened
residuals are:

| Family | Count | Residual dimension | OFF mean r^T Sigma^-1 r | ON mean r^T Sigma^-1 r |
|---|---:|---:|---:|---:|
| GLIM | 1,189 | 6 | 143.806 | 293.016 |
| Kimera | 581 | 6 | 253.948 | 273.721 |
| WNOA | 1,192 | 12 | 414.115 | 557.381 |

These are post-fit compromises, not calibrated chi-square tests or comparable
percentages of authority. OFF/ON total costs also use different source noise
models and are not a model-selection likelihood test. Source pose gradients
oppose one another strongly in both modes (final translation-block cosine
-0.916 OFF, -0.913 ON); stationarity means these competing gradients balance,
not that every factor is correct.

A stronger directional check differentiates the **frozen linearized solve**
with respect to a family's log precision. With whitened system A,r,
H=A^T A, and its unapplied Gauss–Newton step delta:

`d(delta)/d(log precision_family) = -H^-1 A_family^T (r_family + A_family delta)`.

All state variables are retained in that solve. The table reports RMS position
shape response after removing the best infinitesimal common rigid motion.
No calculated step is applied to a nonlinear graph; this is an infinitesimal
diagnostic, not a new weighted replay or an accuracy prediction.

| Precision differentiated | OFF response, m per unit log precision | ON response |
|---|---:|---:|
| GLIM | 0.017487 | 0.050755 |
| Kimera | 0.016708 | 0.050328 |
| WNOA | 0.001844 | 0.001894 |

For scale, a 1% precision increase corresponds locally to about 0.5 mm RMS
shape movement for either ON source family. Its direction may improve or hurt
GT agreement; this derivative alone does not decide that. The much smaller
WNOA response provides **no support for treating WNOA as the dominant driver
of final trajectory shape at these solutions**. It does not prove WNOA is
unimportant for velocity, interpolation or live extrapolation, nor predict
what would happen if it were removed.

Raw Jacobian curvature also illustrates why trace is insufficient: Kimera's
summed translation-block diagonal of J^T Sigma^-1 J is 3.216e10 OFF versus
1.434e10 ON, despite the lower median ON rho covariance trace. Its rotation
block rises from 7.692e11 to 1.882e12. These coordinate-dependent block sums
include full covariance coupling and Jacobians at each fitted graph; they are
not marginal information, independent evidence counts, or fusion weights.
See `linear_summary.json` and `interaction/` for all checkpoints and blocks.

### 6. Validation and boundaries of the conclusion

- Exact original executable and preserved model settings: independent clock,
  WNOA Qc `[0.00375,0.00375,0.00375,0.015,0.015,0.015]`, original priors,
  scale 1, full-history iSAM2, nominal strict publication checks, no central
  IMU/bias/acceleration variables. Source profiles and classic CBS are unchanged.
- Both journal identity and accounting gates pass. Maximum factor-vs-input
  covariance element difference: 6.776264e-20.
- Reconstructed factor costs agree with the saved graph to maximum relative
  error 9.4652e-11. Eleven snapshots include complete original payload probes:
  covariance difference 0, maximum residual difference 4.1138e-13. The other
  seven are explicitly marked without that optional probe, not claimed probed.
- Fresh normalized gradients across all 18 reconstructed snapshots are at
  most 9.8053e-6, below the unchanged 1e-5 threshold. Existing copied-graph
  batch/polish/full-backsub checks at five snapshots per mode differ from the
  captured solution by at most 2.479e-9 m position, 5.928e-9 rad rotation,
  2.855e-8 m/s velocity and 1.303e-7 rad/s angular rate. There is no evidence
  here of an unconverged incremental solution causing the performance gap.
- Sparse linear-solve residual ≤1.535e-12; central-difference validation of
  final-graph log-precision derivatives has relative error ≤9.808e-7. Summing
  all family derivatives cancels to ≤6.186e-13, as expected for common scaling.
- All **205 protected file hashes** and the pre-existing tracked diff hash
  remain unchanged. No estimator C++ edit, new production build, new source
  capture or sanitizer claim. Read-only diagnostic exporter compiled; analysis
  scripts passed syntax checks. Two generated figures were visually checked.
- Frozen replay wall times with snapshot diagnostics were 386.274 s OFF and
  392.737 s ON. These are offline diagnostic runtimes, not new live-throughput
  certification. The earlier full OFF operational failure beyond 218 s and
  the paused full-run automation remain unchanged.

Reproduction scripts are retained in the artifact root: `prepare.py`,
`run_replay.py`, `check_replay.py`, `beliefs.py`, `prepare_snapshots.py`,
`interaction.cpp`, `run_export.py`, `linear_analysis.py`, `trace_events.py`,
`score.py`, and `finish_audit.py`. Exact commands/container images are recorded
in the command JSONs and logs. `audit_validation.json` records final checks.
Scripts refuse overwriting the primary replay/evaluation artifacts.

**Assessment:** no-central-IMU WNOA remains a numerically valid, demonstrated
bounded operating model, not a guarantee of accurate current-time prediction
or full-sequence throughput. This audit separates two observed issues:
(1) larger ON live prediction corrections, and (2) a persistent final-graph
ON accuracy loss, accompanied by degraded relative translation means and
non-uniform covariance changes. It does not assign a causal percentage to
mean error, covariance calibration, source correlation or interpolation.
Stable-ID replacement avoids stacking versions of the same belief but does
not remove shared-IMU, CBS-feedback or cross-edge correlations.

**Recommended next step, not executed:** review these findings before the
planned Step-4 counterfactual. A narrowly isolated final-graph diagnostic could
keep ON means/topology fixed and substitute the matched OFF full covariance
for each of the 1,770 identical physical intervals, leaving WNOA/priors fixed.
That would test a covariance contribution without selecting a scale from ATE.
It is an explicitly retrospective mechanism control, not a proposed online
policy or a universal fix. No such intervention is authorized by this audit.

Final HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit made.
Existing unrelated dirty/untracked work is preserved. Stop after Steps 1–3.

## Authorized Step 4 — one final-graph covariance counterfactual

The user subsequently authorized the single proposed substitution. No other
counterfactual, scale sweep, reverse swap or source run was performed.
Artifacts: [gnssfgo_eth1_wnoa_covariance_counterfactual_20261007](/home/yeranis/results/gnssfgo_eth1_wnoa_covariance_counterfactual_20261007).
This section supersedes the preceding **not-yet-executed** status only.

### Intervention and meaning

Starting from the same final CBS-ON objective and its optimized states, match
each source factor to CBS-OFF by `(source, physical from timestamp, physical to
timestamp)`, verifying source indices. All **1,189 GLIM and 581 Kimera** factors
match uniquely. Use each run's latest observed version by the common cutoff
`1727775127854656555` ns. Revision numbers need not match, and no later OFF
information from its longer capture is used.

For every matched factor, the diagnostic calls GTSAM
`cloneWithNewNoiseModel(Gaussian::Covariance(Sigma_OFF, false))` on the
**original ON factor**. Thus its measurement mean, endpoint interpolators,
residual, Jacobians, factor type and support keys are preserved. Only the
complete dense 6×6 covariance is reassigned. No diagonalization, Schur
conditioning, scaling, rotation/translation separation or new frame conversion
is introduced. Both runs use the same calibrated CPT7 body and `[phi,rho]`
right-local residual convention.

The substituted objective is exactly:

`J_cf(X,V,W) = J_WNOA + J_priors + 0.5 sum_edges r_ON(X,V,W)^T Sigma_OFF^-1 r_ON(X,V,W)`.

This definition reassigns a noise model to the **ON residual**; it does not
claim that OFF's entire probability distribution, centered on a different
measurement mean, has been transported or calibrated for ON. All 1,192 WNOA
factors, the three priors, all 1,193 support times, Qc and the absence of central
IMU factors remain fixed. The state estimates are allowed to optimize; it is
the **measurement means**, not the estimated states, that are held fixed.

This is **retrospective final-objective optimization**, not a causal-arrival
replay, a live iSAM2 trial or a deployable covariance policy. The original
online/as-played trajectories are neither changed nor relabeled. A copied
unchanged-ON objective is checked as a numerical control, not another
scientific ablation. It returns the exact original positions and velocities.

The existing snapshot-audit optimizer settings are reused: GTSAM LM
`CeresDefaults`, `MULTIFRONTAL_QR`, up to 500 iterations then 100 polish
iterations, relative/absolute tolerance 1e-11. Both final objectives must
pass the original fresh normalized-gradient limit of 1e-5 before scoring.
No convergence tolerance or budget was changed after seeing results.

### Accuracy result

The new curve uses the **same** 1,161-point common grid, 626 prism matches
(62.100 s GT coverage), 1,151 exact one-second RPE intervals, no-scale SE(3)
alignment and CPT7→prism lever arm. Both the prism timestamp mask and the RPE
start/end pairs match the earlier final comparison exactly.

| Final retrospective fusion | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Common-grid body path m |
|---|---:|---:|---:|---:|
| OFF means + OFF covariances, frozen baseline | 0.045295 | 0.016425 | 0.311870 | 71.741407 |
| ON means + ON covariances, frozen baseline | 0.116862 | 0.027224 | 0.430579 | 72.906873 |
| **ON means + matched OFF covariances**, single diagnostic | **0.084021** | **0.028159** | **0.424457** | **72.757038** |

The original/native trajectories have not changed or been rerun. Their
OFF/ON ATE remains GLIM **0.028295 / 0.038209 m** and Kimera ROS odometry
**0.541048 / 0.382791 m**, as in the primary table above. Those are different
outputs from all three fused-final rows here.

For the new diagnostic, full support-grid body path is 73.179652 m, common-grid
evaluation prism path 72.705689 m, GT-overlap estimate prism path 48.266680 m,
and matched GT prism path 47.766353 m. These paths have different sampling,
frame and coverage conventions; they are not interchangeable.

Relative to ordinary ON, the covariance reassignment changes:

- ATE: **28.10% lower**, a reduction of 0.032841 m.
- Translation RPE: **3.43% higher**, an increase of 0.000935 m.
- Rotation RPE: **1.42% lower**, a reduction of 0.006121 degrees.

Numerically, it closes 45.89% of the matched OFF→ON **ATE difference**. That
is an arithmetic comparison for this counterfactual, **not** a statement that
45.89% of the algorithm's errors have a uniquely identified cause. It remains
0.038726 m worse than OFF in ATE and does not recover OFF's local accuracy.

### Validation and provenance

`PROTOCOL.json` was saved before the run. The diagnostic includes the existing
validated snapshot reconstruction and the original `SourceWnoaModel.h`; it
does not implement a new process or relative-measurement model. The production
receiver source/executable and all recorded source products are untouched.

| Check | Result |
|---|---|
| Exact matched source factors | 1,770 / 1,770 |
| Unchanged non-source factor objects | 1,195: 1,192 WNOA + 3 priors |
| Final graph states / scalar variables / factors | 1,193 / 14,316 / 2,965 |
| Minimum donor covariance eigenvalue / symmetry error | 2.011844e-10 / 0 |
| Maximum original reconstruction relative cost error | 9.465150e-11 |
| Original full-payload covariance / residual probe error | 0 / 4.771877e-14 |
| Maximum substituted covariance vs donor element difference | 2.202286e-20 |
| Unwhitened residual / Jacobian change due to substitution | **0 / 0**, checked at the ON solution and a deterministic perturbed state |
| Unchanged-control position / velocity change | **0 / 0** |
| Unchanged-control normalized gradient | 7.253004e-7 |
| Substituted objective at initial / optimized state | 551955.268258 / 491831.128522 |
| Substituted LM / polish iterations | 12 / 1, no budget exhaustion |
| Substituted fresh normalized gradient | **5.920509e-7**, passes 1e-5 |
| Frozen input/source/model files rehashed unchanged | **253** |

The raw same-world pose change from ordinary ON to the diagnostic has position
RMS 0.078099 m, maximum 0.113090 m and rotation RMS 0.198590 degrees. These
are solution differences, not aligned GT errors. Objective cost decreases
within the substituted graph; costs from different covariance assignments
must not be used as a cross-model accuracy comparison.

Isolated diagnostic build passed; Python syntax and `git diff --check` passed.
Build output retains existing unused-stub/renamed-entry-point compiler warnings;
those code paths are not invoked and were not modified. No new sanitizer or
full production regression suite is claimed. Solver wall time was 2.310 s
for the substituted objective and 1.151 s for unchanged-control checks; total
diagnostic process time was 7.931 s. This says nothing about live throughput.

The diagnostic binary SHA-256 is
`5509b4d68121a67b0644f460086489293c133047c612d5e0c87fd69d306c1455`.
`execution_environment.json` records pinned container images, mount layout and
commands. `build_command.json`, `run_command.json`, `evaluation/commands.json`,
`mapping.csv`, `factor_checks.csv`, `substitution_validation.json`, both
`optimizer.json` files, `metrics.json` and `validation.json` retain the checks
and exact outputs. Build/solve/scoring containers had networking disabled and
read-only source/baseline mounts; only the new artifact directory was writable.

### Conclusion and stop boundary

**The exported covariance assignments materially affect the final ATE loss
for these frozen ON means.** The controlled reassignment improves ATE while
every measurement mean and non-source factor is held fixed. Therefore the
covariance hypothesis has direct supporting evidence in this WNOA case, not
only in older IMU/WNOJ tests.

It is still **not a complete explanation or a validated fix**: translation
RPE slightly worsens, the remaining ATE gap is substantial, and both sources'
entire covariance matrices changed together. This test cannot assign the effect
to Kimera versus GLIM, scalar trace versus anisotropy/cross terms, or quantify
the contribution of shared-source correlation. Nor does a stationary solution
prove a global optimum of this nonlinear objective. The earlier evidence of
worse typical relative translation means remains relevant.

Do not turn this result into universal covariance inflation, adopt OFF
covariances as an online rule, or claim improved live prediction. No further
case or production-policy change was run. The full-run automation remains
untouched. Final HEAD is `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit
made, pre-existing tracked/untracked work preserved. Stop after this comparison.

## Authorized Step 5 — isolate GLIM versus Kimera covariance

The user authorized the two proposed source-specific substitutions. Both are
complete. **Exactly two new retrospective optimizations** were performed;
the original ON and previous both-source substitution are reused, not rerun.
No native estimator, live receiver, additional sequence, scale sweep or
production setting was run/changed.

Artifacts: [gnssfgo_eth1_wnoa_covariance_source_isolation_20261007](/home/yeranis/results/gnssfgo_eth1_wnoa_covariance_source_isolation_20261007).
`PROTOCOL.json` was saved before execution. The prior Step-4 stop boundary is
superseded only for these two explicitly authorized controls.

### Frozen graph and exact intervention

Both solves start from the **same original final ON Values**, never from the
other diagnostic's result. All ON measurement means, physical endpoints,
interpolators, keys, factor counts, priors and WNOA parameters remain fixed.
There are 1,193 support states, 1,192 WNOA factors, three priors and 1,770 source
factors: 1,189 GLIM and 581 Kimera. No central IMU factors are present.

Use the same exact source/physical-endpoint matches, verified source indices
and full 6×6 OFF covariance donors from Step 4. Both modes' versions are the
latest observed by the common arrival cutoff `1727775127854656555` ns, not
necessarily the same revision number. No additional future OFF information is
introduced. Each selected ON factor is copied with `cloneWithNewNoiseModel`;
**the unselected source and all non-source factors retain the original objects**.

| New case | GLIM covariance | Kimera covariance | Changed / unchanged source factors |
|---|---|---|---:|
| `glim_off_cov` | Matched OFF | Original ON | 1,189 / 581 |
| `kimera_off_cov` | Original ON | Matched OFF | 581 / 1,189 |

These are complete covariance reassignments in the unchanged right-local
`[phi,rho]` convention, not scalar rescaling or a coordinate transformation.
They do not assert that OFF uncertainty is calibrated for the ON measurement.
LM, initialization and acceptance settings are identical to Step 4:
`CeresDefaults`, `MULTIFRONTAL_QR`, 500 iterations plus 100 polish,
relative/absolute tolerance 1e-11, fresh normalized gradient ≤1e-5.
No retries or tolerance changes were needed.

### Accuracy: complete two-source covariance comparison

All rows below are **final retrospective fused trajectories**, not native
GLIM/Kimera or online/as-played paths. The four ON-mean combinations use exactly
the same ON measurement means. The first row is the actual OFF reference,
which also has different measurement means, and is outside that factorial
comparison. Existing native/online metrics above remain unchanged.

| Final fusion | Means | GLIM covariance | Kimera covariance | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Common-grid body path m |
|---|---|---|---|---:|---:|---:|---:|
| Actual OFF, frozen | OFF | OFF | OFF | 0.045295 | 0.016425 | 0.311870 | 71.741407 |
| Original ON, frozen | ON | ON | ON | 0.116862 | 0.027224 | 0.430579 | 72.906873 |
| **Change GLIM covariance only** | ON | **OFF** | ON | **0.123630** | **0.026234** | **0.425606** | 72.722449 |
| **Change Kimera covariance only** | ON | ON | **OFF** | **0.075816** | **0.029181** | **0.429465** | 72.954623 |
| Change both, frozen Step 4 | ON | OFF | OFF | 0.084021 | 0.028159 | 0.424457 | 72.757038 |

Scoring preserves the 1,161-point common 100 ms grid, 116 s common span,
626 prism matches / 62.100 s GT coverage and 1,151 exact one-second IE-TC pairs.
Both new curves' prism timestamps and RPE start/end pairs match the frozen
masks exactly. Same evaluator, no-scale alignment and CPT7→prism lever arm.

Relative to original ON:

- **GLIM-only covariance reassignment:** ATE **5.79% worse** (+6.768 mm),
  t-RPE **3.64% better**, r-RPE **1.15% better**.
- **Kimera-only covariance reassignment:** ATE **35.12% better** (−41.046 mm),
  t-RPE **7.19% worse**, r-RPE **0.26% better**.

The Kimera reassignment also improves ATE with GLIM already reassigned:
0.123630→0.084021 m. Conversely, reassigning GLIM covariance worsens ATE with
either Kimera covariance choice. Thus the ATE benefit of the previous joint
substitution comes from the **Kimera covariance reassignment**, partly offset
by GLIM's reassignment, for this particular fixed-ON-mean experiment.

The finite metric interaction `both − GLIM_only − Kimera_only + original_ON`
is +0.001438 m for ATE, −0.00003137 m for t-RPE and −0.00003537 deg for r-RPE.
These describe non-additivity of the measured scores, not a unique causal
decomposition of estimation errors or an assumption of independent sources.
The best diagnostic ATE remains 0.030520 m worse than actual OFF, and none
recovers OFF's local relative accuracy.

| New case | Full support body path m | Evaluation prism path m | GT-overlap estimate prism path m |
|---|---:|---:|---:|
| GLIM covariance only | 73.152151 | 72.680107 | 48.261667 |
| Kimera covariance only | 73.428437 | 72.879867 | 48.298851 |

Matched GT prism path is 47.766353 m in both. These path definitions have
different sampling/frame/coverage and must not be substituted for one another.

### What changed in the graph's compromise?

To avoid confusing a new noise scale with improved residual agreement, each
of the four ON-mean solutions is evaluated read-only using **the same original
ON factor covariances**. WNOA and priors already have identical noise models.
The table reports mean squared whitened residual per factor, not total cost,
calibrated NIS, independent information or a percentage of fusion authority.
Compare down a family column, not across families of different dimension.

| Solution, scored under fixed original ON noise | GLIM, 6D | Kimera, 6D | WNOA, 12D |
|---|---:|---:|---:|
| Original ON | 293.016 | 273.721 | 557.381 |
| GLIM covariance only | 401.338 | 231.532 | 530.309 |
| Kimera covariance only | 246.651 | 881.848 | 497.159 |
| Both covariances | 350.272 | 824.674 | 468.690 |

The Kimera substitution permits considerably more disagreement with Kimera
under its original ON noise metric, while the solution agrees more closely
with GLIM and WNOA. The GLIM substitution has the complementary effect: less
agreement with GLIM and more with Kimera. The unwhitened source residuals
confirm this is an actual change in fitted measurement agreement:

| Solution | GLIM rho RMS mm | GLIM phi RMS deg | Kimera rho RMS mm | Kimera phi RMS deg |
|---|---:|---:|---:|---:|
| Original ON | 8.803 | 0.08000 | 5.630 | 0.03092 |
| GLIM covariance only | 9.848 | 0.08373 | 5.000 | 0.02924 |
| Kimera covariance only | 7.826 | 0.07545 | 9.007 | 0.05430 |
| Both covariances | 8.987 | 0.07922 | 8.417 | 0.05302 |

These are **factor-to-fused-solution residuals**, not source-to-GT errors.
Rho is the translational SE(3) Log component, not a new Cartesian model.
Median/p95/max and each case's own-noise costs are also saved in
`validation.json`; per-factor records are in `fixed_on_noise/` and each case
directory. Changed-objective costs must not be compared as model-selection
likelihoods. The matrices change nonuniformly, so this is not proof that a
uniform reduction of Kimera precision would have the same effect.

Raw same-world state movement relative to original ON is:

| New case | Position RMS / max m | Rotation RMS deg | Max velocity change m/s | Max angular-rate change rad/s |
|---|---:|---:|---:|---:|
| GLIM covariance only | 0.035246 / 0.105064 | 0.284606 | 0.136597 | 0.033493 |
| Kimera covariance only | 0.072625 / 0.185433 | 0.425355 | 0.145446 | 0.164645 |

These are solution changes before evaluation alignment, not GT errors.

### Validation, provenance and stop

- Both isolated substitutions preserve all source means, unwhitened residuals
  and Jacobians **exactly** at the original Values and a deterministic nonzero
  perturbation. Factor types/keys/dimensions/counts are unchanged.
- Original full-payload covariance probe error is 0; maximum residual probe
  error is 4.771877e-14; maximum reconstruction relative cost error 9.465150e-11.
- Maximum covariance-vs-donor element error: GLIM case 2.752857e-21;
  Kimera case 2.202286e-20. All donors are SPD, minimum eigenvalue
  2.011844e-10, symmetry error 0.
- Unchanged source and non-source initial factor records are exactly equal
  to the previous original-ON control. Original graph fingerprint remains
  unchanged after each solve. All 1,193 physical support timestamps match.
- GLIM substitution: **11 LM + 1 polish** iterations, normalized gradient
  **6.712214e-7**, solve 2.054 s. Kimera substitution: **12 + 0** iterations,
  gradient **5.532362e-7**, solve 2.136 s. Both pass the unchanged 1e-5 gate
  without budget exhaustion. Total diagnostic process runtime 6.276 s.
- **305 protected files** rehashed unchanged, including all prior diagnostic
  artifacts and protected source/model/capture inputs. Frozen prior metrics
  are reused exactly. No production executable, source profile or graph model
  changed. Repository HEAD and pre-existing tracked-diff hash are unchanged.
- Isolated build, Python syntax, post-run validation and `git diff --check`
  passed. Same pre-existing unused-stub/renamed-entry-point compiler warnings
  as Step 4; those paths are not invoked. No new production regression suite,
  sanitizer run, live-throughput certification or global-optimum claim.

Diagnostic executable SHA-256:
`cda3bad9652fa0f1da78c10562285358f3b3eabf16e23dca46efc58d0ff94225`.
`*_container_command.json` records pinned images, no-network/read-only source
mounts and exact commands; only this new artifact root was writable inside
the containers. `manifest.json`, individual identity checks, optimizer logs,
evaluation commands, `metrics.json` and `validation.json` retain the evidence.

**Conclusion:** Kimera's full covariance reassignment is the source-specific
driver of the covariance-related ATE improvement in this frozen WNOA test.
GLIM's reassignment partly opposes that ATE improvement. This narrows the
covariance question, but does **not** establish a production remedy: the
Kimera substitution worsens t-RPE, does not match OFF accuracy, and changes
rotation/translation scales, directions and cross terms together. It does not
isolate correlation or show that Kimera should generally be downweighted.
Earlier observed relative-mean degradation remains a separate issue.

No follow-up weighting, channel experiment, source run or live run was started.
The full-run automation was not touched. Final HEAD remains
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit made; existing dirty and
untracked work preserved. Stop after these two controls.
