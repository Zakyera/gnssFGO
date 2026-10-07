# GrandTour live asynchronous gnssFGO receiver

**2026-09-29 update:** matched SPX-3 120 s CBS-OFF/classic CBS-ON captures
completed with native GLIM, native Kimera and gnssFGO concurrently at 0.10×.
Use the [frozen live baseline](/home/yeranis/repos/gnssFGO/GNSSFGO_GRANDTOUR_CLASSIC_LIVE_BASELINE_20260929.md)
and [completed report](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_LIVE120_HEADLESS_OFF_ON_20260929.md)
for the current operating point. The earlier 30 s and transport-only readiness
statements below are historical, not the current completion status. No 0.3×,
0.5×, 1×, or other-sequence capacity is certified.

2026-09-28 historical 30 s status. **Update after the bounded live trials:** matched SPX-3 30 s
CBS-OFF and classic CBS-ON captures have now run with GLIM, Kimera and gnssFGO
concurrently at 0.10x. Their effective settings and complete event accounting
are checked in [the algorithm/settings audit](GNSSFGO_SPX3_LIVE30_OFF_ON_ALGORITHMIC_SETTINGS_AUDIT_20260928.md).
At that stage, 120 s and 0.3x/0.5x/1x throughput had not been established. The validation
section below describes the earlier transport-only preparation unless stated
otherwise; it is retained as that stage's evidence.

For the recovered classic source profile, native Kimera uses its historical
`KIMERA_IMU_SAMPLE_SUPPORT=current` packet path (also selected when unset).
The receiver uses ALL_RIGHT. Do not describe this as the earlier three-source
ALL_RIGHT pilot or silently change the source convention. For every matched
OFF/ON run, hold receiver graph-recording load equal. The successful 120 s
capture omitted graph snapshots in **both** modes and disabled both source
factor-graph inspectors while retaining full revision logs. The earlier 30 s
ON visualization run enabled graph snapshots while OFF did not; see its audit
for that comparison's startup exclusions and provenance limits.

## What is now live

The existing `grandtour_ct_replay` executable now accepts an incremental event
stream (`--event-stream FIFO`, or `-` for stdin). It keeps **one persistent
iSAM2 graph** and processes each newly available event. It does not wait for
completed trajectory CSVs, repeatedly restart the graph, or run a new solver.

The ROS1 adapter subscribes to `/boxi/cpt7/imu`, observes the existing full
revision diagnostic logs, and feeds that stream. It publishes:

- `/gnssfgo/pose`: `geometry_msgs/PoseStamped`, current support pose as computed.
- `/gnssfgo/path`: `nav_msgs/Path`, accumulated **as-played** support poses.

These are in a receiver-local `gnssfgo_world` frame, not prealigned to GT. No TF
or invented pose covariance is published. Native GLIM and native Kimera outputs
remain separate and untouched. Keep comparing **native GLIM, native Kimera,
and their fusion**, not source-only receiver graphs masquerading as native.

## Receiver preset (shared across compatible GrandTour profiles)

| Item | Frozen live preset |
|---|---|
| Support clock | Uniform 100 ms, anchored at the 59th central IMU sample; independent of all LiDAR/camera belief endpoints |
| Startup | Existing 59-sample initialization; existing gravity/bias/priors |
| Inertial factors | Existing Combined IMU and exact fractional ALL_RIGHT integration |
| Motion/interpolation | Existing gnssFGO WNOJ, Qc variance 10000; existing exact/single/double/same-support relative-factor paths |
| Solver | Full-history iSAM2; no new fixed-lag approximation |
| Source factors | Full 6×6 covariances, scale 1; stable-ID replacement, not accumulation of duplicate revisions |
| Excluded | GNSS, gating, robust loss, health weighting, covariance tuning |
| Source-to-source CBS | `classic`; **never enable `live_revision` for this preset** |
| Initial trial playback | 0.10×; not a claim that 1× will keep up |

The source registry is a **list**, not a four-LiDAR/three-camera architecture.
One, two, seven, or another number of IDs uses the same routing/ledger. Actual
capacity is a separate performance question. Each active source gets a unique
ID and session, its own log, and a verified calibration/profile. The present
log adapters understand GLIM and Kimera; they do not launch arbitrary new
estimators or perform extrinsic conversion. Every supplied belief must already
describe the **same calibrated central IMU body frame** (`cpt7_imu` here).

## Source protocol versus downstream revision transport

Read the [classic CBS runbook](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md)
before selecting any source settings. Source exchange remains exactly classic
CBS. Enable only the established diagnostic logging:

```bash
CBSMS_CBS_PROTOCOL=classic
CBSMS_EXPORT_ENDPOINT_AUDIT_ENABLE=true
KIMERA_CBS_REVISION_AUDIT=1
CBSMS_DIAGNOSTIC_NODE_OUTPUT=log
```

The adapter consumes `GLIM_CBS_EXPORT_BELIEF_ROW`,
`GLIM_CBS_REVISION_BELIEF_ROW`, and `KIMERA_CBS_REVISION_EDGE_ROW`. It preserves
their full mean/covariance. It does not publish back to either estimator.
In the current ROS1 launch, GLIM's ROS stdout is under `ROS_LOG_DIR`, whereas
Kimera's glog rows are in the source run's `estimators.log`. Set
`ROS_LOG_DIR=${CBSMS_RUN_ROOT}/${CBSMS_RUN_NAME}/ros_logs` before starting the
source runner. The saved SPX-3 source preset does this. The two configured log
paths are deliberately different; verify both sources increment in
`transport_status.json` during playback.

**Availability is the ROS simulated time when the adapter observes a complete
log line**, not its physical endpoint and not a backdated source-log timestamp.
Thus file flushing/polling adds a real, recorded transport delay. This is an
interim live transport using existing instrumentation, not a production ROS
revision-message interface. Kimera's logger lacks a reliable independent
arrival stamp. Receipt wall time and raw source revision/update metadata are
saved in `receipts.jsonl`.

The receiver-facing version is a monotonically increasing sequence per
`(source, session, i, j)`. Changed snapshots are retained in observed order;
identical repeated payloads are deduplicated. This handles the known repeated
Kimera source-version-zero snapshots without discarding changed means or
covariances. Original logger revision numbers remain in the receipt audit.
It is **not** a change to the source CBS revision/replacement policy.

Physical endpoints remain unchanged. Factors wait if a required right-hand
support is not yet causally available. Late factors attach to retained historical
states through the existing CT factors. There is no GLIM-selected master grid,
GLIM/Kimera pairing requirement, or measurement snapping.

Equal ROS-time receipts are ordered by **observed journal order**. A closing
IMU sample can create/publish a support before a subsequently observed belief
with the same clock stamp. Replaying that journal is deterministic; do not
claim universal equality with a CSV reader that pre-groups tied timestamps or
sorts sources differently. The byte-identity test below concerns the specific
tested event sequence. Endpoint timestamps in source diagnostics are logged
floating-point seconds; the adapter converts their printed values to integer
nanoseconds. Their precision is limited by that source logging format, not
upgraded to original device nanosecond precision by this transport.
`/clock` and the IMU arrive on separate ROS connections. A received IMU stamp
may lead the latest `/clock` callback slightly; the adapter uses the sample
stamp as the earliest possible receipt time and records the lag. This is
observable in `imu_clock_lag_events` and `max_imu_clock_lag_ns`.

## Files and launch procedure

- Receiver template: [config/grandtour_live_receiver.json](/home/yeranis/repos/gnssFGO/config/grandtour_live_receiver.json).
- Source example: [config/grandtour_spx3_live_sources.env](/home/yeranis/repos/gnssFGO/config/grandtour_spx3_live_sources.env).
- Receiver launcher: [tools/launch_grandtour_live_receiver.py](/home/yeranis/repos/gnssFGO/tools/launch_grandtour_live_receiver.py).
- ROS adapter: [tools/grandtour_live_receiver_ros1.py](/home/yeranis/repos/gnssFGO/tools/grandtour_live_receiver_ros1.py).

The SPX-3 source example uses the retained September-27 profiles, Hesai,
front-stereo Kimera, shared CPT7, and a **bounded 60 s / offset 32 s** window.
It is not silently applicable to ETH1/EIG-2 or a new sensor arrangement. For
each future sequence select the correct mission, interval, source profiles,
calibration, data paths and GT/evaluation configuration separately. Do not tune
shared CBS or fusion parameters while preparing a new sequence.

1. Copy the receiver JSON to a run-specific file. Set `sequence` explicitly
   (mission + window), verify source IDs/session/frame declarations, ROS master
   port, and log globs. `SOURCE_RUN` is substituted by the launcher. The
   receiver output and source run must be separate fresh directories under
   `/home/yeranis/repos/V4RL` so both existing containers can access them.
2. Start a caller-owned ROS master on the selected port **before** sources or
   bag playback. In `cbsms_ws_gpu`, source `/opt/ros/noetic/setup.bash`, run
   `roscore -p 11368` in its own terminal; set `ROS_MASTER_URI` accordingly and
   `/use_sim_time=true` in another terminal. Do not reuse another experiment's
   clock/master. Keep this master alive until the receiver has drained.
3. Start the receiver on the host, after a read-only preflight:

```bash
python3 /home/yeranis/repos/gnssFGO/tools/launch_grandtour_live_receiver.py \
  --config /ABSOLUTE/PATH/TO/sequence_live.json \
  --source-run /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/NEW_SOURCE_RUN \
  --out /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/NEW_RECEIVER_RUN \
  --check
# Repeat the same command with --start instead of --check.
```

4. Wait for `NEW_RECEIVER_RUN/transport_ready.marker`. Then launch the existing
   source runner using that sequence's validated environment/profile. Set
   `CBSMS_USE_EXISTING_ROS_MASTER=true` and matching `CBSMS_ROS_MASTER_PORT`.
   The only source-runner change in this task is this opt-in master reuse;
   default behavior remains unchanged. For the SPX-3 example, copy/source the
   `.env` inside Noetic, explicitly set `CBSMS_CBS_ENABLE=true` or `false`, and
   choose a fresh `CBSMS_RUN_ROOT`/`CBSMS_RUN_NAME` matching `--source-run`.
   Then invoke the existing
   `/workspace/cbs_gtsam4.3/src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh`.
5. After the source runner has completed **and all source output has drained**,
   create `NEW_RECEIVER_RUN/STOP` or call ROS service `/gnssfgo/finish`. This
   flushes remaining complete log lines, sends an explicit end marker, and
   allows the receiver to finish. Do not use a guessed timeout as evidence that
   all source revisions arrived. Stop the caller-owned master only afterward.

The launcher never starts a bag, GLIM or Kimera on its own. It refuses existing
receiver output directories and already-started source log directories.
Interrupting the launcher is an **abort**, not a complete experiment; it stops
only its own receiver/adapter PIDs, not source estimators or other ROS masters.
Source/log restarts, ambiguous paths, clock regressions, changed endpoint
bindings, invalid covariance and queue overflow are failures, not silently
repaired data. The template's IMU-gap guard is 30 ms and must be reviewed if a
different IMU/rate is ever used; it is a transport integrity check, not gating.

## Outputs, timing, and visualization

`events.csv` is the ordered receiver input journal (`I`, `B`, explicit `E`).
`receipts.jsonl` records observation time and source version metadata.
`transport_status.json` reports source counts, queue size, latest IMU/output
timestamps, and output age. `receiver/stream_progress.csv` records consumed
event counts and wall-clock progress. A near-empty pipe-writer queue alone is
not proof of low solver latency: also compare progress and output age.

`receiver/online_trajectory.tum` and `online_states.csv` are flushed as each
support is created; the live ROS path contains those same causal/as-played
poses. Late revisions can change the internal current/historical solution;
the past as-played path is **not** retrospectively overwritten. After explicit
end, `trajectory.tum` and `states.csv` contain the final revised solution.

Check `source_accounting.csv`, `revision_events.csv`, `causal_dispatch.csv`,
`unsupported_beliefs.csv` and `summary.json`. The legacy `glim_*`/`kimera_*`
summary counters only recognize canonical names; for custom source IDs use
`source_accounting.csv`. Messages before initialization or lacking a final
right-hand support are explicitly accounted for, never extrapolated into the
graph. No blanket “all input factors accepted” claim is allowed.

`--graph-snapshots` enables the existing actual optimized pose/factor snapshot
export and flushes it during streaming. It adds IO/compute cost; leave it off
for initial timing measurements. This task exposes live pose/path topics and
the existing graph exports; it does **not** rebuild or relaunch the Rerun
dashboard. A future live viewer must use these live timestamps, not show the
final retrospective trajectory as if it was known online.

## Validation performed

Artifacts: [gnssfgo_live_receiver_20260928](/home/yeranis/results/gnssfgo_live_receiver_20260928/).
ROS smoke: [receiver_live](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_live_transport_test_20260928/receiver_live/).

| Check | Result |
|---|---|
| Release target build | Passed in the existing Humble build environment |
| Existing support/IMU/CT-factor suites | 3 CTest targets, 14 individual tests passed |
| Adapter unit tests | 5 passed: exact dense payloads, revisions/dedup, invalid data, partial log lines/truncation, different source combinations |
| Pipe vs existing offline receiver | Retained first 5 s: trajectory, full state-row history, online history, support clock/acceleration and model-covariance files byte-identical |
| ROS transport at 0.10× | 501/501 IMU samples, 1,203 belief snapshots; exact means/covariances/physical timestamps conserved |
| Live publication | 45 poses received **before** end-of-stream; writer queue maximum 4 events |
| Receiver accounting | 42 GLIM + 21 Kimera stable IDs active; 1,071 replacements; 69 pre-initialization revision rows unsupported; no pending rows at end |
| Captured live-journal repeat | All seven state/trajectory/clock/model-diagnostic files byte-identical; optional graph snapshots exported |
| Negative stream cases | Abrupt EOF, clock regression, invalid covariance, premature end rejected; missing final support recorded without extrapolation |
| Previous independent-receiver synthetic suite | Passed, including future-suffix invariance, delayed IMU, sessions, duplicates/stale versions, all five endpoint patterns |
| Launch/Python syntax | Passed |

The recorded model-covariance diagnostics are **not** a full posterior
covariance export. No claim of separately comparing the complete posterior
covariance is made. No new trajectory-accuracy conclusion or ATE tuning was
performed. No new full-binary ASan/UBSan run is claimed for this task.

The ROS test in this historical validation section used retained data
published by a test fixture, with 1 ms explicit simulated transport margin for
`/clock` delivery; it did **not** run the source estimators. The later 120 s
SPX-3 capture did co-run both sources and the receiver at 0.10×, but longer-run
memory/runtime, logger flush latency, and real-time 1× operation remain to be
tested. Full-history iSAM2 and
retaining the entire IMU/journal are unbounded in memory; the existing
integration implementation is not optimized here. Start with a bounded 0.1×
source trial before committing to long or many-source runs.

## Provenance and next boundary

gnssFGO base HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
CBS runner repo base HEAD: `db9ddcaa47c419ca59987746ff8ddd37410a23d3`.
Both workspaces already contained uncommitted research changes; preserved.
This implementation is not committed in this task. Launch manifests save
binary/config/transport hashes; test manifests retain their actual tested
versions. Existing solver/factor/interpolation math and source estimator code
were not changed.

Final built receiver SHA-256:
`e5af1ade5f72ed1e05fb4505613840eb20e7b2b76623384605db6386bd390c2c`.
Its final-build pipe/offline identity evidence is in
`/home/yeranis/results/gnssfgo_live_receiver_20260928/final_build_equivalence/validation.json`.

**Ready for a bounded live co-running validation, not certified for 1× or for
every GrandTour sequence without its sequence-specific preflight.**
