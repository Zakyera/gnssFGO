# ETH1 shared native covariance provider extended timing result

The single authorized 160-second CBS-ON capacity test stopped at **148.550033
simulated seconds** when the receiver queue crossed its unchanged 1,000-event
limit. The controller observed 1,018 events; the final transport accounting
recorded a peak of 1,020 during shutdown. This is a **failed capacity test with
partial evidence**, not a completed 160-second validation.

GLIM's complete-scan timing improved substantially over the observed prefix.
The late 120–148.57 s p95 was 724.291 ms, versus 1,124.443 ms in the retained
E1/E2 capture restricted to the same physical prefix. The immediate stop was
receiver backlog, not a recorded GLIM provider exception. Receiver publication
refinement alone reached a median 978.562 ms and p95 1,622.498 ms over
140–148.57 s, before other receiver work. Its nominal wall budget at this
playback speed is 1,000 ms per support interval.

No retry, estimator edit, rebuild, parameter tuning, default promotion or push
followed the failure. All owned experiment processes have stopped.

## Configuration and preserved semantics

This is task S5 in both engineering plans. The [predeclared protocol](PROTOCOL.md)
extends S4d's private provider capture using the same binaries and settings.
It does not extend its mathematical validation automatically.

- ETH1 mission `2024-10-01-11-29-55`, offset 0, requested duration 160 s,
  playback 0.10x. Hesai/CPT7 GLIM with a 5 s lag; front Alphasense stereo/CPT7
  Kimera with 25 states. Kimera's experimental IMU-support override is unset,
  as in S4d; no substitution of the older ALL_RIGHT pilot profile.
- Classic `active_window_temporary` CBS, source `live_revision=false`, existing
  native-IMU mean alignment, 0.20 s GLIM horizon, one pair per update, full
  `schur_relative_between` covariance, scale 1, Gaussian factors. External
  factors are removed before marginalization under the existing mode.
  Existing native safeguards remain; no new weighting or gating was added.
- `GLIM_SHARED_NATIVE_PROVIDER=shared_native` replaces the ordinary outgoing
  LOCAL covariance route and disables the independent nonlinear LOCAL
  smoother. MAIN still supplies the mean. Native Gaussian equations have their
  own native-only prior; MAIN's mixed marginal prior is not copied. Native
  equations are nevertheless evaluated at MAIN-conditioned charts, so this
  does not establish statistical independence or calibrated uncertainty.
  Later downstream diagnostic revisions still use their existing MAIN
  covariance path; they were not changed to the ordinary provider.
- Receiver `source_wnoa`, no central IMU factors, independent 100 ms support
  clock, unchanged 59-timestamp adapter startup, full-history iSAM2 QR and E2
  exact-Values reuse. Qc diagonal `[.00375,.00375,.00375,.015,.015,.015]`;
  pose/velocity/angular-rate prior sigmas `1e-5/10/3`; covariance scale 1.
  Strict normalized publication gradient threshold `1e-5` remains unchanged.
- Graph inspectors disabled; diagnostic revision export and existing bounded
  diagnostics retained. This was a live co-running source/receiver capture,
  not a source run followed by offline fusion.

The actual pre-playback process/library admission gate passed. Resolved source
parameters, Kimera flags and receiver model matched S4d; see
[preflight](evidence/preflight/validation.json) and
[effective settings](evidence/source/effective_mode_check.json).

## Stop evidence and partial completion

[Controller failure](evidence/capacity160_on_control/failed.txt) says
`live queue gate exceeded`. Playback stopped once; no automatic retry occurred.
The last completed GLIM scan was at 148.571408938 s, slightly beyond the last
controller IMU observation because source processing and observation are
asynchronous. There are 1,479 complete scan records and 1,476 provider queries.

The receiver finished consuming the abort-ended journal and wrote its partial
summary and trajectories. Neither source nor receiver has a `complete.marker`,
and the controller has no `complete.json`. A journal end record and
`transport_complete.json` certify transport termination, not completion of the
requested 160 seconds. The last `transport_status.json` froze at abort and is
not a final queue-drain measurement.

- Journal: 14,861 clock events, 88,279 belief-version events and one end record.
- Source versions: 70,429 GLIM and 17,850 Kimera; transport duplicates zero.
- Receiver: 2,201 active source IDs (1,476 GLIM, 725 Kimera), 86,053 replacement
  removals, 25 unsupported Kimera versions before initialization, pending zero.
  Thus `88,279 = 2,201 + 86,053 + 25` in the partial summary. These recorded
  totals do not substitute for the unperformed full lifecycle/wire audit.
- Online/as-played and final retrospective outputs each contain 1,481 rows
  spanning 148 s. They remain separate products; neither covers 160 s.
- All 1,481 recorded publication checks were stationary; maximum normalized
  gradient `9.989086623436253e-6`. The receiver retained 4,443 values and 3,684
  active factors at termination, including 1,480 WNOA and 2,201 relative factors.
  Central IMU/bias/acceleration variable counts remain zero.

The source controller log includes a rostopic logging reentrancy exception
during SIGINT shutdown. It follows the queue-triggered stop; it is not the
triggering estimator failure. No provider failure/reset or OOM was observed.

## Complete GLIM timing

All times below are wall milliseconds. Historical values are from the retained
E1/E2 160 s capture, truncated at this capture's last completed physical scan.
These are different coupled source captures, not an identical-input timing
ablation: different published covariance and scheduling can change subsequent
source estimates and optimization work. The table demonstrates an observed
whole-GLIM improvement, not exact attribution of every millisecond to one patch.

| Physical section | New scans | Old median | New median | Old p95 | New p95 | New maximum |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0–40 s | 393 | 205.728 | 164.786 | 511.092 | 304.645 | 501.783 |
| 40–120 s | 800 | 423.012 | 216.632 | 746.573 | 491.224 | 1040.144 |
| 120–148.57 s | 286 | 727.310 | 371.955 | 1124.443 | 724.291 | 1237.495 |
| Entire observed prefix | 1479 | 412.067 | 206.661 | 1036.715 | 622.301 | 1237.495 |

Four new scan times exceeded 1,000 ms, three in the late section. Observed full
prefix and late-prefix p95 are below the desired 800 ms headroom target, but
the predeclared 0–160 and 120–160 gates remain untested. There is no extrapolated
full-duration pass. The continuing capture includes scans around the short-run
cutoff, so its 0–40 s sample count differs from the earlier S4d standalone run.

Every requested 10 s bin is retained below; the final bin is explicitly partial.
MAIN includes native Gaussian capture/commit work. Timers are nested and must
not be added as disjoint runtime components.

| Physical section | Scans | Scan median | Scan p95 | MAIN p95 | Provider query median |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0–10 | 93 | 30.859 | 189.551 | 75.916 | 10.115 |
| 10–20 | 100 | 107.563 | 233.302 | 126.431 | 12.135 |
| 20–30 | 100 | 179.834 | 322.695 | 272.699 | 13.145 |
| 30–40 | 100 | 177.657 | 319.279 | 289.272 | 8.670 |
| 40–50 | 100 | 193.785 | 361.488 | 335.967 | 8.987 |
| 50–60 | 100 | 209.283 | 404.210 | 378.997 | 10.072 |
| 60–70 | 100 | 220.459 | 413.294 | 385.711 | 8.779 |
| 70–80 | 100 | 211.577 | 417.295 | 385.711 | 8.966 |
| 80–90 | 100 | 208.569 | 408.692 | 352.601 | 9.555 |
| 90–100 | 100 | 222.985 | 404.046 | 232.009 | 14.229 |
| 100–110 | 100 | 289.617 | 553.457 | 503.531 | 10.321 |
| 110–120 | 100 | 307.924 | 577.228 | 515.086 | 14.362 |
| 120–130 | 100 | 372.720 | 691.125 | 661.399 | 11.433 |
| 130–140 | 100 | 365.676 | 697.157 | 661.614 | 10.630 |
| 140–148.57 partial | 86 | 381.490 | 748.376 | 711.074 | 12.121 |

No independent LOCAL update was recorded. Provider query time over all 1,476
queries was mean 12.337, median 10.178, p95 28.506, maximum 49.519 ms.
The ordinary covariance scope recorded zero GPU linearization calls. MAIN still
made 1,326,282 synchronous GPU linearization calls and remains GLIM's largest
measured component: 292.047 s summed MAIN scope, versus 18.430 s ordinary
covariance scope and 28.266 s revision scope. These scope totals are not an
additive partition of complete scan time.

## Native history and receiver backlog

All recorded provider queries satisfied `admitted = active + absorbed + removed`;
no provider reset or failure token appeared. At the last query, epoch 2955,
the counters were `30,537 = 1,624 + 28,913 + 0`. This is the last outgoing
query, not a claim about the final callback. GLIM active frames stayed at 49–50
after startup, while active native Gaussian factors grew with graph density to
1,624. Key-window boundedness is not proof of constant factor count or bounded
memory for arbitrarily long runs. The full independent lifecycle/covariance
acceptance checks were not run after the capacity failure.

| Receiver physical section | Refinement median ms | Refinement p95 ms | Refinements over 1000 ms |
| --- | ---: | ---: | ---: |
| 0–40 s | 52.727 | 132.710 | 0/394 |
| 40–120 s | 199.306 | 447.436 | 0/800 |
| 120–148.57 s | 565.610 | 1442.148 | 42/285 |
| 140–148.57 s | 978.562 | 1622.498 | 40/85 |

These are the recorded nominal-publication refinement timings only, not total
receiver cost. At 0.10x, a 100 ms simulated support interval has a nominal
1,000 ms wall budget. Late refinement nearly consumes that budget in the
median and frequently exceeds it. Additional revision processing, dispatch and
output work leaves insufficient headroom. This is direct evidence of a receiver
capacity bottleneck; it does not yet isolate its precise internal cause or
prove a specific future optimization will solve it.

The last live output age was 1.575691541 simulated seconds. The 120–148.57 s
age p95 was 1.150032 s, despite an entire-prefix p95 of only 0.140033 s.
Whole-run averages therefore hide the late backlog. Controller instantaneous
queue samples peaked at 991; its cumulative maximum field exposed 1,018 and
triggered the stop. Final transport peak was 1,020.

Thirty-second process-memory sampling gave maximum sampled RSS of 1,136.922 MiB
for GLIM, 432.379 MiB for Kimera and 550.965 MiB for the receiver. Observed
process high-water marks were 1,143.379/432.379/552.078 MiB respectively. This is
not GPU memory or a full-resolution peak measurement, and memory increased
during the prefix. The stop was the queue gate, not evidence of exhausted RAM.

## Validation and artifacts

Fresh preflight reran and passed all 47 focused Release tests. The S4d 47
ASan/UBSan tests, 18 existing regressions and no-data startup evidence remain
retained prior evidence, not tests newly rerun during S5. Before playback, one
sandbox-denied Docker check and one incorrectly resolved S4d evidence path
were corrected with their failed preflight outputs preserved. Neither launched
sensor playback. There was exactly one source capture.

After the stop, all 747 protected hashes and all six source repository
HEAD/clean-state checks passed. Source DSOs and receiver executable are
unchanged. [Integrity manifest](integrity.json) preserves the checks, raw
artifact hashes and stopped container process inventories. The final
containers contain only their idle `tail` and `zsh` processes.

Analysis scripts pass Python syntax compilation. Authored-file whitespace checks
pass; four copied ROS parameter dumps retain their original trailing blank line,
which the all-files Git whitespace check flags. Raw evidence was not reformatted.

Full source coverage, provider-to-wire covariance validation, detailed source
lifecycle reconciliation and deterministic saved-journal receiver replay were
not run after failure. No ATE/RPE or CBS-OFF comparison is claimed. The existing
full-run validation and analysis scripts are retained but their success paths
were not executed. [Partial summary](partial_summary.json) contains the bounded
statistics; [partial analysis](partial.py) rejects a completed-run marker and
cannot certify a full run. Its queue field is explicitly the controller's
observed maximum, separate from final transport peak; the marker caveat was
corrected to reflect that no receiver completion marker exists.

Raw run root:

`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012`

Source, receiver and controller directories are under `prepared/` with prefix
`capacity160_on_`. Original native GLIM and Kimera odometry, revision logs and
recorded bags remain there. Receiver `online_trajectory.tum` and `trajectory.tum`
are the partial as-played and retrospective products, respectively.
The GLIM timing log is
`prepared/capacity160_on_source/ros_logs/7a11af54-c67f-11f1-b007-362a2d7902c5/glim_rosnode-1-stdout.log`,
SHA256 `9fee6b927b25a70522e686689337877857c76f9ba95c72a615b37ba9115a76f4`.

| Frozen component | SHA256 |
| --- | --- |
| S4d release_2 libglim_ros.so | `0114b435785ae85f58a69f1156abd1a2a320dc7138ca0b2dd4bfd56715d4ebea` |
| S4d release_2 libgtsam_points.so.1.2.1 | `42fb1e8a931ebc86a06ae41a2eaf6ff39b9efbe5a1a4fd9737ee06edb2274228` |
| E2 receiver replay | `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0` |
| Partial fused online trajectory | `caee0f16846833c9ff7bb89af2c3b90319e9d8a1684a74aa1c291a3aca2c35d3` |
| Partial fused final trajectory | `e4dffdbecb63ad60a8d05749470d34aa5f0191b13135c4d498cb7954a539b4ba` |

## Decision and commits

Keep S4d's bounded 40 s pass and S5's partial timing gain, but do not certify or
promote 160 s live capacity. The unchanged queue guard worked as specified.
GLIM's remaining MAIN cost and the receiver's growing publication cost are
separate issues; this capture does not justify a covariance or model change.
The narrow next proposal is a retained-log audit of receiver refinement around
140–148 s, before another live capture or implementation. It was not executed
as a follow-up experiment.

Recovery parent: `8d862639dc66de6f434d37cf6526bcedd3ccceaf`.
Preparation commit: `c9bb99ee99a4ece4a3529d5a63e944ee7745c76a`.
Branch: `engineering/shared-native-live160-20261012`. The completion commit
contains this report, the failed-capture evidence and both updated plan entries;
use `git log -1 --format=%H -- experiments/shared_linearization_s5_live160_20261012/REPORT.md`
to resolve it without a self-referential hash. No GitHub push in this task.

CAPACITY_FAILED_PARTIAL_CAPTURE
