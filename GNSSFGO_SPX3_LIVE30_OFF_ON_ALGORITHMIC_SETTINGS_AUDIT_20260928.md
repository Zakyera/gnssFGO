# SPX-3 live 30 s OFF/ON: algorithm and effective-settings audit

Date: 2026-09-28. Scope: read-only verification of the completed 30 s runs before a full-sequence experiment. No GLIM, Kimera, fusion replay, bag playback, rebuild, covariance tuning, or new performance experiment was executed. The only new executable work was analysis of retained logs and outputs.

## Verdict

The retained evidence supports that both runs executed the intended **recovered classic CBS source configuration plus asynchronous, revision-aware gnssFGO receiver**. No accidental `live_revision` source protocol, missing downstream revision stream, covariance scaling, GLIM-selected support grid, duplicate active source factor, future-dependent factor insertion, or unaccounted late source dropout was found.

This is a bounded configuration/event-integrity validation. It does not establish calibrated statistical independence, optimal fusion settings, full-sequence runtime, or 0.3x/0.5x capacity. The comparison also has a visualization-load difference: receiver graph snapshots/recording were added to ON. Keep that load equal for the full OFF/ON comparison.

**Timing clarification:** gnssFGO uses ALL_RIGHT. Native Kimera uses the recovered classic `current` packet convention; `KIMERA_IMU_SAMPLE_SUPPORT` was unset in both saved process environments, and the code selects `current` when unset. This agrees with `CLASSIC_CBS_BASELINE_RECOVERY_AUDIT.md`, which explicitly preserves that historical path. These runs must not be described as three-source ALL_RIGHT runs or confused with the earlier experimental ALL_RIGHT pilot. No timing setting was changed in this audit.

## Evidence and reproducible audit

Completed captures under `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/`:

- OFF: `gnssfgo_spx3_cbs_off_live30_verified_20260928/`.
- ON: `gnssfgo_spx3_cbs_on_live30_20260928/`.

Each contains `spx3_source/run_metadata.txt`, copied source profiles, copied launch/runner, resolved ROS parameters, ROS launch process arguments/environments, source stdout/glog, native output trajectories, and the receiver journal/accounting/summary. Earlier aborted transport captures are excluded.

New [audit script](/home/yeranis/results/gnssfgo_spx3_live30_settings_audit_20260928/audit.py) and [machine-readable results](/home/yeranis/results/gnssfgo_spx3_live30_settings_audit_20260928/audit.json) retain the exact comparisons, file hashes, effective flags, statistics and assertions. Run with the already-installed numerical Python runtime:

```bash
/home/yeranis/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  /home/yeranis/results/gnssfgo_spx3_live30_settings_audit_20260928/audit.py
```

Both ROS containers were stopped at audit time; they were left stopped. No new receiver-execution or sanitizer result is claimed. Existing regression artifacts were inspected separately below.

## 1. Native source settings: requested versus actually launched

| Item | Verified effective setting in both runs |
|---|---|
| Mission/window | GrandTour SPX-3 `2024-11-02-17-43-10`, bag offset 32 s, duration 30 s |
| Playback | 0.10x |
| Source runner | Existing `run_grandtour_eth1_glim_kimera.sh`, copied in each capture |
| GLIM inputs | `/boxi/hesai/points`, `/boxi/cpt7/imu`; CPT7 body frame |
| Kimera inputs | Saved front stereo profile; `/grandtour/front_left/image_raw`, `/grandtour/front_right/image_raw`, shared `/boxi/cpt7/imu`; `cpt7_imu` body |
| GLIM smoother | 5.0 s, saved GPU odometry profile |
| Kimera smoother | `nr_states: 25`, saved camera extrinsics/intrinsics and IMU profile |
| Source protocol | `classic`, `cbs_live_revision_enable=false` in actual GLIM parameters and Kimera command line |
| Source publication | Enabled in OFF and ON; `time_horizon_window`, 0.20 s horizon, 0.06 s horizon tolerance, stride 1, maximum 1 pair/update |
| External factor handling | Persistent; OFF disables receiving, ON enables both directions |
| Receive start delay | 0.0 s |
| Covariance extraction | `schur_relative_between`, dense 6x6 |
| Covariance source | GLIM `direct`; Kimera local smoother extraction enabled, persistent covariance sidecar disabled |
| Source covariance scales | K→G 1.0, G→K 1.0 |
| Source health weighting | Master switch false; relative trust switches false. Diagnostic sender/NIS flags do not enable weighting |
| Source matching | GLIM 0.12 s, Kimera 0.05 s; duration gate enabled, 0.06 s tolerance, ratio 0.5–2.0 |
| Diagnostic revision exports | GLIM endpoint/revision audit on, `KIMERA_CBS_REVISION_AUDIT=1`; source exchange remains classic |
| Source graph inspection | GLIM stride 1, Kimera stride 5; inspectors enabled in both |
| Presentation EKF | Disabled; `ekf_rows=0` |

The actual native source process flags match across OFF/ON, except output/config paths and intended receiving/mode switches. Kimera's `cbs_backend_enable=true` also appears in OFF: this keeps its export/backend machinery available; OFF's receive switch is false and actual applied external count is zero. It does not mean CBS corrections were enabled in OFF.

All 16 files in each GLIM profile and all 14 files in each Kimera profile are byte-identical between OFF/ON and the frozen September-27 SPX-3 OFF profile copies prescribed by the runbook. This includes camera calibration, IMU settings and all flag files; the metadata's generic `eth1_mono_shared_cpt7` overlay name is not the selected camera mode. Actual mode and topic remaps confirm stereo.

The JSON-of-relative-path/content-hashes manifest digests used by **this audit** are GLIM `036e74b83367531796334246c0acfc3d137c0e36106bfc91018c7c2270ae3abd` and Kimera `b8f8779cc86d46d04edcba33fe70800a487938626ac05d73afd71fe4047d70e4`. This serialization differs from the older runbook manifest serialization; per-file comparisons, not cross-format aggregate digest comparison, establish equality.

## 2. Source exchange actually happened online

Independently recounted from the original source logs, without counting ROS `latest` symlinks twice:

| Evidence | OFF | ON |
|---|---:|---:|
| Native GLIM poses | 292 | 292 |
| Native Kimera ROS odometry poses | 149 | 149 |
| G→K queued external factors | 0 | 144 |
| G→K actually applied persistent factors | 0 | 144 |
| K→G actually injected persistent factors | 0 | 145 |
| Revision extraction error rows | 0 | 0 |

Thus ON was actual source-to-source exchange during playback, not an offline reconstruction. OFF published beliefs for downstream fusion while applying zero external CBS factors. The normal classic source timestamp/duration matching can reject or defer candidate edges; the table reports actual applications, not an assumption that every published candidate was injected. Exact final source active-factor inventories and all retirements are not fully exported (`active_factor_detail` logging is off); total injections must not be mistaken for simultaneously active factors.

Native GLIM and Kimera run concurrently with the receiver. Source revisions are exported from their normal backend-update path. Kimera's additional diagnostic revision query only appends to the outgoing CBS queue if `cbs_live_revision_enable` is true; it was false. See `VioBackend.cpp` around lines 3555–3688. Diagnostic queries impose compute/IO load, so unchanged estimation semantics do not imply identical thread scheduling to a run without logging.

## 3. Fusion graph and frozen settings

The receiver executable retained on disk hashes to exactly the hash saved at both launches:

`e5af1ade5f72ed1e05fb4505613840eb20e7b2b76623384605db6386bd390c2c`

Path: `/home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay`. Saved transport copies also match both manifest hashes and one another. Configuration manifest hashes validate against each saved resolved JSON.

| Fusion component | Verified setting/behavior |
|---|---|
| Execution | Persistent incremental event stream, `mode=sources`, `timeline=record` |
| Support clock | Uniform 100,000,000 ns; starts at the 59th received IMU sample |
| Source independence | No GLIM support input, pairing, source snapping, or GLIM master timeline |
| State blocks per support | Pose `X`, velocity `V`, bias `B`, angular rate `W` |
| Solver | Full-history iSAM2, QR, relinearization threshold 0.01, skip 1 |
| IMU | Combined IMU, ALL_RIGHT, exact fractional interval overlap |
| GP prior/interpolation | Existing WNOJ, diagonal `Qc` variances 10000 |
| Relative factors | Original full 6x6 covariance, Gaussian, scale 1; both endpoint poses evaluated through existing CT interpolation |
| Revision identity | `(source, session, from_index, to_index)` |
| Replacement | Remove the ledger's old factor index and insert exactly one revised mean+covariance factor |
| Excluded | No GNSS, EKF, robust kernel, receiver NIS gate, health weighting or covariance scaling |

All 436 supported distinct source factors use the double-interpolated endpoint path in these runs. Neither source has privileged exact-grid endpoints. Every one of the 64,093 saved ON source-factor snapshot rows has the correct `X/V/W` support keys for its two actual physical endpoints, with no duplicate source ID within a snapshot. Final active graph: 4 priors + 294 Combined IMU + 294 WNOJ + 290 GLIM + 146 Kimera = **1,028 factors**, over 1,180 variable keys / 295 support states.

Relevant receiver code: `GrandTourCsvReplay.cpp`, `IntegrateAllRight` (~502), `AddRelativeFactor` (~605), `IncrementalBackend::ReplaceFactor` (~731), initialization/model construction (~943), priors (~1085), `process_arrival` (~1258), support update (~1460), and output writing (~1540).

### Frozen initialization and noise

Both runs have exactly identical raw IMU payloads, startup state and initialization summary. Initial timestamp `1730565903454194000 ns` is exactly sample index 58 (59 samples). All 295 support timestamps match between OFF/ON. The 3,000 IMU samples have approximately 10 ms spacing, maximum 10.001 ms; no missing-sample gap was found.

The canonical ordered IMU timestamp+six-value payload digest, excluding observation timestamps, is `82c093f559eb09ceba7f7e5f32d17b67c938f69f380fbc1cd0f905cebfd4192f` in both.

Initial pose identity and velocity zero; gravity `[-0.30599823623420908, 0.028027105700951221, 9.805186360328273]`; accelerometer bias `[0.00529344565252049, -0.000484839267870418, -0.169619347320376]`; gyro bias `[-0.00223672487062670, -0.00618468117663416, -0.00507523388586054]`.

Initial prior standard deviations: pose `[0.174533,0.174533,0.00174533,1e-5,1e-5,1e-5]` in rotation/translation order; velocity 0.001 each; bias `[0.1,0.1,0.1,0.01,0.01,0.01]` in accelerometer/gyro order; angular rate 0.1 each. IMU densities: gyro `1.113913052107e-5`, acceleration `5.928679563445e-4`, gyro bias `0.000266`, acceleration bias `0.0043`; integration covariance `1e-8 I`, bias integration covariance `1e-5 I`. These are the unchanged existing settings, not a new claim that stationary initialization is physically exact during this moving section.

## 4. Complete factor-payload and replacement audit

The audit independently parses each raw diagnostic source log and compares ordered source IDs, physical endpoints, all six mean entries and all 36 covariance entries with the actual `events.csv` input. All rows match exactly as floating-point payloads; none were silently diagonalized or changed. All covariances are finite, symmetric and positive definite. The receiver additionally recorded its inserted Gaussian covariance comparison, whose maximum absolute discrepancy is roundoff-level.

| Check | OFF | ON |
|---|---:|---:|
| GLIM revision rows transported | 12,730 | 12,909 |
| Kimera revision rows transported | 3,400 | 3,400 |
| Total version rows | 16,130 | 16,309 |
| Accepted factor-version insertions | 16,080 | 16,259 |
| Previous-factor removals | 15,644 | 15,823 |
| Final unique GLIM / Kimera factors | 290 / 146 | 290 / 146 |
| Unsupported version rows | 50 | 50 |
| Duplicated/stale version actions | 0 / 0 | 0 / 0 |
| Pending at end | 0 | 0 |
| Minimum GLIM covariance eigenvalue | 5.4274e-9 | 3.4993e-9 |
| Minimum Kimera covariance eigenvalue | 2.5232e-9 | 1.7676e-9 |
| Maximum source covariance symmetry error | 0 | 0 |
| Maximum input→factor covariance element difference | 1.7347e-18 | 1.0842e-17 |

For every accepted action, the audit verifies monotonic per-ID version numbering, unchanged endpoints, correct old factor slot, unique active new slot, and the active-factor count `4 + 2*created_support + number_of_active_source_IDs`. Every final supported factor uses that ID's last observed source version. Thus revisions replace information rather than accumulating thousands of parallel copies.

The transport renumbers changed snapshots monotonically per ID because the existing Kimera logger can repeat a source revision number with changed payload. Original source revision/update metadata remain in `receipts.jsonl`. This is downstream transport bookkeeping, independent of source-to-source classic CBS policy.

### Exact startup exclusions

The 50 unsupported messages in each run are 25 versions each of two Kimera intervals:

- `1730565903072597500 → 1730565903272595900 ns`.
- `1730565903272595900 → 1730565903472594000 ns`.

Both start before the initial receiver support at `1730565903454194000 ns`. Consequently Kimera supplies 148 unique source edges, of which 146 can be represented by the initialized receiver. GLIM supplies 290/290. These are explicit startup exclusions; there is no 55 s gap and no later unsupported edge. Claiming all 148 Kimera edges were fused would be incorrect.

## 5. Causality and actual live progress

For **every** accepted version, retained dispatch data verify `insertion >= receipt`, `insertion >= required right-support availability`, and `required_support <= created_support`. The complete input journal is nondecreasing in observed availability time. Source means are used at their original physical endpoints, even when the belief arrives later. The GP factor attaches to retained historical states; the receiver updates the existing graph.

The log adapter observes a complete line before making it available. Availability is adapter-observed simulated time; physical endpoint time is not substituted for arrival. GLIM/Kimera file flush and polling delay therefore remain part of this interim live transport. Slight separate `/clock` callback lag is handled by the received IMU stamp floor; maximum recorded lag is ~1.04 ms, with zero belief-clock-floor events.

Initial-publication endpoint→observation delay (simulated seconds) is small; later versions naturally refer back into the ~5 s source window:

| Source/mode | Initial-publication median | p95 | maximum |
|---|---:|---:|---:|
| GLIM OFF | 0.112457 | 0.116278 | 0.121212 |
| GLIM ON | 0.111454 | 0.122937 | 0.128762 |
| Kimera OFF | 0.012740 | 0.015299 | 0.018256 |
| Kimera ON | 0.012702 | 0.014206 | 0.015206 |

The audit joins `receipts.jsonl` to `stream_progress.csv` by event identity and wall time, rather than using an empty final queue as the sole throughput evidence:

| Progress diagnostic | OFF | ON |
|---|---:|---:|
| Receipt→consumed-progress wall delay median | 0.0522 s | 0.0596 s |
| Receipt→consumed-progress wall delay p95 | 0.2777 s | 0.3826 s |
| Receipt→consumed-progress wall delay maximum | 1.8697 s | 0.8473 s |
| Observed event backlog maximum at progress checkpoints | 132 | 69 |
| Support age at checkpoints, simulated-time median | 0.0500 s | 0.0510 s |
| Support age at checkpoints, simulated-time p95 | 0.0800 s | 0.0900 s |
| Support age at checkpoints, simulated-time maximum | 0.2300 s | 0.1300 s |
| Total revision-update compute | 68.2185 s | 75.6235 s |

Support age above is reconstructed from recorded progress checkpoints and latest adapter observation; it includes the support-creation cadence and is not a separately recorded ROS pose-publication latency trace. The last data event was consumed about 0.0003 s after its receipt in both runs; all beliefs were observed before the last IMU, and neither run required a long post-playback solver catch-up. The end marker was deliberately sent after source shutdown/drain, so total process wall time should not be interpreted as pure solver cost.

These results support keeping up on this 30 s segment at **0.1x**. They do not establish sustained full-sequence or 0.3x throughput: full-history graph size grows, and ON additionally recorded graph snapshots. The earlier estimate comparing 76 s revision compute with a 100 s budget at 0.3x is only an initial plausibility estimate, not a throughput validation.

## 6. Output and visualization meaning

The primary products remain original/native GLIM, original/native Kimera ROS odometry, and their gnssFGO fusion. The receiver additionally saves two distinct fusion histories:

- `online_trajectory.tum`: pose written once when each support is created. Later revisions update the internal graph, but do not rewrite old as-played output rows. `/gnssfgo/pose` and `/gnssfgo/path` use this product.
- `trajectory.tum`: final retrospectively revised graph after all received data. The Rerun bottom-left fusion trail uses this final product, revealed by physical timestamp. It must not be interpreted as what the live robot knew then.

The ON receiver graph snapshots record actual optimized historical states and source factor keys. Colored offsets are layout, not residuals or trajectory motion. Source graph panels show actual fixed-lag inspectors, while gnssFGO retains full history. No estimator was altered to change these views.

Existing scores were not recomputed. The 30 s reports use common nominal bounds but source-specific ATE sample times; Kimera has fewer native samples. All report 281 valid 1 s RPE pairs. For the full comparison, use the same common scoring mask/interpolation policy for OFF/ON and retain coverage counts. Do not infer an algorithmic correctness verdict from ATE improvement alone.

## 7. What is verified, and what is still a limitation

1. **Settings and implementation path:** saved profiles, command-line flags, receiving mode, model summary, transported data and actual factor event ledger agree with the intended recovered classic + independent receiver setup.
2. **Statistical model:** source factors are treated as separate Gaussian constraints. Shared IMU, overlapping source intervals and CBS feedback introduce correlations not explicitly modeled in the receiver. Native full covariance and stable-ID replacement do not remove cross-source/common-information correlation. This audit does not certify probabilistic consistency or attribute observed accuracy differences to this assumption.
3. **WNOJ auxiliary acceleration:** the existing implementation computes/caches acceleration inputs using estimates available at support creation. Later factor replacement does not rebuild all those auxiliary inputs. Therefore the same multiset of eventual beliefs in a different arrival order is not guaranteed to yield byte-identical results. Repeated identical journals are the relevant determinism contract; this behavior was already documented in the independent-receiver audit.
4. **Source timing:** classic Kimera uses `current`; central preintegration uses ALL_RIGHT. Preserving this matches the recovered experiment. Moving Kimera to RIGHT would be a separate controlled change, not a silent full-run launch correction.
5. **Run matching:** source profiles/flags match, but ON adds receiver graph snapshot recording. That does not change graph equations; it can affect CPU/IO contention and observed arrival order. Full matched runs should record the same graph outputs in both modes.
6. **Archival provenance:** receiver binary/transport hashes were saved and reverified. Source executable and shared-library hashes were not saved per capture; no retrospective byte-identical source-build claim is possible from HEAD/profile alone. Raw bag paths match, and central IMU payloads match exactly; retained logs do not provide a per-consumed-image/point-cloud checksum audit. Save executable/library and input manifests at the next authorized launch.
7. **Runtime:** 0.1x passed the bounded live check. Full-history memory, per-revision cost and graph-export volume grow; 0.3x/0.5x and full-sequence deadlines remain untested.

## 8. Existing regression evidence and audit non-regression

The retained `results/gnssfgo_live_receiver_20260928/final_build_equivalence/validation.json` reports byte-identical stream/offline output for seven trajectory/state/clock/model-diagnostic files with the same built receiver. `async_regression/validation.json` passes the independent support, version/session, endpoint, delayed-input, and future-suffix checks. `failure_checks/validation.json` passes disconnect, clock regression, invalid covariance, premature end and unavailable final-support cases. These are existing test artifacts, not freshly rerun tests in this audit. The earlier OFF report also records its 30 s journal repeat; no new ON 30 s repeat is claimed here.

New checks performed here: complete raw-log→journal payload comparison; covariance numerical validation; OFF/ON IMU identity; support-clock identity/spacing/startup; every dispatch causality condition; every factor slot replacement/count; latest revision retention; every ON graph snapshot source-key binding; source queued/applied/injected recount; complete profile comparison; manifest/binary hash verification; wall-time progress reconstruction. All passed. `git diff --check` passed for gnssFGO.

No production source, launch, configuration, old trajectory or recorded graph was changed. The workspace already has research changes and is not clean. gnssFGO base HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created. This report, its standalone analysis artifacts, and documentation pointers are the audit outputs.

## Full-run boundary

The current **classic source protocol plus existing asynchronous receiver** is suitable for extending the bounded experiment under the same frozen settings, with matched logging and explicit coverage/runtime monitoring. This is permission-independent readiness advice; no full run was launched. Preserve the current model and source settings, capture stronger provenance, keep OFF/ON instrumentation equal, and do not label 0.3x as validated without a bounded speed check.

**SETTINGS_AND_EVENT_SEMANTICS_VERIFIED_FOR_THE_30S_PILOT**
