# ETH1 source-WNOJ nominal profile: incremental validation

## Outcome

**The no-central-IMU model runs causally and deterministically with the nominal Step-B motion uncertainty, but its incremental implementation does not yet pass the declared tight convergence test at every publication.** All 150 same-information batch references converge. Incremental support-position differences remain below 1 mm, whereas occasional newest-state velocity differences reach 0.380 m/s OFF and 0.244 m/s ON.

There is a second, distinct limitation: some newest-state motion excursions remain in the converged references. They occur beyond the latest arrived source endpoint. Thus improving numerical convergence alone will not eliminate all provisional-output movement. This is not evidence that central IMU factors are mathematically indispensable, nor certification that this motion model is physically calibrated.

No GT, ATE selection, new uncertainty sweep, source-estimator run, production change or follow-up solver fix was used. The nominal profile remains an isolated provisional engineering configuration, not the official live baseline.

Evidence: [artifact directory](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006), [snapshot comparisons](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/snapshot_comparison.csv), [analysis](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/analysis.json), [compact supplementary results](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/report_summary.json), [verification](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/verification.json). Prior: [Step B](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_MOTION_UNCERTAINTY_STEP_B_20261006.md).

## 1. Scope and fixed configuration

This task tests numerical and causal validity, not OFF-versus-ON accuracy. Both modes replay the same saved eight-second ETH1 journals used in Step B:

```text
OFF /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/off_events.csv
SHA 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed

ON  /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/on_events.csv
SHA a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

Journal availability: `1727775008003475000`–`1727775016003475000 ns`. The 75 independent support states begin at `1727775008583475000 ns`, span 7.4 s, and have 100 ms spacing. The existing 59-timestamp startup clock anchor is unchanged. In `source_wnoj`, raw IMU values do not initialize or constrain the state: CPT7 timestamps supply the independent clock only. No central ALL_RIGHT integration occurs in this branch; the sources' established timing remains unchanged.

| Item | Configuration used |
|---|---|
| State per support | Pose X, world velocity V, body angular rate W, six-coordinate tangent acceleration A |
| Motion model | Existing `GPWNOJPriorFull` and full WNOJ endpoint interpolation |
| Process spectral density | `Qc = diag(2.5,2.5,2.5,10,10,10)` in angular/linear order |
| Interpretation | Acceleration-innovation standard deviations 0.5 rad/s² and 1 m/s² over 100 ms; not hard motion bounds |
| Initial conditions | Identity pose, zero V/W/A; existing prior sigmas: pose `1e-5`, V 10 m/s, W 3 rad/s, A 10 in its respective units |
| Receiver | Persistent full-history iSAM2, QR, relinearization threshold 0.01, skip 1; existing Gauss–Newton/wildfire behavior unchanged |
| Source factors | Original full six-dimensional means and dense covariances; scale one, no added uncertainty |
| Revision handling | One active factor per stable source ID; newer version removes/replaces the previous factor |
| Excluded | Central IMU factors, bias states, gravity input, GNSS, gating, robust losses, new correlations, source/CBS changes |

The Qc value was the pre-existing nominal engineering candidate, not chosen from this replay's results. It specifies a stochastic smoothness assumption; acceleration is estimated, not fixed or read from an accelerometer. Its physical suitability across missions is unestablished.

## 2. Actual incremental path and isolation

Unlike Step B, which changed Qc only in copied batch graphs, this diagnostic supplies nominal Qc to the actual continuing receiver from startup. The isolated [receiver copy](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/build/DiagnosticReplay.cpp:1210) accepts `WNOJ_INCREMENTAL_PROFILE=nominal`; the default remains baseline. Only this fixed Qc selection and corresponding diagnostic metadata differ from the retained Step-B receiver.

[IncrementalBackend](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/build/DiagnosticReplay.cpp:900) retains one iSAM2 instance. Support updates add state values and the stock WNOJ factor. Revision updates remove the previous slot and add the revised factor; their existing `force_relinearize` flag requests a relinearization check, not an unrestricted converged batch solve. No additional solver iterations or refinement policy were enabled.

At every noninitial support publication, the audit copies the actual graph, its current state and the active revision ledger. It also audits the final graph after the remaining arrivals within the eight-second cutoff: **75 snapshots per mode, 150 total**. This covers every support publication, not every internal revision callback. Batch solutions never feed back into the continuing receiver, its initialization of subsequent supports or its output.

The [snapshot helper](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/build/SnapshotAudit.h:164) verifies the selected full 18×18 WNOJ covariance against stock `calcQ3`, checks factor payload fingerprints before/after diagnostics, and saves:

- Incremental and converged X/V/W/A at identical physical timestamps and information sets.
- Factor keys, current source IDs/revisions, covariances, residuals, costs and normalized gradients.
- Dense poses every 10 ms using the stock interpolation model.
- Full-back-substitution estimates from an independent iSAM2 copy, without feedback.

For this constant-Qc model, Qc cancels from the conditional-mean interpolation gains, as analytically and numerically checked in Step B. The retained dense-sampling helper therefore produces the same conditional mean despite retaining its original isotropic Qc argument. It does not export interpolation uncertainty; no GP covariance is added to source R. The actual receiver's endpoint interpolators receive nominal Qc.

## 3. Criteria fixed before execution

[Pre-run protocol](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/PROTOCOL_BEFORE_RUN.md), SHA-256:

```text
8b0118d7244fd74d8390e86984769c2905f949a1b34070df6058985e3ac13693
```

Batch reference: existing Step-A/B LM Ceres defaults, multifrontal QR, at most 500 iterations, relative/absolute objective tolerance `1e-11`, then a maximum-100-iteration restart. Acceptance requires column-normalized gradient infinity norm ≤`1e-5`, restart maximum position change <`1e-6 m`, velocity change <`1e-4 m/s`, and neither iteration cap exhausted.

Incremental/reference agreement requires all of:

| Maximum difference | Threshold |
|---|---:|
| Position | `1e-4 m` |
| Orientation | `1e-4 rad` |
| Velocity / angular rate | `1e-3 m/s` / `1e-3 rad/s` |
| Linear / angular tangent acceleration | `1e-2 m/s²` / `1e-2 rad/s²` |
| Relative objective excess | `1e-6` |

Relative excess is `(incremental cost − reference cost)/max(1, reference cost)`. These are tight numerical agreement targets, not truth-error limits or calibrated physical constraints.

## 4. Reference convergence and incremental disagreement

All **150/150 references pass**. Main solves take 0–9 iterations, restarts 0–1. Maximum normalized gradients are `5.1121e-6` OFF and `1.2524e-6` ON. Maximum restart changes over both modes are `1.77e-7 m` position and `2.62e-6 m/s` velocity. This establishes local stationarity and repeatability, not global optimality.

Across the 18 snapshots shared with the prior Step-B nominal references, maximum differences are 1.34 µm position, `3.17e-6 rad` orientation, `2.04e-5 m/s` velocity and `1.64e-4 m/s²` linear tangent acceleration. Different initial guesses produce closely agreeing, not byte-identical, converged references.

Each maximum in the next table is over every state in all 75 snapshots of that mode. The maxima need not occur in the same snapshot.

| Incremental versus converged reference | OFF | ON |
|---|---:|---:|
| Snapshots passing every tight target | 7/75 | 7/75 |
| Position difference, mm | 0.964576 | 0.783935 |
| Orientation difference, rad | 0.000224573 | 0.000246052 |
| Velocity difference, m/s | 0.380120 | 0.243873 |
| Angular-rate difference, rad/s | 0.006257 | 0.002930 |
| Linear tangent acceleration difference, m/s² | 0.286859 | 0.209429 |
| Angular tangent acceleration difference, rad/s² | 0.063673 | 0.018448 |
| Dense interpolated position difference, mm | 7.163768 | 4.446204 |
| Relative objective excess | 0.260833 | 0.199537 |
| Incremental normalized gradient | 59.2318 | 32.0944 |

All maximum support-state differences occur at the newest state in their respective snapshots. The first failed snapshot is support 8 in both modes, initially **only the tight objective criterion**. First state-coordinate failure is support 32: velocity differences 0.003974 m/s OFF and 0.004834 m/s ON. Accordingly, 7/75 is not a count of catastrophic filter failures.

Concrete largest velocity differences:

| Case | Support / physical timestamp ns | Incremental V, m/s | Reference V, m/s | Norm difference |
|---|---|---|---|---:|
| OFF | 60 / `1727775014583475000` | `(0.297992, 0.196217, 0.026314)` | `(0.207827, 0.148382, 0.392474)` | 0.380120 m/s |
| ON | 58 / `1727775014383475000` | `(-0.149627, 0.102646, -0.194838)` | `(-0.102663, 0.297064, -0.055303)` | 0.243873 m/s |

Their information-availability bounds are respectively `1727775014583476000` and `1727775014383476000 ns`. These compare the same graph at the same physical time, not a propagated pose against an earlier pose. Their objectives decrease from 14666.2003 to 12785.7214 OFF and 6802.8865 to 6051.6885 ON.

Full back-substitution alone does not remove these worst differences: its maxima against batch remain identical. It does change some smaller values (maximum position change versus ordinary `calculateEstimate()` is 3.04 µm OFF and 2.21 µm ON; velocity changes <0.000100 m/s). Thus truncated back-substitution alone is not the explanation. This task does **not** isolate which relinearization/iteration policy would resolve the remaining discrepancy, and changes none.

Final states are much closer than some intermediate publications:

| Final-graph maximum difference | OFF | ON |
|---|---:|---:|
| Position, mm | 0.049755 | 0.026580 |
| Orientation, rad | 0.000028393 | 0.000004094 |
| Velocity, m/s | 0.000704 | 0.000635 |
| Linear tangent acceleration, m/s² | 0.013108 | 0.007623 |
| Relative objective excess | `2.887e-5` | `2.537e-6` |

A close final graph does not validate every earlier published estimate.

## 5. A separate issue: the provisional prediction tail

Auditing every support publication reveals events missed by Step B's nine selected snapshots. The previous nominal maximum speeds of 0.584/0.465 m/s were explicitly sampled maxima, not whole-replay bounds.

| All-publication motion diagnostic | OFF | ON |
|---|---:|---:|
| Largest support speed, incremental / converged, m/s | 1.845675 / 1.845760 | 2.642435 / 2.642926 |
| Largest linear tangent acceleration, converged, m/s² | 9.327592 | 11.550327 |
| Maximum-speed support | 59 | 61 |
| Its physical timestamp ns | `1727775014483475000` | `1727775014683475000` |
| Latest arrived source endpoint ns | `1727775014355358600` | `1727775014555357500` |
| Newest state beyond that endpoint | 128.1164 ms | 128.1175 ms |
| Largest converged 10 ms speed at/before latest arrived endpoint, m/s | 0.587647 | 1.085750 |
| Largest departure within wholly endpoint-covered support intervals, mm | 5.404677 | 7.900247 |

“Endpoint-covered” here only means before the latest endpoint of an active, already-arrived source factor. It does not imply complete independent measurement coverage or that all future revisions have arrived.

The largest speed peaks persist almost unchanged after numerical convergence. All intervals exceeding the descriptive 1 cm departure threshold occur outside the set of wholly endpoint-covered intervals: four snapshot-interval occurrences OFF and two ON. Maximum departures over all intervals are 11.664 mm OFF and 14.486 mm ON. Some of those intervals straddle the latest measurement endpoint.

This locates a remaining issue in the provisional newest part of the trajectory, where the model must predict before new information arrives. It does not prove the true platform violated any motion limit; no GT or hardware acceleration bound was used. Nor does it establish whether source disagreement, derivative observability, or the assumed motion statistics dominate that behavior.

The distinction also matters for path length:

| Path definition, same 75 support timestamps | OFF, m | ON, m |
|---|---:|---:|
| Actual as-published newest-state sequence | 3.309205 | 3.109097 |
| Newest-state sequence from independently converged same-information snapshots | 3.308472 | 3.107042 |
| Final retrospectively revised incremental graph | 0.863926 | 0.874406 |
| Final converged graph | 0.863946 | 0.874405 |
| Final converged curve sampled every 10 ms | 0.881479 | 0.896816 |

The first two rows stitch successive provisional publications together; they are **not** one final coherent graph or true traveled distances. Even converging each publication leaves almost the same longer published polyline. Therefore the online/final difference is not explained primarily by the measured incremental convergence error. We have not silently substituted the smooth final path for live/as-played output.

## 6. Source fit and model-validity limits

The unchanged source residuals still show the smoothness/source tension seen in Step B. Final converged squared whitened residual per source coordinate is 46.513 GLIM / 92.877 Kimera OFF and 19.426 GLIM / 71.345 Kimera ON. These are post-fit residual diagnostics, not independent innovation NIS or percentages of influence. They do not identify the source of error or establish covariance calibration.

The model has a fixed pose gauge, proper original derivative priors and finite outputs. The smallest column-normalized reference information eigenvalue over these snapshots is positive, about `2.94e-9`; that also signals weak numerical directions and is not a general observability or conditioning certificate. Shared information through the source IMUs and classic CBS remains; removing central IMU factors does not make the source beliefs independent.

The defensible conclusion is narrower than “fully validated”: a coherent, executable experimental graph with preserved event semantics and converged local references exists. Numerical publication accuracy and behavior while waiting for source measurements remain open. No claim is made about ATE, cross-sequence generalization, real-time throughput or physical uncertainty calibration.

## 7. Validation and preservation

- Isolated Release build succeeded using existing libraries; no installation or dependency change.
- Primary and repeated OFF/ON receiver outputs are byte-identical. All 150 repeated snapshot numerical products match; timing fields are excluded where applicable.
- Original-Qc regression reproduces all twelve protected Step-B receiver output/accounting products and the previous baseline snapshot products exactly, excluding runtime fields.
- Each mode processes 2572 source versions: 93 initial inserts, 2455 replacements, 24 unsupported messages unchanged. Final source IDs: 71 GLIM and 22 Kimera. No stale or duplicate revisions, no pending messages at end.
- Every accepted event satisfies arrival ≤ insertion, required support exists and its clock availability ≤ insertion. Twenty-two arrivals per mode wait for support creation; maximum additional wait is about 14.023 ms.
- Six-second prefix controls preserve retained input rows exactly. Their first 55 published poses, state rows, clock and acceleration outputs match the eight-second run's prefix byte-for-byte. Later input does not influence earlier output.
- All 36 reported/effective source covariance entries remain equal. Recovered Gaussian factor covariance roundoff is at most `1.53e-21` OFF and `7.63e-21` ON. No scaling or noise addition.
- Final graphs: 4 priors + 74 WNOJ + 93 relative factors; 300 keys, 1350 scalar coordinates. Zero central IMU factors and zero bias variables.
- All 3290 protected files retain their original hashes, including production source/configuration, prior experiments, native trajectories and reference inputs. The existing dirty worktree is preserved, not declared clean.
- This task ran the focused Release replay/regression checks, not a new complete production test suite or sanitizer campaign.

## 8. Reproduction and provenance

Artifact root:

```text
/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006
```

The [driver](/home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/run.py) records exact compile/run argv in `commands_build.json`, `commands_primary.json`, `commands_repeat.json`, `commands_regression.json` and `commands_prefix6.json`; console logs are under `logs/`. The commands used the existing image and read-only source/runtime mounts:

```bash
docker run --rm --name source-wnoj-incremental-nominal-primary \
  --network none --read-only --user 1000:1000 \
  --tmpfs /tmp:rw,exec,size=2g -e LD_LIBRARY_PATH=/runtime/lib \
  -v /home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006:/audit \
  -v /home/yeranis/repos/gnssFGO:/source:ro \
  -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005:/previous:ro \
  sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d \
  python3 /audit/run.py primary
```

The same container invocation with corresponding name/action was used for `build`, `repeat`, `regression` and `prefix6`. The driver runs both OFF and ON. It sets nominal Qc for primary/repeat/prefix6 and original Qc for regression. Primary/repeat audit indices 1–74 plus final; regression audits the previous eight selected indices plus final. These commands describe completed runs: logs open exclusively and outputs must be preserved, so reproduction requires a fresh artifact copy/destination.

Primary receiver arguments:

```text
WNOJ_INCREMENTAL_PROFILE=nominal
WNOJ_AUDIT_INDICES=1,2,...,74
/audit/build/nominal_replay --receiver-model source_wnoj --mode sources
  --timeline record --solver isam2 --support-policy uniform_imu
  --support-period-ns 100000000 --receiver-startup-samples 59
  --event-stream /previous/inputs/{off,on}_events.csv
  --out /audit/{off,on}_primary
```

Analysis and checks:

```bash
/home/yeranis/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  /home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/analyze.py
/home/yeranis/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  /home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/report_summary.py
/usr/bin/python3 \
  /home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006/verify.py
```

Build wall time: 35.78 s. Instrumented primary OFF/ON: 94.66/90.07 s; repeats: 92.92/87.55 s. These include 75 copied-graph solves, restarts, dense linear-algebra diagnostics and file output. Some validation processes overlapped. They are not live receiver throughput benchmarks.

Executable SHA-256: `cb5d4b57d3dad7d68b2977e4233182c69987dd28636fd9aaa2d57d919e4d362b`. Full hashes and preservation manifest are retained in the artifact directory.

Repository HEAD before/after: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. Diagnostic implementation lives outside the repository; this report is the only new repository file from this task. `git diff --check` passes; pre-existing changes are untouched.

## 9. Disposition

Retain this nominal profile as a provisional diagnostic model, not a promoted production baseline. The next engineering question is how to make incremental publication adequately solve this fixed graph, while separately characterizing the still-provisional prediction tail. Neither issue should be concealed by feeding offline results into the live output or adjusting uncertainty until a preferred result appears.

No further solver change, Qc search, full sequence or estimator rerun was performed.

**NUMERICAL_PUBLICATION_VALIDATION_INCOMPLETE**
