# ETH1 source WNOJ publication convergence

## Result and scope

The isolated no-central-IMU receiver now meets the existing numerical convergence targets at all **150 checked outputs** in the saved eight-second ETH1 CBS-OFF and CBS-ON replays. Previously only 14 of those 150 outputs met every tight target. The change makes iSAM2 finish solving the same current graph before publishing; it does not change the motion model, source beliefs or their weights.

This resolves the measured numerical publication discrepancy on this short test, not the entire model-validation problem. Motion peaks in the newest predicted states remain after convergence. Full eight-second repeats are byte-identical, but an unaudited six-second control has tiny repeat-to-repeat differences, documented below. The prototype is not promoted to the production live baseline.

In plain words: previously the optimizer sometimes published an unfinished answer. It now reaches an answer that agrees closely with an independently converged solution using exactly the same available information. That answer can still move undesirably while waiting for the next source measurement. Solving the graph accurately and having a physically suitable graph are different requirements.

Evidence: [artifacts](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006), [snapshot comparisons](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/snapshot_comparison.csv), [convergence statistics](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/convergence_analysis.json), [verification and caveats](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/verification.json). Prior: [nominal incremental validation](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_NOMINAL_INCREMENTAL_VALIDATION_20261006.md).

## Frozen inputs and model

The input journals are unchanged:

```text
OFF /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/off_events.csv
SHA 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed
ON  /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/on_events.csv
SHA a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

Journal availability spans `1727775008003475000`–`1727775016003475000 ns`. There are 75 supports at 100 ms spacing, beginning at `1727775008583475000 ns`, with 7.4 s output span. The existing 59-timestamp startup clock anchor is unchanged.

| Component | Unchanged configuration |
|---|---|
| Support state | Pose X, world velocity V, body angular rate W, six-coordinate tangent acceleration A |
| Motion factors and interpolation | Existing full WNOJ implementation with estimated acceleration |
| Qc diagonal | `[2.5,2.5,2.5,10,10,10]`, angular then linear |
| Initial mean | Identity pose and zero V/W/A |
| Initial prior sigmas | Pose `1e-5`, V `10 m/s`, W `3 rad/s`, A `10` in its respective units |
| Solver | Persistent full-history iSAM2, QR, Gauss–Newton; ordinary threshold `0.01`, skip `1` |
| Source measurements | Full GLIM and Kimera relative means and dense covariance, scale one |
| Source revisions | Stable-ID removal and replacement, one active version per ID |
| Timing | Original arrival journal and independent support clock |
| Absent | Central IMU factors, bias variables, gravity input, GNSS, robust losses, gating or added source uncertainty |

The receiver uses IMU timestamps as a clock, not accelerometer/gyro values as measurements. Therefore it does not perform central ALL_RIGHT integration in this branch. Source-estimator timing and classic source-to-source CBS are unchanged. No source estimator was rerun. These are saved-journal receiver replays, not three live co-running estimators.

Qc remains the prior provisional engineering value, not a calibrated cross-sequence noise model. Removing central IMU factors also does not eliminate correlations between the source beliefs.

## Numerical change

The change exists only in the [diagnostic receiver copy](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/build/DiagnosticReplay.cpp:995) and [refinement helper](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/build/FullPublicationRefinement.h). Production source and launch defaults are untouched. The diagnostic switch is enabled by setting `WNOJ_STRICT_PUBLICATION=1`; its implementation checks environment-variable presence, so **unset it to disable it**, not `=0`.

Immediately before each noninitial support publication and final export:

1. Evaluate the current nonlinear objective and column-normalized gradient.
2. If the gradient exceeds `1e-5`, copy the continuing iSAM2 solver and request an empty update with both `force_relinearize=true` and `forceFullSolve=true`.
3. Obtain the candidate with full back-substitution. Check that factor slots and identities are unchanged and that the objective is finite and nonincreasing.
4. Commit the candidate only after successful checks. Repeat until stationary, rejected or the predeclared 20-update cap is reached.

No batch solution is fed into the receiver. No factors are added by refinement. Revisions and future state initialization continue through the existing receiver path. A tiny objective decrease alone is not accepted as convergence while the gradient remains too large.

In the **actual frozen runtime**, [GTSAM relinearization selection](/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/include/gtsam/nonlinear/ISAM2-impl.h:386) explicitly applies a zero threshold when `forceFullSolve` is set. In contrast, `force_relinearize` alone requests a check but retains the usual delta threshold. A focused test verifies the runtime behavior with a deliberately large ordinary threshold. All 177 real-data refinement steps also report exactly `4 × (support index + 1)` relinearized variables: the complete current X/V/W/A state set.

This is a full-relinearization diagnostic using existing GTSAM operations, not a new graph optimizer or a claim that all future live updates should unconditionally rebuild the full linearization. The iteration cap bounds update count, not execution time or memory. Copying and relinearizing full history grows more expensive as history grows.

Failure behavior is diagnostic: a rejected candidate rolls back; reaching the cap or an exception is logged explicitly. The last accepted estimate would still be recorded with its nonstationary status. This is **not yet a production publication gate that suppresses failed outputs**. No real-data candidate was rejected or reached the cap in this test.

## Criteria and reference independence

The [protocol written before execution](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/PROTOCOL_BEFORE_RUN.md) has SHA-256 `79459454e77dc176bd14459aefe31a39158d09a37e1419b7893d0f0a9bcc169a`.

At each of 74 support publications plus final export per mode, an independent copied-graph LM solve and restart use the previous validation settings: multifrontal QR, up to 500 main and 100 restart iterations, and `1e-11` objective tolerances. The reference must have normalized gradient ≤`1e-5`, restart position change <`1e-6 m`, velocity change <`1e-4 m/s`, and neither iteration cap exhausted.

Unchanged incremental/reference thresholds are position `1e-4 m`, orientation `1e-4 rad`, V/W `1e-3` in their respective units, linear/angular tangent acceleration `1e-2`, and relative objective excess `1e-6`. Relative excess divides the cost difference by `max(1, reference cost)`. These test numerical agreement, not ground-truth accuracy, uncertainty calibration or global optimality.

The new reference starts from the refined state. To avoid relying only on a reference initialized near that state, the refined results are also compared with the retained prior nominal batch references, which used different initialization histories. Both comparisons satisfy the original targets. Factor ledgers, types, keys and covariance blocks match the prior same-information graphs.

## Convergence results

Each maximum below covers every state in all 75 snapshots of that mode; maxima need not occur together. Differences are between same-time, same-information estimates.

| Numerical quantity | CBS-OFF | CBS-ON |
|---|---:|---:|
| Outputs passing every target before this change | 7/75 | 7/75 |
| Outputs passing every target now | 75/75 | 75/75 |
| Maximum position difference from new reference, m | `2.11e-8` | `9.12e-9` |
| Maximum orientation difference, rad | `3.94e-8` | `3.37e-8` |
| Maximum velocity difference, m/s | `8.21e-7` | `1.77e-7` |
| Maximum angular-rate difference, rad/s | `1.51e-6` | `9.07e-7` |
| Maximum linear tangent acceleration difference, m/s² | `7.85e-6` | `1.94e-6` |
| Maximum angular tangent acceleration difference, rad/s² | `1.63e-5` | `9.68e-6` |
| Maximum relative objective excess | `9.99e-13` | `1.88e-12` |
| Maximum publication normalized gradient | `7.96e-6` | `6.84e-6` |
| Publications needing 0 / 1 / 2 extra updates | 4 / 51 / 20 | 4 / 56 / 15 |
| Rejections or cap hits | 0 | 0 |

The previous maximum velocity discrepancy was `0.380120 m/s` OFF and `0.243873 m/s` ON. It is now below `1e-6 m/s` against the newly converged references. Previously observed sub-millimetre position differences also disappear to the displayed numerical scale. Dense interpolated position comparisons at 10 ms satisfy the same position target.

Against the **older independently initialized references**, maximum position differences are `6.17e-8 m` OFF and `1.70e-6 m` ON; velocity differences are `7.72e-7 m/s` and `2.18e-5 m/s`. Other coordinates also pass, with the largest ON angular tangent acceleration difference `6.18e-4 rad/s²`. This is close numerical agreement, not a claim that independently initialized nonlinear solves are byte-identical.

The evidence supports an insufficient numerical solve/relinearization policy as the cause of the earlier incremental-versus-converged discrepancy. It does not identify covariance calibration as that cause, and it does not establish that the converged motion is physically correct.

## Remaining prediction behavior

The largest speeds and accelerations remain in the converged graph:

| Diagnostic | CBS-OFF | CBS-ON |
|---|---:|---:|
| Maximum support speed, m/s | 1.845760 | 2.642926 |
| Maximum linear tangent acceleration, m/s² | 9.327592 | 11.550327 |
| Support of maximum speed | 59 | 61 |
| Newest state beyond latest arrived source endpoint | 128.1164 ms | 128.1175 ms |
| As-published newest-state polyline, m | 3.308472 | 3.107039 |
| Same polyline from independently converged snapshots, m | 3.308472 | 3.107039 |
| Final retrospectively revised support path, m | 0.863946 | 0.874406 |

The as-published polyline joins provisional estimates from successive information sets. It is not one final coherent trajectory or the true traveled distance. The numerical change barely changes that polyline, although it fixes significant temporary velocity differences. Thus incomplete optimization was not the main cause of the online/final path-length gap.

At the speed peaks the receiver is estimating motion beyond the latest available source endpoint. Future source information and revisions subsequently alter those states. Convergence alone cannot supply missing measurements. This localizes a remaining prediction issue but does not yet separate the effects of source disagreement, weak derivative constraints and motion-model suitability. It does not prove that central IMU measurements are necessary.

No ground truth was used and no ATE comparison was made. This test makes no claim that CBS-ON is more accurate than CBS-OFF.

## Validation and repeatability caveat

The focused Release tests pass **7/7**, covering stationarity, bypassing the usual relinearization threshold, iteration limits, objective-increase rollback, exception rollback, stable factor slots across later revisions, and invalid policies. The same seven tests pass with ASan/UBSan. Existing runtime libraries are not themselves sanitizer-instrumented; `EIGEN_MALLOC_ALREADY_ALIGNED=1` matches their allocation ABI. This is a focused test campaign, not a fresh complete production suite.

Stationary, constant-velocity and constant-acceleration synthetic fixtures pass. Their maximum position errors are respectively 0, `9.24e-7 m` and `3.29e-6 m`. Replacing all raw IMU values while keeping their timestamps produces identical fixture outputs, confirming that this branch does not consume raw IMU motion.

All full eight-second primary/repeated numerical outputs and all 150 snapshot products match exactly, excluding runtime fields. Full runs without the independent batch snapshots also repeat exactly and match the primary trajectory/state products. With the new refinement switch unset, the old nominal outputs and snapshots are reproduced exactly.

There is a narrower repeatability caveat that prevents a universal bitwise-determinism claim:

- The initial verifier failed a byte-equality assertion comparing an unaudited six-second prefix with the audited eight-second run.
- A matching audited six-second prefix reproduces all first 55 publications exactly, including states, trajectory, support clock and accelerations.
- The unaudited six-second prefix differs slightly from the unaudited eight-second prefix, and a repeat of the unaudited six-second execution also differs slightly. A second strict repeat assertion therefore failed.
- Maximum published-state differences are `7.50e-9 m`, `1.71e-7 m/s`, `1.42e-6 m/s²` OFF, and `2.95e-9 m`, `4.18e-8 m/s`, `4.22e-7 m/s²` ON. Orientation differences are ≤`1.31e-9 rad`. All are below the original numerical thresholds; none of those thresholds was relaxed.
- Support clocks, source covariance products, source accounting and causal dispatch through the last compared publication match exactly. The low-level cause of the tiny repeated six-second numerical variation is not isolated. It is not established to be future-data leakage or a mutation caused by diagnostics.

The final verifier records the failed byte-equality conditions explicitly and accepts only the already-declared numerical agreement for that control. It does not relabel those products byte-identical. The result is numerical validation on this bounded test with a repeatability caveat, not unrestricted production certification.

For each mode, source accounting remains 2572 versions: 93 first insertions, 2455 replacements and 24 unsupported messages. Final IDs are 71 GLIM and 22 Kimera, with no stale/duplicate revisions or pending messages. Every accepted arrival precedes its insertion and all required supports are available. Twenty-two arrivals wait for support creation, at most about 14.023 ms.

All 36 effective source covariance entries equal their reported values. Final graphs retain four priors, 74 WNOJ factors and 93 relative factors, with 300 keys and 1350 scalar coordinates. There are zero central IMU factors and zero bias variables. All **9585 protected files** retain their hashes, including prior results and production configuration/source.

## Computational cost

Primary refinement cost includes solver copying, objective/gradient checks and accepted updates, but excludes the independent batch-reference snapshots:

| Refinement cost | CBS-OFF | CBS-ON |
|---|---:|---:|
| Median per checked output, ms | 7.403 | 6.336 |
| p95, ms | 23.445 | 30.704 |
| Maximum, ms | 26.173 | 33.870 |
| Total, s | 0.660 | 0.785 |

Without batch snapshots, the complete eight-second journal replay took 5.43 s OFF and 5.88 s ON; repeats took 5.63/5.73 s. Disabled-policy controls took 5.02/5.07 s. Some processes overlapped, so these descriptive times are not a controlled speedup benchmark. Fully instrumented primary replays took 87.59/124.19 s, dominated by independent reference solves and diagnostics.

An eight-second offline replay does not demonstrate sustained real-time performance of full-history relinearization alongside GLIM and Kimera. No live-capacity setting is changed.

## Reproduction and provenance

Artifact root:

```text
/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006
```

[run.py](/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/run.py) records exact compile/run argv in timestamped `commands_<action>_*.json`; stdout/stderr are retained under `logs/`. The completed actions are `unit`, `sanitizer`, `build`, `fixtures`, `primary`, `repeat`, `disabled`, `prefix6`, `prefix6_audited`, `prefix6_repeat`, `timing`, `timing_repeat` and `timing_disabled`.

```bash
docker run --rm --network none --read-only --user 1000:1000 \
  --tmpfs /tmp:rw,exec,size=2g -e LD_LIBRARY_PATH=/runtime/lib \
  -v /home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006:/audit \
  -v /home/yeranis/repos/gnssFGO:/source:ro \
  -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005:/previous:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_incremental_nominal_20261006:/nominal:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_convergence_20261006/integration:/fixtures:ro \
  sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d \
  python3 /audit/run.py primary
```

Substitute the listed action to reproduce the corresponding operation, using a fresh output destination to preserve artifacts. The driver sets the nominal profile and strict-publication switch; `disabled` and `timing_disabled` leave the latter unset. Primary/repeat/disabled request snapshots 1–74 plus final. Receiver arguments are unchanged:

```text
--receiver-model source_wnoj --mode sources --timeline record --solver isam2
--support-policy uniform_imu --support-period-ns 100000000
--receiver-startup-samples 59
--event-stream /previous/inputs/{off,on}_events.csv
--out /audit/{off,on}_primary
```

Analysis uses the retained `analyze.py`, `convergence_analysis.py`, `report_summary.py` and `verify.py`. NumPy scripts use `/home/yeranis/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3`; the verifier uses system Python. Numerical results, source/factor invariants and the caveat are saved in JSON rather than inferred from screenshots.

Release receiver build: 35.70 s. Executable SHA-256: `6d3163f08263051e7524af07d93651afac9d6ed551c8623ae6d3e2b3a8f91ced`.

Repository HEAD before/after: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. All implementation changes are in the new artifact directory; this report is the only new repository file from this task. The pre-existing dirty worktree is preserved and is not claimed clean. `git diff --check` passes.

## Disposition

Keep this as an isolated numerical-policy candidate. The measured convergence deficiency is resolved on the short frozen test; the prediction tail, physical calibration, scaling to long histories and the narrow bitwise-repeatability caveat remain open. Production settings, source estimators and covariance weights remain unchanged. No next experiment was started.

**NUMERICAL_CONVERGENCE_TARGETS_PASSED_WITH_REPEATABILITY_CAVEAT**
