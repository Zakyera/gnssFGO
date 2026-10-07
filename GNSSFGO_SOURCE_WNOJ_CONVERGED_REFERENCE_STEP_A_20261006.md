# ETH1 source WNOJ converged reference

Step A establishes a converged local reference for the unchanged no-central-IMU graph. **All 18 sampled graphs converge and reproduce deterministically, but substantial derivative excursions and motion between support poses remain.** Incomplete incremental optimization is one problem; it is not the complete explanation. Converging this objective does not, by itself, make the current source-driven model a validated replacement for the central-IMU baseline.

This is an eight-second frozen-input diagnostic, not a new native GLIM/Kimera run, a full-sequence accuracy comparison, or a parameter search. Neither the production implementation nor its settings changed. Step B and Step C were not run.

Artifacts: [gnssfgo_source_wnoj_step_a_20261006](/home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006). Machine-readable evidence is in [analysis.json](/home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006/analysis.json), [snapshot_summary.csv](/home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006/snapshot_summary.csv), [geometry.json](/home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006/geometry.json) and [verification.json](/home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006/verification.json).

## Scope and frozen information

The input journals are the exact prefixes retained by the [first-spike diagnostic](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_FIRST_SPIKE_DIAGNOSTIC_20261005.md):

```text
OFF /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/off_events.csv
SHA 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed

ON  /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/on_events.csv
SHA a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

Each journal spans `1727775008003475000` through `1727775016003475000 ns`. The independent clock initializes at `1727775008583475000 ns`, producing 75 supports at 100 ms spacing over 7.4 seconds. The raw IMU values are not measurements in this model; only their timestamps provide the clock. There is no central Combined IMU factor, bias state or gravity initialization.

Frozen settings are X/V/W/A states; stock `GPWNOJPriorFull` and `GPWNOJInterpolatorFull`; estimated acceleration; Qc diagonal 10000; source covariance scale one with all dense covariance entries preserved; original source means, timestamps, arrivals and stable-ID revision replacement; and original startup priors. Initial pose is identity with sigma `1e-5` in all six coordinates. V, W and A start at zero with sigmas 10 m/s, 3 rad/s and 10 in their respective tangent acceleration coordinates. The 59-sample clock anchor is unchanged; it does not average IMU measurements for this experimental model.

Both modes contain 800 clock rows and 2572 belief versions. They retain 93 source IDs, comprising 71 GLIM and 22 Kimera factors, after 2548 accepted insertions including 2455 replacements. The same 24 unsupported messages remain excluded; there are no duplicate or stale revisions. The final graph has four startup priors, 74 WNOJ factors and 93 relative factors: 171 active factors, 300 keyed variables and 1350 scalar coordinates. No additional gate or source selection was introduced.

The prior diagnostic examined six mode-specific snapshots around first spikes. Step A extends it to **eight matched snapshot times per mode**, at support indices 10, 20, 30, 38, 48, 58, 68 and 74, plus each final graph after the remaining arrivals within the eight-second journal. This gives 18 references. The final graph is distinct from the online output at support 74: a few additional source revisions arrive after that support output but before the journal cutoff.

## Reference construction

The existing diagnostic receiver and snapshot helper were copied into the new artifact directory. Only read-only diagnostic additions were made there: capture the final graph after the remaining arrivals, and sample the converged continuous-time interpolation every 10 ms. The repository receiver was not edited. The optional bounded empty-update refinement from the previous task is not enabled; the comparator is the original incremental receiver whose outputs have already been frozen.

For every selected event:

1. Preserve the actual active factor objects, slots, keys, latest revisions, covariances and available state values. No graph is reconstructed from final trajectories.
2. Copy the graph, initialize from the same-time incremental estimate, and solve using existing GTSAM Levenberg–Marquardt with Ceres defaults, multifrontal QR, maximum 500 iterations and relative/absolute objective tolerances `1e-11`.
3. Restart LM from that result, with maximum 100 iterations, to test whether the solution moves further.
4. Independently linearize the final objective and check the column-normalized gradient:

   `max_i abs((Jᵀr)_i) / sqrt((JᵀJ)_ii)`.

5. Accept a local reference when the normalized gradient is at most `1e-5`, the restart changes position by less than `1e-6 m` and velocity by less than `1e-4 m/s`, and neither optimization exhausts its cap.

Batch results never feed back into the causal receiver or its subsequent initial guesses. Every incremental-versus-batch comparison uses the **same physical timestamp, graph and information set within that mode**. Snapshot ledgers verify that accepted source arrivals do not exceed the corresponding availability bound or the eight-second cutoff. OFF and ON contain different source beliefs, so their different costs are not an accuracy ranking.

All 18 graphs pass. Main LM iterations range from 2 to 15. Normalized final gradients range from `5.06e-10` to `1.23e-6`. Maximum changes caused by the restart, across all supports in all snapshots, are:

| Quantity | Maximum restart change |
|---|---:|
| Position norm | 3.09e-7 m |
| Orientation angle | 3.88e-8 rad |
| World velocity norm | 2.92e-6 m/s |
| Body angular rate norm | 3.20e-7 rad/s |
| Translational tangent acceleration norm | 2.34e-5 m/s² |
| Angular tangent acceleration norm | 2.30e-5 rad/s² |

These checks establish a reproducible stationary local solution, not global optimality or physical correctness. The normalized information spectrum remains weakly conditioned; small gradients alone do not establish strong derivative observability.

## Same time velocity results

Time is relative to the start of the eight-second journal, not the full bag. Entries are the newest support's estimated velocity norm, not a finite difference of successive published graph outputs.

| Time s | OFF incremental m/s | OFF converged m/s | ON incremental m/s | ON converged m/s |
|---|---:|---:|---:|---:|
| 1.58 | 0.01791 | 0.01789 | 0.01491 | 0.01493 |
| 2.58 | 0.04459 | 0.04457 | 0.08741 | 0.08738 |
| 3.58 | 0.05709 | 0.05723 | 0.04191 | 0.04205 |
| 4.38 | 6.12118 | 4.52361 | 1.00184 | 0.98135 |
| 5.38 | 19.45910 | 10.03905 | 2.10993 | 2.07987 |
| 6.38 | 4.12597 | 3.95578 | 7.78498 | 4.78320 |
| 7.38 | 16.13852 | 10.99199 | 6.52347 | 5.67177 |
| 7.98 before final arrivals | 6.76701 | 5.69977 | 2.45227 | 1.89755 |
| 7.98 after final arrivals | 1.07084 | 1.07054 | 0.49704 | 0.49578 |

Examples of the accompanying objective reduction, where objective means half the sum of squared whitened factor residuals:

| Mode and time | Incremental objective | Converged objective |
|---|---:|---:|
| OFF at 4.38 s | 29200.329 | 10.680 |
| OFF at 5.38 s | 625079.997 | 230.452 |
| OFF at 7.38 s | 950706.098 | 712.187 |
| ON at 6.38 s | 4201.287 | 72.388 |
| ON at 7.38 s | 8220.271 | 161.239 |

Thus there is real incomplete-optimization error, particularly in derivative states. Yet convergence still leaves a newest estimated speed of 10.99 m/s and tangent acceleration of 100.55 m/s² in the sampled OFF graphs, and 5.67 m/s and 51.96 m/s² in ON. These maxima describe the sampled snapshots, not every timestamp in the stream.

Position changes from incremental to converged solutions are much smaller than the velocity changes: at most 2.44 mm OFF and 2.35 mm ON across these snapshots. Therefore a visually similar support-pose trajectory does not imply correctly solved velocity/acceleration states.

## Motion within a single converged graph

The final solution was evaluated using the **same stock WNOJ conditional-mean pose interpolation as the relative factors**, at 10 ms intervals between its 100 ms support poses. Interpolated values at support timestamps match the optimized support positions to below `1e-10 m`. These are samples of one coherent graph solution, not a path formed by mixing past publications.

| Final eight-second-prefix graph diagnostic | OFF | ON |
|---|---:|---:|
| Path through optimized 100 ms support poses | 1.722683 m | 1.185825 m |
| Path through the same solution sampled every 10 ms | 6.162684 m | 3.061473 m |
| Maximum estimated support speed | 4.085174 m/s | 2.298254 m/s |
| Maximum 10 ms position secant speed | 4.211752 m/s | 2.275004 m/s |
| Maximum translation tangent acceleration state | 47.581280 m/s² | 39.941461 m/s² |
| Maximum world position second difference at 10 ms | 121.883854 m/s² | 66.449884 m/s² |
| Intervals with over 1 cm departure from linear interpolation between support poses | 47 of 74 | 44 of 74 |

The path values are sampled polyline lengths, not exact integrated arc length and not GT errors. The acceleration-state column uses the existing full-WNOJ tangent derivative; the world-position second difference is a separate geometric diagnostic. They are not interchangeable raw accelerometer readings.

Two concrete internal intervals illustrate what sparse pose output hides:

- **OFF supports 47 to 48:** timestamps `1727775013283475000` to `1727775013383475000 ns`. Endpoint separation is 6.71 cm, but the sampled internal path travels 24.83 cm. Its maximum departure from the position obtained by linear interpolation at the same time is 12.27 cm. Projected along the direction from the first endpoint to the second, it initially moves backwards by 9.01 cm, then returns and ends 6.71 cm forwards. This is a concrete reversal inside one 100 ms interval.
- **ON supports 59 to 60:** timestamps `1727775014483475000` to `1727775014583475000 ns`. Endpoint separation is 2.22 cm, while the sampled path travels 13.34 cm, with maximum same-time departure of 6.53 cm.

Both intervals are in historical time, before the latest already-arrived source endpoint. Consequently this behavior is **not exclusively an unsupported newest-tail prediction**, nor can it be explained solely by jumping between different graph publications. Being before the latest endpoint does not mean every internal state is directly or strongly constrained by a measurement.

At the final snapshot the newest source endpoints are 108.476 ms behind the current support for GLIM and 28.1251 ms behind it for Kimera. Most earlier selected snapshots have approximately 208 ms and 28 ms tails respectively. That difference matters for latest-state prediction, but does not account for all the internal motion above.

## Final graph objective decomposition

| Factor family | Count | OFF converged cost | ON converged cost |
|---|---:|---:|---:|
| Startup priors | 4 | 0.000289 | 0.000063 |
| Full WNOJ | 74 | 663.827965 | 177.592167 |
| GLIM relative beliefs | 71 | 151.933420 | 20.268912 |
| Kimera relative beliefs | 22 | 9.650519 | 25.461861 |
| Total | 171 | 825.412193 | 223.323003 |

WNOJ retains substantial residual cost in the converged compromise. **Cost is not an information weight or evidence that one source is more accurate.** This table does not isolate whether the remaining motion is primarily caused by the selected process covariance, source inconsistency/correlation, interpolation constraints or their combination. No covariance adjustment is justified solely by these costs.

The final original incremental pose solution already lies close to its batch reference: maximum position changes are 0.0843 mm OFF and 0.0781 mm ON. It nevertheless contains the internal motion reported above. This is particularly strong evidence against treating all of the problem as a solver stopping issue.

## Validation and preservation

- Release compilation of the isolated diagnostic succeeded. It reused the existing local runtime and copied diagnostic code. No dependency was downloaded or production binary installed.
- All 18 batch references pass the specified convergence and restart checks.
- A complete second replay produces byte-identical incremental, batch, restarted and dense-interpolation outputs. Per-snapshot summaries match excluding wall time. Existing overlapping first-spike snapshots remain byte-identical as well.
- Twelve numerical/accounting products per mode match the original frozen receiver exactly, including online/final states and trajectories, covariance records, clock/dispatch records and accepted/unsupported accounting. Revision events match excluding execution time. This verifies that the copied-graph diagnostics do not alter the receiver.
- Factor keys, slots, dense covariance matrices and fixed-value residual probes are checked before and after batch optimization; graph copies match the original probe. These probes and ledgers are retained in each snapshot directory.
- All 410 protected files pass SHA-256 checks, including production/model code, source profiles, native outputs, previous inputs/results and the installed receiver executable.
- `git diff --check` passes. No new model or production unit-test code was introduced. This task did not rerun the whole ROS test suite or claim new sanitizer coverage; the prior model/refinement tests remain unchanged.

There is no new ATE/RPE result here. Ground truth was not used to choose parameters or to rank OFF versus ON. The primary question was whether numerical convergence removes the observed motion defects; the answer is no.

## Reproduction and runtime

The local image is `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`. The primary execution was:

```bash
docker run --rm --name source-wnoj-step-a-primary \
  --network none --read-only --user 1000:1000 \
  --tmpfs /tmp:rw,exec,size=2g -e LD_LIBRARY_PATH=/runtime/lib \
  -v /home/yeranis/results/gnssfgo_source_wnoj_step_a_20261006:/audit \
  -v /home/yeranis/repos/gnssFGO:/source:ro \
  -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005:/previous:ro \
  sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d \
  python3 /audit/run.py primary
```

The same mounts and image were used with `run.py build` and `run.py repeat`, with separate container names. `commands_*.json` retains the compiler and executable argument arrays. The executable command for each mode is:

```text
WNOJ_AUDIT_INDICES=10,20,30,38,48,58,68,74
/audit/build/step_a_replay --receiver-model source_wnoj
--mode sources --timeline record --solver isam2
--support-policy uniform_imu --support-period-ns 100000000
--receiver-startup-samples 59
--event-stream /previous/inputs/{off,on}_events.csv
--out /audit/{off,on}_primary
```

Use fresh output directories for a new replay; existing ones are protected against overwrite. `prepare.py` records input identities and preexisting state. `analyze.py` computes convergence/motion statistics, `geometry.py` checks within-interval motion, and `verify.py` checks preservation and determinism.

The Release build took 31.69 s. The complete primary diagnostic processes took 17.01 s OFF and 17.41 s ON, including the receiver and nine snapshot solves per mode. Recorded snapshot optimization/export/linear-health time sums were 4.85 s OFF and 4.94 s ON; those timings exclude their initial graph-copy and pre-solve health work. They are descriptive, not a live-throughput benchmark. The repeat took 16.96 s OFF and 17.26 s ON.

Executable SHA-256:

```text
38c6f17fa63f2b807fc95915ff60a312fae4d1fbb9ac8976398edc0c4c121c7d
```

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. The worktree was already dirty and remains so; this task adds only this report to the repository, with diagnostic code/results isolated under the new results directory.

## Conclusion and boundary

We now have a valid local optimization reference for the short frozen graphs. Two issues are distinct:

1. The incremental solver can emit poorly optimized derivative estimates at some arrivals.
2. Even a converged solution of the unchanged objective exhibits strong internal bending and large derivatives. Better solver convergence alone cannot resolve that second issue.

This does not prove that central IMU is indispensable, that all no-central-IMU models fail, or that CBS-ON is more or less accurate. It also does not identify a correct new Qc. It establishes the reference needed to evaluate a separately authorized, physically motivated motion-model uncertainty study without confusing solver error with model behavior. No such study was started here.

Step A complete. Production configuration unchanged.
