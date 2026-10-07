# Source-independent asynchronous receiver — bounded engineering validation

Date: 2026-09-28. This is an opt-in extension of the existing GrandTour gnssFGO receiver, not a new estimator, CBS protocol, or live ROS launch. No GLIM/Kimera source rerun, covariance tuning, or EKF work is part of this task.

## Reuse audit and implementation boundary

The upstream `online_fgo/src/graph/GraphTimeCentric.cpp::constructFactorGraphOnIMU()` already builds supports from IMU frequency / optimization frequency. `IntegratorBase.h::findStateForMeasurement()` already distinguishes synchronized, interpolated and cached measurements. These establish the appropriate time-centric architecture. However, those ROS classes also carry application initialization, optional no-measurement/zero-velocity suspension, gravity/calibration choices and fixed-lag lifecycle not used by the validated GrandTour revision receiver. Switching wholesale to that application would change several experimental assumptions at once.

Accordingly this patch extends `online_fgo/src/offline_process/GrandTourCsvReplay.cpp`, reusing its existing `IncrementalBackend`, `AddRelativeFactor`, `PoseAtEndpoint`, `MakeWnojInterpolator`, `IntegrateAllRight`, priors, startup averaging, Combined IMU and WNOJ constructors. All factor/model headers and the ALL_RIGHT integration helper remain unchanged. There is no second solver, residual, interpolation model or preintegrator.

The only new reusable component is `CausalSupportClock.h`: a small transport-independent support clock fed by IMU physical and availability timestamps. A future ROS adapter can feed this clock incrementally. **The complete graph/event receiver has not yet been extracted into a shared live/offline class, and no live ROS adapter is implemented or certified here.** This is the first bounded source-independent receiver milestone, not completion of live multi-sensor integration.

## Opt-in semantics

- Existing default `--support-policy glim_endpoints` remains available and retains the frozen source-dependent grid.
- New `--support-policy uniform_imu --mode sources` requires no GLIM or Kimera first-belief file. One revision stream may contain arbitrary source IDs. The frozen two-source recording is accepted directly; `g/k` are adapter aliases for `glim/kimera`.
- `--receiver-startup-samples 59` fixes startup without reading any belief endpoint. X0 is anchored at the 59th IMU sample, using all first 59 samples for the unchanged gravity/bias averaging. In the retained SPX-3 capture these are the same 59 initialization samples as the legacy run, but X0 moves slightly earlier to that IMU sample's timestamp.
- `--support-period-ns 100000000` creates a uniform 100 ms clock. `--receiver-duration-ns 60000000000` caps physical support creation at 60 s after the first input IMU sample. These are predetermined engineering settings, not selected by ATE.
- An ALL_RIGHT boundary can lie inside an IMU interval. Its support is unavailable until the sample closing that interval is received. Prefix-maximum receipt time additionally waits for earlier delayed IMU samples. Clock unit tests include this case.
- IMU availability is read from `record_timestamp_ns` when present, otherwise explicitly falls back to the physical timestamp. This dataset CSV contains that column. Dataset record time is not a measurement of future live callback, compute, or network latency.
- The CSV adapter materializes the clock outputs for lookup, but optimizer keys are created only at emitted availability times. Source timestamps never enter clock generation or startup. Relative factors cannot use keys that have not yet been created; an assertion enforces this and measurement availability on every insertion.
- Beliefs before initialization or beyond the configured support horizon are reported individually in `unsupported_beliefs.csv`, not extrapolated, re-timed or silently clamped. Initialization coverage remains a real boundary; this change does not magically provide poses before startup.
- Factors arriving before their right support enter the existing pending queue. Accepted factors keep exact physical endpoints and native full covariance. Any source can use the exact, single-interpolated, same-support or double-interpolated constructor, without source-specific logic.
- A relative source event is identified by `(source, session_id, from_index, to_index)`; `session_id` defaults to `default` for historical files. A restart must provide a new session. Each newer complete revision removes the old iSAM2 factor and inserts one replacement. A first-seen nonzero revision is accepted in independent mode; older arrivals are stale, identical duplicates are ignored, conflicting same-revision payloads and changed physical endpoints are rejected.
- Existing deterministic arrival ordering is `(record time, source, ordinal, stable input order)`. At a support availability tie, create the support, release pending factors that now have all keys, then process newly tied arrivals. Multiple supports released by one delayed IMU are handled in increasing physical order. Recorded input order is part of the tie contract; this is not arbitrary-thread-arrival reproducibility.
- Combined IMU/WNOJ are constructed once per support interval, regardless of source count. Full-history iSAM2 retains historical keys for delayed replacement. No fixed-lag substitution is made.

The generic input contract is already-calibrated timestamps and relative means/full covariances in the common CPT7 body convention and existing `[phi,rho]` tangent. This patch does not implement new sensor extrinsic/time-offset calibration or validate an arbitrary uncalibrated sensor feed. Synthetic seven-source acceptance is not certification of seven physical sensors.

## Frozen settings and important caveat

Combined IMU, WNOJ Qc diagonal variances 10000, ALL_RIGHT exact interval integration, priors, initialization averaging formula, full-history iSAM2, and source covariance scale 1 remain unchanged. No GNSS, gating, robust loss, health weighting, or source-to-source protocol change. Classic CBS source recordings are reused unchanged.

WNOJ support accelerations are computed using the pose/bias estimate at support creation and cached; the priors/interpolators use these constants. Therefore a physical-time offline insertion order and an arrival-time insertion order need not even construct identical auxiliary factor payloads. This pre-existing behavior was not changed. We test repeated identical event streams and unseen-future-prefix invariance; we do **not** claim batch/order-independent equivalence or live shared-core equivalence.

## Validation and results

Release build completed in the existing Humble container (`ubuntu-jammy-humble-nvidia`). Three CTest targets pass: six relative-factor tests, four ALL_RIGHT integration tests and four new clock tests (14 tests total). The new clock helper also passes focused ASan/UBSan; this does not mean the full externally linked GTSAM/ROS stack was sanitizer-instrumented. The existing `eig2_revision_receiver_smoke.py` passes (four IDs, duplicate/stale handling, unchanged duplicate/stale trajectory, invalid covariance rejection). `git diff --check` and Python syntax checks pass. Repository-wide style/lint checks and a clean rebuild of the entire ROS workspace were not performed.

`synthetic_v3/validation.json` records successful seven-source checks: all five endpoint-constructor patterns, fractional ALL_RIGHT integration, delayed/out-of-order IMU receipt, revision replacement, stale/duplicate arrivals, first-seen nonzero revision, restarted source session, source dropout, deterministic repeat, reordered input rows with distinct event-order keys, and rejection of conflicting payloads/changed stable endpoints/pre-endpoint availability. Changing future **beliefs and IMU values/availability** leaves earlier online state, cached acceleration and clock prefixes byte-identical. This tests the actual receiver executable, not just a mocked scheduler. An initial test-fixture attempt accidentally put a changed endpoint after its availability time and was correctly rejected; the corrected fixture has a legal timestamp and passes. That was a fixture correction, not a relaxation of the receiver check.

The retained classic CBS-ON recording is `runs/spx3_graph_inspection_20260928/spx3_classic_on_graphs_first60_start32`, bag offset 32 s, duration 60 s. No source estimator was launched. The existing CPT7 CSV is reused; a read-only check of the raw SPX-3 IMU bag confirms its 6,000 samples over the 60 s interval have bag record time equal to header time (minimum/maximum difference zero). This confirms the dataset chronology, not host scheduling latency.

| Receiver accounting | Frozen GLIM-grid replay | Independent IMU-clock replay |
|---|---:|---:|
| Support poses | 592 | 595 |
| Physical graph duration after initialization | 59.099751 s | 59.4 s |
| Initial physical timestamp ns | 1730565903588830000 | 1730565903584194000 |
| Startup samples | 59 | 59 |
| GLIM IDs accepted | 590 | 590 |
| Kimera IDs accepted | 289 | 290 |
| Replacements | 33,530 | 33,530 |
| Skipped version messages | 76 | 75 |
| Exact / double-interpolated source factors | 590 / 289 | 0 / 880 |
| Queued arrival messages | 287 | 153 |
| Maximum support wait | 10.878262 ms | 5.734262 ms |
| Maximum covariance insertion discrepancy | 6.07e-18 | 6.07e-18 |
| Full final body path | 33.698435 m | 33.971503 m |
| Wall runtime (concurrent validation pass) | 250.36 s | 339.74 s |

The initialization gravity, gyro bias and accelerometer bias are exactly equal numerically. X0 is 4.636 ms earlier, as a direct consequence of anchoring to the 59th IMU sample instead of the first GLIM endpoint. The independent horizon also extends farther: it admits the last Kimera edge previously outside the GLIM grid. The remaining 75 unsupported versions belong to the first three Kimera IDs, whose starts precede initialization. Consequently this is **not a pure same-factor-set accuracy ablation**: startup time, grid geometry, end coverage and one final Kimera ID differ. No tuning follows from these metrics.

### Common evaluation

The scoring wrapper uses the existing native GLIM `odom_imu.txt`, native Kimera `kimera_odometry.csv`, and the two receiver products. It resamples them only for scoring to 590 common 100 ms samples from `1730565903.68883` through `1730565962.58883`, using the existing scorer's quaternion interpolation. Every product has the same 372 prism-matched samples, covering 36.9 s in three segments; AP20 prism gaps are not filled. ATE uses the existing lever arm and no-scale SE(3) Kabsch alignment. All six products have 580 valid 1 s IE-TC RPE pairs. Source files and receiver outputs are not overwritten.

| Trajectory | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Evaluation prism path m | GT-overlap prism path m |
|---|---:|---:|---:|---:|---:|
| Original/native GLIM | 0.016058 | 0.021418 | 0.485143 | 34.316899 | 29.033913 |
| Original/native Kimera ROS odometry | 0.090182 | 0.030429 | 0.432440 | 33.984063 | 28.847613 |
| Fused, legacy grid, final revised | 0.038440 | 0.011047 | 0.457480 | 33.938004 | 28.735637 |
| Fused, independent grid, final revised | 0.038286 | 0.010539 | 0.310345 | 33.932439 | 28.731792 |
| Fused, legacy grid, causal as-played | 0.072883 | 0.065877 | 2.795639 | 36.204136 | 30.715527 |
| Fused, independent grid, causal as-played | 0.073423 | 0.066530 | 2.833252 | 36.106395 | 30.628507 |

Authoritative matched evaluation: `spx3_60s/evaluation_native/comparison.json`. An earlier `evaluation/` scratch comparison used GLIM ROS odometry instead of the established native saved trajectory; it is retained for transparency and **is not the primary native comparison**. The native saved trajectory convention is corrected in the repository scoring wrapper. The small differences from historical short-window numbers also reflect explicit common-time resampling/masking; those historical artifacts remain untouched.

The independent final trajectory is bounded and has essentially unchanged ATE, with different rotational RPE. Causal as-played errors remain materially larger than the final retrospectively revised errors. This task does not diagnose or fix that difference, infer source independence, or claim the fusion is better than native GLIM.

The new receiver replay requires roughly 5.7 wall seconds per simulated second in the concurrent run; its isolated repeat without graph snapshots takes 273.55 s (about 4.6× the recording duration). Both sources now exercise GP interpolation and 34,410 factor-version insertions are processed without coalescing. This is **not 1× real-time validation**. A 0.1× playback budget is more generous, but live co-running estimator contention, backlog, and latency must be measured before claiming feasibility.

### Completed identity and timing checks

`spx3_60s/validation.json` verifies:

- Legacy `trajectory.tum`, `states.csv`, `online_trajectory.tum`, and `online_states.csv` are byte-identical to the frozen capture's original receiver products.
- Independent repeated outputs are byte-identical for all four products, `support_clock.csv`, `support_accelerations.csv`, and `causal_dispatch.csv`, despite graph snapshots being enabled only on the first pass.
- All 34,410 accepted version events satisfy `insertion >= arrival`, `insertion >= required support availability`, and `required support <= created support`. Accepted plus unsupported version events account for all 34,485 input rows.
- Input file and executable SHA-256 checks pass after replay. Native source artifacts were not modified. Factor/model headers and `GrandTourImuSupport.h` have no git diff.

Independent final trajectory SHA-256: `b6e32f5e96d5ca717e0055fc196530f499d4a31edc6f1475faabf74fe9aa40ff`. Online trajectory SHA-256: `4b392d7047465c1d1326944f4f7910439e40824112bc16b9c8e94a4207f1798c`.

Extra support waiting has median/p95 zero, mean 6.21 microseconds across all version messages, maximum 5.734 ms. Support creation's physical-to-availability delay is median 2 microseconds, maximum 4 microseconds on this dataset; synthetic delayed receipt tests exercise much larger waits. iSAM2 per-version update time on the concurrent pass is mean 9.08 ms, median 3.56 ms, p95 36.16 ms, maximum 157.31 ms. These are wall timings, not deadlines guaranteed by the implementation.

Final source-wise squared whitened relative residuals are retained without gating: GLIM median/mean/p95/max = 105.18 / 311.17 / 1373.70 / 7117.06; Kimera = 19.14 / 37.13 / 127.39 / 854.97. These are **post-optimization residuals**, not independent-filter innovation NIS, and no chi-square consistency conclusion or weighting change is made here.

Artifacts: `/home/yeranis/results/gnssfgo_async_receiver_20260928/`.

Reproduction scripts:

- `tools/test_async_receiver.py`: synthetic seven-source integration and failure-path checks.
- `tools/run_spx3_async_receiver_validation.sh NEW_OUTPUT_ROOT`: unchanged legacy replay, independent 60 s replay, deterministic repeat, hashes and byte comparisons; run inside the existing Humble container after sourcing `/opt/ros/humble/setup.bash`.
- `tools/score_async_receiver_validation.py`: common-time scoring-only resampling then unchanged GrandTour prism/1 s RPE evaluators. Native source outputs remain native, never substituted by source-only receiver controls.

Repository HEAD at start: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. The worktree already contained unrelated modifications and research artifacts, including edits to the replay file. They were preserved. No commit or clean-worktree claim is made.

Final HEAD remains the same. Receiver source SHA-256 is `cdbb920a1883ca48c472b343a1470bdc0df324f6107388f055d36785e7fe1375`; clock header SHA-256 is `398dac7f81e0efc1815e144a9586f7f0c8343ff9740cce166e8bb63df4657e89`.

## Exact reproduction and remaining scope

Build:

```bash
docker exec -u root ubuntu-jammy-humble-nvidia bash -lc 'source /opt/ros/humble/setup.bash && cmake --build /home/yeranis/workspaces/gnssfgo_ct_ws/build/online_fgo --target grandtour_ct_replay test_causal_support_clock test_grandtour_imu_support test_ct_relative_factor -j2'
```

Bounded validation (choose a new output directory; the script refuses an existing one):

```bash
docker exec ubuntu-jammy-humble-nvidia bash -lc 'source /opt/ros/humble/setup.bash && bash /home/yeranis/repos/gnssFGO/tools/run_spx3_async_receiver_validation.sh /home/yeranis/results/gnssfgo_async_receiver_20260928/spx3_60s'
```

The actual new mode uses only `--imu`, `--revision-beliefs`, `--mode sources`, `--timeline record`, `--solver isam2`, `--support-policy uniform_imu`, `--receiver-duration-ns 60000000000`, and the output path (plus optional graph snapshots). There is no GLIM support input. Model settings stay frozen.

No new default production launch or CBS profile is installed. Remaining work before a live receiver: extract the existing graph/event handling into a shared transport adapter/core, implement and test live ROS subscriptions with calibrated source/session metadata and explicit tie order, measure load/backlog, and define transactional recovery for solver failures. CSV decoding currently eagerly validates the file; valid-future suffix invariance is tested, but streaming malformed-message handling is not implemented. Future fixed-lag operation requires a policy for revisions whose old information is already absorbed into marginal priors. Merely switching solver type would not preserve replacement semantics.
