# ETH1 source WNOJ bounded solver refinement

The bounded refinement reduces large incremental optimization errors, but **does not make the no-central-IMU receiver healthy**. At the previously identified first spikes, estimated speed falls from 6.121 to 4.524 m/s in CBS-OFF and from 7.785 to 4.781 m/s in CBS-ON. The saved as-played position paths barely change. Most estimates still fail the explicit first-order convergence test.

This is a small, opt-in solver change, tested on the same saved eight-second ETH1 prefixes as [the first spike diagnosis](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_FIRST_SPIKE_DIAGNOSTIC_20261005.md). It is **not a validated replacement for the production receiver**, a new CBS accuracy comparison, or evidence that central IMU is indispensable.

Artifacts: [/home/yeranis/results/gnssfgo_source_wnoj_convergence_20261006](/home/yeranis/results/gnssfgo_source_wnoj_convergence_20261006).

## Scope and frozen model

GLIM and Kimera were not rerun. The receiver replays their original causal arrivals and stable-ID revisions. The journals span `1727775008003475000` through `1727775016003475000 ns`. Initialization starts at `1727775008583475000 ns`, leaving 75 support states over 7.4 seconds inside that eight-second journal window.

Each mode contains 800 IMU-clock rows and 2572 source-belief versions. Both end with 71 active GLIM IDs and 22 active Kimera IDs. There are 2548 accepted version insertions including 2455 replacements; the same 24 unsupported messages remain excluded. No new measurement is added or gated by this change.

Input SHA-256:

```text
OFF 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed
ON  a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

The following remain unchanged: X/V/W/A state, stock full WNOJ prior and interpolation, estimated acceleration, Qc diagonal 10000, independent 100 ms support grid, initial priors, source means and complete covariances at scale one, source arrival order, revision replacement, and full-history iSAM2. There are no central IMU factors, bias variables, gravity initialization or raw-IMU-value inputs in `source_wnoj`; IMU timestamps supply only the clock. This does not eliminate correlations already present between the source beliefs.

## Implementation

The selector is `--source-convergence-check true`. It defaults to false and is rejected outside experimental `source_wnoj` with iSAM2. The installed production binary and live profiles are untouched.

Implementation:

- [BoundedISAM2Refinement.h](/home/yeranis/repos/gnssFGO/online_fgo/include/offline_process/BoundedISAM2Refinement.h) contains the policy, stationarity test and transactional empty updates.
- [GrandTourCsvReplay.cpp](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp) invokes it immediately before each post-initialization support output, and once after draining the already-available final events. It writes `convergence_checks.csv`.
- [test_bounded_isam2_refinement.cpp](/home/yeranis/repos/gnssFGO/online_fgo/test/test_bounded_isam2_refinement.cpp) tests convergence, bounds, stalls, rejection and rollback; its target is registered in CMake.

The policy was fixed before running these real-data tests:

1. Evaluate the true nonlinear objective on the current estimate, including all active priors, WNOJ and source factors.
2. Linearize that exact graph at the current estimate and compute `max_i |(Jᵀr)_i| / sqrt((JᵀJ)_ii)`. Use sparse factor-graph operations, not a dense whole-history Hessian. A value at most `1e-5` is labelled `stationary`; this does not prove observability, global optimality or physical correctness.
3. If needed, attempt at most five extra empty iSAM2 updates. Each runs on a copy of the solver. No measurement, variable, factor or covariance is added.
4. Commit only a finite candidate with non-increasing nonlinear objective and unchanged factor-slot identities. Reject an objective increase or candidate exception without committing that candidate. Earlier accepted steps remain committed.
5. Stop on stationarity, rejection, the iteration cap, or objective decrease at most `1e-8 + 1e-6 * abs(previous objective)`. That last condition is explicitly labelled `stalled_nonstationary` if the gradient remains high.

The existing QR solver, relinearization threshold `0.01`, skip `1`, and back-substitution policy are unchanged. `force_relinearize=true` requests a relinearization check; it does **not** bypass the delta threshold for every variable. Therefore several empty updates can stall before the external gradient criterion is satisfied.

Unlike the previous snapshot-only diagnostic, accepted refinements feed the continuing receiver and its subsequent initial guesses. All output rows remain saved, including flagged nonstationary estimates: this permits the same-time comparison without hiding difficult timestamps. This switch is not a certified publication gate and does not refuse every unhealthy output.

## Same physical time results

The comparison uses the same information set and physical output timestamp within each mode. OFF and ON first-spike timestamps differ; the table diagnoses each run rather than ranking their accuracy. Objective is half the summed squared whitened residual, not ATE or NIS.

| Mode | Support | Stream offset s | Original objective | Refined objective | Original speed m/s | Refined speed m/s |
|---|---:|---:|---:|---:|---:|---:|
| OFF | 36 | 4.18 | 6265.656 | 2.639 | 3.319 | 2.473 |
| OFF | 38 | 4.38 | 29200.329 | 10.682 | 6.121 | 4.524 |
| OFF | 48 | 5.38 | 625079.997 | 230.458 | 19.459 | 10.039 |
| ON | 56 | 6.18 | 375.466 | 44.015 | 4.424 | 4.137 |
| ON | 58 | 6.38 | 4201.287 | 72.414 | 7.785 | 4.781 |
| ON | 68 | 7.38 | 8220.271 | 161.257 | 6.523 | 5.672 |

The refinement removes most of the solver error at these events. It does not remove the large velocities. The prior batch diagnostic already showed similarly excessive velocity even after tightly converging the same objective; this short test is consistent with that finding.

Across all 75 saved as-played states:

| Diagnostic | OFF original | OFF refined | ON original | ON refined |
|---|---:|---:|---:|---:|
| Speed RMS m/s | 8.498 | 5.426 | 2.807 | 2.419 |
| Maximum speed m/s | 33.095 | 14.400 | 9.588 | 7.827 |
| Samples with speed above 5 m/s | 27 | 26 | 7 | 4 |
| Translation tangent acceleration RMS m/s² | 46.637 | 46.688 | 20.352 | 20.346 |
| As-played position path m | 22.169352 | 22.156399 | 9.123466 | 9.121485 |
| Final retrospectively revised position path m | 1.722610 | 1.722583 | 1.185593 | 1.185595 |

The 5 m/s marker is the previous diagnostic trigger, not a new measurement gate or calibrated velocity limit. Acceleration here is the existing WNOJ translational tangent-derivative state, not a new raw accelerometer reading. Path length is computed directly from successive saved positions; it is not a ground-truth accuracy metric. In particular, lower emitted velocity spikes must not be mistaken for a repaired position trajectory. No ATE-based parameter selection was performed.

## Convergence and cost

There are 74 online checks after the initial state, plus one final check per mode.

| Online check outcome | OFF | ON |
|---|---:|---:|
| Meets normalized gradient threshold | 5 | 5 |
| Stalls above gradient threshold | 53 | 43 |
| Rejects a worsening candidate | 16 | 26 |
| Exhausts five-update budget | 0 | 0 |
| Candidate exception | 0 | 0 |

Each rejected attempt leaves the latest accepted solver intact; rejection does not mean the source factor was discarded. Median final gradient norms are 0.0222 OFF and 0.0268 ON, above the `1e-5` criterion. Both final checks also remain nonstationary. Merely raising the iteration cap is not supported by these results: the observed stopping mechanisms were stalls and objective increases, not cap exhaustion.

| Refinement timing | OFF | ON |
|---|---:|---:|
| Median per online check ms | 2.23 | 2.19 |
| p95 ms | 40.76 | 15.63 |
| Maximum ms | 48.73 | 20.85 |
| Total including final check s | 0.650 | 0.399 |
| Maximum attempted empty updates per online check | 4 | 3 |

Whole replay process times were 9.18 s OFF and 8.68 s ON, versus 7.88 and 8.23 s for the switch-disabled runs. These are descriptive local timings, not a controlled throughput benchmark: some validation/build work overlapped. Copying and checking a full-history graph grows with graph size. A bounded iteration count does not establish bounded execution time or full-sequence live capacity.

## Validation and preservation

- Six new Release unit tests pass; the same six pass focused ASan/UBSan. Existing dependency libraries are not sanitizer-instrumented. The Eigen allocation ABI definition matches the prebuilt library; no sanitizer suppression is used.
- Twenty-one existing focused tests pass: full source WNOJ, IMU support, support clock, model-discrepancy covariance and relative-factor tests. This is not a claim to have rebuilt/tested every ROS package.
- Existing synthetic receiver integration passes, including byte-identical default `imu_wnoj` behavior against the frozen baseline executable.
- Enabled synthetic stationary, constant-velocity and accelerating cases pass. Maximum position errors are 0, `2.373e-5 m` and `4.925e-5 m`. Changing every raw IMU value leaves enabled state/trajectory outputs byte-identical. Source payloads and dispatch stay unchanged.
- OFF/ON repetitions have byte-identical state, trajectory, covariance, residual and accounting files. Convergence records are identical excluding execution time.
- With the selector disabled, 12 numerical/accounting products per mode exactly match the frozen pre-change eight-second replay.
- Enabled and original runs have identical source covariance records, dispatch/clock records, source accounting, unsupported records and stable factor-slot/revision ledgers. Final graph: 4 priors, 74 WNOJ factors, 93 active relative factors, 300 state variables, zero central IMU factors and zero bias variables.
- All 267 protected files pass SHA-256 verification. Production binary, source configurations, source estimator results, model/interpolator code and prior reports remain unchanged. `git diff --check` passes.

During synthetic test development, one initial assertion incorrectly expected the production delta threshold to achieve a tighter scalar root tolerance. The dedicated descent test now uses a tighter synthetic threshold; a separate test verifies that a loose threshold is flagged as stalled. The experimental real-data policy was not changed. Both test-attempt logs are retained.

## Reproduction and provenance

The existing local image is `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`. No network or downloads were used. The executable is built in the artifact directory with `-std=c++20 -O3 -DNDEBUG`, using the frozen local GTSAM/gnssFGO runtime. It is not installed over the live executable.

The executed primary command was:

```bash
docker run --rm --name source-wnoj-convergence-primary \
  --network none --read-only --user 1000:1000 \
  --tmpfs /tmp:rw,exec,size=2g -e LD_LIBRARY_PATH=/runtime/lib \
  -v /home/yeranis/results/gnssfgo_source_wnoj_convergence_20261006:/audit \
  -v /home/yeranis/repos/gnssFGO:/source:ro \
  -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005:/previous:ro \
  sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d \
  python3 /audit/run.py primary
```

`run.py` records the complete compiler and executable argument arrays in `commands_*.json`. The primary invocation for each mode includes:

```text
/audit/build/convergence_replay
--receiver-model source_wnoj --source-convergence-check true
--mode sources --timeline record --solver isam2
--support-policy uniform_imu --support-period-ns 100000000
--receiver-startup-samples 59
--event-stream /previous/inputs/{off,on}_events.csv
--out /audit/{off,on}_primary
```

Existing result directories are protected against replay overwrite. Use fresh output directories for a future reproduction. `manifest.json` freezes the pre-run policy and protected inputs; `analysis.json`, `verification.json`, and both integration validation files retain numerical evidence. Source snapshots and pre-edit backups are in `build/` and `baseline/`.

Executable SHA-256: `188a4b46e5d26dc11bab0e3dd4e9e16dc45c17d721a2bd38ed08354527c67ccb`.

Final HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created. The worktree was already dirty and remains so. This task adds the optional receiver hook, refinement header, focused tests/CMake registration and this report; unrelated changes are preserved.

## Engineering conclusion

Keep this feature experimental and disabled by default. It demonstrates a useful numerical improvement and makes nonconvergence visible, but it is not sufficient to replace the working central-IMU baseline.

Two distinct issues remain: the unchanged incremental relinearization policy often stops above the strict gradient criterion, and the unchanged motion/source objective admits excessive derivatives even in the earlier tightly converged batch checks. A subsequent WNOJ-uncertainty validation must use physically justified motion assumptions and check solver convergence independently, rather than selecting a covariance from CBS-ON ATE. No WNOJ strength or source covariance was changed in this task.
