# E1b.3c — ETH1 first-40-second GLIM LOCAL profile

**Result: the false admission check is fixed, and one complete frozen CBS-ON capture passed validation.** This is a profiling result, not a new accuracy experiment or a speedup claim.

## Repair and provenance

The E1b.3b gate assumed that all GLIM DSOs would appear in `/proc/<pid>/maps` under `fixed_release`. Five of the six files there are symlinks to the unchanged `e1b_local_profile_20261012/release_2` DSOs, so Linux reported their resolved paths. E1b.3c compares each mapped basename against its unique resolved real path. The gate accepted the retained E1b.3a actual startup map and rejected six altered-path controls before playback. The new capture's own pre-playback `effective_mode_check.json` then passed, showing the repaired `fixed_release/libglim_ros.so` and five pinned `release_2` dependencies at their expected actual paths. No estimator or factor math changed.

- Task branch: `engineering/e1b-profile40-mapguard-20261012`, parent `c66ef94`.
- New run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1b_profile40_mapguard_20261012`; earlier E1b.3/3b failed roots were not overwritten.
- Preflight: 670 protected hashes, 18 focused Release tests, resolved source parameters and Kimera flags, pinned bag identities, library linkage and six positive/six negative map-path checks passed.
- Profiled GLIM wrapper SHA-256: `567bb4bc3f0e1dacf8994ecf7ca4aa12ff128e0f91640dda3adf760cd7c85df8`; verified 3344-byte bridge allocation. Receiver executable SHA-256: `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.
- ETH1 mission `2024-10-01-11-29-55`, bag offset 0, duration 40 s, playback 0.10×, wall duration 445.34 s. Native GLIM used Hesai/CPT7, native Kimera front stereo/CPT7, classic bidirectional CBS with `active_window_temporary`, 0.20 s belief horizon, one pair/update, complete source covariance scale 1. The concurrent receiver was the frozen source-driven WNOA/E2 graph, independent 100 ms clock, no central IMU factors, Qc `[.00375,.00375,.00375,.015,.015,.015]`, Gaussian source factors and downstream stable-ID revision replacement. Both factor-graph inspectors stayed off.

## Capture and correctness gates

The source, receiver and controller have complete markers and no failure marker. Native GLIM has 391 poses over 39.000 s; native Kimera ROS odometry has 184 poses over 39.600 s. The receiver has 393 online/as-played and 393 retrospectively revised final poses, each over 39.200 s. Their path lengths are 14.048, 13.746, 15.285 and 13.997 m, respectively. The Kimera startup gap is retained in the coverage record; this short profile does **not** establish full-sequence accuracy. No ATE/RPE was computed or used to choose a setting.

The native admission check found 181 K→GLIM and 180 GLIM→Kimera aligned factors constructed, with exact wire/full-covariance equality and valid causal endpoint cutoffs. Temporary-factor lifecycle checking observed GLIM 181 queued / 156 removed / 25 remaining and Kimera 180 tracked / 156 removed / 24 remaining. Sampled dense-factor whitening/Hessian/gradient checks passed. This validates logged factor handling, not independence from all older marginal-prior information.

The receiver recorded 3,986 CPT7 timestamp samples, 21,970 source-belief versions (17,720 GLIM, 4,250 Kimera), 570 accepted stable IDs, 21,375 replacements and 25 explicitly unsupported versions; no input revisions were lost, and pending-at-end was zero. Its graph has zero central IMU, bias or acceleration factors. Every publication passed the frozen `1e-5` normalized-gradient gate (maximum `9.90334e-6`). Maximum recorded transport queue was 48; the sampled live-monitor queue maximum was 2, and recorded output-age p95 was 0.090005 s. All nine checked receiver trajectory/state/clock/event files were byte-identical in one replay of the saved journal. The live/as-played trajectory is distinct from the final retrospective trajectory.

The authoritative validation files are `prepared/profile40_on_validation.json`, `profile40_on_native_validation.json`, `profile40_on_lifecycle_validation.json`, `profile40_on_diagnostics_validation.json`, `profile40_on_control/complete.json` and the captured source/receiver logs under the run root. The complete replay validator returned `passed=true`.

## Measured timing attribution

The opt-in profile SHA-256 is `2ad662fb5299da1dc5a57157e61459640d68a9ec5deec6758de2fd4f80b4865c`. Its 1,565 complete JSONL rows comprise 392 MAIN, 392 LOCAL, 389 COVARIANCE and 392 REVISION contexts. There are 391 scan-matched LOCAL rows plus one earlier startup LOCAL row. Every row has finite nonnegative cells, zero aborts/exceptions, no worker entries in this pinned non-TBB build, and an exact owner-exclusive partition of its root duration. These checks are reproduced by `analyze_profile.py`; results are in `profile_analysis.json`.

For the 392 LOCAL rows, measured root wall time totals 39.131 s. **37.724 s (96.405%) of its disjoint owner-exclusive time is `gpu_sync_linearize`**. The next portions are `isam_batch` 0.291 s (0.744%), warning output 0.239 s (0.612%), unassigned BPSAM body 0.229 s (0.586%), and GPU-linearize wrapper 0.124 s (0.317%). Backup is 0.088 s (0.225%) and measured marginalization 0.028 s (0.070%). The nested inclusive stage totals must not be added to these shares.

LOCAL made **143,950 GPU-factor linearization calls; all 143,950 took the synchronous fallback; zero used a cached linearization**. MAIN separately made 168,874 such calls, again all fallback and zero cache hits. By contrast, COVARIANCE had 195,592 GPU linearization calls with 195,592 cache hits and zero synchronous fallbacks. That contrast is direct evidence that the existing covariance batching path is active, whereas the ordinary smoother-update paths still enter the synchronous path. In the pinned factor code, that path emits a warning and calls `derivatives->linearize(linearization_point)` when `linearization_result` is absent. It is **host elapsed fallback time**, not GPU kernel execution time; warning output itself accounted for only 0.239 s of LOCAL's exclusive time.

Across the first 0–10 s of scan-matched profile, LOCAL root median was 4.195 ms and synchronous-fallback calls median was 8/update. In the last 30–40 s, those were 188.566 ms and 722/update. Weighted fallback time per call stayed similar (0.2725 vs 0.2601 ms). Median active keys were 150 in both windows; median factor slots rose from 421 to 825 and non-null factors from 351 to 793. Thus this measured early-window growth is driven primarily by **more synchronous factor linearizations**, not a large change in time per call or an observed explosion of active state keys. It does not by itself prove which individual evaluations are mathematically redundant.

The legacy GLIM callback timer independently recorded 391 LOCAL updates totaling 38.942 s, close to the 39.130 s of matched profile roots; definitions/instrumentation differ, so this is a consistency check, not an isolated profiler-overhead measurement. Legacy scan-total median/p95 were 307.376/535.236 ms. The capture passed at 0.10×; it does not certify 1× throughput or the unmeasured 120–160 s tail. Profile JSON serialization/flush occurs outside the measured root; context root times must not be summed as if they were disjoint whole-scan phases.

## Decision and boundary

**E1b.3c is valid.** The next *proposed* isolated engineering task is to test whether the existing gtsam_points batch-linearization facility can be applied to the LOCAL smoother update at the exact Values/factor set required by iSAM2, with strict cache invalidation and dense-factor/state/covariance equivalence. Do not blindly reuse a linearization across changed Values or remove any factor. The MAIN update has a similar synchronous-fallback signature, so a LOCAL-only gain would not solve all GLIM cost. No such patch, model change, covariance tuning or additional capture was made here.

Post-capture: all 670 protected hashes still match; all six pinned source repositories remain clean; owned ROS and receiver processes are stopped. No push.
