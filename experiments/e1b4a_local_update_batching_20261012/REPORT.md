# E1b.4a — implementation and correctness report, before timing

Historical pre-timing report. The subsequently authorized benchmark is recorded
separately in [TIMING_REPORT.md](TIMING_REPORT.md); statements below describe the
earlier stopping point and are not the current timing status.

**Status: synthetic correctness gate passed; timing NOT RUN.** The user requested
a report before the timing test. Work stops at this boundary. No speedup,
real-time capacity, or live-sequence equivalence is claimed.

## What was implemented

The existing LOCAL optimizer is unchanged. A default-inactive, thread-local
callback around `GlimCbsBridge::updateSidecar()` lets an already-requested
`NonlinearFactorGraph::linearize(values)` prepare its compatible GPU factors
together, using the exact Values supplied by iSAM2. The existing E1
synchronous-compatible batching helper returns the ordered Gaussian factors.

This covers new-factor graph linearization and the existing full-window iSAM2
recalculation branch. Direct individual-factor calls in incremental affected
factor relinearization and marginalization remain unchanged. No factor is
dropped, no relinearization decision is altered, and no result is retained for
reuse at a different state. MAIN, outgoing covariance preparation, Kimera and
gnssFGO run outside this scope.

- Selector: `CBS_E1B_LOCAL_UPDATE_GPU_BATCHING=1`; absent/default is ordinary.
- One TLS dispatch owner in the private `libgtsam`; no existing class layout or
  vtable change and no CUDA dependency added to GTSAM.
- Reentrant calls use ordinary execution; nested non-null scopes are rejected.
  Scope state is restored on exceptions and is independent across threads.
- Unknown/custom CPU factors, unsupported GPU types, duplicate factor pointers,
  and pending native GPU results take whole-graph ordinary fallback. Exact
  native IMU, bias, pose/velocity prior/between and marginal-container types
  are allowed. This is deliberately conservative, not a universal GPU hook.
- Reuses existing cache draining and synchronous-compatible inlier/error-cache
  behavior. No GPU kernel, covariance formula, source policy, noise, lag,
  marginalization rule, source mean, or receiver model was changed.

## Isolated build and preservation

Implementation/test commit: `440fba99e9ce4d59dec03758b9d06f55927b2470`, on branch
`engineering/e1b4a-local-update-batching-20261012`, parent `db40f27`. This report
and plan update are the subsequent documentation commit; no push.

Private root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012`.
Container prefix is `/workspace/cbs_gtsam4.3/runs/` instead. `prepare.py`
constructs narrowly patched copies of the pinned profiling sources; production
source checkouts and installed/default libraries are untouched. The new GTSAM
graph/dispatch units and all three GLIM bridge-header users were rebuilt. The
other corrected, pinned objects/dependencies were reused; this is a focused
private build, not a clean build of the entire ROS workspace.

Candidate Release library hashes:

| Artifact in `release_2/` | SHA-256 |
| --- | --- |
| `libgtsam.so.4.3a1` | `369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37` |
| `libglim_ros.so` | `83592aba553adbc2879f27b5f2cf3551826ab497a58201ba0101543934f49dec` |

The exact compile/link commands and per-attempt source snapshots are retained
under each private build directory. Six pinned source repositories remain clean
at their recorded HEADs. All 670 protected baseline hashes, 672 host-protected
paths including preparation inputs, and 429 container build-input paths match
their originals (these sets overlap; do not add them as unique files).

## Correctness results

| Check | Result |
| --- | --- |
| Final focused Release suite | 32/32 passed: 27 graph/cache/scope + 5 solver tests |
| Focused host UBSan suite | 30/30 passed; preceding test version, before the two additional Release controls |
| Supported CPU ASan+UBSan | 10/10 passed: six existing CPU regression + four dispatch lifecycle tests |
| Historical CPU oracle fingerprint | Exact, 4,716 numeric values; SHA-256 `f156d2b49e28f9d56f1ac6305c0a1ee1c7a0ef64d658c36ff6107d023b4e3d12` |
| All printed Gaussian/joint/relative-covariance comparisons | Maximum absolute difference 0 |
| Repeated-solver position / velocity / bias / covariance | Maximum differences 0 / 0 / 0 / 0 |
| Repeated-solver orientation | Maximum `1.3877787807814457e-17 rad`, below the frozen `1e-10 rad` gate |
| Factor slots/types/keys, active keys, relinearization/re-elimination and tested branch decisions | Exact between ordinary and batched instances |

The fixed-lag fixtures run 40 updates, including native-style X/V/B states,
preintegrated IMU, bias factors, CPU and GPU factors, factor removal/replacement
and retirement. Ordinary/candidate histories own independent mutable GPU factor
instances. Tests compare linearization points as well as estimates, complete
augmented covariance and dense endpoint-relative covariance. Both forced
full-window and incremental paths are exercised. A repeated candidate run also
passes the same comparisons.

The larger fixture uses 64 new GPU factors per step. Its equivalence control
does not add covariance-linearization calls between updates: it compares states
throughout and complete covariance at the end. Its benchmark remains disabled
and was not executed. Test harness durations are not performance evidence.

Additional checks cover null slots, unary factors, fresh Values, native-cache
interleaving, MAIN/LOCAL sharing within each independent history, duplicate and
unknown-type fallbacks, wrong/missing Values, a supported CPU factor exception
after GPU work, cache draining, thread isolation, and recursion. The failure
fixture explicitly restores the caller's existing smoother checkpoint (including
timestamps) and checks the next successful update against the ordinary oracle;
this patch does not add or claim a stronger general solver rollback API.

Sanitizer limits: new host graph/dispatch units and relevant header/test code
are instrumented; dependency libraries and CUDA kernels are not newly instrumented.
The ASan build uses the retained E1 `EIGEN_MALLOC_ALREADY_ALIGNED=1` ABI setting.
No GPU ASan/device-memory sanitizer, leak-check, or complete repository-suite
claim is made.

## No-data integration check

With the new selector explicitly enabled, the GLIM wrapper started on a separate
ROS master, registered the expected Hesai/CPT7 subscriptions, remained alive,
and shut down with exit code 0. Six mapped libraries match their resolved pinned
paths; only the new GTSAM and wrapper DSOs replace their prior counterparts.
There were **zero sensor measurements**, no bag playback, no Kimera and no
receiver. Thus this checks startup/ABI/library admission, not live processing.
The owned node and master both exited normally.

## Retained preparation corrections

No failed attempt was overwritten. The first compile needed an explicit
GaussianFactorGraph include; the first test compile needed a fully qualified
symbol-key helper to avoid an inherited type alias. A first audit script
included the newline when hashing the old fingerprint; after matching the
historical splitlines convention, the values and fingerprint match exactly.
The preliminary ASan build omitted the established Eigen allocation-ABI flag;
it was not used for acceptance. `asan_2` is the compatible validated build.
None of these involved changing a numerical tolerance or the estimator model.

## Evidence and next boundary

Tracked summaries: `correctness_summary.json`, `integrity.json`, tests and build
scripts beside this report. Detailed local evidence:

- `release_2/checks3/`: final 32-test XML/logs and historical fingerprint.
- `ubsan_1/checks1/`: 30-test UBSan XML/logs.
- `asan_2/checks1/`: ten supported CPU sanitizer tests.
- `no_data_startup/`: resolved parameters, process maps, zero-measurement
  registration state and clean shutdown evidence.
- `source_git.json`, `*_source_snapshot/`, `*_commands.json` and integrity files:
  exact build/source provenance and unchanged-input checks.

**Do not promote this candidate or start playback based on these checks alone.**
The next pending gate is the already-declared alternating synthetic update
benchmark, comparing both median and p95 to the ordinary path (target <=80%),
with matching mathematical outputs and CPU-fallback overhead reported. The user
requested this report before that test; it has not started. Real-source
equivalence and a separately scoped bounded throughput capture remain later
steps. No MAIN optimization, direct-loop extension, model change, or live run
was bundled into this task.
