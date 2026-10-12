# E1b.4 — proposed GPU batching inside the GLIM LOCAL update

Status: design prepared on 2026-10-12. Implementation, builds and experiments
have not started. This proposal follows the validated E1b.3c profile at commit
`11a99e8ac6529bef907bf6265ef4b737dc835958` and preserves the current E1/E2 runtime.

## Decision

Keep the existing LOCAL fixed-lag optimizer and its mathematical decisions.
Change the execution of an already-requested set of GPU factor linearizations:
prepare compatible factors together at the exact supplied Values, then return
the same ordered Gaussian factors to the existing solver.

The first patch, **E1b.4a**, should cover the existing
`NonlinearFactorGraph::linearize(Values)` boundary while the GLIM LOCAL update
is running. This covers new-factor linearization and iSAM2's full-window
recalculation branch. It deliberately leaves direct individual calls in affected
factor relinearization and marginalization alone. Measure their remaining cost
after this patch before deciding whether to extend the same mechanism there.

This is a batching-only hypothesis. There is no new persistent result cache,
factor deduplication, modified relinearization threshold, parallel MAIN/LOCAL
execution, or reduction in the requested mathematical work.

## Evidence and scope

The first40s profile reports 392 LOCAL updates, 143,950 synchronous GPU-factor
linearizations and zero GPU preparation-cache hits. LOCAL exclusive synchronous
linearization took 37.724 of 39.131 seconds (96.405%). These are host elapsed
durations including device work and waiting, not CUDA kernel-only times.

The same profile records 620 graph-linearize calls, 228 full-window iSAM2
recalculation calls and 164 incremental recalculation calls. Source inspection
shows graph linearization for new factors and the full-window branch. These
counts motivate the first interception point; the aggregate profile does not
provide the exact fraction of GPU time attributable to that point. Do not claim
the first patch will eliminate all 96.405%.

Early/late first40s median active keys are both 150, while nonempty factor counts
are 351/793 and synchronous calls per update 8/722. This does not establish
unbounded growth or redundant factors. Preserve factors and iSAM2's decisions;
record factor/evaluation counts while measuring the proposed scheduling change.

The E1 compatibility path already passed six real covariance-preparation
comparisons with 60–540 GPU factors, including zero observed Hessian, full joint
covariance and relative-covariance differences. That is evidence for reusing its
kernel execution, not validation of the new optimizer integration.

## Exact implementation boundary

Paths below are relative to `/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/`.

| Location | Proposed responsibility |
| --- | --- |
| `glim_ros1/src/glim_ros/cbs_bridge.cpp`, `GlimCbsBridge::updateSidecar()` | Enable an exception-safe, thread-local linearization callback only around `sidecar_->update(...)`. Experimental switch defaults off. |
| `gtsam/gtsam/nonlinear/NonlinearFactorGraph.cpp`, `NonlinearFactorGraph::linearize()` | Check for the scoped callback before the existing serial loop. When absent, execute the existing implementation. Callback receives the exact graph and Values of this call. |
| New small GTSAM internal header/implementation | Own the scoped callback and recursion suppression in one shared library. Expose only GTSAM graph/Values types; introduce no GTSAM dependency on CUDA or GLIM. |
| `glim_ros1/include/glim_ros/cbs_local_graph_linearizer.hpp` | Adapt the existing E1 helper for scoped use, including an explicitly bypassed ordinary fallback. Preserve output indices and null entries. |
| `gtsam_points::NonlinearFactorSetGPU::linearizePreservingSynchronousVGICP()` | Reuse its existing buffers, kernels, synchronization and synchronous-compatible result handling. |

The scope exists only during LOCAL's update on the backend thread. MAIN,
outgoing covariance preparation, revision export and Kimera run outside it.
Reject unsupported nested/reentrant use or execute the ordinary path; do not let
an unrelated nested calculation inherit acceleration implicitly. The pinned
GTSAM build has TBB disabled. Enabling a new threading configuration is outside
this patch and must not silently activate this path there.

Suggested experimental selector: `CBS_E1B_LOCAL_UPDATE_GPU_BATCHING`, default
false, evaluated in implementation-local storage. Avoid adding members to
`GlimCbsBridge`, BPSAM or ISAM2 just to store this switch. In particular, changing
the shared BPSAM base to `ISAM2Ext` would also affect Kimera and is not proposed.

The existing calls that would reach the callback are:

1. `ISAM2-impl.h::linearizeNewFactors`: `newFactors.linearize(theta)`.
2. `ISAM2.cpp::recalculateBatch`: `nonlinearFactors_.linearize(theta_)`.

The callback must use that exact `theta`/`theta_`. A separate
`calculateEstimate()` result or the Values from before `ISAM2::update()` can be
different. Preparing the whole graph once at the entry to `updateSidecar()` would
therefore be incorrect.

## Mathematical contract

For each requested factor f and supplied linearization point theta, the candidate
must return the same Gaussian contribution as the ordinary f.linearize(theta),
including keys, Hessian blocks, gradient vector and constant error term.
Equivalently, the local quadratic approximation in delta is unchanged:

`q_f(delta) = 0.5 * delta^T H_f delta - delta^T b_f + c_f`.

Preserve ordered factor assembly, slot/null layout, iSAM2's existing cached
Gaussian-factor decisions, variable ordering, elimination method, updateDelta,
fixed-lag retirement, marginal priors and rollback behavior. Existing IMU factors
in native GLIM are unchanged; the central WNOA receiver remains unchanged.

This retains the normal CBS mean from MAIN and uncertainty from LOCAL, current
temporary external-factor removal, and the separate current-smoother revision
export path. It does not reinterpret all downstream revisions as LOCAL
covariances or claim independence from shared sensor/nonlinear history.

## Cache, inlier and exception contract

Use the E1 **synchronous-compatible** path, not the default native GPU hook.
The default `store_linearized()` also compacts inliers and writes an error cache;
that difference caused the original E1 joint-covariance failure. The compatible
path preserves those behaviors of the synchronous baseline.

- Only unique, exact supported VGICP factor types without pending prepared
  linearizations are eligible. Preserve the existing fallback for duplicate
  pointers, unsupported GPU types and already-prepared native results.
- Prevalidate supported factor keys/types before submitting any GPU work.
- Prepared results live for this one graph-linearization call. Consume them at
  the exact supplied Values and leave no pending result on success or exception.
- Reuse the existing E1 helper's draining discipline and test it in this new
  scope. Preserve pre-existing native error caches. No silent catch-and-retry of
  a partially executed batch; catchable failure propagates after cleanup.
- GPU factors can be shared by MAIN and LOCAL. Their mutable caches and inlier
  state make concurrent evaluation unsafe. Keep the present sequential source
  execution; validate MAIN -> LOCAL -> MAIN interleavings on shared objects.
- The E1 helper evaluates GPU factors before CPU factors and restores Gaussian
  output order. This relies on supported CPU factors having no cross-factor
  evaluation side effects. Inventory types in fixtures; unknown/custom
  stateful cases must take ordinary execution. Exception tests must compare the
  next successful solve against an ordinary oracle, not merely assert that an
  exception was thrown.
- Ordinary fallback must bypass the scoped callback to prevent recursion.

Batching still performs GPU synchronization. The existing facility submits the
factor tasks before awaiting their results and aggregates transfers/buffers; it
does not promise one kernel or one CUDA synchronization for the entire graph.

## Validation sequence and decision gates

### E1b.4a: isolated implementation and deterministic fixtures

Build in a new private directory with pinned compiler, dependencies and flags.
Use dependency-aware rebuilding of every affected translation unit/header user;
do not reuse stale objects across changed headers. Keep the working libraries
and prior failed/successful run roots intact. Verify loaded resolved library
paths and node startup before any later playback.

1. With the selector off, existing deterministic solver fixtures, factor call
   counts and outputs must remain exact. MAIN/Kimera isolation is tested.
2. With it on, compare each Gaussian block and assembled graph to the ordinary
   path at identical Values. Exercise new factors, forced full-window branch,
   mixed CPU/GPU factors, unary/binary factors, null slots and changing poses.
3. Test repeated fixed-lag updates through retirement, factor removal/slot reuse,
   backup/copy/restore, pending native caches, errors before/after linearization,
   fallback and exceptions. Check selected keys, slots, branch decisions and
   covariance after each update. Include the existing E1 regression fixtures.
4. Run focused Release and supported ASan/UBSan tests, declaring CUDA/dependency
   coverage limits. No change in test tolerance in response to a failure.
5. Benchmark the same deterministic update sequences in alternating ordinary/
   candidate order after correctness passes. Record LOCAL median/p95/max,
   number of prepared/consumed/fallback factors, reason for fallback, callback
   site and count, factor/state counts and total update time. Calls that stay
   synchronous are reported explicitly.

Proposed numerical gates, frozen before implementation results:

| Comparison | Gate |
| --- | --- |
| Input identities, graph order, keys, slots, update/retirement decisions | Exact |
| Existing deterministic CPU solver fingerprint | Exact |
| CPU Gaussian matrices | maxabs difference <= `1e-12 + 1e-12 * maxabs(reference)` |
| GPU augmented Hessian | maxabs difference <= `1e-8 + 1e-6 * maxabs(reference)` (existing E1 gate) |
| Full joint and dense 6x6 relative covariance | maxabs difference <= `1e-10 + 1e-6 * maxabs(reference)` (existing E1 gate) |
| Same-input solver pose translation, velocity and each bias component | maxabs difference <= `1e-9` in its native units |
| Same-input solver orientation | relative rotation angle <= `1e-10 rad` |

Record observed differences even when they are zero. If a fixture lacks a
well-defined anchored covariance, repair its definition before benchmarking;
do not dismiss real discrepancies as gauge effects. Passing kernel tests alone
does not establish repeated-solver equivalence.

Suggested performance gate: for the representative GPU-heavy fixed-lag fixture,
both median and p95 total LOCAL update time <=80% of ordinary execution, with
the identical factor requests and numerical gates satisfied. CPU-only fallback
must not show a material regression beyond measured repeatability. This is a
component acceptance criterion, not a claim of real-time throughput.

### Subsequent bounded real-data gate

After E1b.4a passes, use a separately scoped short ETH1 same-input validation
and throughput test. Existing text logs do not serialize complete GPU factors,
point clouds/voxel maps, mutable caches and exact per-update Values. A receiver
journal replay alone cannot validate this source optimizer change.

Real-graph comparisons require independent ordinary/candidate LOCAL instances
with independent mutable GPU evaluation state receiving identical native update
inputs. Compare Gaussian contributions and LOCAL states/covariances through
marginalization. Do not claim independent oracles by copying shared factor
pointers. If the required fixture cannot be reconstructed, specify the bounded
capture needed before interpreting any live speed result.

For throughput, run the same first40s ETH1 profile at0.10x with the retained
settings and admission/lifecycle/conservation/publication/journal checks.
Distinguish instrumentation overhead and changed live source arrival order from
same-input numerical correctness. No automatic full160 or full-sequence run.

### Extension only if evidence requires it

Direct calls in `ISAM2::relinearizeAffectedFactors()` and `marginalizeLeaves()`
remain ordinary in E1b.4a. If they dominate the residual cost, E1b.4b would apply
the same callback to their exact selected ordered subsets, preserving existing
Gaussian-cache reuse and marginalization decisions. That needs its own narrow
diff and tests; it is not bundled into the first patch.

## Completion and boundaries

Accept an engineering candidate only after numerical, lifecycle and timing gates
pass. A faster but mathematically different candidate is rejected. A correct
candidate without meaningful speed benefit is reported as such and not promoted.

MAIN also showed synchronous fallback in the latest profile; receiver
full-history refinement has separate growth costs. Neither is changed by this
proposal. The initial result should answer one question: can the existing LOCAL
graph update produce the same answer faster by batching its requested GPU work?

This document and the plan ledger are the only changes made during preparation.
No implementation, estimator, receiver, experiment, tuning or push was performed.
