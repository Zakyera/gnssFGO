# GLIM shared linearization feasibility

## Conclusion

**The supervisor's method is implementable using the existing GLIM/GTSAM
infrastructure, but not by simply switching the outgoing covariance query to
MAIN.** The native Gaussian equations already exist and are publicly accessible.
The missing engineering is a pre-marginalization snapshot, native factor
provenance, a separate Gaussian prior, and reset/transaction handling.

S1 passes the read-only architectural feasibility gate. This is not a numerical
validation, runtime improvement result, or authorization to publish the new
covariance. No estimator code, executable, configuration, or experiment changed.
The independent LOCAL implementation remains the reference.

This audit follows [the saved plan](GLIM_SHARED_LINEARIZATION_LOCAL_COVARIANCE_PLAN_20261012.md)
from parent commit `a84436b6429abd68810ea5f1c86c181c0fb2b78a`.
Source and binary hashes are in
`experiments/shared_linearization_s1_20261012/INTEGRITY.json`.

## The actual MAIN implementation

MAIN uses `gtsam_points::IncrementalFixedLagSmootherExtWithFallback`, wrapping
`IncrementalFixedLagSmootherExt` and `gtsam_points::ISAM2Ext`. It is not the
ordinary `gtsam::ISAM2` path used by the independent BPSAM LOCAL sidecar.
The earlier plan's ordinary-ISAM2 references were useful background, but are
not the MAIN capture locations.

The retained capture's loaded-library map identifies
`/workspace/cbs_gtsam4.3/devel/lib/libgtsam_points.so.1.2.1` and the private
`e1b_local_profile_20261012/release_2/libgtsam_points_cuda.so.1.2.1`.
The saved build rules select the default 4.3 optimizer sources, not the
`gtsam4.2/` variants. The CPU library exports the inspected `ISAM2Ext` update,
relinearization and marginalization methods.

The fallback wrapper already forwards these queries to its actual inner smoother:

- `getFactors()` — nonlinear factor slots, for identity and provenance.
- `getLinearFactors()` — corresponding cached Gaussian factor slots.
- `getLinearizationPoint()` — the reference Values `theta`.
- `getISAM2Result()` — new factor slot assignments for the latest update.

No GPU derivative interface needs to be invented. A VGICP `linearize()` returns
a `HessianFactor` with diagonal/cross Hessian blocks, linear terms and constant
term. The cache contains these weighted equations, not necessarily raw per-point
Jacobians. Gaussian elimination needs these equations; recovering raw Jacobians
or whitening the Hessian again would be unnecessary and wrong.

**Wrapper trap:** the outer fallback class also inherits a base smoother while
owning a different inner smoother. Its inherited `getISAM2()` is not forwarded.
Do not use that inherited accessor to inspect MAIN's live cache or fixed keys.
Use the forwarded getters or collect the required metadata inside the real
inner smoother.

## Cache coverage and coordinate consistency

The inspected GLIM constructor leaves `cacheLinearizedFactors` at its GTSAM
default `true`; it explicitly enables unused-slot reuse. MAIN maintains the
cache in all normal update paths:

| Path | Existing behavior | Reuse consequence |
| --- | --- | --- |
| New factors | Linearize at `theta`; write each Gaussian into its assigned slot | Native admission IDs can be mapped through `newFactorsIndices` |
| Incremental relinearization | Keep unchanged cached factors; refresh factors incident on relinearized keys, including GPU factors | Snapshot the completed update, not intermediate mutable GPU results |
| Batch recalculation | Linearize the whole graph at `theta`; replace the Gaussian cache | Same snapshot interface works; no need to change optimizer selection |
| Removal | Clear nonlinear and cached slots | Registry must remove the same identity, not interpret a reused slot as its predecessor |
| Marginalization | Delete incident factors and old Values; insert linear marginal factors | Post-update-only observation loses the native equations needed to create a separate prior |

The code provides structural cache coverage. The retained logs do not establish
a measured 100% cache-validity rate at every historical update. Future shadow
validation must check cache enablement, nonnull slots, ordered keys/dimensions,
finite blocks and reference Values at every snapshot. Missing data invalidates
the candidate; it must not trigger an unreported fresh GPU linearization.

The target is explicitly the cached native Gaussian system at MAIN's recorded
`theta`, not all native factors freshly re-evaluated at the latest published
estimate. MAIN calculates that estimate by retracting a delta from `theta`.
Older cached native equations are acceptable only under MAIN's unchanged-cache
invariants. This policy reproduces MAIN's linear approximation, not an
independently optimized LOCAL estimate.

## Exact capture and lifecycle points

The normal scan performs the following sequence:

1. GLIM creates native factors/Values/timestamps.
2. The CBS pre-update callback schedules expired external-factor removal,
   currently updates the independent LOCAL sidecar, then appends incoming CBS
   factors. Register native identities at the existing sidecar-admission point,
   **before** those external factors are appended.
3. The real inner smoother computes expiring keys and calls `isam_.update(...)`.
4. Immediately after that call returns, and **before** `marginalizeLeaves`,
   snapshot native cached factors, their reference Values, new slot assignments,
   explicit removals, expiring keys and a solver-generation/update ID.
5. MAIN performs its existing marginalization. Collect its optional deleted-slot
   and new-marginal-slot outputs and remaining/fixed keys. These outputs are
   bookkeeping; do not change the elimination or import the marginal factors.
6. Only after the inner update and fallback wrapper succeed, commit the staged
   native Gaussian history. Failure discards the staged candidate state.
7. The scan normally performs an additional empty smoother update. Observe this
   too: it can change linearizations without adding a new LiDAR frame.
8. The CBS post-update callback publishes using the final accepted snapshot.

The missing hook belongs between lines 95 and 105 of
`gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.cpp`.
The existing GLIM `on_smoother_update_finish` callback is too late for step 4.

For the frozen `active_window_temporary` mode, `applyTemporaryFactors()` returns
without doing a separate temporary solve. Do not confuse this mode with the
different `temporary_linear` implementation and instrument the wrong path.

Observers must not call `calculateBestEstimate()`, force relinearization, or
request additional lazy solves to create the snapshot. Copy the equations and
`theta` already available. At publication, use the same endpoint estimates the
existing export path already obtains.

In every mode, observer/helper errors must be contained inside the candidate
boundary: mark its history invalid without throwing into MAIN's fallback
handler. Otherwise a diagnostic could change estimator recovery. Candidate-mode
failure stops candidate publication and is reported to the run controller;
shadow failure records the evidence while leaving MAIN and existing LOCAL alone.

## Native history without a second optimizer

The proposed candidate stores immutable native Gaussian blocks and its own
Gaussian marginal prior. Stable internal IDs are assigned at native admission;
they are distinct from downstream CBS BeliefIDs. Factor class or slot number
alone cannot distinguish native and external relative factors. Retain ownership
and generation metadata to handle pointer/slot reuse safely.

At retirement, eliminate expiring variables from the native prior and incident
native factors; retain other native factors separately. Absorb each contribution
once, preserve the linear/constant terms, and discard absorbed active blocks.
Use existing GTSAM Gaussian elimination and marginal queries. The Gaussian
`Marginals` constructor does not require nonlinear optimization, both in the
pinned local implementation and [upstream GTSAM](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/Marginals.cpp).

### A useful existing safeguard for prior coordinates

MAIN marks the keys in its marginal factors as fixed linearization variables.
Its ordinary relinearization selector excludes them, and its batch path does
not independently move `theta`. This offers a simpler normal-path strategy than
rebasing a separate prior at every update:

- Save the native prior's separator reference Values when eliminating.
- After each successful update, require those retained separator keys to be
  fixed in MAIN and their reference Values to remain unchanged.
- Refresh native active factors from the current MAIN cache; keep the native
  prior at its original, unchanged separator chart.

For the same elimination schedule, native separators are expected to be covered
by MAIN's fixed separator set. This is a structural inference, not a measured
invariant from the old logs. Make it an explicit synthetic and shadow assertion.
If it fails, stop the candidate rather than freezing additional MAIN variables
or silently shifting a Hessian to a new chart.

### Covariance at the exported endpoint poses

The exported mean remains the existing MAIN mean. The cached covariance is in
the increment coordinates at `theta`; the current helper's between-pose
Jacobians are evaluated at the exported poses. Those coordinates must agree.

For endpoint `i`, let `X_i = Retract(theta_i, delta_i)`. The required first-order
coordinate map is the derivative at zero of

`Local(X_i, Retract(theta_i, delta_i + epsilon))`.

Use its block-diagonal endpoint map `A` to form
`P_endpoints = A P_theta A^T`, retaining the full 12x12 joint covariance and
cross blocks, then pass matching information blocks to the existing full 6x6
relative-covariance algebra. Check the derivative numerically using the actual
compiled Pose3 retraction. This is coordinate transport, not covariance tuning.
Do not assume `A` is identity when MAIN's delta is nonzero.

The existing SPD solve helper already has a jitter fallback and reports its use.
Do not add or alter that mechanism. Dense correctness fixtures must succeed
without it; shadow records must expose any existing jitter use rather than
claiming an unregularized result.

## Failure handling and information exclusion

The fallback wrapper can catch an update/estimate exception, rebuild its inner
smoother, add fixation/damping factors, and adjust timestamps. A successful
return from the outer update is therefore not sufficient to prove continuity
of the previous information history. Its `fallbackHappened()` flag is reset at
the next update, so polling only after the scan's extra update can miss a reset.

The minimal candidate needs an explicit generation/reset notification inside
`fallback_smoother()`, before replacement of the inner smoother. On any reset,
invalidate the candidate and prevent candidate covariance publication. In shadow
mode, record this without throwing into MAIN or causing another fallback. Do
not bootstrap a new prior from MAIN's rebuilt anchors. Recovery semantics beyond
stopping the candidate are not part of this first implementation.

Keep MAIN's combined posterior and marginal factors out of the native registry.
This is a conservative provenance boundary—not a claim that every MAIN marginal
necessarily contains explicit CBS information. In an ideal linear system,
removing external factors incident on retiring variables before elimination can
prevent their explicit absorption. Filtering a combined posterior without that
full lifecycle proof is nevertheless not the proposed method.

Native Jacobians can still depend indirectly on CBS-corrected states and
correspondences. This method neither proves statistical independence nor
guarantees calibrated covariance or better accuracy.

## Smallest proposed implementation

Three bounded areas are sufficient for the normal-path prototype:

1. **Smoother observation:** disabled-by-default pre/post marginalization events
   in the actual inner smoother, plus commit/abort/reset generation forwarding
   through the fallback wrapper. Use existing Gaussian getters and optional
   marginalization index outputs.
2. **Native Gaussian history:** an isolated helper for factor identity, immutable
   cache snapshots, native-only Gaussian elimination, frozen-separator checks,
   endpoint covariance and transactional failure. Reuse GTSAM linear algebra.
3. **GLIM bridge selection:** native registration before CBS insertion and a
   default-off shadow/candidate covariance provider at the existing export path.
   The current independent LOCAL path remains unchanged and authoritative first.

No GPU kernel change, reimplementation of iSAM2, new process model, Kimera edit
or gnssFGO edit is required by this design. A private `libgtsam_points` build
would be needed for the lifecycle hook; protect its CPU library as well as the
already protected private CUDA library. Preserve ABI isolation and verify the
actual mapped libraries before any later run.

## Validation boundary and next step

The previous exact GPU snapshot checker explicitly supports the clean LOCAL
BPSAM sidecar. It does not provide a saved MAIN Gaussian lifecycle oracle. Its
tests and copy utilities can be reused, but their earlier pass cannot certify
this method. No numerical covariance equality or reuse timing was measured in S1.

S2 must first pass dense Gaussian, retirement, chart, external-exclusion,
slot-reuse and reset/rollback tests. A bounded shadow capture is then needed
unless suitable MAIN snapshots become available. One capture can validate early,
middle and late windows; no separate sequence campaign is necessary.

Before S2, the saved plan calls for B0: the user's requested fresh unchanged
overall-timing reference, proposed as the same ETH1 first 40 s at 0.10x. It was
not launched here. The completed E1b.4a reference remains intact.

Integrity recheck: all 677 inherited protected file hashes and six source
repository HEAD/status checks passed. The audit additionally pins the mapped
CPU `libgtsam_points` and inspected source/build records. Only this report,
integrity evidence and planning/ledger documentation are changed. No build,
new runtime tests, experiment, push, tuning or default promotion.

The CPU library was not included in the inherited 677-file manifest. Its retained
loader map proves the historical path, not historical byte identity. This audit
newly records its current hash; it does not retroactively certify those bytes
against the earlier run.

**S1 outcome: FEASIBLE_WITH_LIFECYCLE_HOOKS.**

## Source anchors

All paths below are under `/home/yeranis/repos/V4RL/cbs_gtsam4.3`.
The integrity record pins their exact bytes.

| Evidence | Location |
| --- | --- |
| Actual MAIN type | `src/glim/include/glim/odometry/odometry_estimation_imu.hpp:114` |
| Native admission, initialization, extra update | `runs/e1b_local_profile_20261012/overlay/odometry_estimation_imu.cpp:268`, `:427`, `:604` |
| Forwarded cache and reference accessors | `src/gtsam_points/include/gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp:54` |
| Inner accessors and owned solver | `src/gtsam_points/include/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp:99`, `:125` |
| New/cached/batch equations | `src/gtsam_points/src/gtsam_points/optimizers/isam2_ext.cpp:77`, `:237`, `:478`; `include/gtsam_points/optimizers/isam2_ext_impl.hpp:407` within that package |
| Pre-retirement hook | `src/gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.cpp:95` |
| Factor retirement, fixed separators | `src/gtsam_points/src/gtsam_points/optimizers/isam2_ext.cpp:699`; `include/gtsam_points/optimizers/isam2_ext_impl.hpp:353` within that package |
| Fallback reset and anchoring | `src/gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext_with_fallback.cpp:93`, `:328`, `:400` |
| Native versus external registration; normal-mode temporary-solve exclusion | `runs/e1b4a_local_update_batching_20261012/release_2/libraries_source_snapshot/overlay/cbs_bridge.cpp:1269`, `:3113`, `:3229` |
| Current export covariance selection | Same bridge `:3355`, `:3682` |
| Gaussian marginal query | `src/gtsam/gtsam/nonlinear/Marginals.cpp:56` |
| Covariance helper and existing jitter | `src/cbs/include/cbs/utils/relative_pose_covariance.h:136`, `:258` |
| GPU Hessian and consumable cache | `runs/e1b_local_profile_20261012/overlay/integrated_vgicp_factor_gpu.cpp:190` |
