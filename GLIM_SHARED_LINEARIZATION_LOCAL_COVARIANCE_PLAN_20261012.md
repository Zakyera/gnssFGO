# GLIM local covariance from shared native linearizations

## Decision and scope

Plan the supervisor's proposal as an isolated alternative to the independent
nonlinear LOCAL smoother. Preserve the validated E1b.4a implementation as the
reference and rollback path. **Planning, preservation, S1 feasibility and B0's
unchanged timing reference are completed. Shared-linearization candidate
implementation and experiments have not started.**
See [S1 feasibility findings](GLIM_SHARED_LINEARIZATION_FEASIBILITY_S1_20261012.md)
for the actual MAIN implementation and exact lifecycle hooks.

The proposed architecture performs one nonlinear optimization in MAIN, retains
the individual native factors' Gaussian equations, and uses those equations in
a separate native-only Gaussian graph to obtain outgoing covariance. That graph
still needs elimination and its own marginalization history, but not another
nonlinear state optimization or another LiDAR correspondence search merely to
recreate the same linearization.

This is not the same hypothesis as E1b.4a. E1b.4a accelerates the existing LOCAL
solver without intentionally changing its mathematics. Sharing MAIN's
linearization policy changes where LOCAL information is evaluated. It must be
validated as a covariance-construction alternative, not presented as guaranteed
equivalence to the independent LOCAL estimator.

## Preserved working point

- Reference commit: `3206123f4812d66158db58bd1f577167fe901875`.
- Reference branch: `engineering/e1b4a-local-update-batching-20261012`.
- Local recovery tag: `checkpoint/e1b4a-before-shared-linearization-20261012`.
- Planning branch: `planning/glim-shared-linearization-20261012`.
- Timing evidence: [E1b.4a report](experiments/e1b4a_timing40_20261012/REPORT.md).
- Recovery manifest and boundaries:
  [checkpoint](experiments/checkpoints/20261012_shared_linearization/README.md).

The completed ETH1 first-40-second capture reduced LOCAL median time from
117.148 to 7.600 ms and complete-scan median from 307.376 to 182.365 ms.
Functional checks and deterministic receiver replay passed. This does not
certify a full 160-second run, real-time operation, or full-history mathematical
equivalence between different live source captures. Defaults remain unchanged.

MAIN still accounted for 43.264 s of profiled work and 162,254 synchronous GPU
linearizations in that capture. LOCAL update now accounts for only 2.856 s.
Removing the remaining LOCAL update alone therefore cannot reproduce the large
speedup already obtained. Measure covariance reconstruction and MAIN separately;
do not promise this proposal solves every remaining runtime bottleneck.

## What the covariance would mean

The intended definition is:

> Native GLIM factor information, linearized using MAIN's recorded reference
> states, with a separately maintained native-only marginal prior.

"Native" includes GLIM's own LiDAR, IMU, bias and initialization factors according
to the existing LOCAL admission rules. It excludes received CBS factors, their
explicit information blocks, and MAIN's combined marginal prior/Bayes tree.
The receiver's no-central-IMU setting does not remove GLIM's native IMU factors.

For a common increment coordinate vector, the Gaussian information is

\[
H_L = H_{P_L} + \sum_{f\in\mathcal N_{\rm active}} H_f,
\qquad H_f=J_f^T\Sigma_f^{-1}J_f.
\]

The implementation should use the Gaussian/Hessian factors GTSAM actually
produces, which already include their weighting; it must not whiten them again.
It must retain the linear and constant terms as well as the Hessian. A covariance
does not require a second nonlinear optimum, but the linearization point and
prior approximation remain part of its definition.

MAIN has been influenced by CBS. Its poses can affect native Jacobians,
correspondences and existing weights. Thus excluding explicit Kimera factors
does **not** make this a statistically independent CBS-OFF covariance, erase
all feedback, or calibrate uncertainty against actual error. The independent
LOCAL smoother and this proposal need not return identical covariances.

### Do not confuse MAIN's estimate with its linearization point

iSAM2 exposes both an estimate and the stored linearization point `theta`.
Cached equations may describe the latter, not the latest published pose.
The first feasibility gate must establish the exact chart and version for every
imported block. The initial target is MAIN's recorded linearized system, not an
unverified claim that all factors were re-evaluated at the current estimate.

Use the existing outgoing MAIN relative mean unchanged. Before applying the
existing relative-covariance propagation, express the endpoint joint covariance
in its required tangent coordinates. Validate any transport from the recorded
linearization chart to the current endpoint poses by finite differences. Include
both endpoint covariance blocks and their cross covariance; use the same full
6x6 relative-pose extraction convention, not independent endpoint marginals.

If the available MAIN cache cannot provide a coherent, traceable linearization,
stop this gate. A fresh native-only batched linearization at an explicitly chosen
state could still avoid nonlinear LOCAL optimization, but it would be a revised
proposal with additional GPU work—not a silently substituted cache hit.

## Reuse the existing implementation

The inspected baseline provides the following integration points. Locations
refer to the preserved versions, not to a promise that an existing public API
already exposes everything needed.

| Existing component | Proposed use | Important constraint |
| --- | --- | --- |
| GLIM bridge `onSmootherUpdate`, `updateSidecar`, `onSmootherUpdateFinish` | Identify native admission, MAIN update/temporary-CBS completion, publication, and retirement boundaries | A post-update callback alone cannot recover factors/keys already marginalized |
| GPU VGICP `linearize()` | Retain an immutable copy of the returned Gaussian/Hessian factor | Its temporary GPU result is consumed and reset; do not read it twice or share mutable caches between solvers |
| iSAM2 `linearFactors_`, `theta_`, insertion/relinearization/marginalization paths | Trace the exact factor generation and reference Values | Cached blocks are not a complete native-only posterior; slot reuse and marginal factors need provenance |
| GTSAM Gaussian factor graphs, elimination, Gaussian `Marginals` constructor | Compute native-only marginal information without nonlinear optimization | Gaussian elimination still costs time; do not reuse MAIN's combined Bayes tree |
| Existing LOCAL admission and relative-covariance helpers | Preserve factor classification, ordering and full endpoint covariance algebra | New chart compatibility must be demonstrated before reuse |
| E1/E1b tests, exact GPU snapshots and private build tools | Reuse correctness fixtures, isolated builds and timing instrumentation | Snapshot completeness for MAIN must be checked, not inferred from prior LOCAL tests |

Concrete source anchors:

- `runs/e1b4a_local_update_batching_20261012/release_2/libraries_source_snapshot/overlay/cbs_bridge.cpp`:
  `onSmootherUpdate` around 1239, `onSmootherUpdateFinish` around 1487,
  `updateSidecar` around 3255, outgoing mean/covariance extraction around 3615.
- `runs/e1b_local_profile_20261012/overlay/integrated_vgicp_factor_gpu.cpp`:
  `linearize` around 190 and the Hessian return around 211.
- `src/gtsam_points/src/gtsam_points/optimizers/isam2_ext.cpp`: actual MAIN
  cached/fresh equations around 77–143 and factor retirement/fixed separator
  handling around 699–754. Ordinary `gtsam::ISAM2` is the LOCAL path, not MAIN.
- `src/gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.cpp`:
  missing pre-retirement observation point between the update at 95 and
  marginalization at 105. Existing forwarded `getLinearFactors()` and
  `getLinearizationPoint()` expose the required cache and chart.
- `src/gtsam/gtsam/nonlinear/Marginals.cpp`: Gaussian-graph constructor around
  56; it eliminates a supplied linear graph rather than optimizing nonlinear
  states.
- `src/gtsam/gtsam/nonlinear/LinearContainerFactor.cpp`: `linearize` around 104.
  Its preserved implementation shifts Gaussian linear/constant terms. Do not
  assume that this alone proves arbitrary SE(3) covariance-chart transport.

These paths are relative to `/home/yeranis/repos/V4RL/cbs_gtsam4.3`.

## Independent native marginalization history

Maintain a native Gaussian factor registry and a separate prior `P_L`. Begin at
startup using the same native initialization constraints; never bootstrap from
MAIN's mixed marginal prior. Resuming halfway requires a verified saved native
prior with its reference states, not merely the current MAIN covariance.

At each successful backend update:

1. Stage new/relinearized native Gaussian blocks with ordered keys, dimensions,
   reference Values, stable factor identity and generation, backend epoch,
   geometry/weight version, and native provenance. Retain required retiring
   blocks before MAIN discards them. Capture MAIN-produced equations; do not
   call GPU `linearize()` again just to observe them.
2. Replace an old active block when that same factor is relinearized. Do not add
   both versions. Distinguish factor replacement, factor removal, and variable
   retirement explicitly. A reused slot or pointer is not a stable identity.
3. For expiring keys, eliminate them from `P_L` plus the native factors incident
   on them. Keep nonincident active factors separately. Retire each absorbed
   factor exactly once. Eliminating the entire graph and then re-adding retained
   factors would double count information.
4. Store the resulting native-only prior with its reference coordinates and
   frozen linearization policy. Do not manufacture a new nonlinear prior or
   silently rotate/reinterpret its Hessian when MAIN's chart changes.
5. Commit the new registry/prior only when the corresponding MAIN update succeeds.
   Failure, fallback, rollback or replay must leave both histories consistent.
6. Query the requested endpoint joint covariance, perform the validated tangent
   transport, and use the existing full relative-covariance extraction.

For a Gaussian system partitioned into removed variables `m` and retained `r`,
the reference algebra is

\[
H'_r=H_{rr}-H_{rm}H_{mm}^{-1}H_{mr},\qquad
\eta'_r=\eta_r-H_{rm}H_{mm}^{-1}\eta_m.
\]

Use GTSAM elimination/linear solves, not explicit inverses in production. Preserve
the native gauge constraints and rank handling. No hidden diagonal loading,
covariance scaling or extra anchor is permitted to make a test pass.

This is a separate Gaussian information history, **not** a second nonlinear
optimizer. Memory should follow the active native graph and its separator prior,
not retain every expired native factor indefinitely.

## Stages and stopping points

The stages below are proposals. Complete one, update the runtime ledger, report
its result, and obtain the next decision. Do not start a campaign automatically.

| Stage | Bounded work | Acceptance / stop condition |
| --- | --- | --- |
| S0 | Save baseline and this plan | Local checkpoint, integrity record and clean committed documentation; completed in this task |
| S1 | Read-only feasibility audit of MAIN equations, charts, factor lifetimes and retirement hooks | Completed: feasible with lifecycle hooks, native provenance and explicit chart/reset checks; no numerical or runtime certification. See S1 report |
| B0 | Fresh unchanged overall-timing reference before candidate implementation, as requested by the user | Completed and validated: same ETH1 first40s at0.10x, E1b.4a/E2 builds; functional checks and byte-identical receiver repeat pass. See experiments/shared_linearization_b0_20261012/REPORT.md |
| S2 | Default-off Gaussian information backend and synthetic/dense-reference tests in a private build | Complete: 26/26 Release including actual MAIN cache fixture; 25/25 UBSan and ASan+UBSan; dense-oracle and twelve-retirement gates pass. No live hooks or authoritative covariance change. See experiments/shared_linearization_s2_20261012/REPORT.md |
| S3 | Shadow validation on retained exact inputs where sufficient; at most one authorized 40 s capture if MAIN snapshots are missing | Existing LOCAL remains authoritative; compare candidate against a same-chart native Gaussian oracle through multiple retirements; no candidate CBS publication |
| S4 | Candidate-only LOCAL covariance timing with identical retained inputs, then one separately authorized matched 40 s live check | Disable the second nonlinear LOCAL solver only in the candidate; validate factor conservation, publication, covariance and end-to-end runtime; report expected covariance/trajectory differences from the old definition |

S2 is complete as an isolated default-off backend. S3 real lifecycle integration
and shadow validation is the next decision, not an automatic run. Both unchanged
timing captures remain preserved; another baseline capture is not required. S3 shadow
runtime is not the candidate performance result,
because both paths run together. Reuse one suitable capture rather than creating
separate data runs for every test. No full 160-second run or new sequence follows
automatically.

Proposed selector semantics are `independent_local` (unchanged default),
`shared_native_shadow` (compute but do not publish), and `shared_native` (publish
only after acceptance). These are design labels, not implemented launch flags.
Never change covariance providers midway through a candidate run without a
valid matching native history. On a candidate failure, stop and recover using
the preserved reference; do not silently substitute MAIN covariance.

## Required correctness tests

The primary oracle is an independently assembled native-only Gaussian graph at
the **same recorded charts, factor versions and prior history**. Equality to the
old independently optimized LOCAL graph is not the correctness criterion for
this different linearization policy.

Before live tests, require:

- Equal ordered Gaussian blocks, full information vector, Schur prior and
  endpoint joint covariance against dense reference elimination. For fixed,
  well-conditioned double-precision fixtures, predeclare relative Frobenius
  tolerances of `1e-9` for Gaussian/Schur algebra and `1e-8` for covariance;
  report absolute errors and conditioning too. Copy-through blocks should be
  exact. Freeze any GPU-specific tolerance from existing kernel evidence before
  examining candidate results, not from ATE.
- Multiple consecutive retirements, native initialization, IMU/bias variables,
  factor replacement, slot reuse, removals, stale-version rejection, rollback
  and deterministic repeat. Check that every native contribution is either
  active or absorbed exactly once.
- Direct-information exclusion: hold native equations and their charts fixed,
  vary external factors arbitrarily, and verify unchanged local information,
  including after retirement. Separately let external factors change MAIN's
  states and document the resulting indirect dependence; do not call that a
  violation of the stated covariance definition.
- Manifold/tangent transport and full relative-covariance Jacobians against
  numerical derivatives, including nonzero MAIN increments, rotations, dense
  endpoint cross covariance and nonzero pose/velocity/bias cross terms.
- Finite/symmetric covariance, rank/PSD checks within predeclared numerical
  tolerance, and explicit handling of underconstrained fixtures. No matrix
  projection or new noise floor to hide a failure.
- In shadow mode, candidate code must not alter MAIN equations, GPU inlier/error
  caches, source means, outgoing authoritative covariance, factor admission or
  solver decisions on identical inputs. Measure observation overhead separately.
- Relevant Release tests and focused ASan/UBSan, respecting GPU sanitizer limits
  already recorded for this project. Preserve existing tests.

For real data, inspect early, middle and late active windows, including multiple
5-second GLIM lag retirements. Check the full 6x6 output rather than traces only.
The dense oracle must not simply call the candidate implementation again.

## What remains frozen

GLIM's MAIN optimization, native factors/noise models, 5-second lag, sensor
calibration, incoming CBS timing/alignment and removal rules remain unchanged.
Kimera stays on the saved ETH1 front-stereo/CPT7 25-state profile with its current
experimental IMU-support override unset. Classic `active_window_temporary`
source-to-source CBS stays distinct from downstream stable-ID revisions.

The gnssFGO receiver remains full-history source-driven WNOA, independent 100 ms
support clock, E2 exact-Values reuse, no central IMU/bias/acceleration/GNSS,
Qc `[.00375,.00375,.00375,.015,.015,.015]`, existing priors, covariance scale 1,
Gaussian factors and strict nominal publication checks. No Huber, gating, health
weighting, cadence change, covariance tuning, alternate receiver or MAIN speed
patch belongs in this experiment.

When candidate covariance is eventually published, the coupled CBS system can
change numerically even though the native estimator settings are frozen. Such
changes must be reported, not hidden as a pure runtime optimization. Compare
original GLIM, original Kimera and fused online/final outputs under the same
coverage conventions. Accuracy is an observation, not a selection criterion for
the linearization policy or its parameters.

## Decision criteria

Proceed beyond the short candidate only if native information is accounted for
correctly, chart/history semantics are explicit, the oracle tests pass, and
end-to-end timing shows a worthwhile benefit. Report median/p95/max for MAIN,
LOCAL/covariance, complete GLIM update, receiver queue age and output latency;
also report GPU calls, factor counts and memory. Do not sum overlapping timers.

Reject or revise the proposal if reliable equation reuse requires extensive
extra evaluations, coordinate/prior provenance cannot be guaranteed, or most
runtime remains elsewhere with little overall benefit. A mathematical pass
without a runtime benefit is not a speed solution. A speed pass does not prove
calibrated uncertainty, independent source information, better ATE or longer-run
capacity.

The safe fallback is the retained E1b.4a independent LOCAL implementation—not
discarding its successful batching work and not extracting MAIN's mixed posterior.
