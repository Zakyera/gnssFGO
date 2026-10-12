# Native Gaussian covariance prototype validation

## Result

S2 passes its isolated correctness gate. The prototype maintains native-only
Gaussian information, performs fixed-lag Gaussian elimination and extracts the
full relative covariance without running a second nonlinear LOCAL optimizer or
re-evaluating native LiDAR/IMU factors. A synthetic test using the actual
`gtsam_points::ISAM2Ext` cache adds zero native factor evaluations and leaves
MAIN's equations and estimate unchanged.

This is **not yet live GLIM integration or a measured speedup**. The current
independent LOCAL implementation remains authoritative and unchanged. No dataset,
ROS estimator, fusion experiment, tuning, production build or publication switch
was run. No covariance was sent to Kimera or gnssFGO by this prototype.

## Implemented boundary

`NativeGaussianHistory.h/.cpp` accepts a snapshot containing positively admitted
native factor IDs, versions, cache slots, MAIN reference Values, retiring keys and
the post-retirement fixed-key contract. Its default mode is `Disabled`; explicit
`Shadow` enables computation only. There is no publication mode or launch flag.

The helper uses existing GTSAM Gaussian factors, `EliminateCholesky`, Gaussian
Bayes-tree joint marginals, the compiled Pose3 retraction derivative, and the
unchanged CBS `schur_relative_between` helper. It does not reconstruct raw GPU
Jacobians, whiten cached equations twice, scale information, add anchors or
optimize new nonlinear states. The small endpoint relative-pose Jacobian remains
part of the existing covariance extraction, not another LiDAR evaluation.

Each transaction:

1. Checks generation/epoch, native identity/version/slot, dimensions and finite
   equations; deep-copies the positively admitted cached equations.
2. Replaces refreshed active equations instead of adding duplicate information.
   Unknown removals, silent removals, stale versions and reused retired IDs fail.
3. Combines the separate native prior with native factors touching retiring keys.
   GTSAM eliminates those keys; retained nonincident factors are not absorbed and
   added twice. The prior retains Hessian, information vector and constant term.
4. Verifies that prior reference charts did not move and retained prior keys are
   fixed by MAIN. It does not rebase a prior silently or import MAIN's prior.
5. Stages the result; only an explicit matching commit makes it queryable.
   Abort or Gaussian-elimination failure leaves committed information unchanged.
   Reset clears history and requires rebuilding from native initialization.
6. Queries the endpoint joint covariance, including cross blocks, transports it
   to the current endpoint tangents and applies the existing relative Schur
   extraction. Underconstrained, constrained-noise or jitter-requiring queries
   fail explicitly. Observer-facing `tryCovariance` contains exceptions.

The coordinate transport is
`A_i = d Local(X_i, Retract(theta_i, delta_i + epsilon)) / d epsilon` at zero,
where `X_i = Retract(theta_i, delta_i)`. The current joint covariance is
`diag(A_i,A_j) P_theta diag(A_i,A_j)^T`. Finite-difference tests use nonzero
rotations/translations and dense endpoint, pose/velocity and pose/bias coupling.

The native admission registry and pre-retirement hook are supplied by the test
adapter in S2. S3 must connect these to GLIM's actual admission and smoother
lifecycle. Passing a falsely labelled native ID is outside this helper's trust
boundary; factor class alone is deliberately not used to infer provenance.

## Correctness results

The independent oracle constructs `J^T J`, `J^T b` and `b^T b` from the original
fixture matrices, retains their historical versions in the test only, and uses
dense LDLT Schur elimination. It does not call the candidate elimination code.
The fixtures mix 6D poses, 3D velocity and 6D biases. They are algebraic Gaussian
fixtures, not a claim to reproduce the physical GLIM IMU model.

| Check | Maximum absolute Frobenius error | Maximum relative error | Gate |
| --- | ---: | ---: | ---: |
| Native augmented information through retirement | 1.34124e-14 | 8.34148e-16 | 1e-9 relative |
| Initial endpoint joint covariance | 3.25214e-15 | 1.36307e-15 | 1e-8 relative |
| Endpoint covariance after repeated retirement | 4.46715e-15 | 8.30520e-16 | 1e-8 relative |
| Transported covariance versus numerical chart derivative | 8.17892e-10 | 3.16318e-10 | 1e-8 relative |
| Relative Schur covariance versus numerical Jacobian propagation | 3.21448e-10 | 1.49292e-10 | 1e-8 relative |

Maximum reported fixture information condition number: **154.927**. These are
well-conditioned synthetic gates, not certification for all real GLIM windows.
Copy-through equations and repeated numerical diagnostics match exactly. The
numerical derivative uses the predeclared central step `1e-6`; its larger error
than the algebraic oracle is expected and remains below the frozen tolerance.

The 25 helper tests cover disabled mode, mixed Hessian/weighted Jacobian input,
full covariance cross blocks, twelve successive retirements, no double counting,
refresh/replacement, explicit removal, slot reuse, stale/duplicate/missing IDs,
nonfinite/dimension failures, prior-chart/fixed-key guards, abort/reset, singular
elimination rollback, underconstrained queries, complete component removal,
constant preservation, empty-history queries, deterministic repetition and deep
copy protection. After twelve retirements, 27 admitted native factors account for
26 absorbed plus one active factor; only six Values remain. Expired factors are
not retained by the candidate.

Changing excluded external Gaussian equations by a large amount has exactly zero
effect with native equations/charts held fixed, including after retirement.
Changing native equations changes the covariance. Thus this removes **direct
external Gaussian information**, not the indirect influence of CBS on MAIN's
state, correspondences or native linearizations. It is not a calibrated or
statistically independent covariance claim.

## Actual MAIN cache check

The additional Release fixture uses the installed CPU `ISAM2Ext`, native pose
prior/relative factors and a synthetic external relative factor of the same
class. Native slots come from `newFactorsIndices`, not inferred factor types.
It captures cached equations and `theta` before `marginalizeLeaves`, then commits
after MAIN's normal retirement. Counted native/external factor evaluations do
not increase when the observer stages, commits or queries covariance. MAIN's
cached information and estimate remain exactly unchanged by observation.

This tests the real solver interface using CPU pose factors. It does not exercise
actual GPU LiDAR caches, live provenance callbacks, fallback/reinitialization
notifications or the complete ROS source lifecycle. Those remain S3 gates.

## Builds and evidence

All final tests use the frozen private GTSAM binary SHA256
`369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37`.
The MAIN-cache fixture additionally loads the existing protected CPU
`libgtsam_points` and private CUDA dependency; no GPU sensor factor is evaluated.
Builds and execution occurred inside the existing `cbsms_ws_gpu` container.

| Private build | Result | Scope |
| --- | --- | --- |
| `release_6` | 26/26, repeated 26/26 | Helper plus actual MAIN cache fixture |
| `ubsan_2` | 25/25 | Helper and independent oracle; no sanitizer findings |
| `asan_2` | 25/25 | Helper and oracle, ASan+UBSan and leak detection; no findings |

Existing linked libraries were not rebuilt with sanitizers; this is focused new
CPU-code coverage, not GPU/whole-workspace sanitizer certification. The final
source bytes match all three private build snapshots. Test logs, XML, loaded
library paths, build commands, executable hashes and numeric summaries are in
`evidence/`. `integrity_before.json` and `integrity_after.json` verify all 680 B0
protected files, 19 S1 pins, six unchanged clean source repositories and the
MAIN CPU library.

Development attempts remain under
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s2_20261012/`.
The first compile exposed pinned Eigen/API mismatches; these were corrected in
the private build. A lazy Eigen expression in a test fixture was materialized
before final validation. Early test launches loaded workspace libraries because
the container environment overrode the executable search path; those are not the
acceptance runs. Final launches set a scoped library path and check the actual
GTSAM hash before executing. No installed files were changed to resolve these.

## Remaining work and stopping point

S3 should add default-off real native-admission and pre-marginalization capture
hooks, stable generation/identity/cache-refresh metadata, fixed-chart validation,
and observer-contained fallback/transaction handling. Validate on exact retained
MAIN equations if available, otherwise seek authorization for one short shadow
capture. Independent LOCAL must remain authoritative during that test.

The prototype currently assembles dense information for its rank guard and uses
Gaussian elimination for each covariance query. This is a correctness-first
implementation; it does not yet establish efficient query caching or end-to-end
capacity. A live speed claim belongs to the later timing gate, not these small
synthetic timings. The new covariance definition may differ from independently
optimized LOCAL even when it is correct.

Stop at S2. No automatic S3 capture, timing run, covariance promotion, receiver
change, full-sequence experiment or GitHub push follows this result.

Parent checkpoint: `4b2804cf094c590c8404e3c5fecd00ce682b2dbc`. Implementation and
validation are saved in the commit containing this report, on the isolated
`engineering/shared-native-gaussian-s2-20261012` branch. The recovery tag
`checkpoint/e1b4a-before-shared-linearization-20261012` is unchanged.
