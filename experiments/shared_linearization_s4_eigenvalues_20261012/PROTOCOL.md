# S4b: eigenvalues-only rank check

Authorized 2026-10-12 after S4a; parent 9c7c89001bf4cc9bfdab3c9d26214e4ac3766804.
Scope: a private copy of the native Gaussian history, not the active LOCAL
provider. No sensor run, estimator edits, new covariance, tuning or promotion.

## Single hypothesis and frozen change

The dense positive-definiteness check uses only eigenvalues, but currently asks
Eigen for eigenvectors too. Replace exactly one constructor with
`Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h, Eigen::EigenvaluesOnly);`.
Keep the existing strict `min > 1e-12 * max(1, max)` test and all other code.
The helper checks both the active Hessian and transported endpoint covariance.
Do not remove the dense check or introduce jitter/fallback/threshold changes.

Use the exact S4a standalone reader and timer-only transformation. Reference
binaries and objects come from S4a's retained Release and non-PIE ASan builds.
Build new candidate objects separately; never overwrite originals. Private root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_eigenvalues_20261012`.

## Correctness gates before timing

- Existing 25 Gaussian history tests unchanged, including dense covariance,
  rank rejection, native-only history, retirement and transactional rollback.
- Test actual original and candidate rank helpers on finite/symmetric, empty,
  nonfinite, nonsquare, asymmetric, positive-definite, indefinite and singular
  matrices. Test diagonal threshold equality and adjacent representable values
  at multiple scales, plus deterministic rotated dense near-threshold matrices
  and representative 12/96/765 dimensional systems. Require identical accept /
  reject decisions and reasons; do not change tests or tolerances to force a pass.
- Pose-only and mixed pose/velocity/bias saved fixtures: byte-identical complete
  query output; corrupted-header/truncated-input failures stay failures.
- Recorded ETH1 first40s native journal: all783 stage/commits,313 retirements,
  20 covariance queries; require byte-identical query matrix output against
  S4a/reference. Retain independent recorded oracle gates of1e-9 information
  relative error and1e-8 covariance relative error.
- Focused ASan/UBSan tests and both saved fixtures. Use non-PIE link from the
  start, as S4a's original PIE sanitizer startup failure is unresolved. Prebuilt
  GTSAM itself is not sanitizer-instrumented. Do not retry functional failures.

## Matched timing

After correctness passes, replay identical saved native equations with no
compilation running. One warm-up pair then three interleaved measured pairs,
alternating reference/candidate order. Each pass includes all783 updates and
the20 saved covariance checkpoints. Report median/p95/max, per-pass totals,
early/middle/late epoch bins, and byte identity. Run a separate instrumented
reference/candidate pair to attribute rank-check cost; do not mix those times
into the primary uninstrumented comparison. Use identical thread environment.

Acceptance requires correctness plus a repeatable measured reduction in query
time; no speed threshold is achieved by relaxing mathematics. Desktop CPU clocks
are uncontrolled: report environment and paired spread. Saved Hessian factors
do not reproduce original GPU subclass cloning cost. Only20 query checkpoints
were retained versus389 direct LOCAL queries in S3. Any cadence projection is
explicitly not an all-query or live benchmark. No end-to-end speedup claim.

## Integrity and stop

Verify S2 source pins, S3 protected716 files, S4a identities, baseline680 files,
S1 pins and six source repository HEAD/status checks. Preserve all earlier
evidence. Save patch/scripts/raw timings/test output/report, update both plans,
commit locally, and stop. Old independent nonlinear LOCAL remains authoritative.
