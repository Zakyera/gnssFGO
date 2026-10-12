# Conservative rank certificate test

S4c, authorized 2026-10-12. Parent feb737397453e10e8f13423993df020e41bbd3e6.
Private experiment only. Preserve S2/S3/S4a/S4b, active LOCAL, native covariance,
CBS, source settings and all frozen artifacts. No new sensor or fusion run.

## Single hypothesis

Replace the expensive active-Hessian rank inspection with a sufficient numerical
certificate when possible; otherwise execute the unchanged S4b full eigenvalue
test. Keep the cheap 12D endpoint rank check unchanged. Do not replace the rank
policy with successful Cholesky alone or a heuristic reciprocal condition estimate.

An unshifted factor alone does not certify the required spectral threshold.
Use pinned Eigen's existing sparse Cholesky on a temporary shifted matrix, then
check a conservative reconstruction-error bound. The original GTSAM elimination
and covariance path remain byte-for-byte unchanged. No shift/jitter is added to
any source information, covariance or factor used for estimation.

## Certificate derivation and numerical boundary

Let H be the symmetric matrix defined by the input's lower triangle, exactly as
the existing Eigen eigensolver reads it. Keep its finite/symmetry/empty guards.
Normalize H by an exactly reversible power of two, c, to obtain A=H/c. Fall back
if normalization is not exactly reversible, unsupported floating-point mode,
nonfinite arithmetic or dimension outside 1..4096. Use double, no fast-math,
IEEE round-to-nearest, gradual underflow. Do not alter the process rounding mode.

An outward-rounded infinity norm U bounds lambda_max(A). Define an upward bound
t on 1e-12 * max(1/c,U). The original exact spectral acceptance criterion is
lambda_min(A) > 1e-12 * max(1/c,lambda_max(A)).

Set gamma=gamma_(2n+8), using machine epsilon rather than half-epsilon as a
conservative operation-rounding bound. Select fixed shift s=t+32*gamma*U.
Factor the temporary A-sI using existing Eigen SimplicialLLT/AMD. Obtain L and
permutation P. Compute the residual of the *unrounded target* expression

    E = P A P^T - s I - L L^T.

Bound ||E||_2 by an outward-rounded row/column norm bound of its computed
residual plus gamma*(U+s+||L||_inf*||L||_1), with a conservative absolute
underflow allowance. This also accounts for the two residual subtractions;
the factorization's own errors are observed in the residual, not assumed away.

Since L L^T is positive semidefinite regardless of factorization error,

    lambda_min(A) >= s - ||E||_2.

Accept the fast path only if its downward-rounded lower bound exceeds t by
an additional 8*gamma*U guard. Otherwise use the original eigensolver, including
its exact failure reasons. The fixed shift/guard constants are numerical
conservatism, not a changed rejection threshold or tuned estimator parameter.
No sweep or adjustment after seeing the retained-data timing is allowed.

The analytic certificate targets the represented matrix's exact spectral
criterion. The original floating-point eigensolver is itself approximate;
agreement at every possible floating-point threshold is not claimed. Borderline
cases must fall back and empirical decision-equivalence is tested explicitly.
Only the active-Hessian call uses this wrapper. Every fallback repeats the exact
old code. Certificate construction failure never grants acceptance.

## Acceptance and stop gates

1. Preserve all 31 S4b tests (25 history plus six rank groups). Add focused
   tests for certified diagonal/banded/rotated sparse SPD matrices, permutations,
   ill-conditioned and indefinite matrices, near-threshold fallback, unsupported
   normalization, forged/bad reconstruction, and deterministic repeated checks.
   Test the certificate's lower bound against independent dense eigenvalues;
   every fast acceptance must also pass the original criterion.
2. Release and focused non-PIE ASan/UBSan, including saved pose and mixed fixtures;
   prebuilt GTSAM itself is not sanitizer-instrumented. No retry on a numerical
   disagreement; archive and stop. Build/harness issues may be repaired explicitly.
3. Saved ETH1 native journal: all783 updates,313 retirements,20 queries,
   5321 admitted =4513 absorbed +808 active. Full covariance/transport output
   byte-identical to S4b; original1e-9 information/1e-8 covariance oracle gates.
4. Only after correctness: one warm-up pair and three interleaved measured pairs
   against S4b's38ms candidate, not S4a's156ms implementation. Report median/p95/
   max, early/middle/late, certificate successes, failures/reasons, fallback rate,
   and total check/covariance time. No compilation during measured timing.
5. Material speed gate: at least25% lower median query time, faster query-total
   in every pair, and no p95 regression. If high fallback/overhead prevents this,
   report the failure and stop; no second numerical strategy in this task.

This does not compare against old independent LOCAL or certify live GLIM speed.
Keep the20-query sampling limitation. Only after a separate authorization could
a matched40s old-provider/replacement live test disable old LOCAL. No automatic
promotion, full sequence, source-covariance tuning or next method.

## Preservation

Private root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_certificate_20261012`.
Archive code, tests, exact commands, binary/input hashes, raw output and report.
Verify S3 protected716 files, baseline680 files,19 S1 pins, six source HEAD/status
checks and S4a/S4b identities. Update the shared plan and runtime ledger; local
commit only, then stop.
