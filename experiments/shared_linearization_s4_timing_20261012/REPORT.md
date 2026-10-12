# Dense rank inspection dominates the shared covariance query

The unchanged candidate spends **95.94% of its covariance-query time in a dense
information-matrix rank check**. Maintaining its separate native Gaussian history
is inexpensive on the retained ETH1 first40s inputs. The current implementation
is **not ready to replace LOCAL as a speed improvement**; the measured bottleneck
is the candidate's validation work, not another LiDAR evaluation or nonlinear
optimization. No estimator was run and no algorithm was changed in this task.

## Scope and numerical checks

S4a replays S3's saved equations through the unchanged S2 implementation. Each
pass includes all 783 stage/commit updates, 313 retirements and 20 exact saved
endpoint covariance requests. One warm-up per variant precedes three interleaved
original/instrumented Release passes. The instrumented copy adds timer statements
only; stripping them produces the original source byte-for-byte.

The journal contains weighted Gaussian information matrices, ordered keys and
dimensions, reference poses/velocities/biases, factor identities and versions,
retirements and committed-count records. It does not preserve whether each input
originally used a JacobianFactor or HessianFactor subclass. The standalone reader
reconstructs Hessian factors, excluding file parsing from the measured candidate
operations. Thus the equations are identical but factor conversion/copying costs
need not exactly match an in-process MAIN-cache observer.

Every pass accounts for 5321 admitted native contributions as 4513 absorbed and
808 active. Original and instrumented full output covariance/transport matrices
are byte-identical across all eight real-input passes. Maximum relative
Frobenius differences against the saved S3 outputs are 1.6943e-17 for augmented
information and 3.7292e-15 for covariance/transport, within the frozen 1e-9/1e-8
gates. This reuses S3's independently validated reference; it is not a new
independent proof of its equations.

## Cost of the unchanged implementation

Pooled statistics below cover the three measured passes, including startup.
History stages and commits have 2349 samples each; covariance queries have 60.
All numbers are milliseconds. Subphase medians need not sum to total medians.

| Operation | Median | p95 | Maximum |
| --- | ---: | ---: | ---: |
| Original history stage | 0.682 | 0.923 | 1.753 |
| Original history commit | 0.035 | 0.060 | 0.468 |
| Original full covariance query | 160.274 | 167.686 | 170.294 |
| Instrumented full covariance query | 160.480 | 168.006 | 191.610 |
| Clone query graph | 0.102 | 0.132 | 0.151 |
| Assemble dense information | 3.052 | 3.304 | 3.517 |
| Dense symmetry and eigenvalue rank check | **153.513** | **160.205** | **183.718** |
| Sparse Gaussian elimination | 0.836 | 1.464 | 1.766 |
| Joint endpoint marginal | 1.799 | 3.213 | 3.942 |
| Extract and transport endpoint covariance | 0.007 | 0.008 | 0.011 |
| Check 12D endpoint covariance rank | 0.009 | 0.011 | 0.015 |
| Existing relative covariance helper | 0.034 | 0.047 | 0.261 |

In each instrumented pass, the dense check takes about 2.712 s of the 2.827 s
spent on the 20 queries. Other query work totals about 0.115 s. Its per-query
median is 6.330 ms **excluding the dense check**, but that is a decomposition,
not an accepted implementation with the safety check removed.

History stage plus commit totals approximately 0.535 s per original 783-update
pass. Native-factor validation/cloning dominates staging (instrumented median
0.440 ms), followed by finalization/key bookkeeping (0.157 ms). At the 313 actual
retirement events, Gaussian retirement elimination itself has median 0.038 ms,
p95 0.278 ms and maximum 0.647 ms. It is not the bottleneck on this window.

The original full-query median is 92.087 ms in epochs 1–195, 161.034 ms in epochs
196–587, and 162.969 ms in epochs 588–783. These are **epoch bins**, not exact
ten-second timestamp masks. Query cost rises as the window fills, then remains
similar across the middle and late samples. No longer-duration claim follows.

Instrumentation changes combined measured stage/commit/query totals by -0.54%
across the repeats, within observed run variation, not a speed optimization.
One instrumented query has a retained 191.6 ms tail. The original median closely
reproduces the live shadow's 159.656 ms. CPU: Intel Core Ultra 9 275HX; reported
governor powersave; clocks/desktop load were not locked. No compilation overlapped
the measured passes. Parsing, numeric comparison and output are separately timed
in the archived logs; they are not counted as covariance work.

## Why the dense check is expensive

`NativeGaussianHistory::covariance()` first materializes the active dense Hessian
and calls `positiveDefinite()`. That helper constructs
`Eigen::SelfAdjointEigenSolver<Matrix> eigen(h)` with its default options.
The pinned Eigen header confirms that the default is `ComputeEigenvectors`.
The code then uses **only the minimum and maximum eigenvalues**, enforcing
`lambda_min > 1e-12 * max(1, lambda_max)`. The eigenvectors are unused.

This computes a full dense spectral decomposition of up to 765 scalar variables
at the saved query checkpoints before performing the sparse Gaussian elimination
needed for the requested 12D covariance. Timing includes matrix validity checks
with the eigensolve; it does not separately prove how much of the 153.5 ms comes
from eigenvectors rather than the eigenvalues themselves.

## Actual query cadence and comparison boundary

The retained GLIM node log contains **389** direct outgoing-pair calls, each
requesting and returning one covariance/belief. The source ran at roughly 10 Hz;
the shadow computed only 20 of those requests. There were also 17,331 later
revision diagnostic rows. Those use MAIN's separate `jointMarginalInformation()`
path and are **not 17,331 extra calls to this LOCAL covariance provider**.

Code anchors in the preserved S3 private bridge:

- `onSmootherUpdateFinish`, lines 1521–1555: separate MAIN revision diagnostics.
- `publishOutgoingBeliefs`, lines 3354–3373: old LOCAL covariance reconstruction.
- `getOdometryBeliefsDirectForPairs`, lines 3640–3707: one endpoint query per pair.
- Lines 3845–3850: retained successful-pair count logging.

Multiplying the unchanged candidate's sampled mean query time by 389 gives about
55.17 s of query compute, plus approximately 0.535 s of history work. This is a
**cadence extrapolation**, not a measurement of all unsampled requests or total
GLIM runtime. Only 20 exact query inputs are retained in the native journal.

B0's already-batched independent LOCAL update plus covariance-reconstruction
scopes total 3.415 + 4.164 = 7.579 s over its separate first40s live capture;
their medians are 10.901 and 12.066 ms respectively. Those are different evolving
graphs and a different covariance definition. Nevertheless, the current 160 ms
candidate query clearly does not justify disabling the faster preserved LOCAL
path. Neither proposal removes MAIN's remaining native LiDAR optimization cost.

## Validation and limits

- Release 25/25 existing S2 tests pass; original/instrumented pose and mixed
  IMU/pose/velocity/bias journal fixtures have identical full covariance outputs.
- Focused ASan/UBSan 25/25 tests and both saved fixtures pass with leak checking
  using the private non-PIE executables. Bad magic and truncated-journal controls
  fail as required. Prebuilt GTSAM/dependencies are not sanitizer-instrumented.
- The first PIE sanitizer attempt passed all 25 unit tests and both pose runs,
  then terminated on mixed-fixture startup with `AddressSanitizer:DEADLYSIGNAL`,
  before any output file or stage record. Its logs are preserved. Relinking the
  **identical compiled objects** with `-no-pie` completed the bounded validation.
  This is a test-harness workaround, not proof of that startup signal's root
  cause and not a production-code fix. Release timing binaries were unchanged.
- All 716 S3 protected files, 680 inherited baseline files, 19 S1 pins and six
  source HEAD/clean-worktree checks remain unchanged. Input and executable hashes,
  commands, raw timings, failed-attempt logs and numeric results are retained.
- MAIN-cache capture/copying, ROS transport, GPU contention, original nonlinear
  LOCAL work and actual all-belief publication are outside this standalone timing.
  Nothing was published, tuned, installed or enabled in an estimator.

## Next isolated change

Keep the current provider. The smallest next patch is to request
`EigenvaluesOnly` in the private rank-check helper, retaining the same dense
matrix, exact threshold, finite/symmetry checks and rejection behavior. Test
well-conditioned, near-threshold, singular and indefinite fixtures and the
same saved equations, then repeat matched timing. This removes unused work;
the remaining dense eigenvalue calculation may still be too slow.

Do not replace the rank policy with a mere successful Cholesky, delete the check,
add jitter, or change covariance to make timing pass. Those would require a
separate numerical-policy decision. No such patch or live run occurred here.

S4a cost attribution is complete. Authoritative covariance remains the old LOCAL
provider. Parent `36ded67cc20275d55656842bc39a293810e390d4`; this report and harness
are saved on `engineering/shared-native-gaussian-s4-timing-20261012` in the commit
containing this report. No push or follow-up experiment.

S4A_COST_ATTRIBUTED_NO_PROMOTION
