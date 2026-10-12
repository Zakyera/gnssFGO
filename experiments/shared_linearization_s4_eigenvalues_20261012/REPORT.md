# S4b: same covariance, 4.1× faster query on saved equations

The private shared-native covariance query is **4.12× faster by median** after
requesting eigenvalues only in its rank check: **156.711 → 38.045 ms** in the
matched measurement. All saved covariance outputs are byte-identical. The rank
threshold and all other numerical operations remain unchanged.

**Accept this isolated private optimization, not a production replacement.**
The remaining dense rank check still dominates. No new sensor or estimator run
occurred; the independent nonlinear LOCAL implementation remains authoritative.
This result is not an end-to-end GLIM speedup or a change in published covariance.

## One-line change and mathematical scope

The sole numerical-code change is in a private copy of
`NativeGaussianHistory.cpp`, in `positiveDefinite()`:

```cpp
// Before
Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h);
// After
Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h, Eigen::EigenvaluesOnly);
```

The pinned Eigen header documents the default as `ComputeEigenvectors`
(`devel/include/gtsam/3rdparty/Eigen/Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h`,
lines159–173). Its `compute()` at lines431–460 passes the option through
tridiagonalization and the eigensolver. Our helper never uses eigenvectors: it
checks solver success and the strict condition
`lambda_min > 1e-12 * max(1, lambda_max)`. Finite/symmetry checks and that exact
condition are unchanged. The option applies to both the active Hessian and the
12D transported joint endpoint covariance check. Neither check is removed.

Native-only factor admission, reference charts, retirement, Gaussian elimination,
endpoint cross-covariance, tangent transport and the existing Schur relative
covariance helper are unchanged. No new correlations, approximation, jitter,
threshold, source covariance or CBS parameter was introduced. The original S2
implementation, S3 capture and S4a reference remain untouched. See
[exact patch](candidate.patch) and [candidate source](evidence/candidate.cpp).

## Inputs and validation before timing

The [predeclared protocol](PROTOCOL.md) uses only S3's saved weighted Gaussian
equations and reference charts. Each real-input pass reconstructs all783
stage/commit updates,313 retirements and20 saved covariance requests. Conservation
is exact:5321 admitted =4513 absorbed +808 active; zero explicit removals.

- **31/31 Release and31/31 focused ASan/UBSan tests pass.** The25 existing S2
  tests are unchanged, including dense-reference covariance, rank rejection,
  native-only marginalization, bounded history and transactional failure.
- Six new test groups compare the **actual candidate helper** with verbatim
  original helpers. All130 tested matrices give identical decisions and failure
  reasons. Coverage includes exact diagonal threshold equality and adjacent
  representable values, scales0.25/1/1e6,90 deterministic rotated near-threshold
  cases,12/96/765-dimensional matrices, singular/indefinite/nonfinite/nonsquare/
  asymmetric/empty inputs. This is bounded numerical validation, not a proof
  for every floating-point matrix arbitrarily close to the threshold.
- Pose and mixed pose/velocity/bias saved fixtures produce identical complete
  query files between reference and candidate in Release and ASan/UBSan.
  Corrupt-header and truncated-input controls fail as required.
- All12 real-journal passes (two initial validation, eight warm-up/measured,
  two separate instrumented) match the original S4a output byte-for-byte. No
  accepted query is lost. Maximum relative Frobenius differences against S3's
  recorded oracle remain **1.6943e-17 information**, **3.7292e-15 covariance /
  transport**, within the unchanged1e-9/1e-8 gates. Reusing that oracle does not
  make this a new independent proof of its equations.
- New sanitizer executables are linked non-PIE from the outset, retaining S4a's
  documented workaround for its unresolved PIE startup signal. No failed S4b
  attempt or retry occurred. GTSAM/dependencies are prebuilt and are not themselves
  sanitizer-instrumented. The real journal is checked in Release; sanitizers
  cover the focused tests and small saved fixtures.

Test-only access to the private helper does not expand the production API;
see [rank tests](rank_tests.cpp) and [test access](evidence/rank_access.cpp).

## Matched timing results

One warm-up pair followed by three alternating-order measured pairs. All builds
and sanitizer tests completed before timing. Reference binaries/reader objects
are the preserved S4a ones; new candidate objects use the sameRelease flags and
libraries. All processes use the same thread environment. CPU: Intel Core Ultra9
275HX, powersave governor, unlocked desktop clocks. No compiler/estimator process
was detected in the experiment container before measurement; this is not a
globally isolated real-time benchmark.

Milliseconds below pool three passes:60 queries or2349 stage/commit calls.
Parsing, saved-oracle comparison and binary output are excluded from these scopes.

| Operation | Original median | Candidate median | Original p95 | Candidate p95 | Original max | Candidate max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Complete covariance query |156.711|38.045|164.459|39.834|165.753|41.540|
| History stage |0.678|0.666|0.913|0.878|1.658|2.273|
| History commit |0.034|0.033|0.058|0.058|0.491|0.411|

Mean query time is138.643 →33.877ms, a75.565% reduction. Query-total speedups
are4.000×,4.089×,4.190× across the three pairs; the gain is not a single outlier.
Each candidate pass spends0.672–0.683s on20 queries, versus2.733–2.816s originally.
History stage/commit totals average0.515s per783 updates, versus0.524s originally;
the small difference there is not an algorithmic stage optimization.

| Saved query epoch bin | Original median ms | Candidate median ms |
| --- | ---: | ---: |
|1–195 (15 pooled queries)|91.273|22.967|
|196–587 (30)|157.324|38.465|
|588–783 (15)|158.684|37.893|

These are epoch bins, not exact ten-second masks. They show the gain persists
after the active window fills. They do not establish longer-sequence behavior.
The new reference median differs from S4a's160.274ms by about2.2%; use this
matched156.711ms reference for the4.12× result, not unmatched earlier timings.

## What remains expensive

A separate instrumented reference/candidate pair uses S4a's unchanged timer-only
transformation. Its measurements are not mixed into the primary timing above.

| Query subphase | Original median ms | Candidate median ms |
| --- | ---: | ---: |
| Dense symmetry/eigenvalue rank inspection |149.984|33.351|
| Dense information assembly |3.098|3.151|
| Sparse Gaussian elimination |0.820|0.715|
| Joint endpoint marginal |1.774|1.715|
|12D endpoint rank check |0.009|0.006|

The dense active-rank check is still about**83.9%** of total instrumented
candidate query time. Removing unused eigenvectors helped substantially, but
computing all eigenvalues of the dense matrix (up to765 scalar dimensions)
remains expensive. This is not another nonlinear solve or GPU LiDAR evaluation.
It is validation of the candidate's native information matrix.

S3 observed389 direct outgoing LOCAL queries but retained only20 exact query
snapshots. Multiplying this candidate's sampled mean by389 projects**13.178s**
of covariance work, plus about0.515s history work, compared with53.932s query work
for the unchanged candidate. **This is a cadence projection, not a measured
all-request runtime.** The reconstruction uses Hessian factors rather than the
original cached Gaussian subclasses, so copying/conversion costs can differ
from an in-process observer.

For context, B0's already-batched independent LOCAL update and covariance scopes
totaled7.579s in a separate first40s live capture. The projected shared-provider
cost still does not justify claiming a speed improvement over that preserved
path. These are different evolving graphs and covariance definitions—not a
matched old-LOCAL/new-LOCAL comparison. MAIN optimization, capture overhead,
ROS arrival interleaving and actual covariance publication are absent here.

## Integrity, artifacts and decision

All716 S3 protected files,680 inherited baseline files,19 S1 pins,7 S4a identity
pins and six source repository HEAD/clean-status checks pass. The old CPU library
and runtime binaries are unchanged. The candidate source differs from S2 by only
the constructor option; removing timing lines from the instrumented candidate
reproduces the candidate exactly. No source profile or production selector was
changed. Existing LOCAL remains authoritative; no new covariance was published.

Evidence:

- [Machine-readable results](summary.json), [binary/input hashes](hashes.json),
  [test and timing evidence](evidence/timing_results/complete.json).
- Exact real-journal SHA256:
  `25ab6edd4d33e55c6993abbaf63982b9711e2f634709d02a2b834460e258de1f`.
- All real query output files SHA256:
  `3652c745b2f0005c4dbaf230dbcd5b26ecb89d2fd2cecc85bff8afd7d20841bd`.
- New Release candidate SHA256:
  `03dfd95754fdaf8acc7a29a90cc15d9e9769499a91985af539ce254e4499b788`.
- Private build/results root:
  `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_eigenvalues_20261012`.

**S4b accepted as a bounded private numerical-equivalence/runtime improvement.**
It does not establish equality with the independent nonlinear LOCAL covariance
definition, statistical independence from CBS-conditioned MAIN linearization
points, live throughput or an end-to-end speedup. No further optimization, live
provider trial, full sequence or push is performed. Stop here for the next decision.

Parent `9c7c89001bf4cc9bfdab3c9d26214e4ac3766804`; branch
`engineering/shared-native-eigenvalues-only-20261012`. The local completion commit
contains this report, evidence and both plan updates. Resolve its exact SHA with
`git log -1 --format=%H -- experiments/shared_linearization_s4_eigenvalues_20261012/REPORT.md`.
