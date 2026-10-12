# S4c: conservative rank certificate passes the saved-input gate

The shared-native covariance candidate is **3.33× faster by median than S4b**:
**37.285 → 11.196 ms per complete query**, with byte-identical saved outputs.
All 20 unique recorded query checkpoints use the sufficient certificate; none
needs the unchanged eigenvalue fallback. All 40 Release and 40 focused
ASan/UBSan tests pass, including the previous boundary/rejection tests.

**Accept this private optimization for the next integration decision.** No
source run, covariance publication, production provider switch or push occurred.
The authoritative implementation remains the already-batched independent LOCAL
smoother. This is a covariance-query speedup, not measured whole-GLIM speedup.

## Scope and one numerical change

The [predeclared protocol](PROTOCOL.md) permits one check-only sparse
Cholesky/reconstruction certificate, with S4b's original eigensolver on every
uncertain case. Constants were frozen before validation/timing and not tuned.

Only the active information-matrix rank check is wrapped. The finite/symmetry/
empty guards, original eigensolver, threshold, 12D endpoint check, information
assembly, Gaussian elimination, joint covariance, tangent transport and Schur
relative-covariance calculation remain unchanged. Stripping the wrapper and its
two includes reproduces S4b's candidate source exactly; the analyzer verifies it.

The candidate does **not** merely trust successful Cholesky, and it does **not**
reuse MAIN's mixed marginal covariance. It factors a separate, temporary shifted
matrix for checking, then bounds reconstruction error. The shift never enters
the covariance calculation or any estimator factor. This implementation uses
the pinned local Eigen `SimplicialLLT` with AMD ordering; its public API exposes
the factor and permutation needed for the check. See the
[Eigen API reference](https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1SimplicialLLT.html).
The linked documentation is not a dependency upgrade; all builds use the
existing workspace headers/libraries.

Files: [certificate](RankCertificate.h), [exact wrapper patch](candidate.patch),
[generated candidate](evidence/candidate.cpp), [focused tests](certificate_tests.cpp).

## Why the certificate is sufficient

The existing spectral condition is

    lambda_min(H) > 1e-12 * max(1, lambda_max(H)).

Use the same lower-triangle self-adjoint interpretation as the original solver.
Normalize by an exactly reversible power of two, `A = H/c`. Construct an
outward-rounded row/column norm upper bound `U >= lambda_max(A)` and

    t >= 1e-12 * max(1/c, U).

For a temporary shift `s`, compute a sparse factor L of the rounded `A-sI`.
Independently inspect the reconstruction against the original A:

    P A P^T = L L^T + s I + E.

Since `L L^T` is positive semidefinite, even when L is imperfect,

    lambda_min(A) >= s - ||E||_2.

Bound `||E||_2` using computed residual row/column norms plus product/subtraction
roundoff allowances. The product allowance uses upper bounds on
`||L||_inf * ||L||_1`; each dot product has at most n terms. Thus factorization
error is included in the measured reconstruction, not assumed negligible.
Scalar bound operations use outward `nextafter` rounding. The standard
roundoff model and `gamma_n = n*u/(1-n*u)` construction are described in
Higham, *The Mathematics of Floating-Point Arithmetic*, pp.37–38,
[LMS Newsletter 493 (2021)](https://nhigham.com/wp-content/uploads/2021/04/high21m.pdf).
The particular certificate and safety constants here are our derivation, not
an Eigen/Higham certification of this implementation.

We conservatively use machine epsilon (twice unit roundoff),
`g = gamma_(2n+8)*U`, `s = t+32*g`, and an extra `8*g` acceptance guard.
The fast path accepts only if its downward-rounded lower bound exceeds
`t+8*g`. All other cases use the unchanged eigensolver. Dimensions outside
1..4096, unsupported rounding mode, FTZ/DAZ, fast-math, irreversible scaling,
nonfinite bounds and factorization failure cannot grant fast acceptance.
Absolute underflow allowances use the minimum normal double, conservatively
larger than a subnormal rounding error. The code changes no rounding mode.

This is a sufficient test for the represented matrix's exact spectral condition.
The original floating-point eigensolver is also approximate. Universal bitwise
decision equality for every conceivable matrix at the numerical threshold is
not claimed. The guard, unchanged fallback and adversarial decision comparisons
address this boundary without relaxing the threshold. No interval-arithmetic
library or formally verified implementation is claimed.

## Correctness results before timing

- **40/40 Release, 40/40 focused ASan/UBSan.** All 25 previous history tests and
  six S4b rank groups are preserved unchanged. Those groups compare all 130
  matrices against verbatim old helpers, including failure reasons, strict
  threshold equality/adjacent representable values, rotated near-threshold
  systems and large 765-dimensional systems.
- Nine new groups exercise sparse SPD certification, dense rotations of sparse
  systems, near-threshold fallback, singular/indefinite/asymmetric inputs,
  unsupported scaling/rounding, deliberately wrong reconstruction, input
  immutability and repeatability, 36 seeded sparse systems, and an analytic
  residual/norm bound. Every tested fast acceptance passes the old criterion;
  its lower bound is checked against a separate dense eigendecomposition.
- Pose-only and mixed pose/velocity/bias saved fixtures have byte-identical full
  query output between S4b and S4c in Release and sanitizer runs. Corrupted and
  truncated input files fail as required. No numerical failure or retry occurred.
- All 12 real-journal passes (initial validation pair, warm-up pair, three
  measured pairs and separate profile pair) have the same output SHA256:
  `3652c745b2f0005c4dbaf230dbcd5b26ecb89d2fd2cecc85bff8afd7d20841bd`.
  This covers saved information, joint/current/transported endpoint and relative
  covariance outputs, not a new trajectory experiment.
- Each real pass processes **783 updates, 313 retirements and 20 queries**.
  Conservation: **5,321 admitted = 4,513 absorbed + 808 active**, zero explicit
  removals. Maximum relative oracle errors remain **1.6943e-17 information** and
  **3.7292e-15 covariance/transport**, below the unchanged 1e-9/1e-8 gates.

The new sanitizer binaries were non-PIE from the start, retaining the existing
documented environment workaround. Dependencies/GTSAM are prebuilt, not themselves
sanitizer-instrumented. Sanitizers cover focused tests and small fixtures; the
complete real recording is validated in Release. Reusing the saved S3 oracle is
not a new independent verification of how the original equations were generated.

## Matched runtime

One warm-up pair, then three alternating-order measured pairs against the saved
**S4b eigenvalues-only binary**, not the much slower pre-S4b implementation.
Same Release options and libraries; single-thread environment variables; Intel
Core Ultra 9 275HX, powersave governor, unlocked desktop clocks. Compilation and
sanitizer tests had finished; no estimator ran. The preflight checks container
processes, not every host task. A host integrity hash check overlapped warm-up
and part of the first measured pair, ending at 20:04:16 UTC. Later pairs are
unaffected by that check and show consistent improvement. This is an engineering
benchmark, not an isolated real-time scheduling measurement. No pairs were
discarded or rerun.

Milliseconds, pooled across 60 queries or 2,349 history calls:

| Operation | S4b median | S4c median | S4b p95 | S4c p95 | S4b max | S4c max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Complete covariance query | 37.285 | 11.196 | 40.467 | 22.551 | 42.035 | 35.624 |
| History stage | 0.666 | 0.663 | 0.852 | 0.853 | 3.030 | 1.364 |
| History commit | 0.034 | 0.034 | 0.057 | 0.055 | 0.323 | 0.397 |

Median query reduction: **69.972%**. Mean: **33.365 → 12.708 ms** (61.912%
reduction). Query-total speedups per pair: **2.548×, 2.653×, 2.681×**.
Candidate query totals: 0.266/0.248/0.249 s for 20 queries, versus
0.677/0.659/0.667 s. History stage/commit totals average **0.510 s** per complete
recording; this path was not optimized. All three predeclared speed gates pass:
at least 25% median reduction, every pair faster, no query p95 regression.

| Recorded epoch bin | S4b median query ms | S4c median query ms |
| --- | ---: | ---: |
| 1–195 | 21.947 | 7.054 |
| 196–587 | 37.548 | 13.716 |
| 588–783 | 37.042 | 10.385 |

These are update-epoch bins, not exact equal-duration sections. Only 20 unique
queries were captured, not every outgoing request. Matrices span 45–765 scalar
dimensions. All 60 measured queries certify; **fallback fraction 0%**. Original
fallback remains tested on adversarial inputs. No future fallback-rate guarantee
is implied by this recording.

The measured certificate itself has median **4.806 ms**, p95 **14.154 ms**, max
**26.275 ms**. A separate profile pair gives dense S4b rank-check median
31.784 ms versus S4c certificate median 4.589 ms. The remaining candidate query
time still includes symmetry validation, dense assembly, Gaussian elimination,
joint marginal and covariance transport. It is not zero-cost cache access.
At epoch 325, the profile certificate costs 24.881 ms with 99,828 nonzeros in L;
its complete query is 33.732 ms. Sparse fill therefore remains a practical cost,
even though every checkpoint passes. No second optimization is attempted here.

## What this means for overall GLIM

At S3's observed 389-request cadence, scaling the sampled query mean projects
**4.943 s** of S4c query work versus **12.979 s** for S4b, plus about 0.510 s
for history. This projection is not a measured all-request or live runtime.

B0's separate already-batched independent LOCAL update-plus-covariance scopes
totaled 7.579 s. The projected candidate cost is now promising enough to justify
a matched live integration test, but these are different graphs, reference
states, covariance definitions and capture costs. Subtracting them is not a
measured GLIM speedup. MAIN nonlinear/GPU work is also unchanged and remains a
major cost. No live throughput, total-scan latency or longer-run claim follows.

The shared-native definition still excludes explicit CBS equations and maintains
its own native prior, but evaluates native information in MAIN's recorded charts.
MAIN can be influenced by CBS; this is not statistical independence or guaranteed
equality to the independent LOCAL covariance. S4c preserves S4b on identical
equations; it does not erase that distinction.

## Preservation and decision

Integrity passed for 716 S3 protected files, 680 baseline files, 19 S1 pins,
7 S4a identities, 8 S4b identities, 13 staged input pins, the original CPU
library and all six source repository HEAD/clean-status checks. No production
file, installed library, source profile or frozen result changed.

- [Machine-readable result](summary.json), [hashes](hashes.json),
  [raw timing commands](evidence/timing_results/commands.json),
  [Release tests](evidence/release_results/unit.xml),
  [sanitizer tests](evidence/asan_results/unit.xml).
- Real input SHA256:
  `25ab6edd4d33e55c6993abbaf63982b9711e2f634709d02a2b834460e258de1f`.
- New Release candidate SHA256:
  `eff6bbc2bdefcebd696ecd8690abe25fa1e7009b7dff00912dcfccb88a910e47`.
- Private artifacts:
  `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_certificate_20261012`.

**S4c accepted as a bounded private correctness/runtime improvement.** Stop
here. The next separately authorized stage would integrate the candidate
covariance provider and compare against preserved independent LOCAL over a
matched 40-second run, with complete-scan timing and publication/history checks.
No provider promotion, experiment launch, covariance tuning or push is included.

Parent `feb737397453e10e8f13423993df020e41bbd3e6`; branch
`engineering/shared-native-rank-certificate-20261012`. The local completion commit
contains this report and both plan updates. Resolve the exact SHA using
`git log -1 --format=%H -- experiments/shared_linearization_s4_certificate_20261012/REPORT.md`.
