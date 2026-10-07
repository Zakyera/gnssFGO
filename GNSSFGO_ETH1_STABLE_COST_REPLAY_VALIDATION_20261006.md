# ETH1 stable objective comparison and frozen replay validation

The 96.98 s OFF WNOA failure was caused by the numerical comparison of two
large objective totals. Replaying the same journal with the old guard reproduced
the same failure and byte-identical causal states, trajectories, source dispatch
and support clock. A factorwise cost calculation shows that the rejected
candidate decreases the evaluated Gaussian objective. A fresh QR solve of the
same graph confirms local stationarity and descent.

The isolated publication guard now uses this stable comparison for apparent
cost increases. All four frozen 40 s OFF ON and WNOA WNOJ regressions pass with
byte-identical scientific outputs. The longer validation stopped at a different
boundary, 62.18 s, where the stable evaluated cost itself increases slightly.
The specific summation defect is resolved, but the complete 120 s numerical
acceptance problem is **not yet resolved**. No remaining long cases were run.

## Scope and frozen model

This is offline causal replay of the retained ETH1 journals, not a live source
rerun. Native GLIM and Kimera, classic CBS exchange, source covariance, revisions
and availability times are unchanged. There are no central raw IMU factors,
bias variables, GNSS factors, robust kernels, covariance scaling or gating.
IMU timestamps drive the independent 100 ms support clock; the initialization
clock offset remains 59 samples. This is the experimental source only receiver,
not the production Combined IMU baseline.

WNOA uses X pose, world V and body angular W; its frozen Qc diagonal is
`[.00375,.00375,.00375,.015,.015,.015]`. WNOJ additionally estimates local
tangent acceleration, with Qc `[2.5,2.5,2.5,10,10,10]`. Priors, stock GP
interpolation, full history iSAM2 QR and stable ID factor replacement are frozen.
No dynamics, residual, Jacobian or source weight was edited.

## Reproduced failure

At support 964, physical timestamp `1727775104983475000`, the graph contains
965 support states and 2394 factors. The relevant second refinement step is:

| Quantity | Value |
| --- | ---: |
| Ordinary total before | 342987.60748445161 |
| Ordinary total after | 342987.60748445254 |
| Difference of ordinary totals | +9.313225746154785e-10 |
| Sum of individual factor cost differences in extended precision | -1.544419086131831e-10 |
| Stable whitened residual cost difference | -1.545795208291840e-10 |
| Linear model predicted difference | -5.638127288531847e-10 |
| Normalized gradient before | 1.2922106403310365e-5 |
| Normalized gradient after | 9.501015268423775e-8 |
| Maximum pose translation step | 5.323271989128781e-8 m |
| Maximum pose rotation step | 1.611919588962188e-8 rad |
| Maximum velocity step | 1.172634199619355e-7 m/s |
| Maximum angular rate step | 3.192952371914574e-7 rad/s |

The apparent total increase is 16 double precision units. Even summing the
individual existing factor error differences resolves a decrease. Summation
therefore accounts for the sign reversal in this observed failure; the result
does not depend on changing residuals or solver weights.

A separate fresh multifrontal QR linearization and solve, never fed into the
receiver, produces stable cost change `-1.789606118366961e-10` and gradient
`9.904627717728845e-8`. Its largest position step is `5.323279733616118e-8 m`.
The earlier zero iteration LM result is not used as a convergence certificate.

## Stable acceptance calculation

For Gaussian whitened residual vectors r before and s after, evaluate

```text
delta E = sum_j 0.5 * (s_j - r_j) * (s_j + r_j)
```

Cast the residual entries to extended precision before arithmetic and use
deterministic compensated summation. This avoids subtracting two large squared
norm totals. The linear model check similarly evaluates
`sum_j (J delta)_j * (0.5 * (J delta)_j - b_j)` directly, where b is the
linearized factor right hand side.

Ordinary finite non increasing graph cost steps retain their existing path.
For an apparent increase, the new path requires all of:

1. Resolved negative residual cost change and negative predicted change.
2. Each decrease exceeds its arithmetic summation bound.
3. Candidate normalized gradient at most `1e-5`, strictly below its prior value.
4. Finite state step and unchanged factor pointers and slots.

The numerical bound is `gamma_(8m+2) * sum(abs(term))`, using long double
epsilon and m residual terms. It budgets basic arithmetic and accumulation of
the supplied floating point residuals. It is explicitly **not** a bound on
Pose3 logarithms, interpolation, whitening or numerical Jacobian evaluation.
The gradient is a local first order check, not proof of a global optimum.

Unlike the earlier eight epsilon rule, this path does not permit a positive
stable cost change. No increased epsilon multiplier was selected. The unchanged
20 iteration cap and transactional copy swap ensure a rejected step cannot
partially mutate the receiver. Unsupported robust or constrained noise models
fail closed in the stable comparison.

Implementation is opt in:
[StableGaussianCost.h](/home/yeranis/repos/gnssFGO/online_fgo/include/offline_process/StableGaussianCost.h),
with integration only in the new isolated `FullPublicationRefinement.h` builds.
The previous policy, production defaults, old executables and reports remain
unchanged. Copied batch checks now explicitly save the normalized gradient and
stationarity result; equal states or zero iterations alone never establish
validity.

## Validation

Seventeen focused Release tests and the same seventeen ASan UBSan tests pass.
They cover stable accumulation, cancellation by a large common cost, analytic
Gaussian comparison, required descent and stationarity, invalid inputs,
unsupported robust noise, stationary no mutation, nonlinear convergence,
factor identity, iteration budget exhaustion, true worsening rollback and
exception rollback. Six are unchanged legacy refinement tests. External frozen
GTSAM and gnssFGO shared libraries are not rebuilt with sanitizers; this is not
a claim of a fully sanitized application or complete repository test suite.

All four 40 s cases pass: OFF WNOA, ON WNOA, OFF WNOJ and ON WNOJ. Each has 395
publication checks, 394 complete state and ledger snapshot comparisons, and
five copied graph checkpoints. Online and final trajectories, states, model
covariance exports, source covariance exports and event accounting are
byte identical to the preceding validated experiment. The old support 344
exception now has stable cost change `-2.557601396572840e-10`, rather than
requiring permission to increase the objective.

Initial build attempts exposed a diagnostic API and include mismatch, which
were corrected before the final builds. A synthetic cancellation fixture was
corrected to use a genuinely unresolved common total; the final suite passes.
The first short replay launch had a wrong read only input mount and exited
before creating outputs; it was corrected to the actual frozen 40 s directory.
All failed attempt logs are retained.

## Longer replay stopped at a distinct numerical boundary

OFF WNOA was attempted first under the predeclared new rule. It stops at support
616, physical timestamp `1727775070183475000` or 62.18 s after journal start.
The preceding 615 publications pass; the last causal output is at 62.08 s.
All 40 s scientific prefixes remain byte identical. Seven copied graph checks
before failure pass; the eighth, at the failed boundary, does not.

| Quantity at support 616 | Value |
| --- | ---: |
| Retained objective | 284207.81531742122 |
| Candidate ordinary objective | 284207.81531742151 |
| Stable candidate cost change | +6.298693143344395e-11 |
| Arithmetic summation bound | 9.080676304947056e-18 |
| Linear predicted cost change | -2.559103176434691e-10 |
| Retained normalized gradient | 1.036842270133747e-5 |
| Candidate normalized gradient | 6.963151166150495e-8 |
| Aggregate candidate step norm in mixed units | 2.627464215559190e-7 |

The candidate passes the gradient check but fails the new resolved descent
condition. This differs from support 964: changing the accumulation method does
not reverse the sign here. It cannot be explained as the same sum of large
totals defect. The arithmetic bound does not cover nonlinear residual evaluation
or numerical differentiation, so it does not certify a true mathematical
increase either. The precise remaining mechanism is not established by these
numbers alone.

This step was accepted under the old eight epsilon allowance. An earlier stop
under the new stricter rule is therefore not evidence that the trajectory became
worse. Through support 615 it follows the same frozen calculation. The previous
guard treated this case as roundoff without the stable residual check; the new
guard refuses to make that assumption.

The copied LM and restart again return zero iterations with gradient
`1.036842270133747e-5`, so both explicitly fail the new stationarity certificate.
No copied solution was injected into the receiver. The rejected solver copy was
discarded. Retained newest speed is approximately 0.062195 m/s.

A diagnostic only reproduction of this boundary confirms the same scientific
prefix and rejection. A fresh QR solve, linearized directly at the retained
estimate, gives stable cost change `-2.768891445280207e-11` and normalized
gradient `6.252650407086217e-8`. Its predicted change is
`-2.559103186090323e-10`, essentially the same as the incremental candidate.
Thus a fresh solve can produce an acceptable local solution at this boundary;
this does not establish that the motion model is invalid. Both calculations
use the same graph and input estimate. The exact source of their tiny arithmetic
difference is not localized further here. The fresh solution remains diagnostic
and does not replace iSAM2 state or its internal cached linearization.
The largest difference between the two proposed local coordinate steps is only
`1.184607965025111e-13`; such a small step difference is enough to change the
sign of this near zero evaluated cost change. This evidence cautions against
treating strict tiny cost comparisons as an exact arithmetic certificate.

The replay exited through its explicit guard after 69.37 s wall time, with
314156 KiB peak child RSS. It was not an OOM, timeout or source failure. ON WNOA
and both long WNOJ cases, repeats and GT scoring were withheld. No epsilon,
gradient threshold, weight or model parameter was changed after this result.

## Reproduction and provenance

New artifacts:
[/home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/](/home/yeranis/results/gnssfgo_eth1_stable_cost_20261006).
The directory contains frozen asset hashes, build and run commands, mechanical
diffs, failure factor costs, state increments, independent QR evidence, tests,
per case validity checks and the predeclared validation protocol.

Execution uses existing image
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`
with network disabled, container filesystem read only, and frozen source inputs
and runtime mounted read only. Only the new artifact directory is writable.

```bash
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh build2 wnoa
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh diagnostic wnoa
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh stable_build2 wnoa
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh stable_build wnoj
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh unit2 wnoa
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh sanitizer wnoa
# The short action was run for each model wnoa/wnoj and mode off/on.
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh short wnoa off
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh primary wnoa off
# The longer primary exits 1 at its declared validity guard.
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/validate.py off_wnoa
# Its validity checker correctly exits 2.
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh build616 wnoa
bash /home/yeranis/results/gnssfgo_eth1_stable_cost_20261006/run.sh diagnostic616 wnoa
```

The diagnostic executable was compiled before the stable integration; its
mechanical diff is preserved separately. Runners require fresh output names.

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was made.
The pre existing dirty worktree is preserved; it is not reported as clean.
The preservation check verifies 35,644 protected files with zero mismatches.
The two primary new binary hashes are:

```text
WNOA 34c68d65595e0189a09d292770f981176abca1af5d379b918e8c531e42656eab
WNOJ ec4d3d3a7a91c8885259e9c48e4d9db1cdc447d5eee7ce6b0a28a4d3bdc26f53
```

## Conclusion

The 96.98 s sum of large totals defect is identified and addressed without a
larger epsilon allowance. The new check passes the existing four short cases
without changing their scientific outputs. The stronger check also exposes a
different near stationarity issue at 62.18 s that the previous allowance hid.
Its fresh QR diagnostic succeeds, but incorporating such a fallback safely
would require a separate validated numerical integration with iSAM2 bookkeeping.
That fallback is not implemented or promoted here. The complete 120 s baseline
therefore remains unvalidated. All replay processes have stopped; no tuning,
source rerun or follow up accuracy experiment was performed.
