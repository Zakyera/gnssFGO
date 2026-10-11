# Incremental CBS and WNOA runtime engineering plan

Agreed with the user on 2026-10-11. Implement one change at a time, validate it,
report the result, and obtain the next decision. This document is the execution
ledger, not permission to run every proposed experiment automatically.

## Read and update this plan for every task

Before work, identify the task ID below and read its prerequisites, the canonical
CBS runbook, and the exact saved experiment protocol. Mark that task in progress.
After work, record the patch/build identity, inputs, checks, performance, limits,
and decision in the ledger. Link the detailed report and commits. A failed or
incomplete test stays failed/incomplete; do not silently promote its candidate.
Tell the user which task was completed and what remains. Never combine multiple
performance hypotheses into one acceptance experiment.

## Baseline and evidence

The detailed read-only audit is archived at
`experiments/checkpoints/20261011/results/gnssfgo_runtime_engineering_audit_20261011/REPORT.txt`.
The archive README and the cbsms checkpoint manifest identify the exact source,
profiles, dependencies, build instructions and recovery boundaries.

Current reference executable, retained locally and never overwritten:

```
/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build/replay
SHA256 62124b496ad6f854079218862acb149892aa452d4af908c50bc96067e49c9dbb
```

This private experimental build is NOT the default gnssFGO CMake target. Its
include chain also uses the preceding publication-cache build. Both are archived.
Keep existing binaries, frozen input journals, and result directories unchanged;
build candidates in new isolated locations/branches.

The latest requested 160 s ETH1 CBS-ON capture stopped at 143.790032 s after
crossing the queue gate. It is not a completed 160 s validation. All 1433 recorded
publication checks passed the existing stationarity threshold; this does not
establish accuracy or sustainable long-duration throughput. The saved replay and
Release/sanitizer checks remain historical evidence, not newly run tests.

Evidence motivating the order (not promised speedups):

- GLIM LOCAL sidecar update plus covariance reconstruction occupied about 59.45%
  of measured scan time. The auxiliary path misses existing GPU batching.
- Every recorded receiver refinement step relinearized all current variables.
- Normal publication refinement, not just recovery, dominates receiver cost.
- Mixed source logs contain millions of synchronous GPU warnings; transport and
  logging are separate supporting costs, not the entire explanation.

## Frozen semantics

- Classic source-to-source CBS with the current `active_window_temporary` policy;
  remove external factors before their marginalization. No source `live_revision`.
- Preserve the separate LOCAL covariance conditioning, current source mean
  alignment, all full dense source covariances and their extraction method.
- Downstream stable-ID replacement, every admitted version, availability times,
  physical endpoints and independent 100 ms support clock remain unchanged.
- Receiver `source_wnoa`: pose X, world velocity V, body angular rate W;
  full-history iSAM2 QR, no central IMU measurement/bias/acceleration/GNSS factors.
  CPT7 supplies clock timestamps, not a central IMU measurement factor.
- Qc `[0.00375,0.00375,0.00375,0.015,0.015,0.015]`, current initialization and priors,
  covariance scale 1, Gaussian loss, no new gating/health weighting/robust loss.
- Preserve sequence-specific sensor profiles and existing native safeguards.
  The current classic Kimera source leaves its experimental IMU-support override
  unset; do not replace this with an assumption from the old ALL_RIGHT pilot.
- Preserve nominal publication checks, including the 1e-5 normalized-gradient
  threshold and transactional rollback/failure behavior.

Removing an external factor before marginalization does not remove its influence
on the active main graph while it is present. Hence outgoing covariance cannot
blindly come from that main graph. The current normal GLIM mean uses the corrected
main smoother, while its covariance uses the LOCAL sidecar. Keep that distinction.
A second optimizer is not mathematically mandatory in every possible design, but
removing it is NOT the first optimization. This policy also does not prove that
shared IMU, past means or nonlinear history are statistically independent.

## Ordered tasks and acceptance gates

| ID | Isolated task | Required evidence before acceptance | Status |
| --- | --- | --- | --- |
| E0 | Save this plan and push a recoverable checkpoint | Exact-commit manifest; archive hashes; verified Zakyera branch SHAs; original runtime untouched | In progress |
| E1 | GPU-batched GLIM LOCAL covariance reconstruction | Same LOCAL factors and Values; same Gaussian blocks, full 6x6 covariance and means within predeclared arithmetic tolerance; focused tests; measured stage latency | Not started |
| E2 | Receiver evaluation reuse at identical Values | Correct invalidation on any referenced Values/factor/revision change; identical outputs and decisions for arithmetic-preserving reuse; timing of normal and recovery paths | Not started |
| E3a | Remaining WNOA motion-factor derivatives | Independent numerical derivative checks; manifold/frame tests; frozen-journal state/cost/covariance checks with predeclared tight tolerances | Not started |
| E3b | Remaining interpolated pose derivative blocks | Separate patch and independent checks as for E3a; do not redo already validated V/W substitution | Not started |
| E4 | Selective incremental publication refinement | Same factors/events and fresh current-Values stationarity certificate; reference full refinement fallback; short/medium/late replay comparisons | Not started; higher risk |
| E5 | Transport/logging/bookkeeping efficiency | One subchange per test; lossless ordered revision delivery and auditability; same factor accounting; no silent dropped/coalesced versions | Not started |

E1 first reuses existing gtsam_points batching around LOCAL graph preparation;
it retains the auxiliary smoother and its conditioning. GPU-aware sidecar update,
if still necessary, is a separate E1b proposal after the E1 result.

E2 must not reuse stale linearizations under a merely small state change. Exact
same-Values reuse may have limited benefit when many states move; measure that
honestly. E4 is not a one-flag return to nominal iSAM2: previous incremental
publication attempts failed. Full history still has growing storage/graph costs.

## Common validation workflow

1. Record the task hypothesis, candidate diff, oracle/input hashes and numeric
   tolerances BEFORE examining results. No ATE-driven parameter selection.
2. Run focused unit/reference tests, relevant Release tests and ASan/UBSan where
   applicable. Test replacement, invalidation and transactional failure paths.
3. Use identical retained inputs for correctness first. Compare X/V/W, full
   relevant covariance, residuals/cost, stationarity and publication decisions;
   check exact IDs, revisions, slots, timestamps and event conservation.
4. Benchmark the same short, medium and late sections. Record median/p95/max
   latency, global graph visits/Jacobian calls, queue age, memory and relevant
   stage timings. Do not add overlapping timers as independent costs.
5. Only after frozen-input checks pass, request/perform the authorized bounded
   live test. Preserve 0.1x and headless settings for comparison; predeclare a
   latency budget with margin. Do not solve failure by increasing the queue cap.
6. Report accept/reject/blocked and update the ledger before proceeding.

GLIM text/covariance logs are not serialized GPU graph fixtures. If identical
factor/Values fixtures are missing, report that limitation and scope one bounded
instrumented capture before launching it. Faster live execution can legitimately
change arrival/update interleaving, so live trajectories need not be byte-identical;
this does not excuse differences in a same-input mathematical kernel test.

## Deferred work, not authorized by this plan

No covariance tuning, CBS redesign, changed priors/Qc, fixed-lag receiver,
revision coalescing, source cadence changes, extra sequence, or longer stress run.
Batching distinct revisions into a different nonlinear solve order also needs
its own explicit algorithmic decision. Fixed lag may bound resources but brings
historical revision and marginal-prior semantics; reconsider only separately.

## Execution ledger

### E0 — preservation checkpoint

Requested: save the plan and push before optimization changes. Current work is
documentation/archival/version control only; no estimator implementation or run.
Checkpoint evidence is recorded in `experiments/checkpoints/20261011/README.md`
and cbsms `docs/GITHUB_CHECKPOINT_20261011.md`. Complete E0 only after remote
verification; record the verified result here in a follow-up documentation commit.

### Template for the next task

Copy this block under its task ID; do not erase earlier attempts.

- Date and authorization:
- Status: proposed / in progress / passed / rejected / blocked
- Single hypothesis and exact scope:
- Baseline and candidate commits/build hashes:
- Input/protocol hashes and predeclared tolerances:
- Tests and correctness evidence (links):
- Performance before/after, tails and measurement limits:
- Preserved semantics and any deviations:
- Decision, rollback/recovery point, next authorized step:
