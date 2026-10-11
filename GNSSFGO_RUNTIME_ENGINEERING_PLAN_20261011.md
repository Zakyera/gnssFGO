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
| E0 | Save this plan and push a recoverable checkpoint | Exact-commit manifest; archive hashes; verified Zakyera branch SHAs; original runtime untouched | Completed 2026-10-11 |
| E1 | GPU-batched GLIM LOCAL covariance reconstruction | Same LOCAL factors and Values; same Gaussian blocks, full 6x6 covariance and means within predeclared arithmetic tolerance; focused tests; measured stage latency | Re-audit 18/18 tests and zero matrix differences; component acceleration repeats. Earlier live60 headroom target missed; no full-run acceptance; default-off |
| E2 | Receiver evaluation reuse at identical Values | Correct invalidation on any referenced Values/factor/revision change; identical outputs and decisions for arithmetic-preserving reuse; timing of normal and recovery paths | Re-audit: 50/50 Release and sanitizer, matched replay exact; isolated redundant whitening fixed. Same-executable/core mean ~5.2% shorter on frozen60; full/live capacity uncertified; default-off |
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

Requested: save the plan and push before optimization changes. Completed with
documentation/archival/version control only; no estimator implementation or run.

- Branch `checkpoint/grandtour-engineering-20261011` pushed without force to
  Zakyera's gnssFGO, cbsms, glim_ros1, Kimera-VIO, glim, Kimera-VIO-ROS, cbs and
  liorf repositories. All eight remote SHAs checked against local checkpoint refs.
- Initial preservation commits: gnssFGO `fcbd8d2a50e6684bea7edb226970939fdae3d6fb`,
  cbsms `f073910917d7ad34bdf61a7bc83582e732b784dd`,
  glim_ros1 `35b57e6215b810df6bf1c43d055f4e3a5915bf55`,
  Kimera-VIO `560ad0fd33c162ff2a0a64a1372f9b61e1d8d8f9`.
  This follow-up documentation commit records completion; the cbsms recovery
  manifest pins the final gnssFGO checkpoint revision. Unchanged package SHAs
  are recorded there too. No default/upstream branch was modified.
- 865 archived files matched their originals; 43 original dirty-file contents
  preserved; all 193 protected runtime/input hashes still match. Exact pins for
  25 repositories verified. 221 Python and 30 shell/environment syntax checks
  passed; changed/new files screened for common credentials and binary/oversized
  content, and outgoing history checked for blobs above 50 MB (none).
- HTTPS push authentication was unavailable for gnssFGO; the push succeeded
  using SSH to the SAME Zakyera fork. No credentials or remotes were changed.
- Recovery details: `experiments/checkpoints/20261011/README.md` and cbsms
  `docs/GITHUB_CHECKPOINT_20261011.md`. Data, event journals, binaries and installed
  runtimes remain local, not a complete GitHub backup of experiment data.
- Decision: preservation gate passed. E1 is next, but has NOT started. Future
  candidates must use separate branches/build directories, leaving this recovery
  branch and frozen inputs intact. Report after E1 before expanding scope.

### E1 — GPU-batched LOCAL covariance reconstruction

- Authorization: user "ok proceed" after E0 checkpoint handoff, 2026-10-11.
- Status: prototype implemented/tested, NOT ACCEPTED. Only E1; no E1b,
  receiver change, new fusion or long run. Do not enable its experimental switch.
- Candidate branch: `engineering/e1-local-gpu-batching-20261011` in cbs,
  glim_ros1 and gnssFGO. Checkpoint branches remain intact.
- Protocol and predeclared tolerances:
  `experiments/e1_local_gpu_batching_20261011/PROTOCOL.txt`.
- Hypothesis: batch the same LOCAL GPU linearizations before the unchanged
  marginal elimination; keep filtered graph/Values and sidecar unchanged.
- Report: `experiments/e1_local_gpu_batching_20261011/REPORT.txt`; exact logs,
  build commands, binary hashes and preservation check are archived beside it.
- cbs `7c3a1e66767938e105d46fd072bd853ad5ec50c2`; glim_ros1
  `d53c2a9821d332d19dd89890de08d5b4e97fd706`. gtsam_points unchanged.
  Default-off selector; original installed and frozen private binaries untouched.
- Private Release library build passed; focused tests 8/9 passed. GPU full-joint
  covariance max difference 2.37628e-5 exceeds the predeclared 1.10310e-8 bound,
  reproducibly 5/5 times. Extracted full 6x6 relative covariance passes its gate;
  that does NOT override the failed joint-covariance test. Tolerances unchanged.
- Caught and corrected a stale GPU error-cache hazard introduced by the new
  batching prototype. Native batched store_linearized also compacts inliers,
  unlike the old synchronous path; exact numerical cause needs further isolation.
- Focused CPU ASan/UBSan 5/5 passed with matching Eigen allocation ABI. GPU ASan
  attempt blocked by CUDA allocation failure, not marked passed. No full suite
  or live validation after the numerical rejection.
- Synthetic 64-GPU-factor median linearization+marginal/query time: 13.6125 ms
  old vs 0.462739 ms batch. NOT an ETH1 or whole-pipeline speedup. No recorded
  real active graph was reconstructed; complete GPU fixtures are not in text logs.
- Preservation: 190/193 protected entries unchanged, three expected GLIM source/
  CMake changes; no unexpected changes or missing inputs. All baseline binaries
  unchanged. No launch/profile enabled the candidate.
- Decision: keep disabled and retain the checkpoint. Next E1-only proposal:
  preserve ordinary GPU inlier/cache semantics in a scoped batching compatibility
  path, then rerun the same strict fixture before any bounded real capture.
  Do not begin E2/E1b or treat this as the runtime-delay problem solved.

### E1 continuation — synchronous-compatible GPU batching

- Authorization: user "Proceed" after the E1 prototype rejection, 2026-10-11.
- Status: synthetic kernel validation passed; NOT production/live accepted.
  E1 only. No E1b, E2, source/fusion run or tuning.
- Hypothesis: an opt-in call reusing the native GPU batch implementation but
  preserving ordinary VGICP inlier/error-cache semantics will pass the SAME
  strict fixture. No tolerance change or altered covariance formula.
- Protocol: `experiments/e1_sync_compatible_batching_20261011/PROTOCOL.txt`.
- Preserve the previous rejected candidate, checkpoint and installed binaries.
  gtsam_points receives a separate candidate branch/private build; default native
  batching behavior and global hook registration remain unchanged.
- Report: `experiments/e1_sync_compatible_batching_20261011/REPORT.txt`;
  commands, XML, comparison summaries and binary hashes archived beside it.
- gtsam_points `857819676921650510e249018cd82d45c08d7e8d`; glim_ros1
  `fad72d6e11219e42cbbe09658341fff4512769dc`; cbs remains
  `7c3a1e66767938e105d46fd072bd853ad5ec50c2`. Local candidate branches only.
- Original nine tests pass unchanged. Expanded 14 tests pass five consecutive
  times. Printed Hessian/full joint/full Schur-relative covariance differences
  are zero, including independent changing-Values histories. Same tolerances.
- Old-API native-hook probe under installed vs candidate library passes those
  gates. Small cross-process Hessian/error variation also occurs installed vs
  installed; covariance is exact. Not claiming bit-identical native probe logs.
- CPU ASan/UBSan 5/5; expanded UBSan 14/14 with both changed GPU host files
  instrumented. CUDA kernels/dependency libraries are not sanitizer-instrumented;
  GPU ASan remains unvalidated. Private wrapper builds/resolves in Noetic.
- Synthetic idle-build timing (linearize + Marginals/query), 64 GPU factors:
  median 13.6634 ms old vs 0.475765 ms batch; p95 14.4535 vs 0.697894 ms.
  Includes ordinary sync warnings, excludes sidecar optimization and live queues.
  No whole-pipeline speedup or completed 160-second run is claimed.
- Preservation: prior candidate/checkpoint binaries unchanged, 190/193 protected
  entries match checkpoint, same three already-known source changes, zero NEW
  protected changes. All source candidate worktrees clean after local commits.
- Decision: keep selector default-off. Next proposed E1 gate is one bounded
  real-graph capture/comparison at identical LOCAL factors/Values, then a separate
  authorized headless acceptance run. Neither was started here; E2 stays pending.

### E1 real-graph gate — bounded paired diagnostic

- Authorization: user "Proceed" after synthetic E1 handoff, 2026-10-11.
- Status: real-graph equivalence PASSED. One first-20-second ETH1 source-only CBS-ON capture,
  0.1x, no receiver. Six selected LOCAL publications compare ordinary/batched
  linearization and covariance on identical factors/Values; ordinary result
  remains authoritative. No production selector enabling, E1b/E2 or tuning.
- Protocol: `experiments/e1_real_graph_20261011/PROTOCOL.txt`.
- Candidate runtime/instrumentation isolated under a new run root. Existing
  profiles, source settings, checkpoint and all previous candidate binaries stay
  unchanged. Stop on comparison failure; no retry or tolerance relaxation.
- Report: `experiments/e1_real_graph_20261011/REPORT.txt`; archived protocol,
  private instrumentation/build commands, effective settings and validation.
- Six samples reached: publications 10/30/60/100/150/180; 60–540 real GPU factors,
  complete joint dimensions 180–765. Ordinary A / batched B / ordinary A2 have
  exactly zero saved Hessian, joint-covariance and Schur-relative differences.
  Sidecar Values and main endpoint means unchanged. Oldest active pose advances
  x0 to x131, covering retirement from the five-second window.
- Focused Release regression 14/14 passed again with the exact private libraries.
  Prior E1 sanitizer results remain prior evidence, not rerun on this live capture.
- Requested 20 s: IMU 19.850004 s; native GLIM 18.997596 s / 191 rows;
  native Kimera 19.599893 s / 84 rows. Existing coverage gates pass; Kimera has
  a 3.199979 s startup gap (post-startup maximum 0.200001 s), not hidden by span.
  Source CBS injections G2K=80 / K2G=81; 56 GLIM committed removal log rows.
- Largest graph linearization 126.867 ms ordinary vs 7.256 ms batched; complete
  joint construction/query still about 58 ms. Six individual measurements only,
  fixed A/B/A2 order, extra diagnostic work and synchronous warnings included.
  Not a whole-pipeline speedup or proof that the 160 s queue problem is solved.
- Post-check: 190/193 original protected entries unchanged; same three prior
  intentional E1 differences, zero new changes. Frozen binaries/profile bytes
  unchanged; isolated source processes exited. No receiver or GT scoring ran.
- Decision: keep production selector off. Next proposed E1 gate is a separately
  authorized bounded headless live acceptance test using the candidate normally,
  with existing queue/publication gates. Do not start E1b/E2 or a full run here.

### E1 live gate — bounded 60-second acceptance

- Authorization: user "Proceed" after real-graph equivalence handoff, 2026-10-11.
- Status: bounded operating/correctness gates PASSED; extra scan-headroom target
  NOT MET. One first-60-second ETH1 CBS-ON co-running capture at
  0.1x, unchanged receiver and source settings except private E1 batching enable.
- Protocol: `experiments/e1_live60_20261011/PROTOCOL.txt`.
- Reuse the tested ordinary candidate wrapper, not A/B/A2 instrumentation.
  Preserve queue/publication/coverage gates and validate one saved-journal repeat.
  No default enabling, full sequence, E1b/E2 or performance/covariance tuning.
- Report: `experiments/e1_live60_20261011/REPORT.txt`; exact private inputs,
  validators, source settings and small evidence archived beside it.
- 14/14 focused Release tests; 60 s capture completes with native GLIM 591,
  native Kimera 284 and fused 593 rows. Existing Kimera 3.20 s startup gap reported.
- Source native alignment/full 36-entry covariance/lifecycle checks pass;
  34142 source versions conserved, 33247 replacements, 870 active IDs,
  25 explicitly unsupported before-initialization versions, pending 0.
- 593/593 nominal publications stationary (max gradient 9.97652e-6).
  One saved-journal repeat has byte-identical online/final states/trajectories,
  exact clock/dispatch/slot accounting. No new sanitizer/full-suite claim.
- Output age p95 .110009 s; transport peak queue 36 (gate 1000), final 0.
  All 224 protected files unchanged from preflight; prior binaries/profile bytes
  preserved, no new source changes, owned processes exited.
- Historical same-stamp first 60 s comparison: LOCAL covariance preparation median
  162.828 -> 13.392 ms; entire scan median 513.142 -> 462.594 ms, but p95
  791.005 -> 853.998 ms, missing predeclared desirable 800 ms (20% headroom).
  Main/sidecar/refinement costs were higher. This is not an identical-workload
  randomized A/B run and does not identify their cause. No full-run speed claim.
- Decision: component speedup validated, limited 60 s live gates passed, global
  tail-delay problem unresolved. Keep default-off/opt-in. Stop for the next
  engineering decision; do not silently promote or launch a longer run/E2.

### E2 — exact receiver evaluation reuse

- Authorization: user "ok proceed" after the E2 proposal, 2026-10-11.
- Status: completed bounded experiment. Correctness PASSED; performance NOT
  CERTIFIED. No new source/live capture and no working-receiver replacement.
- Protocol: `experiments/e2_exact_evaluation_20261011/PROTOCOL.txt`.
- Preserve E1 and the checkpoint; isolated private receiver build only. No
  covariance/Qc/priors/event-order changes or relaxed convergence/rollback gates.
- Report: `experiments/e2_exact_evaluation_20261011/REPORT.txt`; 84 small evidence
  files archived byte-identically, with commands/tests/failure records and hashes.
- Branch `engineering/e2-exact-evaluation-20261011`; parent
  `e6e51d388e42736c421e747b19f467f5709323eb`. The completion commit contains this
  ledger/report; obtain its exact SHA with `git log -1 --format=%H --
  experiments/e2_exact_evaluation_20261011/REPORT.txt`. Local only, no push.
- Observing-only live60 probe: 14.5869% of measured linearization time repeated
  at exact same inputs, passing the predeclared 5% opportunity gate. This is
  NOT an end-to-end speedup; ideal removable probe work is about 5.58 of 67 s.
- Opt-in same-factor/bit-identical X/V/W memoization, immutable revision objects,
  clone-on-return, fresh clone/replacement entries and exception invalidation.
  No error/Jacobian/whitening/solver/publication formula or threshold change.
- 19/19 probe Release tests; 47/47 candidate Release and 47/47 ASan/UBSan tests.
  Exact Gaussian/Hessian/cost/gradient and full synthetic joint covariance;
  invalidation, dense interpolation, mutation, concurrency and rollback covered.
  Dependency libraries are not sanitizer-instrumented; not a full ROS suite.
  Build include-list and fixture-path setup failures retained and corrected
  without changing original tests/assertions or numerical tolerances.
- Retained 60 s and partial143.79 s journals match all 18 reference products
  exactly after excluding only timing fields, including online/final states,
  covariances exported by this runner, residuals, decisions and revision ledger.
  Real full-history posterior covariance is not exported; do not claim it was
  separately measured. Recovery decisions match on the longer journal.
- 34142/85210 source versions conserved, 33247/83059 replacements,
  870/2126 active IDs; 25 before-init versions explicitly unsupported in each.
  No stale/duplicate/pending versions; max gradients 9.97652e-6 / 9.80407e-6.
- Timing reference -> candidate: saved60 66.00 -> 105.76 s; partial143
  517.58 -> 387.96 s; reverse-order saved60 repeat 59.57 -> 56.53 s.
  Numerical outputs/hit counts remain exact across the repeat. The 1.87x change
  between candidate timings prevents a reliable headline speedup. Core placement/
  frequency/shared-machine effects were not controlled sufficiently to attribute
  the variation. Do not hide the unfavorable first sample or promote the 25% one.
- Reuse removes ~15% of short-journal and ~25% of long-journal derivative calls,
  but retains full-graph visits/checks/copies. Cache payload grows 4.07 -> 9.90 MiB;
  long-journal process RSS 500.81 -> 513.90 MiB. Short-repeat late p95 slightly
  worsens, 131.59 -> 132.95 ms, despite a lower median. No live queue test.
- 273/273 protected originals and all 11 frozen candidate build files unchanged.
  Candidate executable SHA256
  `d05fd1917195ec47c903534715ae8f6a831d5b01aee5eefcd2673f24cff3a1b2`.
- Decision: retain mathematically validated prototype, default-off; runtime
  acceptance and full-sequence capacity remain unproven. No retries/retuning or
  further candidate edits after benchmarking. E3a/E3b/E4/E1b are NOT started.

### E1/E2 correctness audit continuation

- Authorization: user "we need to make sure that e1 e2 have no bugs before we
  move to other solutions. proceed", 2026-10-11.
- Status: COMPLETED, bounded correctness validated. E1/E2 only; no E1b/E3/E4, source capture,
  live experiment, default promotion, covariance/model/timing-policy change.
- Isolated branch: `engineering/e1-e2-correctness-audit-20261011`, parent
  `267d2c5edf8537d75a137272a432a6b87efde672`.
- Protocol: `experiments/e1_e2_correctness_audit_20261011/PROTOCOL.txt`.
- First audit existing code and exception/cache integration paths, add missing
  focused tests, then fix only reproduced defects in isolated patches. Existing
  binaries/journals/test results stay immutable. Separate mathematical
  correctness from controlled performance evidence; never claim proof of no bugs.
- Report: `experiments/e1_e2_correctness_audit_20261011/REPORT.txt`; 50 archived
  small evidence files and full artifact hashes. No reproduced remaining
  numerical defect, not an assertion that all bugs are impossible.
- E1: 18/18 Release and GPU-host UBSan; 6/6 CPU ASan/UBSan. Release and UBSan
  each print 263 zero-difference Hessian/joint/relative comparisons. No E1
  implementation change; repeated 64-factor component median 14.023 -> .44781 ms.
- E2 original and fixed candidate: 50/50 Release; fixed candidate 50/50 ASan/
  UBSan. Fixed duplicate augmentedJacobian construction/whitening on misses only,
  retaining weighted finite check. New private executable SHA256
  `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.
- First pinned OFF vs old unpinned oracle failed exact equality. Paused before
  ON and isolated with unchanged receiver: pinned original matches pinned OFF
  exactly; unpinned original reproduces old oracle exactly. Cross-environment
  maximum position difference ~9e-9 m. Existing TBB cap is already 1; do not
  attribute this to multiworker reductions. P/E Eigen cache-size difference
  confirmed, precise floating-point operation not traced. No tolerance relaxed.
- Matched pinned original vs corrected cache OFF/ON: all 18 non-timing products
  exact, both repeats deterministic; 34142 versions conserved, 33247 replacements,
  870 IDs, 25 explicit pre-init unsupported, 0 pending/stale/duplicate. 593/593
  publications stationary, max gradient 9.976530603e-6. Full real posterior
  covariance is not exported; full synthetic covariance tested instead.
- OFF times 59.4404/59.3413 s; ON 56.4364/56.1856 s, mean reduction 5.1857%.
  Diagnostic original controls interrupted ABBA and are retained; two samples
  per mode, same executable/core, no frequency lock. Late p95 gain inconsistent.
  No new live/143-second replay or whole-pipeline throughput claim.
- All 557 protected originals unchanged; source worktrees unchanged, working
  binary/defaults preserved, owned processes exited. GPU ASan/device sanitizer
  and full repository suite not claimed. Isolated candidate remains default-off.
- Commit: the local completion commit containing this ledger/report (parent
  above; `git log -1 --format=%H -- experiments/e1_e2_correctness_audit_20261011/REPORT.txt`).
  No push. Stop at E1/E2; no other engineering task started.

### E1/E2 combined live60 integration gate

- Authorization: user "Ok proceed" after the bounded combined-test proposal.
- Status: PASSED bounded integration, 2026-10-11. One first-60-second ETH1 CBS-ON capture,
  0.1x, headless. Existing E1 private source libraries plus corrected E2 cache.
- Protocol: `experiments/e1_e2_live60_20261011/PROTOCOL.txt`.
- No mathematical patch, tuning, changed affinity, source retry, E1b/E3/E4,
  default promotion, GT scoring or 160-second run. Retain strict stationarity,
  lossless revisions, coverage and queue gates; report the 800 ms scan-p95 goal.
- Parent `65ef74c43c8f85f6ce1033cd50f8ad956128e484`; isolated branch
  `engineering/e1-e2-live60-20261011`. Previous artifacts remain unchanged.
- Report: `experiments/e1_e2_live60_20261011/REPORT.txt`. 18/18 focused E1
  Release tests, 603 protected hashes unchanged; no new estimator/math patch.
- Source covariance/alignment/lifecycle and all 34175 revision versions pass;
  33280 replacements, 870 active IDs, 25 explicit pre-init unsupported, 0 pending.
  593/593 stationary publications, maximum gradient 9.87662655e-6, no recovery.
- One saved-journal repeat: all online/final X/V/W and trajectories byte-identical,
  all 19 non-timing products exact (including E2 counters). No retry/tolerance edit.
- GLIM scan median/p95 359.524/627.810 ms; desirable p95 < 800 ms PASSED.
  Receiver age p95 .090013 s, queue peak 48 < 1000, final 0. Maximum scan
  1019.767 ms exceeds one paced interval; not a worst-case or long-run guarantee.
- Historical E1-only scan p95 853.998 ms. Different live nonlinear workloads/
  arrivals, not an isolated E2 speedup claim; prior controlled ~5.2% result stands.
  Source and receiver later-window costs still grow. No 160-second certification.
- Decision: combined bounded gate passed; longer capacity test is now a separate
  proposed next step, NOT launched. Opt-in defaults retained, owned processes
  exited. Local completion commit contains this report/ledger; no push.

### Template for subsequent tasks

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
