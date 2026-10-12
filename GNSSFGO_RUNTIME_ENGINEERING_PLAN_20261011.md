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

**Planning boundary (2026-10-12):** Task S0 preserves the completed E1b.4a
checkpoint and plans the supervisor's native-equation reuse proposal in
`GLIM_SHARED_LINEARIZATION_LOCAL_COVARIANCE_PLAN_20261012.md`.
Planning/checkpoint work and S1 read-only feasibility are complete; all 677
protected hashes and six source HEAD/status checks passed. S1 identifies the
actual MAIN `gtsam_points::ISAM2Ext` cache and required lifecycle hooks; see
`GLIM_SHARED_LINEARIZATION_FEASIBILITY_S1_20261012.md`.
No candidate implementation or new run is authorized
by this entry. Unlike E1/E1b, this proposal would change the
LOCAL linearization policy; it must not be called arithmetic-equivalent to the
independent LOCAL smoother. All frozen semantics below still govern the current
working implementation. Complete one proposed stage and report before the next.

**Current status (2026-10-12):** E1b.3c completed and validated the first40s
profile after the private startup and admission fixes. LOCAL synchronous GPU
linearization accounts for 96.405% of its measured exclusive wall time;
143,950/143,950 calls took fallback. E1b.4a now has an isolated default-off
implementation with its synthetic correctness gate passed (32 Release, 30
UBSan, 10 CPU ASan/UBSan checks and no-data startup). The subsequently authorized
E1b.4a synthetic timing gate PASSED: GPU-heavy median112.310->4.30792ms and
p95115.43625->4.66825ms, with unchanged tested states/covariances/decisions.
This is a small-cloud synthetic LOCAL-update result, NOT whole-GLIM or live
capacity acceptance. The default remains off pending real-source gates.
The subsequent real-input entry audit found that the required independent
GPU-state oracle was not yet available; playback was held before launch, not
reported as a failed batching experiment. The subsequent isolated snapshot
checker now passes 40 Release/40 host UBSan/14 CPU ASan-UBSan tests, including
authoritative-update fidelity and persistent copied histories. The subsequent
first20s real-input diagnostic also PASSED: six independent checkpoint checks,
zero p/v/bias/Gaussian/full-relative covariance differences, maximum orientation
difference6.83e-18rad, including fixed-lag retirement. Ordinary execution remained
authoritative. The subsequent observer-free first40s timing also PASSED:
LOCAL median117.148->7.600ms,p95218.181->14.391ms; complete-scan
median307.376->182.365ms,p95535.236->385.682ms. All functional gates and
byte-identical saved-journal receiver replay passed. MAIN still made162254
synchronous GPU calls. No default promotion or full160 acceptance occurred.
See the E1b.4a real-input diagnostic ledger below.
The earlier E1/E2 live160 remains functionally
validated but misses runtime headroom; E4 was rejected and remains disabled.
MAIN and receiver full-history costs also remain. Working source/receiver
mathematics and protected libraries remain frozen. The original checkpoint
evidence below is historical and is not the latest run's status.

The detailed read-only audit is archived at
`experiments/checkpoints/20261011/results/gnssfgo_runtime_engineering_audit_20261011/REPORT.txt`.
The archive README and the cbsms checkpoint manifest identify the exact source,
profiles, dependencies, build instructions and recovery boundaries.

Original checkpoint reference executable, retained locally and never overwritten:

```
/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build/replay
SHA256 62124b496ad6f854079218862acb149892aa452d4af908c50bc96067e49c9dbb
```

This private experimental build is NOT the default gnssFGO CMake target. Its
include chain also uses the preceding publication-cache build. Both are archived.
Keep existing binaries, frozen input journals, and result directories unchanged;
build candidates in new isolated locations/branches.

At that original checkpoint, the requested 160 s ETH1 CBS-ON capture stopped at
143.790032 s after crossing the queue gate. It is not a completed 160 s validation. All 1433 recorded
publication checks passed the existing stationarity threshold; this does not
establish accuracy or sustainable long-duration throughput. The saved replay and
Release/sanitizer checks remain historical evidence, not newly run tests.

The later completed E1/E2 live160 uses
`/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay`,
SHA256 `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.
Its authoritative status is in
`experiments/e1_e2_live160_validation_20261011/REPORT.txt`: functional pass,
late GLIM p95 headroom miss. Neither old failure nor E4 rejection replaces it.

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
| E1b.1 | Diagnose remaining GLIM LOCAL smoother update cost | Trace actual call chain and GPU evaluation semantics; inspect retained early/late timing; distinguish measured cost from missing instrumentation; specify one next patch | Completed 2026-10-12; report/evidence saved, no estimator change. LOCAL subphase costs not yet separated |
| E1b.2 | Instrument LOCAL update subphases | Opt-in timing only; no extra factor/solver evaluations; scope/cache/exception tests; disjoint accounting; measured overhead before bounded profiling | Private implementation/tests passed 2026-10-12: 29 Release +29 UBSan +8 profiler ASan/UBSan; original-library fixture exact; no source capture or solver replacement |
| E1b.3 | Bounded LOCAL subphase profile | Frozen source settings; explicit profiler overhead/coverage; separate nested/worker/outer wall costs; select one measured dominant subphase | E1b.3c validated first40s: 96.405% of LOCAL exclusive wall in synchronous GPU linearization; first40s limits retained |
| E1b.4 | Batch LOCAL update linearizations at exact solver Values | Keep solver decisions and ordered Gaussian blocks; reuse E1 synchronous-compatible kernels; same-input repeated-solver and covariance checks before bounded live timing | Synthetic, sampled real-input and observer-free first40s timing gates PASSED. Default-off; longer capacity and full evolving-history equivalence remain uncertified |
| E3a | Remaining WNOA motion-factor derivatives | Independent numerical derivative checks; manifold/frame tests; frozen-journal state/cost/covariance checks with predeclared tight tolerances | Not started |
| E3b | Remaining interpolated pose derivative blocks | Separate patch and independent checks as for E3a; do not redo already validated V/W substitution | Not started |
| E4 | Selective incremental publication refinement | Same factors/events and fresh current-Values stationarity certificate; reference full refinement fallback; short/medium/late replay comparisons | Tested 2026-10-12; NOT ACCEPTED: strict state bounds failed,86.8% fallback,matched refinement~40% slower; stopped at130.68s |
| E5 | Transport/logging/bookkeeping efficiency | One subchange per test; lossless ordered revision delivery and auditability; same factor accounting; no silent dropped/coalesced versions | Not started |
| S0 | Preserve E1b.4a and plan native Gaussian-equation reuse | Recoverable local checkpoint; saved staged plan; distinguish native information from MAIN conditioning and mixed marginal priors | Completed 2026-10-12; all 677 hashes and six source HEAD/status checks passed; no implementation/run/push |
| S1 | Read-only feasibility of MAIN native-equation reuse | Exact capture/retirement hooks, factor provenance, coherent reference charts and minimal isolated patch surface | Complete: feasible with lifecycle hooks; 677 hashes and six source repositories unchanged; no numerical certification, implementation or run |
| B0 | Fresh unchanged first40s overall timing reference | Preserved E1b.4a build/settings; original functional gates and exact saved-journal repeat; report MAIN, LOCAL, covariance and complete-scan timing | Passed: LOCAL median/p95 10.965/15.385 ms; complete-scan 206.942/437.376 ms; functional gates and exact receiver repeat pass. MAIN remains dominant |
| S2 | Private default-off native Gaussian history prototype | Synthetic same-chart dense oracle, native-only retirement, tangent transport, lifecycle/rollback and focused sanitizers | Passed: 26/26 Release; 25/25 UBSan and ASan+UBSan. Twelve retirements, dense oracle, zero extra native evaluations in actual MAIN cache fixture. No live integration, publication or run |
| S3 | Private default-off MAIN lifecycle and native-admission observer | Actual fallback-wrapper update/retirement/reset fixtures; no added native evaluations; unchanged MAIN; same-chart shadow oracle before promotion | Real-data shadow gate passed: one ETH1 first40s capture;783 updates,313 retirements,20 covariance checks, max joint relative difference6.01e-15.44/44 Release twice and ASan/UBSan;18 regressions;receiver repeat exact. LOCAL authoritative. See experiments/shared_linearization_s3_capture_20261012/REPORT.md |
| S4a | Saved native equation cost attribution only | Original/instrumented covariance equivalence, isolated stage/query timings and query-cadence audit | Complete: dense rank check95.94% of query cost; original query median160.274ms. Not a faster provider; proposed EigenvaluesOnly patch not implemented. See experiments/shared_linearization_s4_timing_20261012/REPORT.md |

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

### S4a — isolated saved equation cost attribution

- Authorization: user "proceed" after the S3 timing clarification, 2026-10-12.
- Status: COST ATTRIBUTED; NO PROMOTION. Parent36ded67; branch engineering/shared-native-gaussian-s4-timing-20261012.
- Replay S3's recorded native equations only. Time the unchanged Gaussian
  history and covariance query with original/instrumented numerical comparison.
  No source run, speed patch, covariance-provider promotion or tuning.
- Protocol: experiments/shared_linearization_s4_timing_20261012/PROTOCOL.md.
- Three interleaved measured passes plus warm-ups of all783 saved updates and
  20 exact queries: unchanged and timer-instrumented covariances byte-identical.
  Original query median160.274ms; dense rank check153.513ms,95.94% of query time.
  Stage/commit total0.535s per783 updates; retirement elimination median0.038ms.
  The dense check requests unused eigenvectors. Next proposed single patch:
  EigenvaluesOnly with identical rank policy, correctness and timing gates.
- Release25/25 and focused non-PIE ASan/UBSan25/25 plus saved mixed fixtures pass.
  Initial PIE sanitizer mixed-fixture startup DEADLYSIGNAL retained; identical
  objects relinked non-PIE pass, root cause not claimed. No production changes.
- Observed389 LOCAL queries vs20 sampled. MAIN's17331 revision rows are separate.
  Current candidate not a speed replacement. Report/evidence:
  experiments/shared_linearization_s4_timing_20261012/REPORT.md.

### S3 continuation — recorded Gaussian oracle and first40s shadow gate

- User authorized proceeding from the completed S3 synthetic gate. Parent
  `1cf5a22750316695d28c7654668e17a7c5fd52af`; one capture, old LOCAL authoritative.
- Private binary recorder and independent SciPy dense Schur oracle prepared in
  `experiments/shared_linearization_s3_capture_20261012/`. Every pre-retirement
  snapshot has a separate successful-commit marker. Covariance query spacing
  is 40 epochs; no source publication-cadence change or native GPU reevaluation.
- Preflight passed: 44/44 Release twice and ASan/UBSan, existing18/18, original
  disabled MAIN output byte-identical, no-data ROS startup with seven verified
  private/pinned DSOs. Pose-only and mixed pose/velocity/bias recording fixtures
  pass independent information, joint and relative covariance checks through
  14 and 7 retirements respectively. S2 bytes unchanged.
- First preflight stopped on the expected receiver-wrapper output-path change;
  the path comparison was corrected explicitly. Model/config bytes unchanged;
  no data had played. Both preflight logs retained.
- Single authorized ETH1 first40s capture completed, root
  `runs/gnssfgo_shared_linearization_s3_20261012`, port11420,445.405s wall.
  All783 updates committed;313 retirements;5321 native contributions accounted
  for as4513 absorbed+808 active.20 checkpoints pass: information relative
  error3.85e-17,joint covariance6.01e-15,relative covariance7.06e-15.
- Native/lifecycle/diagnostic/coverage/conservation/stationarity gates pass;
  all393 receiver publications satisfy1e-5,21970 versions accounted for,queue
  max47/end0,receiver repeat nine files byte-identical.716 protected run files,
  680 inherited files,19 S1 pins and six source HEAD/clean-state checks unchanged.
- No shadow failure, retry, candidate publication or S4 promotion. Complete-scan
  median/p95210.474/431.506ms remains a dual-path shadow result. Candidate query
  median159.656ms warrants same-recorded-input cost work before any speed claim.
  Report:`experiments/shared_linearization_s3_capture_20261012/REPORT.md`.

### S2 — isolated native Gaussian history prototype

- Authorization: explicit user proceed after S1/B0 and clarification that MAIN
  Gaussian reuse was not yet implemented. Parent `4b2804cf094c590c8404e3c5fecd00ce682b2dbc`.
- Added a private Disabled/Shadow-only helper and tests under
  `experiments/shared_linearization_s2_20261012/`; no production source edit,
  launch flag or covariance publication. Uses existing GTSAM Gaussian elimination
  and unchanged CBS relative Schur extraction, not a second nonlinear optimizer.
- Separate native prior, positive-ID/version/slot contract, exact-once absorption,
  chart/fixed-key guards, tangent transport, transaction rollback/reset and
  observer-safe failure path. Actual provenance/hook integration remains S3.
- Final Release 26/26 twice; focused UBSan 25/25 and ASan+UBSan 25/25. Twelve
  retirements preserve bounded active history. Dense information relative error
  <=8.342e-16; endpoint covariance <=1.364e-15; numerical manifold covariance
  comparisons <=3.164e-10. Repeated numerical diagnostics exact.
- Actual ISAM2Ext synthetic cache fixture adds zero native evaluations and
  leaves MAIN equations/estimate unchanged. This is not GPU/live validation.
- 680 protected hashes, 19 S1 pins, six source HEAD/status checks and MAIN CPU
  binary unchanged. No covariance/Q/CBS/receiver changes, dataset or timing run,
  full build, production promotion or push. Report and evidence:
  `experiments/shared_linearization_s2_20261012/REPORT.md`.
- Stop at S2. Next decision: default-off real lifecycle capture and S3 shadow
  validation; current independent LOCAL stays authoritative.

### B0 — fresh unchanged ETH1 first40s reference

- Authorization: user "proceed" after completed S1; parent `6bcc1d9b37c1ff245ae361a032cff30b82002796`.
  One CBS-ON capture only, same preserved E1b.4a/E2 binaries and profile settings.
  Fresh root `runs/gnssfgo_shared_linearization_b0_20261012`, port11418 and session.
  Existing stopped containers restarted without rebuild/reconfiguration.
- Passed all retained functional checks, 18 Release regressions and the one
  saved-journal receiver repeat. Nine compared files byte-identical; state
  differences zero. 21970 versions =570IDs+21375replacements+25unsupported;
  393 strict publications, max gradient9.747059774e-6; no pending/duplicate/stale.
- LOCAL median/p95 10.965/15.385ms versus unbatched117.148/218.181ms.
  Complete-scan median/p95 206.942/437.376ms, late p95469.574ms;800ms gate passes.
  Previous batched p95 was385.682ms. Identical settings do not imply identical
  live solver history; more evaluations occurred. No new code regression claim.
- MAIN50.719s,185378synchronous calls;LOCAL3.415s,155556batched hits/zero
  synchronous fallback;covariance4.164s. Remaining MAIN cost is not solved.
  Queue max43,final0;output-age p950.090005s;controller445.37s.
- Report and evidence: `experiments/shared_linearization_b0_20261012/REPORT.md`.
  All680run hashes,inherited677hashes,19S1pins and six clean source HEADs pass.
  All owned processes exited; containers idle. Original/native GLIM, native
  Kimera, online fusion and final fusion remain distinct in coverage reporting.
- No source/model change, new covariance backend, GT scoring, tuning, retry,
  full160 run, push or default promotion. Stop here; S2 is the next decision.

### S1 — MAIN native-equation reuse feasibility

- Authorization: user "ok proceed" to the saved next stage; read-only audit
  only. Parent `a84436b6429abd68810ea5f1c86c181c0fb2b78a`; same planning branch.
- Result: `FEASIBLE_WITH_LIFECYCLE_HOOKS`, not numerical or speed acceptance.
  Actual MAIN is `gtsam_points::ISAM2Ext` inside the fallback smoother, not the
  ordinary LOCAL ISAM2. Existing forwarded Gaussian-cache/reference getters
  suffice; add pre-retirement capture and transactional commit/reset handling
  in a future private prototype, not a second nonlinear solver or GPU kernel.
- Native admission before CBS insertion, exact-once independent Gaussian prior,
  fixed-separator chart assertions and endpoint covariance transport are the
  required information/coordinate boundaries. MAIN fallback can otherwise hide
  a history reset across its extra update; observer errors must not trigger it.
- Report: `GLIM_SHARED_LINEARIZATION_FEASIBILITY_S1_20261012.md`.
  Evidence: `experiments/shared_linearization_s1_20261012/INTEGRITY.json`.
  All 677 inherited hashes and six source HEAD/status checks pass. Additional
  hashes pin 19 inspected source/build/library records. The CPU library was
  absent from the inherited manifest; only its current hash is newly pinned.
- No estimator/build/configuration change, test run, experiment, tuning,
  automatic follow-up, push or default promotion. Current LOCAL stays intact.
- Next: B0 fresh unchanged first40s overall-timing reference if authorized,
  then S2 isolated Gaussian helper and dense-reference tests. No run started.

### S0 — shared-native-linearization proposal and preservation

- Authorization: user requested a plan for the supervisor's method and saving
  current progress, not implementation or another experiment.
- Preserved baseline `3206123f4812d66158db58bd1f577167fe901875` with annotated
  local tag `checkpoint/e1b4a-before-shared-linearization-20261012`. Original
  E1b.4a branch remains at that commit. Documentation is on the separate
  `planning/glim-shared-linearization-20261012` branch.
- Plan: `GLIM_SHARED_LINEARIZATION_LOCAL_COVARIANCE_PLAN_20261012.md`.
  Proposed native-only Gaussian history reuses MAIN's factor equations but
  never its mixed prior/Bayes tree. Explicit chart, provenance, retirement,
  tangent transport, transaction and independent dense-reference gates precede
  any candidate publication. No statistical independence or equality to the
  independent LOCAL covariance is claimed.
- Rehashed all 677 protected files (10,554,314,414 bytes): zero differences.
  Six source HEADs/worktrees match the clean frozen records. Added hashes for
  15 retained private source/build/runtime/evidence files. Integrity and recovery
  boundaries: `experiments/checkpoints/20261012_shared_linearization/`.
- Default-off/private implementation, current native source settings and WNOA
  receiver remain unchanged. No build, estimator run, tuning, new correctness
  test, automatic follow-up or push. Existing validation remains historical.
- Next proposed step is S1 read-only feasibility. The user's requested fresh
  unchanged overall-timing reference (B0, proposed same first40s) precedes any
  candidate implementation and needs separate run authorization. Stop here.

### E1b.4a observer-free first40s timing — passed

- Authorization: latest user "proceed"; parent64be474f7a84d272fbbcddef0b577f81877dbff4.
- One first40s ETH1 classic temporary CBS-ON capture at0.10x with native GLIM,
  native Kimera and unchanged E2 source-WNOA receiver. Use the validated private
  release_2 LOCAL-update batcher; no snapshot checker or estimator/build edits.
- Protocol: `experiments/e1b4a_timing40_20261012/PROTOCOL.txt`.
- Compare retained E1b.3c profile under the same low-overhead instrumentation;
  require existing functional gates, and predeclare LOCAL median/p95 <=80% of
  baseline as the component timing gate. Report complete-scan/late-window costs
  separately; first40s is not full160 headroom validation. No automatic retry,
  default promotion, further optimization or push.
- Outcome: one capture446.404s wall;391nativeGLIM/184nativeKimera/393fused
  online and final poses. LOCAL median117.148->7.600ms,p95218.181->14.391ms,
  total39.131->2.856s. LOCAL GPU calls130972cached,zero synchronous fallback.
  Complete-scan median307.376->182.365ms,p95535.236->385.682ms; both declared
  timing/headroom gates pass. MAIN remains43.264s/162254synchronous GPU calls.
- All677protected hashes,six source HEADs,18preflight Release regressions,
  effective library/profile checks,native admission/lifecycle/dense covariance
  algebra,coverage,21946version conservation,strict publication gates passed.
  Maximum gradient9.851213209e-6,transportmaxqueue42,endqueue0;output-age
  p950.090004s. One saved-journal receiver replay:9files byte-identical.
- No estimator/build/default edit; raw artifacts preserved,all owned processes
  stopped. Report `experiments/e1b4a_timing40_20261012/REPORT.md` and
  `comparison.json`. Separate live captures are not identical-input histories;
  fixed-rate playback wall duration did not improve and no accuracy claim made.
  Stop here: no longer run,MAIN extension or receiver change in this task.
- Timing harness/report/evidence commit:
  `575bcf3308723039ed98acbc15787df3e1bf3942`; subsequent documentation checkpoint
  records this identity. No push.

### E1b.4a real-input diagnostic — sampled gate passed

- Authorization: user "proceed" after the exact-state checker checkpoint,
  parent `0a8e36d380b7ae8659c52be203a95f4611fdf32b`.
- Integrate the validated checker in a private wrapper only. One first20s ETH1
  source-only classic temporary CBS capture at0.10x; no receiver or GT scoring.
- Ordinary LOCAL execution remains authoritative. At update calls
  10,30,60,100,150,180 compare independent ordinary/batched snapshots and check
  copied ordinary against the actual ordinary result. Include fixed-lag retirement.
- Protocol: `experiments/e1b4a_real_input_20261012/PROTOCOL.txt`.
  This sampled exact-boundary gate complements the persistent-history synthetic
  tests; it is not an independently evolving full real-sequence branch.
- Preserve current profiles, source mathematics, frozen libraries and defaults.
  Stop on unsupported inputs/failure, no automatic capture retry. Diagnostic
  overhead must not be interpreted as production timing. Observer-free throughput
  remains a separate gate; update this entry with evidence before stopping.
- Outcome: exactly one capture,242.052s wall including startup,0.10x playback,
  drain and diagnostic overhead.192 LOCAL calls; all six selected checks passed,
  including four after first retirement at call51.126 retirement updates,423
  retired variable keys. Same factor identities/slots and tested solver decisions.
- Max ordinary/batch and authoritative/copy differences: p/v/bias/Gaussians/full
  covariance/latest relative covariance0; orientation6.830473686658665e-18rad.
  Nine batched graph requests,1126 prepared GPU factor evaluations;506 at the
  final selected update. No fallback at those graph requests.
- Native coverage: GLIM191poses/18.9976s,Kimera84poses/19.5999s,CPT7span19.8500s;
  startup-gap and tail gates passed. No GT accuracy conclusions. Source CBS
  remained active:74 G2K and81 K2G injections,56 committed GLIM retirement rows.
- Private wrapper compiled successfully;40 frozen checker Release tests rerun
  against its dependency chain passed. Before-playback settings/loader checks,
  copied profile equality,852 protected hashes and six clean source HEADs passed.
  All owned processes exited. No covariance/model edits, no receiver, no retry.
- Report/evidence: `experiments/e1b4a_real_input_20261012/REPORT.txt`.
  Implementation/evidence commit `2074422cb384b0908923fa49ce55844c3be33da7`;
  subsequent documentation checkpoint records it. No push.
  No default switch or promotion. Stop before the separately authorized
  observer-free first40s throughput gate. This result is not full160 acceptance.

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

### E1/E2 live160 capacity gate

- Authorization: user "Proceed" after the combined live60 pass and explicit
  proposal for a separate 160-second validation.
- Status: CAPTURE COMPLETE; VALIDATION BLOCKED; HEADROOM MISSED, 2026-10-11.
  One first-160-second ETH1 CBS-ON capture at
  0.1x, headless; exact E1 + corrected E2 binary/profile/settings from live60.
- Protocol: `experiments/e1_e2_live160_20261011/PROTOCOL.txt`.
- Parent `7109c4cac7a8c24e2b37c5742fbabb33fb7453b8`; isolated branch
  `engineering/e1-e2-live160-20261011`. No estimator edits, tuning, affinity
  changes, E1b/E3/E4, retry, production-default change or new sequence.
- Preserve strict publication, covariance, lifecycle, conservation and queue
  limits. Stop on capture/validation failure; report late-window growth and
  800 ms scan-p95 target independently from basic completion.
- Report: `experiments/e1_e2_live160_20261011/REPORT.txt`; 81 byte-identical
  small evidence files archived, including the failing validator traceback.
- Capture completed without queue abort: peak 60, final 0; output-age p95
  .120033 simulated seconds. 1593 logged stationary publications, max gradient
  9.94881047e-6. No source retry; no saved-journal replay was reached.
- Native validator failed at `assert (ri,rj) in mats`: 780 aligned Kimera
  candidates vs 775 matrix logs. All five missing edges match unchanged
  `drop_before_stage` guard decisions. Alignment is logged before admission;
  the validator assumes construction too early. Original validator and guard
  unchanged. No newly reproduced E1/E2 numerical defect established by this.
- Existing Kimera guard calls: 5 pre-stage drops, 47 staged rollbacks, 21 active
  factor removals/retries. Do not describe Gaussian/health-off as guard-free.
  End-to-end covariance/lifecycle/conservation and repeat certification remain
  blocked; later validation stages were not executed or silently waived.
- Full/late GLIM scan p95 1045.406/1129.950 ms miss the 800 ms target; 101 of
  398 late scans exceed 1000 ms. Late mean 617.202 ms allows this capture to
  finish, not a long-run capacity guarantee. Late receiver refinement p95
  929.644 ms. Main/LOCAL sidecar costs remain; covariance preparation late p95
  27.595 ms. Historical comparisons are not identical nonlinear workloads.
- 18/18 focused E1 Release tests; 635 protected hashes unchanged, source
  worktrees clean, owned processes exited. No full suite/sanitizer rerun claim.
- Proposed next step only: lifecycle-aware validator with explicit guard
  outcomes, then resume on the saved capture; no need to recapture sources
  solely for this blocker. No guard/estimator/default edits, E1b/E3/E4 or tuning.
- Local completion commit contains this report/ledger; no push.

### E1/E2 live160 validation continuation and bounded bug audit

- Authorization: user "Ok proceed and check for any other bugs that cause these
  instab", 2026-10-11. Status: FUNCTIONAL VALIDATION PASSED; HEADROOM MISSED.
- Scope: correct/test candidate-admission validation, then resume retained gates
  on the SAME completed live160 capture and ONE unchanged-binary journal replay.
  Inspect recorded rollback/recovery paths for concrete defects. No source rerun,
  tuning, guard change, production edit, E1b/E3/E4 or default promotion.
- Parent b6c94ca46739761073b5213c5b03f718a539ba46; isolated branch
  engineering/e1-e2-live160-validation-20261011.
- Protocol: experiments/e1_e2_live160_validation_20261011/PROTOCOL.txt.
- Preserve original failed validation and all capture/binary/profile bytes.
  Unexplained missing factors, covariance mutation, wrong-update or duplicate
  guard evidence must FAIL. Factor construction and post-stage rollback remain
  distinct. Existing numerical limits and performance gates are unchanged.
- Report: experiments/e1_e2_live160_validation_20261011/REPORT.txt.
- Two validator defects corrected only in continuation scripts: pre-stage guard
  rejection versus construction, and active-factor/nested rollback accounting.
  781 GLIM constructions; Kimera780 candidates/775 constructions/5 explicit
  pre-stage drops,59 undone constructions,716 retained admissions,698 removals,
  18 remaining.47 staged rollback transactions and17 active rollback transactions
  (21 old-factor guards). All restored slots/counts reconcile.
- 24 new validator tests; unchanged focused receiver50/50 Release and50/50
  ASan/UBSan. No source re-run, new build, full suite or dependency/GPU sanitizer.
- ONE saved-journal replay,481.537839s: all19 non-timing products exact; online
  and final1593-row state/trajectory products byte-identical, zero differences.
  95162 source versions exactly conserved,92767 replacements,2370 IDs,
  25 explicit unsupported pre-init,0 pending/stale/duplicate. All1593 outputs
  stationary, max gradient9.94881047e-6;67 numerical recoveries reproduce.
- No additional E1/E2 defect reproduced in the checked paths. First Kimera
  staged residual guard at101.151367s; this is separate from receiver numerical
  recovery beginning84.78s. Individual residual regression is not proof of an
  optimizer bug. No complete posterior covariance/source rollback equivalence
  claim; this validates recorded lifecycle and retained receiver replay.
- 635 protected originals,3 full source logs and81 earlier archive files
  unchanged; source worktrees clean, owned processes exited. Original failed
  logs/report remain immutable. No push; local completion commit holds report.
- Headroom still fails: full/late GLIMscan p951045/1130ms, target800ms.
  Every ordinary receiver refinement relinearizes all retained X/V/W states.
  No promotion, tuning, longer capture, E1b/E3/E4 or follow-up change performed.

### E4 — guarded selective refinement, saved160 only

- Authorization: user "ok proceed" after the explicit selective-refinement
  proposal, 2026-10-12. Reprioritized ahead of E3; E3/E1b remain unstarted.
- Status: COMPLETED, NOT ACCEPTED. One hypothesis: gradient-selected variables plus their
  one-factor neighbours can reach the unchanged global stationarity certificate
  more cheaply than unconditional full relinearization.
- One selective trial per publication, on a private solver copy. Accept only
  finite non-increasing/stable-descent objective and fresh gradient <=1e-5.
  Otherwise discard it and invoke unchanged reference full refinement on the
  original solver with the remaining budget (20 total attempts, not 20+1).
- No model, source, covariance, priors, revision, event-order, lag, publication
  tolerance or working-binary changes. No new source capture or ATE tuning.
- Protocol: experiments/e4_selective_refinement_20261012/PROTOCOL.txt.
- Isolated branch engineering/e4-selective-refinement-20261012, parent
  b59948c5dbb72d63f4b1b9d970f66df1e1ea0dbd. Preserve saved160 and old binaries.
- Report: experiments/e4_selective_refinement_20261012/REPORT.txt.
- Candidate a9bad7448eb15e3388f872c3c167d43f8ce41c7380a46197ecd32ca00cac1d1d;
  default-off, old working executable unchanged.63/63 Release and63/63 ASan/
  UBSan focused tests pass (50 existing+13 new); dependency/GPU suite not claimed.
- Same-build disabled full160 reproduces frozen online/final states/trajectories
  byte-identically. Enabled run stopped on observed prefix-equivalence failure
  at130.68s, not a crash.1301/1301 completed outputs stationary; state max deltas
  p8.12549e-5m,q1.34762e-6rad,v7.61396e-7m/s,w5.61191e-8rad/s.
  Small numerical differences, not physical instability; frozen bounds not waived.
- Earliest posthoc orientation bound failure at1.58s, position at90.98s.
  Current prefix check triggered stop later; no full160 candidate/final result
  or deterministic real repeat. Preserved partial/trailing records explicitly.
- 169 selective successes;1057 nonstationary trial rejections;51 all-key direct
  fallbacks;24 initial stationary.86.7659% nonstationary checks require fallback.
  Matched refinement238.870->333.506s (+39.6%);120-130.68s p95 .850->1.038s.
  One paired timing only. Local trial cheap, global verification/fallback costly.
- 655 protected files unchanged; no source rerun/tuning/default promotion/push.
  Completion commit contains report/evidence. STOP; do not expand to another
  selective policy or fixed lag without a new user decision.

### E1b.1 — remaining GLIM LOCAL update diagnosis

- Authorization: user "Ok proceed step by step and refer to the plan in order
  to not lose track", 2026-10-12, after the GLIM-first/GPU-backend discussion.
- Status: COMPLETED. First step only: inspected existing code and retained
  E1/E2 live160 source measurements; no acceleration patch implemented.
- Parent a936c56b34760d38585b2aed4724439c4726aa11; isolated branch
  engineering/e1b-local-update-audit-20261012.
- Protocol: experiments/e1b_local_update_audit_20261012/PROTOCOL.txt.
- Freeze E1/E2, source settings and receiver; E4 stays disabled. No GTSAM/CUDA
  upgrade, optimizer replacement, covariance/lag/cadence change, new source
  run or fusion experiment. Keep separate LOCAL conditioning and all revisions.
- If retained logs cannot partition LOCAL update time, say so and specify the
  smallest instrumentation needed; do not invent GPU/QR/marginalization shares.
- Report and update this entry before the next implementation/experiment.
- Report: `experiments/e1b_local_update_audit_20261012/REPORT.txt`;
  `evidence.json` contains timing/latency distributions and input/source hashes.
- Confirmed E1 batches LOCAL covariance preparation only. Actual LOCAL update
  still uses generic iSAM2 and synchronous GPU-factor fallback; native main uses
  gtsam_points batching. No uncontrolled LOCAL state growth: lag5s,150/153 keys
  before marginalization after startup,zero external LOCAL factors throughout.
- Main/LOCAL late wall shares 55.37%/33.44%; medians 354.15/335.21 ms. These shares
  use disjoint matched per-scan timers, not added medians. Late scan p95 remains
  1129.95 ms. One final callback timing row is absent, explicitly excluded only
  from subphase statistics;1,591 normal scan totals retained.
- Matched 75,912 GLIM versions to transport, 1,589 first versions. Late first
  belief producer-age p95=.254984 simulated seconds; first-version log-tail
  observation p95=.001025 simulated seconds overall. This does not measure
  receiver solve/queue or Kimera acceptance. No new payload/scoring experiment.
- Backup and post-estimate work occurs in LOCAL despite empty external sets,
  but its cost is not separately measured.2,416,049 synchronous GPU warnings
  cannot be phase-attributed from mixed untimestamped logs. Do not remove guards
  or guess GPU/elimination cost percentages based on them.
- Validation:5 parser tests, before/after seven input hashes and six clean
  source repos;635 protected manifest hashes match. Completion commit contains
  report/scripts/evidence/ledger. No estimator edits, runs, upgrades or push.
- Decision: next E1b.2 is one instrumentation-only patch/profiling proposal
  specified in REPORT section6. No automatic implementation/capture here.

### E1b.2 — isolated LOCAL update instrumentation

- Authorization: user "Proceed" after E1b.1; 2026-10-12.
- Parent 75e315974f86b158bb0b50962000e0ffbf4fafdb; branch
  engineering/e1b-local-profile-20261012.
- Status: PRIVATE IMPLEMENTATION/TESTS PASSED; not a performance optimization
  or live acceptance. Default-OFF scoped timing in private source overlays/builds;
  owning source checkouts and working libraries unchanged. No source capture.
- Protocol: experiments/e1b_local_profile_20261012/PROTOCOL.txt.
- No optimizer upgrade, semantic change, source/fusion run, tuning or automatic
  next acceleration patch. Report test limits and measured instrumentation cost.
- Report: `experiments/e1b_local_profile_20261012/REPORT.txt`; `validation.json`
  and `evidence/` retain commands, XML, source/build identities and overlay diffs.
- Measures existing backup/BaseSAM/estimate/marginalization/cleanup and GTSAM
  phases plus GPU sync/cache/warning work. No extra solver/factor evaluation.
  Pinned GTSAM internal TBB is disabled; worker utility tests do not establish
  parallel GTSAM execution. Do not change that flag on this evidence.
- 29 focused Release tests and29 UBSan pass;8 focused profiler ASan/UBSan pass.
  One preexisting timing benchmark remains disabled, not counted as passed.
  Original-library vs candidate/repeat:4,716 state/covariance numbers exact;
  counted factor calls unchanged. Shared GPU matrix/cache/inlier fixtures pass.
- Synthetic RAM timing overhead median1.12us/update (~2.3% paired); file-output
  fixture adds4.53–5.84us vs bracketing OFF medians (~10–13%). Not a live GLIM
  overhead bound. JSON output excluded from measured root, included in outer
  fixture timing; record this limitation during any future source capture.
- 635 protected files,492 shared relink inputs and six clean pinned source repos
  unchanged. Private same-version host relink only; no production installation.
  Missing test -ltbb link fixed in retained release_2; no math/test tolerance fix.
- Decision: stop here. E1b.3 proposes one short frozen source profile (first40s),
  explicitly not a reproduction of the late120–160s tail. Pick one acceleration
  only after measured attribution. Commit locally; no push.

### E1b.3 — first40s concurrent GLIM profiling capture

- Authorization: user "proceed", 2026-10-12; parent83fc5ed6dd06457321bdb9d32fc9679c96ed2f55.
- Status: BLOCKED BEFORE PLAYBACK. Branch engineering/e1b-profile40-20261012.
- Protocol: experiments/e1b_profile40_20261012/PROTOCOL.txt.
- One first40s ETH1 CBS-ON capture at0.10x, frozen E1/E2/temporary sources,
  co-running WNOA receiver; private E1b.2 timing DSOs ONLY on GLIM. No tuning,
  lag/covariance/model edits, E4, retry, optimization or default promotion.
- Reuse corrected admission/lifecycle validators and exact journal replay.
  Partition measured LOCAL work before proposing one next engineering patch.
- Report: experiments/e1b_profile40_20261012/REPORT.txt; validation.json and
  evidence retain failure, preflight, private harness and protected identities.
- One launch only. Preflight/18 regression tests passed; actual GLIM process
  aborted at startup with malloc.c:4036 _int_malloc assertion, exit-6, before
  readiness or bag playback. Root cause unknown; no workload-growth inference.
- Zero IMU/belief/output events and zero profiler rows. Actual DSO-map gate,
  source/fusion numerical validation and journal replay never reached. Existing
  focused tests were insufficient to certify whole-application startup.
- Owned controller interrupted/cleaned; both containers idle.680 protected
  files and six clean source repos unchanged. No retry, optimization or push.
- Next proposed bounded task is private-build startup debugging without data,
  not another capture or speed patch. E1b.2 is NOT runtime-ready pending that
  diagnosis. Preserve original attempt and require new authorization to retry.

### E1b.3a — private GLIM startup diagnosis/fix

- Authorization: user "proceed", 2026-10-12, following E1b.3 startup failure.
- Status: SPECIFIC BUILD BUG FIXED; Release no-data startup validated.
  Branch engineering/e1b-startup-fix-20261012.
- Parent7c1b07f4c79e55a16163b5a5cb5a33ed6899f2a9.
- Protocol: experiments/e1b_startup_fix_20261012/PROTOCOL.txt.
- Debug startup with no data; fix only a reproduced private-build defect,
  add regression and validate startup. No40s retry, source algorithm edits,
  optimizer/covariance/lag changes, new dependency, acceleration or push.
- Report: experiments/e1b_startup_fix_20261012/REPORT.txt; validation.json
  and evidence retain the corrupting write, dependency closure and tests.
- Private E1b.2 relink reused stale glim_ros.cpp.o allocating3296 bytes with
  a current bridge requiring3344. ASan reproduces an8-byte constructor write
  immediately outside that allocation. Working E1 already allocates3344.
- Recompile unchanged wrapper/inspector header users, reuse exact instrumented
  bridge and all other DSOs. No production build or source/math edit. Five
  fail-closed layout regressions; actual Release profiling OFF/ON startup,
  idle and exit0 pass with runtime DSO maps and zero sensor measurements.
- 29 focused Release tests,8 profiler ASan/UBSan pass;1 timing test disabled.
  Original/repeated solver4,716 state/covariance numbers still byte-identical.
- ASan actual-node limits: one early DEADLYSIGNAL without stack; two verbose
  repeats reach readiness/exit0 without host error but each has6 CUDA allocation
  warnings. Not full CUDA/dependency sanitizer certification. Preserved all logs.
- 680 protected paths,225 shared relink inputs,six clean source repos unchanged;
  owned processes stopped. E1b.3 failed capture remains unchanged. No speed claim.
- STOP. A new explicitly authorized first40s capture may use fixed_release plus
  startup/layout gates; no automatic retry, next speed patch, promotion or push.

### E1b.3b — repaired first40s ETH1 LOCAL profiling capture

- Authorization: user "proceed" after E1b.3a report, 2026-10-12.
- Status: BLOCKED BEFORE PLAYBACK; branch engineering/e1b-profile40-fixed-20261012.
- Parent391e936dd846a831d9cffc343c2b21b9f53b2171.
- One fresh first40s ETH1 classic CBS-ON capture at0.10x with concurrent native
  GLIM, native Kimera and frozen E2 WNOA receiver. Reuse E1b.3 source profile,
  priors, source/receiver binaries and integrity gates; change only run paths,
  ROS port, unique source session and GLIM's repaired private libglim_ros.so.
- Preflight must verify3344-byte wrapper allocation, actual process maps and
  profiler environment before playback. Validate source/receiver conservation,
  publication stationarity, original native outputs and deterministic replay.
- Stop on failure, no automatic repeat. Partition LOCAL timing only from a
  successful, complete capture. No optimization/tuning/follow-up sequence/push.
- One capture launched. Preflight passed 668 protected hashes and 18 tests, but
  `verify_effective.py` falsely required symlinked dependency `libglim.so` to
  appear under `fixed_release` in `/proc/maps`; Linux reported its real pinned
  `release_2` target. Source stopped before bag playback; transport counts all
  zero. Retained E1b.3a actual maps prove the repaired wrapper path and resolved
  dependency paths. Post-stop 668 hashes unchanged, owned processes stopped.
  No timing or trajectory result and no retry. Full evidence/report:
  `experiments/e1b_profile40_fixed_20261012/REPORT.md`. Next task, if
  authorized, must correct/test only this pre-playback gate and use a new root.

### E1b.3c — symlink-aware admission repair and fresh first40s capture

- Authorization: user "fix it" after E1b.3b false admission failure, 2026-10-12.
- Status: VALIDATED FIRST40S PROFILE; branch engineering/e1b-profile40-mapguard-20261012.
- Parent c66ef94. Correct only the process-map admission check, prove it
  accepts the retained repaired-build maps and rejects missing/wrong DSOs,
  preflight a fresh run root, and execute one first40s ETH1 CBS-ON capture.
- Freeze native GLIM/Kimera, E2 WNOA receiver, data, source settings, profiler
  library, support clock, priors, covariance and classic CBS semantics. Stop on
  any new source/receiver/validation failure; never overwrite E1b.3/3b.
- If the run completes, validate native coverage, source/receiver accounting,
  publication stationarity, saved-journal replay and LOCAL timing before any
  performance attribution. No optimization, tuning, new sequence or push.
- The symlink-aware real-path gate passed six wrong-path controls and the new
  actual process map before playback. Fresh ETH1 first40s CBS-ON finished at
  0.10x with native GLIM/Kimera and E2 WNOA receiver. Native, lifecycle,
  dense-factor, conservation, strict publication and nine-file byte-identical
  saved-journal validation passed. 3,986 timestamp samples, 21,970 belief
  versions, 393 online supports, queue maximum48, pending0. No accuracy score.
- 1,565 complete profile rows; 392 LOCAL. LOCAL owner-exclusive fallback GPU
  linearization37.724/39.131s=96.405%;143,950/143,950 LOCAL GPU linearizations
  synchronous, zero cache hits. MAIN also168,874/168,874 fallback; COVARIANCE
  has195,592 cache hits and zero fallbacks. First40s result only, not the late
  120–160s tail or an isolated profiler overhead bound. One measured next
  candidate is exact-Values LOCAL batch linearization, not implemented here.
- Full report `experiments/e1b_profile40_mapguard_20261012/REPORT.md`; parser
  and JSON analysis alongside it. Post-run670 protected hashes match, six
  source repos clean, owned processes stopped. No speed patch or push.

### E1b.4 — concrete LOCAL update batching proposal

- Authorization: user asks to prepare a suggested solution preserving the
  mathematics and algorithm reasoning, 2026-10-12. Preparation only.
- Status: PROPOSAL READY; no implementation, build or experiment.
- Parent `11a99e8ac6529bef907bf6265ef4b737dc835958`; documentation branch
  `engineering/e1b4-local-batch-proposal-20261012`.
- Proposal: `experiments/e1b_local_update_batch_proposal_20261012/PROPOSAL.md`.
- First patch would scope an optional graph-linearization callback to LOCAL's
  update and reuse E1's synchronous-compatible VGICP batching at the exact
  Values supplied by iSAM2. Cover new/full-window graph calls first; retain
  direct incremental/marginalization calls until residual timing justifies an
  extension. Preserve default path, graph ordering, cache/inlier semantics,
  fixed-lag decisions and all source/receiver mathematics.
- Concrete code boundaries, recursion/fallback/exception handling, shared-factor
  risks, build/ABI discipline and numerical/performance gates are documented.
  Existing CUDA batching still synchronizes; no single-kernel claim or promised
  whole-system speedup. No new cross-update memoization or factor dropping.
- Validation for this preparation: code-path inspection against pinned source
  and E1/E1b evidence; documentation diff checks only. Prior tests are not
  represented as new tests of this proposed integration.
- Next implementation step is E1b.4a isolated patch/fixtures. Real source
  equivalence needs complete GPU fixtures or a bounded capture; saved receiver
  journals alone are insufficient. No automatic full160 run or push.

### E1b.4a — scoped LOCAL graph batching implementation

- Authorization: user "ok proceed and implement the plan and report once you do
  one by one", 2026-10-12. First implementation/fixture step only.
- Status: CORRECTNESS GATE PASSED; TIMING NOT RUN. Parent `db40f27`; branch
  `engineering/e1b4a-local-update-batching-20261012`.
- Protocol: `experiments/e1b4a_local_update_batching_20261012/PROTOCOL.txt`.
- Preserve the E1b.4 design and numerical/performance gates. Private sources and
  build products only; scoped graph-linearize dispatch for LOCAL new/full-window
  requests, default disabled. Existing direct incremental/marginalization calls
  and source/receiver settings remain unchanged. No live capture or push.
- Complete focused implementation, deterministic/exception/equivalence tests,
  supported sanitizer checks and matched fixture timing, then report before
  extending to another solver path or starting a real-source capture.
- User subsequently requested "report me before the timing test". Stop after
  correctness and reporting; do not interpret ordinary test elapsed times as a
  performance result. Performance gate remains frozen and untested.
- Implementation/test commit `440fba99e9ce4d59dec03758b9d06f55927b2470`.
  Report: `experiments/e1b4a_local_update_batching_20261012/REPORT.md`.
  Private root `runs/e1b4a_local_update_batching_20261012`; production sources,
  default selector and protected runtimes are untouched. New TLS dispatch is
  scoped only to LOCAL graph-linearize requests; direct individual loops remain.
- 32 focused Release tests, 30 host UBSan tests and ten supported CPU ASan/UBSan
  tests passed. Historical 4,716-number CPU fingerprint exact. Independent
  repeated fixed-lag fixtures through removal/marginalization have zero p/v/bias
  and covariance difference; max orientation difference 1.39e-17 rad. Same keys,
  slots and tested solver decisions. Synthetic heavy-workload correctness only,
  not its timing. No full-workspace or CUDA-device sanitizer certification.
- Selector-enabled no-data GLIM startup/expected subscriptions/clean shutdown
  passed; six real mapped library paths verified. All 670 baseline hashes and
  additional build inputs unchanged; six pinned source repos clean. No source
  playback, receiver run, covariance/model changes, push or benchmark. Candidate
  not promoted pending timing and later real-source gates.

### E1b.4a timing — frozen synthetic benchmark

- Date/authorization: 2026-10-12, user "ok proceed to the timing test".
- Status: SYNTHETIC TIMING GATE PASSED; continuation of E1b.4a, no new
  implementation hypothesis. No live acceptance or promotion.
- Parent `2a3823c7387baeb48b38db4a8e19bfd96c4f8d79`. Use the already validated
  `release_2/test_solver_3` unchanged, no rebuild or source experiment.
- Protocol: `experiments/e1b4a_local_update_batching_20261012/TIMING_PROTOCOL.txt`.
- CPU-only and 64-new-GPU-factor fixtures; six alternating paired repetitions,
  all 40 updates included. Existing numerical and <=80% median/p95 timing gates
  unchanged. CPU fallback and repeatability reported separately.
- Container was stopped at entry; inspected command is only `tail -f /dev/null`.
  Restarted the existing container without starting any sensor/estimator node.
- Benchmark ran exactly once (31.281s wall), 960 measured updates,12 paired
  histories and24 dense matrix comparisons, all passed. GPU-heavy median/p95
  112.310/115.43625ms ordinary vs4.30792/4.66825ms batched (26.07x/24.73x).
  Every GPU paired repetition passes the unchanged <=80% gate independently.
- GPU linearize calls111,822 in both modes; synchronous fallback111,822->384,
  prepared-result consumptions0->111,438. Remaining64/run occur at update1.
  No factor calls removed. Active keys and slots agree; no direct-loop extension.
- Max p/v/bias/full and relative covariance differences0; orientation3.71e-18rad.
  CPU median+0.20%, p95+3.68% (0.31/6.59microseconds), within observed paired
  repeat variation, not proof of zero overhead. All protected hashes and six
  source HEADs/clean states preserved before/after.
- Limits: 864-point synthetic cloud,21 active keys,472 factor slots; forced
  full-window relinearization fixture. Update-scope time only, not entire frame.
  CPU/GPU clocks unlocked, desktop GPU load present. No live throughput claim.
- Report: `experiments/e1b4a_local_update_batching_20261012/TIMING_REPORT.md`;
  raw evidence `runs/e1b4a_local_update_batching_20261012/release_2/timing1/`.
  Two preflight metadata-check issues were corrected/preserved before execution;
  no benchmark retry or source/binary/runner change.
- Stop here; real-source equivalence/throughput is a separate authorization.
  No MAIN optimization, further implementation, live run or push.

### E1b.4a real-input gate — entry audit

- Date/authorization: 2026-10-12, user "proceed" after the synthetic timing
  handoff. Parent `899896fa3bef7ee3d8d038dc9768739b402aa4af`.
- Status: HOLD BEFORE PLAYBACK: independent real-input oracle prerequisite
  missing. Entry audit completed; real-data equivalence/throughput NOT RUN.
  No new optimization or default change is authorized by this entry.
- First reuse the E1 real-graph diagnostic and E1b.3c first40s controller where
  suitable. Do not mislabel old E1 covariance-only comparisons or a receiver
  journal replay as E1b.4a independent LOCAL-update equivalence.
- The existing GPU factor `clone()` constructs a fresh factor, rather than
  preserving runtime settings, inlier buffers and cached results. A shallow
  smoother copy shares GPU state. Neither alone satisfies the exact-state
  independent-reference requirement. This is an oracle limitation, not a
  demonstrated estimator/batching bug.
- Evidence and proposed diagnostic-only next step:
  `experiments/e1b4a_real_gate_preflight_20261012/REPORT.txt`.
  Do not weaken the gate, launch throughput as a substitute, or conflate this
  hold with rejection of the passed synthetic timing result. No source/library
  edits, builds, experiments, default changes or push were made in this audit.

### E1b.4a real-input checker — isolated implementation

- Date/authorization: 2026-10-12, user "proceed" after the prerequisite hold.
- Status: CHECKER FIXTURE GATE PASSED; REAL CAPTURE NOT RUN.
  Parent edaa3e69031099b243cee73b38a767a031b5c9b5.
- Implement diagnostic-only exact-state GPU snapshots and independent LOCAL
  update comparisons. Preserve the validated batching DSOs, production source
  checkouts and all estimation settings. No playback or timing in this task.
- First prove snapshot fidelity, independent mutable storage, and rejection of
  unsupported inputs; then exercise ordinary/batched LOCAL updates through
  replacement, exceptions and fixed-lag marginalization. Record limitations.
- Protocol and evidence: experiments/e1b4a_snapshot_checker_20261012/.
- Exact device geometry/cache/inlier snapshots, alias-preserving factor copies
  and clean LOCAL checkpoint branches implemented in private friend-only
  overlays. Actual ordinary update is a separate fidelity oracle; covariance
  checks operate on disposable copies. Unknown/nonresident inputs fail closed.
- 40 Release, 40 host UBSan and 14 CPU ASan/UBSan tests pass; persistent branches
  through replacement and marginalization. p/v/bias/H/full/relative covariance
  differences zero; maximum orientation difference 6.993e-18 rad. Original
  tolerances unchanged. No full-workspace/device-sanitizer claim.
- 670 baseline /686 combined host hashes, 429 container build inputs and six
  pinned source repos unchanged; sets overlap. Validated batching DSOs unchanged.
- Report: experiments/e1b4a_snapshot_checker_20261012/REPORT.txt. Next: separately
  integrate bounded real-input observer, then observer-free throughput only
  after equivalence passes. No speed result is inferred from checker runtime.
- Implementation/validation commit: a3846056bc081fc5af040a47c15993b55154328c;
  subsequent documentation-only entry records this hash. Not pushed.
- No automatic live capture, next optimization, parameter change or push.

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
