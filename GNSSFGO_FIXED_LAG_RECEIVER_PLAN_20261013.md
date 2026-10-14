# Fixed lag WNOA receiver implementation and validation plan

Latest status 2026-10-14: the fresh FL4c ETH1 first30s CBS-OFF live pilot
completed at0.10x. Native GLIM, native Kimera and the fixed-lag WNOA receiver
co-ran; GLIM's provider-required ActiveWindowTemporary bookkeeping did not
enable external belief reception. No source-to-source factors were injected.
All15,884 downstream versions were accounted for,293 fused supports were
published, p95 whole-support service was64.41ms, and saved-journal repeat
matched16,365 committed files plus numerical outputs byte-for-byte. Kimera
had a3.20s startup gap. See [FL4c report](experiments/fixed_lag_live_fl4c_20261014/REPORT.txt).
This validates one bounded OFF operating point, not GT accuracy, CBS-ON,
fixed-lag nonlinear stationarity, long-run capacity or production equivalence.
FL3i's strict whole-replay failure remains failed. A matched ON capture is a
separate decision, not an automatic continuation.

Created 2026-10-13. Historical FL4a status: live preparation passed its
wrapper check but the OFF source launch failed before bag playback. S4d's
shared_native provider requires ActiveWindowTemporary while the OFF launcher
selects observe_only. Zero IMU/belief events; no live accuracy or timing result.
All owned processes stopped; no retry or method change. Resolve this bounded
launch/provider contract before another pilot, without relaxing the source
guard or enabling belief reception. See [FL4a report](experiments/fixed_lag_live_fl4a_20261014/REPORT.txt).
Earlier stopped stages and later remedies below remain historical evidence;
the FL3i strict-equivalence failure has not been relabelled as a pass.

FL3j completed2026-10-14: same-input retirement audit at support1018 PASSES.
Stock and slot-reuse paths retain identical source factors and fixed anchors;
canonical marginal/whole-graph H,g,constant differences0. Immediate state
difference3.47e-18; native slot-order assembly difference1.03e-17. Observation
leaves62,043prefix files byte-identical;118,986protectedfiles unchanged.
No bookkeeping defect found at the target. Stop precision investigation and
recommend a short, explicitly experimental live pilot next, with conservation,
queue and native/fused coverage checks. No live run authorized by this ledger
entry; no production promotion or relabelling of FL3i's strict gate failure.
See [FL3j report](experiments/fixed_lag_slot_audit_fl3j_20261014/REPORT.txt).

FL3i completed2026-10-14: isolated marginal-slot reuse fixes backing-array
growth (175slots versus1,571 in the ONE148.58s saved capture), but full
numerical equivalence to FL3h FAILS. As-played position difference
RMS.01843mm/max.27943mm; finalized RMS.02100mm/max.17217mm, not GT errors.
50,818Release/repeat/sanitizer checks and deterministic20s prefix pass;
source accounting and covariance payloads remain exact. Wall66.550s versus
66.446s, no meaningful speedup.505protectedfiles unchanged. No tolerance
relaxation, extra full replay, source/live run or production promotion.
Candidate retained privately; previous FL3h remains unchanged. FL4 held.
See [FL3i report](experiments/fixed_lag_slots_fl3i_20261014/REPORT.txt).

FL3h completed2026-10-14: separately authorized native-style incremental path
from initialization PASSES synthetic1193Release/repeat/sanitizer checks,
deterministic20s prefix and ONE complete existing148.58s arrival capture.
Original authored marginalization/fixed linearization points; no custom chart
reload or repeated stationarity solve.66.446s wall; all2,201IDs/86,053revisions
accounted for. As-played position versus full-history RMS.354mm/max2.701mm;
fixed-lag-finalized versus full-history-final RMS.393mm/max1.894mm. Not GT errors
or strict nonlinear equivalence. Whole support-cycle p9593.628ms/max132.402ms;
0.1x recorded-arrival queue drains. Active supports71/factors<=172/RSS25MiB,
but factor-slot extent grows to1571: no indefinite-memory/cost certification.
No live run, tuning, production change or promotion; old failures remain failed.
Next engineering issue, if authorized, is stock-preserving empty-slot lifecycle,
not returning to chart reload or tuning WNOA. See
[FL3h report](experiments/fixed_lag_native_fl3h_20261014/REPORT.txt).

FL3g diagnosis completed2026-10-14: native GLIM/Kimera use bounded ordinary
incremental updates, not our repeated full nonlinear publication certificate.
Our anchored-marginal/reload path also differs from their fixed-linearization
marginal handling. Saved-objective curvature supports a coupled slow nonlinear
mode, with WNOA contribution but no demonstrated WNOA equation bug or precise
single-factor cause. No estimator/source/journal run or repair;430protected
files unchanged. The marginal/publication contract needs a deliberate design
decision before another solver trial. Existing acceptance criteria are NOT
relaxed. See [FL3g report](experiments/fixed_lag_cause_fl3g_20261014/REPORT.txt).

FL3f completed2026-10-14:one bounded incremental policy is REJECTED,not promoted.
One ordinary iSAM2 step before/after original retirement/reload completes in
32.86ms excluding separately timed reference reconstruction,but differs from
the converged reference by max.369657m/.381882deg after retirement;velocity
and angular-rate screens also fail. Joint prior at identical reference Values
changes beyond all three declared information/gradient/nullspace limits.
Reference pre/final Values exactly reproduce FL3e;75000prefix archivefiles and
429protectedfiles unchanged. Lifecycle and source accounting pass;output is
explicitly a private approximate candidate,not converged production output.
No next event,source run,full replay,parameter sweep or automatic retry.
This rules out the tested shortcut,not fixed lag or all incremental policies.
See [FL3f report](experiments/fixed_lag_incremental_fl3f_20261014/REPORT.txt).

FL3e completed: the WHOLE original failing update passes with a private,
target-only shared200-step ceiling.116pre-refinements+retirement+reload+
7post-refinements=125steps;committed gradient6.71063e-6 versus1e-5.
Complete event1.358424s,of which retirement.000188s and reload.007093s.
Retains71supports/174slots;correct2absorbedIDs,1pendingadmission,one finalized
state andoneonlinepublication. Retained position change from converged pre-solve
to committed state max22.09micrometres.117pre-solveValues/costs/gradients
exactly match FL3d;75000prior archivefiles and287protectedfiles unchanged.
Stopped immediately after event84958. No production policy change, later event,
source run or full-replay/live-capacity acceptance. The earlier suggestion of
a full saved-journal replay was superseded by the user's concern about future
latency and the separately authorized bounded-policy FL3f test above.
See [FL3e report](experiments/fixed_lag_complete_update_fl3e_20261013/REPORT.txt).

FL3d completed: original iSAM2 state restored by one saved-prefix replay.
At the known failed pre-retirement update, the unchanged solve loop reaches
gradient7.97781e-6 below1e-5 at116steps (200ceiling),116accepted/0rejected,
1.272s including observation/checks. Initial and first20Values/costs/gradients
exactly reproduce FL3a;75000prior archive files byte-identical;161protectedfiles
unchanged. Step20 to116 changes position by.104520mRMS/.107055mmax and
orientation by.108191degRMS/.110616degmax. Diagnostic throws before retirement
and publication, restoring prior event84957/newest1223. Single solve success
does not validate the complete retirement/publication cycle or live throughput.
Production20-step policy unchanged; no later events, retry or new source run.
See [FL3d report](experiments/fixed_lag_isam_convergence_fl3d_20261013/REPORT.txt).

FL3c incomplete: authorized single-window GN200 diagnostic stops at its
reference-path check at step2. Recursive batchQR differs7.83e-6 in local
coordinates(maxposition8.17micrometres) against1e-8 reproduction tolerance.
This is NOT a production failure or proof that200steps fail.22Release and22
focusedASan/UBSan checks pass;148protectedfiles unchanged. No retry/tolerance
relaxation or source/journal run. Exact incremental-state reproduction remains
needed before concluding how much additional work the original solver needs.
See [FL3c report](experiments/fixed_lag_convergence_fl3c_20261013/REPORT.txt).

FL3b completed: the user's selected adaptive-damping test does NOT repair the
saved failure within20attempts. Installed GTSAM CeresDefaults diagonal/adaptive
LM,QR,unchanged objective/threshold:20accepted,0rejected,finalgradient0.04378656
versus1e-5.42Release/42focusedASan/UBSan checks pass;134protectedfiles unchanged.
No production integration, full replay, settings sweep or automatic retry.
This one policy is not adopted; no claim that every adaptive policy must fail.
See [FL3b report](experiments/fixed_lag_damping_fl3b_20261013/REPORT.txt).

FL3a diagnosis completed:the failed publication uses20accepted,cost-decreasing
steps and exhausts its budget at gradient.000547608. Fresh batchQR reproduces
iSAM2 steps to2.33e-12;the original marginal Jacobian agrees with independent
finite differences. Remaining errors alternate beside the old boundary;most
position motion is a common rigid displacement, not path deformation.75000
archive/index files byte-identical and94protectedfiles unchanged. Bounded
defaultLM controls do NOT pass the strict gradient check. No repair or complete
replay accepted. Next proposed decision is a private, bounded nonlinear-solver
test on the saved single objective with all model/acceptance settings preserved.
See [FL3a report](experiments/fixed_lag_failure_fl3a_20261013/REPORT.txt).

FL3 result:20s saved ETH1 prefix passes, including129retirements and exact observer/repeat products. ONE full saved148.58s attempt stops before publicationX1224 (122.4s after receiver initialization) on `pre-retirement stationarity`.1224previous publications pass;1153retirements retain71supports. Successfulportion inclusive supportservicep95.123539s,max.203624s,RSS27448KiB;no completed operational acceptance. All admitted versions/payloads reconcile;80protectedfiles unchanged. No retry, tuning, source/live run or default change. Failedgradient/stepcount were not retained by FL3;the separately authorized FL3a diagnosis above now supplies them. No automatic iteration/threshold change. See [FL3 report](experiments/fixed_lag_replay_fl3_20261013/REPORT.txt).

FL2b result: original stream decoder + frozen clock/model/smoother now drive a transactional pending/active/absorbed receiver, with stable-ID replacement and on-disk retirement/output archive.1004 assertions pass with byte-identical Release logs/archives, core/stream ASan/UBSan clean, and651 unchanged FL2a regressions.48 protected files unchanged.145 synthetic publications/74 retirements retain71 supports, with138 peak active IDs and212 factor slots. Online and fixed-lag-finalized paths remain distinct. Endpoint cache is explicitly off in the private candidate; source/model/Qc/priors/covariances remain unchanged. No speed or live-readiness claim. See [FL2b report](experiments/fixed_lag_lifecycle_fl2b_20261013/REPORT.txt).

FL2a result: keeping each joint marginal's original Gaussian and chart inside an explicit factor, with correct current-chart derivatives, passes the unchanged publication gate. Only the retained graph is reloaded after standard retirement; no eliminated states return. The original failing fixture remains an unchanged negative control. The candidate passes651 assertions in two byte-identical Release runs and focused ASan/UBSan. Across145 synthetic publications and74 actual retirements it retains71 supports and bounded factor-slot storage.38 protected files remain unchanged. This validates the isolated boundary adapter, not the complete receiver or its speed. See [FL2a report](experiments/fixed_lag_boundary_fl2a_20261013/REPORT.txt).

FL2 result: linear retained means/full covariance pass independent dense-reference checks. On a synthetic boundary-touching revision, standard fixed-lag iSAM2 refinement leaves normalized gradient `0.001841722` against the frozen `1e-5` threshold; the same revision without retirement passes, and an independent solve of the same frozen-prior objective passes. Failed publication rolls back. This is a fixed-linearization/publication-contract issue, not proof that fixed lag is unusable. See [FL2 correctness-stop report](experiments/fixed_lag_receiver_fl2_20261013/REPORT.txt). No threshold, prior, model or production default changed.

FL1 selects **7.0 s** using the predeclared `ceil(max required support age + 1 s)` rule: observed maxima are 5.3 s GLIM and 5.0 s Kimera. All 88,254 admitted versions remain processable with zero added losses in metadata simulation, with 71 supports after retirement (72 transiently). Source retirement is documented for all 2,101 hypothetically absorbed IDs; 74 unfinished source IDs remain retained at EOF. This is not a fixed-lag optimizer or speed result. See [FL1 report](experiments/fixed_lag_retention_fl1_20261013/REPORT.txt).

## Purpose and decision

Test a bounded-history gnssFGO receiver using the existing incremental fixed-lag smoother. Keep GLIM, Kimera, WNOA and their uncertainty models unchanged. The intended benefit is to stop optimizer work growing with total sequence length, not to make CBS-ON win an accuracy comparison.

This is a deliberate change in historical inference, not a bit-equivalent performance patch. Marginalization preserves a linearized summary of older information but gives up subsequent nonlinear reoptimization of eliminated states. Full-history iSAM2 remains the reference and rollback path. Neither exact full-history equality after marginalization nor a particular speedup is promised.

The current S5e checkpoint is `d3d152a28440f97ced3bd7a50ee072874beafe19`. Its [report](experiments/receiver_linear_completion_s5e_20261013/REPORT.txt) shows that completing the cached linear solve alone did not meet the strict publication criterion at any of 18 selected snapshots. S5c localized most late CPU time to our repeated full-history publication refinement. These findings motivate testing bounded history; they do not establish fixed-lag correctness or throughput.

## Preserve the working system

Use a separate opt-in experimental build and fresh result directory, leaving full history the default. Preserve the checkpoint, existing binaries, input journals and frozen results by hashes before and after. Do not install over production libraries or push as part of this plan.

Use E2b's observer-free replay/model as the implementation starting point, not S5e's snapshot instrumentation:

- Sources: `experiments/receiver_endpoint_reuse_e2b_20261013/`.
- Binary: `/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/build/replay`, recorded SHA256 `b4eab40dacf074cdbcd13ae23ed8175cba5bb57b0b493eec06f5b9a2de0d72d3` (reverify before use).
- Offline oracle: `/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1`.
- Runtime: `/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime`, pinned GTSAM 4.2a8. Do not substitute the other local GTSAM 4.3 tree.
- Saved journal: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv`.

That capture stopped at 148.55 s and drained to 148.58 s. It is not a completed 160 s experiment. The small historical live-versus-offline discrepancy remains unresolved; the exact numerical oracle is the retained offline replay, not the old live output.

Freeze the following:

- `source_wnoa`: pose X, world velocity V, body angular rate W; QR; independent 100 ms supports.
- No central IMU, GNSS, bias or acceleration factors. CPT7 supplies the support clock, not a central motion measurement.
- Existing WNOA/interpolation equations, Qc `[0.00375,0.00375,0.00375,0.015,0.015,0.015]`, initialization and pose/V/W prior sigmas `1e-5/10/3`.
- Full dense source covariances, Gaussian loss, scale 1; no gating, robustification, health weighting or tuning.
- Every admitted ordered source revision, stable source/session/edge identity, physical endpoint timestamps and real availability timestamps. No revision coalescing.
- Native sources and their classic `active_window_temporary` CBS policy; no source `live_revision`, source rerun or LOCAL-covariance changes.
- Existing bounded nonlinear-refinement budget and numerical stationarity threshold `1e-5`. Adapt the objective evaluation to include the marginal prior correctly; do not waive the test or loosen it to make the candidate pass.

## Reuse and actual integration boundaries

The reusable solver is [IncrementalFixedLagSmoother](online_fgo/src/solver/IncrementalFixedLagSmoother.cpp), backed by iSAM2. It already implements timestamp selection, constrained ordering and `marginalizeLeaves`; its update accepts factor removals and exposes insertion indices through `getISAM2Result()`. Reuse these operations and existing factor factories. No new optimizer or hand-written production Schur-complement engine is needed. See also the [official GTSAM fixed-lag overview](https://borglab.github.io/gtsam/fixedlagsmoother/).

Four concrete boundaries must be addressed:

1. The current private replay rejects record-time/revision-aware fixed lag, and its `ReplaceFactor`, graph audits and publication path assume a bare full-history iSAM2. Enabling the old `--solver fixed_lag` flag is insufficient.
2. The smoother normally marginalizes inside `update`, before the replay dispatches newly eligible pending beliefs. Use its existing marginalization controls or a minimal opt-in staging extension so retirement occurs only at the defined publication boundary, after those events. Preserve the default smoother entry point. Do not refresh old timestamps or use `relatedKeys` to retain an ever-growing suffix.
3. `RebuildFullHistoryQR` explicitly rejects marginalized fixed variables. `StableGaussianCost` and `CachedGaussianEvaluation` expect Gaussian `NoiseModelFactor` objects, not arbitrary linear marginal factors. Do not reuse these helpers unchanged after marginalization or remove their guards.
4. The retained CT test `RunsThroughIncrementalFixedLagSmoother` has two states at 0 and 1 s with a 2 s lag. It demonstrates insertion/optimization, not actual retirement or revision replacement across retirement. New focused tests must cover those operations.

The pinned headers explain that marginalization fixes the *linearization points* of affected surviving variables, not their physical estimates. `LinearContainerFactor` without an explicit anchor can report zero through its ordinary nonlinear `error()` interface. FL2 must verify the pinned runtime's actual representation: a successful generic graph error/gradient call is not proof that the marginal prior was evaluated correctly.

## FL1 Determine the required retention window without running an estimator

Replay only the event bookkeeping from the saved journal, preserving its recorded order. Use the existing support-binding rules, including both interpolation brackets, source/session identity, pending activation and end-of-stream handling. Do not use GT, covariance magnitude or ATE to choose the lag.

For each admitted revision e, calculate:

```
c(e) = physical time of the newest receiver support at actual dispatch
k(e) = oldest physical support time referenced by either endpoint
h(e) = c(e) - k(e)
```

Compute this for first publications and all later replacements, per source and jointly. Include pending beliefs that need a future right bracket; report first/last-revision ages, edge durations, maxima and the identities causing the maxima. Distinguish source arrival delay from receiver wall-clock queue/service delay. A delayed clock event must not advance retirement by host wall time.

Choose one conservative lag L strictly covering the maximum required support age, with a separately declared timing margin and the exact boundary convention tested on the 100 ms grid. Report the proposed value and resulting state/factor bounds before building. Do not choose 5 s merely because GLIM uses 5 s, or search lag values for better ATE.

Run a metadata-only lifecycle simulation to verify that no currently admitted version would arrive after its factor/supports were absorbed. Retain the original before-initialization/end-of-stream exclusions separately; they must not hide new lag-induced losses.

A truncated log only proves coverage for recorded versions: its last observed version is not necessarily a sender's finalized version. Check whether source retirement/terminal markers exist; do not invent them. No finite lag can guarantee acceptance under unlimited source or transport stalls. The prototype therefore has an explicit bounded-lateness contract and fails closed if that contract is violated. A future late-data policy would be a separate decision, not an automatic fallback.

FL1 deliverable: one lag recommendation, an exact lifecycle count, worst-case examples, required API/build changes and a go/no-go decision. No GLIM, Kimera or fusion execution.

## FL2 Implement the isolated receiver and prove marginalization correctness

### Factor and state lifecycle

Use the existing stable ID `(source, session, from_index, to_index)` and unchanged revision ordering. Maintain explicit pending, active and absorbed states. An active ID maps to one explicit source factor slot and its latest admitted version. Never let a recycled slot be mistaken for the old factor.

- New or revised factors are admitted only when all actual X/V/W support keys exist and the previous version, if any, is still explicit and replaceable.
- Replacement removes exactly the old factor and inserts exactly the revised mean/covariance; it does not add a second concurrent observation.
- When marginalization consumes a source factor, record its final admitted revision and retire its replaceable slot mapping. Its information now lives in the marginal prior.
- An absorbed factor cannot be revised by appending another factor, resurrecting old keys, or pretending to subtract its old nonlinear contribution from the prior. A new admissible version targeting it is a hard validation failure in this prototype.
- Preserve duplicate/stale/conflicting-payload rules while the ID is retained. Retired events outside the supported contract must fail explicitly, not silently become new IDs.
- Pending events whose required old bracket would cross the boundary must be detected before retirement. Report a retention-contract failure rather than indefinitely pinning history.

Marginalize X/V/W together by physical support age. Use the existing strict-before cutoff convention, with exact-boundary tests. Do not advance key timestamps on revision receipt. Final source factors are summarized, not deliberately dropped before receiver marginalization: the source-to-source temporary-CBS policy is distinct from this downstream fusion history policy.

### Transaction order

Preserve existing event and tie ordering; no sorting by endpoint time and no future knowledge. At each support/publication boundary:

1. Add the new independent support and unchanged WNOA factor.
2. Dispatch all beliefs already eligible at that point in the original pending/arrival order, one revision at a time.
3. Refine the currently retained graph, including any existing marginal prior, within the unchanged budget and threshold.
4. Save outgoing states at their last validated estimate; perform standard ordered marginalization and record eliminated keys, consumed factor slots and newly created marginal slots.
5. Verify the retained objective/state and bookkeeping after marginalization before committing publication and retirement output. Account for any ordering update required by the existing smoother; do not hide it in timing or change the original support cadence.

Make the boundary transactional. A rollback must restore the iSAM2 tree/delta, fixed linearization anchors, timestamp maps, marginalization flags, solver result, factor-slot ledger, pending ownership and output cursor. The superclass has an atomic flag, so do not assume an implicit smoother copy constructor suffices. Cache isolation/invalidation must cover rollback and slot reuse. No already-published output may be overwritten by a failed transaction.

### Marginal prior and strict publication

The candidate optimizes retained nonlinear factors plus a linearized boundary prior:

```
F_window(x) = sum 0.5 ||whitened source/WNOA residuals(x)||^2
            + 0.5 ||A_m delta_boundary(x; anchor_m) - b_m||^2
```

Retain the marginal's full joint information, cross terms and reference chart. Preserve fixed-anchor bookkeeping; do not re-anchor a prior as if it were a fresh independent pose measurement. Use the solver's actual coordinates and verify the chart mapping independently, including finite-difference derivatives on Pose3. Do not confuse a zero shift of the linearization anchor with a state constrained not to move.

Publication checks must include this prior exactly once and assess all retained variables, including the boundary variables. Use the same `1e-5` numerical threshold for the correctly defined retained objective. This is not a certificate for the eliminated full-history nonlinear objective. Do not apply the old graph helper to a zero-error marginal wrapper and call it stationarity.

Start with copy/rollback refinement supported by the existing fixed-lag/iSAM2 machinery. The full-history rebuild fallback remains forbidden after any marginalization. If valid refinement cannot complete under the existing budget, stop and report; do not silently publish an unchecked estimate or design another optimizer. Any marginal-aware recovery must first preserve its charts, slots and timestamp state against an independent reference.

### Focused acceptance tests

- Long-lag/no-retirement mode reproduces the full-history control under identical events, including default-path regression. Declare any unavoidable ordering differences and tight state tolerances before running; do not require bit identity across different elimination orderings.
- Linear Gaussian fixtures match independent dense QR/Schur elimination for retained means and full covariance, over repeated retirements and replacements. Compare the same linearization and information set.
- Nonlinear X/V/W fixtures match a separately assembled retained graph with the same frozen marginal prior. Check objective, chart-correct gradient and covariance; include substantial orientation changes, not only scalar examples.
- Mean and full-covariance revisions immediately before retirement; newer revisions after absorption; out-of-order/duplicate/conflicting events; distinct source/session IDs; reused slots.
- Exact endpoint, one/two interpolated endpoints and shared brackets; pending right bracket; cutoff ties; source stall/reset; drain and truncated stream.
- Failure during update, refinement or marginalization causes complete rollback and no output mutation. Deterministic repeat and bounded cache/ledger retirement are verified.
- Release suite and focused ASan/UBSan; one consistent header/runtime definition across translation units. Compile only the minimal existing solver dependencies needed by the private build; no dependency upgrade.

Predeclare concrete absolute/relative tolerances per synthetic quantity in the FL2 protocol before results. Structural accounting, payload identity and frozen input hashes are exact. Do not use full-history nonlinear equality after retirement as the marginalization oracle; it is a different objective.

FL2 ends with a correctness report. No real source replay or live capture is part of this stage.

## FL3 Replay the same retained data and measure the complete cost

After FL2 passes, use the single frozen lag from FL1. First replay a prefix long enough to contain several actual retirements, then the complete saved 148.58 s journal. Do not claim a fixed-lag test if the prefix is shorter than L. Use a fresh result root and the same ordered inputs, model, covariance, runtime, CPU allocation and diagnostics in each comparison.

No GLIM/Kimera rerun is needed. Reuse the existing full-history results for correctness, and use a limited paired full-history/fixed-lag timing comparison when authorized to control machine load. No profiler, visualization or extra dense covariance queries in the timing path; run expensive diagnostics separately.

Report:

- Exact input/version accounting, original exclusions and zero additional lag losses; active/replaced/absorbed IDs and marginalization generations. No duplicates or factor resurrection.
- Active X/V/W count, explicit source/motion factors, marginal dimension/density, factor-slot high-water mark, pending count, cache size and RSS through early/middle/late sections.
- Total service time per support, including intervening revision processing, refinement, checks, marginalization, copying and required output. Separately show phase median/p95/max and refinement counts.
- Service-rate/backlog against the recorded 0.1x arrival schedule. Offline queue simulation is not a live-capacity claim.
- Common-time online p/orientation/V/W differences and selected retained covariance differences against full history; retirement-state differences reported separately. Repeat the fixed-lag replay deterministically.
- Existing ATE and 1 s RPE only under the same GT/frame/alignment/mask, with coverage stated. Accuracy is descriptive here; do not change L/Qc/covariance based on it or require CBS-ON to beat OFF.

Proposed operational target: p95 total support service at most 0.8 s against the 1.0 s wall interval at 0.1x, with mean service below arrival budget in late bins and no accumulating queue. Keep the existing queue safety cap. This is a proposed headroom target, not a hard real-time guarantee; freeze the precise timing protocol before execution. Marginal fill-in, factor density and arrivals may still cause spikes even with bounded state count.

Retire old hot-path endpoint/pose/ledger storage as well as solver states. Keep trajectory and audit history append-only on disk, not as an all-history structure scanned/copied every update. If some archival memory remains unbounded, report it rather than claiming bounded end-to-end RAM. Do not introduce revision dropping or a second logging redesign to conceal that limitation.

Stop FL3 on any information loss, invalid marginal prior, failed publication, unresolved bookkeeping error, or missed operational target. Preserve results and decide the next action explicitly; do not automatically enlarge lag, queue, iteration budget or tolerances.

## FL4 Bounded live confirmation after replay acceptance

Only with new authorization: reuse the latest validated ETH1 source profiles and private GLIM provider, headless at 0.1x. Keep the native sources and classic CBS unchanged. Choose a duration covering multiple lag turnovers; a subsequent 160 s capacity attempt must be a fresh explicitly authorized run, not an automatic retry of S5.

Validate actual arrival/processing delays, native coverage, complete conservation, publication correctness and repeat of the saved journal using the same fixed-lag build. Do not waive the old live/offline mismatch; distinguish reproducible offline results from live capture validation. No Rerun load until after capture. Matched OFF/ON experiments follow only after this engineering gate, not as a prerequisite for each code patch.

## Output and visualization contract

Save two distinct fused products:

1. **Online/as-played:** the current support estimate actually published at each boundary, never retrospectively rewritten.
2. **Fixed-lag finalized:** each old state at retirement, followed at EOF by the validated remaining active window. Record each state's finalization availability time. It is not a globally retrospectively optimized trajectory.

Keep the original/native GLIM and native Kimera trajectories as the source outputs. A full-history final graph remains a separate diagnostic reference. Store historical graph geometry/events for later Rerun playback; retaining a whole visualization does not require retaining all optimization variables.

## Decisions and stop points

| Stage | Question answered | Required handoff |
| --- | --- | --- |
| FL0 | Is there a concrete safe plan? | This plan; no estimator changes |
| FL1 | Can one bounded lag cover the recorded revision lifecycle? | Lag, exact lifecycle simulation and implementation surface |
| FL2 | Does the candidate preserve correct information through retirement? | Synthetic/dense-reference correctness, default regression and sanitizer results |
| FL3 | Does it work and bound cost on the retained real stream? | Conservation, stationarity, state differences and whole-service timing |
| FL4 | Does the validated candidate keep up while sources run? | Separately authorized live capacity and saved-journal validation |

A successful FL3 would be a bounded-data engineering result, not proof of indefinite real-time operation or covariance calibration. FL3 stopped at its correctness gate: the20s prefix passes but the full attempt fails at122.4s. FL3a diagnosed the failure;FL3b's one adaptive policy failed and was not adopted. FL3c's recursive batch reference failed at step2 and gave no convergence conclusion. FL3d restored actual iSAM2 and passed the single pre-retirement solve at116steps. Subsequently authorized FL3e passes that complete transaction at125totalsteps/1.358s, using a target-only200ceiling;retirement and checked publication now pass for THIS update. No complete saved replay or production integration has passed. No automatic continuation, retry, sweep or production-policy change. FL1's7.0s lag and all model/settings remain frozen;FL2a/FL2b synthetic gates remain their prior evidence. Stop for the next decision. Full history remains default/reference;FL4 remains on hold.

Subsequent FL3f tested bounded ordinary incremental refinement instead of a
higher forced-refinement ceiling. It passed structural checks but failed the
predeclared raw-state and boundary-prior fidelity screens,so it is rejected.
The unresolved issue is fast,reliable boundary refinement,not source payload
loss or growing retained state count. No blanket conclusion about every future
sequence or alternative optimizer is established. No follow-up authorized here.
