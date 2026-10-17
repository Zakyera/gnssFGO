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

**FL4v COMMON-PREFIX SCORING COMPLETE (2026-10-17):** User-requested OFF/ON
table from retained FL4t OFF and FL4v ON; no estimator rerun or model change.
Frozen evaluator/IE-TC reference, identical1055 timestamps spanning105.39990s
(1727775011.5999973..1727775116.9998968),1045one-second RPE pairs. OFF->ON
ATE m: nativeGLIM .041708->.025926, nativeKimera .687756->.235494,
LIVE fusion .065905->.050717 (23.05% lower). Fusion1s t-RPE .032071->.035051m,
r-RPE1.270812->1.562678deg: worse local errors despite better ATE. OFF remains
a failed120s capture; prefix scores do NOT certify completion or stationarity.
Startup gap and unavailable OFF tail excluded, not labeled full120s. Delayed
finalized outputs omitted because ON has later future support. Inputs
unchanged; no new estimator run. Evidence:
`experiments/fixed_lag_live_on_fl4v_20261017/common_prefix_off_on.json`;
script beside the FL4v prepared run root: `common_prefix_off_on.py`.

**FL4v STANDALONE120S ON COMPLETE (2026-10-17):** Explicit user-authorized
ETH1 ON despite incomplete FL4t OFF; exact FL4t/FL4s binaries/profiles/guards,
not FL4u instrumentation. Three estimators live at0.10x; classic temporary CBS,
unchanged7s no-central-IMU source-WNOA.1191GLIM/584Kimera/1193fusion poses span
119.00/119.60/119.20s; Kimera startup3.20s gap separately disclosed.580G->K/
581K->G injections; all584accepted steps non-increasing,76existing LM fallbacks,
66cap hits; max cache/fresh0,tree1.07002e-9. No recovery/failure/retry, but no
stationarity certificate or universal robustness claim.70756belief versions
accounted:1770insert+68961replace+25preinit, pending/unsupported0,covariance
error0,71supports/175slots. All1193online outputs live; peak queue42;
support p95 .080328s/max .114025s. Saved-journal repeat byte-identical for
numeric products and73113archive files. Full-saved IE-TC ATE m: nativeGLIM
.026277,nativeKimera .270672,onlinefusion .058737,DELAYED finalized .055768.
All saved rows scored; common115.90s mask separately reported. Finalized1s
RPE .016239m/.341362deg vs online .035396m/1.615035deg. IE-TC sharesCPT7,
not independent prism. Fusion does not beat nativeGLIM ATE. No matched120s
OFF/ON claim: FL4t OFF failure remains unresolved.1131protected hashes and
30profile files unchanged; containers restored stopped; no tuning/model/code
change/default promotion/commit/push; HEAD67c00392 unchanged.
Report: `experiments/fixed_lag_live_on_fl4v_20261017/REPORT.txt`.

**FL4u DIAGNOSTIC COMPLETE; CONVERGENCE NOT CERTIFIED (2026-10-17):** One
Kimera-only OFF capture at0.10x reproduced update531's original failure. All531
preceding native messages byte-identical to FL4t; objective/acceptance rows
exact excluding wall times. Ordinary proposal raises cost2155.442364->2156.901108.
LM returns unchanged after ONE outer iteration at lambda0.1; wrapper's no-change
break then strict-decrease failure is directly observed, not cap exhaustion.
Only measured downhill probe improves1.446e-8, below existing2.156e-7 tolerance;
no sufficient descent or full stationarity certificate. Analytic directional
slope is6.274x the observed finite-difference slope, limiting gradient-based
certification; factor/cause not localized. Do not accept unchanged states or
increase iteration caps on this evidence. Release/focusedASan/UBSan pass;
1084protected/40frozen diagnostic entries unchanged, both containers restored
stopped. No GLIM/fuser/ON, retry, covariance/model/acceptance repair or promotion.
HEAD67c00392 unchanged. Protocol, results and limits:
`experiments/fixed_lag_stopping_fl4u_20261017/REPORT.txt`.

**FL4t120S LIVE GATE FAILED (2026-10-16):** Exact FL4s binaries/profiles,
0.10x,temporary classic CBS,native mean alignment,unchanged7s source-WNOA
receiver without central IMU. OFF stops near109.38s captured input at Kimera
update531: `FL4Q no decreasing current-chart LM step`; backend returns no
output. ON NOT STARTED, no retry/tuning. Accepted0..530 all non-increasing;
17fallbacks,1cap hit; max cache/fresh0,tree4.84114e-13. Failed-step cost/iteration
not retained; cannot claim iteration exhaustion or root cause from the guard.
Adjacent stereo tracking failure is evidence, not a causal conclusion.
1087GLIM/531Kimera/1089fusion saved poses,~109s,NOT completed120s. Fusion queue0
at last monitor,max48; partial journal drains64391beliefs with pending0,
unsupported0,71supports/175slots. Its complete flag does not certify capture.
No ATE/RPE,matched covariance,deterministic repeat or full-window pass claimed.
All1059protected hashes and FL4s sensor profiles unchanged; stopped containers
restored; existing dirty work preserved,HEAD67c00392 unchanged,no commit/push.
Report:`experiments/fixed_lag_live_fl4t_20261016/REPORT.txt`.

**FL4s MATCHED30S LIVE GATE PASSED (2026-10-16):** Exact FL4q Kimera
candidate in BOTH ETH1 OFF/ON captures, all three estimators concurrent at0.10x;
unchanged shared-native GLIM, FL4e7s fixed-lag source-WNOA receiver, no central
IMU or covariance tuning. Real native temporal mean alignment enabled;
classic temporary CBS, not live_revision. Each291GLIM/134Kimera/293fusion
poses; all134accepted Kimera decisions non-increasing in each mode. OFF1
fallback(44),ON3(53/63/87),ON2cap hits. No rollback/recovery; bounded descent,
not stationarity. ON130G->K/131K->G injections,106Kimera retirements. Both
receiver journals repeat byte-identically including committed archives.
Covariance roundtrip error0; no pending/unsupported; max71supports/175slots;
full-support p95 .080896s OFF/.087867s ON. All293outputs published live.
Full-saved IE-TC ATE OFF->ON m: native GLIM .057256->.050283; native Kimera
.087878->.043791; fusion online .057332->.054424; finalized .053795->.037504.
Kimera/fusion1s RPE WORSENS: mixed accuracy, not universal improvement.
Every saved pose scored; explicit initial3.2sKimera gap; separate common
260timestamp/25.9s mask, not mislabeled30s. IE-TC sharesCPT7,not prism GT.
Initial setup stopped before any playback on a stale pinned wrapper PATH;
preserved, exact path registration corrected in freshFL4s2 root, no failed
data rerun or solver change.990protected hashes unchanged, containers restored
stopped; priorRelease/ASan/UBSan reused by hash. Existing dirty work preserved;
HEAD67c00392 unchanged,no commit/push/promotion. Longer live gate unrun.
Report:`experiments/fixed_lag_live_fl4s_20261016/REPORT.txt`.

**FL4r LATE-WINDOW GATE PASSED (2026-10-16):** One94s Kimera-only replay
with the unchanged FL4q DSO completes454 native poses/93.59949s;450 exact
incoming wire messages/update bindings/full covariances;426 logged retirements.
All134 native messages and objective/acceptance records from FL4q reproduce
exactly (timing excluded).29 harmful proposals replaced;425 ordinary accepted;
all454 accepted decisions non-increasing on their same post-removal graph.
At candidate update438: before1,154,442.12, native proposal15,667,368.27,
accepted17,283.57. It has a different preceding trajectory from baseline438,
not a same-graph comparison against the old44.7M→107.1M result.21fallbacks hit
10iterations; descent only, not stationarity, and late fallbacks are frequent.
Cache/fresh H/g exact; max relative tree error4.977e-9 passes frozen1e-8 gate
(larger than30s gate). Decision-stage median8.03ms/p95159.94ms/max265.79ms;
excludes rebuild/native update/copy/marginalization/export. Wall972.29s at0.10x,
not1x certification. No recovery, rollback, retry, ATE, GLIM/fuser or tuning.
879baseline/37FL4q/30FL4r hashes unchanged; container restored stopped.
Prior Release/ASan/UBSan reused, not rerun; production/defaults unmodified.
HEAD67c00392 unchanged, no commit/push. Stop here; live bidirectional accuracy
and throughput remain untested. [Report](experiments/fixed_lag_current_chart_fl4r_20261016/REPORT.txt).

**FL4q SHORT-WINDOW CANDIDATE PASSED (2026-10-16):** User-authorized private
current-estimate LM fallback preserves immutable original-chart Gaussian priors
and rebuilds the undamped ISAM2 core at the accepted estimate, preserving sparse
factor slots/native pointers/CBS metadata. Release and focused ASan/UBSan pass:
dense covariance, next update, marginalization, temporary-CBS removal, cached
LOCAL/full transaction rollback and deterministic fixture repeats. One30s
Kimera-only frozen-input ETH1 capture completes134poses/29.60s,130exact incoming
payloads/update bindings/covariances,106retirements. Native0..88 byte-identical
to FL4n. Three harmful proposals replaced at89/101/113; all134accepted steps
non-increasing in same objective. Update89 cost373023.81→native409915.52,
candidate79697.75. Cache/fresh H/g exact, tree relative error≤2.28e-13. Fallback
decision times.129/.180/.174s EXCLUDE rebuild/checks; not total backend timing.
Two fallbacks hit10iterations: descent only, not stationarity. No ATE, full
sequence, GLIM/fusion run, covariance tuning or production promotion. All879
protected/37candidate hashes unchanged; container restored stopped. Branch
`engineering/fl4q-current-chart-fallback-20261016`, HEAD67c00392 unchanged;
no commit/push. Longer SAME frozen prefix through late failure remains unrun.
[FL4q report](experiments/fixed_lag_current_chart_fl4q_20261016/REPORT.txt).

**FL4p DIRECTION AUDIT COMPLETE (2026-10-16):** One23s frozen-input
Kimera-only prefix reproduces99 native messages/22.60s and objective rows
exactly;95 incoming payloads/bindings/covariances exact. At unchanged update89,
native direction initially descends but full step overshoots: cost373023.81
to409915.52; quarter step366105.97. Cached slope-128306.54 versus measured
-49129.34. Native IMU explains99.93% of signed slope discrepancy; boundary
factor x63-v63-x64-v64-b63 alone98.63% (cached-40.31 vs actual+78050.06;
fresh current-point Jacobian agrees).16/27 pose anchors fixed, max3.06deg/
.219m anchor-to-estimate offset. Forced solve does not override fixed-anchor
relinearization exclusion. No tiny-step visual validity switch; reset-clone
effect<=5.88 cost units, not the dominant cause. Release/focused ASan/UBSan
and exact nonmutation pass;879protected/32candidate hashes unchanged;
container restored stopped. Local model/overshoot identified, NOT all later
failures or a production repair. No solver/covariance/policy change, GLIM/fuser,
retry, commit or push. Report and proposed original-chart current-estimate
solve boundary: `experiments/fixed_lag_direction_fl4p_20261016/REPORT.txt`.

**FL4o CANDIDATE STOPPED; REPAIR INCOMPLETE (2026-10-16):** Private nonlinear
step acceptance with original-chart prior cost and16-halving bound passes
Release and focused ABI-matched ASan/UBSan synthetic checks: accepted-delta
persistence through reads/next update/marginalization, unchanged Gaussian
equations/covariance, smart-cache pointer-preserving rollback, whole-wrapper
failure atomicity, exact fast-path parity and deterministic repeat. One94s
Kimera-only frozen-input replay was attempted and STOPPED at update108.
Shortening89/101/103 reduces their costs; at108 before1,532,262.188827,
full1,933,392.808644, best trial1,532,262.570811: no tested decrease. Transaction
rolled back; no native108 output.105 exact frozen inputs/bindings/covariances
received;108 native poses span24.3999s, last at bag25.5553s;80 retirements.
First native change89; no other recovery/drop. NOT a completed94s experiment,
accuracy claim or production fix. All879 protected and31 candidate hashes
unchanged; container restored stopped. No automatic LM/covariance/policy change
or retry. A reliable original-chart descent direction remains unresolved; do
not merely lengthen the run or relax acceptance. See
[FL4o report](experiments/fixed_lag_step_acceptance_fl4o_20261016/REPORT.txt).

**FL4n LINEAR-SYSTEM GATE COMPLETE (2026-10-15 UTC/16 local):** One private
Kimera-only frozen-input94s replay captures28/89/438 before marginalization.
Cached and fresh same-theta H/g are EXACTLY equal, including every factor;
tree relative H/g discrepancy <=3.824e-16; actual scaled equation residual
<=1.187e-15. No cached orphan/missing factor or surviving retired identity.
Nevertheless update438 repeats cost44.711M→107.129M. All454 native messages
(excluding receipt time) and all objective rows exactly reproduce FL4m remove;
450exact inputs/bindings,426retirements,83cost increases. Scaled H condition
~2.05e11 at438 and25/27 pose linearization points fixed; this is not a forward-
accuracy/convergence certificate. Synthetic parity/removal/nonmutation and
deliberate-corruption tests pass;879protected hashes unchanged; container
restored stopped. Select original-chart nonlinear step acceptance/bounded
numerical fallback, NOT a guessed cache/removal patch. Behavioral repair remains
unimplemented; no tuning, GLIM/fusion run or production promotion. See
[FL4n report](experiments/fixed_lag_linear_parity_fl4n_20261015/REPORT.txt).

**FL4m RETIREMENT MECHANISM IMPLICATED (2026-10-15):** Two94s Kimera-only
0.10x captures use450 identical frozen post-alignment FL4l GLIM factors. Private
switch changes only scheduled retirement, preserving current active-window
admission/solve/recovery (NOT historical persistent mode). First28 native outputs
and objectives are exactly equal before retirement. At update438, remove case
reproduces target error9.38→134.07mm and cost44.71M→107.13M; retain-through-
marginalization gives6.55→2.25mm and4.21M→0.606M. Cost-increasing steps83vs1;
late native~0.2s translation-error RMS29.23vs5.34mm. Each has454poses/93.60s,
all450 exact payload/binding/update checks,426retirements,zero rollbacks. Runtime-
API synthetic lifecycle/reference checks pass;879protected hashes unchanged.
Container restored stopped. This implicates retirement/accumulated prior history,
not uniquely a code defect or justification to inherit external information.
No GLIM/fusion, tuning, production promotion or follow-up repair. See
[FL4m report](experiments/fixed_lag_retirement_fl4m_20261015/REPORT.txt).

**FL4l SAME-GRAPH BAD STEP CONFIRMED (2026-10-15):** One private94s ETH1
source-only CBS-ON prefix captured update438 with preceding history. Same
original-chart objective rises44.712M→107.129M in the accepted production
step; cloned reference LM reduces it to16,772.87 without changing any factor,
covariance or marginal prior. Target relative translation error rises9.38→134.07mm
in production, versus0.71mm in the reference; this reproduces harmful deformation,
not the historical exact432mm magnitude. Native source-IMU cost dominates the
increase;25% of the production step decreases cost. Reference hit30-iteration
cap, scaled gradient0.01045; no convergence/trajectory-performance claim.
Prior-chart equality and synthetic derivative test pass. All879 protected
paths/profile trees unchanged, container restored stopped. Initial startup
path mismatch stopped before playback and is retained separately; actual capture
completed once. No fusion, production fix, weighting/tuning or binary promotion.
Full nonlinear graph was solved in process, not archived for standalone replay.
Next proposed gate is a transactional original-chart objective safeguard and
bounded numerical fallback, not covariance changes. See
[FL4l report](experiments/fixed_lag_live_fl4l_20261015/REPORT.txt).

**FL4k DIAGNOSTIC GATE COMPLETE (2026-10-15):** Corrected only the optional
Kimera/BPSAM shadow-objective diagnostic: unanchored Gaussian priors are now
evaluated in the owning iSAM2 theta chart; invalid chart/type/dimension/cost
fails closed. Release 2/2 synthetic tests pass, including actual marginalization
and nine wrapper updates with exactly unchanged states/per-key covariances when
the diagnostic is enabled. Dense reference cost 3.1645055780930065 was previously
reported as 0. Deterministic repeats match. Address/UB checks pass; wrapper
LeakSanitizer reports 73 bytes in unchanged-library allocations (explicit limit,
not a clean full sanitizer claim). All 879 FL4g protected paths unchanged.
No production binary rebuild, source capture, replay, covariance or solver
policy change. CBS source patch remains uncommitted on 7c3a1e6. Scalar objective
repair does NOT establish the cause or repair ETH1 failure; original-chart
same-graph capture/reference solve remains next, unexecuted. See
[FL4k report](experiments/fixed_lag_live_fl4k_20261015/REPORT.txt) and retained
synthetic receipt under `runs/fixed_lag_objective_fl4k_20261015/receipt`.

**FL4j BOUNDED MECHANISM AUDIT COMPLETE (2026-10-15):** New retained-data
finding: multi-view visual factors touching the incoming Kimera edge supply
nonzero diagnostic endpoint information for187/300 at28.955s,11/228 at96.955s,
0/204 at105.755s and0/193 at110.355s. These are fresh evaluations at the current
estimate, NOT the actual cached iSAM system; exact triangulation rejection
reason and cause/consequence remain unknown. Confirmed diagnostic defect:
the existing disabled shadow objective sums unanchored LinearContainerFactor
error, which returns0, omitting the marginal prior; fresh cross-family
gradient diagnostics also mix prior/current charts. This is not a production
root-cause claim. Existing residual rollback discards18 incoming GLIM factors
with reference-error median7.08mm; a numerical failure does not establish a
bad sender. Recommend ONE original-chart same-graph source-solver decision
gate, not covariance tuning, repeated sequence runs or a blind provider
rollback. No production edit, estimator run, build, container start or tuning.
See [FL4j report](experiments/fixed_lag_live_fl4j_20261015/REPORT.txt) and its
reproducible30-snapshot evidence. Proposed diagnostic repair/capture remains
unexecuted; unique initiating cause is not yet isolated.

**FL4i LAUNCH/SOURCE-LINEAGE AUDIT COMPLETE (2026-10-15):** FL4g effective
settings/argv/profile bytes/mapped core libraries match FL4f; no accidental
120s launch switch found. They are NOT historical persistent CBS: temporary
factors, native-IMU alignment and GLIM covariance history changed. On the exact
115.9s mask, Oct5 persistent ON native Kimera ATE.375084m; Oct11 independent
LOCAL temporary/aligned ON already2.849717m; latest shared-native ON3.745897m.
The failure predates the shared-Jacobian provider. Native ON GLIM instead
improves.036950→.034628→.025248m; matched wire relative translation RMS
.024973→.011122→.009544m. Latest/Oct11 translation covariance median ratio1.0036,
not collapse. OFF Kimera poses/timestamps are exactly unchanged Oct5→now;
OFF GLIM ATE worsens.030474→.044372m and is not claimed identical. Exact
source-side causal change remains unisolated; do not convict temporary CBS or
the provider from coupled runs. No estimator/receiver run, fixes or tuning.
See [FL4i](experiments/fixed_lag_live_fl4i_20261015/REPORT.md). Further FL4h
solver experiments remain unexecuted; next question is the earlier source-CBS
semantic transition, not a blind speed-work rollback.

**FL4h SAVED-SOURCE AUDIT COMPLETE (2026-10-15):** All580 G→K constructed
payloads/covariances/timestamps match wire; aligned GLIM relative translation
RMS9.81mm/max32.09mm. Kimera nevertheless develops source-graph deformation:
at update438 /91.755s, target-edge reference error16.08→432.48mm while the
aligned GLIM error is6.45mm. Saved-pose residuals reproduce the pre-marginal
solver logs within6.7e-13, localizing this jump to optimization, not immediate
retirement or output formatting. One-step Gauss–Newton plus a marginal prior
that can freeze25 of26 active pose linearization points is a concrete next
hypothesis, NOT a proved unique cause. Native IMU factor costs grow sharply.
First recovery at100.555s is later than onset:580 constructions,18 undone,
562 retained admissions;533 normal+11 guarded old removals,18 left.22 nested
rollback transactions reconcile. Corrected FL4g's construction/admission
wording. Report: [FL4h](experiments/fixed_lag_live_fl4h_20261015/REPORT.md).
No new source/fusion capture, estimator edit, covariance change or tuning.
Exact same-objective prior-chart diagnostic remains proposed, not executed.

**FL4g MATCHED LIVE 120 s COMPLETE (2026-10-15):** The matched ETH1 first-120 s
CBS-OFF and classic CBS-ON extension of FL4f completed at 0.10× with the same
FL4e receiver SHA, 7 s source-WNOA fixed lag, profiles, Gaussian full source
covariances, and no central IMU factor. Both source/receiver cases passed
complete coverage, exact belief accounting and byte-identical deterministic
receiver replay. ON constructed580 G→K candidates (FL4h:562 retained after
recovery) and581 K→G source factors; OFF injected none. On 1,160 identical
timestamps spanning115.9s, native GLIM ATE improves
`0.044372→0.025248 m`, but native Kimera ATE deteriorates
`0.458070→3.745897 m`; fused **online/as-played** ATE worsens
`0.055480→0.131993 m` and retrospective finalized ATE worsens
`0.052336→0.127574 m`. Independent prism partial coverage corroborates the
direction. On matched final edge revisions, Kimera's ON covariance trace
usually **increases**, not collapses. The underlying cause is not established;
no settings were tuned. See [FL4g protocol](experiments/fixed_lag_live_fl4g_20261015/PROTOCOL.md)
and [FL4g report](experiments/fixed_lag_live_fl4g_20261015/REPORT.md). The
full-window IE-TC reference shares CPT7; prism covers only about 65 s. This
certifies neither 1× throughput nor nonlinear stationarity.


**FL4f GIT MILESTONE ARCHIVE (2026-10-14):** The fixed-lag engineering
experiment lineage, FL4e isolated receiver repair, and FL4f matched live
result are saved with the [FL4f milestone manifest](experiments/fixed_lag_live_fl4f_20261014/MILESTONE.md).
That manifest includes byte-checked effective source/receiver settings, ETH1
GLIM/Kimera profile trees, compact score/covariance evidence, and explicit
limits of what is *not* archived in Git. This is archival only: no estimator
change, new run, binary promotion, tuning, or claim beyond the bounded pilot.


**FL4f MATCHED LIVE 30 s COMPLETE (2026-10-14):** Fresh ETH1 CBS-OFF and
classic CBS-ON, both with FL4e receiver SHA `53c96a2b...`, completed at 0.10×
in a fresh root. The first two setup attempts stopped **before playback**
(Docker permission; omitted copied source-library overlay) and remain
preserved. Both successful source/receiver cases passed exact conservation and
byte-identical saved-journal replay. ON injected 130 G→K and 130 K→G factors;
OFF injected none. On 260 identical timestamps over 25.9 s after Kimera's
startup gap, native GLIM ATE `0.044864→0.020256 m`, native Kimera ATE
`0.060388→0.042746 m`, and fused **online/as-played** ATE
`0.051990→0.033126 m`. Complete saved-output scores over each 29.0–29.6 s
span and the separate retrospectively fixed-lag-finalized scores are in the
[FL4f report](experiments/fixed_lag_live_fl4f_20261014/REPORT.md). Final
matched-edge GLIM covariance traces mostly contract under ON, but Kimera's
translation trace usually increases; no shared scale-collapse claim or tuning.
The IE-TC reference shares CPT7; independent prism covers only a partial
window. FL3i whole-replay numerical and nonlinear-stationarity limitations
remain, as does unproven longer-run/1× capacity. No new model/sequence run or
code commit in FL4f.

**FL4e SAVED-DATA GATE PASSED; NOT LIVE-PROMOTED (2026-10-14):** Original and
stock-slot receivers both reproduce FL4d's missing `x71` at event8129. After
event8128 retirement, `x71` is absent from graph/theta/index/node map but
remains in a surviving Bayes-tree clique. The private candidate adds retiring
keys themselves to constrained extra re-elimination, not just descendants;
it leaves zero stale `x71` clique references and processes all8148 retained
ON events. Complete OFF journal passes with original counts and only small
ordering-level state differences (online position RMS1.01e-7m; finalized
RMS1.55e-6m), not byte-identical to the old binary. Release and ASan/UBSan
synthetic lifecycle gates pass50,818 checks; OFF and partial ON repeats are
byte-identical;814protected files unchanged. No fresh live run or GT/ON
accuracy comparison. A future matched live comparison must use the SAME
candidate binary in OFF and ON, not old FL4c OFF versus candidate ON. Existing
FL3i numerical/stationarity limits remain. See [FL4e report](experiments/fixed_lag_failure_fl4e_20261014/REPORT.md).

**FL4d STOPPED ON RECEIVER FAILURE (2026-10-14):** The predeclared matched
ETH1 first-30s CBS-OFF/ON comparison reused validated FL4c OFF and launched
one fresh classic ON capture with matched profiles, pinned executable and
receiver settings. Static and effective before-playback checks passed; all
814 protected artifacts remained unchanged. ON failed after approximately
14.78s at receiver `FL4A_STREAM_FAILURE Requested non-existent variable from
VariableIndex: 120 71` (`x71`). Last successful event8128 retired index71;
the next journal entry was a Kimera revision, but its causality is not proven.
Maximum transport queue was21 with zero queued in the latest snapshot, so this
is not evidence of a transport backlog. The source/controller aborted. No
valid ON 30s result, GT metric, covariance comparison or deterministic repeat;
no automatic retry or tuning. FL3i numerical and nonlinear-stationarity limits
also remain. See [FL4d protocol](experiments/fixed_lag_live_fl4d_20261014/PROTOCOL.md)
and [failure report](experiments/fixed_lag_live_fl4d_20261014/REPORT.md).

**FL4c BOUNDED LIVE CBS-OFF PILOT PASSED (2026-10-14):** After the FL4a
provider-mode startup mismatch and an FL4b copied-profile path error (both
before bag playback), a fresh ETH1 first30s headless capture completed at
0.10x. GLIM's required ActiveWindowTemporary provider bookkeeping was used
with actual CBS belief reception disabled in both sources; zero cross-source
injections and no GLIM incoming subscription were checked before playback.
Native GLIM covered29.00s; native Kimera spanned29.60s but had a3.20s startup
gap. The fixed-lag WNOA receiver published293as-played supports over29.2s,
accounted for all15,884 source versions (420insert/15,439replace/25before
initialization), and ended with no pending events. p95 whole-support service
64.41ms, max queue32, active support bound71. Saved-journal repeat in the same
receiver container matched16,365 commit files and all numerical trajectory/
summary products byte-for-byte.814 protected inputs unchanged. This is a
bounded operational result only: FL3i's strict whole-replay numerical failure
is unchanged, fixed-lag stationarity is uncertified, no GT/ON result or1x/
long-run capacity claim. See [FL4c report](experiments/fixed_lag_live_fl4c_20261014/REPORT.txt).

**FL4a STOPPED BEFORE PLAYBACK (2026-10-14):** Thin live wrapper passes
saved-prefix equivalence:10,503archive files byte-identical,200ROS-compatible
pose rows. Private adapter explicitly admits incremental fixed-lag policy;
production unchanged. One30s ETH1 OFF source launch then fails at GLIM startup:
S4d shared_native provider requires ActiveWindowTemporary, but OFF runner
selects observe_only. Exact guard at release_2/overlay/cbs_bridge.cpp:684-687.
No bag played;0IMU/0beliefs/0fusion poses. Scoped processes stopped;791protected
entries unchanged. No retry, covariance fallback, source rebuild, CBS-ON or
solver changes. Fix ONLY this launch/provider contract before a fresh pilot;
not evidence of a fixed-lag runtime/convergence failure. Earlier FL3i strict
failure remains unchanged. See [FL4a report](experiments/fixed_lag_live_fl4a_20261014/REPORT.txt).

**FL3j COMPLETE — SAME-INPUT BOOKKEEPING PASS (2026-10-14):** One prefix
reconstruction stops at event70350/support1018. Fork identical pre-retirement
ISAM2 into stock and slot-reuse copies:170surviving factor pointers/slots,
removed keys and fixed anchors exact; new marginal and canonical H/g/constant
differences0. Immediate state difference3.47e-18; native slot-order augmented
information difference1.03e-17.62,043prefix files byte-identical to FL3i;
118,986protected files unchanged. No bookkeeping defect found at this target;
round-off accumulation is supported but its entire causal path not proved.
Stop micro-difference investigation. Recommend a short experimental live pilot,
not production-equivalence certification. FL3i strict gate remains FAILED;
no tolerance change, source/live run or automatic promotion. No new model.
See [FL3j report](experiments/fixed_lag_slot_audit_fl3j_20261014/REPORT.txt).

**FL3i COMPLETE — SLOT GROWTH FIXED, NUMERICAL GATE FAILED (2026-10-14):**
Private pinned marginalization copy preserves elimination verbatim; final
bookkeeping reuses vacant slots and updates cache/VariableIndex. 50,818
Release/repeat/ASan-UBSan checks pass; 1,029 synthetic retirements retain 13
slots versus stock 1,042 with exact state/covariance. Nonlinear WNOA state
max1.49e-12/covariance relative1.69e-11. Repeated20s prefix byte-identical.
ONE148.58s capture preserves all2,201IDs/86,053revisions and exact covariances;
slot extent stays175 versus1,571, activefactors<=172/supports71. Runtime
66.550s versus66.446s: no demonstrated speedup. Full FL3h state-equivalence
gate FAILS: as-played position RMS.01843mm/max.27943mm; finalized
RMS.02100mm/max.17217mm (NOT GT errors). First as-played failure support1018,
orientation1.787e-8 versus1e-8. Changed factor-index/assembly order is a
plausible numerical cause, not proven exact mechanism. No threshold relaxation,
second full replay, model change or promotion. 505protectedfiles unchanged;
base/finalHEADd3d152a, private work uncommitted. FL3h/production preserved;
FL4 still held. See [FL3i report](experiments/fixed_lag_slots_fl3i_20261014/REPORT.txt).

**FL3h COMPLETE — STOCK SAVED-REPLAY GATE PASSED (2026-10-14):** Separate
native-style incremental fixed lag from initialization, not early stopping
inside FL2a's chart-reloaded history. Authored smoother plus read-only outgoing
state observer; raw marginal fixed linearization points, ordinary incremental
updates, no chart conversion/reload or full nonlinear publication certificate.
1,193Release/repeat/ASan-UBSan checks pass; stock-reference state/covariance0;
20s prefix archives byte-identical. ONE complete148.58s capture succeeds in
66.446s:2,201IDs/86,053replacements, exact ordered source/payload accounting,
1,481publications,71retained supports. As-played position difference from
full-history RMS.354mm/max2.701mm; finalized versus full-history-final
RMS.393mm/max1.894mm. Not GT errors or exact posterior/stationarity equivalence.
Whole100ms-cycle service mean44.812ms/p9593.628ms/max132.402ms;0.1x simulated
queue peak49events/max75.752ms and drains. Former problem event1.490ms rather
than1.358s/125steps, but different histories/objectives, not equal-solution
speedup (retained position differs from old FL3e by max.428m). Fresh copied
gradients1.14–4.13 are explicitly not converged at1e-5. Active factors<=172,
RSS25MiB; slot extent159->1571 grows, so unbounded-duration storage/cost NOT
certified. No compaction, production promotion, live/source run or tuning.
433protected files unchanged; base/finalHEADd3d152a, private work uncommitted.
Old strict failures/acceptance remain unchanged;FL4 not automatically authorized.
See [FL3h report](experiments/fixed_lag_native_fl3h_20261014/REPORT.txt).

**FL3g DIAGNOSIS COMPLETE (2026-10-14):** Native GLIM normally takes insertion
+1 incremental update; Kimera's saved numOptimize=1 takes one. Neither requires
our fresh full nonlinear gradient1e-5. Stock marginalization preserves fixed
linearization points; our FL2a adapter instead creates anchored chart factors,
reloads the retained graph and forces whole-window refinement. This confirms a
solver/publication-contract mismatch, not a WNOA equation bug. Saved-objective
diagnostics reproduce FL3a/FL3d and show significant residual curvature in one
late oscillating direction:total estimated curvature4.776–4.949 vs GN2.752.
WNOA contributes; the marginal's own extra curvature is tiny, so do not blame
the chart derivative alone. Source second-curvature attribution is step-sensitive;
no exact sender blame/eigenmode proof. Two tiny private diagnostic evaluations,
no optimizer/source/journal run or production/model change;430protected files
unchanged. Next design issue is consistent marginal/publication semantics, not
blind Qc/iteration tuning. No fix promoted;FL3 incomplete,FL4 held. See
[FL3g report](experiments/fixed_lag_cause_fl3g_20261014/REPORT.txt).

**FL3f COMPLETE — BOUNDED CANDIDATE REJECTED (2026-10-14):** One ordinary
iSAM2 refinement before and after unchanged retirement/reload,4charged steps,
completes structurally in32.86ms excluding separately timed reference work.
However,post-state differs from converged FL3e by max.369657m/.381882deg,
.022191m/s and.015725rad/s;all predeclared max-state screens fail. Same-Values
joint-prior comparison also fails:6.293%max normalized information change,
1.539normalized gradient RMS change,1.876e-5relative nullspace leakage.
Finalgradient.00186663 is explicitly NOT converged at1e-5. Exact reference
reconstruction reproduces116pre/7post and byte-identical FL3e Values;
75000previous archivefiles and429protectedfiles unchanged. Source conservation,
71supports/174slots,2absorbedIDs/1pending admission all pass. Private target
rows explicitly labelled incremental_candidate/publication_candidate,never
production online output. No source/full-journal run,sweep,retry,tuning or
production integration. One low-cost refinement policy is ruled out;fixed lag
itself is not. FL3 incomplete,FL4 held. See
[FL3f report](experiments/fixed_lag_incremental_fl3f_20261014/REPORT.txt).

**FL3e COMPLETE — ONE WHOLE UPDATE PASSES (2026-10-13):** Target-only200-step
publication ceiling on frozen lifecycle code. At event84958/support1224:
116pre-refinements+1retirement+1chartreload+7post-refinements=125steps;
committed gradient6.71063e-6 passes unchanged1e-5. Whole event1.358424s,
including dispatch/copy/checks/archive. Retirement.000188s,reload.007093s;
retained position shift across retirement/reload/post-solve max22.09micrometres.
All117initial/pre-solve Values and costs/gradients exactly reproduce FL3d;
75000prior archivefiles byte-identical;287protectedfiles unchanged.71retained
supports,174slots;2sourceIDs absorbed,1pending admitted;exact version/ID
conservation and unchanged surviving payloads/slots. One private publication,
then STOP before next event. Production20-step policy unchanged;no source run,
tuning,full-journal continuation or new synthetic/sanitizer campaign. This
closes this update's lifecycle check,NOT FL3 or live capacity. FL4 held. See
[FL3e report](experiments/fixed_lag_complete_update_fl3e_20261013/REPORT.txt).

**FL3d COMPLETE — SINGLE PRE-RETIREMENT SOLVE PASSES (2026-10-13):** Actual
iSAM2 restored from the saved prefix, not a batch substitution. Initial plus
first20Values byte-identical to FL3a; costs/gradients identical;75000prior archive
files and161protectedfiles unchanged. Original refinement passes1e-5 at step116
(gradient7.97781e-6),116accepted/0rejected,1.272s targettime;93.34s prefix+target.
Compared with step20, position changes .104520mRMS/.107055mmax;orientation
.108191degRMS/.110616degmax. Gradient oscillates despite decreasing cost.
Forced diagnostic stop rolls back to event84957/newest1223 before retirement,
publication or later events. This establishes budget insufficiency at THIS
solve, not full fixed-lag acceptance or universal iteration/runtime bounds.
Production still20steps; no model/source/threshold change or live run. FL3
remains incomplete and FL4 held. See
[FL3d report](experiments/fixed_lag_isam_convergence_fl3d_20261013/REPORT.txt).

**FL3c INCOMPLETE — REFERENCE-PATH ABORT (2026-10-13):** Authorized GN200
fixture stopped at step2 BEFORE the larger-budget test: step1 matches original
local coordinates to1.93e-12, but recursive batchstep2 differs7.83e-6, exceeding
the predeclared1e-8 reproduction tolerance. Physical position difference max
8.17micrometres; not a production trajectory failure. Original cost/gradient
and saved objective exact;22Release/22focusedASan/UBSan pass,148protectedfiles
unchanged. Earlier QR checks prove isolated-step, not recursive-path agreement.
No convergence/budget conclusion, retry, relaxed tolerance, source/journal/live
run or production change. Need exact incremental-state reproduction before
claiming original solver's required iterations. See
[FL3c report](experiments/fixed_lag_convergence_fl3c_20261013/REPORT.txt).

**FL3b COMPLETE — CANDIDATE FAILS (2026-10-13):** One predeclared installed
GTSAM adaptive diagonal-LM policy tested on FL3a's exact saved initial objective.
20 attempts,20 accepted,0 rejected:cost decreases, but final normalized gradient
0.04378656 fails the unchanged1e-5 gate (79.96x the retained GN20 gradient).
42 Release and42 focused ASan/UBSan checks pass; wrapper matches native adaptive
iterate(), original graph/covariance unchanged,134protectedfiles unchanged.
No source/journal run, production integration, parameter sweep, budget increase
or real-fixture retry. This policy is not adopted; does not rule out all damping.
FL3 remains failed/incomplete,FL4 held. See
[FL3b report](experiments/fixed_lag_damping_fl3b_20261013/REPORT.txt).

**FL3a DIAGNOSIS COMPLETE (2026-10-13):** Failure at support1224 is20accepted
cost-decreasing Gauss-Newton steps exhausting the frozen budget;gradient
.000547608 remains54.76x the1e-5 limit.75000prior archive/index files byte-identical,
94protectedfiles unchanged. Fresh batchQR directions match iSAM2 to2.33e-12;
original marginal chart Jacobian matches independent finite differences.
The remaining gradient alternates at old boundary poses1154/1155. Large raw
position motion(.330mRMS) is almost rigid-window motion(.061mmRMS after rigidfit).
Combined Jacobian fullrank but illconditioned;two bounded defaultLM controls
also fail strict gradient, despite lower costs. Serialized same-objective fixture
saved;no production repair, covariance/threshold/budget change, source/live run,
or continuation beyond failed event. Next proposed task is isolated existing
GTSAM adaptive damping/trust-region validation on this fixture, not automatic
solver substitution. FL3 remains failed/incomplete;FL4 held. See
[FL3a report](experiments/fixed_lag_failure_fl3a_20261013/REPORT.txt).

**FL3 STOPPED (2026-10-13):**20s real-data prefix passes exact accounting,
pre-retirement state tolerance,129retirements and byte-identical observer repeats.
ONE full148.58s journal attempt stops at support1224/122.4s after initialization:
`pre-retirement stationarity`. No failed output, retry, threshold/budget/model
change or live run.1224acceptedpublications/1153retirements retain71supports;
72659receivedversions=1816insert+70817replace+25startup+1pending,exactoracleorder.
Covariancepayloads unchanged. Successfulportion whole-servicep95.123539s,
max.203624s;RSS27448KiB;simulatedqueue49at0.1x. Runtime is promising but the
complete gate FAILS numerically;failedgradient/attemptcount were notretained.
Same-supportfullhistorypasses3refinementsteps. Nextproposedwork is onefailing
retained-objective diagnosis, not tuning or live capture.80protectedfiles unchanged.
See [FL3 report](experiments/fixed_lag_replay_fl3_20261013/REPORT.txt).

**Fixed-lag lifecycle integration (FL2b, 2026-10-13):** SYNTHETIC RECEIVER PASS.
Private pending/active/absorbed stable-ID receiver uses FL2a/E2b/gnssFGO and the
original event-stream decoder. Retirement, replacement, complete rollback and
atomic archival output pass1004 assertions; unchanged FL2a regression passes651.
Repeated Release logs/archives are byte-identical; focused core/stream ASan/UBSan
have no findings;48 protected files unchanged. No-retirement matches independent
bare iSAM2:state max4.07e-19,covariance0.145 publications/74 retirements retain71
supports,peak active IDs138/slots212; proper finalized versus online paths saved.
An early endpoint-identity guard ordering bug was corrected and covered by new
tests. Raw clock samples do not copy the graph unless they emit a support.
Optional endpoint cache is explicitly off for this transactional candidate.
No source/model/default change, real journal, speed or live-capacity claim.
Next proposed task is FL3 saved-data validation/timing, not a live run. See
[FL2b report](experiments/fixed_lag_lifecycle_fl2b_20261013/REPORT.txt).

**Fixed-lag boundary correction (FL2a, 2026-10-13):** SYNTHETIC PASS, private only.
An explicit original-chart joint marginal with correct current-chart derivatives
resolves the FL2 boundary fixture: gradient9.7154e-7 vs unchanged1e-5. Existing
retirement/QR is reused; only retained states are rebuilt, never full history.
145 synthetic publications/74 retirements retain71 supports, bounded slot storage,
and pass strict publication.651 assertions pass in two byte-identical Release
runs and focusedASan/UBSan;38 protected files unchanged. The initial marginal-slot
growth bug was caught and corrected without moving source slots or relaxing gates.
Complete receiver lifecycle/cache/output integration remains unimplemented;
FL3/FL4 stay held. No speed/live claim or default change. See
[FL2a report](experiments/fixed_lag_boundary_fl2a_20261013/REPORT.txt).

**Original fixed-lag correctness gate (FL2, 2026-10-13):** BLOCKED in its frozen prototype.
Private existing-smoother staging passes linear means/full-covariance checks.
A nonlinear boundary revision fails unchanged strict publication (gradient
.001841722 vs1e-5); identical no-retirement revision and independent same-prior
reference pass. Rollback is verified. No complete source lifecycle integration,
real replay, source run or speed claim. Full history remains default; FL3/FL4
held. See [FL2 report](experiments/fixed_lag_receiver_fl2_20261013/REPORT.txt).

**Fixed-lag retention feasibility (FL1, 2026-10-13):** Metadata only, complete.
Observed oldest-support ages max5.3s GLIM/5.0s Kimera select7.0s by the
predeclared ceil(max+1s) rule. Every88,254 admitted version reconciles with the
offline oracle; zero added losses,71 retained supports/72 transient.15 tests,
8 byte-identical repeated products,35 protected files unchanged. All2,101
hypothetically absorbed IDs have source-retirement evidence;74 unretired source
IDs remain retained. No estimator/build/timing or model change. Next proposed
FL2 is isolated marginalization correctness, not a live run. See
[FL1 report](experiments/fixed_lag_retention_fl1_20261013/REPORT.txt).

**Fixed-lag planning decision (FL0, 2026-10-13):** The user requested a plan for
an isolated fixed-lag WNOA receiver. See
[the staged plan](GNSSFGO_FIXED_LAG_RECEIVER_PLAN_20261013.md).
Planning is complete; no receiver implementation/build/replay or source run was
performed. Full history remains the reference/default. Next proposed task is
FL1, a read-only revision/support-age and lifecycle feasibility check, not a
solver switch. Marginal-aware publication/recovery and revision retirement must
be validated before timing. E3a/E4 and all tuning remain on hold.

**Latest diagnosis (S5e, 2026-10-13):** Full back-substitution without nonlinear
update on18 copied initial-publication snapshots passes0/18 frozen1e-5 gradient
checks. It is cheap (median2.185ms, maximum3.552ms), but remaining nonlinear
corrections reach2.187mm position and.110798m/s velocity. Median97.10% objective
reduction is not stationarity/state equivalence. Installed GTSAM4.2a8's
update.forceFullSolve also forces zero-threshold eligible relinearization;
calculateBestEstimate alone does not. This isolates a custom global nonlinear
publication burden, not an intrinsically slow backsolve.68/68 Release and
ASan/UBSan; both prefixes and ONE full saved-journal replay match all19 offline
products and E2b counters exactly. Initial snapshots/gradients exactly match
S5d. No policy/default/source change or live run. Cheap solve alone cannot replace
strict publication; E4 rejection and live/offline mismatch remain. See
[S5e report](experiments/receiver_linear_completion_s5e_20261013/REPORT.txt).

**Latest diagnosis (S5d, 2026-10-13):** Private read-only snapshots pass68/68
Release and ASan/UBSan, exact observer-OFF/ON prefixes and ONE complete saved
CBS-ON journal replay against the E2b OFFLINE oracle (all19 non-clock products
and endpoint counters). At18 preselected publications, initial-to-final maximum
position/V corrections reach2.179mm/0.110798m/s: first-pass work is not all noise.
After the first pass, remaining differences max0.172509mm/0.000734446m/s; the
first pass supplies at least99.9363% of that publication's objective decrease.
At142.4s, four blocks violate the gate after pass5, yet pass6 relinearizes all4275;
trial/recovery positions differ only6.754e-11m. This localizes costly global
polishing in our strict publication wrapper, not a new covariance/model fault.
It does NOT validate early stopping: all snapshots inherit previously refined
history, and first-pass full back-substitution versus nonlinear relinearization
is not separated. No production change, source/live run, timing benchmark or
tolerance relaxation. Original live/offline mismatch and E4 rejection remain.
751 protected files,49 raw artifacts,six source snapshots and offline oracle
preserved. See [S5d report](experiments/receiver_publication_s5d_20261013/REPORT.txt).

**Latest isolated optimization (E2b, 2026-10-13):** Private default-off exact
endpoint reuse passes64/64 Release and ASan/UBSan with cache disabled/enabled,
exact five-second controls, and four exact full saved-journal replays against
the S5c OFFLINE oracle. Same-executable/core ABBA runtime447.272→396.674s mean
(11.31% shorter); late publication p95 improves8.1–8.9%.14.68million/24.77million
endpoint requests reused, bounded8192 entries, no cache failures. Late p95 is
still1.316–1.321s (total service p951.464–1.469s), so live160 capacity remains
uncertified. No production/source changes or live rerun. The old live/offline
discrepancy below remains unresolved. See
[E2b report](experiments/receiver_endpoint_reuse_e2b_20261013/REPORT.txt).

**Latest measured diagnosis (S5c, 2026-10-13):** Original-executable sampling
localizes the delay to full-history publication refinement (79.52% CPU overall,
88.18% late). Disjoint dominant kernels: source residual/Jacobians 35.30%,
Gaussian QR 24.98%, WNOA motion residual/Jacobians 12.51%. The accepted 20 ms
profile has 23879 samples, zero truncation/errors, exact equality to the original
unsampled OFFLINE control and 1.49% wall overhead. That original offline control
equals S5b timers-OFF exactly but still differs slightly from the original LIVE
capture: the old failed live-reference gate remains failed, and instrumentation
alone is not its cause. Do not claim exact live work-count reproduction.
First proposed optimization is now E2b shared exact endpoint evaluation: 4402
final factor endpoint uses refer to 2204 source endpoint times. Reuse the
original pose/Jacobian calculation only at bit-identical complete inputs;
at that point actual savings were unmeasured; E2b now reports them above.
E3a stays on hold. Full-history scaling/QR remain;
no promise of indefinite-duration capacity. 751 protected inputs, 49 raw
artifacts and six source repositories unchanged. No estimator edits or live/
source reruns. See [S5c report](experiments/receiver_sampling_s5c_20261013/REPORT.txt).

**Earlier measurement boundary (S5b, 2026-10-13):** Do not start E3a from the candidate
identified below. Private timing instrumentation passed 50 Release checks with
timers disabled and enabled, focused profiler ASan/UBSan checks, and exact
five-second replay. The full timers-disabled replay failed the exact reference
gate: maximum online position difference0.161446mm, ten late publication
decision rows differ, and recovery rows65→74. All1481 publications remain
stationary and source accounting is unchanged, but this is not an exact timing
surrogate. The full timers-enabled replay was not launched. No internal cost
percentages or optimization are accepted.751 protected inputs,49 raw artifacts
and six source repos remain unchanged. Next measurement should sample the
unmodified frozen executable; current per-process `perf` access is blocked by
host permissions. See [S5b report](experiments/receiver_subphase_s5b_20261013/REPORT.txt).

**Earlier audit (S5a, 2026-10-13; candidate order superseded by S5c):** Retained-data analysis localizes the late
receiver backlog to growing global publication work plus more refinement passes.
In 140–148.58 s, publication median/p95 is 984.068/1619.918 ms; even calls
without recovery have p95 1329.959 ms against the 1000 ms nominal wall budget.
Source-version arrival rate stays approximately 610/s simulated; the exact cache
records no failures and improves its late hit fraction to about 39%.
The frozen WNOA motion prior still finite-differences all six X/V/W blocks;
earlier analytic V/W reuse only covered source interpolation. Recommend the
first isolated E3a step: motion V/W blocks only, preserving numerical pose blocks
and the exact residual, with independent checks before any replay. No patch or
new estimator execution occurred. No retained timer separates derivatives from
QR/copying/gradient/IO cost, so no exact speedup is promised. See
[S5a report](experiments/receiver_publication_audit_s5a_20261013/REPORT.txt).

**Latest capacity result (2026-10-13):** S5's one frozen shared-native provider
160s capture stopped at148.55s when receiver queue1018 exceeded1000 (final
transport peak1020). Observed GLIM late120–148.57s scan p95 improved from
1124.443 to724.291ms versus the retained E1/E2 prefix; this is not full160
acceptance. Receiver nominal refinement alone reached median978.562ms and
p951622.498ms in140–148.57s against a nominal1000ms wall support budget.
All1481 recorded publications were stationary; no provider reset/failure
was recorded. Full post-run lifecycle/wire/replay validation was not run
after the capacity stop.47 fresh Release checks pass and747 protected files
plus six source repos remain unchanged. No retry, tuning, default promotion
or push. See [S5 report](experiments/shared_linearization_s5_live160_20261012/REPORT.md).
Earlier acceptance records below retain their original bounded scope.

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
| E3a | Remaining WNOA motion-factor derivatives | Independent numerical derivative checks; manifold/frame tests; frozen-journal state/cost/covariance checks with predeclared tight tolerances | On hold: S5c measures source interpolation as the larger kernel; E2b exact reuse proposed first, no derivative patch authorized |
| E2b | Exact shared endpoint pose/Jacobian evaluation | Complete bit-exact input keys, immutable bounded/thread-safe reuse, revision/rollback/invalidation tests; exact frozen-journal outputs/decisions before matched timing | Passed2026-10-13:64 Release/64 ASan+UBSan OFF/ON; four exact full offline replays; mean11.31% shorter runtime. Late p95 still above budget; private/default-off, no live promotion |
| E3b | Remaining interpolated pose derivative blocks | Separate patch and independent checks as for E3a; do not redo already validated V/W substitution | Not started |
| E4 | Selective incremental publication refinement | Same factors/events and fresh current-Values stationarity certificate; reference full refinement fallback; short/medium/late replay comparisons | Tested 2026-10-12; NOT ACCEPTED: strict state bounds failed,86.8% fallback,matched refinement~40% slower; stopped at130.68s |
| E5 | Transport/logging/bookkeeping efficiency | One subchange per test; lossless ordered revision delivery and auditability; same factor accounting; no silent dropped/coalesced versions | Not started |
| S0 | Preserve E1b.4a and plan native Gaussian-equation reuse | Recoverable local checkpoint; saved staged plan; distinguish native information from MAIN conditioning and mixed marginal priors | Completed 2026-10-12; all 677 hashes and six source HEAD/status checks passed; no implementation/run/push |
| S1 | Read-only feasibility of MAIN native-equation reuse | Exact capture/retirement hooks, factor provenance, coherent reference charts and minimal isolated patch surface | Complete: feasible with lifecycle hooks; 677 hashes and six source repositories unchanged; no numerical certification, implementation or run |
| B0 | Fresh unchanged first40s overall timing reference | Preserved E1b.4a build/settings; original functional gates and exact saved-journal repeat; report MAIN, LOCAL, covariance and complete-scan timing | Passed: LOCAL median/p95 10.965/15.385 ms; complete-scan 206.942/437.376 ms; functional gates and exact receiver repeat pass. MAIN remains dominant |
| S2 | Private default-off native Gaussian history prototype | Synthetic same-chart dense oracle, native-only retirement, tangent transport, lifecycle/rollback and focused sanitizers | Passed: 26/26 Release; 25/25 UBSan and ASan+UBSan. Twelve retirements, dense oracle, zero extra native evaluations in actual MAIN cache fixture. No live integration, publication or run |
| S3 | Private default-off MAIN lifecycle and native-admission observer | Actual fallback-wrapper update/retirement/reset fixtures; no added native evaluations; unchanged MAIN; same-chart shadow oracle before promotion | Real-data shadow gate passed: one ETH1 first40s capture;783 updates,313 retirements,20 covariance checks, max joint relative difference6.01e-15.44/44 Release twice and ASan/UBSan;18 regressions;receiver repeat exact. LOCAL authoritative. See experiments/shared_linearization_s3_capture_20261012/REPORT.md |
| S4a | Saved native equation cost attribution only | Original/instrumented covariance equivalence, isolated stage/query timings and query-cadence audit | Complete: dense rank check95.94% of query cost; original query median160.274ms. Cost attribution only; follow-up optimization is S4b. See experiments/shared_linearization_s4_timing_20261012/REPORT.md |
| S4b | Eigenvalues-only rank inspection in private Gaussian backend | Same rank decisions, unchanged equations/covariances, focused boundary/sanitizer tests, matched saved-input timing | Complete:31/31 Release and ASan/UBSan;130 rank cases agree; saved outputs byte-identical. Query156.711→38.045ms median,4.12× faster; still no provider promotion. See experiments/shared_linearization_s4_eigenvalues_20261012/REPORT.md |
| S4c | Conservative fast rank certificate with original eigensolver fallback | Unchanged covariance and rank decisions, bounded error certificate, adversarial tests and matched timing on saved equations | Complete:40/40 Release and ASan/UBSan; saved outputs byte-identical; query37.285→11.196ms median vs S4b,3.33×. All20 checkpoints certify. Private only; no live run. See experiments/shared_linearization_s4_certificate_20261012/REPORT.md |
| S4d | Private ordinary-publication provider; disable independent LOCAL in one matched first40s capture | Same extraction/helper and lifecycle tests; exact provider-to-wire covariance; frozen source/receiver gates; complete-scan latency | Passed:47 Release/47 ASan+UBSan,18 regressions;389 exact wire covariances,zero LOCAL updates/provider GPU calls. Complete-scan median206.942→167.421ms,p95437.376→312.422ms; exact receiver repeat. Private/default-off; no long-run certification |
| S5 | Frozen private provider first160s capacity validation | Same S4d binaries/settings, complete source coverage, native history/wire checks, strict receiver publication and exact journal repeat; early/middle/late timing | Capacity failed at148.55s receiver queue gate. Partial scan timing improved; full correctness/replay gates unaccepted. No retry, model edit or promotion |
| S5a | Retained receiver publication-cost audit around140–148s | Trace frozen executable sources; separate graph size, iterations, recovery, revision delivery and unmeasured subcosts | Completed2026-10-13: global publication work localized; E3a motion V/W derivative candidate proposed, not implemented |
| S5b | Receiver subphase timing without changing estimation | Private timing-only instrumentation, exact-output/decision checks and overhead control on saved S5 journal; disjoint inclusive/exclusive accounting | Stopped2026-10-13: full exact-reference gate failed; no full timed profile, no optimization; external profiling permission unresolved |
| S5c | Unmodified receiver sampling diagnosis | User-space profiling, exact frozen-journal output gates, overhead/sample coverage and dominant-code inspection | Complete2026-10-13: offline exact/quality gates pass,23,879 samples,1.49% wall overhead; publication/source-Jacobian/QR cost quantified. Original live equality remains failed; E2b proposed only |
| S5d | Incremental estimate versus strict publication | Read-only Values/gradient snapshots; exact offline decisions/accounting/output gates; separate p/orientation/V/W and per-pass progress | Complete2026-10-13:68 Release/68 ASan+UBSan, exact prefixes/full journal. First pass supplies>=99.9363% objective reduction at18 selected checkpoints; later global polishing localized. No early-stop/policy validation or promotion |
| S5e | Complete cached linear solve versus nonlinear update | Copied solver, no update; independent cache-isolated fresh verification, S5d snapshot equality and exact replay gates | Complete2026-10-13:68 Release/68 ASan+UBSan; exact prefixes/full replay. Full backsub passes0/18 strict gradient checks; remaining V correction.110798m/s. No replacement policy/promotion |
| FL0 | Plan an isolated fixed-lag WNOA receiver | Reuse existing smoother; define lag from arrivals/revisions, marginal-aware checks, preservation and staged gates | Planning complete2026-10-13; see GNSSFGO_FIXED_LAG_RECEIVER_PLAN_20261013.md |
| FL1 | Bound recorded revision retention without an estimator | Exact clock/dispatch/identity reconciliation and lifecycle simulation | Passed2026-10-13;7s candidate lag, zero added losses; followed by FL2 below |
| FL2 | Isolated existing-smoother marginal correctness | Dense linear covariance, nonlinear current-chart objective, revision and rollback gate | Original failure retained; corrected in FL2a, private receiver integration passes FL2b. Synthetic gate complete; FL3/FL4 unstarted |
| FL2a | Exact original-chart marginal boundary adapter | Independent derivative/dense-reference, strict publication, repeated retirement, slot ownership, rollback and sanitizer gates | Synthetic pass2026-10-13:651 assertions, repeated Release identical, ASan/UBSan clean,74 retirements/71 retained supports. Private only; no real replay or timing |
| FL2b | Pending/active/absorbed receiver and transaction/archive integration | Original parser, no-retirement control, replacements/retirement/rollback, conservation and deterministic outputs | Synthetic pass2026-10-13:1004 assertions,651 inherited regressions, Release/ASan/UBSan and archive audit pass. Next gate FL3, no live/default promotion |

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

The 2026-10-13 FL0 decision authorizes fixed-lag **planning only**, separately
documented above. It does not change the full-history baseline or authorize
executing every FL stage. The restrictions below still govern implementation
and experiments until the corresponding next stage is explicitly authorized.

No covariance tuning, CBS redesign, changed priors/Qc, fixed-lag receiver,
revision coalescing, source cadence changes, extra sequence, or longer stress run.
Batching distinct revisions into a different nonlinear solve order also needs
its own explicit algorithmic decision. Fixed lag may bound resources but brings
historical revision and marginal-prior semantics; reconsider only separately.

## Execution ledger

### FL2b — fixed-lag receiver lifecycle integration (2026-10-13)

- Authorization: user "Proceed" after FL2a. SYNTHETIC PASS; private integration
  only, no real journal replay/live run, tuning, default edits or push.
- Reuse FL2a smoother, E2b WNOA prediction/interpolation/source factors, existing
  independent support clock and revision rules. Add pending/active/absorbed
  ownership, transactional publication/output and bounded hot-path bookkeeping.
- Freeze7s/100ms/model/Qc/priors/dense Gaussian scale1/strict1e-5/20-step gate.
- Protocol: experiments/fixed_lag_lifecycle_fl2b_20261013/PROTOCOL.txt.
- Implemented transport-independent Receiver plus minimal stream adapter reusing
  the exact E2b EventStream parser. One current factor per stable ID; every ordered
  version inserted/replaced individually; stale/duplicate rules preserved.
- Consumed factors become on-disk absorbed IDs, not replaceable recycled slots.
  Pending-old-bracket/new-after-absorption cases fail closed. Timestamp maps
  are not refreshed, support count bounded, no lost information hidden as exclusions.
- Staged solver/ledger/clock/output transaction, immutable commit archive and
  explicit online versus fixed-lag-finalized trajectories. Disconnected stream
  fails without finalization. No crash-resume/fsync durability claim.
- No-retirement bare-iSAM2 comparison:state4.0657581468206426e-19,covariance0.
  Same-prior retained-reference covariance difference1.1096821839239393e-12.
  145 publications/74 retirements:71 retained/72 transient supports,371 admitted
  versions,276 IDs=148 absorbed+128 active;peak active138,slot high-water212.
  Max strict gradient5.6751479353189249e-6,max2 publication-phase solver steps.
- Code review fixed stable-ID validation BEFORE startup exclusion/pending
  classification. Regression cases added; thresholds unchanged. Raw clock events
  without support use scalar-only commit, with byte-identical old/new products.
  Endpoint memoization disabled explicitly to isolate rollback; frozen E2b untouched.
- 1004 assertions pass twice byte-identically in Release and focusedASan/UBSan;
  source-schema stream repeated archives identical and sanitizer clean; independent
  validator reconciles archive lifecycle and exports paths.651 unchanged FL2a
  regression assertions pass.48 protected files unchanged. No full-repository-suite
  claim; no real-data/timing result. Head remains
  d3d152a28440f97ced3bd7a50ee072874beafe19; no commit/push.
- Report:experiments/fixed_lag_lifecycle_fl2b_20261013/REPORT.txt.
  Artifacts:/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013/.
  FL3 prefix/full saved-journal validation and whole-service timing proposed next,
  not executed automatically. Full history remains the rollback/default path.

### FL2a — anchored marginal boundary refinement (2026-10-13)

- Authorization: user "Proceed" after the FL2 correctness stop. SYNTHETIC PASS;
  only the boundary adapter is completed, not the full receiver.
- One isolated hypothesis: retain each joint marginal's original chart in an
  explicit factor with correct current-chart derivatives, then rebuild ONLY
  retained variables/slots in existing iSAM2. Existing gnssFGO retirement stays.
- This is a deliberate marginal-aware continuation, not the prohibited old
  full-history rebuild and not a threshold relaxation. No removed states return.
- Freeze 7 s, model/Qc/priors/covariance,20-step budget and1e-5 publication gate.
- Protocol: experiments/fixed_lag_boundary_fl2a_20261013/PROTOCOL.txt.
  No datasets, real journal replay, timing campaign, source/default edits or push.
- Implemented AnchoredMarginal with pinned Pose3 chart derivatives and immutable
  original Gaussian/anchor/cross terms. Existing staged smoother still marginalizes;
  an active-only iSAM2 reload permits current-chart refinement without refreshing
  the prior or resurrecting states. Source slots remain stable.
- Original failing negative control still fails. Same boundary revision now passes
  normalized gradient9.715419674596865e-7; independent same-prior reference and
  actual iSAM2 covariance checks pass. Three nonlinear marginalizations match
  independent full pre-retirement QR covariance to max2.941505546083523e-14.
- 145 streaming publications across14.4s/7s lag:74 actual retirements/reloads,
  222 retired blocks,71 retained supports/213 variable blocks/852 scalars;
  slot high-water144,gradient max2.0608890249919413e-6,max3 publication-phase
  solver steps including retirement and reload. No speed claim.
- Initial test_release1 caught marginal slot growth(high-water32). Newly generated
  receiver-owned marginals now reuse empty slots and trim empty tail; source
  slots do not move. Original storage bound passes(high-water10). Failure retained,
  protocol correction documented; no mathematical/tolerance change.
- Final651 assertions pass twice in Release with byte-identical logs and focused
  ASan/UBSan(no findings).38 protected files unchanged; validator passes.
  Report:experiments/fixed_lag_boundary_fl2a_20261013/REPORT.txt.
  Artifacts:/home/yeranis/results/gnssfgo_fixed_lag_boundary_fl2a_20261013/.
- HEAD unchanged:d3d152a28440f97ced3bd7a50ee072874beafe19; no commit/push.
  Next bounded work: receiver pending/active/absorbed ownership, transaction/output
  and cache retirement integration plus remaining FL2 tests. FL3/FL4 unstarted.

### FL2 — isolated fixed-lag correctness gate (2026-10-13)

- Authorization: user "Proceed" after FL1. BLOCKED, FL2 only; not promoted.
- Frozen lag 7.0 s, E2b model and pinned runtime; full history stays default.
- Start with the marginal-prior/strict-publication gate before wiring a real
  journal receiver. Reuse the existing smoother, no new optimizer or fallback.
- Protocol: experiments/fixed_lag_receiver_fl2_20261013/PROTOCOL.txt.
- No source run, real journal replay, tuning, library installation or push.
- Implemented private StagedSmoother plus audit-only anchored marginal; actual
  existing ordering/marginalizeLeaves reused, atomic/maps/solver snapshot copied.
- Linear retained mean max1.29e-14/full covariance max1.15e-10 pass. Synthetic
  nonlinear X/V/W at100ms,7s cutoff and first actual retirement pass; then a
  still-explicit boundary-edge revision fails correct normalized-gradient gate:
  .001841722>1e-5. Second proposed update raises cost and is rejected, not a
  20-iteration timeout. Same revision without retirement passes3.49e-7; same
  frozen-prior independent reference passes2.02e-9. FD confirms test gradient.
- Failed publication/retirement transaction rolls back. Standard fixed anchors
  are excluded from relinearization; no fallback, threshold or model changes.
  Full lifecycle/pending/interpolation/cache tests deliberately held at this gate.
- Report: experiments/fixed_lag_receiver_fl2_20261013/REPORT.txt. Detailed final
  Release/sanitizer/determinism checks in result-root validation.json. HEAD
  remains d3d152a28440f97ced3bd7a50ee072874beafe19; no commit/push.
- Final85 supporting assertions pass in two byte-identical Release runs and
  focusedASan/UBSan(no findings); acceptance remains BLOCKED/exit2 in all.
 29 protected files unchanged. No full-suite or real-data correctness claim.

### FL1 — recorded revision retention feasibility (2026-10-13)

- Authorization: user "proceed" after FL0; FL1 only.
- Status: PASS FOR RECORDED STREAM. Metadata-only journal/dispatch reconciliation
  and lifecycle simulation; no estimator execution or model edits.
- Protocol: `experiments/fixed_lag_retention_fl1_20261013/PROTOCOL.txt`.
- Lag rule declared before numeric analysis: maximum dispatched oldest-bracket
  age, plus 1.0 s engineering margin, rounded UP to a whole second. Strict-before
  retirement on the 100 ms physical support clock; no wall-clock retirement.
- Result:7s lag, observed maximum5.3s,1.7s headroom;88,254 admitted versions,
  86,053 replacements,25 unchanged startup exclusions,zero added losses.
  71 retained supports,72 transient,2,101 absorbed IDs all source-retired,
  100 still explicit;74 source IDs right-censored and retained at EOF.
- Validation:15 metadata tests;8 repeated byte-identical outputs;35 protected
  files unchanged. Original CSV/estimator/model untouched. Corrected a derived
  CSV writer schema error in incomplete analysis1; retained it and added test.
- Report:experiments/fixed_lag_retention_fl1_20261013/REPORT.txt.
  Artifacts:/home/yeranis/results/gnssfgo_fixed_lag_retention_fl1_20261013/.
- HEAD unchanged:d3d152a28440f97ced3bd7a50ee072874beafe19. No commit/push.
  No solver correctness, covariance, speed or live-capacity claim. Stop before
  FL2; next proposed step is private fixed-lag implementation/synthetic tests.

### FL0 — fixed lag WNOA receiver planning

- Date/authorization: 2026-10-13, user "ok plan how to do it correctly".
- Status: PLANNING COMPLETE; checkpoint d3d152a28440f97ced3bd7a50ee072874beafe19.
- Plan: GNSSFGO_FIXED_LAG_RECEIVER_PLAN_20261013.md. Use the existing gnssFGO
  IncrementalFixedLagSmoother and E2b observer-free model; keep full history as
  default/reference, source profiles and source_wnoa/covariance settings frozen.
- Read-only inspection confirms full-history-only revision/publication guards,
  a recovery helper rejecting marginalized fixed variables, and Gaussian-cost
  helpers that do not support arbitrary marginal factors. The old two-state
  fixed-lag CT test does not actually marginalize. These are integration/test
  boundaries, not a claim of an existing fixed-lag experiment or measured bug.
- Next proposed FL1: metadata-only support/revision lifetime analysis, one
  conservative lag and zero-added-loss lifecycle simulation. Future FL2 proves
  marginal information/charts/rollback; FL3 measures retained real data; FL4 is
  a separately authorized live gate. No ATE-driven lag selection or auto-retry.
- Validation: documentation/source/API inspection and git diff checks only;
  no estimator edits, code tests, builds, journal execution, live run or push.
- Full-history correctness/performance limits and old live/offline mismatch
  remain unchanged. Stop after delivering the plan; no solver promotion.

### S5e — full linear back-substitution versus nonlinear relinearization

- COMPLETE2026-10-13, authorized by user "proceed" after S5d.
- One hypothesis: on a copied initial-publication iSAM2 snapshot, can
  calculateBestEstimate() complete the existing linear solve without an update
  and supply the first-pass correction? Strict original publication remains
  authoritative. No source run, policy change, covariance/threshold tuning.
- Same18 preselected S5d support checkpoints, frozen S5 journal/E2b oracle.
  Isolate verification factor evaluations from authoritative mutable caches;
  require exact observer-OFF/ON outputs, decisions and cache counters.
- Private protocol/code: experiments/receiver_linear_completion_s5e_20261013/.
- Parent332d3a1e24829d759f6a222336d2be296c18cac3; private branch
  audit/receiver-linear-completion-s5e-20261013. Replay SHA256
  7a83639b47a39f6d571fd887184724e716c078b259c66178daa509ae7e2d1535.
- 68 Release/68 ASan+UBSan checks pass; original fixtures' scalar equals(...,0)
  strict-inequality mistake and typed-null API build correction documented, all
  failed development evidence retained. Final consistent-header rebuild also
  passes68/68 Release and ASan/UBSan; inherited tests now select this private
  model consistently (see API_NOTES.txt). No estimator replay retried.
- Observer-OFF/ON prefixes and ONE full saved journal match19 nonclock products
  plus E2b counters exactly. All18 initial Values/objectives/gradient coordinates
  match S5d exactly; original1481 publication checks stationary.
- Full backsub alone passes0/18. Remaining max p/q/V/W differences to strict
  publication2.186764mm/.000163536rad/.110798m/s/.007761619rad/s.
  Backsub median2.185ms,max3.552ms; copied-solver/verification times separate.
  Raw439.026s observer replay is not an accepted timing benchmark.
- 751 protectedfiles,49 raw artifacts,six source repositories and E2b/S5d
  references unchanged at final check;191 result files hashed. Original live/offline mismatch
  unresolved. E4 remains rejected. No source/live run, tuning, production change
  or push. Necessary nonlinear work remains; no claim that every factor must
  be globally rebuilt. E3a remains a next isolated candidate, NOT executed.
- Checkpoint: local commit containing this entry and REPORT.txt; stop after S5e.

### S5d — incremental estimate versus strict publication diagnosis

- Authorization: user "ok proceed" on2026-10-13 following the authors-versus-
  receiver cost inspection. COMPLETE. One hypothesis: distinguish meaningful
  state corrections from numerical polishing in existing full-history refinement.
- Use the frozen S5 journal and E2b offline oracle only. No GLIM/Kimera/bag/live
  run, alternative policy, covariance/model/derivative edit or default promotion.
- Retained logs lack pre-refinement full Values. Add a private observer that
  copies already-computed Values/gradient/diagonal at selected early/middle/late
  publications; no extra factor or solver evaluations. Match the unchanged
  publication decisions, source accounting and estimator outputs exactly.
- Protocol and detailed result: experiments/receiver_publication_s5d_20261013/.
- Preserve the existing1e-5 gate; do not claim that small one-step corrections
  prove safe accumulated behavior if refinement were removed. E4 stays rejected.
- Parent d0d675fbdf985737b9d3a7d622b8ce5a2a4f4530, private branch
  audit/receiver-publication-s5d-20261013. Replay SHA256
  c0098e67acc0c427dadda42d9951b0eaf773eab8c803824fd4b030190140535c.
- Added only observer hooks/copies, not extra factor/solver evaluations.68/68
  Release and68/68 ASan/UBSan passed; inherited disabled benchmark excluded.
  Observer disabled/enabled five-second controls and ONE full enabled replay
  match all19 non-clock oracle products and E2b cache counters exactly.
- All1481 publication checks stationary;88279 versions,88254 insertions,
  86053 replacements,2201 active IDs,25 unsupported skips unchanged.
- 18 predeclared support snapshots: initial-to-final p/orientation/V/W maxima
  2.178697mm/1.63536e-4rad/.110798m/s/.00771523rad/s. After first pass maxima
  .172509mm/3.74521e-6rad/.000734446m/s/.000684004rad/s. First pass objective
  decrease fraction>=99.9363%; these are correction sizes, NOT GT accuracy.
- At142.4s, pass5 leaves four violating blocks but pass6 relinearizes4275.
  Stationary trial rejected by existing objective rule; accepted QR recovery
  differs by6.754e-11m at most. Later passes are physically small but gate-
  necessary under the unchanged policy. Raw run time595.841s is observer-
  instrumented and NOT a speed benchmark. Retained oracle provides timings.
- Before/after751 protected files,49 raw artifacts,six source HEAD/status
  snapshots and40 E2b oracle files plus journal unchanged. Owned replay exited.
- No stopping-rule change/fixed lag/tuning/source or live run/push. Proposed
  next read-only distinction (not executed): full linear back-substitution only
  versus nonlinear relinearization on isolated solver snapshots. A one-step
  diagnosis cannot prove a safe repeated publication policy. Checkpoint is the
  local commit containing experiments/receiver_publication_s5d_20261013/REPORT.txt.

### E2b — exact shared endpoint evaluation

- User authorizes the proposed implementation/correctness/timing gates, not a
  source/live run. Parent492a82cc855022325d6c86cc6c4932ded057e1e3.
- Private branch perf/receiver-endpoint-reuse-e2b-20261013; protocol/code under
  experiments/receiver_endpoint_reuse_e2b_20261013. Original binaries preserved.
- OFF/ON must reproduce the S5c original OFFLINE control exactly, including
  solver decisions/accounting. Earlier live mismatch remains out of scope.
- One cache hypothesis only; Qc, covariance, source mathematics, numerical
  derivatives and strict publication remain unchanged. No tuning/retries.
- Implementation gate:64/64 Release tests with cache OFF/ON and64/64 ASan+UBSan
  OFF/ON passed; one inherited disabled microbenchmark is not a correctness test.
  Original offline five-second control and candidate OFF/ON match all19
  non-timing products. Private replay SHA256
  b4eab40dacf074cdbcd13ae23ed8175cba5bb57b0b493eec06f5b9a2de0d72d3.
- Initial replay compilation caught duplicate original/private headers through
  SnapshotAudit.h. Preserved failure log; copied that unchanged header into the
  private overlay, then compilation passed. No estimator/replay failure retried.
- All four full saved-journal controls passed all19 exact non-timing products;
  enabled cache counters repeat byte-identically. ABBA wall seconds:
  disabled449.124286,enabled397.473468,enabled395.874464,disabled445.419639.
- Both paired performance gates pass:11.50%/11.12% shorter runtime and late
  publication p95 reductions8.87%/8.06%. Mean speedup1.1276x. Cache reuses
  14,677,392/24,768,616 requests(59.26%); maximum8192 entries, no failures.
- Late publication p95 still1.316–1.321s; total support-completion interval
  p951.464–1.469s. This is a partial speedup, NOT live160 capacity acceptance.
  No source/library/production edit, live rerun, tuning, promotion or push.
- After checks preserve751 protected inputs,49 raw artifacts,six source HEAD/status
  snapshots and38 S5c oracle files. Report/code/evidence in
  experiments/receiver_endpoint_reuse_e2b_20261013; checkpoint identity is the
  local commit containing that report (git log -1 --format=%H -- its path).

### S5c — unmodified executable sampling diagnosis

- User authorizes continued overnight diagnosis and a specific evidence-backed
  fix proposal. No implementation, new source run, model or covariance change.
- Parent8650acbd11b0705d69b6d8fbe048eb77d019dc8d. S5b remains stopped; its failed
  exact gate is not waived. Prefer sampling the unmodified frozen executable.
- Protocol and working status: experiments/receiver_sampling_s5c_20261013/.
- Private user-space collector feasibility first; no security setting changes,
  root, global installations or kernel profiling permission bypass.
- Full unmodified unsampled replay equals S5b's timers-disabled replay exactly,
  but not the retained live output. Preserve that failed live-reference gate;
  instrumentation alone is not its cause. A separately declared matched offline
  profiling scope uses the unchanged offline control, not a relaxed tolerance.
- First10ms sample matches that offline control exactly (~3.8% overhead), but
  110/48832 stacks exceed4096 frames. Quality gate remains failed. A final20ms,
  separate8192-frame observer passed cadence, exact-prefix and full exact-output
  gates:23879 samples,maximum depth4430,zero errors/truncation/overruns,
  99.9967% coverage,1.49% wall overhead. Its CPU attribution is the primary result.
  Censored10ms comparison assigns all capped stacks to UNKNOWN and agrees on
  the dominant work. Failed historical gates remain failed, never overwritten.
- Overall/late publication CPU shares79.52/88.18%. Disjoint overall kernels:
  source relative residual/Jacobians35.30%,QR24.98%,motion12.51%; copy0.95%,
  journal parsing0.38%. NormalizedGradient inclusive37.02% overlaps these costs,
  not an extra additive percentage. Current-value certificates linearize the
  entire retained graph and refinement forces full solves.
- Unsampled late publication median/p95941.077/1526.115ms; nonrecovery
  p951256.881ms, already above the1000ms support budget at0.1x. Late fresh
  linearizations:1,210,380 source and809,200 motion; cache failures0.
- End graph1481 supports/4443keys/3684factors.88,254 version operations replace
  old versions; only2201 source factors remain, not88,254 accumulated factors.
- E2b proposal: shared endpoint interpolation results/Jacobians at exact
  complete inputs.4402 endpoint uses/2204 unique source endpoint times reveal
  redundant cross-factor work, not a measured hit rate or speedup. Extend E2,
  preserve all formulas, source covariances, revision order and strict checks.
  E3a reprioritized, E4 stays rejected, no optimization implemented.
- Final integrity PASS:751 protected files,49 raw S5 artifacts and six source
  HEAD/status snapshots unchanged. No source/live run or production binary
  changes. Small live/offline discrepancy remains unresolved; isolated profile
  does not certify sustained live capacity. Follow-up disabled on completion.
- Completion commit is the commit containing this report/ledger; no push.

### S5b — measure before selecting an optimization

- Authorization: user asks to establish the exact delaying work before trying
  solutions. Parent e66243471100a97921f2f405fa709c683c99ba7d; branch
  audit/receiver-subphase-s5b-20261013.
- Scope: private diagnostic build of the frozen receiver; instrumentation only.
  Preserve all formulas, solver decisions, factor/event order, tolerances and
  logs. Use the saved S5 journal, including its startup history; do not restart
  a late graph from incomplete state or run new GLIM/Kimera/bag playback.
- Measure nested/exclusive work for publication, solver copying, iSAM2 update,
  factor-family linearization, motion/interpolation derivatives, current-value
  gradient checks, recovery, revision insertion and IO. Uninstrumented library
  work must remain explicitly named as a remainder, not mislabeled QR time.
- Validate profiler scope/exception/accounting behavior and disabled/enabled
  outputs. Require unchanged ordered events, publication decisions, state
  outputs and certificates before interpreting cost percentages. Measure
  instrumentation overhead against the same private binary with timers off.
- E3a and all other optimizations are on hold. No derivative replacement,
  fixed-lag change, relaxed certificate, covariance tuning or live experiment.
- Protocol/evidence: experiments/receiver_subphase_s5b_20261013/.
- Result: profiler Release and ASan/UBSan fixtures pass; existing50 receiver
  checks pass with timers off and on; five-second saved-journal outputs match
  exactly across original/private-off/private-on. Full private-off replay
  completes in470.163s but fails exact reference equivalence. Do not interpret
  that wall time as a live capacity comparison or accept full cost attribution.
- First certificate-number difference occurs near8.88s; first changed
  publication decision at143.88s. Ten of1481 decision rows differ. Maximum
  online/final position differences are0.161446/0.0418404mm; all1481 outputs
  still satisfy the unchanged stationarity rule. Revisions have identical
  IDs, factor slots and outcomes. Numerical cause of rebuild difference is
  not established; it is not proof of a source/model bug.
- Full timers-enabled replay was NOT launched. The predeclared stop rule was
  enforced. No retry, estimator fix, E3a, live source run, or tuning followed.
- After-integrity check passes751 protected files,49 raw artifacts, six source
  repos and17 pinned private build files. Compile and prefix-fixture mistakes
  are retained separately in BUILD_NOTES.txt; not relabeled as solver failures.
- External sampler feasibility only: host perf is installed but task-clock
  access is denied (perf_event_paranoid=4); sudo requires a password. No sysctl,
  privilege, installation or container changes. gprofng exists on host but not
  in the pinned container. No external profile has been collected.
- Detailed result: experiments/receiver_subphase_s5b_20261013/REPORT.txt.

### S5a — bounded retained receiver publication audit

- Authorization: user "yes proceed to the next audit", 2026-10-13.
- Parent333410526a1f40966f6d5e1554d327a368d0ba63; branch
  audit/receiver-publication-s5a-20261013.
- Status: completed. Inspected S5's existing logs and exact E2 executable
  implementation. Central section140–148.58s; earlier bins only as controls.
- Distinguish all-history work per iteration, iteration/recovery counts,
  stable-ID revision processing, source availability, and logging/transport.
  Do not infer unrecorded subphase durations from call counts.
- No source/receiver replay, build, estimator edits, settings changes, covariance
  tuning, new performance benchmark or automatic next implementation.
- Protocol/evidence: experiments/receiver_publication_audit_s5a_20261013/.
- Evidence: 1480 support publications, 2446 ordinary refinement steps and
  65 recovery rows; 1481 stationary checks including final. Late mean attempts
  rise to3.85 (median4), versus2.24 in130–140s. No-recovery late p951329.959ms
  already exceeds nominal1000ms wall/support. Refinement82.541s vs revision
  update9.925s in the late physical/insertion-time cohorts; not a complete
  wall-clock partition. Full graph has4443 keys and3684 factors at the end.
- Next proposal: E3a motion V/W derivative-only isolated wrapper, first
  independently validated and microbenchmarked. Do not switch all pose
  derivatives, relax stationarity, repeat rejected E4 or change the model.
- Integrity passed:747 protected files,49 frozen raw artifacts,62 archived
  evidence files,21 inspected inputs and six source HEAD/status pairs unchanged.
  Analysis/verification syntax and independent count checks passed. No new
  Release/sanitizer or estimator tests; S5 remains partial and capacity-failed.
- Completion commit: the commit adding this S5a report and ledger; resolve via
  `git log -1 --format=%H -- experiments/receiver_publication_audit_s5a_20261013/REPORT.txt`.

### S5 — frozen provider first160s validation

- Authorization: user "Proceed" after the proposed single 160-second ETH1
  CBS-ON validation. Parent 8d862639dc66de6f434d37cf6526bcedd3ccceaf;
  branch engineering/shared-native-live160-20261012.
- Status: CAPACITY FAILED / PARTIAL CAPTURE, completed2026-10-13.
  Reused exact S4d release_2 source libraries and E2 WNOA receiver; only duration
  and fresh run identities changed. No build, covariance-policy change, tuning,
  source retry, default promotion or push. Preparation commit c9bb99e.
- Protocol: experiments/shared_linearization_s5_live160_20261012/PROTOCOL.md.
- Preserve strict gates. Report every 10-second timing bin, remaining MAIN
  work, native history/retirement, source delays, receiver backlog and coverage.
  Accuracy is not the acceptance criterion; no new OFF run or method follows.
- Stopped148.550033s, final completedGLIMscan148.571408938s,1479scan records.
  Controller queue1018>1000; transport final peak1020. All owned processes
  stopped; no source/receiver/controller successful completion marker.
- Matched observed-prefix comparison against oldE1/E2: overall scan
  median412.067→206.661ms,p951036.715→622.301ms; late120–148.57
  median727.310→371.955ms,p951124.443→724.291ms. Separate coupled captures,
  not exact-input patch attribution or a full120–160/0–160 gate pass.
- Provider1476queries,median10.178ms,p9528.506ms; zero LOCALupdates and
  covariance-scope GPU evaluations. Zero resets/failure tokens. Query counters
  balance; last30537=1624active+28913absorbed+0removed. Activeframes49–50,
  but native factor density and memory still grow; no universal boundedness claim.
- Late140–148.57 receiver refinement median978.562ms,p951622.498ms;
  40/85 refinement records exceed1000ms, excluding other receiver work.
  All1481publicationchecks stationary(max9.989086623436253e-6), yet backlog
  grows. This localizes measured cost without identifying its exact internal cause.
- Fresh47Release tests pass; prior47sanitizer/18regression evidence retained,
  not rerun.747protectedhashes/sixsourceHEADs+clean states unchanged.
  Full wire/lifecycle and deterministic receiver repeat NOT RUN after failure.
  No ATE or CBS-OFF comparison. Raw artifacts retained, hashes archived.
- Report and evidence: experiments/shared_linearization_s5_live160_20261012/.
  Next proposal only: bounded retained-log receiver refinement audit140–148s;
  no automatic live retry, implementation, model change or next sequence.

### S4d — private ordinary-publication provider integration

- Completed, authorized by user "Proceed" after S4c. Parent
  c22a7aad39320f737ef0b368eb56191765dcee29; branch
  engineering/shared-native-provider40-20261012.
- One private candidate replaces the ordinary outgoing LOCAL covariance route
  with S4c native Gaussian history and skips the independent LOCAL optimizer.
  Defaults and diagnostic revision covariance (currently MAIN) remain unchanged.
- Focused adapter/lifecycle tests and no-data launch admission precede one
  ETH1 first40s CBS-ON 0.10x capture. Compare complete-scan timing with preserved
  B0, not just query speed. No full sequence, tuning, production promotion or push.
- Protocol: experiments/shared_linearization_s4_provider_20261012/PROTOCOL.md.
- Result: BOUNDED PRIVATE INTEGRATION PASSED.47/47 Release and non-PIE
  ASan/UBSan,18 existing regressions; original MAIN control exact. No-data
  startup and effective-library/parameter admission pass. Initial compile-type
  mistake and sanitizer PIE startup failure retained; no failed sensor attempt.
- One complete ETH1 first40s capture:391 scans,389 ordinary provider queries,
  all full6x6 wire covariances exact; zero LOCAL updates/provider GPU calls.
  Native transport, temporary-factor lifecycle and numerical checks pass;
  all nine saved receiver repeat files byte-identical.21970 versions conserved,
  393 publications, queue max46/end0, gradient max9.921e-6<1e-5.
- Observed complete-scan mean183.525→140.044ms, median206.942→167.421ms,
  p95437.376→312.422ms. This is overall GLIM timing, not just a query projection.
  Coupled MAIN workload also changed; do not attribute the entire gain directly
  to deleted LOCAL. New query mean12.913ms,p9533.259ms; native history cost is
  included in MAIN, not separately measured. No longer-run or accuracy claim.
- Integrity:680 baseline/19 S1 pins,716 S3 paths,7/8/10 S4a/b/c pins and six
  source HEAD/status checks pass. No global provider promotion or push.
- Report: experiments/shared_linearization_s4_provider_20261012/REPORT.md.
  Completion commit is the local commit containing this report and both plans.

### S4c — conservative rank certificate

- Authorization: user "Proceed" after the bounded S4c proposal. Parent
  feb737397453e10e8f13423993df020e41bbd3e6; branch
  engineering/shared-native-rank-certificate-20261012.
- Status: ACCEPTED PRIVATE OPTIMIZATION; NO PROVIDER PROMOTION. Derived and
  tested one conservative check-only shifted sparse Cholesky/reconstruction
  certificate using the pinned Eigen library.
  It cannot simply reuse the unshifted factor as a rank guarantee. Preserve
  the existing GTSAM covariance elimination; the shift never enters that graph.
- The full S4b eigenvalue check remains the fallback on every uncertain case.
  No threshold relaxation, production/provider change, live source run or push.
- Protocol: experiments/shared_linearization_s4_certificate_20261012/PROTOCOL.md.
- Result:40/40 Release and40/40 focused ASan/UBSan pass, retaining all31 previous
  tests and130 rank fixtures. All12 real-journal passes have byte-identical query
  output,783 updates,313 retirements,20 queries;5321=4513 absorbed+808 active.
- Matched query median37.285→11.196ms (3.33×), p9540.467→22.551ms; all three
  speed gates pass. All20 unique checkpoints certify; fallback0/60 measured
  queries. Candidate history stage/commit0.510s per journal;389-request query
  projection4.943s, not measured live/all-request timing. MAIN is unchanged.
- Integrity:716 S3 protected files,680 baseline files,19 S1 pins,7 S4a/8 S4b
  identities,13 staged inputs and six source HEAD/status checks pass. Existing
  independent LOCAL remains authoritative. No code/library/profile promotion.
- Report: experiments/shared_linearization_s4_certificate_20261012/REPORT.md.
  Completion commit is the local commit containing this report and both plans.
  Stop for a separate matched40s integration decision; no automatic next run.

### S4b — eigenvalues-only rank inspection

- Authorization: user "Ok proceed" after S4a's proposed isolated rank-check patch.
- Parent: 9c7c89001bf4cc9bfdab3c9d26214e4ac3766804; branch
  engineering/shared-native-eigenvalues-only-20261012.
- Status: ACCEPTED PRIVATE OPTIMIZATION; NOT A PROVIDER PROMOTION. Protocol:
  experiments/shared_linearization_s4_eigenvalues_20261012/PROTOCOL.md.
- Preserve original S2/S3/S4a and all production/source files. Change only the
  private rank helper's Eigen solver option to EigenvaluesOnly. Keep every rank,
  finite/symmetry, lifecycle and covariance check. No new sensor/fusion run,
  default change, covariance publication, threshold relaxation or push.
- Result:31/31 Release and31/31 focused ASan/UBSan tests;130 rank-decision cases
  agree. Pose/mixed fixtures and all12 real-journal outputs byte-identical.
  Every real pass preserves783 updates,313 retirements and20 covariance queries.
- Three paired measured replays: full-query median156.711→38.045ms,4.12× faster;
  p95164.459→39.834ms. Same rank threshold. Dense rank check still83.9% of query
  cost. Candidate history stage/commit total0.515s per783 updates.
- Cadence projection only:389 requests × sampled query mean =13.178s, not an
  all-request/live measurement and not yet better than preserved B0 LOCAL.
  No second nonlinear optimization or GPU factor evaluation in this candidate.
- All716 S3 protected files,680 baseline files,19 S1 pins,7 S4a identities and
  six source HEAD/clean-status checks pass. No source or fusion rerun.
- Full evidence/report: experiments/shared_linearization_s4_eigenvalues_20261012/REPORT.md.
  Completion commit is the local commit containing this ledger/report; stop here.

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

### CP-FL4v — preserve and push the actual ETH1 launch

- Date/authorization: 2026-10-17, user requested saving settings for future
  chats and pushing all relevant changes to Zakyera GitHub.
- Status: ARCHIVE VERIFIED; repository checkpoint/push in progress.
  Archival/documentation task only; no estimator run,
  rebuild, tuning, default promotion or alteration of frozen evidence.
- Scope: preserve FL4v's actual launch, copied sensor profiles, private build
  source overlays, build provenance and compact validation/results. Preserve
  the intervening uncommitted diagnostics and push named milestone branches.
- Baseline: gnssFGO 67c00392cd640a432bda9b67afcb57be4face381;
  receiver/Kimera binary hashes and limits in the FL4v report.
- Crucial limit: completed 120 s ON is not a completed 120 s OFF/ON pair.
  FL4t OFF stopped early; the OFF/ON comparison is a common 105.4 s prefix.
- Recovery: `experiments/checkpoints/20261017_fl4v/` and
  `GNSSFGO_GRANDTOUR_FL4V_LAUNCH_20261017.txt` (this task).
- Evidence: 170 byte-identical archived files, 15 binary identities checked;
  original source/profile/build inputs unchanged. Read-only checks passed:
  30 Python syntax, 7 shell syntax, 31 strict JSON, 14 XML launch files;
  15 GLIM JSON-with-comments profiles hash-checked without rewriting.
- Existing CBS objective-audit/tests preserved unchanged in cbs ed93343;
  this task did not rebuild or rerun their C++ tests. Original validation
  receipts retain their scope; checkpoint checks are not estimator validation.

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
