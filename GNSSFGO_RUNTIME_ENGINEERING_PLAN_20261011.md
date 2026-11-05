# Incremental CBS and WNOA runtime engineering plan

## CP-V112 attempt1 — corrected LEICA-2 fresh pair,2026-11-05; COMPLETE, post-audited

Both fresh428.75s OFF/ON captures, four gates per arm, cleanup, saved-only EVO
and report verification passed03:14:32local. H=experiments/
leica2_split_imu_live_pair_cpv112_20261105_attempt1; authoritative RESULTS_REPORT,
results.json,complete.json,report_verification.json and supplemental post_closeout.
REPORT.txt remains a sealed historical startup snapshot; do not edit it. Original
failed CP112 remains immutable, never reused as a qualified arm. No further run.
Correct selected6.0s/rosbag6.099999100s restored the intended absolute window;
scientific profiles, strict exact-header gates, CP82 GLIM, corrected985cKimera
in BOTH arms and CP-K19 seven-second fusion unchanged. Runtime DSO/maps and
OFFreceivefalse/ONreceivetrue passed before playback.108EVO archives verified.

Method1 EVO1.36.3 SE3/no-scale versusIE-TC at the calibrated AP20 point. The
reference is NOT independent prismGT. Common100ms six-curve comparison has
3844rows/384.0supportedseconds(89.5627% of428.75s),378reference-distance0.5m
RPEpairs,205.8974m reference. OFF->ON common RMSEs:

| Product | ATE m | ATE improvement | translation RPE m | rotation RPE deg | estimated path m |
|---|---|---|---|---|---|
| GLIM | .498242 -> .501302 | -.61% | .019023 -> .016755 | .389261 -> .327163 | 205.7319 -> 205.5826 |
| Kimera | .836987 -> .481459 | +42.48% | .067940 -> .020546 | .469991 -> .486466 | 211.3859 -> 205.1373 |
| ONLINE fusion | .552047 -> .501756 | +9.11% | .031803 -> .029873 | 1.475251 -> 1.468669 | 223.3019 -> 218.5818 |
| DELAYED 7s-finalized | .551374 -> .501167 | +9.11% | .017100 -> .016153 | .263808 -> .245387 | 205.1476 -> 205.5315 |

DELAYED is fixed-lag-finalized, not ONLINE/full-history. Fusion improves against
OFF but does not beat either ON native source on common ATE; Kimera rRPE does
not improve. Full-native cadence is separately scored, never substituted for
the matched common mask: ATE GLIM.572734->.574095,Kimera.841780->.486306,
ONLINE.630471->.576433,finalized.629670->.575966m. Fullnative tRPE GLIM
.019791->.017396,Kimera.073962->.020830,ONLINE.033788->.031134,finalized
.017824->.016918m; rRPE.333863->.235970/.278635->.301506/1.537520->1.522918/
.246866->.248705deg. Full-native segmented paths/reference: GLIM207.1413->
207.7624/205.9036m;Kimera212.2259->205.3681/205.4371->205.4423m;
ONLINE224.4384->219.6369/205.9289m;finalized205.3093->205.7093/205.9289m.

Coverage: GLIM4280scoredrows each/427.8995s; native receipts4281rows each.
Kimera1938->1933rows/387.5961->385.1961supportedseconds,11->15gaps>0.5s,
maximum5.099946s. Fusion4282rows/428.1s both. Common segments0.684928–57.584928,
60.484928–195.384928,196.684928–304.184928,305.684928–390.384928s; last38.365072s
excluded from the common mask by sparse native support. Full capture is NOT
dense six-curve coverage. No gap filling or omitted tail relabeling.4287matched
stereo each,1preinit camera pair each,0terminalcamera loss; both exact IMU
header sequences42875CPT7/85688Alpha pass. Intended through-last-Hesai retained.

Actual arrivals/order/IDs/revisions preserved:252537->253503beliefversions,
GLIM204882->206014/Kimera47655->47489;6187->6181inserts,246129->247116replacements.
50preinit exclusions each;171expiredKimera versions/24IDs ->156/25, unchanged
seal_at_absorption. No unsupportedEOF,duplicates,stale or pending; expiry is
an omission, not proven harmless. Payloadmean roundtrip<3.1e-14,covarianceerror0.
Framewire4278GLIMboth,1937->1932Kimera;0unmatched. ON1907K2G/1888G2K persistent
insertions checked; full6x6 once-only Alpha->CPT7 conversion passes, native
GLIM covariance not separately exported for native/wire matrix parity.
MedianKimera translationtrace native9.713685e-7->6.993648e-7m2, transported
9.949962e-7->7.156456e-7m2;rotation3.856615e-8->3.384317e-8rad2. MedianGLIM
transportedtranslation1.781438e-6->1.784170e-6m2. These unpaired relative-belief
distributions do not certify global uncertainty or independence. Optimize
median201.193->213.286ms;processingp9518.740->18.337ms;supportcyclep95
66.237->63.613ms. Previouspose revisionmax.769022->.033547m.

Post-closeout03:17:51 passed all273236pins,647sealedoutputs,667Hesai copies/
originals, originalfailed262969regularfiles+2specialmetadata(noFIFOread),24exact
owned process exits including supervisor/controller, both initially-stopped
containers restored stopped. Exactly2sourcewrappers,0post-audit source runs.
No duplicate capture/validation/scoring or terminal artifact overwrite.
Scoped heartbeat PAUSED after terminal notification; no further work
authorized here. Mixedprior/native-onlyoutgoingFALSE, feedbackcorrelation and
experimentalAlpha model remain. IE-TCsharesCPT7; estimated polylines are not
physical travel; incrementalparity is not nonlinearstationarity/health/calibration.

Historical preparation/launch record below is superseded by the closeout above:

User proceed authorized one fresh OFF/ON following the preserved original
coverage failure. H=experiments/leica2_split_imu_live_pair_cpv112_20261105_attempt1;
P from prepared.json. Original H/P without attempt1 are immutable FAILED data,
not an accepted OFF control and never to be restarted. Read new PROTOCOL,
PRELAUNCH_REVIEW, PREPARATION_REPAIR, REPORT and live stage receipts.

Only operational correction: translate selected-topic offset6.0s to rosbag
all-bag-relative6.099999100s via separate CBSMS_BAG_PLAYBACK_START_SEC. Keep
CBSMS_BAG_START_SEC=6.0 and exact intended window
1732549034915071665..1732549463665071665ns, duration428.75s/.20x/25sdrain.
Host original-bag test passes exact timestamps for4288Hesai/42875CPT7/
85688Alpha/4287matchedstereo; old6.0 fails allfive, reproducing1/10/20/1/1
extraearly/missingtail messages. No timestamp shift, truncated tail or relaxedgate.
Actual Noetic equivalent check precedes any master/source/receiver startup.

Prelaunch unrestricted port substitution damaged seven stagingJSON copies;
host timestamp/adapter tests rejected them before admission or any launch.
Bad copies/diff retained; restored raw audits/headers/fixture byte-identical to
originals, bounded port replacement used. Successful30adapter/40geometry/
14negative schemas/5EOF/5syntheticEVO tests and final parity supersede rejected
staging hashes. No sensor retry occurred during preparation. All scientific
profiles/binaries/receiver math remain unchanged, corrected985cKimera BOTH modes.

Final admission01:54:47 passes273236pins/Hesai667, original10253 plus entire
failed H/P262969regular files and2special metadata entries preserved (noFIFOreads).
174311362560Binitialfree>25GiBreserve;20GiBoutput allowance,5GiBruntimefloor.
New11653/11654ports and leica2_cpv112a1_{off,on}_20261105 sessions.
Started ONCE01:55:06local, launcher86072exited, supervisor86073/start1938945,
controller86075/start1938946 exact host identities/commands verified. Containers
initially stopped. ActualNoetic30fixture passed01:55:45 and exactfive-stream
window test passed01:55:52. OFFcapture86777/start1943545; effective checks
passed985cDSO/maps/body/TF/native_local/receptionfalse/unchangedAlpha parameters.
Playback records selected6.0/rosbag6.099999100; actual first CPT7header
1732549034919515000 and Alpha1732549034918935177 match corrected expectations.
Both native outputs and concurrentfusion confirmed:57ONLINEposes/1730versions,
queue0/max26 startup snapshot. ON pending; no final results. Scoped same-task
heartbeat ACTIVE for attempt1 only, normal progress quiet.

Controller exclusively owns OFF+fourgates->ON+fourgates->ownedcleanup/restore->
savedEVO/verification; monitor must not duplicate stages. No automaticretry,
tuning/build/install/delete/protected edits or unrelatedsequence. Afterterminal
and exact controller/supervisor exit, post_closeout.py once ifabsent; immutable
seals/REPORT preserved, laternotes new supplements, finaldocs and pauseheartbeat.
Method1/common100ms/native/finalized distinctions and all correlation/experimental
Alpha/IE-TC/nonstationarity limitations from protocol remain. No new scores yet.

## CP-V112 — LEICA-2 pair,2026-11-05; failed OFF coverage, post-audited

Controller failed01:31:55local after OFF playback; ON never started. Read
experiments/leica2_split_imu_live_pair_cpv112_20261105/FAILURE_ANALYSIS.txt,
failed.json,capture_off.log and supplemental post_closeout.json. No EVO or
qualified full-pair result exists; saved receiver completion is not source acceptance.

Confirmed harness cause: expected headers use first selected-topic epoch
1732549028915071665ns, while --start=6.0 playback follows all-bag epoch
1732549028815072565ns. Actual interval is99.9991ms earlier at both ends:
1732549034815072565..1732549463565072565ns instead of
1732549034915071665..1732549463665071665ns. Read-only original-bag inspection
matches BOTH saved IMU sequences exactly to the former interval. Total counts
42875CPT7/85688Alpha match, but10CPT7/20Alpha extra early stamps replace the
same number of intended tail stamps. Strict coverage_control.py:12 correctly
rejected them. Preflight changed the audit epoch without translating the
launcher offset; the recorded assertion that unused-topic starts do not affect
playback was wrong. This is not evidence of random CBS/covariance failure.
No estimator setting or scientific gate changed, no failed capture relabeled.

Receiver saved4282ONLINEposes/253227versions(205622GLIM/47605Kimera), queue0/max46;
native Kimera1937rows, final stamp1732549462310630904ns. These remain diagnostic
failed-run artifacts, not full-pair scores. No ON tree and no source complete
marker/source_coverage.json; all four gates and EVO were not reached.
Postaudit01:34:12 verifies10253protected hashes/Hesai667,9exactownedjobs exited,
both containers restored stopped, one source wrapper started and0postaudit runs.
No complete seal exists. Original CP111/older evidence remains protected.
Scoped heartbeat PAUSED. No retry/replay/install/build/tuning/deletion performed.
Any future repair must reconcile bag-origin and selected-topic absolute bounds
before a fresh explicitly bounded protocol; do not resume the old controller.

Historical preparation/launch record below is superseded by this terminal
status and retained for provenance, not authorization to continue:

Latest user proceed authorizes one fresh full usable quiet-start428.75s OFF/ON,
offset6s/.20x/25sdrain, GLIMHesai+CPT7, corrected-private KimeraAlphaIMU and
concurrent CP-K19 7sWNOA/no centralIMU/GNSS both arms. H=experiments/
leica2_split_imu_live_pair_cpv112_20261105. Same CP111/108 scientific profiles,
private binaries, Gaussian1/persistent ON/receptionfalse OFF, native smart25,
CP82 MAIN/shared_native/native_local5s, Alpha biasRW.0043/.000266/200Hz.
No covariancefreeze/seedchange/tuning/build/install/delete/retry orothersequence.

Missing original AlphaIMU bag downloaded viaexisting CLI/nooverwrite,
6607426bytes,sha ba392c29402e102bb469bf68a19d8580d4d7ac86a99fe74ef542870670f2065e.
Raw4288Hesai/42875CPT7/85688Alpha/4287matchedstereo; no missingright. Exact
historical CP-K29 selectedepoch1732549028915071665, window
1732549034915071665..1732549463665071665ns;quiet/calibration checks pass.
Alpha quietaccelnorm~9.57 vsCPT7~9.85m/s2 recorded,not tuned away. Profiles
match missionTF<1e-9. Host30fixturebitparity/40fullcovgeometry/14negative
schemas/5EOFcontrols/5syntheticEVOtests pass;87201referenceposes support fullgrid.
Finalhostadmission00:53:33 passes10253protected hashes/Hesai667, inheriting
CP1119721pins and415sealedoutputs/postaudit, alloldterminal evidence preserved.
20GiBforecast+5GiBreserve,5GiBruntimefloor, fresh11651/11652ports.
Started ONCE00:54:25local:launcher63091exited,supervisor63092/start1574815,
controller63094/start1574815 verifiedwithhostvisibility. Initialfree178098688000B.
Actual Noetic30fixture passed; OFFeffectiveDSO/params/body/TF/receivefalsechecks
passed beforeplayback. Native outputs and concurrentfusion confirmed inOFF:
51ONLINEposes/1371beliefversions,queue0/max14 startup snapshot. ONpending,
no finalscores. Existing scopedheartbeat reactivatedCP112 only; normalprogressquiet.
Sole finitecontroller owns capture/gates/restore/
savedEVOverification. Monitor cannot duplicate work. Afterterminal/exactexits,
postcloseout once;REPORT immutable ifsealed, laternotes supplements;pausemonitor.

Report six-curve common100ms Method1EVO1.36.3 SE3/no-scale vsIE-TC,reference
distance.5mRPE,paths/reference; full-native and7s-finalized separateONLINE.
Keepactualgaps/omissions/revisions/timing/native+transportedcovariance. IE-TC
sharesCPT7/notindependentprismGT;pathnotphysicaltravel;paritynotstationarity/
health/calibration. Mixedprior/native-onlyFALSE/feedbackcorrelation not repaired.

## CP-V111 — full SPX-3 live pair,2026-11-04; completed and post-audited

User requests SPX3 next before drawing cross-sequence conclusions; approvals
handled by configured automatic review, not repeated chat confirmations.
H=experiments/spx3_split_imu_live_pair_cpv111_20261104. One fresh full usable
197.02s offset29s quiet-start OFF/ON at.20x/25sdrain, same CP108 corrected-private
Kimera/GLIM/receiver scientific settings, split Alpha/CPT7 IMUs, persistent ON,
both receivefalse OFF,7sWNOA/no centralIMU/GNSS, no covariancefreeze/tuning.
Original SPX3 Alpha IMU downloaded via existing CLI auth; all raw inputs and
missioncalibration checked.1971Hesai/19702CPT7/39370AlphaIMU/1970matchedstereo.
Final admission passed9721protected hashes/Hesai667;181559201792bytes free
against21GiB reserve. Launched once20:45:43local: launcher16179 exited,
supervisor16180/start82628,controller16182/start82629 verified on host.
Actual Noetic30fixture and both preplayback effective checks passed, corrected
DSO/split topics/body/TF and requested OFF/ON reception. Both captures and all
source/payload/receiver/frame gates completed; cleanup/restore preceded scoring.
Controller complete21:22:39local;48EVO CLI archives verified. Read sealed
RESULTS_REPORT.txt/results.json/report_verification.json/complete.json and
post_closeout.json. REPORT.txt is sealed; later notes must be supplements.

Method1 EVO1.36.3 SE3/no-scale vsIE-TC, six-curve common100ms1931rows,
192.7s/197.02s(97.807%),167reference-distance0.5m pairs,reference90.7671m:
GLIM ATE.016738->.017135m(-2.37%),Kimera.642416->.105379m(+83.60%),
ONLINEfusion.127351->.047204m(+62.93%). TranslationRPE .006773->.007751 /
.052750->.021985 / .026023->.027637m;rotationRPE .256731->.285556 /
.818359->.581779 /1.420132->1.367180deg. Estimated paths90.9522->91.0552 /
91.8220->90.5480 /93.8367->93.9612m. FusionATE improves, tRPE slightly worsens.
Separate7s-finalized ATE.126740->.042982m(+66.09%);tRPE.009511->.009792,
rRPE.463332->.410165deg,path90.5633->90.5379m;NOT ONLINE/full-history.
Full-native ATE GLIM.016766->.017149,Kimera.651979->.108232,
ONLINE.132543->.050728,finalized.131348->.045225m. Scored GLIM1962rows each,
196.099s;Kimera958->957rows,193.498->194.198supported seconds,4->3gaps>0.5s;
fusion1965rows each/196.4s. No gap filling or missing stereo pair;1preinitframe.

Retained receiver116645->116666versions:GLIM93645->93791,Kimera23000->22875.
1965ONLINE/finalized each;25preinit and1unsupportedterminalKimera version each,
0expired/duplicate/stale/pending. Full6x6 payloaderror0,meanroundtrip<2e-14;
wirecounts1960GLIMboth/957->956Kimera and935K2G/925G2K ON insertions verified.
Kimera native translationtrace median3.72006e-6->2.13112e-6m2,transported
3.81099e-6->2.16039e-6m2;rotation5.74082e-8->3.89937e-8rad2. GLIM transported
translation1.66588e-6->1.67757e-6m2. Unpaired relative-belief distributions,
not calibrated global uncertainty; feedback provenance remains native-onlyFALSE.
Kimera optimize median82.747->85.037ms;receiver processing p9522.736->22.260ms,
supportcycle p9569.015->68.239ms;previouspose revisionmax.33100->.05399m.

Supplemental audit21:27:34 passes9721pins/415sealedoutputs/Hesai667 and all23
exactownedjobs exited;both containers restored stopped. CP108383/CP10914/
CP110327 evidence preserved;no capture/scoring repeated,no data deleted.
Heartbeat paused. Do not start LEICA2 or another sequence without next authority.
IE-TCsharesCPT7/not independent prismGT;polyline not physical travel;incremental
parity not nonlinear stationarity/health/calibration. Alpha model experimental.

## CP-V110 — regional late-revision intervention,2026-11-04; completed, not a fix

H=experiments/eig2_fusion_intervention_cpv110_20261104. Read sealed REPORT.txt,
results.json,validation.json,report_verification.json,closeout.json,complete.json
and two diagnostic PNGs. Four full receiver-only replays completed, no source
run or new live pair. Original CP108ON baseline reproduced byte-exact five
state/TUM/summary files,221071archivefiles and249253logicalprogress rows.
273Kimera physical IDs with both endpoints50–70s or275–310s; last accepted
arrival-age<=2.5s anchor held only in3393later rows. hold_cov/hold_mean/hold_both
preserve untargeted events/order/arrival/GLIM/support/model/prior/EOF/runtime.
Actual consumed mean/full6x6 audit passes (covarianceerror0,meanroundtrip<1e-12).
50preinit/0expired/1unsupportedEOF and3545ONLINE/finalized publications unchanged.

Method1 EVO1.36.3 SE3/no-scale versusIE-TC, fixed common3532rows/349reference-
distance0.5m pairs: ONLINE ATE baseline.13465044,hold_cov.13355237(+.8155%),
hold_mean.14017844(-4.1054%),hold_both.13824787(-2.6717%)m. Separate7s-finalized
ATE.13230966/.13107693/.13785753/.13576609m. Full-native3545rows separate in
report, with paths/RPE/support. No gap filling. Under fixed baseline alignment
whole ONLINE RMS.13465044/.13473989/.14667914/.14199616m: covariance-only tiny
ATE gain is alignment-sensitive, not an overall fixed-fit improvement.
Fixed-fit peak-window298–304s .213534->.200232m forhold_cov, while own-fit
.213534->.213558m. Early/late local benefits coexist with worsened other regions.
Mean holding helps some early downstream positions but worsens late/overall.
Only2.6266% of ON's excess squared error above saved OFF is removed byhold_cov.
No new OFF was run. This is insufficient to solve fusion degradation.

This conditional test does not rule out earlier overconfidence, directional
misweighting, mean bias or source feedback correlation. Source beliefs already
contain CBS history; regional receiver changes do not undo it. Holding matrices
is not uniform inflation. Do not deploy a global covariance/mean freeze. Next
suggestion: saved-data first/young-belief and directional residual/weight audit;
no additional trial is authorized. IE-TCsharesCPT7; pathnotphysicaltravel;
incremental/payload parity does not certify stationarity or uncertainty.

48EVOarchives independently verified. Final audit passes8873protected hashes,
CP108383sealed outputs/CP109seal/original221071archivefiles/Hesai667 originals
and copies. Four new221071-file archive manifests and327outputs sealed. Exact
owned process identities exited; both containers restored stopped. Originals
preserved, no build/install/delete/globalpolicy change. Heartbeat remains paused.

## CP-V109 — EIG-2 fusion error localization, 2026-11-04; completed saved analysis

User requested onset/peaks and surrounding causes, not another run. Read
experiments/eig2_fusion_localization_cpv109_20261104/REPORT.txt,findings.json,
localization.json,belief_analysis.json,verification.json,complete.json and plots.
Reuse exact CP1083532-row common mask/global alignments/error arrays; no new EVO.
Same initial ONLINE pose verified; raw OFF/ON separation exceeds5cm continuously
from53.136s through353.736s. Not a unique causal onset; global alignment alone
cannot date one. ONLINE saved-error window RMS OFF->ON(m):50–70s .086577/.126583,
170–178s .068831/.181654,298–304s .079719/.213534. Largest5sRMScenter300.836s
OFF.082852/ON.216663m. Finalized penalty remains;280s-to-end39.51%net extra
ONLINE squared error. OFF single-point maximum is larger than ON, near144s.

Actual received/terminal matrices,1615same Kimera intervals,3450GLIM intervals.
Kimera medianterminalON/OFFtranslationtrace.59549; age2.5->terminal pairedmedian
.53754ON/.98525OFF. ON ratioofmeantraces.38553 while relativeIE residualRMS
7.699->7.968mm; all6matrixdirectionscontract62.35%. GLIM late~1/ONvsOFF1.00214.
50–70sKimera localmeansworsen2.821->4.612mm despite slightly higherfinaltraceON;
275–290s residual2.943->4.190mm whilemeantrace.4964->.3338mm2;290–310s
5.057->5.325mm and.6421->.3754mm2. GLIM improves in these windows. Timing,
payload and omission evidence does not show a large transportfailure at peaks.
Localized meanbias plus tighter mature weights/correlation is a supported concern,
not proof that persistentfeedback specifically caused the fusionATE penalty.

1600trajectorydiagnosticchecks and40520SPDmatrixchecks pass. All383CP108sealed
outputs/10406readinputs unchanged. No capture/replay/tuning/build/install/delete.
IE-TCsharesCPT7,notindependentGT; normalizedresidualnotNEES/NIS;5srelative
diagnosticnot0.5mEVO RPE. Proposed only: separately authorized parity-first
receiver-only fixed-stream means-versus-covariance intervention in thesewindows.
No automatic intervention or nextsequence; heartbeat remainspaused.

## CP-V108 — EIG-2 latest-setup fresh live pair,2026-11-04; COMPLETED and audited

Terminal update: both full355.02s OFF/ON source+live-fusion captures passed,
all source/payload/receiver/frame gates exit0; saved-only48directEVO1.36.3 CLI
archives/report verification passed. Completed14:40:51local; post_closeout
14:55:11 verifies8873pins,383sealed outputs,Hesai667copies/originals,exact owned
job exits and both containers restoredstopped. complete/REPORT/results immutable.
No replay, source retry, scoring retry, deletion or further experiment started.

Method1 SE3/no-scale, calibrated prism point versusIE-TC; common100ms3532rows,
353.0supportedseconds(99.431%),349reference-distance0.5m pairs,ref188.0137m:
GLIM OFF->ON ATE.046328->.047447m(-2.42%),tRPE.019707->.017653m,
rRPE.638466->.634358deg,path189.7889->189.7217m.
Kimera ATE3.189958->.366042m(+88.53%),tRPE.358022->.043514m,
rRPE.907520->.868306deg,path237.8221->187.4987m.
ONLINEfusion ATE.083176->.134650m(-61.89%),tRPE.039580->.041280m,
rRPE1.996855->2.018519deg,path201.2903->198.5043m.
Separate7s-finalizedfusion ATE.078442->.132310m(-68.67%),tRPE.018001->.018698m,
rRPE.671155->.663871deg,path186.7931->187.0396m. Not ONLINE/full-history.

Full-saved native cadence remains separate, with different mask/RPEpairs/path:
GLIM3543rows/354.1993s,ATE.046492/.047602,path192.1784/192.7878m/ref188.0976.
Kimera1763/1762rows,353.8972/353.7972supportedseconds,ATE3.197040/.365911,
path246.0363/188.3769m/ref186.6136/186.6069. Fusion3545rows/354.4s each,
ONLINE ATE.085702/.135921,path223.9456/218.4264m/ref188.1258;
finalizedATE.078444/.132332,path188.2508/188.5280m/ref188.1258.
Native RPE values and exact segments/gaps are in RESULTS_REPORT.txt/results.json.
No unmatched raw stereo. One pre-init pair each; nativeKimera maxgap.8sOFF/
1.0sON retained;nointerpolation>0.5s,10common queries excluded.50before-init
beliefversions each,0expired. Exactly1ON terminalKimera revision omitted because
future100ms support lies beyond recording; frozenfiniteEOFgate passes, notzero
loss, no state-clock extension. OFFunsupported0. Full captures, not failedprefixes.

Fusion213742/213750versions,208405/208415replacements,71retainedsupports;
support-cycle median51.344/51.934ms,p9572.362/71.936ms;receipt processing
median9.818/9.762ms. Sourcewire3541GLIM each,1762/1761Kimera;0unmatchedwire,
ON1742G2K/1748K2Gsourceinsertions. Payloadcoverror0; nativeKimera->wire6x6
relativeerror<=6.64e-16. Logged revisions43,444/43,394Kimera,170,298/170,356GLIM.
Kimera previouspose revisionmax5.626->.269m;>0.5m27->0; optimize median
248.044->271.101ms,p95422.666->462.486ms. Incrementparity notstationarity.
All-version pooled Kimera transportedtranslationtrace mean4.0288e-6->1.2044e-6
m2 butmedian5.2196e-7->5.8260e-7; nativeAlphamean4.0066e-6->1.1888e-6,
median5.0779e-7->5.6907e-7. Rotationtrace median3.6798e-8->3.3462e-8rad2.
GLIM transportedtranslationmedian1.8598e-6->1.8804e-6m2. Sourcewire young
beliefs differ from pooled mature revisions. These summaries are not paired-age
calibration, independence or proof of fusion error mechanism. No covariancefix.
IE-TCsharesCPT7,not independentprismGT; estimatedpolyline not physicaltravel.
Mixed-prior/native-onlyFALSE remains; experimentalAlpha model unchanged.
Scopedheartbeat PAUSED after completion notification. No SPX3/LEICA2 launch authorized
by this closeout. Earlier launch chronology retained below.

### Launch chronology (superseded by terminal update above)

H=experiments/eig2_split_imu_live_pair_cpv108_20261104; P is prepared_path in
prepared.json. Read initial PROTOCOL plus PRELAUNCH_REVIEW/PREPARATION_REPAIR,
REPORT,prepared/launch/start/status and controller.log. User supplied original
AlphaSense IMU bag; no conversion needed. Initial blocked inventory preserved.
Started ONCE13:36:59local: launcher76672exited,supervisor76696/start1382137,
controller76698/start1382138; exact host command/start verified. OFF capture
77252/start1383595; source/receiver/master receipts match live identities.

One fresh355.02s/offset5.7s/.20x/25sdrain OFF then ON; GLIMHesai+CPT7/HG4930,
KimeraAlpha stereo+built-inIMU, liveCP-K19fusion both arms. Corrected CP102
Kimera985c115d BOTH modes; CP103 persistent/Gaussian1/Alpha biasRW.0043/.000266,
200Hz/7sWNOA Qcx1 unchanged. OFFbothreceivefalse,ONbothreceivetrue. No central
IMU/GNSS factors; nativeAlpha->CPT7 full6x6 once,GLIMidentity,actual revisions/
arrivals/order preserved. NO CP106 covariance freeze or scientific tuning.

Raw audit3551Hesai/35502CPT7/70980Alpha,3550left/right exact pairs, no omitted
right. Alpha median dt.004995s,maxgap.0220241s;quiet threshold passes, norm~9.54
recorded without calibration claim. Profiles byte-identicalCP103; mission
calibration matches<1e-9, exact EIG2 transform used. Host+actualNoetic30fixtures,
14negative/2real source schemas,5EOF controls,5syntheticEVO CLI tests pass.
Actual OFF preplayback effective topics/body/TF/loadedDSO/archive checks pass;
receiver consumes both sources and publishes ONLINE. These are not final scores.
Admission8873protected incl inherited8120/CP10362/104318/10512/106186/10714
sealed outputs;Hesai667copies/originals pass.205832794112bytesfree>21GiBreserve,
5GiB runtimeguard. Fresh ports11631/32. Draft guard/clock and path-only admission
assertion repairs were BEFORE any source launch; evidence preserved, no retries.

Finite controller owns OFF+gates then ON+gates, cleanup/restoration, saved EVO,
verification and immutable complete. Both containers initially stopped; restore
on exit. No duplicate jobs, retries, tuning, build/install/delete or protected
edits. Post_closeout only once after exactcontroller/supervisor exits. Six-curve
common100ms mask separate from full-native and DELAYED7s-finalized; actual
coverage/EOF omissions retained. IE-TC not independentprismGT; path not travel;
incremental parity not nonlinear stationarity/health. Mixed-prior/native-only
FALSE and experimentalAlpha model remain. Scoped heartbeatACTIVE, quiet until
terminal/action, then pause. SPX3/LEICA2 only future intentions, not launched.

## CP-V107 — saved covariance maturation and provenance,2026-11-04; completed

H=experiments/eth1_covariance_provenance_cpv107_20261104. User requested further
covariance analysis; no new capture/receiver/EVO/build/install/tuning/deletion.
Current CP103ON versus historical CP95OFF; not same-binary source causal pair.
All36778OFF/36746ON native Kimera matrices joined to receiver per-edge version
order, full6x6 Alpha->CPT7 covariance relative error<=6.10e-16. Receiverordinal
is stableedge, notnativeupdate: initial diagnostic parser failure/repair retained.
Same1454physical intervals as CP105: late age2.5->terminal paired mediantrace
ratio.267863ON/.920147OFF; native Alpha.268708/.921628. Source, not transport.
In last0.4s of saved accepted lifetime, median per-revision translationtrace
ratio.835611ON/.996299OFF (4316/4322transitions); persistent late tightening.
Full6D late ordered eigenvalue medians ON .200/.337/.463/.730/.872/.910;
all directions contract83.77%ON vs36.93%OFF. Not a single scalar-weight effect.
ON meanvariance falls80.76% while tangent IE discrepancy5.131->5.142mm; paired
median decrease73.21% is a different summary. Reportednorm1.929->.846mm.
GLIM terminal paired ON/OFF trace1.00454, no analogous mode-dependent collapse.
All1472actual source insertions eachdirection finite/SPD/exact loggedpayload;
classic source wire stream distinct from full mature diagnostic fusion revisions.
Current frozen persistent code consumes peers through marginalization, removes
their old tracking, then LOCAL excludes only explicit peer/anchor factors.
CP67 native fixture demonstrated mixed-prior route; CP98 demonstrated separate
CBS-state-dependent native visual information. Exact CP103 causal partition is
unavailable: ordinary logs lack full historical graphs, retirement occurs on
every source update after8.499951s. Do not treat correlation as causal proof.
Receiver replaces sameID slots correctly but does not model crosssource/edge
joint covariance. Separate IMUs/native-only Gaussian history do not establish
statistical independence. IE-TC discrepancy is not true NEES/NIS/calibration.
Next proposed scope, NOT launched: native-only positive-admission marginal
history qualification, same fixed measurements/linearization state and keys,
before/after absorption comparison. Preserve MAIN corrections; do not delete a
mixed prior, tune a scale, or promote CP106's first-matrix freeze as a fix.
If saved checkpoints cannot recover history, seek separate short diagnostic
capture authorization. Then separately evaluate changed export in saved replay.
Independent26544matrix checks/whitening pass;8120protected,CP10362/104318/
10512/106186/95426sealed outputs,Hesai667copies+originals preserved. Both
containersstopped;14newoutputs sealed. No oldartifact edits; heartbeatpaused.

## CP-V106 — saved ON covariance-revision intervention,2026-11-04; completed

H=experiments/eth1_covariance_freeze_cpv106_20261104. User approved the CP105
test, not new sensor captures. Exact original CP103 ON events/CP-K19 binary/
settings; baseline then conditional first-accepted Kimera covariance variant.
Baseline reproduces ONLINE/finalized TUM/CSV,summary,all211422archivefiles
bit-for-bit and237614logical progress rows. Original numerical scores reproduced.
Variant changes only35249Kimera covariance rows for1472IDs; keeps full6x6 cross
terms, all mean revisions, timestamps, endpoints, IDs, order and GLIM/clock/EOF.
36587accepted Kimera payloads verified, covariance error0; same4924insertions/
197912replacements/25pre-init/134expired,0unsupportedEOF/pending,3456each output.
Right-local CPT7 noise schedule defined at each revised mean; NOT posterior
transport/second sensor adjoint. No settings/code/source CBS changes or GT fit.
DirectEVO1.36.3/SE3no-scale/IE-TC prism point, frozen2966query/365reference0.5m
pair mask (296.3s/85.56%,no gap fill): original->held ONLINE ATE.09433461->
.05241466m (-44.44%); separate7s-finalized.09133572->.04380850 (-52.04%).
Common tRPE ONLINE.03649227->.03660302m,DELAYED.01282574->.01327796;
rRPE1.744175->1.745136deg/.323322->.322830; paths214.14590->214.05005m/
201.96974->201.97134 versus201.37418m. Not a uniform accuracy improvement.
Full-native3456rows/345.5s/99.763%,366pairs separately: ATEONLINE.09503443->
.05696725 (-40.06%),DELAYED.09160238->.04872428 (-46.81%); paths222.79317->
222.70192/202.79474->202.77976m versus201.47755m. Estimated polylines,nottravel.
Conditional causal effect of covariance schedule on this saved stream's ATE
demonstrated; source-side contraction mechanism/true calibration not established.
First-matrix freeze is diagnostic, NOT promoted. Correlated peer information
remains; ONLINE jitter/RPE/path excess remains a separate issue. IE-TC shares
CPT7; no nonlinear-stationarity or health certificate/new live OFF/ON claim.
Host missingBoost1.74 rejected prelaunch; original Ubuntu container runtime
used without install/build; two receiver processes each ran once/exited0,
original mappedDSOs equal, cbsms stayedstopped,Ubuntu restoredstopped.
24directEVO archives/API parity0/TUM1.69e-15. Final8120pins,CP10362/CP104318/
CP10512terminal outputs,original211422archivefiles,Hesai667copies+originals pass.
186outputs plus two211422-file archive manifests sealed. Read REPORT/results/
validation/report_verification/closeout/complete; never overwrite or rerun.
No additional ablation, source run, tuning, deletion or production promotion.
Further covariance-policy and publication diagnosis needs a new scoped decision.

## CP-V105 — saved fusion degradation diagnosis,2026-11-04; completed

H=experiments/eth1_fusion_diagnosis_cpv105_20261104. Read REPORT,localization,
belief_analysis,verification,closeout,complete. Diagnostic only:0captures,
0receiver replays,0new EVO calls; no tuning/build/deletion/deployment.
CP95 saved OFF versus corrected CP103 ON is not a new same-binary matched pair.
Existing common ONLINE ATE.061504->.094335m; separate7s-finalized.056428->.091336m.
Longer ONLINE polyline/jitter is distinct from persistent finalized ATE loss.
69.43% of extra aligned ONLINE squared error occurs before90s; alignment means
this is error localization, not proof of causal onset. Typical source arrival
and queue delays nearly unchanged; receiver configs, wrapper and adapter math
match. Both frame/payload/lifecycle gates pass; no duplicate-factor accumulation
demonstrated, revision replaces its previous factor.
Actual received/last-accepted full6x6 covariance on1454identical Kimera intervals:
ON age2.45->4.85s translation trace mean3.72073e-6->7.15866e-7m^2 (-80.76%);
RMS tangent IE-TC discrepancy5.131->5.142mm,reported norm1.929->.846mm.
Paired median ON/OFF trace ratio first1.0126,last.2122; GLIM last1.0045.
Terminal Kimera mean variance ratio.15006 versus squared discrepancy ratio
.33040; these are distinct aggregate statistics from paired median. Later
revisions show confidence tightening that initial source-wire exports miss.
17of1471eligible Kimera edges omitted by fixed-age/terminal requirements;
2970GLIM intervals,0excluded. No gap filling. Not trajectory RPE, true NEES/NIS
or calibration proof; IE-TC sharesCPT7 and its uncertainty is unknown.
Receiver uses full supplied covariance directly; cross-source/overlap dependence
is not modeled. Revised overconfidence plus peer correlation is the strongest
candidate mechanism, NOT yet demonstrated causality or a production correction.
Next proposed test needs approval: saved ON parity replay, then first-accepted
Kimera covariance held fixed (full6x6/frame checked) with all mean revisions,
endpoints, event timing/order and remaining settings preserved. No replay started.
Independent eigen-whitening checks35392matrices (1.46e-13relative discrepancy),
input hashes and saved EVO-error norm parity pass. Final8120protected,
CP104318/CP10362/CP95426outputs,Hesai667copies+originals preserved;12CP105outputs
sealed. Do not overwrite/rerun; next work supplemental. Heartbeat remains paused.

## CP-V104 — saved corrected-ON accuracy,2026-11-04; completed

User approved saved CP103 scoring versus historical CP95 OFF/ON and CP91 ON.
H=experiments/eth1_prior_fix_evaluation_cpv104_20261104. Read INTERPRETATION,
RESULTS_REPORT,results,coverage_review,report_verification,closeout,complete.
No source/receiver replay, new capture, tuning, build, deletion or deployment.
Method1 direct EVO1.36.3 CLI54calls, SE3/no-scale, same calibrated prism point
against IE-TC (not Leica/independent GT). Eleven primary curves share2966rows,
296.3s/346.32s (85.56%),365reference-distance0.5m RPE pairs. Three DELAYED7s
curves use same mask but are separate from ONLINE/full-history smoothing.
ATE m CP95OFF / CP95ON / correctedCP103ON:
GLIM .039497 / .053071 / .038280; Kimera .737605 / .198843 / .192637;
ONLINE .061504 / .118626 / .094335; DELAYED .056428 / .116118 / .091336.
CP91 source-only ON on same mask: GLIM.041514,Kimera.209101; no fusion existed.
GLIM deterioration did not recur; correction plus new realization is not a
causal comparison/randomness proof or a new same-binary matched OFF/ON pair.
ONLINE remains53.38% worse historicalOFF; Kimera rotationalRPE worsened from
CP95ON.527284 to.658833deg despite slightly lowerATE. No uniform-improvement claim.
CP103 common tRPE/rRPE: GLIM.015656m/.335889deg,Kimera.034126/.658833,
ONLINE.036492/1.744175,DELAYED.012826/.323322. Paths202.743/202.069/214.146/
201.970m versus reference201.374m; estimated polylines, not physical travel.
Full-saved CP103 separately: ATE.039995/.192608/.095034/.091602, native rows
3454/1492/3456/3456; supported345.299607/296.798338/345.5/345.5s. Historical
CP95/91 full-saved results reused unchanged. No gap filling or cadence mixing.
Old2969sample mask loses3queries due CP103 Kimera support; common tail ends
301.744604s (44.575396s omitted). All3454 GLIM first-published CSV poses scored;
native shutdown dump3455 contains extra final346.219259s pose0.100005s later.
Do not replace first-published CSV with revised dump or compare rawbody paths
to prism-point scores. Missing right203.898841154s and Kimera12gaps>0.5s stay;
25pre-init/134expired fusion versions accounted,0unsupportedEOF. No covariance
calibration, native-only independence, nonlinear-stationarity or health claim.
Archive/mask verification passes54directCLI archives,130inputpins,parity error0.
Final seal318outputs;8120protected,CP103all62/CP95all426/CP91all221 outputs and
Hesai667copies+originals preserved; both containers stopped, old owned identities
exited. CP104 terminal evidence immutable; future work must be supplemental.
Heartbeat remains paused. No further experiment or score run started.

## CP-V103 — corrected prior full live ON qualification,2026-11-04; passed

User's overnight approval covers continued solution/validation with preserved
scientific settings/results. H=experiments/eth1_prior_fix_live_cpv103_20261103;
read PROTOCOL,PRELAUNCH_REVIEW,PREPARATION_REPAIR,copy_parity and live receipts.
One new full346.32s ETH1 ON/offset0/0.20x/25sdrain with GLIM,Kimera,CP-K19fusion.
Same CP99 scientific settings and all DSOs except CP102 private Kimera985c115d.
Failure-only archive/rethrow configured on Kimera; no tolerance/weight change.
No new OFF, no saved OFF replay, no same-binary randomness-repeat claim. One
missing configuration-only OFF descriptor needed by adapter fixture restored
before launch; ON-only CLI and no OFF source tree prevent unintended OFF run.
Admission8120protected/Hesai667/host fixtures passes. Started ONCE23:37:10local:
supervisor104398/start1346319,controller104400/start1346319,launcher104397exited.
Actual Noetic fixture + preplayback new DSO hash/maps/archive/frames pass.
Completed00:08:14local: full346.32s capture and all4gates pass,0prior-failure
archives;1492Kimera updates,1472persistent injections in each direction.
Fusion202995versions =4924insert+197912replace+25before-init+134expired;
0duplicates/stale/unsupportedEOF/pending,3456ONLINE and3456separate7s-finalized.
Native GLIM3455poses/345.399612s/maxgap0.100990s; Kimera1492/343.598087s/
maxgap5.099976s/tail2.421558s. Missingright203.898841154s remains omitted.
Payload covariance error0; all3452GLIM/1491Kimera wire messages matched.
Saved polylines:GLIM201.9413m,Kimera201.4864m,ONLINE218.9020m,DELAYED201.6503m;
different native cadences/coverage, NOT physical travel/common-mask scores.
No EVO or reference-path scoring; no new OFF or same-binary repeat claim.
New MORNING_REPORT.txt records diagnosis/correction/private regressions/live
qualification and limits. Supplemental post_closeout.json passed00:11:18local:
8120protected/62sealedoutputs/Hesai667copies+originals,13exactowned identities
exited,both containers restoredstopped. Do not rerun that absent-only audit or
overwrite sealed REPORT.txt/complete outputs; later work must be supplemental.
Completed scoped heartbeat PAUSED; no further run/scoring/global deployment.
Historic CP99 failed matrices/theta remain absent: exact cause still unproven;
no GLIM ATE/randomness/covariance/health/nonlinear-stationarity certification.
Mixed-prior feedback and experimental Alpha-calibration limitations unchanged.

## CP-V102 — private backend integration,2026-11-03; qualified

See experiments/prior_anchor_integration_cpv102_20261103/REPORT.txt,complete.json.
Preserved CP76 sources/link inputs reconstruct byte-identical b7d6829d baseline.
New private Kimera985c115d includes exact CP101 headers and out-of-line failure
recorder; CBS82f85c2a byte-identical, no installed/global replacement/settings
change. Baseline and candidate pass CP76 compatibility,48metrics identical;
6persistent replacement cases,9absorption boundaries,9injected rollbacks.
Actual DSO recorder tests pass3unique archives and all3exact matrix/typed-anchor
readbacks, no overwrite, success no-write, fatal guard preserved when recording
disabled or path invalid/relative, real smoother transaction rollback preserved.
One premature test request rejected before launch while predecessor cleanup
active; preserved, unchanged test commands subsequently pass in tests_v2.
Final23:30:57local:7354inputpins/original6425 and historical terminal evidence,
CP99failure,Hesai667copies/originals intact; jobs exited,both containers stopped.
No live CP102 capture/scoring/deletion; exact CP99 failure remains unprovable
without its unrecorded matrices. This is not covariance/ATE/health certification.
User now authorizes continued work while away. Next bounded fresh CP103 ON-only
full-window validation uses corrected private backend and failure archive, same
source/fusion settings. Preserve CP99 OFF/failed ON; do not rerun OFF implicitly.

## CP-V101 — private exact-anchor correction,2026-11-03; offline qualified, not deployed

User approved isolated candidate and offline regressions after CP100. Read
experiments/prior_anchor_candidate_cpv101_20261103/REPORT.txt,analysis.json,
complete.json and authoritative qualification_v3. Two private headers only;
no deployed source/DSO/setting change, installation, deletion or live capture.
OriginalChartPrior validates first and clones original Gaussian at exactly equal
immutable anchor; no epsilon or reanchoring. Moved branch byte-identical, same
original objective. CPK9 retains1e-8 original/cache AND converted/cache guard,
key/ownership and atomic replacement checks. Typed failure has separate errors,
slot/keys/three matrices/anchor; standard what() logs scalar summary. Production
matrix-export catch handler remains to be integrated, not claimed installed.

1100exact-anchor cases:126baseline false rejections,0candidate, coefficient
error0; all1098CP100 baseline errors/rejection decisions reproduced exactly.
66660moved comparisons (including1e-12) matrix/objective difference0.15negative
controls reject; late-slot failure leaves earlier graph pointers/locks unchanged.
Four synthetic iSAM2 lifecycle cases (baseline0;candidate0/82.1589/1000 origin)
pass15marginalization/promotions each, finite C, leaf and retained-key checks.
Baseline/candidate near-origin max pose-local difference3.42114e-49. Not full
Kimera/CBS integration, nonlinear stationarity, covariance calibration or live
success proof. Actual CP99 failed prior/theta unavailable; cause still unproven.

Harness corrections preserved: initial bitwise raw-vs-stored matrix mismatch
1.80249e-17; v2 synthetic ordering crash. Baseline-only trace crashes at k3 BEFORE
first marginal prior; absolute-index groups violate CCOLAMD subset range. v3
uses0oldest/1rest with leaf checks; no production ordering change. GDB absent,
ASAN inconclusive; all six earlier failed/debug stages preserved. Candidate
headers unchanged throughout. Seven standalone diagnostic builds, no estimator
build or real-data optimizer replay. Root failed.json is initial stage only.

Final22:46:26local integrity/exit seal:7052unique inputpins,original6425,
CP100all213/CP98all38/CP95all426/CP91all221terminal files and CP99failure preserved;
Hesai667copies/originals pass, exactowned jobs exited, both containers stopped.
Next separate authorized stage: private Kimera/CBS integration build, wire typed
failure archive, bounded integration regressions before any live deployment.
Do not repeat saved CP99OFF or launch ON automatically; heartbeat remainspaused.

## CP-V100 — offline unchanged-anchor prior conversion,2026-11-03; mechanism demonstrated

User approved offline-first diagnosis, not another live run. See experiments/
prior_conversion_offline_cpv100_20261103/REPORT.txt,analysis.json,complete.json.
Two standalone C++ probes against exact frozen CP76 OriginalChartPrior/CPK9
headers and existing GTSAM; no estimator build, optimizer/sensor/receiver run,
settings/tolerance change or deletion. Known identical original/cache isolates
the converted-cache comparison. At last published CP99 pose1475 with synthetic
SPD Hdiag(1,1,1,1,1e6,1), conversion error1.50812974694e-8 exceeds unchanged1e-8
guard while original/cache and exact-anchor clone errors are0. Same exception
reproduced; failed graph pointers/locks unchanged, real cache corruption rejects.
Finite-difference identity conversion is numerically fragile; not proof of the
actual CP99 failed branch because its matrices/theta were not recorded.

196initialcases and902nearby controls. All35original CP98 native priors pass;
nearby translated saved0/738reject versus56/164synthetic. Larger1000m/1e6m
stress offsets separated, not attributed to real experiment coordinates.
Representative published pose is not failed theta; per-slot cache reconstructed
identical for isolation because CP98 stores aggregate cache only. No GLIM ATE
causality/calibration claim. Exact clone is a diagnostic control, not deployedfix.
Next proposed isolated implementation: exact original Gaussian at exact anchor,
unchanged guard/original-cache comparison and qualified moved-anchor behavior;
add branch-specific failure metrics/matrix capture and offline regression suite.
No live ON/repeated OFF authorized. Final22:15:30local audit6642/6776inputpins,
original6425,CP91/95/98 and CP99failure,Hesai667copies/originals pass; exactowned
jobs exited,both containers restoredstopped. Original/terminal artifacts frozen;
monitor remainspaused. Update this ledger only after separately authorized work.

## CP-V99 — identical-settings repeat,2026-11-03; ON failed, full OFF saved

User authorized ONE additional OFF/ON pair with GLIM,Kimera,concurrentfusion
to assess repeatability. Preparationpassed20:08:51local; read experiments/
eth1_split_imu_repeat_cpv99_20261103/PROTOCOL.txt,PRELAUNCH_REVIEW.txt and live
launch/start/status receipts. Fresh paths/sessions/11611–11612ports only;
same CP95 profiles/private binaries/input/receiver/normal logging, no CP98
recorder or seed tweak. Each346.32s offset0 at0.20x/25sdrain. No scientific
patch/build/install/deletion. BothpersistentON,receivingOFF control,splitIMU
GLIM CPT7/Kimera Alpha,7sWNOA Qcx1. Admission6425pins/Hesai667/109fileparity;
saved source schemas+14negativecases and5synthetic EVOtests passed. Prep-only
missing fixture restored byte-identical, no sensorretry. Originalevidencefrozen.
Finitecontroller OFFcapture/gates thenONcapture/gates,cleanup,Method1EVO0.5mRPE,
verified pairreport then original-vs-repeatcommonmask comparison. No auto retry.
Started ONCE20:10:06local:supervisor19094/start103915,controller19096/start103916;
launcher19093exited,captureOFF19803. Actualcontainerfixture30bit-identicalpayloads
and effectivepreplaybackchecks passed. Full346.32s OFF and all4gates passed
20:40:50local. ON failed at Kimera update1476/currentstate299.498319976s with
`CPK9 promotion changes cached prior`; source coverage failure is secondary.
See FAILURE_REVIEW.txt/failure_post_closeout.json, preserved separately from
immutable original failure/REPORT. Cached augmented-information comparison
exceeds1e-8; actualerror/prior slot/which of2comparisons unlogged. Numerical
tolerance versus substantive inconsistency unresolved. No fullpair/EVO/repeat
comparison exists. No source retry/scoring/tuning/build/install/deletion.
Controller closeout21:07:04local; independent audit verifies17ownedhostidentities
exited,bothcontainers restoredstopped,6425protected/426CP95/38CP98terminalpins
and Hesai667copies/originals. FullOFF/partialON preserved. Experiment heartbeat
PAUSED after failure notification; success-only post_closeout.py not executed.
Two realizations are not a variance distribution or proof of randomness/no bug.
Preserve coverage gaps and full-native/per-paircommon/cross-repeatcommon split;
ONLINE/DELAYEDdistinct; accepted uncertainty/provenance limitations remain.

## CP-V98 — fixed-measurement visual gating, 2026-11-03; mechanism demonstrated

One user-authorized12s ETH1 source-only both-persistent split-IMU ON capture,
unchanged CP91 profiles/private binaries, no fusion/fullrun/tuning/deletion.
Read experiments/eth1_early_native_capture_cpv98_20261103/REPORT.txt,analysis.json,
verification.json and terminal complete.json. Existing recorder saves168native
snapshots across44updates,40insertions eachdirection; allinput/EOF/coverage and
clone-roundtrip gates pass.121stereopairs=3preinit+117processed+1unsupportedEOF
afterlastIMU1.402ms,explicitly retained. Observer costs6.963s total; scheduling
not held invariant. No sensor retry. Preserve initial preparation raw-span
assertion and offline fixturecompile failure;analysis_v2 fixes constructor type.

Saved-only GTSAM contrasts at7.899955s: SAME845smart factors, identical complete
measurements/calibrations/weights, fresh triangulation under CP91 versus CP95
saved MAIN poses. Endpointcontributors16->66of287;52OUTLIER->VALID and2lose
validity. Allvalid136->190,visualHessiantrace73.733M->194.022M(2.63143x).
Pinned3pixel gate tests maximum individual reprojection norm, not average;
ZERO_ON_DEGENERACY makes rejectedfactor informationzero. Thus state change
alone can strongly change native information, without changed observations.
This is not a complete historical decomposition: original CP91/95 smart
measurements were not captured. No proof of unjustifiedconfidence or full-run
GLIM ATE causality. Historical pose-only branches test visualinformation only,
not reconstructed historicvelocity/bias/fullcovariance.

NewcaptureeventCtranslation5.2712e-7m2 (old9.1424e-6,new6.5940e-7). Captured
LOCALgraph independentMarginals+between propagation reproducesall23full6x6
to2.56e-13relative, explicitpeerremoved/nativeinitial0->1preserved/no marginal
prior. Cache-reset Cdifference0.04564% at event,max7.1573%over23updates;
stale cache doesnot explain large drop at this captured event. Initial->final
fixedfactor C doesnot monotonicallyshrink. PIMbiasHatmatchesMAINk-2 exactly
for21intervals; confirms feedback but notfrontendper-callhistory or a biasbug.
All773admissionpins/CP91/95terminaloutputs/CP96/97/Hesai667 preserved, exact
ownedjobs exited and bothcontainers restored. Next useful savedcheck is newly
acceptedtrack and returnedbelief consistency; no weight/threshold change or
furthercapture authorized by this entry. Calibration and ATEcause remainopen.

## CP-V97 — early native Kimera covariance, 2026-11-03; localization of difference, not causal closure

Saved-data only, authorized by user proceed after CP96. See experiments/
eth1_kimera_early_covariance_cpv97_20261103/REPORT.txt and verified supplement.
No source/receiver/optimizer run, tuning/build/deletion or terminal artifact edit.
At7.899955s x22->x23 native LOCAL translationtrace9.14241475e-6->6.59398787e-7m2
(13.86x lower); transported K2G13.58x lower. Audit excluding explicit peer factors
matches first outgoing native6x6 exactly. No marginal prior exists yet; first
retirement8.499951s. First covariance difference4.099975s, live LOCAL factor-count
difference4.499973s424->419, first trace<half5.499967s. Thus absorbed prior history
cannot explain INITIAL drop, while later persistence/correlation remains open.
At detailed event smart factors827->886; endpoint contributors20->70, zero-Hessian
287->217, total touching307->287; visual Hessian trace2.8843x. Native IMU23,
bias23,explicitpeer20 each,ordinary initial-prior Hessian traces equal. Group
traces are not a causal decomposition of marginalized relative covariance.
Independent direct propagation relative error2.12e-14/3.48e-14;36755queries each
allok/no jitter/Schur/tmp_marginals. No numerical scaling/inversion bug shown.
LOCAL graph is linearized at current CBS-influenced estimates,not independent
OFF states. Changed measurements and changed triangulation/linearization remain
confounded. Backend-to-frontend bias feedback is a code-supported possible path;
actual bias-use/complete visual factor observations absent,not a proven cause.
Event mean errors improved,so smaller covariance is not automatically erroneous.
Whole-run GLIM ATE cause remains open. Proposed next discriminating test needs
short~12s passive full native graph/triangulation/bias capture, then fixed-factor
and fixed-state covariance counterfactuals; no run authorized by this entry.
221CP91/426CP95terminaloutputs and3CP96verification manifests preserved. Initial
analysis.json empty-slot caveat resolved in source_review.json: counter counts
nonnull retained factors; detailed sums878/937 corroborate. Keep supplements.

## CP-V96 — saved GLIM ON accuracy investigation, 2026-11-03, diagnosis complete; cause open

User explicitly requested investigating the larger CP95 GLIM ON error rather
than explaining it away as small random variation. Saved-only protocol in
experiments/eth1_glim_on_regression_cpv96_20261103/PROTOCOL.txt. Partition existing
EVO errors, inspect longer-baseline drift and actual incoming raw/aligned beliefs
at exact IE-TC intervals; distinguish numerical variation, matching and causal
claims. No source/fusion/optimizer replay, tuning, build, deletion or terminal
artifact change. All new diagnostic outputs are supplemental, not replacement
CP91/95 results. Completion monitor remains paused.

Read CP96 REPORT.txt,analysis.json,supplement.json,covariance_timeline.json and
three verification receipts. Identical2969timestamps/365pairs; ON ATE .041500
->.053058m,medianAPE .034745->.052468,maximum .135183->.092266m. ExtraSSE90%
in30–90s/150–240s under original global alignment (NOT causal onset). EVO0.5/5m
RPE improves,10/20m worsens,50m improves withonly3pairs; no uniform jitter increase.
1473K2G actual mean/cov joins: raw translationRMSE .007856->.007648m,aligned
.009836->.009609m; no mean-quality deterioration on average. Whole-run median
translationtrace1.20222e-5->1.26278e-5m2 hides matched-event changes:91traces<.5old,
2<.1old. At5.699967s identicalg46->48 trace9.88xlower;7.899955s identicalg68->70
trace13.58xlower. Incoming/inserted covariance parity exact,not scaling added by
insertion. Full-matrix ratios .030707–8.601074; no calibration/causality claim.
First GLIM native difference at.919669590s2.367576mm, beforeCBS. FirstK2G raw
mean/C exact; firstG2K differs, returnedK2G C differs at4.099975s;large C changes
precede first changedbinding8.299952s.15changedbindings are NOT provenATEcause.
HistoricalK16e schedule-dependent sampler still configuredrandom-grid/2threads;
not proof of THISATEdelta. Need nativeKimera information/retained-state check
over5–10s before a controlled frozen-input covariance test; no test launched.
29core inputs/221CP91+426CP95terminaloutputs pass; supplemental hash checks pass.
No source/receiver/optimizer runs,settings/build/deletion/terminal edits.

## CP-V95 supplemental continuation — 2026-11-03, full live pair complete

User approved strict validator schema repair and remaining saved OFF gates,
then only the as-yet-unstarted full ON if gates pass. See experiments/
eth1_split_imu_live_pair_cpv95_20261103_continuation1/PROTOCOL.txt.
Original attempt3 OFF and failure evidence remain unchanged; no OFF rerun,
model/settings/build/deletion or guard relaxation. Full source validator passes
saved CP89OFF/CP91ON plus14negative schema cases. All four saved OFF gates now
pass;202999payloadversions/3456ONLINE,137expired accounted,zeroEOF omissions.
Real saved-data report contracts and5synthetic EVO calls pass.5849pins including
original5744,Hesai667,32unchangedprofile/buildfiles eacharm; entireOFF212291files
plus symlink/FIFO metadata preserved.223.04GBfree>15.51GBreserve.
Launched ONCE15:59:56local:supervisor171100/start1698543,
controller171102/start1698544,launcher171099exited. Completed16:32:57local with
all source/payload/receiver/frame gates,cleanup,EVO and report verificationpassed.
Read continuation1/RESULTS_REPORT.txt,results.json,complete.json and supplemental
post_closeout.json. Two fresh346.32s live arms; this continuation capturedONonly.
SavedOFF212291regularfiles and specialmetadata unchanged; no OFF rerun/replay.

Method1 EVO1.36.3,SE3/no-scale,reference-distance0.5mRPE,physicalAP20prism point.
Six-curve common100ms mask2969queries/296.6s(85.643%),365RPEpairs,ref201.3745m.
OFF->ON values (ATE m;translationRPE m;rotationRPE deg;estimatedpath m):
- GLIM .039574->.053058; .018377->.014303; .352970->.329290;202.9502->202.4156.
- Kimera .738453->.198778; .085253->.035072; .619426->.527284;211.0250->201.8619.
- ONLINE fusion .061500->.118581; .037904->.035882;1.823789->1.729998;216.1807->213.5990.
- DELAYED7s-finalized separately .056414->.116078;.013384->.011685;
  .337977->.318803;201.4344->201.8687;not ONLINE or full-history.

ATE improvement GLIM-34.07%,Kimera+73.08%,ONLINE-92.82%,DELAYED-105.76%.
Full-native ATE GLIM .038510->.051503,Kimera .744750->.198362,
ONLINE .059788->.113615,DELAYED .053513->.110691m;different masks,not substitutes.
Native rows3454GLIM/1493->1492Kimera/3456fusion.11Kimera gaps>0.5s preserved;
common mask ends301.9446s (unsupportedtail44.3754s). Missingright203.898841154s
and25preinitversions each retained as omissions; no gap filling.
ON1473persistentinsertions eachdirection; receiver202999->202925versions,
197911->197837replacements,137->138expired accounted,unsupportedEOF/pending/
duplicates0;3456ONLINE+finalizedeach. Frames:3452GLIM,1492->1491Kimera wirejoins;
1473ONreceive means eachdirection. Covariancepayloaderror0,wirecov<=5.48e-16.
Kimera previous-state revision p95 .113872->.022887m,max .748554->.162806m,
>0.5m3->0;support-cyclep95 .074191->.073848s;Kimeraoptimizep95 371.135->387.936ms.
Wiremedianrotation/translationtraces GLIM9.550e-9->9.473e-9rad2/
3.439e-6->3.421e-6m2;Kimera6.953e-8->5.759e-8rad2/1.095e-5->1.235e-5m2.
NativeKimera all-version mediantranslation4.209e-6->2.045e-6m2;transported
4.263e-6->2.065e-6m2. Wire/all-revision populations differ;notmatched-age
calibration proof. GLIMnative matrix parity not separatelylogged. No universal
accuracy gain; persistent peerfeedback/native-onlyprovenanceFALSE,experimental
Alpha noise remain accepted limitations. IE-TCsharesCPT7,not independentprismGT;
path is estimatedpolyline,notphysicaltravel;incrementalparity is notstationarity.

Final48directEVO CLIarchives verified. Post-closeout5849pins/original5744,
426terminaloutputs/45reportinputs,CP89all148/CP91all221/CP84all187,Hesai667copies/
originals,allpriorfailureevidence andsavedOFFunchanged. Exactownedjobs/controller/
supervisorexited,bothinitially-stoppedcontainersrestoredstopped. HeartbeatPAUSED.
REPORT.txt is now immutable (complete.json hash); newauditsmustbesupplemental.
No newrun/retry/tuning/build/deletionauthorized; await user'snextdecision.

Read-only comparison with CP-V91 after user's repeatability question:
old/new common reference.tum and RPE pair_indices.json are byte-identical,
all2969query timestamps match; OFFKimera TUM is identical. ON actual16GLIM+
14Kimera profile files and2privateKimera DSOs match; effective scientific
settings match, GLIM private-library paths match. GLIM OFF differs too
(.038062->.039574m); GLIM ON ATE .041500->.053058m while tRPE improves
.015195->.014303m. ON native GLIM differs before the first peer insertion.
1473K2Ginsertions each,identical sender IDs but15different localGLIM edgepairs;
first k24->k25 maps oldg73->g75 vsnewg72->g74,~8.2s/~100ms shift. G2K endpoint
IDs identical,5application-update headers differ. Duration-aware matcher uses
currently available states. This is concrete non-repeatability evidence,not
proof that concurrent fusion load or those15bindings caused the ATE difference.
41CP91/57CP95selected score/reference/pair artifacts still match terminalhashes.
No rescore/source/receiver/configuration/terminalartifact change; next proposed
only: saved-log onset/arrival/edge-binding attribution before tuning or newruns.

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

**CP-V95 — fresh full ETH1 split-IMU live OFF/ON + fusion (2026-11-03; ATTEMPT3 VALIDATOR FAILURE, OFF CAPTURE SAVED):**
User explicitly selected fresh all-three live pair after CP94 missing-timing
preflight; not saved-stream replay. Fresh experiments/
eth1_split_imu_live_pair_cpv95_20261103/PROTOCOL.txt freezes CP89/91 sources,
CP-K19 receiver and comparison. Integrate/qualify only the split-body transport
conversion, preserve physical endpoints and actual observed arrivals/versions.
Sequential full346.32s OFF gates then ON at0.20x; no tuning/build/retry/deletion.
Persistent on both sides for ON, receivefalse both OFF; unchanged Qc x1.0.
Method1 EVO common six-curve100ms mask and full-native metrics, reference0.5mRPE,
estimated path, covariances/revisions/timing and explicit omissions. Correlation
and experimental Alpha noise limitations remain. Original infrastructure startup
stopped before master/source/receiver due transient shell curl update in newly
started container. No sensor capture. All5285pins/Hesai667passed,bothcontainers
restored,supervisor126690/controller126692exited. Failed evidence preserved.
Fresh roots end_attempt1,ports11597/11598,only<=30s startup-quiescence wait added;
same idle allowed set,curl not whitelisted. No source/model/settings change.
Attempt1 stopped pre-master/source/receiver at Noetic adapter import:missingSciPy.
Cleanup5432pins/Hesai667passed,bothcontainersrestored,exactjobs131402/131404exited.
Both failed roots preserved. Attempt2 fresh paths/ports11599/11600 uses equivalent
adjoint mean/full6x6 conversion with fixed scalar sums,noSciPy;all saved versions/
wire parity passed,actualcontainer fixture parity required before any playback.
Attempt2 passed Noetic fixtures then host receiver --check failed on missingNumPy.
Only owned ROS master ran/exited0, no source/receiver started.5581pins/Hesai667
passed,bothcontainersstopped,supervisor135545/controller135547exited. Preserveall3.
Attempt3 freshpaths/ports11601/11602 standard-library-only equivalent frame math;
require exact host system-Python fixture/CLI imports plus actualNoeticfixture.
No source/model/noise/weight changes. Read attempt3 PRELAUNCH_REVIEW.txt,REPORT.txt.
Admission passed5744protected/Hesai667,unchanged32profile/buildfiles eacharm,
227.69GBfree>15.51GBrequired. Exact host CLI/fixture and actual Noetic fixture
passed. StartedONCE15:07:01local:launcher140870exited,supervisor140871/start1381076,
controller140873/start1381077. OFF actual playback with all3 verified; source
effective split-IMU/receivefalse gate passed, receiver consumed both native
streams and published online poses, earlyqueue0. No full result yet. OFF gates
precedeON; controller exclusively owns all stages, no duplicate work/retries.
Experiment-only heartbeat monitors attempt3, quietuntilterminal/requiredaction.
Terminal15:37:49 after full346.32s OFF capture completed/source+receiver exited0.
First saved source validator failed at line41: OFF external_injections integer0
treated as dictionary(.values()). Harness schema bug,not an estimator crash.
OFF coverage passed all34618CPT7/69186Alpha/3463pairs;3455GLIM/1493Kimera rows,
actual sparse tail retained. Receiver completed202999versions and3456ONLINEposes,
duplicates0,maxqueue47. Remaining gates/EVO not run; ON never launched. No full
pair/scientific result. Saved OFF and failure evidence preserved without edits.
Cleanup and independent read-only audit pass5744protected,148/221/187prior
terminal hashes,Hesai667copies/originals. All exact owned identities exited,
both containers restored stopped. Supplemental MONITOR_FAILURE_REVIEW.txt.
Monitor paused after terminal notification; no repair/revalidation/replay.
Next proposal: fix validator schema in new supplement,validate savedOFF through
remaining gates,then ON continuation; do not repeat already-saved OFF capture.

**CP-C7 — pre-Kimera-refresh run cleanup (2026-11-03; COMPLETE, 30 PERMISSION LEFTOVERS):**
User explicitly authorized deleting obsolete runs preceding the linearization
refresh. Boundary CP-V38/39/40; preserve CP-V35+ fix evidence and all subsequent
runs, all datasets/Livox, code/build/runtime dependencies and compact reports.
See experiments/pre_refresh_cleanup_cpc7_20261103/REPORT.txt,completion.json.
Permanently removed10,292,155generated files and10,217,254empty directories
across214old roots:170.059GiBfile allocation,208.704GiBnet free-space gain,
213.741GiBavailable atclose. NotTrash/no newbackup. No broad recursive deletion,
fusion/source launch,build or run-settings change. Unknown/dependent files kept.
First permission failure and journals preserved; supplemental resume_permissions/
reconciled565prior deletions and removed10,291,590remaining writable targets.
30files/131.082MiB retained due permissions;BLOCKED_FILES.txt is exact list.
No ownership/permission changes or administrative workaround. Completion passes
for accessible targets,but all_planned_files_deleted=false. Never restart tools.
Verified328905retained identities,7644dataset entries,6273protected hashes,
Hesai667copies/originals,CP89all148/CP91all221terminal outputs/complete hashes.
ExactPID84337/start696018and4workers exited;both containers stopped and no
host-visible experiment/open old-run handles(58processesinaccessible).
214root decommission notices and checkpoint supplement supersede historical
bulk-retention claims without rewriting original reports/terminal evidence.
CP-V94 storage blocker resolved;timing/method choice still open,no fusionstarted.

**CP-V94 — split-IMU ETH1 fusion replay preflight (2026-11-02/03; PREFLIGHT COMPLETE, REPLAY NOT LAUNCHED):**
User approved validation followed by fusion-only OFF/ON replay of saved CP89/
CP91 sources,identical receiver settings and preserved recorded timing. No
source rerun/tuning/deletion authorized. Use fresh experiments/
eth1_split_imu_fusion_cpv94_20261102. Validate native Alpha->CPT7 full relative
mean/covariance conversion;do not feed Alpha-native log rows as CPT7. Check
full revision availability/order and storage before any optimizer launch.
CP-K19 seven-second source_wnoa,Qc x1,100ms independent supports,Gaussian1,
seal_at_absorption,explicitEOF remain intended receiver contract. No fusion
has been launched. Missing recorded transport timing or insufficient reserves
must not be silently replaced with inferred times or weaker storage guards.
Read REPORT.txt andpreflight.json. Standalone native-log Alpha->CPT7 candidate
passesall36778OFF/36755ONKimera rows,roundtripcovrelative<=9.01e-16;
same-version parityall1492OFF/1491ONwire messages<=5.48e-16cov/2.33e-16mean;
numericaladjointJacobian2.31e-9. Candidate NOT installed in production adapter.
CP89/91 contain no fusion events/receipts/full-revision arrival journal. Bags
hold3452GLIM+1492OFF/1491ONKimera native wire publications(allrev0,including
repeatedintervals),not full diagnostic revisions. Exact receiver-observed timing
and intersource tie order cannot be recovered because no receiver ran.
Need user choice: new matched live pair,explicitly reconstructed-timing replay,
or changed native-wire-only input policy. Do not select silently. Storagealso
fails:7.123GiBfree vs10.787GiB conservative receiver-pair budget (5GiBreserve),
3.664GiBshortfall. Fresh live pair requires larger forecast. No deletion or
guard relaxation.148OFF+221ONterminal hashes intact;no fusion results. Initial
preflight numpy-bool serialization failure preserved;only read-only helper
repeated,not source/receiver. Await user direction before expanding scope.

**CP-V93 — actual inserted external-belief covariance audit (2026-11-02; COMPLETE):**
User clarified the target is the full covariance of external factors inserted
GLIM->Kimera and Kimera->GLIM, not the later native revisions studied in CP92.
Saved CP91 attempt1 insertion matrices/receipts and CP89 OFF transmitted beliefs
only; no source run, optimizer, build, tuning, container, deletion or rescoring.
Fresh experiments/eth1_inserted_covariance_cpv93_20261102. Join actual applied
factors to time-alignment/frozen/source records, verify runtime body conversion
and scaling, summarize separate rotation/translation traces and eigenvalues over
time. Compare identical source intervals with OFF exports in the receiver body;
OFF exports are controls, not OFF insertions. Preserve terminal artifacts. Trace
decline alone does not establish miscalibration or the cause of feedback.
Read REPORT.txt,analysis.json,verification.json and inspected trace/ratio plots.
Actual1473each direction;full matrices equal alignment/frozen/transport records;
wire->applied parity after actual body transform maxrelative6.38e-16G2K/0K2G.
All SPD,scale1,healthawareOFF. Forty G2K information checksC*Omegaerror9.87e-15;
no corresponding K2G information records. G2K actual medianrotation/translation
traces1.00866e-8rad2/3.59946e-6m2;K2G5.77873e-8/1.20222e-5. Approx0.20s
relative-motion factors,NOT global trajectory covariance.1473G2K/1466K2G exact
OFF physical-interval controls;6ambiguousOFF versions and1missingK2G excluded
only from paired statistics,not1473ON. Median ON/OFF trace ratios rotation/
translationG2K.999472/1.007178,K2G.813271/1.019091. No universal insertion
collapse;K2Grotation smaller in180--240s.13K2G translation ratios<.1 at16.6,
183.6--184.5,256.6--259.3s,minimum.024676at184.2196s amidOFFspike. Trace alone
cannot classify as overconfidence. Actual support~3.8--302s;no tail fabrication.
No unintended extra shrinking at insertion;source uncertainty may still be
miscalibrated. Mean-only timestamp covariance approximation/peer correlation
unresolved. CP92 later-native-revision shrinkage is a different population.
19inputs/148OFF+221ONterminal hashes unchanged;independent rawlog/Cholesky/trace
checks pass. Only fresh compact supplemental artifacts;no source run or tuning.

**CP-V92 — ETH1 saved covariance consistency audit (2026-11-02; COMPLETE):**
User asks whether CBS-ON covariance decreases more than justified. Saved CP89
OFF and CP91 attempt1 ON only. No source/optimizer/container run, settings change,
weight selection, deletion or trajectory rescoring. Fresh analysis directory
experiments/eth1_covariance_audit_cpv92_20261102. Compare transmitted beliefs on
identical physical endpoints, then same endpoints and actual source-update ages
for revisions. Right-local full6x6 CPT7-body residuals against IE-TC, separate
rotation/translation, uncertainty/error distributions and time bins. Preserve
all originals. Normalized reference discrepancy is not true NEES/NIS because
reference covariance/correlation is unknown; adjacent edges are not independent.
Analysis and independent verification complete;REPORT.txt,analysis.json,
selected_records.npz,verification.json and inspected covariance_revision_age.png.
Wire common3452GLIM/1473Kimera;fixed-age3411/1435matched intervals,actual ages
GLIM~1.001/2.001/4.002s,Kimera~1.200/2.200/4.200s,identical OFF/ON update times.
Wire Kimera17repeated edges per mode,earliest retained forfirst cohort;2OFF/1ON
unmatched edges. Native age selection excludes162OFF/185ONrows lacking a logged
update time;all retained forwire parity. No sparse-tail belief fabrication.
First paired median ON/OFF covariance traces GLIMt/r1.00459/1.00134,Kimera
1.01789/.81396;not universal newly transmitted covariance collapse. Mature~4s
GLIMt/r1.00466/1.00132,Kimera.36995/.66020. Mature Kimera translation EVOerror
9.0079->5.1145mm,rotation.143559->.142258deg;covariance confidence increases
more than reference accuracy supports,especially rotation and revision age.
Within same1435edges,KimeraONfirst->4.2s mean translation variance-93.69% vs
error2-58.59%,normalized aggregate mismatch6.56x worse;OFF-79.57%/-40.02%,2.94x.
ONrotationvariance-43.73%,error2+8.62%. ON2.2->4.2s translationvariance-68.37%
whileerror2-0.020%;EVOrelativeRMS5.1150->5.1145mm. This is the clearest plateau.
95%nominal marginal inclusion mature KimeraOFF/ONt23.14/11.50%,r8.50/2.58%;
not true NEES/NIS because unknown reference covariance/cross-correlation.
All13inputs/148CP89+221CP91terminal artifacts unchanged;whitening via independent
eigendecomposition agreeswithin3.4e-12,SE3/matching/synthetic/frame tests pass.
No parameter change or estimator/fusion run. Proposed next gate ONLY:localize
2.2--4.2s shrinkage vs marginalization and persistent-peer prior provenance.
Cannot conclude permanent factors are unique cause,choose a scale,or certify
uncertainty from this single correlated-reference experiment.

**CP-V91 — full ETH1 split-IMU persistent CBS-ON (2026-11-02; COMPLETE):**
User approved CBS-ON continuation of CP89. One full346.32s ETH1 source-only
run at0.20x/offset0/drain25s; GLIM Hesai+CPT7 and Kimera stereo+Alphasense,
BOTH persistent, Gaussian1. No fusion or changes to CP89 noise, priors,
extrinsics, timing, initialization, private binaries or source algorithm.
Fresh experiments/eth1_split_imu_persistent_on_cpv91_20261102_attempt1,port11592.
Existing Alpha/CPT7 relative-pose/adjoint covariance bridge must pass actual
saved-payload checks and preplayback effective-frame checks. Live saved logs
will check both receive paths. Preserve CP89/90 and all prior pins/Hesai667.
Single finite controller, exact input/coverage/numerical/lifecycle/storage gates,
container restoration and saved-only Method1 EVO1.36.3 vs IE-TC, common OFF/ON
mask and full-native coverage separate. No retry/build/tuning/deletion/default
promotion. Mixed-prior feedback limitation remains accepted, not repaired.
Attempt0 startup guard failed18:18:00 before sensor playback: copied diagnostic
path off_source/e1b_profile instead of on_source. Cleanup18:18:30passed4789pins/
Hesai667,bothcontainersstopped,exact supervisor139781/controller139783 exited.
Zero raw playback/recorders/initialized native states; failure evidence retained.
Attempt1 changes only diagnostic destination plus fresh paths/port11592. Launched
ONCE18:24:53local,supervisor142175/controller142177 proc_start3120104;
launcher142174 exited. Admission4940protected/Hesai667,10.897GBfree>10GiB,
3.830GBforecast. Startup actual topics/body/TF/private DSOs and both persistent
receive=true passed BEFORE raw playback.
Preflight1492Kimera/3452GLIM saved payload frame parity passes; Kimera full6x6
covariance relative error5.48e-16. Completed18:55:39;post_closeout18:57:19passed.
Exact owned jobs/controller/supervisor exited,both containers restored stopped;
4940protected/221terminal/33report-input hashes,Hesai667copies/originals pass.
Both persistent live routes1473insertions;3452GLIM/1491Kimera wire means match,
same-version Kimera full6x6 covariance relative error4.70e-16. Incoming1473raw
means each direction pass;not proof of unlogged final factor covariance or
uncertainty calibration. Mean-only timing covariance approximation unchanged.
Method1 EVO1.36.3 SE3/no-scale vs IE-TC,reference-distance0.5m RPE: common2969
100ms queries/365pairs,296.6s/346.32=85.643%,reference201.374540m. GLIM ATE
OFF->ON.038062->.041500m(-9.03%),t/rRPE.016331->.015195m/.336865->.333948deg,
path202.879510->202.624335m. Kimera ATE.738453->.209054m(+71.69%),t/rRPE
.085253->.032740m/.619426->.587296deg,path211.024963->202.001747m. Saved
CP89 OFF rescored on four-curve mask(one fewer query),not new matched capture.
Full-native separately GLIM3454poses both/99.705% support,ATE.037891->.043435m,
path205.236377->205.054666m vs201.475912m. Kimera1493->1492poses,85.816->85.787%
support,ATE.744750->.208661m,path214.718296->202.447146m vs201.020155/201.019635m.
Eleven Kimera gaps>0.5s each arm,sparse~5.1s tail after302s;no gap chords.
Wire median covariance rotation/translation traces GLIM9.4246e-9->9.4730e-9rad2/
3.4415e-6->3.4563e-6m2,Kimera6.9534e-8->5.7549e-8rad2/1.09494e-5->1.15966e-5m2.
All-native Kimera revisions instead median5.4336e-8->3.7305e-8rad2 and
4.2085e-6->2.2106e-6m2;do not claim all covariances unchanged or independence.
Retained translation revisionsp95/max.113872/.748554->.022934/.151270m;
rotationp95 improves.304137->.254647deg butmax increases1.07254->2.56958deg.
Optimizationmean/p95187.025/353.724->200.853/372.454ms;not1xqualification.
Raw3462Hesai/34618CPT7/69186Alpha and3463stereo pairs conserved;one missing
right203.898841154s retained. GLIM3455internal poses/3454saved;no gap filling.
Alpha bias model remains experimental;IE-TC sharesCPT7,not independent prismGT;
path is estimated polyline,not physical travel;increment parity not nonlinear
stationarity. No fusion. Read RESULTS_REPORT.txt/results.json/frame_validation.json/
post_closeout.json. Mutable REPORT.txt updated;immutable outputs preserved.
Experiment-only heartbeat paused. No extra run,tuning,retry or default promotion.

**CP-V90 — ETH1 AP20 saved-data position comparison (2026-11-02; COMPLETE):**
User approved additional raw Leica/AP20 check of CP89 GLIM+CPT7 and Kimera+Alpha.
Scope saved data only,not rerunning estimators or changing reference in old
tables. Fresh experiments/eth1_ap20_saved_comparison_cpv90_20261102/PROTOCOL.txt.
Common raw AP20 timestamps,declared bounded interpolation,globalSE3/no-scale
EVO1.36.3 APE;no AP20 orientation metric. Same-mask IE-TC supplemental check;
report actual partial coverage and no-gap paths. Preserve all CP89/Hesai pins.
No launch/build/tuning/CBS/fusion/default promotion or extra sequence.
Saved-only evaluation completed with5EVO1.36.3 CLI/API-parity comparisons and
34terminal artifacts pinned. RawAP20 same3873timestamps:GLIM ATE.031473509m,
Kimera.684861892m;exact-mask IE-TC.032813967/.686583358m. IE-TC vsAP20.010988039m.
Thus choosing IE-TC does not explain this positional gap on the observed mask;
not proof of full-sequence independence,noise calibration or Kimera failure cause.
Common194.514120s/346.32=56.166008%,13segments including3singletons;rawAP20
228.310992s=65.924865%.674omitted rawrows(72startup/602tail),no gap filling.
Common segmented paths AP20/GLIM/Kimera/IE152.455646/153.293176/156.854371/
152.383621m,not full-route physical travel. Native3454/1493poses and99.705%/
85.816% support remain separate. No attitude/RPE from position-only AP20.
Supplemental audit reconstructs all accepted brackets and matches rawIE-prism
to old CP89 references within1.5e-14m. Pre/post3716protected/148terminal/28input
hashes,Hesai667copies+originals pass;both containers stopped. Prior results,
calibration and estimator outputs unchanged. Read experiments/
eth1_ap20_saved_comparison_cpv90_20261102/REPORT.txt,results.json,verification.json,
supplemental_verification.json and complete.json. No further experiment launched.

**CP-V89 — full ETH1 Alpha bias-RW CBS-OFF validation (2026-11-02; COMPLETE):**
User approved the full ETH1 follow-up to CP88. One346.32s/offset0 capture at0.20x,
25sdrain, GLIM Hesai+CPT7 and Kimera Alpha; both receiveOFF,no fusion/ON.
All CP88 profiles/binaries unchanged,including biasRW.0043/.000266. Fresh root
experiments/eth1_alpha_bias_rw_full_off_cpv89_20261102,port11590. Read PROTOCOL.
Input/storage/identity gates before launch; source-only,one finite controller.
No retry,tuning,build,deletion,default promotion or additional sequence.
Launched ONCE15:58:08local:supervisor45570,controller45572 proc_start2239673.
Admission3716protected/Hesai667,12.417GBfree,3.545GBforecast passed. Raw full
audit34618CPT7/69186Alpha/3462Hesai,3464left/3463right;one right missing at
203.898841154s explicit,3463matched pairs agreesCP-K23. Inputs unchanged.
Prelaunch helper issues preserved,not source failures/retries. Startup verifies
both receiveOFF and actual profiles/body/topics/current DSOs before playback.
Completed16:28:43local; once-only supplemental post_closeout16:29:51 passed.
Owned controller/supervisor/stage identities exited,both containers stopped;
3716protected/148terminal/28report-input hashes,Hesai667copies/originals PASS.
All raw IMU stamps conserved;3463pairs=3preinit+3460processed+0EOF,zeroexternal
injections. One unmatched left retained. No source retry/extra capture.
Method1 EVO1.36.3 SE3/no-scale;reference-distance0.5m t/rRPE. Common100ms:
2970queries,296.7s support/346.32s (85.672%),365pairs,reference201.375536m.
GLIM ATE.038060m,t/rRPE.016331m/.336865deg,path202.881834m;Kimera ATE.738754m,
t/rRPE.085253m/.619426deg,path211.027943m. Full-saved native separately:
GLIM3454poses/99.705%support,ATE.037891m,t/rRPE.016919m/.254654deg,
path205.236377vs201.475912mreference;Kimera1493poses/85.816%support,
ATE.744750m,t/rRPE.119927m/.509363deg,path214.718296vs201.020155mreference.
Kimera11gaps>0.5s;after302s ~5.1s sparse keyframes,not dense full-time output.
No gap chords included in evaluated path. Native coverage reports3455GLIMposes,
but3454saved ROS poses available (one early native pose absent from CSV).
Exact CP88 prefix561queries/46pairs reproduces ATE.144141m,t/rRPE.074174m/
1.002433deg,path26.190106vs25.304035m;full-window alignment not pilot alignment.
Kimera previous revisionp95/max.113872/.748554m,3>0.5m,0>1m;optimization
mean/p95/max187.025/353.724/464.763ms. Logged36778covariances finite/symmetric/
SPD;median rot/trans traces5.433613e-8rad2/4.208529e-6m2,not calibration proof.
15EVO CLI calls/API parity0;all reports verified. Successful full acquisition,
not a proof of health or a full matched CPT7 control. Bias histories/physical
IMU residuals unavailable;split-body CBS/fusion unqualified. IE-TC sharesCPT7,
not independent prismGT;estimated path not travel;increment parity not nonlinear
stationarity. Completion notified;experiment-only heartbeat now PAUSED. No next
experiment authorized. Details in H/REPORT.txt and immutable results.json.

**CP-V88 — ETH1 Alpha bias-RW-only OFF60 ablation (2026-11-02; COMPLETE):**
Started ONCE15:24:18local: supervisor37766/controller37768,proc_start2036654.
Preparation passed3368protected hashes,Hesai667 and12.657GBfree; no duplicate.
User approved CP87's single60s test. Read experiments/
eth1_alpha_bias_rw_off60_cpv88_20261102/PROTOCOL.txt. Fresh CP85 Alpha configuration,
ONLY accel/gyro random walk changed to.0043/.000266; retain200Hz, white noise,
extrinsics,timing,initialization/priors,CP76/CP40/native25states and all binaries.
GLIM remains Hesai+CPT7, bothCBSreceiveOFF,0.20x/offset0/drain25s,no fusion.
EVO Method1 common masks and full-native separately against saved CP85/86.
No sweep/retry/build/calibration correction/deletion/default promotion/full run.
Single finite controller; unique port11589,10GiB admission/5GiB runtime reserve.
Completed15:30:56,post-closeout15:31:57:owned jobs exited,both containers stopped;
3368protected/104terminal/34report-input hashes,Hesai667copies/originals passed.
CommonEVO mask561queries/56s/46distance0.5mRPEpairs exactlyCP85/86. Alpha
ATE23.192259->0.144141m,tRPE13.281672->0.074174m,path1323.428703->26.190106m
vs25.304035reference;rotationRPE0.932099->1.002433deg did NOT improve. Healthy
CPT7controlATE0.047226m remains better. New GLIMATE0.030953m(no settingschange).
Revisionp95/max37.122535/75.824811->0.054173/0.359724m;0>0.5m vs128before.
NativeAlphaATE0.144665m/path26.742070vs25.334488reference,284poses56.399676s
support;startupgap3.199979s not bridged. Exactrawreceipts;601=3+597+1EOFcamera.
Strong bias-model sensitivity evidence,not Alpha calibration/independence proof;
current split-body belief/fusion still requires qualification. No defaults
promoted. Proposed next ONLY longer/full ETH1 CBS-OFF validation; no new run.

**CP-V87 — ETH1 split-IMU saved-data diagnosis (2026-11-02; ANALYSIS COMPLETE):**
User requested checking GLIM/HG4930 + Kimera/Alphasense integration. Read
experiments/split_imu_diagnostic_cpv87_20261102/PROTOCOL.txt. Inspect CP85/86
saved inputs, routing, calibration, timing, noise, initialization and failure
onset. Supplemental analysis only; no source run, settings change, build or
CBS/fusion integration authorized. Preserve both terminal records.
Authority REPORT.txt,analysis.json,supplement_1.json,supplement_2.json and
verification.json in that root. No new EVO scoring; existing CP85/86 scores.
284 identical native timestamps; Alpha previous-state revision first>0.1m at
18.899898s,>0.5m at20.899888s,>1m at22.899877s; CPT7 never>0.1m. Alpha native
post-update IMU objective grows>10 at6.499962s, then14907.369/15300.162 total
at20.899888s. Weighted factor errors, not physical residuals or stationarity.
Actual Alpha routing/mission-TF composition/units/header-delta/density-squared
mapping checked; no obvious axis swap or integration-rate bug. Small physical
extrinsic/timing errors remain open. Both receiversOFF, no fusion; no belief
insertion can explain this native failure. Future split-frame adapter unqualified.
Alpha bias random walks imply9919x accel/20294x gyro bias-between information
relative to healthy CPT7, despite larger white noise. Leading ablation hypothesis,
NOT proven root cause or hard-locking absolute bias. Initial bias priors same.
Reconstructed0.2..0.3s initialization uses near-gravity identity branch even with
round0 (Alpha1.99859deg/CPT1.00999deg); initial tilt/bias ambiguity remains.
Raw Alpha at20.655504/51.635929s reaches-39.2m/s2: possible clipping, not proven
firmware range. supplement_2 corrects supplement_1's broad quantization claim:
only10.24% pure integer counts,99.9916% tenth-count grid. Divergence begins earlier.
Lever-arm-adjusted5Hz acceleration residualmedian.300948/p951.320592m/s2;
gyro correlations>.994. No fitted calibration/scale/filter applied to estimators.
Proposed ONLY (not launched): one matched60s Alpha CBS-OFF bias-random-walk-only
ablation to existing CPT7 values(.0043/.000266), all else fixed; not production
noise calibration. Actual bias trajectories/unwhitened IMU residuals missing.
No full sequence, CBS-ON, source retry, build, tuning, or default promotion done.
Final verification PASS:39 analysis-manifest entries,268 CP85/104 CP86 immutable
terminal output hashes, all667 Hesai checkpoint copies and originals unchanged.

**CP-V86 — ETH1 matched CPT7 control (2026-11-02; COMPLETE, CPT7 HEALTHY):**
User approved single60s source-only CBS-OFF current-binary control for CP85.
Authority: experiments/eth1_cpt7_control_off60_cpv86_20261102/PROTOCOL.txt.
Same cameras/Hesai/input window0..60s,0.20x,25sdrain, current CP82/CP76+CP40.
Restore validated CPT7 topic/body, camera T_BS and IMU rate/noise only; preserve
backend/initialization/lag and both receive=false. GLIM runs for matched load,
Alpha stream still played/recorded but Kimera consumes CPT7. No fusion/ON/full.
No2.4deg/3ms fitted corrections, tuning, build, deletion or default promotion.
One new root/port11588; pin CP85/Hesai originals;10GiB/5GiB storage guards.
EVO Method1 common100ms intersection with saved CP85, native cadence separately.
Question: does healthy CPT7 control isolate the unhealthy Alpha integration?
It does not by itself isolate extrinsic versus noise versus physical hardware.
Admitted3017protected files/Hesai667;13.066GBfree above10GiB guard. Launched
11:15:36local supervisor10073/controller10075(proc_start544448). Actual startup
verifies shared-CPT7 subscriptions/body/private builds and both CBS receiversOFF.
Controller owns all stages; no parallel validation/scoring or protected edits.
Terminal11:22:13; post_closeout11:22:58 passes3017protected/104terminaloutputs/
29reportinputs/Hesai667. Exact owned jobs exited,bothcontainers restored stopped.
One60s capture,601camera pairs=3preinit+597processed+1EOF1.569425ms beyondCPT7.
BothrawIMUheader streams exact;zeroCBSinsertions.284Kimera/591GLIMoutputs.
EVO Method1 common100ms identical to CP85:56.0s/60,561queries,46delta0.5m
referencepairs. NewKimeraCPT7 vs savedAlpha ATE.047226/23.192259m,
tRPE.025292/13.281672m,rRPE.665586/.932099deg,path25.219278/1323.428703m
vs25.304035m reference. NewGLIM ATE.045248m,path25.788650m. Oldscores reused.
Fullnative Kimera ATE.047371m,tRPE.024554m,rRPE.502632deg,path25.489573m
vs25.334488m reference,56.399676s no-gap support. Startup3.199979s gap excluded.
Fullnative GLIM ATE.047154m,path26.175111m vs25.395797m,58.999369s support.
Kimera previous-state revisionp95.017472m/max.078017m vsAlpha37.122535m/
75.824811m;0vs128revisions>0.5m.282pairedsteps,missingdiagnosticID2 retained.
Shape revisionp95.012718m/max.055496m;optimizationmean92.067ms/p95123.632ms.
6750logged covariances retained,medianrotationtrace1.6677065e-8rad2/translation
6.7350221e-7m2;notcalibrated.12newEVOCLI calls,APIerror0,TUMerror1.39e-15.
Conclusion: healthy CPT7 current-backend result isolates Alpha integration or
configuration-dependent interaction; not a hardware/extrinsics/noise root cause.
No2.4deg/3ms correction applied,noCBS-ON/fusion/full run/build/tuning/deletion.
IE-TCsharesCPT7,notindependentprism;pathnotphysicaltravel;no stationarity claim.

**CP-V85 — ETH1 separate-IMU qualification (2026-11-01; FINISHED, UNHEALTHY):**
User approved GLIM CPT7 / Kimera Alphasense pilot. First gate: one60s CBS-OFF
source-only run,0.20x/offset0/drain25s, current CP82/CP76+CP40/CPK9 binaries.
ETH1 camera transforms change to author Alphasense body;200Hz and author Allan
noise/bias parameters replace CPT7 values, without trajectory-based tuning.
No fusion/ON/full run yet; adapter body-conversion qualification remains needed.
Preserve raw inputs, all earlier results, gap/clipping evidence and frozen defaults.
Read experiments/eth1_split_imu_off60_cpv85_20261101/PROTOCOL.txt and REPORT.txt.
Original18:57:09 launch failed before sensors: GLIM shared_native rejects the
copied observe_only OFF selector. No source-quality result; no data replayed.
19:02:24closeout passed2544pins/Hesai667, containers stopped, exact owned
supervisor210675/controller210678(start10058425) exited. Evidence preserved.
Fresh *_attempt1 launch-only repair restores intended inject_persistent selector
with BOTH receive=false, new paths/port11587. All profiles/binaries/calibration/
weights unchanged. Read LAUNCH_REPAIR.txt; no numerical/source retry.
Finite controller owns capture->restore->saved-only EVO; no stage duplication.
Attempt1 launched19:09:17local, supervisor223871/controller223873(start10131236).
2671protected/Hesai667 admission passes,12.60GBfree>10GiB. Do not edit pinned
files or launch anything competing; original failed attempt remains preserved.
One actual60s playback finished, no source retry; restored containers19:15:45.
Original frontend-count gate failed597<598. Saved endpoint audit reconciles all
601received pairs:3before initialization,597processed,1EOF1.4386ms after last
Alpha IMU.5986CPT7/11972Alpha recorded stamps equal raw inputs element-for-element.
All12EVO CLI scores executed once; inherited diagnostic loader then rejected
Alphasense frame. *_attempt1/report_completion_1 explicitly parses native Alpha
for revision audit and verifies existing EVO archives without re-running CLI.
Final authority there: RESULTS_REPORT.txt/results.json/report_verification.json,
post_closeout.json/complete.json; original strict/report failures preserved.
Method1 common100ms56.0s/60 (93.333%), reference prism path25.304035m:
GLIM/Kimera ATE.047968/23.192259m,0.5m tRPE.032890/13.281672m,
rRPE.545846/.932099deg; estimated paths25.843993/1323.428703m.
Full-saved native ATE.047135/24.126003m,tRPE.031695/22.599541m,
rRPE.477510/.861409deg, paths26.237749/1704.070396m; reference25.395797/
25.334488m,591/284rows,58.999369/56.399676s no-gap support. One3.199979s
startup Kimera keyframe gap excluded from common/native no-gap path support.
Previous-state revisionsp9537.122535m/max75.824811m,282pairedsteps/1missingID2;
shape revisionp955.666327m/max10.102559m. Optimize mean159.646ms/p95205.340ms.
6750logged covariances: median rotationtrace2.58800e-8rad2,translation9.23109e-4m2;
no uncertainty calibration claim. EVO CLI/API error0, source/export error1.44e-15.
Operational separate IMUs verified; accuracy clearly fails. Not evidence that
the Alphasense hardware or split-IMU concept inherently fails; cause not isolated.
No ON/fusion/full run, parameter tuning, current shared-vs-split control or
default promotion. Diagnose native Alpha initialization/IMU consistency first.
IE-TC sharesCPT7, not independent prismGT; path is estimated polyline, not travel.

**CP-V84 BOTH-PERSISTENT FULL (2026-11-01; COMPLETE, ATTEMPT1 VERIFIED):**
Authority: experiments/eig2_symmetric_persistent_full_cpv84_20261101_attempt1;
matching W/runs/.../prepared. Read RESULTS_REPORT.txt/results.json/complete.json,
report_verification.json and supplemental post_closeout.json. Launched once
15:22:15 local; complete 15:54:07. One full355.02s EIG2Hesai live CBS-ON,
both GLIM CP82 release_4 inject_persistent (aecc729f...e918) and Kimera CP76
persistent (b7d6829d...cee8), fresh live bidirectional exchange, concurrent CP-K19
fusion Qc x1.0, offset5.7s/0.20x/25sdrain. No weight/model/calibration changes,
new OFF control, matched OFF/ON claim, default promotion or next campaign.
Method1 direct EVO1.36.3, SE3/no-scale ATE, reference-distance0.5m RPE:
common100ms support353.0s (99.431%),3532queries,349 matched RPEpairs,
reference188.013683m. GLIM/Kimera/ONLINE ATE .047487/.330296/.228121m;
translationRPE .017034/.037836/.039718m; rotationRPE .633874/.845110/
2.293056deg; estimated paths189.9177/188.3852/201.0671m.
Full-saved native cadence: ATE .047655/.330924/.229055m, tRPE .018404/
.044667/.052213m, rRPE .574237/.713130/3.142319deg; no-gap paths193.0726/
189.8885/224.1500m vs reference188.0976/186.6062/188.1258m. Rows3543/1761/
3545; supported354.1993/353.7972/354.4s (99.769/99.656/99.825%). Native
RPE pair counts349/327/348 differ; common table owns the matched comparison.
Kimera's approximately1s gap excluded, not bridged (gap-inclusive189.8953m).
DELAYED7s fixed-lag-finalized separately: common ATE .226980m, tRPE .016307m,
rRPE .670052deg,path187.0592m; full-saved ATE .227062m,tRPE .016041m,
rRPE .668424deg,path188.7955m vs188.1258m reference. NotONLINE/full-history.
Native source gates passed:1741G2K/1747K2G persistent insertions;0temporary
retirements. All213827 transported versions exact (170458GLIM/43369Kimera),
5283receiver inserts/208493replacements/50preinit exclusions/1terminal Kimera
EOF omission/0pending/stale/duplicate/expired. No zero-factor-loss claim.
Previous-state Kimera revisions p95 .031962m/max .221563m, no>0.5m;1745paired
steps,15missing diagnostic IDs. Shape revisionp95 .025231m/max .204703m.
Kimera optimize mean249.298ms/p95467.336ms; receiver supportp9573.217ms,
receipt-processingp9521.591ms at0.20x, not a real-time certificate.
Median covariance trace GLIM/Kimera: rotation6.74994e-9/1.10845e-8rad2,
translation1.86421e-6/6.02616e-7m2; payload error0, raw histories retained.
Mixed-prior Kimera outgoing peer history remains explicitly useraccepted;
native-only information FALSE, independence/calibration unproven; mean-only
covariance alignment unchanged. IE-TC sharesCPT7, not independent prismGT;
paths are estimated polylines, not physical travel. Incremental checks are
not nonlinear stationarity (one observed total-cost increase retained).
24EVOCLI archives verified; CLI/API paritymax0,TUMroundtrip1.89e-15. Post-exit
audit16:01:45 PASS:2304protected/187terminal/75inputs, CP82/80/78 terminal
176/192/218 and authoritativeCP83 eight outputs, original Hesai all667.
All16 exact owned identities/commands absent, including controller177714
proc_start8768965/supervisor177712. Both initially-stopped containers restored.
One source capture/no sensor retry/no receiver replay. Experiment heartbeat
PAUSED. Immutable terminal products untouched; post_closeout already exists.
Original CP84 startup failed before receiver or sensors due wrapper664 vs755;
only master started/exited0, containers restored,2220pins/Hesai667 verified.
Failed evidence preserved/pinned; attempt1 restores original executable mode
and uses fresh roots/port/session only, no estimator settings changed. No sensor
rerun/numerical retry. Do not use old failed status as the completed attempt's
state or duplicate any stage. Earlier admission history below is retained and
superseded by the verified completion above.
User approved pip-cache cleanup then full launch.1433cached files/~4.78GB
removed,16.05GBfree after cleanup; no installed package/dataset/run deletion.
Preparing full355.02s static CP80 harness with tested CP83 bothpersistent overlay
and prospective corrected GLIM-log gate; conservative1.5x full forecast,
8GiBprelaunch/5GiBruntime. No launch yet; require passed prepared.json.
Historical initial storage admission below remains preserved:
User "Proceed to full": one355.02s EIG2Hesai live CBS-ON, CP83 settings unchanged
(GLIM CP82 inject_persistent, Kimera CP76 persistent, CP-K19 WNOA Qc x1.0,
offset5.7s,0.20x/25sdrain). No new OFF/multi-sequence campaign/build/tuning.
Read experiments/eig2_symmetric_persistent_full_cpv84_20261101/REPORT.txt.
Host15:10:10 free11,291,623,424bytes < existing full-run minimum12,643,595,264;
shortfall1.35GB before preparation. Both containers stopped, no new capture,
controller, scoring or monitor launched. Preserve all evidence/storage guards.
Approximately4.5GiB pip download cache is a cleanup candidate pending explicit
approval; nothing deleted. Recheck capacity and remaining preflight afterward.
Prior short-run pass is not a full-run result; mixed-prior limitation unchanged.

**CP-V83 BOTH-PERSISTENT LIVE60 (2026-11-01; COMPLETE, SAVED-REPORT REPAIRS):**
CP82 tests passed; conditional single60s actual all-estimator EIG2Hesai live
launched ONCE14:02:21local, supervisor166061/controller166063, proc_start8289596.
Read experiments/eig2_symmetric_persistent60_cpv83_20261101/PROTOCOL.txt,
prepared/status/launch/start receipts. GLIM CP82 inject_persistent, Kimera CP76
persistent, unchanged CP80 profiles/weights/receiver Qc x1.0/0.20x/25sdrain.
1989pins/Hesai667 pass,12.06GBfree>9.84GBforecast. No fresh OFF control or full
sequence. Controller exclusively owns all stages; no retries/overwriting/pinned
edits. Accepted mixed-prior limitation remains. No automatic full campaign.
One capture completed14:08:28,292G2K/293K2G. Original controller failed only on
GLIM-positive-tag check reading wrong log; supplemental validator included GLIM
stdout with no guard relaxation. Subsequent report inherited a >250s statistic
for60s and stopped after all24EVO CLI jobs had completed. Original failures and
archives retained; report_completion_1 reused archives (zeroCLI repeats), marked
only that diagnostic inapplicable, and verified EVO API parity/input hashes.
Authoritative report_completion_1/RESULTS_REPORT.txt,results.json,complete.json,
report_verification.json,post_closeout.json. COMMON59.0s/591queries/98.333%,
reference34.754858m: GLIM/Kimera/ONLINE ATE.022821/.062271/.059850m,paths35.0632/
34.5645/37.0947m. Native ATE.023262/.062517/.063801m,paths35.7594/34.8636/41.2382m
vs34.8501/34.4625/34.9403m reference; do not interchange sampling/masks.
DELAYED7s native ATE.053265m,path34.8718m,notONLINE. Kimera revisionp95.028878m,
max.084349m; optimize mean98.957ms,p95133.221ms (0.20x,notrealtime).34552exact
payload versions,884inserts/33618replacements/50preinit exclusions,0EOF/pending.
14:15:07hostaudit1989pins/151inputs/8terminalproducts/CP82,80,78outputs/Hesai667
PASS;16ownedjobs absent,bothcontainers stopped. Private compatibility restored,
short live integration passed; not a full-trajectory/covariance-calibration proof.
No capture/receiver retry, covariance/weight/model change, default/full campaign.

**CP-V82 GLIM PERSISTENT COMPATIBILITY (2026-11-01; PRIVATE TESTS PASS):**
User approved minimal private compatibility restoration following CP81 history.
Read experiments/glim_persistent_compat_cpv82_20261101/PROTOCOL.txt. Preserve
existing persistent append/native matching/covariance math; opt-in admission
with all other guards retained. Private bridge + lifecycle/provider/GPU tests
first; conditional single60s EIG2Hesai live with bothpersistent only if passed.
No full campaign/default promotion. CP80 mixed-prior provenance remains accepted.
Completed release_4/UBSan1:29/29 each, real bridge insertion/dedup/retirement;
temporary exact original DSO state/cache/history/covariance parity. GPU/IMU
72updates,13peer retirements,555queries,51,343records/2,590,537numeric entries
exact across batching OFF/ON. Candidate aecc729f...e918; no covariance/weight/math
changes, vendor sanitizer scope limited. Three harness failures preserved, no
sensor failure/retry.1617pins/192CP80/218CP78/Hesai667 unchanged, containers stopped.
Read REPORT.txt,verification.json,complete.json. CP83 conditional60s bothpersistent
live harness now preparing; no full sequence/campaign automatically authorized.

**CP-V81 SYMMETRIC PERSISTENT ADMISSION (2026-11-01; AUDIT COMPLETE, LIVE BLOCKED):**
User requires persistent reception in BOTH GLIM and Kimera, superseding CP80's
asymmetric choice for the next experiment. Verify the current GLIM path before
the proposed conditional 60s EIG-2 Hesai live test, with unchanged covariance,
weights, sensor profile and receiver. Exact CP-I1 source at685 and frozen build
receipts establish shared-native provider admission rejects InjectPersistent
before subscription. This is a static compatibility finding, NOT a demonstrated
numerical defect. Persistent factor append already exists; native admission is
before peer append. Historical S4 tests retained peer factors through retirement,
but CP-K16b's GPU/IMU peer test removes them early; neither certifies current
persistent bridge integration. Disabling provider/native_local or bypassing
guard is not allowed. Need a separate private compatibility patch and focused
bridge/provider/GPU-lifecycle regression tests, then conditional60s if passed.
No capture, build, source/config edit, container start, tuning or promotion.
Read experiments/glim_persistent_admission_cpv81_20261101/REPORT.txt,audit.json.
Read-only12:11:58 audit:1617protected/192CP80terminal/218CP78terminal hashes and
Hesai667 unchanged; bothcontainers stopped. Evidence verified, live admission
FALSE. CP80 mixed-prior limitation remains useraccepted, not this blocker.
Historical follow-up: CP81/HISTORICAL_COMPATIBILITY.txt confirms Sep21 full305s
EIG2 persistent both ways from saved settings/completion and actual1519/1517
K2G/G2K log rows; ETH1/SPX3 saved persistent configs also confirmed. Located the
exact guard addition in Oct12 S4 provider build.py:15; Oct19 CP-I1 retained it.
This was restricted integration scope, not an identified persistent numerical
failure. Reframe next task as minimal compatibility restoration of existing
mode, with focused regressions, not a new factor algorithm or assumed redesign.
No implementation/build/run in this historical review; CP81 original audit kept.

**CP-V80 FULL LIVE PERSISTENT (2026-11-01; COMPLETE/PASS WITH PROVENANCE LIMIT):**
User requests all estimators live and explicitly accepts not excluding absorbed
peer information. Read experiments/eig2_persistent_live_cpv80_20261101/PROTOCOL.txt,
PRELAUNCH_REVIEW.txt,REPORT.txt and prepared/launch/start/status receipts.
One full355.02s EIG-2 Hesai ON with GLIM temporary + CP76 generalpersistent
Kimera + concurrent CP-K19 fusion (7s WNOA,Qc x1.0), live bidirectional exchange.
Same CP49 physical window/profile/calibration/scales/audit,0.20x/25sdrain.
No new OFF, saved-belief replay, FL4m RETAIN, build/tuning/default change.
CP79 provenance blocker explicitly accepted as experimental limitation, NOT fixed:
Kimera outgoing LOCAL includes absorbed peer info, no native-only/independence
claim. Exact payload/route and all other integrity/solver/coverage gates retained.
1617pins, exactbags/profiles/privateCP76DSOs/receiver, Hesai667 pass;19.89GBfree.
Finite exclusive controller and Method1 EVO full-saved/common-mask reporting
prepared; delayed fusion separate. Launched ONCE02:23:06 local, supervisor128696,
controller128698; launcher128695 exited normally. Actual sensor/mode/library
preplayback gates pass, liveG2K persistent/K2G temporary rows and concurrent
fusion publications verified02:24:32. Closed02:54:46 local: full capture and
sensor/route/lifecycle/receiver gates,24direct EVO archives and report checks PASS.
Read RESULTS_REPORT.txt,results.json,report_verification.json and new supplemental
FINAL_INTERPRETATION.txt/post_closeout.json; immutable controller outputs untouched.
Full-saved Method1 GLIM/Kimera/ONLINE: ATE.048002/.321950/.212946m;
reference-distance0.5m tRPE.018141/.044498/.052010m,rRPE.574969/.733095/3.151577deg;
no-gap paths193.1909/190.0336/224.3963m vs188.0976/186.6052/188.1258m reference.
DELAYED seven-second finalized separate: ATE.211210m,tRPE.016378m,rRPE.667703deg,
path188.8831m vs188.1258m; not online/full-history. Full rows3543/1762/3545,
supported354.1993/353.1972/354.4s; Kimera0.6/1.0s gaps excluded, not interpolated.
Common100ms3526queries,352.3/355.02s(99.234%),348RPEpairs: ATE.047793/.321577/
.212067m,tRPE.016789/.037245/.039614m,rRPE.634772/.845746/2.279666deg;
paths189.9111/188.5196/201.1290m vs188.0112m. Common DELAYED ATE.211257m,
path187.1500m,tRPE.016180m,rRPE.670369deg. No mixing native/common masks.
Kimera previous-state revisionp95.031687m,max.216949m,0over0.5m/1m across1748
pairedsteps (13missingIDs); rotationp95.422113deg. Optimize mean249.56ms,
p95480.47ms; receiver servicep9575.31ms/receipt-processing22.84ms at0.20x.
1744source injections each direction,0temporaryKimera retirements, all213753
transported covariance payloads exact;5286inserts/208416replacements,50pre-init
excluded,1terminalunsupportedKimera,0pending/stale/duplicate/expired. Explicit
absorption telemetry absent; incremental checks NOT nonlinear stationarity.
Median transported covariance tr(rot)/tr(trans): GLIM6.8641e-9rad2/1.8729e-6m2;
Kimera1.1165e-8rad2/5.9052e-7m2; not calibrated-independent likelihood evidence.
Independent03:01:14host audit:1617protected/192CP80terminal/218CP78terminal pins
and originalHesai667PASS; all20checked ownedprocesses absent,bothcontainers stopped.
Evidence supports persistence benefit surviving this live feedback scenario:
Kimera ATE.321950 vs historical CP78persistent.319811m, similarpath/revisions.
NOT newly matched OFF/ON or isolated causal effect. ONLINE fusion native path
still19.28%longer than sampled reference; remaining output issue NOT solved.
Mixed-prior provenance remains FALSE/useraccepted; no independence certificate,
weights/smoothing/default promotion or further experiment. Completion heartbeat
paused after terminal notification. Preserve all earlier failures and results.

**CP-V79 FULL LIVE ADMISSION REVIEW (2026-11-01; BLOCKED, NOT LAUNCHED):**
Historical review; user accepted the provenance limitation for CP80 explicitly.
User requests full live GLIM/Kimera/fusion after CP78. Read experiments/
persistent_live_preflight_cpv79_20261101/REPORT.txt. Read-only review of CP49
live protocol, CP67 provenance finding, CP76 compiled source and CP78 limits.
CP76 source SHA70da03c6...c92 still exports relative beliefs via MAIN LOCAL
filter at3570/3584/3593/3736, not separate pose-covariance sidecar. Retained peer
information in marginal priors cannot be removed by explicit-factor filtering.
CP78 one-way saved-input success does not repair this; no new live run/build,
source edit, container start, score or weight change. Need separate authority
for peer-free outgoing marginal-history repair/validation or an explicitly
qualified one-way live diagnostic; no silent direction/mode/default change.
Full live execution requested but not admitted; existing results preserved.

**CP-V78 FULL NATIVE-MATCHING PERSISTENT (2026-11-01; COMPLETE/PASS):**
User explicitly approved full sequence after CP77. Fresh experiments/
persistent_native_full_cpv78_20261101/PROTOCOL.txt. Extend same conditional
Kimera-only temporary/remove vs generalpersistent to355.02s EIG-2 Hesai.
Complete3541 raw CP49 beliefs/native matching/mean alignment; same CP76 binary,
weights/profile,0.20x/25s. No new GLIM/fusion/returned peer or tuning/build.
No forced update IDs or CP73 readiness. Native coverage/admissions may differ;
report full-saved/common-mask Method1 EVO0.5m RPE and gaps/omissions. Stop on
failure, no retries.812pins/3541exact raw beliefs prepared. Launched ONCE00:45:57
local: sleep-inhibiting supervisor85846/controller85848. Controller exclusively
owns both captures, gates, scoring and cleanup; see status.json/replay_attempt.
Closed01:47:00 local: both full captures/gates, EVO scoring and cleanup PASS.
Read RESULTS_REPORT.txt,results.json,report_verification.json,
FINAL_INTERPRETATION.txt and post_closeout.json in this experiment root.
Full-saved Method1 temporary->persistent ATE.887076->.319811m (63.95%better),
reference-distance0.5m tRPE.340378->.043441m,rRPE2.116129->.716961deg;
estimatedpath334.3011->189.8771m vs native-cadence reference186.6050/186.6120m.
Common100ms3531poses: ATE.882818->.319614m,tRPE.228600->.036599m,
rRPE1.595266->.838412deg,path299.6635->188.4087m vs188.0133m reference.
Common supported352.8/355.02s(99.375%);16queries excluded, no bridging gaps.
Native1761/1763poses; temporary0.6/1.0s gaps,persistent0.7s gap disclosed.
3541exact raw inputs each,1741/1743native admissions;1768/1767drops,32/29last
retries,0/2received withoutmatchrow. Not all input beliefs inserted.
Whole previous-pose revisionp95.429936->.031256m(92.73%less); final60s
.391833->.025961m(93.37%less). Mean optimize109.75->279.41ms(2.55xslower).
Policy comparison succeeds across full usable sequence, without stronger weights,
output smoothing or retrospective-state scoring. Not isolated retirement-only:
native timestamps/admissions/aligned means differ; general persistent also changes
replacement/admission. Saved ON input includes prior feedback history, no live
GLIM/fusion/returned peer. CP67 mixed-prior LOCAL limitation remains failed.
812protected/218terminaloutputs/12EVOarchives/Hesai667PASS; ownedjobs exited,
bothoriginallystoppedcontainers restored, host confirmed01:48:39. REPORT and
complete hashes preserved; supplement only. No default or live promotion,
no additional run authorized. Experiment-only completion monitor PAUSED after
verified terminal completion; no further automatic experiment work.

**CP-V77 NATIVE-MATCHING PERSISTENT60 (2026-11-01; COMPLETE/PASS):**
User approved fresh60s EIG-2 Hesai Kimera-only temporary/remove versus general
persistent. See experiments/persistent_native60_cpv77_20261101/PROTOCOL.txt.
Complete original CP49 raw wire input, native matching/mean alignment; no CP70
forced update IDs/CP73 readiness, no new GLIM/fusion/outgoing peer consumer.
Same CP76 private binary both arms; no weights/build/default changes. Native
admission differences reported rather than forced away. Original ON feedback
history and CP67 mixed-prior limitation remain. Launched ONCE00:27:12 local,
controllerPID72061,601 prelaunchpins,590 complete raw beliefs. Controller owns
temporary->validation->persistent->validation->EVO->cleanup. No competing stage.
Closed00:38:57, no retries. Read INTERPRETATION.txt,RESULTS_REPORT.txt,results.json,
report_verification.json,post_closeout.json. Both298 identical native stamps and
292 identical native admission identities/update/endpoint times, all590 raw wire
inputs exact. Native dispositions272drops/24last retries/2received without match
row; no all-inputs-inserted claim. Full-saved Method1 EVO ATE.093192->.057444m,
38.36%better;0.5m tRPE.113062->.031230m,rRPE2.495136->.889706deg; path41.410951
->34.935135m vs34.462534m reference (excess20.16%->1.37%). Common596mask ATE
.084683->.057071m; separate cadence/reference path. Late revision p95.297709
->.042279m (85.80%less), nearly unchanged mean same-update motion. Mean optimize
74.84->110.67ms; IMU cost not uniformly better; different total graph objectives
not directly comparable. Native admissions happen to match but aligned means
can evolve differently: whole persistent policy, not strict retention-only
isolation. Supports earlier CP70 finding; not universal/full/live fix. No default
promotion.601pins/212closedoutputs/12EVOarchives/Hesai667PASS; ownedjobs exited,
bothcontainers restored stopped, source repos clean. No automatic full replay.

**CP-V76 GENERAL PERSISTENT COMPATIBILITY (2026-11-01; PRIVATE GATES PASS):**
User approved CP75's private mode-aware extension and synthetic gates, not a
sensor replay. Read experiments/persistent_compatibility_cpv76_20261101/REPORT.txt,
PROTOCOL.txt, build_fixture1/ and verification_fixture1.json. Explicit
CPV76_PERSISTENT_COMPAT=1 supports existing general Persistent with corrected
CP-K9/CP40; not FL4m RETAIN. Native admission/replacement and weights unchanged.
Mode-aware slots/payloads, replacement outcome, native-factor survival, refresh
and exactly-once marginal consumption checked within original rollback wrapper.
No class data members/ABI change. Release/ASan+UBSan, inherited OFF/temporary
REMOVE/RETAIN, exact extension-disabled states/cache/tree parity and private
backend/ldd-r PASS. Six replacement cases, nine marginal boundaries and nine
injected rollback cases; persistent Schur relative1.503e-16, boundary9.362e-19,
displaced prior-function error0. Generic fault tests are not exhaustive internal
recovery coverage; no stationarity/real-data accuracy claim. Failed initial
test-entry naming collision preserved; no solver ran in that failed build.
Private libkimera b7d6829d...fa1cee8; copied libcbs82f85c2a...712476 unchanged.
41predecessor inputs/544CP73outputs/Hesai667 verified; source git trees clean,
ownedjobs exited, bothcontainers stopped. No replay/tuning/default/deletion/push.
Next separately approved60s native-matching temporary vs persistent comparison
must use complete original wire stream and no outgoing consumer; CP67 mixed-prior
LOCAL limitation remains. This result does not authorize an automatic full run.

**CP-V75 GENERAL PERSISTENT COMPATIBILITY (2026-10-31; AUDIT COMPLETE, NOT RUN-READY):**
User approved compatibility inspection before using the actual old persistent
configuration. Fresh experiments/persistent_mode_compatibility_cpv75_20261031.
Trace persistent flag through CP73 corrected CP-K9/CP40 backend and linked CBS;
check mode guards, ownership/replacement, refresh, marginalization and rollback
against existing tests. Distinguish general persistent from FL4m RETAIN and
persistent sidecar selectors. Saved/source inspection only; no implementation,
build, native solve, replay, guard/weight/default change or automatic next run.
Verdict: not flag-only compatible. cpv40ValidateMode -> cpk9ValidateMode rejects
Persistent before ordinary solve. Post-refresh count/identity checks additionally
assume active-window tracking that persistent slots do not populate. Replacement,
absorption and inner recovery require mode-aware tests with original prior
function/native smart/whole-update rollback retained. Existing corrected suite
tests active-window REMOVE/RETAIN; old FL4m persistent fixture predates corrections.
No mathematical incompatibility or new estimator failure claimed. Candidate
implementation not made. Proposed next private compatibility fixtures only,
before separately authorized60s native-matching comparison. Original full wire
stream3541 differs from CP73 accepted/pre-aligned1743 subset (1798omitted);
future persistent admission test must use complete original inputs, not force
original backend IDs or re-align frozen means. CP49 originals still hash-match.
19static checks,25compiled-source identities,544CP73outputs and25inputs verified.
See REPORT/audit/input_sha256/verification. Production source clean; no native
tests/build/run/container change, weighting, default promotion or CP67 bypass.

**CP-V74 RETAIN TIMESTAMP/RETRY DIAGNOSIS (2026-10-31; COMPLETE):**
User requests explanation before considering old persistent configuration.
Saved CP73 REMOVE/RETAIN logs, physical timestamps, wire payloads and current
frontend/retry/gate source only. Fresh experiments/retention_timestamp_diagnosis_cpv74_20261031.
Locate first physical keyframe divergence, distinguish missing image/selection
from timestamp conversion, trace unaccepted retry versus duplicate wire input.
No build, solver, scoring, live replay, settings/guard/weight changes or retry.
Preserve CP73 failed pair and original full REMOVE; no full retention metric.
First divergence1554:311.197445305s original/REMOVE versus311.297444605s RETAIN,
all earlier stamps exact. Frozen3103->3105 at1555 rejected100ms endpoint error
against50ms tolerance, then internally requeued. Gate marked released before
acceptance; retry triggers duplicate-already-released at1556.1551wire messages
unique/exact,1549actual admissions exact IDs/covariance, meanerror<=2.78e-16.
FullREMOVE1762stamps/1743admissions exact; neither arm logged readinesswaiting.
Raw stereo expected/later frames both present, not proof of runtime delivery.
Adaptive visual keyframes and backend-to-frontend IMU-bias feedback mean equal
update numbers need not retain physical endpoints. Decision/receipt audit absent;
exact frontend cause unknown. New frozen-belief replay did not freeze frontend.
Do not relax guards, change endpoints or auto-retry; not proof oldpersistentmode
failed.544CP73outputs/20inputs verified unchanged. REPORT/analysis/verification
preserve full chain. No build/source capture/solver/scoring/settings/weightchange.
CP70 short result and CP67 LOCAL limitation unchanged; no oldconfig switch made.

**CP-V73 REPLAY READINESS HANDSHAKE (2026-10-31; FAILED/CLOSED):**
User approved CP72 next step. Fresh experiments/retention_readiness_cpv73_20261031
and matching workspace root. Private, opt-in bounded wall-time readiness barrier
before the recorded update: consume only real queued messages, never synthesize,
backdate or change update IDs, payloads, weights or matching. Preserve failed
CP71 and completed CP70. Release/sanitizer/concurrent handoff/negative fixtures,
disabled parity and native compatibility gates before fresh full355.02s frozen
CP49 REMOVE->validate->RETAIN->validate conditional pair. Reuse CP71 frozen
inputs byte-exact; no new freeze/source GLIM/fusion/return consumer. Log waiting
separately from estimator timing. No retries or live/default promotion; CP67
LOCAL provenance remains failed. Read new PROTOCOL.txt and process receipts.
18:21 build gates PASS: Release/san concurrent readiness + strict negatives,
full1743 synthetic payload conservation, native CP69 compatibility, exact
disabled state/cache/tree parity, private build/dependencies. Backend SHA256
141474ecf226f863c81e5f8b70df0843b80420aa2e78f74fb48729a1958169d1.
9091 protected files unchanged, build jobs exited0, both containers stopped.
18:22:36 fresh finite pair launched ONCE; supervisor45229/controller45231.
Prepared9558pins; exact CP71 frozen inputs reused. Actual stage in status.json;
do not duplicate or edit protected files. No full comparison result yet.
19:19:41 closed FAILED: REMOVEfull355.02s validates1762poses/1743admissions;
RETAIN1556poses through311.497443105s, stops at update1556 on internal-retry
duplicate guard after physical timestamp mismatch. No full pair/EVO score.
9558protectedfiles/Hesai667PASS, cleanup/integrity null, containers restored
stopped and ownedjobs exited. No retry; original receipts/reports remain pinned.
Read separateCP74 saved-only diagnosis; no estimator/default promotion.

**CP-V72 SAVED REPLAY-TIMING DIAGNOSIS (2026-10-31; COMPLETE):**
User requested finding why CP71 failed despite older working retention tests.
Fresh experiments/retention_delivery_diagnosis_cpv72_20261031; saved-data and
source inspection only. Preserve CP71 completed FAILED evidence. No estimator
run, native solve, build, guard/weight change or automatic retry authorized.
Trace exact missing685->687 wire bytes, recorded arrival, callback/queue path
and original target346; compare October15 FL4m and successful CP70 timing.
Finding:341byte-exact received inputs include missing685->687;340prefix
admissions exact IDs/covariance, meanroundtrip<=1.67e-16;346native stamps exact.
Target346/69.599331232s input recording10.546610ms AFTER camera stamp; new gate
holds early arrivals but lacks due-input readiness synchronization. Wrapper
buffers callback messages and forwards nominal20Hz wall; exact callback/flush
timings unsaved, do not over-attribute latency. Original FL4m450inputs all before
targetstamp>=41.026628ms (0.10x), both exact-update postvalidation passed.
CP70 remains a valid short run, not scheduling-independent replay certification.
CP71 stopped before update346 solve, RETAIN not tested. Next ONLY proposed:
bounded diagnostic callback readiness acknowledgement, same availability,
payload/update/binding; fixtures before separately authorized fresh replay.
No build/native solve/source replay/scoring/weights/guard changes or retry.

**CP-V71 FULL FROZEN-INPUT RETENTION (2026-10-31; FAILED, CLOSED):**
User selected the first option: extend the conditional remove/retain test to
full EIG-2 Hesai 355.02s, not a live bidirectional three-estimator run.
Fresh experiments/retention_full_frozen_cpv71_20261031 and matching workspace
run root only. Reuse exact tested CP70 backend/CP69 compatibility, no rebuild
or weight change. Freeze original full CP49 ON aligned beliefs and recorded
backend admission schedule; require exact payloads, physical timestamps,
bindings and update IDs. REMOVE then gate then RETAIN, same front stereo/CPT7,
0.20x,25s drain; no new GLIM/fusion/return consumer. Both arms use ON inputs.
Method1 EVO-only ATE/0.5m RPE, full-saved/common coverage/path/revision diagnostics.
CP67 live LOCAL provenance failure remains; no live/default promotion.
Launched ONCE16:52:16 local, supervisor32250/controller32252. Preparation
verified9038pins and exact CP70 binaries; full1743 accepted aligned beliefs
joined to1762 original physical timestamps. Original3541 incoming wire beliefs
include1798 outside accepted set (explicit retained omissions); native tail gaps
.6/1.0s retained as gates. REMOVE startup effective settings/maps pass; playback
then stopped at update346/69.599331232s: delivery gate did not find685->687
in its backend queue at the required update. RETAIN never launched; no scoring.
16:58:50 closeout verified9041pins, original Hesai667, both containers restored;
owned jobs exited. No retries. Read immutable complete.json/status.json and
CP72 separate diagnosis. Do not reinterpret the failed prefix as full results.

**CP-V70 RECORDED-UPDATE DELIVERY (2026-10-31; COMPLETE, HYPOTHESIS SUPPORTED):**
User authorized the CP69 proposed harness correction and bounded comparison.
Fresh experiments/retention_update_gate_cpv70_20261031 and matching W/runs root.
Private delivery gate holds already-arrived frozen beliefs to recorded backend
updates; fail on missing/late/duplicate/unlisted input. No payload, matching,
weights, native model, solver or retirement-mode change. Preserve failed CP69.
Fixtures/disabled parity precede a fresh exclusive60s REMOVE then RETAIN pair,
same original EIG-2 quiet start,0.20x,25s drain. Strict original admission IDs
and identical A/B inputs required. No blind retries or timing-gate relaxation.
Method1/EVO only after both validate. CP67 bidirectional provenance still failed;
frozen incoming history is conditional, not independent or feedback-free.
No production/default promotion, new live GLIM/fusion or full mission authorized.
15:16 build gates PASS: delivery Release/san, unchanged CP69 compatibility,
exact remove state/cache/tree parity, private runtime/ldd. Backend SHA256
6e620d0f24aed055ee8f9f2f638af1b59881ff09e23ebe561614162a93d634ce.
Starting ONE exclusive fresh remove->validate->retain->validate controller;
receipt experiments/retention_update_gate_cpv70_20261031/replay_attempt/start.json.
Do not duplicate. Old CP69 artifacts protected;21GiB storage before capture.
FINAL: fresh60s REMOVE/RETAIN both complete/pass,298poses each,292 exact
original admissions and physical timestamps, exact wire/covariance, first30
poses exact before retirement. Same early belief held206->207 in both; no
missing/late/duplicate input, runtime guard/recovery or capture retry.268 native
retained absorption boundaries match retirement counts exactly.
Method1 FULL SAVED EVO: ATE .097695->.062660m (35.86%better),tRPE0.5m
.111920->.033203m (70.33%), rRPE2.403280->.854870deg (64.43%). Path
41.7278->34.8232m vs reference34.4625m; excess21.08%->1.05%. Same298stamps,
59.599471s span, no gaps. Common100ms596queries/59.5s: ATE .088985->.062449m,
path39.4060->34.5545m vs34.7570m. Never mix native/common references.
Late40..60s previous-revision p95 .285475->.037104m (87%less),maximum
.639667->.113423m; within-update motion mean .092180->.091968m nearlysame.
Full diagnostic296pairs excludes missing keyframe4 ONLY for decomposition,
not accuracy. Strong sustained retirement contribution HERE, not universal
proof/complete stability. IMU cost slightly worse, total objectives different
histories; not stationarity. Optimize p9598.01->89.90ms, not throughput proof.
EVO CLI/API error0,12 saved metric archives rechecked; no scoring reruns.
REPORT.txt/RESULTS_REPORT.txt/results.json/report_verification.json authoritative.
CP67 provenance gate remains failed, no live bidirectional retention/default
promotion. Next only proposed safe MAIN/LOCAL history separation, not launched.
Closeout receipts verify original protected paths, owned jobs exited, both
containers restored and four-sequence Hesai667. No deletion/push/tuning.

**CP-V69 RETAIN COMPATIBILITY (2026-10-31; COMPLETE, COMPARISON NOT ADMITTED):**
experiments/retention_compatibility_cpv69_20261031/REPORT.txt,replay_diagnosis.json.
Private existing FL4m retain with corrected CP-K9/CP40: Release/san, legacy
CP38/40, exact CP66 remove state/cache/tree parity PASS. Nine retained boundaries:
Schur<=1.34e-16, continuity<=1.75e-18, prior function exact; native smart,
residual/slot guards and extra-refresh/postmarginal rollback preserved.
Backend0a4c5bd3dc2319a95d55923d96ca1b4243ccb1e37b5cd2dcecf7b58c2747a3c4.
Conditional60s REMOVE completed298publications/292beliefs/268retirements.
Strict event-equivalence gate FAILED: sender405->407 receiver204->205 admitted
at206/41.5996s instead of original207/41.7996s. Other291 updates exact; all
wire payload/order/endpoints/covariances exact, means<=1.67e-16. RETAIN NOT RUN;
no valid pair or EVO accuracy result. Original timestamps alone do not fix
asynchronous backend delivery timing. No blind retry or tolerance relaxation.
Initial pre-playback equivalent-path assertion preserved; fresh pathcheck root
repaired only path resolution, same input bytes/binaries. No numerical retry.
Next proposed ONLY: deterministic recorded-update release with causal arrival
checks, private fixtures, then a fresh bounded pair. Not implemented/launched.
No weights/default/production changes, new GLIM/fusion, bidirectional retention,
deletion or push. CP67 LOCAL provenance gate stays failed. Final preservation,
owned jobs/container restoration and Hesai667 in verification.json,complete.json.

**CP-V68 EXISTING RETENTION RECONCILIATION (2026-10-31; COMPLETE AUDIT):**
User correctly identified existing persistence. FL4m selector/audit header is
byte-identical in CP66; retirement scheduling branch unchanged. Historical94s
ETH1 remove/retain already implicated retirement, before CP-K9/CP40. Recent
CP66 isolates immediate late effects on corrected EIG-2; CP67 adds one-boundary
handover/provenance evidence, not a new mode. General persistent additionally
changes admission/replacement/force-solve/recovery; do not substitute it for
the isolated FL4m retain switch. Four compatibility checks: CP-K9 remove-only
selector, active-window core, scheduled deletion; CP40 requires retired factor
absent. Disabling CP-K9 also fails CP40 prerequisite and discards validated fix.
Reuse existing implementations/tests; next proposed gate is mode-aware private
compatibility fixtures preserving strict residual/rollback/original-prior rules.
Then separately scoped60s EACH frozen-input Kimera-only remove/retain can test
sustained Method1 zigzag without a NEW feedback loop; not bidirectional safety
or native-only LOCAL certification. CP67 failed LIVE gate remains failed. CP66
ON stream contains original feedback history held fixed, not independent input.
292 alignment rows/recorded bag inventoried, exact wire/admission joins NOT yet
validated. No sidecar redesign needed just for this conditional diagnostic.
17 static assertions/33 pinned inputs; no optimizer/build/source run, covariance
tuning, guard/default/production changes. Detailed REPORT.txt,audit.json and
complete.json in experiments/retention_reconciliation_cpv68_20261031.
No compatibility implementation or further replay launched/authorized here.

**CP-V67 MARGINALIZATION HANDOVER (2026-10-31; COMPLETE; LIVE GATE FAILED):**
User-authorized saved CP66update250/50.4s. Native Gaussian boundary reproduces
recorded postmarginal H/RHS exactly, survivor norm3.66e-17. CP66B reproduced;
retainedoldfactor absorbed once, survivor continuity3.96e-17; immutable prior
functions preserved and22owned locks released on diagnostic adapter copies.
No original incremental-tree restore/forward nonlinear replay claim. Real BPSAM
LOCALfixture: explicitpeer excluded exactly beforeabsorption, mixed-prior peer
information remains afterward(.342872normalizeddifference). Common-chart real
peer-stripped probe3.95349e-5. Exact relativebelief path uses MAIN LOCALfilter,
not separate posecovariance sidecar. Naive retain fails provenance contract;
NO60s candidate, forwardcontinuation or EVOscoring. Not evidence that current
remove-before-marginalization baseline already has this same leak.
Release/sanfixtures,11independentSchurchecks<=2.87e-15,9611pins/Hesai667PASS,
jobs exited/bothcontainers restored. Attempt1FastMapAPI compilefailure retained;
syntaxonlyattempt2passes. No weights/covariance scaling/production/defaultchange.
Detailed experiments/eig2_marginal_handover_cpv67_20261031/REPORT.txt and final
complete.json. Next ONLY proposed: MAIN/LOCAL marginal-history separation or
verified exclusion; first assess existing sidecar. No further implementation.

**CP-V66 RETIREMENT CAPTURE / CAUSAL TEST (2026-10-31; COMPLETE):**
User requested60sensor-seconds each, not just first10s. Fresh private observer
and matched OFF/ON from original quiet start; full frozenCP49settings. Four
checkpoint phases at28..35 and200..298. Prior serialization/nonmutation fixtures
before launch, source/gate stages exclusive, originals preserved. Same-checkpoint
CP65retirement/admission2x2 is separate from live capture; no permanent retention
candidate or weighting change. See experiments/eig2_retirement_capture_cpv66_20261031/PROTOCOL.txt.
00:50:21local launched ONCE after Release/sanitizer/nonmutation and disabled
observer parity passed; launcher293266/controller293273. OFF->gates->ON->gates
exclusive.7691pins. Do not duplicate captures/stages. Fresh private buildSHA
526952811bbda6dbba5dcaff183c7264c8fe7a38baed95f2b3a7e6dec5588015.
01:03capture PASS:848four-phase snapshots, allgates,7691pins/Hesai667,
ownedjobs exited/bothcontainers restored. ON.639667m previous-pose correction
at59.39942s vs OFFmax.037687m; confirmed useful lateevent, not causal finding.
Saved-only11event2x2 under way after capture exit; no further sources/weights.
FINAL01:10:43:11event2x2/88native Gaussian steps complete; allAequations exact,
state reproduction<=3.45e-7ordinary/7.57e-7refreshmixed norm. Retaining only the
expiring group beforemarginalization reduces previous-pose correction at50.4/
59.4/59.6s .335969/.639667/.449398m -> .062328/.107762/.019526m (81/83/96%).
New belief and native inputs fixed. New-only omission leavesbigjumps; no-removal
controls exactzero. Strong immediate retirement contribution at selectedevents,
NOT fulltrajectoryfix or safepermanentretention. B/Cbranchhistoryoriginaleach,
no forward propagation/marginalpriorabsorption. Gyrobias/nativecost notuniformly
better. No weights/covariance/production/defaultchanges/newrun/deletion/push.
528independent metricchecks<=4.15e-14;11realroundtrips/33restoredbytesPASS;
7754pins/Hesai667PASS,ownedjobs exited,bothcontainers stopped. DetailedREPORT.txt
and complete.json are authoritative; captured controller status remainshistorical.

**CP-V65 RETIREMENT TEST DESIGN / SCREEN (2026-10-31; COMPLETE, CAUSAL ARMS NOT RUN):**
experiments/eig2_retirement_test_cpv65_20261031/REPORT.txt,PROTOCOL.txt,
analysis.json,event_manifest.json,verification.json,complete.json.
Executable saved-data screen:1762updates/1743inserts/1720removals;median24active,
no logged empty set after admission;median lifetime4.79996s. First removal30
at6.39989s is oldx4->x5out/newx28->x29in,24->24active.1688removal updates also
admit new peers, so coincidence cannot establish causality.1730OFF/1733ON
eligible Method1 segments: exact published=within-solution+previous-revision
identity<=1.39e-17m;rotation COMPOSITION<=1.36e-15. ON485published-direction
reversals vs40within-solution on1723adjacent pairs; descriptive, not health/ATE.
Nine fixtures/100coordinate cases,10389independent raw-xyz vector checks PASS;
12input hashes unchanged,Hesai667originals PASS. No sources/optimizer/builds/
weights/production changes. Full native causal harness NOT implemented/run.
Proposed native2x2 oldretained/removed x newincluded/omitted, identical initial
checkpoint and two native steps; no persistent-factor promotion. Controls28,29
and retirements30..35 require32missing phase snapshots: CP51ends25. Its serializer
does not support OriginalChartPrior; private lossless anchor/coefficient support
and nonmutation fixtures required before authorized10s passive capture. No
capture launched. Early effects cannot settle later severe295/1203events.

**CP-V64 EXACT NONVISUAL JACOBIANS (2026-10-30; COMPLETE, NO MATERIAL FIX):**
experiments/eig2_exact_jacobians_cpv64_20261030/REPORT.txt,analysis.json,
metrics_check.json,complete.json. Original CP51update6 only; A native /
B exact CBS929 Local chain only / C exact pose-prior0 derivative only.
Same residuals/means/noise, physical snapshot/cache bytes identical; every
non-target Gaussian exact at common state/cache. No combined candidate.
Release/san fixtures and exact real derivatives before all solves pass:
raw/white FD<=4.68e-13; independent<=2.44e-10; directional<=2.87e-9.
Original equations exact/state<=3.34e-10. SIX native GNsteps (two each);
no fixture solves/extra nonlinear iterations/source or full sequence.
Final maxoldrotation A/B/C .322580686/.322579486/.322474373deg;
maxgyro-bias .00746094101/.00746095345/.00746094103rad/s.
Rotation reduction .00037193% CBS-only/.03295690% prior-only, gyro not
reduced. BOTH below predeclared1% benchmark. B/A all-state final difference
<=.384419micrometers/.00000411228deg; C/A<=.014972micrometers/.000265489deg.
Same exact visual support sets:285ordinary used/266refresh used; poststep
266/267valid. Native visual/IMU costs essentially unchanged. Not ATE or
true bias errors. Evidence AGAINST these isolated approximations as main
cause of FIRST disturbance; not universal exclusion or fullzigzag fix.
Independent assembly<=3.43e-17/retraction<=1.39e-16;930scalar/quaternion
metric checks<=8.65e-17. Linear residuals NOT nonlinear stationarity.
22:13closeout10921pins/Hesai667PASS;93retainedfiles; no failures/retries,
ownedjobs exited/bothcontainers stopped. No weights/covariance/production/
default edits/deletion/push; do not promote candidate or rerun fullsequence.
Stop this derivative lead; remaining supported mechanism is short-interval
belief disagreement accommodated by coupled orientation/bias response.
No follow-up experiment launched/authorized; await user's next decision.


**CP-V63 SUSPECT3 DIRECTIONAL AUDIT (2026-10-30; COMPLETE, QUALIFIED):**
experiments/eig2_directional_cpv63_20261030/REPORT.txt,analysis.json,
gate_explanation.json,complete.json. Original CP51update6 saved ordinary/refresh
steps; ZERO optimizer calls/source replay/production or weight changes.
712factor-state models/9256native costs; native equations exact, endpoint
<=8.94e-17; residual norm<=8.98e-14; independent7163fixed-point costs<=4.61e-12.
Native cost5362.1167->184.2149->158.4471; initial IMU cost also shared by
no-CBS control, not proof CBS helps. Existing CP52 no-CBS vs original final:
visual33.7644->93.7005, IMU.052557->1.253754, bias-between.445083->15.749331.
Saved correction uses strong orientation/gyro-bias compensation: ordinary
IMU1.74845 becomes228972.77 if ONLY gyro-bias changes omitted (cost probe,
NOT a reoptimized frozen-bias solution or true bias error).
Refresh retained263tracks cost92.238389->92.251203;3lost/4new account for
native visual96.519202->93.700511 decrease. Fixed-point diagnostic distinct
from native profiled/variable-support cost; no nonlinear stationarity claim.
Strict NONVISUAL derivative gates FAIL: CBS929 ordinary/refresh normalized
7.64e-6/1.39e-5 and poseprior0 refresh2.814e-6 >1e-6, both FD spacings.
Native BetweenFactor omits Local Jacobian (SLOW_BUT_CORRECT flag off);
ExtendedPriorFactor uses identity. Independent exact SE3 costs<=6.12e-14
and slopes explain discrepancies. Effect on optimized response UNTESTED;
not proof of fullzigzag root cause/fix. One extra1e-10 quadratic parity gate
also fails by2.51e-10absolute (IMU655 alpha1.25, cancellation~3.15e6terms).
Original failed verdict preserved, no threshold relaxed or scientific retry.
Initial harness syntax/Eigen-type compile failure retained; fresh
attempt_vector_api fixes only harness; Release/san fixtures/native audit pass.
21:27closeout10830pins/Hesai667PASS;117retainedfiles; ownedjobs exited,
bothcontainers stopped. No tuning/fullrun/covariance/default changes/deletion/push.
Next proposed ONLY: same-checkpoint exact nonvisual Jacobian ablation with
unchanged residuals/weights/smart policy; test CBS/prior separately, no fullrun.
No follow-up experiment launched or authorized by this ledger.

**CP-V62 SUSPECT2 VISUAL MATH AUDIT (2026-10-30; COMPLETE, CHECKS PASS):**
experiments/eig2_visual_math_cpv62_20261030/REPORT.txt,analysis_v2.json,complete.json.
User parks1/authorizes2, superseding old prohibition, not globally disproving1.
Saved original CP51update6 geometry, noCP61candidate.818accepted smart factor-
state instances across initial/postordinary/final;196rejected exactlyzero.
FD at1e-5/1e-6 agrees<=7.58e-9relative; independent analyticF/E<=9.37e-14,
RHS<=4.77e-11;213missing-right rows exactlyzero;whitening<=1.67e-16.
Explicit SVD/normal Schur H/g agree<=9.63e-10relative. Native b'b constant
unreduced by elimination, predicted discrepancy matches; not update driver.
Native captured matrices exact/state<=3.34e-10. Same dense solver on independent
visual assembly gives ordinary increment difference<=2.60e-9deg/2.93e-13m,
refresh<=1.78e-10deg/4.52e-14m. Manual analytic assembly also agrees. Native
2saved linear solves only; no forward nonlinear replay. Condition~1.006e10,
backward checks NOT nonlinear stationarity. Release/san and negative fixtures
pass. No tested visual Jacobian/elimination/assembly bug at first event;
not fullalgorithm proof or later marginalization/nonlinear update validation.
NumPybool JSON output failure preserved in analyze.py/partialanalysis.json;
fresh report-only analyze_v2.py/analysis_v2.json changes serialization only.
21:04closeout10711pins/Hesai667PASS,ownedjobs exited/bothcontainers stopped.
No source/fullrun/production/weights/covariance/default changes/deletion/push.
Next only proposed suspect3 directional response/cost audit along actual
rotation/bias update with fixed weights and explicit valid-set accounting.
No further test launched/authorized by ledger.

**CP-V61 TRACK-LOCAL TRIANGULATION (2026-10-30; COMPLETE, NO MATERIAL RESPONSE FIX):**
experiments/eig2_local_frame_cpv61_20261030/REPORT.txt,analysis.json,complete.json.
Saved CP51update6 original vs local-DLT candidate vs translated candidate,
2native GNsteps each. First left camera defines local chart, result mapped back;
native freshness/guards/weights/Jacobians/Schur intact. No historical cache reset.
2028cold coordinate comparisons pass (statuses same, points<=1.15e-12m,
Gaussian<=8.37e-10relative);1014independent native statusmatches/815points,
440independent state metrics<=2.78e-16. Release/san fixtures pass; original
equations exact/state<=3.34e-10; no failures/retries. Ordinary A/B identical:
285used tracks/0retriangulations. Refresh all338retriangulate; same266accepted,
7already-invalid labels differ. Netmaxoldrotation .322580686->.322537187deg,
gyro-bias .007460941->.007460878rad/s: negligible response reduction, not ATE.
Final A/B<=4.63micrometers/.00012224deg. Candidate B/C coordinate consistency
<=3.48e-13m/7.02e-10deg. Removes tested arbitrary-origin dependence, NOT the
large first-event response or fullzigzag. Large ordinary jump precedes refresh
supportloss. Initial cold candidate282 is diagnostic only, warm285used;
candidate-from-startup history remains untested. No full replay justified here.
20:06closeout10641pins/Hesai667PASS,ownedjobs exited/bothcontainers stopped.
No source/fullrun/production changes/covariance tuning/promotion/deletion/push.
Suspect1 remains open; no move to2 or further test authorized by this ledger.

**CP-V60 SUSPECT1 WORLD-ORIGIN TEST (2026-10-30; COMPLETE, DEPENDENCE CONFIRMED):**
experiments/eig2_world_origin_cpv60_20261030/REPORT.txt,analysis.json,complete.json.
Saved CP51update6 original vs physically equivalent translation[3,-2,1]m,
including pose-prior/cached points/cameras; native functions/cache policy intact.
1068factor coordinate controls pass; nonvisual equations<=1.65e-16, all frozen
point equations<=9.31e-10relative.3138independent transforms<=1.99e-15m;
1352CP56classification matches;250independent state checks<=2.22e-16.
Original equations exact/state<=3.34e-10; four native GNsteps, Release/san pass.
Initial warm-cache ordinary solve invariant (A/B<=4.75e-9deg);285supportboth.
At refresh native retriangulation changes18statuses at samephysical state:
6valid->rankfailure+3valid->outlier; support266->257. Final translated-back
A/B<=.158546mm/.00565807deg. Netoldrotation .322581->.325879deg; gyro-bias
.00746094->.00749091rad/s. Demonstrated native coordinate-consistency problem,
NOT fullzigzag rootcause or extrinsic mistake; early disturbance persists.
No source replay/productionfix/weight or guard changes/suspect2.19:33closeout
10552pins/Hesai667originalsPASS,ownedjobs exited/bothcontainers stopped.
Next proposed only: deterministic track-local triangulation candidate, explicit
unchanged physical tests and coordinate gates first, then saved-event response.
No favorable-origin tuning, fullrun, promotion, deletion or push authorized.

**CP-V59 SUSPECT1 INITIAL CACHE (2026-10-30; COMPLETE, NEGLIGIBLE EFFECT):**
experiments/eig2_initial_cache_cpv59_20261030/REPORT.txt,analysis.json,complete.json.
Saved CP51update6 original vs ONLY571initial cache cleared before ordinary;
native ordinary+refresh per arm, four real GNsteps. No CP58refinement/weight/
guard/production change. Ordinary support285->284, refresh both266;571returns
valid. All338native smart post-evaluation classifications identical both arms.
Net maxoldrotation.322581->.322388deg,gyro-bias.007460941->.007461002rad/s,
position5.721914->5.725811mm. Final A/B<=5.173micrometers/.000218871deg allstates.
This cache discrepancy does not materially explain first-event disturbance;
not fullzigzag rootcause, general cache all-clear, accuracy result or cure.
Original equations exact/state<=3.34e-10;355untouched Gaussians exact, one-cache
restoration gives exact snapshot bytes. Release/san fixtures;250independent
scalar checks<=2.22e-16. Initial harness failure preserved before any real solve:
exact-zero localCoordinates rejects2.676e-19 roundoff on byte-identical states;
fresh attempt_exact_values uses exact serialized identity, no relaxed guard.
19:20final10432pins/Hesai667originalsPASS,jobs exited/bothcontainers stopped.
Suspect1 remains open; separate world-origin-sensitive native triangulation's
effect on graph response untested. Coordinate-equivalent A/B only proposed;
no source/full replay, covariance work, suspect2, promotion/deletion/push.

**CP-V58 SUSPECT1 SAME-STATE REFINEMENT REFRESH (2026-10-30; COMPLETE, NO FIX):**
experiments/eig2_refinement_refresh_cpv58_20261030/REPORT.txt,analysis.json,
complete.json. CP51update6 identical postordinary state: original native refresh
A versus B restoring only CP57 smart-L2 landmark fits for newly-rejected tracks,
with unchanged native rank/depth/10m/3px gates.7of25restore(5geometries,allleft-only),
266->273visual factors.349untouched factor Gaussians exact; restoring7cache entries
recovers original snapshot bytes. Native smart functions/Schur/priors/weights
unchanged. A original equation parity exact/state1.05e-10 beforeB;2real GNsteps.
Net maxoldrotation.322581->.324613deg;gyro-bias.00746094->.00746486rad/s;
position5.72191->5.33937mm;accel-bias.0228479->.0241224m/s2;IMUcost1.25375->1.38487.
Restoring this subset does not materially suppress early rotational/bias response;
not fullrootcause/accuracy proof or justification for promoting a full replay.
One-time landmark-cache counterfactual, NOT a general triangulator/propagated run.
Release/san fixtures pass;19:05verification10347pins/Hesai667originalsPASS,
jobs exited/bothcontainers stopped. No source/covariance/production changes.
Stay on suspect1: initialcache571 beforeordinary remains separate untested
intervention; no move to suspect2 and no further test launched by this ledger.

**CP-V57 SUSPECT1 LANDMARK FEASIBILITY (2026-10-30; DIAGNOSIS COMPLETE, SUSPECT OPEN):**
User explicitly requires finishing suspect1 before suspect2; CP56's proposed next
step below is superseded. experiments/eig2_landmark_feasibility_cpv57_20261030/
REPORT.txt, findings.json, certificate_verification.json and complete.json.
CP51update6 fixed camera/observation/noise geometry: afterordinary25lost outlier
factors=13geometrically feasible+12numerically incompatible at unchanged3px.
Original smart-L2 landmark refinement recovers7instances/5distinct geometries.
Final27outliers=14feasible+13incompatible; plainL2recovers3, distance-constrained
L2adds2. Feasibility/minimax differs from nativeL2; no deployed replacement.
Additional4/3rank rejections remain inadmissible despite feasible points, not an
observability or guard-invalidity proof. Native/cold initial571cache remains
unisolated. Artificial translation changes some native labels but restores NONE
of the actual post-step rejected factors; geometric feasibility invariant under
both transforms. CP56strict origin gate remains failed.2544independent point
checks,39native smart costs match<=8.17e-13,75numerical infeasibility certificates
with300independently recognized necessary halfspaces. Boundary-search and
projection-roundoff failures preserved; inner search repairs never relax gates.
Final18:53:10308pins/Hesai667originals pass; jobs exited/bothcontainers stopped.
Zero graphoptimizer/source calls, covariance work, production changes or scoring.
Partial avoidable rejection established, NOT fullzigzag cause/fix. Next proposed
within suspect1 only: original vs refinement-before-rejection at identical saved
postordinary state feeding refresh; unchanged guards/nativeprior/weights, measure
pose/bias revisions. Cache clearing separate; subsequent propagated comparison
needed. No graph ablation/full rerun launched or authorized by this ledger.

**CP-V56 FIRST-INSERTION VISUAL SUPPORT (2026-10-30; DIAGNOSIS COMPLETE):**
experiments/eig2_visual_support_cpv56_20261030/REPORT.txt, analysis.json,
independent_dlt.json and complete.json. CP51 saved first-admission snapshots:
native visual nonzero factors OFF285->285 and ON285->266 for ordinary/refresh;
final ON267 is post-evaluation only.25valid become outliers before refresh,
27by final (16full-stereo/11left-only); maximum reprojection >3px zeroes whole
track. Cold triangulation does not restore post-step ON support; initial factor571
has cache sensitivity, so no blanket cache all-clear. All4owning-theta Gaussian
systems reproduce exactly. No graph optimizer, covariance work or source replay.
Strict world-origin invariance gate FAIL preserved; diagnostic extension keeps
thresholds and reports false. Pure rotations pass; artificial [3,-2,1]m world
translation changes6/18/11 classifications at initial/postordinary/final states.
Native algebraic DLT/rank sensitivity independently reproduced in4056NumPy SVD
classifications and3677point comparisons <=6.34e-11m. Not proof this artificial
coordinate shift causes actual full-run shaking or a CBS extrinsic bug.
Original compile helper collision and synthetic fixture default-mode failure
preserved; corrected standalone Release/ASan/UBSan fixtures pass, vendor DSOs
uninstrumented. Final18:24:10192protected inputs/Hesai667originals verified,
owned stages exited, both containers stopped. No weights/guards/production fixes.
Next separate suspect2 check: same-point/same-valid-set native visual Jacobians
and direct landmark elimination versus independent derivatives; not launched.

**CP-V55 FRAME/EXTRINSIC AUDIT (2026-10-30; COMPLETE):**
experiments/eig2_frame_extrinsic_cpv55_20261030/REPORT.txt, calibration.json,
payloads.json, native_attempt1/native_test.json, saved_native_attempt3/results.json.
Original EIG-2 TF matches actual camera/Hesai/Velodyne extrinsics <=3e-10matrix;
raw first headers and camera intrinsics agree. Both native graph poses CPT7.
CP49 ON1743G2K/1748K2G andCPV4 ON1744G2K/1757K2G means match sender payloads;
G2K aligned/frozen exact, previousCP51 actualfactor error<=1.445e-13 retained.
Native291smart factors per OFF/ON first-admission snapshot match correct left
rectified extrinsic/stereo calibration exactly, byte-nonmutation.1000native
nonidentity fixtures Release/san pass; missing/inverse/double/world-transform
negative controls all detected. PIM world-gravity gauge parity7.22e-16.
No CBS body conversion fault supported; do not add camera-LiDAR conjugation.
Historical native K2G finalfactor measurements missing, physical calibration
not independently reestimated,7/15explanatory sources historically pinned.
Unrelated right-camera visualization metadata defect documented (zero baseline
in getter vs correct .110085m optimizer stereo baseline); not CBS mean cause.
GLIM dual-OpenCV dependency observation qualified. Standalone compile/header/
link/reader failures preserved with isolated repairs, no estimator retry/tuning.
Verification17:34:9952protected/10037inputs/Hesai667 PASS; both containers stopped.
No production change/promotion/source capture/graphoptimization/covariance weighting.

**CP-V54 GLIM FIRST-EDGE OBSERVATION (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_glim_first_edge_cpv54_20261030/REPORT.txt,analysis.json,
verification.json,complete.json. One10s matched Hesai EIG-2 OFF/ON, CP51 model/
settings byte-identical apart from observer/paths. GLIM native X/V/B/PIM at
updates0..10:22readouts/110IMU-factor observations per mode;60Kimera snapshots/mode.
Joint GLIM5->7 vs own current-bias IMU .119420degOFF/.115709degON over.199967s;
native adjacent residuals .051560/.067944OFF,.046728/.069177ON. Raw RIGHT-support
gyro reproduces PIM <=1.67e-17deg; current-bias approximation vs reintegration
<=3.37e-14deg. Bias-choice gyro change .022960deg does not remove disagreement.
Not pure LiDAR-only residual, rawgyro NOT GT, no GLIMwrong/codingbug/fullrootcause
verdict. Same first ON admission6 at1.599933183s GLIM5->7/Kimera4->5; prior K2G
x4->x6. Native mean vs CP51 difference .001375/.000765degOFF/ON: not bitwise replay.
Private observer bridge d64bbb84f3bb2c7e...; release/san nonmutation/4-step optimizer
parity pass. Original test strict-zero PIM equality/Boostserialization setup failures
preserved; no observer/estimator numerical change or sanitizer suppression.
All original gates plus readout completeness pass; readout mean .165/.153ms,
max .374/.319ms excluding writes. Transport4077beliefs/mode,95poses,zero queued/
duplicates; retained timing caveats in report. Final16:51:43:9952protected/
9059retained/Hesai667originals verified; jobs exited, bothcontainers stopped.
No fullrun/scoring/covariance analysis/weights/defaultpromotion/deletion/push.
Remaining question is Kimera's amplified response to genuine conflicting rotation;
no further test or fix launched by this result.

**CP-V53 FIRST-EDGE ROTATION/IMU SEMANTICS (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_first_rotation_cpv53_20261030/REPORT.txt,analysis.json,
support_audit.json,verification.json,complete.json. Saved CP51firstadmission6
(1.599933s), nativeGLIM5->7 through transport/alignment to actualKimera x4->x5.
Chain mean matches; rawCPT7/camera-border integration reproduces savedPIMdeltaR
1.025e-17deg, endpointbridge6.441e-16deg.16.49/16.46ms offsets accountedfor.
Nativecurrent-bias PIM vs reintegration1.509e-9deg; Kimera vs gyro.000003073deg.
GLIMnative vs Kimera-bias-corrected gyro at SAMEsender times .099529degON/
.102394degOFF; OFF/ONmean difference.003943deg. AlignedGLIM-vs-Kimera.099531deg.
Disagreement preexists transport/bridge; no wrong-frame/time corruption found
for thisedge. GLIM ON alreadyreceivedK2G x4->x6 beforeexport, OFF mismatchalsoexists.
Rawgyro NOT independentGT, Kimera biasestimated; absent GLIMcontemporaneousbias/
PIM prevents verdictwhichrotationwrong. Not proofcodingbug/fullzigzagrootcause.
Four inspectedsourcehashesmatch inheritedpins; fivecurrentexplanatorysources
not historicallypinned, explicitlyqualified; native numericalchecks independent.
Private extraction only; zero graphsolve/source replay/covariancecalculations.
Release+ASan/UBSan/100SO3fixtures pass, wholecheckpointbyteidentity. Reporting
readonlybuffer/header-assumption/readerAPI failures preserved; freshv4 passes
unchangednumericalgates. Final16:05:21:9861pins/30analysis-supportinputs/Hesai667
originals pass, ownedjobs exited, bothcontainersrestoredstopped. No tuning/fix/
fullrun/defaultpromotion/push. Next ONLY proposed: inspect exact GLIMbias/PIM/
IMU-LiDAR residuals withshort passiveobservation ifapproved; notlaunched.

**CP-V52 FIRST-ADMISSION SINGLE-FACTOR ABLATION (2026-10-30; SAVED TEST COMPLETE):**
experiments/eig2_first_factor_cpv52_20261030/REPORT.txt/analysis.json/
visual_support.json; complete.json final authority. Exactly four native GNsteps
on CP-V51 update6 at1.599933s: A exact ON/B omit ONLY CBS929 x4->x5. Native
smart/PIM/prior/no-motion functions/weights and initial state identical. A native
equations exact; ordinary/chained final reproduction3.34e-10/1.23e-10mixednorm.
ActualOFF355 native functions match (smart caches excluded); initial values exact.
B vsOFF <=.0665mm/.001272deg. B/A old-window tilt .02019/.07859deg, max oldpose
rotation .02728/.32258deg; accelbias .002690/.022848m/s2; gyrobias .000343/
.007461rad/s. IMUcost .05256/1.25375; actualnativeCBS angularresidual .10012deg
without/.01722with. Cloned final smart reevaluation285valid B/267A;27valid become
outliers,3degenerate,12newlyvalid. Not a frontend-loss or live active-row audit.
Establishes first-factor-induced coupled response before removal/history, NOT
that initial small response is wrong or causes later largezigzag. Rootcauseopen.
Original compile name collision retained; fresh attempt_compile1 onlyrename,
no numerical/source retry. Release+ASan/UBSanfixtures pass;9084protectedpins,
snapshot identity, final15:36:07 container/job/Hesai667 checks PASS; bothcontainers
stopped/ownedjobs exited,732analysis inputs unchanged. No covariance analysis,
source replay, production fix, tuning, ATE or nonlinearstationarity claim.
Next ONLY proposed: audit this singleedge's native/aligned rotation against shared
raw CPT7 over exacttime/frame/bias conventions; not launched or a fullrun.

**CP-V51 HESAI INSERTION PHASE OBSERVER (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_hesai_insertion_cpv51_20261030/REPORT.txt/analysis.json/
factor_audit.json/verification.json/complete.json. ONE10s matched OFF/ON,
unchanged CP-V49 model/settings, new passive backend692197d81a75c926...;
Release+san BOTH-mode exact nonmutation and old-vs-disabled parity passed.
120snapshots before/after ordinary/after refresh, updates6--25, byte/native
roundtrips. First admission6/1.599933s; OFF/ON activewindows1--5exact. Actual
factor means match frozen aligned <=1.445e-13; one newCBS per observed ONupdate,
noneOFF; native initial+zero-velocity prior functions unchanged across120.
All60ON native H/RHS equal cache at owningtheta; tree/dense solution parity.
OFF intermediate caches can be incrementally stale; ALLafter-refresh exact.
At25/5.3999s ordinaryONtilt2.7366deg vs refresh.1070; accel-bias change.47136m/s2,
gravity-compensation directioncos.999708. IMUcost1873.29->319.70afterrefresh,
OFFfinal7.84; nettiltON2.8245 vsOFF.3271. Firstmarg26,firstCBSremoval30: early
disturbance not initiated by removal. No bug/fix or nonlinear stationarity proved.
OFF920activeposesexactCP49; ONnotexact(max.03163m/1.24129deg); firstmean1.48mm/
.00681degdifferent. Prefix reproduces early response, not full-runlargezigzag.
Legacy family labels include native no-motionx2->x3 andzero-velocityv3; actual
CBS separated by admission joins. Initial saved-inspector three-prior assumption
failed; fresh v2corrects reporting, preserves failed evidence; no source retry.
Bothsource/receiver gatespass;7688pins/272originaloutputs/120snapshots/Hesai667
checked, ownedjobs exited, originally-stopped containers restored. No weights/
covariance/tuning/fullcampaign/defaultpromotion/push. Next proposed only:
identical saved first-admission graph with/without exact first external factor,
inspect sixresidual/state-response directions; no ablation yet authorized/launched.

**CP-V50 HESAI ONSET/INSERTION AUDIT (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_hesai_onset_cpv50_20261030/REPORT.txt/verification.json.
Saved-only CP-V49 diagnosis; zero source/optimizer/build/covariance work/changes.
Active poses identical1--5; first admission/difference6 at1.59993s capture time.
At25/5.39990s ON old-window tilt3.43465deg vs OFF.32711deg, native weighted IMU
cost308.37vs7.84. First marginalization26/5.59990s, first peer removal30/6.39989s:
retirement-only explanation cannot account for early tilt. Not a first-bug claim.
First>.5m old-state revision295/59.39942s:ON.63224m vs OFF.00419m. ON published
step.62274m vs same-update motion.09351m.69large ON events predominantly rigid,
median99.99755% displacement energy explained, not primarily scale expansion.
3541export/incoming and1743aligned/frozen/pre/post means/IDs/causal-time joins pass.
1762-update logged ledger1743insert/1720remove/23remaining, slots/counts consistent.
No recovery. Native current-state refresh consumes no second CBS insertion.
Missing complete graph/theta/X/V/b/PIM and between-phase whole-state snapshots
prevents exact live cache/equation check or separation of ordinary/extra refresh.
No claim that profiles/guards prove all frames, timing or equations correct.
42inputs/272original outputs/7593pins/88report inputs/Hesai667copies+originals
and100independentSE3fixtures pass;containers stopped/owned jobs exited.
Next decision ONLY: short matched observation prefix, unchanged settings,
capture update6 and21--25 full phases, verify observation parity, then compare
cached/fresh native equations. No candidate fix or new capture launched.

**CP-V49 CORRECTED HESAI EIG-2 MATCHED PAIR (2026-10-30; COMPLETE):**
Authorized full three-estimator OFF/ON run, experiments/
eig2_hesai_refresh_full_cpv49_20261030/PROTOCOL.txt. Fresh roots/port11549;
original CP-K24 Hesai/front-stereo/CPT7/full355.02s offset5.7s, .20x/25s,
CP-V40 validated backend in BOTH modes, receiver Qc x1.0 and all weights fixed.
No rejected diagnostic candidate imported. OFF coverage/lifecycle/payload gates
plus saved native Method1 EVO and zero >0.5m same-state revision screen precede
ON. Report full/common-mask EVO ATE,0.5m RPE,path and revisions; delayed separate.
Preserve old Hesai667 checkpoint and all inputs. Forecast from old Hesai sizes;
owned jobs and stopped-container restoration required. No tuning or retry.
Launched once12:47:53local: launcher66944/controller66951; start12:48:01 and
both captures now complete.7593pins/667Hesai originals checked; receiver/settings parity
and exact input hashes passed.33.0GiB free/11.98GiB forecast. Automatic finite
OFF->gates->ON->gates->cleanup->EVO pipeline finished13:51:01; final complete.json
14:00:52verifies7593pins/88report inputs/Hesai667originals, jobs exited and both
containers restored. Controller status.json remains an earlier-stage snapshot.
Method1 shared352.2s (99.206%),reference188.0097m,348reference0.5m RPE pairs:
ATE OFF->ON GLIM.046029->.048301m; Kimera.515214->.862981m;
fusionONLINE.086494->.179195m; delayed.084784->.177431m separately.
Common paths189.8467->189.8845 /186.9171->296.6668 /199.3071->200.4111m.
Kimera translationRPE.030823->.228248m, rotation.819211->1.579625deg.
OFF>.5m same-state revisions0/1745; ON69/1747,max2.736418m. Native IMU-cost
median.511618->709.344, not unwhitened error. Severe OFF zigzag improvement
replicates with Hesai; ON instability persists while GLIM remains near5cm ATE.
Velodyne source degradation is not required for failure; specific root cause
and covariance calibration remain open. Next ON diagnosis only proposed, not run.
Full-native paths differ from common-grid values; both retained in full report.
ON1terminal omission/OFF0, both50initial exclusions; no pending versions.
48matched EVO CLI calls/API parity0; no source retries, tuning or promotion.

**CP-V48 NEW-ROTATION SEED ABLATION (2026-10-30; COMPLETE, CANDIDATE REJECTED):**
User approved CP-V47's saved test. experiments/eig2_rotation_seed_cpv48_20261030/
REPORT.txt,analysis.json,complete.json: five independent CP-V44 windows21--25,
A original/B only new pose rotation initialized with latest-bias IMU prediction.
Stereo translation, old states, velocity/bias, native observations/factors/
weights unchanged. Same ordinary+refresh; all five baseline gates before B.
Native ordinary/refresh equations match cached exactly; state reproduction
max5.925e-8 ordinary/5.880e-8 final mixed-unit tangent norm. Net old-window tilt
A/B degrees:21 1.550510/1.550486;22 4.692479/4.691204;23 2.540166/2.540160;
24 9.508939/9.511847;25 9.350214/9.353689. Initial IMU rotation residual becomes
<=4.45e-16deg, but no material tilt reduction. Actual final B-vs-A rotation
<=.06084deg/position<=1.047mm. Reject backend rotation seed as the dominant
local cause in these saved windows; no full rerun/deployment/promotion.
Initial cost improvement alone explicitly excluded from acceptance. Final IMU
cost at24 6191.63->6202.69 and25 456.25->466.20. Reported smart statuses from
cost-evaluation clones, not equated to Gaussian active factors. Native topology
unchanged.20experimental GN steps +4synthetic fixture steps; no source replay,
covariance investigation, production edit or new accuracy/stationarity claim.
Independent ON-conditioned starts do not test cumulative frontend history or
rematching. Full snapshots begin21, first divergence6 remains unlocalized.
Release/ASan/UBSan fixtures,10snapshot nonmutation,7519pins/Hesai667plus originals
pass;owned jobs exited/both containers restored. No retries/additional run.

**CP-V47 EARLY ON STATE/IMU CONSISTENCY (2026-10-30; COMPLETE, ROOT CAUSE OPEN):**
User authorized CP-V46's saved diagnosis. experiments/
eig2_early_consistency_cpv47_20261030/REPORT.txt,early_analysis.json,analysis.json
and complete.json. CP-V42 OFF/ON active poses identical1--5; first difference6,
first CBS admission sender gp5->gp7/receiver x4->x5. Final IMU cost at6:
.0531366 OFF/1.3464755 ON; same pre-solve3771.693459. Full X/V/b snapshots only
21--25; CP-V44 observer already differs from CP-V42 at6, not exact replay.
All old-state continuity/new-bias/latest-bias velocity prediction checks pass.
New pose rotation equals RAW PIM rotation to4.32e-15deg, but velocity uses
current-bias correction. Rotation discrepancy.03253--.12153deg; stereo/IMU
position discrepancy.01057--.06678m. At22--25 biasHat equals k-2 published bias
exactly, consistent with frontend reset-before-backend-return lifecycle. Older
integration reference is normal; IMU factor/predict applies correction. This
mixed-reference initialization is a testable lead, NOT proof of~10deg tilt cause.
25unique PIMs/340appearances/315byte comparisons unchanged;455factor-state rows,
deltaT error<=1.67e-16s, predicted-state self-error<=1.19e-17. Newest factor
dominates99.915--99.989%initial IMU cost; cost alone is not bug/accuracy evidence.
No raw reintegration or complete early frontend inlier/bias-reset receipts;
current frontend code separately hashed, not asserted original build provenance.
Original compile API failure preserved; explicit optional-argument-only repair
in fresh attempt_api1 passes Release/ASan/UBSan fixtures and15snapshot checks.
7406pins/37analyzed inputs/Hesai667plus originals pass, jobs exited/containers
restored. Zero source/optimizer calls, no covariance/tuning/production changes.
Next proposed ONLY: same-checkpoint initialization comparison replacing just
new pose rotation with latest-bias IMU prediction; stereo translation, old
states, native factors/observations/weights unchanged. Same ordinary+refresh.
Require subsequent tilt improvement, not trivial lower initial IMU cost; reject
if tilt persists. Not executed/authorized here, no full sequence or promotion.

**CP-V46 SAVED CBS EQUATION ABLATION (2026-10-30; COMPLETE, FIX CANDIDATE REJECTED):**
experiments/eig2_cbs_ablation_cpv46_20261030/REPORT.txt, analysis.json and
complete.json. Five CP-V44 pre-ordinary-solve starts21--25, one step per arm:
A removes current explicit CBS, B original, C exact CBS Local/Log Jacobian only.
All B equations reproduce exactly; state error<=5.925e-8 mixed-unit tangent
norm. Old-window tilt A/B/C degrees:21 10.024/.482/.510;22 11.966/5.246/5.246;
23 9.327/2.481/2.487;24 9.128/5.683/5.719;25 15.258/10.212/10.182.
Actual C-vs-B rotation<=.05081deg/old-position<=2.974mm: Jacobian-only change
does not remove large tilt. A worsens tilt magnitude/native cost in all5.
A is ON-conditioned, NOT an OFF reproduction or exoneration of prior CBS.
Native smart/initial-prior approximations unchanged; no stationarity/ATE claim.
Release/ASan/UBSan fixtures,3420exact-Jacobian checks,5snapshot byte nonmutation,
7296pins/Hesai667pass;owned jobs exited/both containers restored.15independent
one-step solves only, no source/covariance/tuning/production patch/promotion.
Source SHA b8988101435cc490987fc7c0eae15885a4bad9e1f464842a18c8ebe88bb39d51.
Next proposed only: locate first ON tilt/bias inconsistency, inspect new-state
initialization/IMU bias-reference consistency through revisions and native
visual activity before choosing another fix. No further test authorized here.

**CP-V45 ON NATIVE CONVERGENCE ADMISSION (2026-10-30; DIAGNOSIS COMPLETE):**
experiments/eig2_on_convergence_cpv45_20261029/REPORT.txt and analysis.json.
Five CP-V44 final native graphs, unchanged factors/priors/weights. All5fail the
exact objective-derivative gate; zero iterative solves.95/95CBS BetweenFactor
instances use the fast Jacobian omitting Local chain rule; full chain agrees
with finite differences (3420checks). Stacked CBS gradient discrepancy only
.137--.150%, so NOT a proven cause of~10deg tilt. Initial pose prior identity
Jacobian and known smart cost/Schur mismatch also fail; IMU/bias/other priors
pass.13fixed fractions of one native GN direction/window: full step lowers
cost4/5but net tilt increases in all5; at24 IMU6191.63->285.79 while tilt
9.51->9.81deg. No claim more iterations fix/cannot fix, stationarity, physical
accuracy or covariance interpretation. Release/sanitizer fixtures and exact
5snapshot nonmutation pass;7090pins/Hesai667verify,containers restored.
No source, covariance work, model substitution, production patch or promotion.
Proposed only: saved equation ablation of CBS Jacobian chain rule alone to
measure causal sensitivity before any live change; not executed/authorized.

**CP-V44 ON PHASE CHECKPOINT (2026-10-29; COMPLETE):**
User approved ONE10s original-start ON observer experiment, excluding covariance
investigation. experiments/eig2_on_checkpoint_cpv44_20261029/REPORT.txt and
analysis.json/gravity_coupling.json:15native phase checkpoints21--25, unchanged
CP-V42/CP-V40/Qc x1.0 settings. Ordinary solve tilts window5.25/5.68/10.21deg
at22/24/25 with accel-bias changes.895/.973/1.750m/s^2 matching gravity projection.
Refresh mostly repairs IMU inconsistency but adds3.94deg tilt at24. Fresh/cache
equations exact; independent native solves reproduce live states<=5.93e-8 norm;
ordinary theta already current. Thus not the same early stale-linearization
mechanism as OFF, but no nonlinear stationarity or specific bug certification.
Release/sanitizer nonmutation and disabled-original parity pass.48native Kimera,
42G2K/43K2G, transport/lifecycle/receiver gates pass. Prefix differs from CP-V42
starting6 before observer21; no exact-run identity claimed.6590pins/Hesai667,
owned jobs exited/containers restored; final_verification.json retains evidence.
No weights/solver change, full rerun, source retry or promotion. Next proposed
only: saved-native-graph derivative/convergence check with unchanged factors.

**CP-V43 ON MECHANISM LOCALIZATION (2026-10-29; SAVED DIAGNOSIS COMPLETE):**
User explicitly excludes covariance investigation. Saved CP-V42 logs/MAIN poses
and executed CP-V40 code only; trace insertion/ordinary solve/added refresh/
retirement and mean timing/frame arithmetic. Protocol:
experiments/eig2_on_mechanism_cpv43_20261029/PROTOCOL.txt. No source replay,
build, weights/prior changes, covariance calculations, scoring or promotion.
REPORT.txt/analysis.json/transport_means.json:3540export-to-wire means and1744
exact receiver endpoint joins pass. ON coherent tilt11.13deg at25/5.4s precedes
first marginalization26 and peer retirement30; retirement cannot solely explain
the early anomaly. Ordinary/final peer residual medians nearly unchanged by
extra refresh. Four same-graph objective rises;1016 increases621025->3423990
while peer residual improves. This is a full-solve/acceptance clue, not a proven
specific equation defect or proof that extra iterations alone fix the run.
472/475large revisions checked, median99.9703%rigid displacement explained;
missing previous snapshots explicitly listed. Complete graph/PIM/X/V/b and
intermediate solve-state costs absent. Proposed only:~10s original-start ON
observer capture21--25, unchanged factors/weights/settings; not launched.

**CP-V42 FULL MATCHED CORRECTED EIG-2 (2026-10-29; COMPLETE, ON STILL UNHEALTHY):**
User renewed CBS-ON authorization following CP-C5/6 storage cleanup. Read
experiments/eig2_refresh_full_cpv42_20261029/PROTOCOL.txt and REPORT.txt.
Fresh355.02s offset5.7s OFF then ON with identical tested CP-V40 Kimera,
0.20x/25s/original Qc x1.0 and frozen remaining settings. CP-V39 only ran
Kimera OFF; new full all-three OFF is necessary for the matched comparison.
Preserve gates/no retries/ownership and all originals. Controller performs
captures/gates/cleanup; direct EVO Method1 report remains a separate saved-data
stage, not old custom scoring. Both full captures and all gates now passed;
jobs exited/containers restored21:16:56local;6253pins/Hesai667verify.
Launched ONCE20:13local, launcher139842/controller139849; inspect status.json
and per-stage receipts, never compete with its active capture/validation.
Preparation6253pins/Hesai667pass;37.10GiB free exceeded11.33GiB pair forecast.
Final RESULTS_REPORT.txt/results.json: direct EVO1.36.3,48CLI calls/API error0,
Method1 common352.3s99.234%/3526queries/348reference0.5m pairs. ATE OFF->ON
GLIM.390391->.384648m(+1.47%), Kimera.515187->2.073638m(-302.50%), fusion
ONLINE.518175->.962769m(-85.80%). Paths204.14->204.20/186.92->791.88/
209.46->212.92m(ref188.01m). Full-native ONLINE paths294.43->259.57m differ
materially from interpolated-grid lengths; preserve both. DELAYED fixed-lag
fusion ATE.516078->.961244m/path187.57->187.33m, not online/full-history.
Kimera previous-state revisions max.288->7.271m and >.5m0->475; translation
RPE.030823->1.072331m/rotation.819211->5.622269deg. Internal ON IMU cost
median2366/max3.17e6 remains high; not itself accuracy/stationarity evidence.
OFF improvement holds, this refresh alone does NOT resolve ON instability.
Transport covariance error0;50pre-init exclusions each/0terminal omissions/
0expired/stale/duplicate/pending. No retries/promotion/tuning or new source run.
Next proposal only: saved ON-specific localization; not authorized by this ledger.
Final complete.json21:27:21 verifies captures+report/6253pins/88report inputs/
Hesai667 and stopped containers. status.json is the earlier controller snapshot.

**CP-V41 BOUNDED EIG-2 ON INTEGRATION (2026-10-29; PASSED, FULL PAIR STORAGE-BLOCKED):**
experiments/eig2_on_refresh_smoke_cpv41_20261029/ contains protocol/prepared/
launch/start receipts; launcher120021. Single30s original-start5.7s,0.20x,
25s drain, validated CP-V40 Kimera; all other settings/Qc x1.0 unchanged.
All sensor/LOCAL/source-lifecycle/receiver gates passed.148Kimera/291GLIM/
295ONLINE poses;142G2K/143K2G,118retirements,16249exact versions;50initial
exclusions,0EOF omissions. No ATE/RPE/full accuracy or stationarity claim.
REPORT.txt/complete.json/closeout.json: jobs exited, containers restored,
6184pins/Hesai667verify. No repeat of this bounded capture.
Full pair NOT launched:9.84GiB free,11.33GiB forecast with25%headroom; even
unchanged original sizes leave7.34GiB before ON (<8GiB launch guard). Need
at least2GiB more free (3recommended) or user-selected output filesystem.
No deletion/guard relaxation/default promotion. Full corrected OFF/ON remains
authorized conditional on adequate space; use CP-V40 in BOTH new modes.

**CP-V40 ON REFRESH ADMISSION (2026-10-29; FOCUSED GATES PASSED):**
User authorized proceeding to CBS-ON safety/bounded gate then full matched
EIG-2 only on success. First private engineering gate under experiments/
kimera_on_incremental_refresh_cpv40_20261029/PROTOCOL.txt. Extend CP-V38's
one-step native refresh to temporary ON with preserved priors/weights, no
duplicate belief processing, post-refresh residual/slot/retirement checks and
whole-transaction rollback. No source replay inside CP-V40; no promotion/tuning/deletion.
REPORT.txt/closeout.json now complete. Release twice deterministic, ASan/UBSan,
disabled OFF/ON and enabled OFF parity, nine-retirement lifecycle, first/mature/
post-marg rollback pass. ON prior function unchanged, LOCAL12x12<=2.619e-12;
no dataset accuracy/stationarity claim. Private backend SHA
750b8c8ffed61d186ec8f071d4f58f068c9b63baccefcc9fc37803d77a118f06.
5966protected pins/Hesai667verify; jobs exited/containers restored. Two failed
fixture-stage attempts retained; numerical candidate/tolerances unchanged.

**CP-V39-VIS METHOD1 RERUN VIEW (2026-10-29; COMPLETE):**
Portable recording/README/open_rerun.sh/verification.json/REPORT.txt under
/home/yeranis/results/eig2_off_refresh_method1_cpv39_20261029/.
Three tabs: new Kimera/reference, before-after, old/reference; CBS OFF only.
All1761absolute backend points each,71004reference,3518447saved Velodyne map
points. Exact CP-V39 common-mask EVO fits, all native samples/gaps retained;
no smoothing, scoring rerun or source/receiver launch. All RRD points/edges
and input hashes verify. Default preview inspected; known screenshot-shutdown
exit102 preserved separately; normal desktop viewer opened. Original evidence
untouched; late timestamp/mask qualifications remain. Not a new GLIM/fusion run.

**CP-V39 FULL EIG-2 OFF REFRESH — POSITIVE PRIVATE EXPERIMENT (2026-10-29):**
See experiments/eig2_off_refresh_full_cpv39_20261029/REPORT.txt, results.json,
REPORTING_REVIEW.txt and final closeout.json. One full355.02s/start5.7s0.20x/25s
Kimera-only OFF run, tested CP-V38 lib SHA560ad4e82323e91db43eae606bafcd3ecec6ccbd85dcf271fb84a7f50658ecf4.
Original prior function/weights/native smart policies preserved; no GLIM/fusion/
incoming CBS/ON/tuning.1761outputs; both containers restored stopped afterward.
Strict timestamp parity failed:126late keyframe stamps differ from311s onward,
76distinct selections. Complete3550stereo frames/35502CPT7 samples received;
adaptive native keyframe policy exists, exact late difference cause unproven.
No identical late factor-set claim. Saved-only fallback intersection per protocol:
7extra excluded queries,352.3s99.23%coverage,348EVO reference-distance0.5m pairs
(old349). Same support/gap rules/SE3/prism protocol; no source retry.
Direct EVO common ATE M1 2.309533->.515187m(-77.69%), M2 1.880779->.482241m(-74.36%).
Common path M1 290.30->186.92m(ref188.01); full-native M1 332.02->188.05m different
cadence. Translation RPE M1 .452902->.030823m, M2 .088765->.017921m; M2 rotation
.80718->.80848deg essentially unchanged/slightly worse. Revision max4.85->.288m,
observed>.5m112->0; paired MAIN steps1744/1745. Native IMU cost max4.22e9->27.08;
not unwhitened residual/accuracy or nonlinear stationarity. Prior-only legacy
counter omits OriginalChartPrior; total includes it. Native optimize mean143.2
->94.6ms, workload differs so no speedup/real-time certification. Strong support
for retained-linearization pathology in our modified baseline, not universal
upstream/other-sequence/ON fix. Preparation and strict reporting failures retained.
No production/default promotion, tuning, deletion or push; original protected
artifacts/Hesai667/RRDs checked in closeout. Further OFF verification proposed only.

**CP-V38 PRIVATE OFF INCREMENTAL REFRESH — FOCUSED GATES PASSED (2026-10-29):**
See experiments/kimera_off_incremental_refresh_cpv38_20261029/REPORT.txt,
PROTOCOL.txt and final closeout.json. Isolated CP-K9-derived headers, opt-in
CPV38_OFF_REFRESH=1: ONE additional current-state native GN step per OFF update
before marginalization, immutable original-prior function and native smart
factors/weights retained; no core rebuild/LM/convergence loop. All-key native
refresh after accounted prior promotion; normal update cadence/slots retained.
Default-OFF old/new output/covariance exact; active Release twice deterministic,
ASan/UBSan pass (vendor DSOs uninstrumented). Nine retirements per13-update
fixture, prior energy error0, independent-GN error1.60e-19, dense Schur1.38e-14,
LOCAL joint covariance<=6.25e-12 and exact rollback on refresh/post-marg failures.
These are mathematical/lifecycle checks, NOT stationarity/ATE/RPE/accuracy.
Private backend built and dependencies resolved; SHA
560ad4e82323e91db43eae606bafcd3ecec6ccbd85dcf271fb84a7f50658ecf4.
Initial Git inventory, test brace and incorrect post-marg adapter-count failures
preserved; candidate code and numeric tolerances unchanged after first compile.
Containers restored after gates; closeout checks5613pins/Hesai667/original RRDs.
No source replay/CBS-ON, tuning, production/default change or push. Proposed next
bounded300s original-start Kimera-only OFF comparison is NOT launched here.

**CP-V37 EIG-2 OFF NATIVE REFRESH — COMPLETE, NO LIVE FIX (2026-10-29):**
See experiments/eig2_native_refresh_cpv37_20261029/REPORT.txt, results.json and
closeout.json. Seven saved CP-V36 checkpoints; ONE undamped native-factor step
at current estimate, immutable original prior function/anchor and all weights/
native smart policies retained. No joint landmarks, iterations, damping, source
replay, CBS-ON or production change. All seven IMU position/rotation/velocity
residual maxima improve;267.8s position .827053->.027510m,268.0s .983587->.061230m,
NOT trajectory errors. Old-equation control within36.7um/9.19e-5deg of capture,
remaining bad residual unchanged.42prior energy/derivative checks, Release and
ASan/UBSan original/displaced fixtures and input nonmutation pass. Linear solve
residual<1.65e-10 is NOT nonlinear stationarity. Smart validity changes0--71,
visual cost worsens4/7; large corrections up to3.175m/19.10deg remain. Supports
retained-linearization mechanism without certifying cause, full path or live
fix. Proposed private incremental OFF integration/lifecycle gate NOT executed.
Unused-libcbs dependency assertion failed before data; original failure retained,
actual pinned GTSAM dependencies checked, seven never-started cases ran once
with unchanged executable. Containers restored; original/Hesai/RRD pins pass.

**CP-V36 EIG-2 OFF COUNTERFACTUAL — COMPLETE, NO LIVE FIX (2026-10-29):**
See experiments/eig2_counterfactual_cpv36_20261029/REPORT.txt,
capture_verification.json, diagnostic_results_v2.json, equation_audit.json and
closeout.json. One300s original0.20x Kimera-only OFF replay,1498native pose/cost
rows and37100MAIN payload rows EXACT original parity. Seven checkpoint readbacks
have zero factor-error/linearization difference. Private observer nonmutation
and offline derivative fixtures pass Release/ASan/UBSan (vendor DSOs excluded).
Cache=fresh equations at STORED theta; cache/tree H,rhs agree~1e-16. Scaled
linear residual<1e-6 does NOT imply nonlinear consistency: max native IMU
position residual267.8s .827m ->1.05e-6m in conditional joint-landmark comparator,
same weights/measurements/original-chart marginal function. Not ATE/RPE.
All8457 joint directional checks pass; native mismatches exclusively smart
stereo. Five of14starts meet gradient gate;9hit30iterations, including all8peak
starts. Native activity changes0--275, joint distance-policy violations2--23;
no all-native-domain stationary-solution or live/full-trajectory fix claim.
Private backend SHA a1fac8bbc12a3b6046fa2f17f3e73431c19f47aee3b3c83beae89d22ce77f67e.
No prior strengthening, CBS-ON, GLIM, fusion, production change or source retry.
5131original/40private pins,667Hesai files and original RRDs preserved; jobs
exited and containers restored stopped. Preflight and report-writer failures
retained; v2 JSON is authoritative. Next proposed single ablation: refresh
native-smart/current-state linearization with unchanged original prior function
on SAVED checkpoints before any separately approved live repair. Not executed.

**CP-V35 EIG-2 KIMERA OFF BASELINE — COMPLETE, CAUSE/FIX OPEN (2026-10-29):**
User authorized OFF-first diagnosis. See experiments/
eig2_kimera_off_cpv35_20261029/REPORT.txt, verification.json, diagnosis_summary.json,
off_diagnosis_v2.png and closeout.json. One300s original-start0.20x Kimera-only
capture; cameras/CPT7/CP-K9/FL4Q0/remove/priors unchanged, no incoming CBS or
GLIM/fusion/ON/tuning. Private copied-factor observer passes Release+ASan/UBSan
wrapper parity fixtures (initial faulty direct-BPSAM fixture failures retained).
All1498native pose/cost rows and37100MAIN revision payload rows exactly match
original OFF through299.797532s.61updates/183stage snapshots localize immediate
revisions to solve, zero surviving-state change at following marginalization.
Native IMU position residual .372->.827->.984m around267.6--268.0s, not ATE;
IMU dt remains~.2s.25/27pose anchors fixed near peak, fixed-anchor rotation
departure19.46deg at1336. Marginal-prior finite common-motion energy softens,
but neither weak-prior cause nor fix demonstrated. Investigate linearization/
chart/cache/solver consistency before CBS-ON; unchanged weights and original
prior-chart nonlinear counterfactual proposed, NOT launched/authorized here.
Current snapshots are not full resumable factor/PIM/cache checkpoints.
Private backend SHA241ee44bca2262d9ff0f7eddd508fa0d30e912e7c6a6d25621907611202cb101.
2871original/38private pins and667-file Hesai checkpoint pass, old RRDs intact.
Capture/owned cleanup succeeded; both containers restored stopped. Post-drain
rostopic warning parsing repaired only in reporting; no source retry or accuracy
rescore. CP-V34 objective-absence statement corrected:1761original H/V Kimera
FL4M rows identical, maxnativeIMUcost4.22e9. Exact bug/fix remains unresolved.

**CP-V34 EIG-2 KIMERA JITTER DIAGNOSIS — COMPLETED, ROOT CAUSE OPEN (2026-10-29):**
User requests diagnosis of pronounced Method1 Velodyne EIG-2 OFF/ON shaking:
shared IMU, belief frames/transforms and Kimera window/scale hypotheses.
Read experiments/eig2_kimera_jitter_cpv34_20261029/REPORT.txt, verification.json
and window_mechanism.png. Original Kimera OFF absolute arrays are exactly equal
between Hesai/Velodyne. Saved MAIN snapshots show previous-pose revision p95
OFF/HesaiON/VelodyneON=.603/.725/1.968m, while same-update motion p95~.193m.
At219.798s VelodyneON:6.343m previous-pose revision, .164m local motion;25shared
poses move coherently13.94deg/6.01m centroid, max rigid-removed shape residual
2.24cm. Median rigid-explained99.9797% over479large moving-window events; NOT
primarily scale pumping. OFF also has nonrigid deformation. Shared CPT7/body
settings confirmed;3540GLIM endpoint exports match incoming bag exactly;
1744V/1740H raw/aligned/frozen candidates match with no unexpected transform.
Endpoint offsets~41msV/~17msH compensated in mean, covariance unchanged: known
approximation remains a suspect, not proven cause. Initial prior is strong;
effective marginal prior/linearization/CP-K9 retirement interaction unresolved.
Existing direct-evo metrics retained; native incremental ON RPE also worsens.
98pins pass; see closeout for667-file original Hesai checkpoint. Proposed300s
Kimera-only parity/phase-cost/prior diagnostic is NOT launched or authorized by
this note. No source/receiver run, tuning, build or production repair.

**CP-V33 ALL-SEQUENCE METHOD1 RERUN EXPORT — COMPLETED (2026-10-29):**
User requests Method1 trajectories alongside Method2. Read experiments/
all_method1_rerun_cpv33_20261029/PROTOCOL.txt. Export all seven original CP-V27
OFF/ON pairs/Qc1.0 with full native absolute GLIM, recovered absolute Kimera
backend and fusion ONLINE paths. Reuse CP-V32 direct-evo SE3 transforms and
metrics, identical CP-V30 reference/cloud context. New separate RRD folder,
no smoothing/integration/cropping, no experiment/model change. Preserve all
Method2 files; exact geometry/readback and native screenshot checks required.
REPORT.txt/verification.json/closeout.json: seven RRDs236772334bytes in
/home/yeranis/results/rerun_method1_all_sequences_20261029/. All113732native
estimator vertices and17534190cloud points retained; all42common samples and
evo APE error arrays exactly reproduce. Full paths reproduce using evo, maximum
render quantization3.813e-6m. Every RRD vertex/edge/cloud and132pins pass;
all10Method2 output files unchanged; Hesai667-file checkpoint intact. Seven
default OFF renders inspected; ON geometry exact readback (no interactive-tab
audit). Known screenshot shutdown panic102 after image save retained, all owned
screenshot subprocesses exited. Separate absolute Kimera BACKEND semantics
explicit in labels/notes. No new source/receiver run or model/settings change.

**CP-V32 ALL-SEQUENCE EVO TABLES — COMPLETED (2026-10-29):**
User requests CP-V31 delta0.5m RPE for all seven baseline pairs and added columns
in both method1/method2 big tables. See experiments/all_evo_tables_cpv32_20261029/
PROTOCOL.txt. Preserve CP-V27/28 data/common masks/Qc1.0; reuse completed ETH1
RPE and score the other six pairs identically with evo1.36.3. Recompute all84
table ATE entries directly using evo SE3-aligned APE translation RMSE; supported
segment path lengths via evo. No custom metric formula or gap bridging. Save
commands/evo ZIPs, tables/report, numerical parity and protected hashes. No
source/receiver run, model edits, container, build, deletion or default change.
REPORT.txt/METHOD1_TABLE.txt/METHOD2_TABLE.txt:42combined rows,84direct-evo ATE
and168RPE entries.372newCLI calls all pass, API error arrays exactly identical,
516newZIPs read back.225pins unchanged; oldATEmaxdifference1.159e-14m; old paths
exact; Hesai667-file original checkpoint passes. RPE pair counts365/348/167/378
for Hesai ETH1/EIG2/SPX3/Leica2 and167/349/378 for Velodyne SPX3/EIG2/Leica2.
Reference-distance actual step ranges and common coverage disclosed, no gap
bridging/scale correction. Method2 CBS improves0.5m translation RPE in GLIM5/7,
Kimera5/7, fusion4/7; rotation improves7/7,2/7,4/7 respectively. Descriptive
single-run counts only. Original reports retained, no estimator/receiver rerun.

**CP-V31 ETH1 EVO DISTANCE RPE — COMPLETED (2026-10-29):**
User requests RPE with path-length delta0.5m and exclusively evo metrics.
Saved original Hesai ETH1 OFF/ON, both clearly separated CP-V27 method1 and
CP-V28 method2. No fresh CP-V29 mix-in. See experiments/
eth1_evo_rpe_distance_cpv31_20261029/PROTOCOL.txt. Use evo1.36.3 directly,
consecutive pairs selected from reference path, unchanged common masks/prism
coordinates, no gap bridging, alignment or scale correction. Save native evo
result archives, exact pair IDs and command receipts; verify CLI/API parity.
REPORT.txt/results.json/closeout.json:24CLI calls passed,48native evo result
archives/39TUMs.365identical reference distance pairs; sampled lengths
0.500561--0.593297m, default consecutive selector, no gap bridging. Only the
287.2s common segment yields pairs; two short segments yield none. Original
346.32s capture is not fully scored. Method2 translation OFF/ON(m) GLIM
.020438/.018545, Kimera .330896/.016697, fusion .064150/.039518; rotation(deg)
.370255/.360012, .448391/.461237, 2.375301/2.219676. All54input/runtime pins
unchanged, CLI/API error parity0, original-array roundtrip<=1.552e-12; Hesai
checkpoint667files pass. All future reported accuracy metrics must use evo
directly per user; prior custom metric tables remain historical, not evo-only.
No source/receiver rerun, build, tuning, deletion, default promotion or push.

**CP-V30 ALL-SEQUENCE METHOD2 RERUN EXPORT — COMPLETED (2026-10-29):**
User requests reusable RRDs for visual inspection, before further diagnosis.
See experiments/all_method2_rerun_cpv30_20261029/PROTOCOL.txt. Seven original
CP-V28 OFF/ON pairs, original Qc1.0, all saved native method2 trajectories,
common-mask rigid alignment and saved ON cloud context. No new experiment,
receiver replay or model change; separate fresh CP-V29 OFF is not mixed in.
Destination: /home/yeranis/results/rerun_method2_all_sequences_20261029/.
REPORT.txt and verification.json: all42commonATE/full-path checks pass,
113,732native estimator points and17,534,190saved ON cloud points retained.
All seven RRDs have exact vertex/edge/cloud readback;819inputs unchanged,
Hesai checkpoint PASS667files. Seven default OFF renders visually inspected.
Screenshot-only host runs save images then hit known Rerun shutdown panic;
receipts retained. ON data verified exactly, no separate interactive-tab audit.
Seven portable RRDs total236,762,911bytes, plus README/opener. No new solver,
source/receiver replay, model change or persistent viewer; next diagnosis paused.

**CP-C4 NON-GRANDTOUR OUTPUT CLEANUP — COMPLETED (2026-10-28):**
User explicitly requested deletion of obsolete non-GrandTour runs/checks.
experiments/non_grandtour_cleanup_cpc4_20261028/REPORT.txt indexes exact path
inventories and two finite deletion receipts.30,669files permanently removed,
15.44GB reclaimed,22.04GB free.3,922protected GrandTour/reference/dependency
files remain hash-identical; four-sequence Hesai checkpoint PASS667files.
Raw datasets, source/safety snapshots, current builds and all GrandTour runs
excluded. No new bulk backup/Trash; no experiment/build/settings change.
This resolves CP-V29's storage blocker without relaxing disk guards.

**CP-V29 LIVE GLIM INCREMENTAL OUTPUT — COMPLETED (2026-10-29):**
Authoritative final evidence: experiments/eth1_live_incremental_cpv29_20261028/
REPORT.txt,results.json,verification.json,final_closeout.json. Full OFF capture
passes original gates. Live GLIM3454ROS poses agree with independent same-run
chain within2.193e-10; paired CSV contains2additional shutdown-only callbacks,
not ROS receipts (bag verifies), retained separately. No interior loss.
Method2 full-saved ATE/path: GLIM .324791/204.521962, Kimera4.496229/196.006964,
fusion2.383948/199.278569(m). Common296.7s mask unchanged, ATE .338390/4.500022/
2.488463. GLIM same-run absolute .046827m: live chaining does not recover that
accuracy. One reporting-only replay in original Humble environment passes
allbyte/event/commit parity and1678pins. Host/container check failure and strict
tail-count scoring failure preserved; actual-ROS supplemental analyzer completes
without source retry or tuning. All7TUM readbacks pass,2499protected hashes and
667Hesai files unchanged, containers stopped/owned jobs exited. No ON launched.

Historical preparation/launch details:
User requests one fresh full Hesai ETH1 experiment. See
experiments/eth1_live_incremental_cpv29_20261028/PROTOCOL.txt. One CBS-OFF run,
CP-K23 settings/Qc1.0, private opt-in output-only bridge; keep original absolute
poses/graphs/beliefs unchanged. Validate the actual C++ accumulator against
saved CP-V28 pairs before capture and new-run graph rows afterward. No tuning,
ON campaign, default promotion or baseline overwrite. Finite controller launched;
read status.json/controller.log for actual stage, never duplicate this attempt.
Initial6.3GiB storage block is now cleared by explicitly authorized CP-C4 cleanup
(20.5GiB available); unchanged8GiB admission and5GiB runtime guards remain.
Private observer built; normal and ASan/UBSan tests each11921checks/3454saved
rows PASS,maxerror1.927e-11.2499protected hashes PASS. LauncherPID140594, scoped
ETH1-CPV29 sleep inhibitor. Initial tool approval timeout launched nothing;
verified absent receipts before the one successful launch retry. One reporting
CP-V23 journal observer follows original capture gates, strict byte/event/commit
parity required. All-three method2 plus absolute/delayed/full/common reporting.
Playback confirmed23:57local: effective settings PASS, original and new GLIM
topic rows matched, Kimera/fusion publishing, queued0. ControllerPID140596 owns
all further stages; completion still pending. Original artifacts unchanged.

**CP-V28 SEVEN-PAIR METHOD2 COMPARISON — COMPLETED (2026-10-28):**
Report: experiments/all_incremental_comparison_cpv28_20261028/REPORT.txt;
summary.json,verification.json,closeout.json retain full evidence. User-selected
method2 for ALL THREE, both RMSE and path: GLIM saved same-update MAIN pairs,
native Kimera incremental ROS CSV (not chained twice), fusion same-publication
pairs from12saved-journal replays using unchanged CP-V23 observer binary SHA
ae79b1f9646843e8dec4456e29e5174e59233c79156a55ebac77d0ae74cfb6c1.
Reused Leica-2 Velodyne CP-V23/26, no repeat. Original Qc1.0, no source/build/
model/tuning change. Every replay matches original ONLINE/finalized files,
summary, deterministic events and publication commits.92124chained poses read
back with exact timestamps;1779replay/dependency and157host inputs verified.
Independent evo ATE discrepancy<=1.56e-14m; matrix RPE discrepancy<=1.39e-11.
All old native Kimera and completed Leica-2 method2 scores reproduce. Frozen
CP-V10 common masks/reference/lever/SE3 fit retained; full native scores and
coverage separate. Common ATE winners Kimera8/14,GLIM6/14,fusion0/14; CBS lowers
ATE GLIM2/7,Kimera6/7,fusion4/7. This causal chain removes graph corrections
from accumulated motion; shorter paths do not ensure smaller drift. Not a
claim about GLIM authors' standard benchmark output or matched evidence age.
Storage preflight stopped after3successful cases before the fourth launch.
Original failure preserved; documented infrastructure-only continuation ran
only9unstarted cases with actual tmpfs/projected persistent capacity checks;
no numerical/parity guard change or duplicate. New evidence1,517,421,515bytes
archived losslessly, every member verified. Staged products restored; only
verified temporary RAM duplicates removed. Both containers restored stopped,
owned jobs exited, Hesai checkpoint PASS4cases/667files. Original reports/
artifacts/EOF omissions retained, no default promotion/commit/push. CP-V27 is
separate method1 evidence, not the response to the user's method2 request.

**CP-V27 SEVEN-PAIR NEWEST ABSOLUTE COMPARISON — COMPLETED (2026-10-28):**
Read experiments/newest_absolute_comparison_cpv27_20261028/REPORT.txt and
summary.json. Four Hesai/three Velodyne pairs; saved-data only, original Qc1.0.
All21608Kimera absolute backend poses recovered from same-output diagnostics;
every paired incremental pose/unique keyframe ID matches ROS CSV, exact CSV
nanosecond times used. Six-digit pose logging bound<=0.504mm, not bit-exact.
GLIM first-published and fusion ONLINE unchanged; all42old full/common ATE
scores reproduce exactly, CP-V10 masks/reference/lever/alignment retained.
Absolute common ATE Kimera OFF/ON: Hesai ETH1 4.209184/1.606029,EIG-2
2.310051/1.101293,SPX-3 .361816/.221140,Leica-2 1.005786/.773176;
Velodyne SPX-3 .361645/.271351,EIG-2 2.307834/1.801042,Leica-2
1.005375/1.532446m. GLIM lowest13/14 modes,Kimera1,fusion0.
Kimera absolute publication traces also inflate path (ETH1 ON782.32m,
Velo EIG-2 ON790.17m common); not physical motion or internal same-update
trajectory proof. Native incremental and delayed outputs remain separate.
109input pins, independent log-relative-pose/evo/RPE checks pass; Hesai
checkpoint4cases/667files verifies. Full-saved/path/coverage tables retained.
No new estimator/replay/settings/defaults/container/build/deletion/push.
Common output policy does not equalize evidence age/cadence/latency. Original
CP-V10/CP-V26 reports remain valid for their labelled products; no source fix.

**CP-V26 ALL-INCREMENTAL LEICA-2 OUTPUTS — COMPLETED (2026-10-28):**
User selects method2 for all three. Bounded saved-data comparison in
experiments/leica2_all_incremental_cpv26_20261028/REPORT.txt/results.json.
GLIM reconstructed from4280same-update pairs per mode/current published poses
exact; native incremental Kimera unchanged, validated CP-V23 fusion chains
reused,original Qc1.0. Frozen CP-V10 common mask3850queries/384.6s.
Common ATE OFF/ON GLIM10.106881/3.331124,Kimera1.094234/2.585334,
fusion3.482370/5.331684m. Full-saved GLIM11.931732/3.867642m versus original
absolute2.829284/.863203m: substantial global correction benefit lost in chain.
Kimera lowest method2 ATE both; CBS improves GLIM but worsens K/fusion here.
Independent chain max1.10e-11,graph reconstruction1.62e-12,edge lengths6.22e-14;
all six historical product full/common scores exactly reproduce,12historical
input pins match. Common rule does not equalize cadence/evidence age/latency.
No source/receiver run,build,container,tuning/default promotion/deletion/push.
Only Leica-2 Velodyne; preserve all original products and other sequences.

**CP-V25 LEICA-2 OFF SINGLE-SOURCE RECEIVER CONTROLS — COMPLETED (2026-10-27):**
experiments/leica2_single_source_controls_cpv25_20261027/REPORT.txt/results.json.
Exactly one full saved GLIM-only/Kimera-only replay, original CP-K19 exe/Qc1.0,
7s/100ms/59sample/expiry/EOF; no builds or sources. I/E and selected B bytes/order
unchanged; retained payload/action histories equal original modulo slot/event
mapping,4282ONLINE/finalized times match per control. Common ATE nativeGLIM
2.782169 -> GLIM-only2.807562m; nativeKimera1.094234 -> Kimera-only1.323540m;
BOTH2.006010m,51.56% worse than Kimera-only. Paths227.105/214.476/277.600m
vs205.913m reference. Single-source ONLINE1s rotation RPE2.301/1.941deg vs
native1.449/.4065deg; BOTH2.551deg. Thus mixed-source penalty AND receiver
newest-output instability: conflict alone is insufficient. Finalized local
motion much better, ATE still2.800047/1.321769/BOTH2.005941m. Source-endpoint
interpolated-curve checks agree; not simultaneous graph factors. Replays140.749/
45.175s, not live throughput. Initial Python3.10 hash-helper preflight failed
before any replay; original receipts preserved, compatible wrapper used once,
no numerical retry.23input/16runtime pins pass; closeout checks container restore
and Hesai checkpoint. No tuning/defaults/deletion/commit/push. Next bounded
Kimera-only newest pose/angular-velocity trace PROPOSED ONLY; no next run.

**CP-V24 LEICA-2 FUSION ACCURACY GAP — COMPLETED DIAGNOSIS (2026-10-27):**
Read experiments/leica2_fusion_gap_diagnosis_cpv24_20261027/REPORT.txt,
results.json and frontier_cohorts_complete.json. Saved-only,original Qc1.0;
no optimizer/build/replay/container or settings change. Original fusion ATE
between native sources, but ONLINE1s rotational RPE worse than both. New full
frontier audit: latest GLIM endpoint median152.779/152.784ms behind newest;
no direct newest GLIM factor. Neither source directly touches newest55.436%/
55.459% of4277post-start publications,50.546%/50.624% inside common intervals.
This is propagation through WNOA, not disconnected graph or wall-time latency.
Kimera usable-belief tail ends390.396/395.696s although sparse native poses
continue; do not call38.19/32.89s final endpoint ages normal transport delay.
Exact chain/graph prism decomposition passes2.15e-14m; removed prior-point
corrections median/p95 OFF2.276/7.462cm,ON1.788/4.163cm. Chaining drops useful
corrections; pure chain not a fix. Finalized removes path inflation but not
roughly2m ATE. Original ATE cause not uniquely isolated; covariance calibration,
representation and mixed-source effects remain hypotheses. Next PROPOSED ONLY:
two saved OFF receiver controls GLIM-only/Kimera-only vs existing BOTH, same
Qc/mask/policies; not native estimators, not executed.16combined pins unchanged,8564
publication identities pass. Analysis clock/cohort serialization corrections
documented; incomplete JSON retained, complete supplemental file authoritative.
No default promotion/deletion/commit/push or new run authorization.

**CP-V23 FULL LEICA-2 INCREMENTAL OUTPUT — COMPLETED, NOT PROMOTED (2026-10-27):**
experiments/leica2_incremental_full_cpv23_20261027/REPORT.txt, results.json,
reconstruction_checks.json and closeout.json. One full saved Velodyne CP-V4
OFF/ON receiver-only replay each,196.984/203.648s wall (not live timing claim).
Original Qc/model/source journals unchanged, observer-only private CP-K19 copy;
Live.cpp byte-identical; no extra solve. Full ONLINE/finalized CSV/TUM parity,
exact summaries,296400/296470 deterministic events and4282 publication payloads
per mode pass.2873input/1665compile pins verified after replay.4282poses/mode.
Chained same-solution increments reduce estimated path280.154058->216.953604m
OFF and235.136662->209.648540m ON versus205.929085m IE-TC, but full ATE worsens
2.000424->3.786712m and1.828658->6.080933m.1s RPE trans0.052887->0.063550m /
0.052738->0.081676m and rotation2.425509->2.751262 /1.774105->1.933959deg.
100ms trans RPE improves5.312->2.086cm /3.029->1.507cm. Original-fit100ms
vector RMS improves34.42%/31.48%, yet original-fit trajectory RMSE worsens
2.010->10.754m /1.834->8.037m. EOF graph/chained separation6.792/15.583m,
32.740/24.936deg (between outputs, NOT GT angular errors). Common-mask ATE
also worse2.006010->3.482370m /1.867962->5.331684m. DELAYED finalized separate.
No future data, added state age or trajectory filter; pure chaining discards
historical corrections. Correction transform reconstructs graph within1.57e-12;
published-to-published control telescopes, first-minute CP-V22A chain matches.
Full savedGT4282/4282,428.1s; common3850rows/384.6s. Native controls unchanged.
Rerun full OFF/ON overlay uses SAME original fit on absolute/chained, optional
GLIM initial/Kimera incremental,776491map points. Exact RRD readback passes;
native screenshot reviewed, normal viewer opened; persistent open_rerun.sh.
Minor display preflight corrected relative/absolute timestamp convention before
RRD creation; screenshot X-access failure and shutdown102 evidence preserved.
Both containers restored stopped after replay jobs exited, Hesai4cases/667files
verify. No source replay, Qc tuning, default edit, commit/push or deletion.
Decision: reject pure chaining as a replacement accuracy fix; keep separate
local-odometry/corrected-graph semantics and explicit correction. Production
publisher and covariance/velocity-frame design NOT implemented. No next run.

**CP-V22 / CP-V22A PAIRED RECEIVER DIAGNOSIS — COMPLETED, NO FIX (2026-10-27):**
Read experiments/leica2_paired_receiver_trace_cpv22a_20261027/REPORT.txt,
comparison.json, numerics.json and closeout.json. One complete saved Hesai
CP-K29 / Velodyne CP-V4 OFF replay each; original Qc/model/times unchanged.
All ONLINE/finalized CSV/TUM byte-identical; summaries,296023/296400event rows
and34565/34604traced commits match. Initial CP-V22 observer crashed because
generic GTSAM Jacobian output vector was not pre-sized; failed prefix retained
and verified exact. CP-V22A changes only observer allocation/null check in
fresh roots, not estimator mathematics or thresholds. No source rerun/tuning.
New GLIM newest-position correction p95:3.085/6.589cm(Hesai/Velodyne); same
Kimera beliefs:4.243/9.973cm. Newest velocity changes much larger with Velodyne;
older states steadier. Retirement p95 only0.084/0.130mm; support-add correction
beyond existing prediction p95 only0.0237/0.0175mm. Main immediate mechanism:
new-belief revisions of newest pose/velocity carried into next support.
42024Jacobian block/step checks and1746endpoint residual checks pass; five
frozen linearizations per case full rank, remaining newest linear translation
0.050--0.380/0.056--0.896mm. No tested interpolation/Jacobian bug or missing
centimetre linear correction; NOT all-code or nonlinear-stationarity proof.
Same586state cohort Velo step RMS5.057->1.376cm reading at age0.2s; actual
added state age, NOT ONLINE improvement/global-drift fix. Current revised
7swindow mean4.217m versus stitched5.220m and IE-TC4.216m. Original output
problem remains. USER-LED ADDENDUM: Kimera saved no_incremental_pose=false
and pinned backend confirm chained same-update relative output, unlike fusion
absolute newest graph poses. No-delay saved-snapshot reconstruction gives Velo
path41.249->33.759m,100ms vector RMS5.015->2.465cm but common-start endpoint
error1.544->3.405m; smoother is not necessarily more accurate.596states/595edges,
no changed solve/weights, first-minute only. incremental_publication_check.json
and REPORT section11 retained. Need separate continuous odometry/corrected graph
outputs, correction/velocity/covariance frame semantics and full-sequence checks
before selecting further tuning; no publisher implementation or promotion.
3022input/1665compile pins verified after replay, final host/checkpoint receipt
in closeout.json. Both containers restored stopped after owned jobs exited.
IE-TC shared CPT7, alignment approximation unchanged; no defaults/commit/push.

**CP-V21 LEICA-2 HESAI/VELODYNE OFF JITTER AUDIT — COMPLETED, NO FIX (2026-10-27):**
Saved CP-K29/CP-V4 OFF comparison; REPORT.txt/results.json/frame_time_checks.json
in experiments/leica2_sensor_jitter_audit_cpv21_20261027/. Same4,282fusion state
timestamps; first60s ONLINE paths34.743/41.249m versus32.608m IE-TC, DELAYED
finalized32.449/32.703m.100ms vector-error RMS2.594/5.015cm. Difference real.
Same actual receiver executable/model; Kimera profiles,1,937native poses and
47,605belief payloads exactly equal; received Kimera endpoint age identical
at every publication. GLIM config differences only lidar topic/frame/extrinsic.
Both extrinsics match recorded TF chains; full first-belief log->event mean/C
and endpoints identical. Fresh first2s point-time check preserves absolute
times; negative Velodyne98--100ms offsets are normalized consistently, not a
demonstrated clock bug. GLIM endpoint freshness differs~13.8ms at publication,
but Velodyne endpoint->receipt is NOT slower. No gross accumulating backlog.
First60s GLIM relative translation RMS0.699/1.855cm versus median covariance
trace3.312e-6/4.523e-6m^2: poorer evidence with limited reported uncertainty
growth, not a calibrated NEES or scalar fusion-weight claim. No confirmed
frame/settings/time-unit bug; interpolation/Jacobian/newest-state update issues
remain unexcluded. Proposed next: unchanged-model paired first60s observer
trace plus actual-factor numerical checks, NOT executed. No source/receiver
replay, build, tuning, container start, deletion, commit or push.113input pins
and original four-case667file Hesai checkpoint pass. Full-range figure reviewed.

**CP-V20 HESAI LEICA-2 RERUN OVERLAY — COMPLETED (2026-10-27):**
User requested the CP-V11 full four-trajectory presentation for saved Hesai
LEICA-2 CP-K29, explicitly leaving ARC-3 aside. Saved-data visualization only:
GLIM first-published, Kimera ROS odometry, fusion ONLINE and IE-TC reference;
separate OFF/ON tabs over the saved Hesai ON cloud. Reuse CP-V10 common-mask
no-scale rigid fits, all native points, no smoothing or estimator replay.
New artifacts: experiments/leica2_hesai_rerun_cpv20_20261027/. No source/model/
covariance edits, container launch, Qc ablation, baseline overwrite or push.
leica2_hesai_full_scene.rrd contains20996estimator points/85750reference points
and1,070,784Hesai map points from22submaps, same map for both tabs. Exact SDK
vertex/edge/cloud readback and63hashes pass; original Hesai667file checkpoint
verifies. Rigid fits reproduce CP-V10 ATE<1e-9m, native lengths unchanged
<1e-10m; max map frame discrepancy0.000046423m. Native screenshot inspected,
normal viewer opened14:35+02, empty log. Known screenshot-only shutdown panic
recorded separately. REPORT.txt/open_rerun.sh saved; no next run authorized.

**CP-V19 ARC-3 GLIM FAILURE DIAGNOSIS — COMPLETED, NO FIX (2026-10-27):**
Read experiments/arc3_glim_failure_cpv19_20261027/REPORT.txt and both JSON audits.
Saved-data/code only. GLIM really diverged during recorded lidar gaps: revised
same-window edge30.777m versus0.688m IE-TC, not merely stitched ONLINE jitter.
CBS K->G had no new usable insertion501.373--512.272s (10.899s); pending25,
explicit active external factors0 after507.173s. Matching requires existing
GLIM states within120ms and duration difference<=60ms; sparse scans defeat
short Kimera factors.52/53selected Kimera edges lack even best-case timestamp
support. Camera and IMU continue; Kimera remains locally much healthier.
Recovery includes69.411m first-publication correction jump, not physical motion.
Then LOCAL provider extraction/chart guard fails at513.5675s,epoch10047,
revision5018->5020/update5023. History-unavailable errors are secondary.
Large post-gap covariance cancellation supports numerical-sensitivity suspicion,
but exact jitter/nonfinite/disagreement branch and failed matrices were not
logged. No certified numerical root/repair or gate relaxation. MAIN7.452ms,
whole scan12.773ms at failure, profiler0aborted; not a throughput stop. Qc0.25
is downstream and cannot repair source support. No replay/build/model edit or
retry. Both containers reconfirmed stopped; CP-V18 remains FAILED PREFIX,
closeout2959pins passed. No full matched ARC-3 result or promotion.

**CP-V18 ARC-3 STANDALONE ON — RUNNING (2026-10-27):**
Launched ONCE12:22:57+02,launcher62623/controller62630;capture12:23:12.
12:24:38 startup effective/sensor gates and actual CP-V15 Qc0.25exe verified.
GLIM3935/Kimera700beliefs,109ONLINE poses,queue0/max23,no duplicates then.
2959pins checked, original Hesai4cases/667files verify. This is startup only,
not full validation or accuracy. Read fresh status; finite controller owns
remaining work. Prelaunch-only sandbox port check and one copied-audit port-
substitution error corrected/preserved BEFORE launch; raw audit exact again,
no bag/settings/threshold/build/capture retry. Details in prelaunch_review.json.
Explicit user authorization for one ON despite failed OFF, to test whether
CBS helps the struggling source. New roots/port11489; identical726.38s physical
window,0.20x,CP-V15 Qc0.25receiver and frozen CP-V17 source settings/guards.
experiments/arc3_velodyne_on_cpv18_20261027/PROTOCOL.txt and REPORT.txt.
No OFF retry, original admission edit, guard/model/covariance tuning or build.
Finite controller owns ON/four gates/saved scoring/cleanup. Full ON and shared
supported failed-OFF prefix are distinct; no full matched-pair claim. No
automatic success/rescue assumption or persistent automation. Check current
launch/status receipts; preparation is not a capture. No deletion/commit/push.

CP-V17 status correction: capture_off failed ~305.93/726.38s on receiver
indeterminate linear system near x3029; Broken pipe follows receiver exit.
Native Kimera became catastrophic (thousands-of-metres increments), whereas
GLIM stayed locally smooth. Recorded camera/lidar inputs are continuous near
failure; >1s lidar gaps occur later around501.86s. Exact numerical mechanism
unproved. Original failed evidence retained, no ON launched. Cleanup passed
23:43:33+02,2874pins unchanged,both containers stopped. Historical text follows.

**CP-V17 ARC-3 VELODYNE FULL Qc0.25 — CBS OFF RUNNING (2026-10-26):**
Launched once23:16:17+02:00,launcherPID19368/controllerPID19375. Startup
confirmed23:17:45: effective frozen sources/sensors passed, actual receiver
exe matches CP-V15 Qc0.25SHA; both source beliefs and77ONLINE publications.
Queue0/max22 then; no failure/duplicate. NOT yet full-case validation/result.
Read current H/arc3/status.json; one finite controller owns remaining pipeline.
2874protected pins and four-case667-file original Hesai checkpoint passed.
LATEST: user approved proposed as-recorded gap-aware test. New raw admission
passes726.38s offset0.2s,7028scans,7262stereo pairs. Exact known absences/partial
scans retained; no interior crop or fake samples. Last boundary adds10ms to
include recorded IMU bracket; v1 failure preserved. One isolated finite OFF/
four gates/ON only after pass/four gates/scoring/cleanup controller prepared.
Exact CP-V15 Qc*.25receiver, frozen CP-V2 sources/settings0.20x, no build.
First-published GLIM/native Kimera/ONLINE six-product gap-aware primary;
refined/finalized secondary. Read H/launch.json and H/arc3/status.json to
distinguish preparation from launch; never duplicate. H is experiment below.
No persistent automation, default promotion, source retry, deletion or push.

Historical input-review findings, now explicitly admitted as described above:
User requests newly downloaded ARC-3 full Velodyne matched OFF/ON with Qc0.25
times ORIGINAL (4x motion information), otherwise unchanged current sources/
receiver. Read experiments/arc3_velodyne_qcquarter_cpv17_20261026/REPORT.txt and
PROTOCOL.txt. Mission2024-11-18-13-48-19 required inputs present,~729s raw span.
Strict raw preflight fails: ONE missing left image at1731934279738235734ns,
200ms left/stereo gap. Six actual Velodyne gaps1.5..1.9s at~502..512s also
conflict with the existing <=1s native GLIM coverage contract; IMU continuous.
Do not silently weaken coverage/stereo criteria, synthesize frames, crop out
interior difficulties or confuse input dropout with compute delay. Read-only
diagnostics additionally find70partial-time scans(<80ms,min60.986ms), but all
7054clouds structurally valid/finite, normalized stamps monotonic. Need explicit
gap-aware full-recording admission decision; preserve all original inputs and
known absences/partial scans, distinguish unavailable input from processing loss.
No fabricated frames, interior crop or changed runtime guards. Proposed detailed
contract and exact evidence in REPORT.txt; original failed assertions preserved.
Both containers remained stopped; no source/receiver/build/bag/default changes.

**CP-V16/CP-V16A LEICA-2 TENTH-Qc TEST — COMPLETED, NOT PROMOTED (2026-10-26):**
User explicitly requested "Try0.1" after CP-V15. ONE candidate Qc0.1times
ORIGINAL, ten times original motion information; unchanged source beliefs,
covariances, timing and all other receiver settings. One full original CP-V4
saved OFF/ON receiver replay each, fresh roots. Four-way comparison reads saved
original/0.5/0.25controls; same masks/gates, separate practical tradeoff.
CP-V16 stopped BEFORE build/replay on a host harness undefined-quarter-variable
error. Original traceback/code/empty result root preserved, containers restored.
Fresh CP-V16A corrects only that comparison-pin setup bug; both historical-check
blocks execute read-only successfully. Model/audit bytes identical; no numerical
or source retry. experiments/leica2_motion_prior_tenth_cpv16a_20261026/REPORT.txt,
results.json,comparison.png,closeout.json. One-line private Qc delta;100-case
actual-noise/cost/interpolation/source-Jacobian audit PASS,10x motion cost.
Binary17f5f967d6311fb39c2b947f5b672754a5f3298b657bffacd62649fe7f56327a.
2885inputs/1665recorded dependencies unchanged;296400/296470event metadata and
255857/255928source payload commits match;4282states/428.1s each. OperationalPASS.
ONLINE100ms vector RMS2.8728/2.5505cm improves45.98%/17.69% vsoriginal and
12.60%/1.70% vs0.25. Paths227.867/223.687m vs205.929m reference: still10.65%/
8.62% excess. ATE2.013129/1.830556m worsens12.705/1.897mm vsoriginal; practical
change small, strict ATE_no_worse screen FAIL. Relative0.25,1s translation RPE
worsens both(.048469->.049211m OFF,.051235->.052432m ON). Other short-step,
moving/turning and rotation metrics improve. ON speed-lag scan+.18s vs+.16s
at0.25 and+.14s original triggers>20ms warning; coarse diagnostic, not measured
transport/control latency. Common mask confirms tradeoff; DELAYED finalized
translation worsens vs0.25 and is separately reported. Diminishing returns,
especially ON;0.25 more conservative working tradeoff, not a promoted optimum.
Replay wall184.332/187.791s includes IO, not live speed certification. Both jobs/
controller exit0; Humble restored after owned jobs, Noetic stayed stopped.
Original/default and all earlier evidence intact. No source capture, next value,
automatic promotion, numerical retry, new automation, deletion, commit or push.

**CP-V15 LEICA-2 QUARTER-Qc TEST — COMPLETED, NOT PROMOTED (2026-10-26):**
User explicitly requested0.25times ORIGINAL Qc,4x motion information after
CP-V14. One full original CP-V4 OFF/ON saved-journal replay each, fresh roots.
experiments/leica2_motion_prior_quarter_cpv15_20261026/REPORT.txt,results.json,
comparison.png,closeout.json. One-line private delta; unchanged Live/Trace,
source means/covariances/timing/receiver settings.100-case isolation audit PASS:
source residual/Jacobian differences0,actual motion-factor penalty exactly4x.
Binary9da4f4d0a8ab76498ff3a282afd3e06af5c6a337f20ce99c2b0e07cdc2aa2ed1.
2879inputs/1665recorded dependencies unchanged;296400/296470event metadata and
255857/255928source-containing payload commits match;4282states/428.1s each.
OperationalPASS. ONLINE100ms vector RMS3.2870/2.5947cm:38.20%/16.27%better
thanoriginal,18.25%/6.05%betterthan0.5. Paths235.806/225.060m vs205.929mGT,
still14.51%/9.29%excess. ATE2.007044/1.829250m:6.620/.592mmworse thanoriginal,
so strict ATE_no_worse screen FAIL. All100ms/1s and moving/turning gates pass.
Speed/angular lag scan shifts+.14/+.08s to+.16/+.10s: disclose one20msbin,
not a measured latency/flight stability claim. Common mask confirms tradeoff.
Original and0.5controls reproduced, finalized separate, default unchanged.
Replays186.161/190.374s include IO, not new rate certification. Both job/controller
exit0; only Humble started/restored, Noetic stayed stopped. No source rerun,
sweep/retry/new automation/deletion/commit/push. Further weight needs a decision.

**CP-V14 LEICA-2 MOTION-PRIOR STRENGTH TEST — COMPLETED, NOT PROMOTED (2026-10-26):**
User-approved ONE predeclared Qc*0.5 receiver-only ablation, full saved CP-V4
OFF/ON journals once each. experiments/leica2_motion_prior_cpv14_20261026/
REPORT.txt,results.json,comparison.png,closeout.json. Source covariance unchanged.
Private one-line Receiver.h delta; Live.cpp/Trace.h identical to CP-V13.
Binary666f864401f8491d7e20e8dd8414830bbadc8077d1582066ee0748bf0bff0ad3.
100-case actual discreteQ/motion cost/source interpolation/Jacobian/noise audit
passes;2875inputs and1665recorded compiler dependencies unchanged. All296400/
296470event metadata rows and255857/255928source-containing payload commits
match originals, same4282states/availability and428.1s coverage. OperationalPASS.
ONLINE vector error RMS5.3185->4.0207cm OFF,3.0988->2.7619cm ON (24.4%/10.9%
improvement). Paths280.154->250.947m,235.137->228.563m vs205.929m reference;
still21.86%/10.99% excess. 100ms/1s RPE and moving/turning cohorts improve;
speed/angular lag diagnostic minima unchanged(+.14/+.08s,not latency proof).
ATE2.000424->2.003606m OFF,1.828658->1.829293m ON worsens3.182/.634mm,
so strict no-worsening scientific screen FAIL. Common mask confirms tradeoff;
DELAYED finalized separate. Partial jitter benefit, not global drift fix or
initial/marginal-prior causality proof. Original configuration stays default.
Both replays/controller exited0; Humble stopped after owned jobs, Noetic stayed
stopped. No source rerun/sweep/retry/new automation/deletion/commit/push.
Further tests require a new decision; do not automatically try another Qc.

**CP-V13 LEICA-2 RECEIVER UPDATE TRACE — COMPLETED (2026-10-26):**
User-approved unchanged-model saved-journal OFF/ON replay with private observer
hooks; experiments/leica2_receiver_trace_cpv13_20261026/REPORT.txt,closeout.json.
Both original ONLINE/finalized CSV/TUM products byte-identical;296400/296470
event metadata rows and selected commits match. 2872inputs/1665recorded compiler
dependencies verified unchanged; original CP-K19 binary untouched. New binary
c2e67106f66807ee16535fb1c5f6444908f509a5787dc669e3faa4fe194313be.
Selected380--390s newest-state median shifts at new GLIM beliefs3.27/2.08cm,
new Kimera6.46/3.72cm OFF/ON; retirement p95~0.009cm. Same-state effect sharply
shrinks with age. Exact per-cycle vector decomposition; simultaneous-window
paths substantially shorter than connected first publications. Same90states
age0 paths7.384/3.454m become2.212/1.965m at age0.2s vs1.606m reference.
Delayed outputs are NOT a fix/current-time score; global drift persists.
Start/marginal prior weakness unproven; ongoing motion/source balance is next
testable hypothesis. Proposed ONE predeclared motion-Qc ablation only after
new approval, no sweep. No source rerun/tuning/lag/default promotion/push.
Only Humble was started for private build/replay; both containers stopped.

**CP-V12 LEICA-2 ONLINE JITTER AUDIT — COMPLETED, CAUSAL TRACE OPEN (2026-10-26):**
User-approved saved-data diagnostic: experiments/leica2_fusion_jitter_cpv12_20261026/REPORT.txt.
Same4282states/428.1s: ONLINE paths OFF280.154/ON235.137m become DELAYED
finalized204.797/205.890m vs reference205.929m. ATE remains~2.000/1.83m:
local jitter and large-scale error differ. Selected380--390s ONLINE7.929/3.673m,
finalized1.773/1.816m vs1.680m reference. Exact99/99 alternation of GLIM-only
versus GLIM+Kimera update intervals; same-state phase offsets differ7.06cmOFF,
1.53cmON.6093appliedversions/mode (replacements,not accumulated factors).
12311input pins unchanged; independent TUM/matrix path/ATE and prediction
exponential checks pass; binary/header match original protected identities.
Per-belief pre/post fused states and simultaneous windows were NOT saved;
individual source, weak-prior or marginal-conditioning causality unproven.
No source/solver replay, build, containers, weight changes or promotion. Next
proposed unchanged-model instrumented saved receiver replay is NOT executed.

**CP-V11 LEICA-2 VELODYNE RERUN VISUALIZATION — COMPLETED (2026-10-26):**
User correction, full-overlay revision COMPLETED: ONE full-size3D scene per
OFF/ON tab, all four COMPLETE aligned paths plus the SAME saved Velodyne map.
776,491points/18submaps; lidar/IMU frame check max5.72e-5m. Exact final RRD
vertex/edge/cloud readback and47input pins pass. All native trajectories and
single global rigid alignments unchanged. Original short-window view retained.
New deliverable: leica2_velodyne_rerun_cpv11_20261026/full_overlay/
leica2_full_scene.rrd. Map is GLIM reconstruction context, not independent GT.
No estimator rerun. This supersedes the earlier default close-up presentation.
User-requested saved-data CP-V4 OFF/ON first-published GLIM, Kimera ROS odometry
and fusion ONLINE against IE-TC. Native Rerun viewer opened; full 3D paths,
selected380--390s XY/XZ close-up, scrub-able10s paths and100ms displacement.
All20,997estimator samples preserved, no smoothing/resampling. CP-V10 common-
mask SE(3) fits, no scale, prism lever arm and gap policies retained. Exact
RRD timestamp/vertex/segment readback passes;15input/helper hashes unchanged.
Selected worst OFF10s excess window visibly jagged;100steps ending(380,390]s
OFF7.929/ON3.673m vs reference1.680m. NOT typical or physical drone motion;
this is connected first-publication output, not the revised window trajectory.
Report/recording/screenshot: experiments/leica2_velodyne_rerun_cpv11_20261026.
Native screenshot shutdown bug and slow CLI verifier documented; SDK readback
and native rendering pass. Viewer left open, only owned verifier stopped.
No source rerun/container/model/settings change, old-artifact overwrite or push.

**CP-V10 FIRST-PUBLISHED GLIM RESCORING — COMPLETED (2026-10-26):**
User-authorized saved-data reevaluation of all seven baseline OFF/ON pairs:
experiments/first_published_comparison_cpv10_20261026/REPORT.txt, closeout.json.
14 live glim_odometry.csv files replace revised odom_imu.txt ONLY in the new
first-publication comparison; old reports/inputs preserved. Same reference,
lever arm/no-scale SE(3), integer100ms common masks and gap-aware RPE/paths.
42 historical ATE values reproduced (max1.55e-8m); independent evo alignment
max1.22e-14m and homogeneous-matrix RPE max3.91e-14. 136 pinned inputs unchanged.
Same-time GLIM refinement ATE benefit0.030--7.090% in13/14; one worsens0.184%.
All14 estimator ATE rankings unchanged: GLIM lowest12, Kimera2, ONLINEfusion0.
GLIM1s translation RPE benefits5.13--19.58%; Velodyne EIG-2 matched path loses
10.42/9.54m with refinement despite only2.22/0.92% ATE benefit. First-publication
CBS ATE improves GLIM3/7, Kimera6/7, fusion6/7. No inference of equal latency:
Kimera publishes accumulated optimized increments, not absolute newest state.
All coverage/shared-CPT7/terminal-omission/alignment caveats retained. Two figures,
per-case JSON/masks/time series and primary/full-saved/refined tables saved.
No source rerun, solver replay, container start, build, tuning, deletion or push.

**CP-C3 OLD NON-GRANDTOUR STAGE CLEANUP — COMPLETED (2026-10-26):**
User-approved ten stage2a/stage2c/stage3a/stage16--18 roots pruned at15:26local.
39,985obsolete files/links and5,502empty directories permanently removed;
56.21GiB recovered during deletion,55.55GiB net after668MiB retained archive.
68.07GiB free.12,922retained entries verified,including unique source/safety
copies/reports/settings and original GEODE Stairs bag.3,558current protected
files unchanged,including all14liveGLIM odometry files; Hesai667file checkpoint
matches originals. Both containers remain stopped. No experiment,scoring,
tuning,build,push or GrandTour deletion. Detailed audit/limitations:
experiments/artifact_cleanup_cpc3_20261026/REPORT.txt and completion.json.
Deleted old bulk histories/builds are intentionally unavailable; notices in
the ten old roots prevent treating pruned runs as fully reproducible.

**CP-V9 GRAVITY-SEPARATED G2K ABLATION — COMPLETED, REJECTED (2026-10-26):**
User-authorized verification completed from saved data only. Original controller
failure preserved: raw matrix vs Exp(Log) check failed on93tiny nonorthogonality
round trips (max3.79e-12). Separately bounded polar equality uses original
tolerance; no runtime change or blanket tolerance increase. All1908actualG2K
factor/covariance/residual/Jacobian checks and1917unchangedK2G pass. Numerical
Jacobian max1.33e-8; projection covariance error4.24e-21. Negative controls pass.
Saved evaluator/summary/decision executed once. Full-saved ATE originalON->new:
GLIM.861349->.724307,Kimera2.584159->3.949346,ONLINE1.828658->2.613159m.
Common384.6s coverage unchanged; KimeraATE+52.32%,total20srotationRPE-24.78%,
but20sgravity-axisRPE+38.47%; ONLINEATE+37.90%. Predeclared scientific screen
FAIL; no promotion. Fixed tilt removal is not an adaptive degeneracy detector.
Candidate scan/backendp95=229.36/188.82ms; projection setupp95=.494ms.
report_repair/final.json passes13:41:35local:3027protected/8savedinput hashes
unchanged,both containers stopped,no source/replay/build/retry/tuning/push.
See REPORT.txt,RESULTS_REPORT.txt,DECISION_REPORT.txt in the experiment folder.
Single experiment closed; further investigation/run requires a new decision.
Original preparation/capture chronology follows (historical, not live status).
User approved the CP-V8 experiment. See
experiments/leica2_gravity_projection_cpv9_20261026/PROTOCOL.txt and REPORT.txt.
Private receiving-only 4D projection with dense marginal covariance; focused
chart/Jacobian/lifecycle checks before ONE full Leica-2 Velodyne ON at0.20x.
Saved CP-V4 OFF/original ON controls, no detector/sweep/default promotion.
Existing BetweenFactor uses an approximate Jacobian away from zero; the new
factor will use its exact projected derivative, default factors unchanged.
Pre-capture geometry1000cases/Jacobian7.86e-10/actual4D insertion/LOCAL native-only
marginal parity/retirement/CP-K9 rollback pass. Disabled source-state/covariance
fixture matches original library byte-for-byte. Final private libcbs1cb35215;
native/installed libraries unchanged. First standalone-factor lifecycle rejection
and compile correction preserved; final immutable BetweenFactor specialization
uses unchanged rollback guard. Launched ONCE12:21:13local(+02:00),launcher25937,
controller25944; effective startup passed,first10actual projected matrices pass.
3027protectedfiles,fresh port11487; three native/ONLINE estimators advancing.
Finite controller owns gates/scoring/decision/cleanup,no persistent automation.
Read status.json before any action; never duplicate. Accuracy not known yet.

**CP-V8 CROSS-SEQUENCE DECISION REPORT — COMPLETED (2026-10-26):**
User-requested saved-data report/proposal in
experiments/cross_sequence_decision_cpv8_20261026/REPORT.txt and PROPOSAL.txt.
Seven baseline pairs; ARC-5 OFF failure and rejected CP-V6A treated separately.
All42native/ONLINE full-saved ATE reproduced within1.56e-8m. New integer100ms
common masks,1/10/20s RPE and same-start sensitivity: Kimera20s rotation worsens
6/7, including3Hesai position-success cases.10609actualG2K factors and exact
pre-states verified,applied covariance difference0,maxprecheck1.64e-10.
IncomingGLIM tilt RMS worse6/7,heading better5/7; EIG-2 prevents a universal
causal claim. Cross-sensor exact endpoint cohorts881SPX/1637EIG/1458Leica;
matched Leica lacks330--390s,selection explicitly disclosed. Three KimeraOFF
files byte-identical across sensors,not independent repeated baselines.
Four figures,full/common/delayed/path/coverage/covariance/timing tables saved.
Proposed one gravity-separated4D G2K marginal information ablation,not a
multiplier/gate or implemented fix.1000dense Gaussian projection checks pass
(algebra ONLY,not real factor/runtime validation). Separate approval required
before any build/capture. Baseline preserved; no run/replay/container/tuning/
settings/deletion/push. closeout.json records unchanged original inputs and
two documented report-editorial corrections,not altered scientific metrics.

**CP-V7 LEICA-2 DRIFT ATTRIBUTION — COMPLETED (2026-10-25):**
User-authorized bounded saved-data investigation; REPORT.txt/PROTOCOL.txt in
experiments/leica2_drift_attribution_cpv7_20261025/. No rerun/solver replay,
container/model/tuning/build/push or one-way experiment.19input hashes unchanged.
Variance10 Kimera rotation RPE original/candidate at1s .5362/.4731deg,
20s3.3257/3.8283; same3050start-cohort confirms short/long tradeoff. At common
390.385s heading OFF/original/candidate -13.352/-21.310/-36.542deg; fixed-start
roll/pitch RMS improve5.230/6.111->2.632/3.274deg while heading RMS worsens.
Repeated directional drift, not largest isolated steps; anchor sensitivity pass.
1905original/1908candidate actual G2K and pre/post snapshots verified;1384exact
matching intervals show incoming GLIM heading healthier but tilt worse than
Kimera pre-state. Paired coverage lacks210--300s,not missing camera data.
No isolated causality/gyro bias/calibrated gating claim. Original full-saved ATE
reproduced within7.01e-9m;384.6s shared coverage,tail gaps/shared-CPT7/mean-only
alignment limitations explicit. Baseline retained,candidate NOT promoted.
Direction-aware trust is a future design question,not authorization to tune/run.

**CP-V6 LEICA-2 ROTATIONAL WEIGHT — COMPLETED (2026-10-24):**
Final14:53local: full428.75s candidate/four original gates/corrected actual-factor
validation/scoring passed; report_repair/final.json is authoritative closeout.
Both containers stopped,3002protected hashes unchanged,no source retry or EOF
omissions.1908actualG2K matrices match treatment exactly,1919K2G unchanged.
Full-saved ATE originalON->candidate(m): nativeGLIM.861349->.713107,
nativeKimera2.584159->4.430475,ONLINEfusion1.828658->2.934918. Kimera1sRPE
translation/rotation improves17.87%/11.76%, but ATE worsens71.45%; fusionATE
worsens60.50%. Common-mask scores confirm directions. No overall improvement;
do not promote variance10. Saved reports include all OFF/originalON/candidate
metrics, coverage and delayed separately. No next experiment authorized.
Chronological preparation follows.
User now authorizes the single intervention proposed after CP-V5. Protocol at
experiments/leica2_rotation_weight_cpv6_20261024/PROTOCOL.txt. One private G2K
rotation-variance10 test; translation/cross blocks unchanged, no source-export
scaling. Same full428.75s physical window/0.20x compared to saved CP-V4 OFF/ON;
no duplicate OFF, sweep, retry, default promotion or push. Build identity and
dense-matrix checks precede capture; all existing safety gates preserved.
Unmodified private CBS rebuild byte-identical to baseline82f85c2a; candidate
1b56f533,1000dense SPD/default/direction/type tests pass. CP-V6 initial harness
missed executable file modes and failed receiver --check before ANY source or
receiver capture. Both containers stopped/3001hashes unchanged. Evidence kept.
Fresh CP-V6A fixes copying modes only,3002pins; same candidate/window/settings.
Latest status/report in experiments/leica2_rotation_weight_cpv6a_20261024/.
Finite controller owns validation/report/cleanup; no persistent automation.
CP-V6A launched14:13local/controller530448. Effective startup passed; partial
actual-factor check156G2K/158K2G, treatment error0. Report-only correction:
FL4M_FROZEN_FACTOR_ROW is pre-BPSAM, so keep it raw; applied factor-matrix rows
are10x rotation as intended. Do not change pinned validator during capture.
Finite report_repair finisher waits for sole known report-gate failure and safe
cleanup, requires all four original gates, checks actual matrices then scores
saved data once. Its final.json records final outcome; original status retained.

**CP-V5 LEICA-2 SAVED BELIEF AUDIT — COMPLETED (2026-10-24):**
User-authorized three checks complete; REPORT.txt and PROTOCOL.txt at
experiments/leica2_velodyne_belief_audit_cpv5_20261024/.507118versions checked,
22160matched first/near4s records,27inputs unchanged. Actual1905G2K/1913K2G
payloads/slots verified,all1905Kimera pre-states checked; no GLIM pre-state claim.
Translation covariance ranks usefully; incomingG2K rotation rank~.047. Incoming
rotation worse than Kimera pre-state83.67%; lower-trace offline choice raises
rotation RMS26.0% while lowering translation9.4%. These are recorded-mean
diagnostics,not a new optimized trajectory. Post-update worsening is not isolated
CBS causality. Independent matrix-log/dense-whitening checks pass. Matched Kimera
200..300s gaps disclosed,not missing camera data; use coverage-explicit figure.
Shared-CPT7/mean-only alignment caveats remain. No coefficient chosen, source
rerun/replay/container/tuning/settings/build/push or follow-up experiment.

**CP-V4 FULL VELODYNE CAMPAIGN — FINISHED WITH ONE FAILED CASE (2026-10-24):**
Final03:40:32local: SPX-3/EIG-2/LEICA-2 pairs complete/all24gates/scoring pass;
ARC-5 OFF numerical failure near x3758 at~380.6/401.29s, ON not launched.
No blind retry/model edit/guard relaxation. SNOW-3 startup blocked; three other
missions lack inputs. FINAL_AUDIT.txt/RESULTS_REPORT.txt in campaign directory
record evidence/metrics/coverage/covariance/timing. All four prepared-case
closeouts pass,protected files unchanged,both containers stopped,owned jobs
exited,Hesai667files verify. Follow-up PAUSED; no further work authorized.
Numerical cause remains unresolved; failed ARC prefix is not a full result.
Chronological record follows. User
authorizes full available-sequence OFF/ON and routine issue handling overnight.
Read experiments/velodyne_full_campaign_cpv4_20261024/PROTOCOL.txt and REPORT.txt.
Sequential SPX-3/EIG-2/LEICA-2 then SNOW-3/ARC-5 after independent input/calibration/
quiet-start checks. ETH1 missing Velodyne; SPX-2/CON-1 missing other required bags.
Freeze CP-V2 source/fuser binaries/settings,0.20x,no tuning. New roots/ports,
OFF gates before ON; preserve failed evidence, no blind retry. Failed numerical
or input cases blocked; continue independent safe cases only after clean exit.
Current35.4GiB free; retain8GiB launch/5GiB runtime reserve. This supersedes
unlaunched CP-V3 preparation, not completed Hesai/Velodyne30 artifacts.
Independent input/profile checks passed SPX-3/EIG-2/LEICA-2/ARC-5; ARC-5
quiet start3.2s,401.29s usable duration. SNOW-3 no verified quiet interval,
blocked.json retained; no threshold relaxation. Campaign started00:07:39local,
controller441787/launcher441780, SPX-3 OFF00:07:55. Initial launcher GNOME
argument-syntax failure preceded all captures/containers; evidence preserved
and infrastructure-only syntax fixed. Current launch_cli_repair.json/log,
five-minute experiment-only follow-up active; no duplicate capture/retry.
00:28local: SPX-3 full197.02s OFF and all four gates passed; ON launched by
controller00:26:24, effective settings/startup pass and both sources/fusion
advance without backlog. Matched accuracy scoring not yet available.
00:45:58local SPX-3 pair completed/eight gates/scoring/closeout passed,2847
protected hashes unchanged. Full-saved ATE OFF/ON nativeGLIM.050405/.053729,
nativeKimera.483414/.376635,ONLINEfusion.128743/.112111m; DELAYEDfinalized
.130321/.110282 separate. One terminal Kimera OFF omission/zero ON, known
EOF predicate; actual spans196s+,common coverage191.799817s. No all-metric
improvement claim: ONLINE translation RPE worsens. Remaining campaign proceeds.
01:19local: EIG-2 full355.02s OFF and four gates passed; controller launched
matched ON01:18:31. Startup checks pass; matched accuracy pending. No retries.
01:50:50 EIG-2 full355.02s pair completed, eight gates/scoring/cleanup passed,
2847hashes unchanged; no terminal omissions. ATE OFF/ON nativeGLIM.283734/
.330320,nativeKimera1.882644/1.802880,ONLINEfusion.534328/.886053m; DELAYED
.526644/.883560 separate. ONLINE path299.811/255.326m vs~188.126m reference
must not be concealed by smoother delayed lengths191.050/189.436m. No tuning
or retry. LEICA-2 OFF launched01:51:04 by existing controller, now advancing.
02:29local: LEICA-2 full428.75s OFF/four gates passed02:28:31; existing
controller launched matched ON02:28:44, effective-mode/sensor startup pass.
OFF4282 ONLINE poses/253524belief versions/queued0. Matched accuracy pending;
ARC-5 queued, no duplicate validation/capture or estimator/settings changes.
03:07:15 LEICA-2 full428.75s pair completed/eight gates/scoring/cleanup pass,
2850protected files unchanged,zero terminal omissions. ATE OFF/ON nativeGLIM
2.828135/.861349,nativeKimera1.097081/2.584159,ONLINEfusion2.000424/1.828658m;
DELAYED2.000564/1.831814 separate. Common384.599633s confirms ATE directions;
native stationary-tail gaps disclosed, full source/revision/covariance evidence
retained. ARC-5 OFF launched03:07:27/PID494891, startup checks pass. SNOW-3
blocked, no guard/settings change or source retry; campaign remains running.

**CP-C2 OLD TRIAL PAYLOAD CLEANUP — COMPLETED (2026-10-23):** User explicitly
requests broader removal of obsolete failed/superseded trials. Read
experiments/artifact_cleanup_cpc2_20261023/REPORT.txt. Remove original/repeat
snapshot histories and debug traces in five preinventoried numerical-trial
roots, keeping compact outcomes, code/builds and two referenced regression
fixtures. All current Hesai/Velodyne datasets/artifacts/runtime dependencies
remain protected. Permanent scoped deletion; no new experiment or push.
Completed23:48:53local:35598 snapshot directories+637top-level debug CSVs,
153285files removed. Recovered21.32GiB additional (CP-C1+C2 total26.75GiB),
free35.41GiB.1380retained hashes/2770current protected hashes pass; Hesai
checkpoint4cases/667files matches originals. Five old roots now retain~335MiB
of reports/final paths/code/builds/two regression fixtures. Historical detailed
replay/reanalysis is intentionally decommissioned, not still fully reproducible
from those outputs. No Hesai/Velodyne/current runtime artifact removed.

**CP-C1 OLD REPEAT SNAPSHOT CLEANUP — COMPLETED (2026-10-23):** User explicitly
authorized deletion of unnecessary historical outputs. Limit to preinventoried
numeric intermediate snapshots in bounded_polish and serial_qr repeat cases;
retain final/last snapshots, original runs, reports, trajectories, code/builds,
regression cases and every modern Hesai/Velodyne artifact. No broad purge.
Read experiments/artifact_cleanup_cpc1_20261023/REPORT.txt and completion.json
when present. Save deleted-path/hash manifest and verify retained/protected
files; permanent deletion, not a backup. No experiment auto-launch in cleanup.
Completed23:40local:9847 directories/42028files removed;45147 retained-file
hashes and2770current protected paths unchanged. Final/last snapshots retained.
Recovered5.43GiB, free~14.15GiB. Storage blocker resolved for CP-V3 preparation;
no full Velodyne capture/harness started. Cleanup notes added to both old roots.

**CP-V3 SPX-3 VELODYNE FULL PAIR — STORAGE RESOLVED, NOT LAUNCHED (2026-10-23):**
User authorized full197.02s OFF/ON after CP-V2. Same physical quiet-start window,
0.20x, profiles/binaries/CBS/covariance/WNOA; no tuning. Read
experiments/velodyne_spx3_full_cpv3_20261023/REPORT.txt. Available8.85GiB but
full OFF projected~1.31GiB plus setup; ON would fail unchanged8GiB launch gate.
Only root data filesystem mounted. Recommend >=12GiB free before preparation;
need user storage action/destination authority. No data deletion, guard change,
container start, harness generation or capture. Preserve all completed work.
CP-C1 subsequently frees5.43GiB, leaving~14.15GiB. Original blocker evidence
retained; new setup still needs frozen preflight. No auto-launch during cleanup.

**CP-V2 SPX-3 VELODYNE30 INTEGRATION — COMPLETED (2026-10-23):** User approved
proceeding from CP-V1. Prepare isolated sensor-only copies and one matched30s
OFF/ON smoke at0.20x, same quiet physical start, unchanged cameras/CPT7/private
binaries/CBS/LOCAL covariance/WNOA. No full campaign yet. Historical28s pair
uses~350MiB; new pair budget1GiB plus unchanged8GiB launch/5GiB runtime reserve.
Read experiments/velodyne_spx3_smoke_cpv2_20261023/PROTOCOL.txt and REPORT.txt.
OFF launched ONCE22:06local,finite controllerPID408471. Preflight300Velodyne/
300stereopairs/3000IMU,63staticTFmessages consistent; only two sensor config
files changed, private binaries/Kimera/model unchanged.2770protected hashes.
Controller completed OFF->fourgates->ON->fourgates->score->cleanup at22:14local;
both modes passed, no terminal omissions, no failure/cleanup error. Protected
hashes2770unchanged, both containers stopped. Hesai667file checkpoint verifies
against originals. Full-saved native GLIM ATE OFF/ON0.067795/0.055356m,
Kimera0.096045/0.175519m, ONLINE fusion0.064922/0.085510m. Saved spans29.00/
29.60/29.40s; reference IE-TC shares CPT7. DELAYED finalized fusion separately
0.064899/0.071076m. Full metrics/covariance/revision/timing/coverage in
experiments/velodyne_spx3_smoke_cpv2_20261023/RESULTS_REPORT.txt.
Decision: integration valid; short-window CBS helps GLIM but hurts Kimera and
ONLINE fusion. No full-sequence or health-ranking conclusion, no0.5x claim.
Preserve all old artifacts; stop for review. No tuning, full sequence or retries.

**CP-V1 VELODYNE CAMPAIGN PLAN — COMPLETED, NOT LAUNCHED (2026-10-23):** User requests
save completed four-sequence Hesai work and plan the same OFF/ON campaign
replacing only the LiDAR with rear Velodyne. No new source/receiver run,
build/tuning/rate change or push authorized. Read-only bags/code inspection,
compact Hesai recovery checkpoint and future-run plan only. Reuse current
CP-K23/24A/28/29 semantics, not old three-GLIM experiment settings. Artifacts:
experiments/velodyne_campaign_cpv1_20261023/ and
experiments/checkpoints/20261023_hesai_four_sequences/.
Checkpoint667files/55.25MB verifies against originals; large logs/journals/bags
remain intact, no push/offsite-backup claim. Rear /anymal/velodyne/points is
present EIG-2/SPX-3/LEICA-2,~10Hz PointCloud2,16ring IDs; ETH1 bag not found.
First mission staticTF agrees with old Velodyne+CPT7 overlay<2.3e-13. Negative
relative point times already handled by native TimeKeeper; verify runtime path
and startupIMUbracketing, no arbitrary offset. Current source runner hardcodes
Hesai in bag/topic/readiness/metadata: adapt isolated copies, not only profile.
Preserve exact physical quiet-start windows,0.20x and all non-sensor settings.
Proposed one30sSPX-3 OFF/ON smoke then fullpairs sequentially; needs separate
approval and storage gate (only~9.5GiB free,5GiBreserve). No runs or code edits.

**CP-T1 FOUR-SEQUENCE TIMING AUDIT — COMPLETED (2026-10-23):** User requests
saved ETH1 CP-K23/EIG-2 CP-K24A/corrected SPX-3 CP-K28/LEICA-2 CP-K29 OFF/ON
optimization timing, dominant costs and0.5x feasibility. Read-only extraction
from existing telemetry; distinguish solver/service/exports and nested timers,
include revision event volume, bursts and queues. No rerun/build/tuning/rate
change authorized. Extracted84hashedinputs; receiver all-event cycle mean/median
match original validator for all8cases. MAINsyncGPUcalls0; no independentLOCAL
optimization contexts. GLIM post-callback47--67% of scan work; Kimera outgoing
57--68% of backend; receiver belief versions78--82% of serial service. All
ON injectioncallbacks>200ms coincide with capped40-sample competition audit;
without audit max<0.55ms. This is distinct from normal LOCAL covariance provider.
Conditional0.5x averagebusy maxima GLIM42.4%,Kimera24.8%,fusion30.4%; transient
GLIMbursts remain, hardwarecontention unmeasured.0.5x is plausible, NOT validated
or launched. No binary/profile/model/covariance edits; no rerun/replay/push.
Report/method/hashmanifest: experiments/four_sequence_timing_cpt1_20261023/REPORT.txt
and timing_v2.json; timing.json is superseded intermediate, not a result source.

**CP-K29 LEICA-2 FULL MATCHED OFF/ON — COMPLETED (2026-10-23):** User
authorized a full comparison on newly downloaded LEICA-2, mission
2024-11-25-16-36-19. Reuse CP-K28 algorithm/binaries, but independently verify
this mission's CameraInfo, TF, clocks, stationary initialization, usable end
and reference coverage. All checks passed: quiet bag6s startup before gait
around8s,428.75s through lastHesai,4287frontstereopairs. Frozen profiles match
mission calibration to numerical precision. No GT-fed initialization, tuning,
estimator edits, automatic retry, default promotion or push. New exclusive
roots/port11479,0.20x,headless; OFF gates before ON. Protocol and actual status:
`experiments/leica2_full_cpk29_20261023/`. OFF launched ONCE16:41local,
controllerPID367687; effective settings pass and all three native/ONLINE outputs
publish. Preflight2837protectedfiles,six input hashes; disk15.61GiB. Finite owned
controller runs OFF->3gates->ON->3gates->score/summary->scoped cleanup, stops on
failure and restores originally stopped containers. Do not duplicate stages.
Initial6s settling and camera/IMU-only tail excluded; IE-TC full-window reference
shares CPT7, prism covers only part. Controller completed17:58:19local; both
captures/threegates/scoring pass,2837protectedfiles and containers stopped.
No EOF omissions. Full-saved ATE OFF/ON(m): nativeGLIM.569231/.570179,
nativeKimera1.097081/.480025,fusionONLINE.696970/.565647. DELAYEDfinalized
.696285/.565905 separately. Actual427--428s outputspans,all savedposes scored;
common384.9996s supported in420.8996s envelope, sparsekeyframe gaps disclosed.
No replay/nonlinear-stationarity/0.5x/default-promotion certificate. Results:
experiments/leica2_full_cpk29_20261023/RESULTS_REPORT.txt and results.json.
Do not duplicate completed stages; checkpoint saved inCP-V1above.

**CP-K28 FULL SPX-3 STATIONARY PRE-ROLL — COMPLETED (2026-10-23):** User
explicitly requested the fresh full SPX-3 rerun after CP-K27. One matched OFF/ON
pair, bag29..226.02 (197.02s),0.20x,port11478. Extend CP-K27's verified quiet
start through the last Hesai scan; no estimator/model/covariance/profile/build
change. OFF must pass all three frozen gates before ON. Same finite-recording
EOF predicate, explicit omissions and absorbed-revision accounting. No retries,
tuning, other sequence, default promotion, commit or push. Prior CP-K26/27
artifacts remain immutable. Finite owned controller restores initially stopped
containers, saves native/ONLINE/delayed metrics and full covariance histories.
Preflight2763protectedhashes and1970stereopairs passed. OFF launched ONCE
14:56local,controllerPID346069; both captures, three gates each and scoring
passed. Closeout15:32:04local restores both containers stopped and verifies
2763hashes. Full-saved ATE OFF/ON(m): nativeGLIM.016576/.016559,
nativeKimera.483414/.110169, fusionONLINE.044145/.040815. DELAYEDfinalized
.037843/.034675 separate. Native/ONLINE spans196.199/196.798/196.400s;
all saved poses evaluated, common191.799817s supported/195.299814s envelope.
One final Kimera rev0 omitted each mode under frozen EOF predicate; no claim
of zero factor loss. Payload covariance error0, MAIN sync calls0, receiver
support p95 OFF64.64ms/ON62.13ms. No fresh replay/stationarity certificate.
Do not duplicate captures/validation/scoring. Protocol/status/report:
`experiments/spx3_stationary_full_cpk28_20261023/`.

**CP-K27 SPX-3 STARTUP PRE-ROLL — COMPLETED (2026-10-23):** One28s OFF/ON
pair,bag29..57,0.20x; quiet3s pre-roll beforeoldbag32movingcrop. No estimator,
prior,covariance,model or build change; allCP-K26 private profiles/binaries
preserved. Three old startup physical intervals at revision24: OFFtranslation
errors79.325/86.806/94.542mm ->1.998/1.409/1.684mm; ON68.782/55.795/43.185mm
->2.346/1.861/1.908mm. All99matched bag32..52edges atage4.8s translationRMS
OFF53.176->4.628mm,ON11.878->2.894mm. Strong support for correcting startup
conditions, not sole-velocity-prior causality or calibrated uncertainty:
rotationRMS~.124deg versus uncertaintyRSS~.012deg remains. ETH1/EIG-2 first
pose reference speeds0,oldSPX~.39m/s,newSPX0; do not generalize moving-startfault.
Full saved newATE OFF/ON: originalGLIM.006806/.006487m,originalKimera
.097535/.032109m,ONLINEfusion.024035/.024241m. DELAYED.010748/.012002m separate.
Same23.9s/240-queryold/new comparison: KimeraOFF.559753->.089231m,
ON.055326->.032873m; ONLINEOFF.047504->.020104m,ON.031046->.020785m.
Allgatespassed,2688protectedhashes; receiverpayloaderror0,noexpired/unsupported
or pendingversions,71supports/175slots,p95support66.45/65.39ms. Bothcontainers
restoredstopped14:41:25. No replay/stationarity/exhaustivebatchingcertificate,
fullrerun,push,retryorproductionpromotion. Different causal source/receiver
histories include earlierCBS and inclusion of formerlypreinitIDs; not a pure
single-priorablation. Fullreport/evidence:
`experiments/spx3_stationary_preroll_cpk27_20261023/REPORT.txt`.

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
- Status: COMPLETE. Archive verified and checkpoint pushes confirmed on GitHub.
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
- Remote checkpoint refs verified on Zakyera's named milestone branch:
  gnssFGO acc908dab665006fec52302572b8980073966505,
  cbsms cc26083e758661582700d1858386b2a513b9f310,
  cbs ed93343e2601c1eb2d777d85fbd6171ca1cdfaf4,
  glim_ros1 fad72d6e11219e42cbbe09658341fff4512769dc,
  gtsam_points 857819676921650510e249018cd82d45c08d7e8d.
  The cbsms exact manifest pins these archive/source commits. This subsequent
  ledger-only closeout does not change their saved launch or implementation.
- Initial upload review required explicit repository/payload approval; user
  supplied it, and the four remaining pushes then succeeded. Unchanged
  Kimera-VIO, Kimera-VIO-ROS and glim base commits were already present on
  their Zakyera checkpoint refs; no new push to those repos was needed.
- Future-chat entry points updated in gnssFGO/cbsms AGENTS and the canonical
  runbook, plus local home/workspace AGENTS. No default branch was changed;
  large data, binaries and full journals remain local. No further run authorized.

### FL4w — matched SPX-3 60 s cross-sequence live comparison

- Date/authorization: 2026-10-17; user requested a different GrandTour
  sequence for60s OFF and ON with the saved ETH1 settings.
- Status: PASSED; matched60s OFF and ON completed with source/receiver/
  deterministic-replay validation. SPX-3 selected by data readiness, not ATE.
- Scope: one matched live OFF then ON, bag32–92s, playback0.10x; reuse
  FL4v/FL4t binaries, temporary classic CBS, shared-native GLIM covariance,
  FL4q Kimera and FL4e7s fixed-lag source-WNOA. No source/model/covariance
  tuning or code rebuild. Change only sequence/window/profile/run identity.
- Preflight: verify actual SPX-3 bags/topics/GT and saved source profiles;
  do not substitute ETH1 calibration. Preserve existing acceptance guards;
  ON only after completed OFF validation. Stop on failure, no automatic retry.
- Evidence/report: `experiments/fixed_lag_spx3_fl4w_20261017/`.
- Preparation:1161 protected files;170-file FL4v archive/original check passed;
  both binaries unchanged. SPX-3's30 saved profile files verified byte-identical
  to ETH1, then copied from SPX-3 provenance. All four input bags hashed and
  required sensor/IE-TC coverage checked. Preparation initially stopped before
  creating a run due to a wrong archive-verifier path; harness path corrected,
  then preflight passed. No estimator was run during that preparation failure.
- OFF:592 GLIM/297 Kimera/595 live fusion poses; full60s playback,
  59.10/59.70/59.40s saved spans.34285 received versions,881 insertions,
  33354 replacements,50 pre-init exclusions,0 pending/unsupported; exact
  covariance payload,71 retained supports,176 peak slots,p95cycle65.3ms.
  Repeat:5 numeric products and35374 archive files byte-identical. One
  Kimera fallback,0 iteration-cap hits,all accepted steps nonincreasing.
- ON:592/297/595 poses;34484 versions,881 inserts,33553 replacements,
  50 pre-init exclusions,0 pending/unsupported;71 supports,176 slots,
  p95cycle64.7ms.289 G2K/292 K2G injections,8 fallbacks/1 cap hit;
  accepted costs nonincreasing. Repeat:5 products/35573 archive files exact.
- All-saved IE-TC ATE OFF->ON: native GLIM .010017->.010808m;
  native Kimera .530451->.065216m; fusion ONLINE .054436->.032306m;
  delayed finalized .040234->.023566m. Full59.1–59.7s saved spans after
  startup,100% reference coverage;58.9s common-mask confirms result direction.
  Report includes1s RPE,path lengths,covariance histories and unchanged settings.
- Decision: stop after this authorized comparison. No retuning/default
  promotion, no claim of1x capacity, nonlinear stationarity or universal gain.
- Report native GLIM, native Kimera ROS odometry and online fusion vs the
  full-window IE-TC reference with coverage/path length; delayed finalized
  output separately. Common-mask scores are not the full requested window.

### FL4x — matched EIG-2 60 s cross-sequence live comparison

- Date/authorization:2026-10-17; user requested another60s GrandTour
  OFF/ON comparison after FL4w. Select EIG-2 by local data/profile readiness.
- Status: COMPLETED; OFF and ON passed source/receiver/determinism checks.
  Established
  bag offset5.7s,duration60s,
  playback0.10x. Reuse FL4w harness and frozen FL4v algorithms/private builds.
- Change only mission/window/profile/run identity, with a fresh root/port11446.
  No source/receiver model edits, rebuilds, covariance/Q tuning or automatic
  retries. ON only after complete OFF/source/receiver/determinism validation.
- Verify EIG-2 sensor topics, clock, profile calibration and full-window IE-TC
  coverage. Preserve exact guards. Report original GLIM, original Kimera and
  ONLINE fusion vs reference; delayed finalized separate; full-saved plus
  common-time scores,1s RPE,path lengths and covariance histories.
- Evidence: `experiments/fixed_lag_eig2_fl4x_20261017/`.
- Preflight:1263 protected hashes; actual binaries unchanged;30 EIG-2 source
  profiles byte-identical to FL4w/ETH1, copied from EIG-2 provenance. Four bags
  hashed; timestamps1730638816.5635462–1730638876.5635462 fully covered by
  IE-TC; IMU/reference max gaps10.001ms/5.001ms. No competing source processes.
- OFF:592GLIM/298Kimera/595fusion poses,59.10/59.60/59.40s saved spans;
  34554 versions,884inserts/33620replacements/50pre-init exclusions;
  0pending/unsupported,71supports/175slots,p95cycle80.95ms. Source0fallbacks,
  no injections, all accepted costs nonincreasing. Repeat5products/35638archive
  files byte-identical;1263 protected hashes unchanged; covariance payload0error.
- ON:592GLIM/298Kimera/595fusion, same timestamps and saved spans as OFF;
  34558versions,884inserts/33624replacements/50pre-init exclusions;
  0pending/unsupported,71supports/175slots,p95cycle154.93ms. Source17fallbacks,
  14cap hits,292G2K/293K2G injections; all accepted costs nonincreasing,
  NOT stationarity-certified. Repeat5products/35642archive files byte-identical.
- Full-saved IE-TC ATE OFF->ON: native GLIM .020988->.024871m;
  native Kimera .081560->.152702m; fusion ONLINE .043372->.081958m;
  delayed finalized .030886->.057231m. Full59.1–59.6s saved spans after
  startup,100% reference coverage;58.9s common-mask confirms direction.
  Kimera/fusion translation RPE worsen; rotation RPE improves slightly.
  GLIM translation RPE essentially unchanged. Negative result preserved.
- Matched590GLIM/296Kimera edges: final-version ON/OFF median trace ratios
  rotation/translation GLIM .862984/.721659,Kimera1.269989/1.176554.
  Covariance changes are not universal across sequences; no causal or
  weighting-calibration claim. Full matrices and source/fusion paths retained.
- Report:experiments/fixed_lag_eig2_fl4x_20261017/REPORT.txt. No tuning,
  retries or new source/model edits. Both idle containers restored to their
  initial stopped state. Stop after this authorized comparison.

### FL4y — saved G2K measurement / pre-injection reference audit

- Date/authorization:2026-10-17; user approved the exact-interval GT check
  after EIG-2 FL4x degraded. Status:COMPLETED, saved-data audit only.
- Scope:read retained EIG-2 logs/reference; SPX-3 retained same-build comparison
  if needed. No estimator/receiver replay, new capture, model edits or tuning.
- Hypothesis:separate raw GLIM relative error, temporal-bridge error, and
  receiver pre-injection error on identical physical CPT7-body intervals.
  Do not substitute later/native output for a pre-injection smoother snapshot.
- Recover exact means from alignment/frozen-factor logs; cross-validate
  preceding-update revision snapshots against explicit pre-injection matrices
  and every recorded residual. Report missing/unrecoverable information honestly.
- Evidence:experiments/fixed_lag_eig2_g2k_reference_fl4y_20261017/.
- Keep all original artifacts/code/settings untouched; report reference
  provenance and coverage, not an independent-GT or causality guarantee.
- All292EIG-2/289SPX-3 injections scored on exact~.2s intervals against
  IE-TC CPT7 body reference;100% coverage,maxGTbracket5ms. Previous-update
  smoother snapshots reproduce every pre-residual and15 explicit matrices
  per sequence (<6e-13 matrix error); frozen mean/covariance exact.
- Translation RMS alignedG/ONpre/OFFpre: EIG-2 10.070/7.823/4.371mm;
  SPX-3 3.479/9.738/47.727mm. G beats OFFpre on18.84% versus85.81% of edges.
  ONpost/OFFpost RMS EIG-2 5.911/3.956mm,SPX-3 4.003/43.657mm.
- GT-oracle bridges leave EIG-2 G translation10.024mm versus actual10.070mm;
  alignment not main translational source in frozen payloads. It is imperfect,
  particularly rotationally; no closed-loop counterfactual claim. ON updates
  often improve their already-CBS-affected prior; not every injection harmful.
- Evidence:REPORT.txt,per-edge JSONs,analysis/verification scripts; no source,
  covariance or receiver changes. Input hashes unchanged. No estimator run.

### FL4z — three-sequence saved covariance / relative-error audit

- Date/authorization:2026-10-17; user requests checking GLIM/Kimera
  covariance scales OFF/ON and their relationship to relative-belief quality
  in the three latest ETH1, SPX-3 and EIG-2 runs.
- Status: COMPLETED. Retained-data analysis only. No estimator/replay,
  model edits, weighting/gating, covariance tuning or new sequence.
- Use matched physical OFF/ON endpoints, separate first-publication from
  equal-source-update later revisions. ETH1 uses only its pre-stop common
  interval; do not compare the longer ON tail to missing OFF data.
- Separate rotation/translation units and full dense covariance-normalized
  reference error. Lower/higher reference-error quartiles are retrospective
  descriptions, not an operational health detector. IE-TC is shared-CPT7,
  not independent noiseless truth; nominal ellipsoid coverage is descriptive.
- Include actual source-to-source injected payloads where retained; keep
  them distinct from downstream revised beliefs and graph-wide influence.
- Evidence:experiments/fixed_lag_covariance_fl4z_20261017/.
- Validated273028 finite/symmetric/SPD saved matrices;3345 matched source
  edges, separate initial and equal-source-update later versions;2216 actual
  aligned injections. All input hashes unchanged; FL4y errors reproduce exactly.
- First-belief translation trace ON/OFF median: GLIM ETH1/SPX3/EIG2
  .99177/1.02500/.99907; Kimera1.16531/.87454/1.43357. No universal shrinkage.
- Kimera translation variance/error rank correlations OFF/ON:
  ETH1 .774/.387,SPX3 .750/.518,EIG2 .440/.536. GLIM .055/.128,
  .421/.416,-.041/-.016. Covariance magnitude is not calibrated against
  IE-TC; reference uncertainty/correlation remains unmodeled.
- EIG2 injected G translation covariance is effectively unchanged when G is
  better versus worse than matched OFF Kimera (worse/better ratio.983).
  No simple covariance-trace injection rule justified. Actual mean-only
  temporal alignment leaves covariance unchanged; no repair made here.
- Analysis draft field-type mismatch corrected before reporting; invalid
  draft quarantined. Dense/Cholesky/SE3/synthetic coverage checks pass;
  no estimator/container/build/default changes, commit or push.

### CP1 — bounded current GLIM covariance provenance

- Date/authorization:2026-10-19; user approves the bounded covariance plan,
  explicitly asking for one step at a time and no rabbit hole.
- Status: COMPLETE with explicit retained-log limits. Saved EIG-2/SPX-3 data and actual frozen provider code
  only. No source/fuser run, rebuild, tuning or changes to estimator settings.
- Scope/stop rule: trace current LOCAL native equations, relative extraction,
  revision/chart pairing; inspect logged endpoint decomposition only if valid
  for this provider, and paired revision evolution. Missing logs are a stated
  limit, not permission to start more experiments. CP2/CP3 remain deferred.
- Checklist:experiments/covariance_provenance_cp1_20261019/PLAN.txt.
- Evidence:REPORT.txt and two evidence JSONs beside checklist;49 original
  input/code hashes checked,109615 exact GLIM source->event versions,2360
  provider->first matrices. No model changes/commit/push;HEAD668d4c7 unchanged.
- First normal GLIM publication uses shared-native LOCAL; later diagnostic
  revision calls MAIN joint marginals. This is explicitly preserved S4d policy,
  not a newly introduced regression. Do not label later covariance shrinkage
  pure native-only uncertainty refinement. No same-update dual-provider revised
  matrices exist here to quantify the isolated effect of that provenance split.
- FIRST extraction checks:rank6/zero jitter,Schur-direct relative difference
  <=8.23e-16. Logged mixed-unit cross cancellation median EIG OFF/ON .302/.301,
  SPX .444/.504; not near-total. Full endpoint matrices unavailable; no separate
  directional/factor-family causal attribution. No new logging/run authorized.
- EIG representative low/high-error first edges:trans norm1.545/1.579mm,
  errors3.779/15.382mm. Local model precision does not reliably track actual
  relative discrepancy. Reference shares CPT7; not certified calibration.
- Exact+40-update matched cohorts550EIG/537SPX:revision means improve in all
  four mode cohorts, so no first-only rollback recommended. First->+40 paired
  trace ratios EIG .640OFF/.425ON,SPX .615OFF/.478ON mix evolution/provenance.
- Stop after CP1. CP2/CP3 deferred. Before tuning, clarify intended downstream
  conditioning without automatically changing the frozen covariance policy.

### CP-D1 — decide consistent outgoing LOCAL covariance policy

- Date/authorization:2026-10-19; user explicitly decides always LOCAL covariance
  rather than re-exporting received peer information through MAIN covariance.
- Status: DESIGN RECORDED; implementation and validation pending. No production
  code/configuration changes, build, estimator run, tuning, commit or push.
- Contract:GNSSFGO_LOCAL_COVARIANCE_POLICY_20261019.txt. Corrected MAIN mean plus
  freshly queried LOCAL covariance for each version, same endpoints/update/chart;
  no silent MAIN fallback. Verify both source paths, preserve stable-ID revision
  replacement and source-to-source classic temporary CBS.
- Means/linearization/shared IMU can still carry common information. No claim
  that this fully resolves double counting or calibrates actual source errors.
- Historical CP1/FL4v/w/x evidence remains unchanged and mixed-policy; no new
  all-LOCAL results exist. Next scoped task is implementation/tests, not an
  automatic campaign or changing frozen results.

### CP-I1 — isolated LOCAL revision covariance implementation

- Date/authorization:2026-10-19; user says proceed after CP-D1 decision.
- Status: ISOLATED GLIM IMPLEMENTATION/TESTS PASSED; strict all-source LOCAL
  certification pending. No bags, captures, tuning, production promotion or push.
- Hypothesis: GLIM revisions can reuse its existing shared-native covariance
  query at the same MAIN endpoint estimates, without a new optimizer or changes
  to classic CBS payloads. Frozen default and artifacts remain unchanged.
- Audit Kimera's actual FL4q first/revision extraction and temporary-factor
  retirement/prior path; do not equate its LOCAL label with an independent prior.
- Work/evidence:experiments/local_revision_covariance_cpi1_20261019/.
- Candidate bridge SHA42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f;
  selector GLIM_REVISION_COVARIANCE_POLICY=native_local requires shared_native.
  Historical default MAIN unchanged; first-publication code unchanged. Same
  provider for later versions, no MAIN fallback, corrected means/IDs preserved.
- Release50/50, focusedASan/UBSan50/50; no extra native nonlinear calls, same-query
  equality, repeat/dense/chart/retirement/failed-query coverage. Original47tests
  unchanged. First new fixture's incorrect off-diagonal assumption corrected to
  dense SPD input; sanitizer missing-libdl link repaired. Failed logs retained.
- Kimera unchanged FL4q LOCAL first/revision path and temporary removal fixture
  passes24pairs, maxcovdifference0 vs native control, Release/sanitizer. However
  it filters MAIN's surviving graph/prior rather than maintaining separate native
  history. No all-source independence/prior certificate or new graph implied.
-18protected hashes unchanged; full provider work per revision can increase
  Gaussian-query cost; live throughput not tested. Next is explicit Kimera prior
  contract resolution, then a separately authorized bounded capture, not campaign.

### CP-I2 — bounded Kimera native-information prior gate

- Date/authorization:2026-10-19; user proceeds with sequential matched60s
  ETH1/SPX-3/EIG-2 runs, conditional on closing the prior/provenance gate.
- Status: PASSED; no dataset run launched in CP-I2.
- Single hypothesis: temporary peer factors removed before retirement cannot
  contribute explicit Gaussian information to Kimera's surviving prior. Verify
  actual FL4q marginalization with nonzero peer corrections, overlapping factors,
  and repeated retirements against an independent dense Schur calculation.
- Test-only snapshot instrumentation; no source optimizer/binary/settings edits.
  Relative Hessian/RHS tolerance1e-8; preserve original compile recipe and hashes.
- Evidence:experiments/kimera_native_prior_cpi2_20261019/. If this gate fails,
  stop the campaign; do not substitute a different policy or rerun estimators.
- Result:13retirements,16disagreeing peers,39native factors/39surviving peer
  observations; dense H/RHS errors<=4.77e-16,17repeat queries exact. Release and
  focusedASan/UBSan pass. Actual native-information prior lifecycle proved;
  separate physical history copy unnecessary in this removal mode. Not an
  independence certificate. Full details/limits in REPORT.txt.

### CP-R1 — sequential matched60s LOCAL-covariance capture gate

- Date/authorization:2026-10-19; user says proceed to proposed OFF/ON campaign.
- Status: STOPPED ON FIRST EIG-2 OFF FAILURE; no ON/SPX-3/ETH1 started.
- Preserve FL4x sensors/window/profiles and FL4v model; only GLIM revision
  covariance selector/private bridge changes (CP-I1). Kimera prior gate CP-I2.
- Require fresh roots/ports, preserved logs/hashes, no automatic retries, no
  tuning; same full-window IE-TC evaluator, online and delayed output separate.
- Existing8GiBinitial/5GiBreserve and all throughput/failure guards remain.
- Receiver event7120 rejected GLIM65->67 revision47 after commit7118 absorbed
  revision46. First-endpoint age7.070107s =4.901039s source history+2.169068s
  source delay. Receiver queue0/peak11; no receiver throughput overload shown.
- Matched64pre-failure GLIM Revision root-wall median1.024278s vs.041550s frozen;
  per-query native Gaussian factorization repeated for every edge. Do not claim
  total-estimator ratio. Stop rather than extend lag, change scale or auto-retry.
-1354protected hashes unchanged; both containers stopped. No full60s metrics,
  deterministic replay or completed provenance gate. Report/evidence in
  experiments/local_covariance_cpr1_20261019/REPORT.txt.
- Next proposal ONLY: one factorization per committed native epoch with exact
  per-edge output/lifecycle tests. No further implementation performed.

### CP-I3 — one native Gaussian query preparation per committed epoch

- Date/authorization:2026-10-19; user approved the focused runtime repair after
  CP-R1's first OFF failure. Status: ISOLATED IMPLEMENTATION/TEST/TIMING PASSED;
  no sequence retry performed in this engineering gate.
- Hypothesis: reuse rank certification and the native Gaussian Bayes tree for
  all pair queries in one committed epoch. Preserve per-pair chart transport,
  complete covariance and relative Schur extraction. No nonlinear/GPU work,
  tuning, lag/playback changes, revision drops or MAIN covariance fallback.
- Isolate new provider build; keep CP-I1 and frozen S4d binaries/artifacts.
  Transaction commit/reset/retirement must invalidate preparation; abort and
  failed stage must not contaminate the committed cache. Single-threaded query
  ownership remains unchanged. Validate against the original uncached code.
- Gate: inherited provider/revision tests, focused lifecycle/chart/failure tests,
  Release and ASan/UBSan, then bounded saved-equation/synthetic timings. Report
  Gaussian query speed only, not whole-estimator speed or live certification.
- Evidence destination:experiments/native_covariance_cache_cpi3_20261019/.
-55/55 Release and55/55 focusedASan/UBSan, each repeated. Original50 tests
  unchanged plus5cache/lifecycle cases; exact matrices versus literal uncached
  code, no extra native/GPU calls or MAIN mutation. One test-only declaration
  compile error corrected; failed log retained, no mathematical adjustment.
- SavedETH1 Gaussian journal783epochs/20query snapshots,80batches including
  warmups,3640pair comparisons: max covariance difference0; historical stored
  matrices relative3.73e-15.60measured batch median .497539->.018838s,
  p95 .991881->.091476s; median paired22.25x. One preparation per epoch.
  Added all-edge workload is not CP-R1's actual EIG revision stream; no live
  latency, whole-estimator speedup, accuracy or completed60s claim.
- New private provider SHA3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7;
  unchanged CP-I1 bridge loads it.1354protected capture hashes plus preparation
  inputs unchanged. Source repos/defaults untouched; no commit/push.
- Stop here. Next decision: fresh short live gate, with documented future
  validator log-source correction, before any matched60s retry/campaign.

### CP-R2 — cached LOCAL covariance short live gate

- Date/authorization:2026-10-19; user approved one fresh short EIG-2 live check
  before restarting matched60s comparisons. Status: PASSED30s OFF; STOPPED.
- Scope: one30s CBS-OFF co-running source/receiver capture, same EIG-2 offset5.7s,
  headless0.10x, unchanged profiles/CP-I1 bridge/FL4q/FL4e7s WNOA receiver;
  only provider DSO changes to CP-I3 release_v2. No ON or campaign in this gate.
- Preserve covariance mathematics, revisions, lag, priors, scales and source
  settings. Fresh roots/port; existing no-overwrite/failure/conservation guards.
- Correct isolated validator's Kimera payload log source to estimators.log;
  LOCAL query telemetry remains in Kimera ROS stdout. No estimator edits.
- Require complete30s coverage, actual mapped DSO hashes, all-version LOCAL
  provenance/payload conservation, bounded receiver throughput and deterministic
  saved-journal replay. Compare revision timing on matched timestamps, not ATE.
- Stop on failure; no automatic retry or changing throughput settings.
- Evidence:experiments/local_covariance_live_cpr2_20261019/.
- Fresh capture completed345.5s wall.292nativeGLIM/148nativeKimera/295online
  poses;16252versions exact.434insert+15768replace+50preinit exclusions,
  no pending/unsupported/late-absorption failure,71supports/175slots,peakqueue38.
-64matched completed updates: revision-root median1.024278->.086050s,
  p952.046901->.156997s. Whole finish-callback median1.051445->.110964s.
  Not wholeGLIM speedup; .086s remains above oldMAIN .04155s revision median.
- Maximum revision first-endpoint age5.156722s<7s; median producing-update lag
  .125652s in measurement clock at0.10x. Source nonincrease/provenance gates pass.
- Deterministic receiver replay:5numeric products+16736archive files identical;
  1418protected hashes unchanged. Two report-parser assumptions (shutdown tail,
  first-export metadata timestamp/index) corrected; no capture/validator rerun.
- Containers restored stopped. No ON/60s/other sequence/default promotion/tuning,
  commit or push. Longer matched capture remains next separately authorized step.

### CP-R3 — explicitly requested EIG-2 60s cached-LOCAL CBS-ON

- Date/authorization:2026-10-19; user explicitly requests60s CBS-ON after CP-R2.
  Status: PASSED60s ON; STOPPED. This is standalone ON, not a matched60s pair;
  only30s OFF exists with the cached all-LOCAL policy. Do not relabel older OFF.
- Scope: one fresh EIG-2 live60s ON, offset5.7s, same CP-I1/CP-I3/FL4q/FL4e
  binaries and profiles, classic temporary bidirectional source CBS,0.10x,
  7s fixed-lag WNOA/no centralIMU, full LOCAL covariance scale1, no tuning.
- Preserve no-overwrite/duplicate/no-retry and source/receiver failure guards.
  Verify actual loaded libraries and both receive directions before playback.
- Validate complete coverage, injection counters, all-version LOCAL provenance,
  payload conservation, bounded latency/retention and deterministic replay.
  If complete/valid, score native GLIM, native Kimera ROS and ONLINE fusion
  over their full saved60s-window outputs versus IE-TC; finalized separate.
- Stop on failure; no other sequence, OFF run or follow-up modification.
- Evidence:experiments/local_covariance_on60_cpr3_20261019/.
- Completed645.649s wall at0.10x:592nativeGLIM/298nativeKimera/595ONLINE poses;
  full saved spans59.100/59.599/59.400s, all GT-bracketed by shared-CPT7 IE-TC.
  Full-output ATE GLIM.025337m/Kimera.138205m/ONLINE.060148m;
  DELAYED finalized.045736m, never labeled online. Full/common metrics retained.
-292G->K/293K->G injections;27452GLIM+7100Kimera all-LOCAL versions exact.
  884insert+33618replace+50preinit exclusions; no pending/lateabsorptionfailure.
  Peakqueue33,71supports/175slots; revision endpoint maxage5.216840s<7s.
  Receiver servicep95.081658s. ON revision-root median.013072s/p95.138413s;
  not a matched before/after speedup or1x/long-sequence certification.
- Source cost-nonincreasing gate passed,9existing LM fallbacks/6iteration caps;
  no stationarity claim. Replay5numeric products+35636archive files identical.
  1484protected hashes unchanged. No source edits, tuning, OFF or further run.
  Saved standalone settings/report; no default promotion, commit or push.

### CP-R4 — matched EIG-2 60s cached-LOCAL CBS-OFF

- Date/authorization:2026-10-19; user explicitly requests matching60s OFF to
  compare against completed CP-R3 ON. Status: PASSED; matched comparison saved.
- One fresh OFF capture only, same EIG-2 offset5.7s/duration60s/0.10x, copied
  profiles, CP-I1/CP-I3/FL4q/FL4e builds, all-LOCAL/scales1,7s WNOA/noIMU.
  Disable both source receive directions; retain publishers/revisions.
- Fresh port/root, no overwrite/duplicate/retry; preserve all failure guards,
  coverage/provenance/conservation/retention gates and deterministic replay.
- Score full native GLIM/native Kimera/ONLINE fusion, delayed final separate,
  with unchanged IE-TC evaluator and common OFF/ON mask; reuse ON unchanged.
- Evidence:experiments/local_covariance_off60_cpr4_20261019/.
- No tuning, source edits, builds, new sequence, ON repeat or Git push. Stop
  on failure or after the validated matched comparison.
- OFF649.188s wall;592GLIM/298Kimera/595ONLINE poses, matching ON counts/bounds
  and common590timestamp mask. All34546versions LOCAL/exact;884insert,
  33612replace,50preinit; no pending/unsupported/latefailure,71supports/175slots.
- Source zero injections/fallbacks; receiver peakqueue39,servicep95.136303s,
  maximum GLIM revision age5.192968s<7s. Deterministic5products+35630archive
  files identical;1555protected hashes unchanged. Source repos remain clean.
- Full-saved ATE OFF/ON: nativeGLIM.020797/.025337m; nativeKimera.081560/.138205m;
  fusionONLINE.043669/.060148m; delayedfinalized.030634/.045736m. Full/common
  1sRPE/path/GTcoverage and covariance histories retained. ON reused unchanged.
- Both modes share cachedLOCAL policy and frozen settings; OFF/ON performance
  direction persists here, not a covariance-calibration or universal CBS claim.
  Containers restored stopped. No model/source/default changes or further run.

### CP-R5 — matched SPX-3 60s cached-LOCAL CBS-OFF/ON

- Date/authorization:2026-10-19; user requests SPX-3 or ETH1 after matched EIG-2.
  SPX-3 selected; status PASSED; both captures and validation complete.
- Fresh matched OFF then ON, mission2024-11-02-17-43-10, bag32--92s,0.10x.
  Reuse CP-R4/CP-R3 harness and cachedLOCAL CP-I1/CP-I3 builds; SPX-3's
  retained FL4w sensor/profile provenance. No model, source or covariance tuning.
- Preserve classic temporary CBS, FL4q Kimera, FL4e7s no-central-IMU WNOA,
  independent100ms clock, priors/Qc/scales1, headless diagnostic load.
- OFF must pass coverage/provenance/conservation/source/replay gates before ON.
  No overwrite, duplicate, automatic retry, extra sequence or default promotion.
- Evidence:experiments/local_covariance_spx3_cpr5_20261019/.
  Compare original GLIM, original Kimera ROS odometry and ONLINE fusion against
  full-window IE-TC; delayed finalized output separately. Retain covariances.
- Result: full-saved ATE OFF/ON originalGLIM.010246/.011237m,
  originalKimera.530451/.070118m,ONLINEfusion.054648/.035018m;
  delayedfinalized.040483/.027114m. Kimera/ONLINE RPE also improve; GLIM
  translation RPE worsens, rotation improves. Same full saved timestamps,
  592GLIM/297Kimera/595fusion poses per mode; common590timestamp check agrees.
- All34287/34413versions LOCAL/exact;881insertions,33356/33482replacements,
  50preinit exclusions each,0pending/unsupported;71supports/176peakslots.
  ON289G->K/292K->G; cost-nonincreasing gates pass,1/6fallbacks OFF/ON,
  0/1cap hits; not stationarity-certified. All595outputs per mode published live.
- Max GLIM revision age5.14721s<7s. Replay5products+35376/35502archivefiles
  byte-identical.1623protected files unchanged, source repos clean. No model
  edit, covariance tuning, rebuild, default promotion, further run or push.

### CP-R6 — matched ETH1 60s cached-LOCAL CBS-OFF/ON

- Date/authorization:2026-10-19; user says proceed to the other sequence after
  SPX-3. ETH1 selected; PASSED, matched captures/validation completed.
- One fresh OFF then ON, ETH1 mission2024-10-01-11-29-55, bag0--60s,0.10x.
  Same CP-I1/CP-I3 all-LOCAL builds, FL4q Kimera and FL4e7s source-WNOA;
  reuse ETH1's archived calibration/profile, not a different mission's timing.
- No tuning/model edits. Preserve temporary classic CBS, corrected MAIN means,
  LOCAL full covariance all versions, Gaussian scales1, independent100ms clock,
  existing priors/Qc, no central IMU/GNSS, headless diagnostic load.
- OFF must pass coverage/provenance/conservation/source/replay before ON.
  Stop on failure, no duplicate, overwrite, retry, further sequence or push.
- Evidence:experiments/local_covariance_eth1_cpr6_20261019/.
  Score all saved original GLIM/Kimera/ONLINE poses versus IE-TC and a separate
  common mask. Disclose native startup gaps; finalized output is delayed.
- Full-saved ATE OFF/ON originalGLIM.046001/.044886m,originalKimera
  .138290/.107645m,fusionONLINE.049713/.066941m,delayedfinalized.045665/.057096m.
  Kimera ATE improves but1sRPE worsens; fusion all3metrics worsen. Small GLIM
  ATE direction reverses on common560timestamp mask; no robust ATE gain claim.
- Both591GLIM/284Kimera/593fusion; actual saved spans~59s; known3.2sKimera
  startup gap disclosed, no missing poses invented. All saved rows GT-covered.
- All34160/34175versions LOCAL/exact;870insert,33265/33280replace,25preinit;
  0pending/unsupported,71supports/175peakslots. ON280G->K/281K->G.
  Source cost-nonincreasing gates pass;1/12fallbacks,0/3cap hits,notstationarity.
- Maximum revision age5.189189s<7s. Replay5numeric products+35242/35273archive
  files byte-identical;1753protected files unchanged, source repositories clean.
  Both captures~645.6s at0.10x; no1x/covariance calibration certification.
- Results/covariance histories saved; no tuning/default promotion/rebuild/push
  or further experiment. Containers restored to initial stopped state.

### CP-A1 — matched all-LOCAL covariance versus relative error

- Date/authorization:2026-10-19; user asks to study the new results' covariance
  and errors before deciding whether tuning is needed. COMPLETE; analysis passes.
- Read only the completed CP-R4/R3 EIG-2, CP-R5 SPX-3 and CP-R6 ETH1 captures.
  No estimator/container/replay/build, model or covariance edit, scale fitting.
- Reuse verified FL4z/FL4y parser, right-local SE3 error and IE-TC reference.
  Match physical OFF/ON endpoints; separate first/latest-common-update and
  exact+40GLIM/+20Kimera source-update cohorts (~4s), report exclusions.
- Test uncertainty/error association and normalized discrepancy, separate phi
  and rho, retain dense6x6; do not treat revisions as independent samples.
- Evidence:experiments/local_covariance_error_cpa1_20261019/.
  This gate is source-belief quality, NOT source-injection alignment consistency
  or receiver factor-family influence. Stop/report before those next gates.
- All206133versions finite/SPD;2640common physical edges,0unmatched.55unique
  hashed inputs unchanged; dense/Cholesky worst relative error1.615e-14.
  Source/reference interpolation max5ms; exact~4s source-update subsets retained.
- No universal ON covariance collapse: revised matched GLIM rho trace ratios
  ON/OFF1.002/1.014/.997 EIG-2/SPX-3/ETH1; Kimera1.193/.721/1.139.
  Kimera~4s rho RMS OFF/ON4.352/4.667,34.449/7.198,3.269/5.240mm.
  GLIM covariance-error ranking weak on EIG-2/ETH1, stronger on SPX-3.
- Large normalized reference discrepancy depends on source/block/sequence/age;
  IE-TC shared-input/reference uncertainty unknown, not true NEES or NIS.
  No numeric calibration selected; deferred actual-aligned-injection CP2 is the
  proposed next bounded gate, not executed. No optimizer/run/tuning/push.

### CP-A2 — actual aligned peer covariance, EIG-2 saved-data gate

- Date/authorization:2026-10-19; user proceeds after CP-A1 recommendation.
  COMPLETE; implements deferred CP2 on one sequence, not all pending work.
- EIG-2 CP-R3 ON only: join logged source payloads, aligned means, installed
  covariance and graph slots; score exact receiver intervals with IE-TC.
  Reuse saved logs/helpers. No capture, optimizer/replay, build, tuning or push.
- Inspect actual mean/covariance transformation; synthetic finite-difference
  check only. Missing bridge telemetry limits magnitude claims, not authority
  for a new run. Preserve frozen data/code and complete input hashes.
- Evidence:experiments/aligned_peer_covariance_cpa2_20261019/.
  Stop after bounded report; no CP3/fix/other-sequence execution.
- All292G2K/293K2Gfactor matrices equal their first source payloads exactly;
  all installed slots verified.15sampledG2Kpre-state normalized residual checks
  within1.514e-11relative; no analogous K2Gindependent pre-state samples.
- Actual means use L Z R, installed covariance unchanged: neither right-local
  J_Z=Ad(R^-1) transport nor bridge covariance terms. Confirms existing mean-only
  approximation in new all-LOCAL capture, not a CP-I1/cache regression.
- Alignment helps versus wrong-time raw control: rho RMS10.594->10.140mm G2K,
 16.030->10.956mm K2G. Cannot subtract raw-source-interval error as bridge noise.
- Synthetic full joint/Jacobian checks pass(max1.111e-10);21hashedinputs unchanged.
  Real L/R/joint uncertainty not directly logged, so magnitude/causal attribution
  unestablished. No tuning or repair; explicit contract needed before calibration.

### CP-R7 — full ETH1 untuned matched OFF/ON

- Date/authorization:2026-10-19; user requests the full ETH1 sequence with
  trajectories/covariance diagnostics before tuning. Extend CP-R6 exactly;
  leave CP-A2 mean/covariance alignment approximation unchanged.
- Initial status: BLOCKED at storage preflight, before any launch/container/replay.
  Only9.21GiB available; measured matched60s capture+validation1.51GiB.
  Full~346s projection8.4--8.7GiB cannot preserve existing5GiB runtime reserve
  and>8GiB per-launch admission. No alternate mounted persistent disk found.
- Need at least20GiB total free space or an approved output volume before
  fresh duration-specific preparation. No deletion, threshold relaxation,
  reduced diagnostics, model edits, tuning, automatic retry or push.
- Preserve CP-I1/CP-I3 all-LOCAL, FL4q, FL4e7s WNOA/noIMU, scales1,
  classic temporary CBS, native mean alignment and ETH1 sensor/profile setup.
- Evidence/protocol:experiments/local_covariance_eth1_full_cpr7_20261019/.
  Native outputs/ONLINE fusion and separate DELAYED finalized, full/shared
  mask IE-TC scores and covariance histories are planned, not completed.
- User cleared storage; recheck47.93GiB free. PREPARATION IN PROGRESS.
  No competing estimator/controller; existing containers started. Full window
  bag0--346.22s covers last Hesai record; camera/IMU-only tail excluded.
  Same pinned CP-R6 implementation; duration-specific harness only.
- Preparation passed:1753old protected hashes unchanged,1823current entries
  frozen; full bag coverage/profile/binary/harness checks passed. OFF launched
  once2026-10-19~14:56local; effective settings and loaded DSOs passed before
  playback. Live source/receiver outputs confirmed; ON not started. CP-R7
  follow-up uses validation-before-ON and stop-on-failure/no-retry guards.
- Final status: STOPPED, not a completed full comparison. OFF near109.5s:
  estimators.log:678001,Kimera VioBackend.cpp:5889,"FL4Q no decreasing
  current-chart LM step" at12:15:14UTC. Preceded by stereo tracking failure;
  causal mechanism not further investigated. Controller exited1, stopped and
  drained the prefix. No ON, retry, tuning, validation replay or ATE scoring.
  Raw prefix receiver summary:64396versions,1090publications,0pending/end
  unsupported,71supports; not a full-run acceptance. Disk~55GiB,queue0 at stop.
  Evidence retained; follow-up paused; owned jobs exited and task-started
  containers restored to stopped. No method change or Git push.

### CP-K1 — Kimera OFF update531 derivative localization

-2026-10-19 user authorizes bounded diagnosis and evidence-backed isolated repair.
 DIAGNOSIS COMPLETE; repair not implemented. Reuse FL4u: failure is after one unchanged LM iteration, with a
 6.274x directional derivative discrepancy. Do not repeat that basic test.
 One logging-only Kimera OFF prefix may identify factor-family/slot mismatch;
 no full run/GLIM/fusion/ON, covariance or acceptance workaround. Preserve
 original exception and exact531-message prefix parity. Private overlay and
 focused tests, per-factor derivative/validity controls at failed decision only.
- Evidence:experiments/kimera_failed_factor_cpk1_20261019/PROTOCOL.txt.
- One111s-bounded Kimera-only OFF capture reproduced531native messages and all
 non-timing decisions exactly. Only smart-stereo factors account for the
 .001072156909 directional derivative gap; IMU/bias/prior agree. Reconstructed
 all687 visual factors exactly. Held-landmark derivative agrees within1.94e-11;
 missing Schur landmark correction explains gap within2.17e-19. Fresh
 triangulation still leaves.000110611097 mismatch: not a cache-only fix.
 Report:experiments/kimera_failed_factor_cpk1_20261019/REPORT.txt. Requires
 consistent visual objective/landmark handling in isolated fallback; no loose
 stopping workaround or solver/covariance/default change made.1863protected/
 43candidate files unchanged; Release/focusedASanUBSan pass. Containers stopped,
 old heartbeat paused, no full/ON retry or push. Does not prove whole-run repair.

### CP-K2 — consistent conditional stereo landmark fallback

- 2026-10-19 user authorizes the proposed isolated repair. SAVED-FACTOR GATE
  FAILED; not integrated, no source replay or production change.
  First gate: reuse CP-K1's saved failing visual factors, not another bag.
  At fixed camera poses minimize the existing whitened stereo residual in
  its 3D nuisance landmark, then form the same Schur pose equations there.
  Check objective/derivative agreement and dense elimination independently.
  No covariance/noise/model/profile/production solver edit or relaxed guard.
  Invalid factors retain their captured activity for this local gate; this
  does not certify their future lifecycle. Private prototype only, followed
  by integration/transaction checks before any bounded source prefix.
  Protocol: experiments/kimera_consistent_landmark_cpk2_20261019/PROTOCOL.txt.
-171/173valid factors complete conditional solves; maximum directional mismatch
  7.74e-10 on that subset. Dominant109497's mismatch disappears.106675/108819
  fail stationarity;106675 has no right observations and escapes to enormous
  depth. Independent depth profile decreases toward infinity. Cannot require
  finite conditional minima for every captured valid point or silently change
  the10m source policy. Native Schur-vs-QR error4.73e-8 also exceeds1e-9 gate.
  Release/sanitizer synthetic fixtures pass; actual saved-data gate remains
  failed. Do not call this a solved update531/full-run repair. Next bounded
  candidate is consistent joint pose/temporary-landmark damping, not more
  conditional iterations, loose guards, covariance tuning or another bag.
  Report:experiments/kimera_consistent_landmark_cpk2_20261019/REPORT.txt.
-Final integrity receipt:2013protected files unchanged; both containers stopped;
  source replays0; production_changed:false. Focused ASan/UBSan fixtures passed
  (linked libraries uninstrumented, leaks unchecked). HEAD668d4c788983 unchanged;
  no commit/push. No further tests or model change in this task.

### CP-K3 — classic Kimera versus custom backend audit

- 2026-10-19 user authorizes the proposed bounded audit after questioning whether
  recent fixes preserve historical Kimera behavior. COMPLETE: code/history
  and retained-evidence inspection only. No estimator edit, replay, build or tuning.
- Separate ordinary incremental Kimera/BPSAM, temporary CBS retirement and the
  later FL4q acceptance/fallback. Check whether the failed OFF prefix already
  used fallback; identify the smallest justified next correction/control.
- CP-K2 joint-landmark follow-up is ON HOLD; no solver redesign is authorized
  by this audit. Preserve all failed artifacts and existing experiment settings.
- Historical/current ETH1 BackendParams.yaml byte-identical (782ee8ade78a...);
  actual backend is not: FL4q replaces17 OFF updates from44, zero CBS retirements,
  then stops531. Native-path behavior without those earlier replacements unknown.
  Historical classic/persistent, current temporary removal and FL4q acceptance
  are separate policies. ON old-anchor evidence remains valid; disabling the
  added wrapper is not yet a validated fix for the temporary-ON problem.
- Report:experiments/kimera_classic_backend_audit_cpk3_20261019/REPORT.txt.
  Proposed next bounded gate uses existing FL4Q_ENABLE=0 for a Kimera-only OFF
  control<=111s; no new solver, production guard change, replay or build in CP-K3.
  Any eventual matched OFF/ON must use the same backend policy. No auto follow-up,
  covariance tuning or push. HEAD668d4c788983 unchanged; dirty work preserved.
- Final check:2013protected hashes unchanged; git diff --check passes; both
  existing containers stopped. Documentation-only changes in this task.

### CP-K4 — ordinary incremental Kimera OFF control

- 2026-10-19 user authorizes CP-K3's proposed bounded control. PREFLIGHT BLOCKED.
  One Kimera-only ETH1 first111s OFF capture at0.10x; existing FL4Q_ENABLE=0,
  no estimator rebuild/edit, new solver, GLIM, fusion, ON, tuning or retry.
- Same CP-K1 diagnostic DSO/profile/observational objective log; verify actual
  process environment because the saved nested launch explicitly overrides
  FL4Q_ENABLE. Private launch copy only. Preserve all old artifacts and guards
  unrelated to the explicitly ablated custom acceptance/rebuild policy.
- Compare native prefix before update44, examine44/531/end, complete saved path
  against IE-TC and common-mask CP-K1. Completion is not nonlinear stationarity
  or a validated temporary-ON repair. Protocol/report under
  experiments/kimera_incremental_control_cpk4_20261019/.
- One startup attempt stopped BEFORE data playback: my harness compared
  `ros_logs/../objective.csv` with `objective.csv` as literal strings. Same path,
  not an estimator failure. Existing fixture passed; zero captured updates.
  Corrected to resolved-path identity and checked without replay. All2013
  protected hashes unchanged. Preserve failed preflight; no automatic retry.
  CP-K4 REPORT.txt records exact launched/corrected hashes and limitations.
- User subsequently explicitly reauthorizes the corrected control. Fresh root
  runs/kimera_incremental_control_cpk4b_20261019, port11456; same111s/OFF/0.10x
  scope and frozen source bytes. Preserve the prior zero-playback failure.
  RUNNING: effective preflight passed, one playback started. First44 logged
  native outputs/objective rows match CP-K1; first objective difference44.
  Actual result will be in CP-K4 RUN_REPORT.txt. No final success claim yet.
- CP-K4b now COMPLETED/VALIDATED: one111s OFF,539 finite native messages,
  passed531 and ended538. First44 messages byte-identical; divergence44 exactly
  at the first saved FL4q intervention. Zero CBS retirements/FL4q calls.
  Same-mask old-FL4q/ordinary ATE.687725/.288268m,tRPE.055604/.033113m,
  rRPE.552969/.553453deg; full-saved ordinary ATE.306224m over110.599s.
  Shared-CPT7 IE-TC, startup gap disclosed; no11s prism substitution.
  This clears the bounded OFF control only, not temporary-ON/full346s repair.
  No rebuild/tuning/promotion/next experiment. See RUN_REPORT.txt and receipts.

### CP-K5 — ordinary incremental temporary-ON saved-input diagnostic

- 2026-10-19 user authorizes the proposed111s Kimera-only follow-up to CP-K4b.
  PREPARING: same ordinary FL4Q_ENABLE=0 backend/private binaries/profile as
  OFF; temporary CBS with native mean alignment, covariance unchanged.
- Replay original GLIM-to-Kimera messages from retained FL4v120s at their
  recorded arrival times with the same ETH1 sensors. This is a conditional
  one-way saved-input test, NOT fresh bidirectional CBS or new GLIM/fusion.
  Preserve serialized means/covariances and endpoint timestamps; no revisions
  substituted for peer messages, no production/default change or build.
- One playback <=111s at0.10x; stop on failure, no retry. Compare complete
  native Kimera output to CP-K4b OFF using existing IE-TC full/common scoring.
  Completion does not establish full346s robustness or nonlinear stationarity.
  Evidence:experiments/kimera_incremental_on_cpk5_20261019/.
- Preparation passed:2066 protected hashes; saved wire1189messages with1100
  inside111s, serialized payload/record-time equality verified. One capture
  launched on11457; actual flags/DSOs pass, first retirement28 observed.
  RUNNING; no completion/performance conclusion yet.
- Final: STOPPED at update495, native bag offset102.199440s, existing temporary
  CBS active_window_rollback/staged_residual_regression. External edge493->494
  residual .149932->.538649 exceeds preserved .5 limit. Not FL4q or process crash.
  Main IMU observational error already grew to1.2083e10 at494; no healthy-ON
  or full111s claim.496native outputs,468retirements,zero FL4q.1013wire messages
  exact prefix,492alignment covariances exact. No full-window scoring/retry.
  CP-K4b clears ordinary OFF only; disabling wrapper does not fix temporary ON.
  Source/settings unchanged, owned jobs exit, containers restored. See CP-K5
  REPORT.txt/failure_evidence.json/integrity.json. No full experiment or push.

### CP-K6 — native-factor refresh / immutable prior regression gate

- 2026-10-19 user authorizes the proposed bounded mathematical/code gate after
  CP-K5. IN PROGRESS; no dataset replay, production edit or covariance tuning.
- Reuse FL4p original-chart prior helper and local GTSAM IMU/marginal APIs.
  Small synthetic prior+IMU+bias+temporary external-factor fixture: removed
  factor absent from the native Schur prior, original prior energy preserved
  off-anchor, fresh surviving-factor derivative consistent in current coordinates.
  Compare stale model and independent finite differences/dense reference.
- Do not repeat old bag/cache audits or revive FL4q/global optimizer. Preserve
  baseline/private builds. Only if this gate supports a defensible integration
  may a separate isolated patch/short-prefix gate follow; no automatic full run.
- Evidence:experiments/kimera_prior_refresh_cpk6_20261019/.
- Final: PASSED the bounded coordinate/information gate, NOT a live Kimera
  repair. Actual iSAM2 remove-before-marginalize prior vs native dense Schur
  relative error1.86e-14; preserved prior energy error0; refreshed prior+IMU/
  bias derivative vs finite differences <=2.67e-10. Stale and naive reanchor/
  retained-peer negative controls discriminate. Release repeat identical;
  focused ASan/UBSan passed (vendor DSOs uninstrumented). Initial harness-only
  Eigen include compile failure retained; no model/tolerance changes.
  No source/receiver build, dataset run, covariance tuning or production edit.
  Original-chart helper reused, not new global optimizer or FL4q revival.
  Separate incremental theta/delta/cache/prior-lifecycle integration remains;
  fixture alone does not certify convergence, smart factors or ETH1 stability.
  REPORT.txt/validation.json record evidence, protected hashes and stopped
  containers. No automatic replay, full run or push.

### CP-K7 — reconcile existing temporary-CBS fixes before further repair

- 2026-10-19 user authorizes the proposed code/history reconciliation after
  asking whether these problems were already studied months ago. IN PROGRESS:
  inspect retained June/July evidence, current source, private overlays and
  CP-K4b/CP-K5 effective settings. Documentation only; no code/model/settings
  edits, compilation, container startup, estimator replay, retry or tuning.
- Resolve which fixes remain active, which were deliberately disabled, and
  which were never integrated. Do not reinvent FL4q's existing prior-preserving
  rebuild or equate a controlled stop with a process crash. Preserve all prior
  failed captures and distinguish lifecycle correctness from numerical health.
- Evidence: experiments/kimera_fix_lineage_cpk7_20261019/REPORT.txt.
- COMPLETE: git blame traces still-active June19 guards to5cc2073b/928f9ca0.
  CP-K5 executes that rollback/retry and publishes update495; controller stops
  on recovery text, all five process exits0. Not a crash or missing lifecycle
  fix; severe preceding IMU error still precludes health/continuation claims.
- Actual CP-K4b/K5 process preflights both showFL4Q=0/remove, sameK1 DSOs.
  October's current-state/prior-preserving rebuild is already implemented but
  disabled in these controls; CP-K6 uses its byte-identical prior helper.
  July reports diagnose related stock-OFF fixed-lag behavior without selecting
  a universal production solver repair. No evidence an old general fix was lost.
- No new optimizer, code edit, build or run. Narrow future question is existing
  refresh/acceptance integration with native smart semantics, not another
  generic prior/rebuild implementation. CP-K2 remains on hold. Preserve source
  lifetime, covariance and guards; no automatic full-run retry or promotion.

### CP-K8 — pin verified settings and block an unjustified relaunch

- 2026-10-19 user requests investigation and fixed settings for the next
  experiment, avoiding repeated costly trials. IN PROGRESS: settings-only
  preparation, no run, solver patch, rebuild, covariance tuning or retry.
- Use CP-R6 matched60s actual profiles/preflights and CP-R7's full-window
  contract, not obsolete FL4v mixed-revision covariance. Inspect native
  backend/profile, process flags, nested environment overrides, source
  lifetime, LOCAL provider, receiver model and OFF/ON parity.
- Pin the verified common contract and build/profile hashes. Explicitly leave
  full-run backend admission blocked if no existing setting passes both modes;
  do not silently select persistent CBS, asymmetric OFF/ON solvers or a guard
  bypass. Evidence: experiments/next_experiment_settings_cpk8_20261019/.
- COMMON SETTINGS PINNED:30profile files match CP-R6 OFF/ON and CP-R7;
  effective source settings/argv match after mode paths and receive switch;
  repeated flagfiles preserved;9private-library paths match. Qc/priors checked.
  Found explicit nested FL4Q=1 override hazard; CP-K4b/K5 correctly used their
  private0 copies. No new accidental parameter mismatch demonstrated.
- settings.lock.json preserves all-LOCAL cached covariance,temporary classic
  CBS,source profiles and7s WNOA/no central IMU. Backend selector deliberately
  unresolved/launch_allowed=false:1fails longerOFF,0does not clear temporaryON.
  No code/model/tuning change or known-failure replay. Ordinary0is a diagnostic
  reference only, not a certified repair. Specific backend integration evidence
  is required before a fresh matched run; no asymmetric OFF/ON solver policies.
- Verification:52pinned hashes unchanged, explicit readiness check rejects
  unresolved backend with exit2, git diff --check passes. No launch/build or
  commit/push. Completed settings audit; full-experiment readiness NOT achieved.

### CP-K9 — isolated incremental original-prior refresh

- 2026-10-19 user authorizes proceeding after CP-K8. IN PROGRESS. One private
  Kimera header candidate; no dataset playback, covariance/model tuning,
  default promotion, guard bypass, full experiment or push.
- Hypothesis: before actual temporary-CBS retirement, replace native Gaussian
  marginal containers with the existing immutable OriginalChartPrior adapter,
  then release only verified prior-owned fixed keys. Ordinary iSAM2 can refresh
  native factors without FL4q global acceptance/LM/core rebuild. No-retirement
  path must remain unchanged. Unknown prior/key/cache cases fail closed.
- Gate: actual incremental removal and repeated marginalization, old-prior
  energy/information, cached-vs-dense equations, native smart-factor identity,
  disabled/OFF parity and transactional rejection; Release/repeat and focused
  ASan/UBSan. Predeclared tolerances and limits in experiment PROTOCOL.txt.
- Evidence: experiments/kimera_incremental_prior_cpk9_20261019/. CP-K6 maths
  reused, not repeated as a new result. CP-K8 launch admission stays blocked
  until integration and separately authorized real-data validation support it.
- FOCUSED GATE PASSED: private header overlay and Kimera DSO built; native
  theta/delta/cache/tree unchanged at prior conversion. Prior energy error0,
  promotion cache error9.73e-13, fresh equations/tree1.37e-16, second native
  peer-removed marginal2.25e-14. Actual12-update smart/IMU/BPSAM fixture handles
  repeated retirements; OFF states/covariances exact, transaction rollback and
  repeat exact. Release and focused ASan/UBSan passed; vendor DSOs unchanged.
- Reuses byte-identical OriginalChartPrior, keeps native factors/guards/weights.
  Private library built and ldd-r resolves all dependencies/symbols. No dataset
  replay or live convergence/throughput claim. No installed/default promotion,
  covariance tuning, new controller, full retry or push. Next proposed gate is
  one saved-input Kimera-only111s ON check through prior102.199s failure, not
  another settings sweep. CP-K8 full-run admission remains blocked.
- CLOSED this code gate:2069protected hashes and52settings pins unchanged;
  both containers stopped, jobs exited, final validation.json passed. Existing
  dirty worktree preserved; no clean-worktree/real-data-success claim.

### CP-K10 — one bounded saved-input Kimera ON validation of CP-K9

- 2026-10-19 user Proceed authorizes the proposed single111s Kimera-only ON
  diagnostic through CP-K5's102.199s failure, using CP-K9's tested private DSO.
  COMPLETED2026-10-20. Same saved GLIM wire messages/record times, sensor bags/profile,
  0.10x and guards. No new GLIM/fusion process, tuning, full run or retry.
- Prepare a new output root/port; explicitly pin CPK9_PRIOR_REFRESH=1 in the
  nested node environment,FL4Q=0,retirement=remove. Verify actual loaded DSO,
  effective argv/profile, frozen input hashes and no competing jobs first.
- Existing failure guard stays active. On failure preserve evidence, clean up
  owned jobs and stop; on completion validate full111s wire/coverage/lifecycle
  and score native Kimera against existing IE-TC/ordinary OFF only. Label this
  conditional one-way replay, not fresh bidirectional CBS or fusion.
- Evidence: experiments/kimera_incremental_prior_live_cpk10_20261019/.
  No build or production/default change; no automatic following experiment.
- One111s capture completed:539finite native outputs,511retirements/promotions,
  zero recovery/rollback/FL4q. First28objective rows byte-identical to CP-K5;
  first difference28at first retirement;492common endpoint bindings identical.
  Prior cache error max5.34e-10; old IMU-error runaway is not reproduced.
- Original post-check failed expected1100vs1099: assumed start100ms late.
  Exact selected-topic query start/end proves1099actually scheduled,allexact.
  Original checker/capture untouched; separate derived-window checker passes;
  correction recorded,not an estimator retry or relaxed numerical guard.
- All-saved native Kimera OFF/conditionalON ATE.306224/.129063m,
  1stRPE.034986/.018499m,rRPE.553292/.570773deg;110.599s span539poses,
  existing3.2sstartup gap disclosed. Shared-CPT7 IE-TC,not independent prism GT.
  NO fresh GLIM/fusion/live bidirectional result or full-ETH1 admission.
- CLOSED:2162protected hashes and28prepared entries unchanged; both containers
  stopped; all owned jobs exited. Report records the post-processing correction.

### CP-K11 — matched live120 ETH1 OFF/ON with CP-K9 Kimera

- FINAL STATUS:COMPLETED after explicitly requested postprocessing-only
  log-source correction below. All original failure evidence is preserved.
- 2026-10-20 user Proceed explicitly authorizes one matched120s live comparison
  after CP-K10. OFF capture completed normally around11:14 local time;
  all four gates passed, including byte-identical saved-journal repeat.
  ON launched once around11:26 local after preflight_on passed; capture completed.
  ON provenance/receiver gates PASS, but candidate validator FAILED on wrong-log
  retirement accounting. STOPPED: no ON journal repeat or matched scoring.
- Same private CP-K9 Kimera in BOTH modes,CPK9_PRIOR_REFRESH=1,FL4Q=0,remove.
  CP-R6 all-LOCAL GLIM provider/profile, native source IMU, classic temporary CBS,
  Gaussian1 and FL4e7s source-WNOA/100ms/no-central-IMU receiver unchanged.
- New root/session/port; preserve former controllers/outputs. Compare native
  GLIM, native Kimera and fusion ONLINE with common/full-saved IE-TC metrics;
  delayed fixed-lag finalized separate. Capture covariance/revision/timing
  histories; validate LOCAL provenance, coverage, conservation and journal repeat.
- Reuse CP-R6 harness mechanically; replace only obsolete FL4q-specific source
  acceptance checks with CP-K9's declared lifecycle/prior-parity checks. No
  nonlinear stationarity claim, model edit, build, covariance tuning or push.
- Evidence: experiments/kimera_prior_matched_live120_cpk11_20261020/.
- Preparation verified1825protected files, unchanged receiver/evaluator helpers,
  exact120s selected-record query and matching private candidate in both modes.
  OFF:1191GLIM/584Kimera/1193fusionONLINE poses;70692belief versions accounted,
  covariance difference0, no unexpected receiver loss. Known3.2sKimera startup
  gap disclosed. No matched accuracy result yet.
- OFF repeat:73216archive files and all five numeric products byte-identical;
  60.888s saved-journal check,not a source rerun.1825protected inputs unchanged.
- ON:actual Kimera node stdout contains556refreshes matching556retirements
  at every update,maxpriorcacheerror1.6458e-9; no error/recovery tokens.
  New validator erroneously searched estimators.log (zero rows). Runtime
  Kimera-token monitor had same log-route limitation; retrospective actual
  node-log scan is clear. See VALIDATION_BLOCKER.txt. No protected script edit,
  relaxed gate, rerun or accuracy claim. Postprocessing-only correction needed.
- CLOSED BLOCKED:1825protected hashes unchanged,git diff --check passes,
  no owned jobs;both containers restored exited;follow-up paused. Final
  all-gates/results checker intentionally not run; no false PASS artifact.
- 2026-10-20 user asks for the OFF/ON comparison: resume ONLY postprocessing.
  Preserve original failed checker; separate actual-node-log correction applies
  identical thresholds plus per-update retirement accounting. Recheck both
  logs;finish ON saved-journal repeat and existing scoring. No source rerun,
  model/settings change or automatic next experiment.
- Corrected source check passed for BOTH modes,original checker unchanged.
  ON saved-journal repeat59.130s:73216archive files plus5numeric products
  byte-identical. Both captures validated;existing evaluator/summarizer pass.
- Full-saved native GLIM ATE OFF/ON.028690/.034553m,Kimera.463011/.156114m,
  fusionONLINE.047754/.078585m. All~119s saved outputs scored against IE-TC,
  not a short prism subset;3.2sKimera startup gap disclosed. Common mask115.9s
  agrees on ATE direction. DELAYEDfinalizedATE.041371/.073055m separate.
  Covariance/revision/payload/timing evidence retained. No tuning/cause claim.
- Final closeout:final_checks.json after restoring containers;follow-up remains
  paused,no source retry,commit/push,default promotion or next sequence.

### CP-K12 — ETH1 60s CBS-ON at 0.30x throughput check

- 2026-10-20 user explicitly requests "try 0.3 for 60sec just to check".
- COMPLETED/PASS: one fresh ON-only capture, same CP-K11 estimator binaries,
  profiles and model, bag0--60s at0.30x, headless25sdrain. No OFF/full retry.
- Only duration/rate/session/path/port changes; source/receiver covariance,
  priors,Qc,7s WNOA lag,classic temporaryCBS and LOCAL exports unchanged.
- New harness includes the already-corrected actual Kimera node-log route
  in runtime monitoring and lifecycle checks; no numerical gate relaxation.
- Check full coverage,exact covariance/revision conservation,source lifecycle,
  saved-journal determinism,source processing/queues and receiver delay tails.
- Evidence: experiments/eth1_speed60_cpk12_20261020/.
- No rebuild,tuning,default promotion,commit/push or automatic next run.
- Capture245.656wallseconds;591GLIM/284Kimera/593ONLINE fusion poses cover60s
  with retained startup/tail gaps. All source stamps match CP-K11's prefix;
  longer reference's one extra terminal GLIM scan needs later IMU support.
- GLIM full-scan median192.705/p95366.139/max772.428ms,internalqueue max2;
  Kimera backend73.395/93.377/119.110ms;receiver support50.114/83.096/122.321ms.
  Writerqueue max42,drained0;receiver wall receipt p95.025714s. No backlog loss.
- All34159versions accounted,LOCAL dense payloads exact;256Kimera retirements/
  refreshes match,cacheerror3.14e-10,no runtime failure. Source injections280/280.
  Oldest GLIM endpoint age5.332s leaves1.668s receiver lagmargin.
- Saved-journal repeat28.296s,35521archive files plus5numeric products identical.
  1886protected files unchanged. No source rerun or accuracy tuning/scoring.
- 0.30x is viable for this bounded60s ON case,not full/1x certification or
  promotion. Any future paired run must hold rate equal. Closeout in final_checks.

### CP-K13 — matched full ETH1 OFF/ON at 0.30x

- 2026-10-20 user authorizes full sequences after CP-K12's60s speed test.
  STOPPED/FAILED OFF: one full ETH1 OFF then ON intended at matched0.30x.
- Exact CP-K11/K12 binaries/profiles/model; CP-K9 prior-refresh1,FL4q0,
  all-LOCAL GLIM covariance,temporary classicCBS,7s no-central-IMU WNOA.
- Duration derived from first selected sensor record through last Hesai:
  346.32s (old346.22 used a different epoch). Headless,25sdrain.
- Four OFF gates before ON; existing coverage/conservation/lifecycle/repeat
  and full-saved/common-mask scoring. No retry,tuning,build or guard relaxation.
- Evidence: experiments/eth1_full_rate030_cpk13_20261020/.
- Preserve original/native GLIM,original/native Kimera,ONLINE fusion;
  DELAYED finalized separately; full covariance/revision/timing evidence.
- No default promotion,commit/push,next sequence or general1x certification.
- Preparation verifies1956protected hashes,unchanged binary/profile/model
  identities. OFF launched once~13:47 local;effective startup checks passed.
  At13:51~58.7s,580ONLINE poses,both source belief counters advance,queue0,
  maxqueue36,no failure marker. ON remains gated by four OFF validations.
- User separately approved experiment-only five-minute follow-up,now ACTIVE
  for CP-K13. Disable on completion/blocker; no retry/other experiment.
- FAILURE:receiver stops at136.448862s on revision after absorption;event93340,
  GLIM1286->1288rev47/update1335. Rev46 was absorbed atcommit93336;first endpoint
  lies8.7159ms before oldest retained support1287. Age7.029499s comprises4.900111s
  source history+2.129388s producing-update-to-arrival delay (adapter.003050s).
- Last receiver queue0,max40;71supports/174slots. Not a receiver queue overflow.
  Which source processing/input stage causes the2.126s delay is not audited here.
- Partial1348GLIM/667Kimera/1358ONLINE poses,NOT full346.32s. No ON,scoring,
  deterministic replay,retry or threshold/model changes. All1956hashes unchanged.
- Owned jobs exited;restore containers stopped,heartbeat PAUSED. Failure
  evidence/closeout saved separately;all-success final_checks deliberately unrun.

### CP-K14 — matched full ETH1 OFF/ON at 0.20x

- 2026-10-20 explicit user "proceed" after proposed0.20x following CP-K13.
- STOPPED/FAILED OFF: single hypothesis is whether slower playback keeps revisions
  within the unchanged7s receiver contract over the full346.32s ETH1 window.
- Exact CP-K13/CP-K11/K12 private binaries,profiles,model,covariance and guards.
  Only rate0.30->0.20,rate-based deadline and fresh roots/session/port11462.
- Evidence: experiments/eth1_full_rate020_cpk14_20261020/PROTOCOL.txt,REPORT.txt.
- OFF once;four existing gates before ON once;no retries/tuning/builds or
  threshold relaxation. Saved-journal repeat is not another source run.
- Preserve nativeGLIM/nativeKimera/ONLINE fusion,DELAYED finalized separate,
  covariance/revision/conservation/timing and full-saved/common-mask IE-TC scores.
- Prior failure remains evidence;0.20x is not presumed sufficient. No default
  promotion,commit/push,other sequence or follow-on method.
- Preparation PASS2026protected entries;identical binaries/profiles/model and
  existing validation/evaluation math. Reviewed harness.diff,39.1GiB free.
- OFF launched once~14:42 local,source effective checks passed before playback:
  rate.20/duration346.32,prior-refresh1/FL4q0/remove,native_local revisions.
  Initial~6.8s/61ONLINE outputs,both sources advance,queue0/max19,no failure.
- Five-minute experiment-only follow-up ACTIVE for CP-K14;four OFF gates
  before ON;disable on completion/failure. No completed comparison yet.
- At15:05 local OFF passed275.6/346.32s,2749ONLINE outputs,queue0/max49;
  all owned jobs alive,no failure. Beyond CP-K13 stop,not yet full validation.
- STOP at307.228464s:revision after absorption,now KIMERA1459->1460rev24,
  sourceupdate1484,event214557. Revision23 absorbedcommit212161;firstendpoint
  age11.130127s,4.129743s earlier than oldest retained support2995.
- Lastqueue0/max49,71supports/174slots,pending0;not queue overflow. Source
  record absent and producing-update endpoint uncaptured;timing attribution
  unestablished. Do not reuse CP-K13's GLIM-specific delay explanation.
- Partial3067GLIM/1485Kimera/3066ONLINE poses;NOT full346.32s. No ON,scoring,
  full validation,replay,retry or model/guard edits.2026protected hashes unchanged.
- Owned jobs exited;containers restored stopped;follow-up PAUSED. Read
  failure_evidence.json/REPORT.txt/failure_closeout.json. No automatic next run.

### CP-K15 — saved-run latency and computational-cost diagnosis

- 2026-10-20 user requests detailed investigation before another slower run.
- COMPLETED; read-only estimator/data audit of CP-K14(0.20x) and CP-K13(0.30x),
  with completed CP-K11/K12 timing references only where useful.
- Separate physical source revision-window age from measurement-processing,
  logging/adapter delivery and receiver service latency; trace actual code.
- Inspect native timestamp gaps and source-update timers, covariance/revision
  work and bounded-state counts. Quantify measured costs, label missing timers.
- Produce report/evidence under experiments/eth1_latency_cpk15_20261020.
- No source/receiver execution, rebuild, tuning, settings or method edits,
  automatic0.10x retry, commit/push or activation of the paused follow-up.
- Recommend a minimal next step only after identifying the mechanism.
- CP-K14: exact native output join resolves previously missing timing split:
  11.099944607s source history+.030182840s freshness. Keyframe gap5.099973322s,
  backend64.0802ms, log delivery6.548ms.25-state Kimera window is not a5s bound;
 7s receiver already absorbed rev23. Slower playback cannot repair this contract.
- CP-K13: true GLIM backlog; last10 scans556.108ms mean vs333.333ms budget.
  Failing scan626.766ms,main603.260ms,post20.374ms. Adapter~8.316ms.
- MAIN sync GPU factor calls consume95.39% of CP-K14 measured MAIN time,
 82.47% of complete scan timers. LOCAL optimizer contexts0; shared native
  provider/cached covariances remain. No GPU precomputed cache hits.
- Exact archived reused ROS object lacks BUILD_GTSAM_POINTS_GPU; nm lacks
  register_hook/GPU factory references; current CMake cache CUDA=OFF corroborates.
  Startup registration text is unconditional. Private bridge/runtime maps match.
  Main GPU targets grow toward15 while49--50 active frames remain bounded.
- Receiver all-event work/support cycle median52.83/p9591.22ms,against500ms
  budget at0.20x;queue0 at stop. Not a receiver/full-history or Rerun bottleneck.
- Report recommends explicit source-finalization/retention contract FIRST,
  and separate MAIN batching build repair gate using existing machinery.
  Native hook changes inlier bookkeeping: require parity before claiming
  math-preserving acceleration. No code repair or speedup validated here.
- evidence_v2.json supersedes initial diagnostic evidence.json's last-journal
  event selection; first uncommitted event is authoritative. Original preserved.
  No experiment/guard/settings altered. Source/protected integrity receipt
  retained; no automatic next run or heartbeat reactivation.

### CP-K16 — plan isolated MAIN GPU batching repair

- 2026-10-20 user requests planning only after CP-K15; PLANNING COMPLETE.
- Plan: experiments/main_gpu_batching_cpk16_20261020/PLAN.txt.
- Reuse synchronous-compatible VGICP batching, not the ordinary global-hook
  flag alone. Preserve actual selected factors/theta, original Gaussian order,
  inlier/error-cache lifecycle, incremental reuse and marginalization.
- K16a private factor-equation/cache gate; K16b actual MAIN fixed-lag and
  CP-I1/CP-I3 LOCAL-provider parity; K16c matched timing, then propose one
  bounded source check. Report/obtain next decision between gates.
- Same existing strict equation/covariance/state tolerances; independent exact
  GPU snapshots, not ordinary clone or shared-state reference branches.
- Existing private builds/settings preserved. No implementation, build/test,
  timing/source/receiver run, container start, promotion or push this turn.
- Computational repair does not fix Kimera's physical revision-window/7s
  receiver mismatch. No lag change or full-run admission implied.
- Next proposed action K16a only; this plan does not authorize all later gates.

### CP-K16a — private MAIN batching integration and factor checks

- 2026-10-20 user "proceed"; FACTOR/DISPATCH GATE PASSED. K16a only.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16A.txt.
- Scoped actual MAIN identity, existing synchronous-compatible helper and exact
  GPU snapshot oracle; no global hook registration or default build changes.
- No K16b lifecycle acceptance, timing, dataset run, tuning or promotion.
- Report: experiments/main_gpu_batching_cpk16_20261020/REPORT_K16A.txt;
  RESULT_K16A.json records zero tested equation/error/joint/relative-covariance
  differences; exact tested inlier/cache snapshots.28 steps repeated3 processes.
- 20 active Release checks (6new+14existing), repeat6;20host UBSan and7CPU
  ASan/UBSan pass. Kernels/reused libraries not fully sanitized; timing disabled.
- Private candidate6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca
  loads with unchanged CP-I1 bridge/CP-I3 objects/CUDA DSO. Original loader
  text-only symlink check stopped before tests; resolved-path/hash check passed
  using the same executable, no numerical gate relaxation or source retry.
- 2026protected files and five source HEAD/status/diffs unchanged. Both
  containers stopped after owned builds/tests finished; heartbeat not touched.
- Next proposed gate K16b actual MAIN lifecycle/shared-native provider parity.
  Compiled integration is not yet runtime-certified or timed. No full retry;
  separate source revision/receiver retention mismatch remains unresolved.

### CP-K16b — actual MAIN lifecycle and shared-native covariance parity

- 2026-10-20 user "proceed"; CORRECTNESS GATE PASSED, K16b only.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16B.txt.
- Reuse immutable K16a candidate and CP-I3 reference; new bounded diagnostic
  executable, actual ISAM2/fixed-lag updates, external removal/replacement,
  retirement/cache/slot decisions and CP-I1/CP-I3 LOCAL publication parity.
- No timing, dataset run, tuning, default promotion, push or retention redesign.
- Report: experiments/main_gpu_batching_cpk16_20261020/REPORT_K16B.txt;
  RESULT_K16B.json and immutable k16b_checks_4/5 receipts.
- Actual outer MAIN77 updates,19 retirements,37 removed-slot reuse occurrences;
  native pose/velocity/bias/IMU/GPU plus temporary peer removal/replacement.
  All278 first/later pair samples preserve full LOCAL joint/relative covariance
  and corrected MAIN means exactly. No extra GPU work in provider queries.
- New/full/affected and partial-clique marginalization sites reached. Separate
  incremental case reuses1 selected cached GPU factor and batches4 fresh;
  cached-only case reuses3 with0 fresh calls. Original Gaussian order preserved.
- Reference, disabled, enabled and hostUBSan histories pass;31,086 records,
  1,429,185 numerical entries and1,105 exact GPU snapshots per comparison;
  all measured state/equation/covariance differences0. No tolerance relaxation.
- Attached MAIN564 derivative requests:564sync calls become564prepared hits
  and0sync calls, without reducing derivative requests. Unattached owner
  unaffected. This is activation evidence, NOT a timing result.
- Three pre-validation harness issues (retired-slot lookup, independent voxel
  reconstruction, omitted frozen intensity buffer) plus one diagnostic compile
  error preserved. Enabled candidate was not run until exact-input validation;
  no solver/library change or numerical failure in the completed gates.
- 2026protected hashes/five source fingerprints unchanged; containers stopped.
  Next proposed K16c matched timing only after authorization. No full-run
  readiness claim; the independent revision-retention mismatch remains.

### CP-K16c — identical-workload MAIN timing

- 2026-10-20 user "proceed" after K16b report; COMPLETED/PASSED.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16C.txt.
- Unchanged original CP-I3 / K16a candidate; diagnostic executable only.
- Synthetic exact-device 5/50-pose loads with 2/15 GPU links per arrival;
  actual fixed-lag/shared-native updates, warmed alternating six paired repeats.
- Separate untimed parity/counters from synchronized update-cycle wall timing.
  Inherited tolerances and filled-load median/p95 <=.8 target remain unchanged.
- No real GPU capture is available for this timing gate; no whole-GLIM speed
  claim, source/receiver/dataset run, tuning, promotion, push or retention fix.
- Report: experiments/main_gpu_batching_cpk16_20261020/REPORT_K16C.txt;
  RESULT_K16C.json and k16c_checks_1 immutable receipts. All6alternating pairs,
  480measured cycles pass; zero observed state/equation/covariance differences.
- Synchronized 3-update cycle median/p95: small6.123/7.037->1.133/1.668ms;
  filled332.019/340.159->24.867/26.228ms. Filled median ratio.07490,
  p95.07711; predeclared <=.80 gate PASS. Synthetic864-point,not ETH1 speed.
- Original synchronous warnings retained; path timing includes existing logging,
  not a pure kernel measurement.89,684filled-history derivative requests in
  both routes; enabled0sync/89,684prepared. Per-batch sizes not exposed.
- 2026protected files/five source fingerprints unchanged; both containers stopped.
  No library rebuild. Next proposal ONE matched60s GLIM-only real-source
  acceptance check requires authorization; no automatic full retry/rate change.

### CP-K16d — matched60s real-source GLIM batching acceptance

- 2026-10-20 user "Let's proceed yes but what about the other kimera problem?"
  Authorized the bounded source acceptance; CAPTURES COMPLETE,
  MATCHED PRESERVATION NOT VALIDATED.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16D.txt.
- Original CP-I3 versus unchanged K16a DSO, CP-K14 ETH1 profiles and all-LOCAL
  exports. Two source-only60s captures at matched0.20x, no Kimera/fuser.
- No library rebuild/tuning/model edit/default promotion/push. Full-sequence
  admission still blocked by separate Kimera revision-history/7s retention.
- Report: experiments/main_gpu_batching_cpk16_20261020/REPORT_K16D.txt.
- Both receive598LiDAR/5986IMU,591same-time native outputs and589first beliefs,
  all LOCAL provenance valid. Scanmedian/p95190.733/364.072->45.215/169.160ms;
  MAIN142.633/336.548->13.531/26.693ms. Candidate0sync/294972prepared requests.
- NOT equivalent histories:26782vs26834revisions,52extra candidate versions;
  state difference beginsframe1(1.83mm), positionRMS.072865m/max.138808m.
  Common covariance relativeFrobenius median1.976%,max46.290%.
  Difference is between runs, not GT accuracy. Paired comparison gate fails;
  diagnostic difference_analysis.json sets acceptance_passed=false.
- Exact initialization and matching per-frame point/IMU/active-state counts do
  not prove equal live GPU inputs. No attribution to harmless nondeterminism
  or batching bug yet. First new-factor count differenceframe39,afterpose1.
- One checker count assumption corrected for unlogged initialization/shutdown
  tail, preserved in VALIDATION_NOTE_K16D.txt; no source retry or gate relaxation.
- All2026protected hashes/five repo fingerprints unchanged; owned jobs exited,
  both containers stopped. No auto next test. Proposed bounded exact real
  first-update comparison needs authorization; full retention fix still separate.

### CP-K16e — exact first real update diagnosis

- 2026-10-20 user "Proceed" after K16d report; BOUNDED DIAGNOSIS COMPLETE.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16E.txt.
- Isolate the first MAIN x0/x1 GPU equations and prepared device inputs using
  independent exact snapshots. At most two2s GLIM-only diagnostic captures.
- Immutable reference/candidate DSOs and CP-K14 profiles; private observer only.
- No60s/full retry, Kimera/fuser, tuning, promotion, push or retention changes.
- Keep failed K16d paired preservation gate and all previous evidence intact.
- REPORT_K16E.txt/RESULT_K16E.json: two2s GLIM-only captures complete,18Hesai/
  186IMU and11native poses each. Same first x0/x1 Values, but only4744/7676
  source XYZ points identical(61.8%). Actual prepared geometry already differs.
- Four exact real GPU equation checks: independent sync/batch/actual augmented
  matrix differences0, exact branch input and post-inlier/cache state.
- Original randomgrid_sampling uses worker-indexed RNG plus guided OpenMP
  assignment. Same44881point scan and resetRNG:2threads yield2different sets in
  sixcalls;1thread yields1set. No production sampler/thread setting changed.
- First live discrepancy is upstream of GPU arithmetic, not proof of a batching
  equation bug. Do not claim this explains every later60s difference or promote
  failed K16d gate. Future exact comparisons need frozen prepared inputs.
- Observer-only builds; two compile/header corrections and one closeout ID
  expectation correction documented. No source capture repeated.
- 2026protected/7pinned hashes,five source fingerprints unchanged; ownedjobs
  exit0,bothcontainers stopped. No estimator rebuild/tuning/push/automation.
- Kimera11.10s revision-history/7s retention contract still unresolved.

### CP-K16f — identical prepared-input real MAIN history

- Current status after separate user reauthorization: BOUNDED GATE PASSED.
  Historical initial failure/correction entries below are retained.
- 2026-10-20 user "Proceed" after proposed6--8s gate; OBSERVER CORRECTED,
  REAL-HISTORY GATE NOT COMPLETED; one fresh attempt approval requested.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16F.txt.
- One8s GLIM-only capture with independent companion MAIN: same exact prepared
  GPU/CPU factors and inputs, original disabled route versus enabled unchanged
  K16a route; no independent stochastic preprocessing comparison.
- Check full state, Gaussian caches/priors, IDs/retirement/inlier/cache lifecycle,
  all retained horizon-pair LOCAL covariance and corrected MAIN means.
- Diagnostic observer only, no estimator rebuild/settings/sampling/tuning,
  timing/full-run/promotion/push or separate Kimera retention change.
- One source attempt stopped at diagnostic update1: CPU equals(...,0) rejects
  even native IMU self-equality (strict abs(deltaT difference)<tol). This is
  our observer assertion error, not demonstrated optimizer mismatch. Preserve
  k16f_source_1; no completed8s/retirement or preservation claim.
- Focused frozen-library proof: self/clone equals0 false,equals1e-15 true;
  Gaussian difference0. Corrected observer plus IMU-inclusive3-update smoke
  passes in fresh k16f_source_2; state/Gaussian/covariance differences0.
- Original numerical acceptance gates unchanged. Real source retry NOT launched;
  ask-before-retry honored. REPORT_K16F.txt/RESULT_K16F_PENDING.json retain
  failure, correction and ready path.2026protected/7pinned/five source
  fingerprints unchanged; owned jobs exited,containers stopped. No promotion.
- 2026-10-20 subsequent explicit user "Yes" authorizes ONE fresh8s attempt in
  k16f_source_2 with the corrected, already built/tested observer. COMPLETED;
  preserve failed source_1, no additional retries or unchanged smoke repetition.
- RESULT_K16F.json:143actual wrapper updates,20retirement updates(first100);
  627native inputs,482exact initialGPU snapshots.3266sync calls versus3266
  prepared consumptions;106286Gaussian,25760GPU runtime,13476covariance checks
  across4492retained pair queries. State/equation/covariance differences0 except
  rotation Log numerical floor9.813e-18rad; IDs/slots/keys/retirements/cache exact.
- 78Hesai/786IMU conserved;71native poses spanning6.999903s after startup in
  requested8s input window.69first/2128revised allLOCAL beliefs; provider/wire
  covariance difference0,minimum eigenvalue3.066e-10,symmetry error0.
- Before/after integrity PASS:2026protected,7pinned,28prepared files,five source
  fingerprints unchanged. Owned processes exited0; container restoration in
  CLOSEOUT_K16F.txt. No estimator rebuild/tuning/promotion/push/next experiment.
- This closes bounded same-prepared-input real-history correctness through
  retirement, not whole-system/full-sequence/CBS-ON certification.80.768s wall
  includes exhaustive diagnostic duplication; NOT a timing benchmark. K16d's
  failed fresh-capture equality and K16e's preprocessing diagnosis stay distinct.
- Kimera11.10s history/7s receiver revision-retention contract remains unresolved.

### CP-K16g — sixty-second identical prepared-input history

- 2026-10-20 user: "Make sure it runs similarly for 60sec". NOT COMPLETED.
- Extend K16f to ONE fresh60s GLIM-only same-input paired capture; no Kimera,
  fuser, inbound CBS, full sequence, tuning, defaults, push or automation.
- PROTOCOL_K16G.txt fixes scope/limits; root k16g_source_1. Same estimator
  binaries/profiles/0.20x/25s drain and numerical gates. Only diagnostic
  duration/update bounds, harness identity and deadline change.
- Recompile the removable observer only; compare its two-line bound diff and
  run the existing IMU-inclusive plumbing before playback. Refuse overwrite.
- Stop at first mismatch, preserve evidence and report; no automatic retry.
- Separate Kimera11.10s/7s revision-retention blocker remains untouched.
- Preflight PASS:2026protected/7pinned/31prepared files,five source fingerprints
  unchanged. Duration-only observer ffededfc5dd8322c83310d724309992cc53982c48ef5266d809213f8accd89b0;
  unchanged three-update IMU plumbing passed, exact598Hesai/5986IMU manifest.
  Capture launched ONCE; startup passed. No duplicate or retry.
- Capture stopped on45s graceful-shutdown timeout; owned controller TERM,node-15.
  Every completed paired comparison passed:434updates/140retirements,404398
  Gaussian/40245covariance checks,p/v/bias/equations/covariances0,rotation1.43e-17.
  This is NOT complete60s:211native poses,44.99965s gapped span,14.18s tail missing.
- Instrumented MAIN last20median3046ms versus500ms/scan; input callback waits
  median2243ms,workload11. Includes exhaustive diagnostic duplication; exact
  internal observer cost split unavailable. Not evidence of candidate arithmetic
  failure or ordinary throughput regression. Separate input observer conserves
  raw receipts,not complete GLIM processing. No full validator/scoring/retry.
- REPORT_K16G.txt/RESULT_K16G_FAILURE.json preserve failure. Before/after integrity
  passed;cleanup completed,container restoration in CLOSEOUT_K16G.txt.
- Next proposed fix: paced/lossless diagnostic input, same numeric/lifecycle
  checks; do not guess another rate/drain or modify estimator. Not executed.
  Independent Kimera revision-retention contract still unchanged.

### CP-K16h — paced sixty-second same-input gate

- 2026-10-20 user "Ok proceed": INCOMPLETE, stopped for diagnostic cost.
- Protocol: experiments/main_gpu_batching_cpk16_20261020/PROTOCOL_K16H.txt.
- Correct diagnostic input pacing only: existing GlimROS direct-input/idle
  interface plus read-only scan acknowledgement. Same K16g observer binary,
  K16a estimator, profiles, numerical/lifecycle checks and exact60s inputs.
- Preflight normalized scan-end IMU coverage; then one new lossless capture
  in k16h_source_1, no retries. Not timing or full-system certification.
- Preserve K16f/K16g artifacts; no estimator rebuild/tuning/promotion/push.
- Separate Kimera source-history/receiver-retention issue remains unchanged.

- Preparation completed: read-only coverage confirms598Hesai/5986IMU,
  one buffered raw scan maximum; final scan uses original native EOS drain
  (20.906ms past selected lastIMU), no extra data. Failed initial strict-order
  and all-scans-covered assumptions preserved as preflight evidence, not runs.
- Private input adapter only built; numerical observer/estimator unchanged.
  Same-stream identities and expected cross-stream paced dispatch are sealed.
  One capture authorized after integrity passes; no source retry.
- One capture launched. Pacing matches expected dispatch/scan acknowledgements;
 247native poses span24.599748s, no>150ms gaps.254scans checked, not full598;
 34.58s requested tail absent, so no60s validation or full provenance claim.
- 495paired updates/181retirements: p/v/bias/Gaussian/covariance difference0,
 rotation1.399e-17rad;61118sync=61118prepared requests;64764covariance checks.
- Agent deliberately interrupted owned controller when last20scan waits reached
 mean30.152s, projecting173more minutes beyond frozen1hwatchdog. No arithmetic
 failure, guard relaxation, retry or estimator change. Checker cost remains
 unlocalized and is not a production GLIM timing result. Next proposed work is
 bounded checker-cost localization/repair, not another automatic dataset run.
- REPORT_K16H.txt/RESULT_K16H_INCOMPLETE.json retain exact partial evidence.
 Before/after2026protected/7pinned/56prepared/five source fingerprints pass;
 all owned jobs exit, original stopped-container state restored. No push.

### CP-K17 — ordinary batched ETH1 matched60s performance

- 2026-10-21 user explicitly requests one60s CBS-OFF/ON comparison of native
  GLIM, native Kimera and fusion instead of continuing exhaustive validation.
- COMPLETED. Protocol: experiments/eth1_batched_live60_cpk17_20261021/PROTOCOL.txt.
- Frozen K16a MAIN batching in both modes; CP-K9 Kimera/CP-I1 all-LOCAL policy/
  FL4e7s no-central-IMU WNOA unchanged. ETH1 offset0,60s,0.20x,headless25sdrain.
- No companion solver/probe, rebuild, tuning, default promotion or saved-journal
  rerun. Existing startup,coverage,lifecycle,LOCAL payload checks retained.
- One OFF then ON if normal checks pass; no retry/overwrite. Full-saved and
  common-mask IE-TC ATE/1sRPE/paths,ONLINE separate from DELAYED finalized.
- K16 incomplete gates and independent Kimera retention issue remain open.
- One capture each,345.58/345.56wallseconds; all ordinary coverage,lifecycle,
  LOCAL source-to-receiver covariance/order/conservation checks pass. No repeat.
- Saved591GLIM/284Kimera/593ONLINE poses each,spans58.999/59.600/59.200s;
  known3.2s initial Kimera gap disclosed. All saved poses scored against IE-TC.
- ATE OFF/ON m: nativeGLIM.045338/.026153,nativeKimera.084842/.092451,
  ONLINEfusion.057810/.050247; same direction on55.90s common mask.
  Fusion tRPE worsens.044872->.047010m,rRPE improves3.093235->2.859829deg.
  DELAYED finalized.048819/.034744m separate; no universal accuracy claim.
- MAINmedian10.909/14.505ms,scanmedian46.598/48.205ms,MAINsync0;
  ON2scans>500ms,max629.19ms but coverage/queues complete. No1x/full claim.
- ON280factors eachdirection,256prior refreshes,maximumcacheerror2.30e-10;
  34171/34175versions conserved,25preinit exclusions each,pending/unsupported0.
- Prepared copy corrected full_history/full_saved field names BEFORE launch;
  estimator/transport semantics unchanged.2097protected hashes remain exact.
- Results/REPORT.txt under experiment directory; all owned jobs exited and both
  initially-stopped containers restored stopped. No build/tuning/push/promotion.

### CP-K18 — full ETH1 with the completed K17 MAIN batching setup

- 2026-10-21 user authorizes full sequence before any Kimera changes.
- Prepared one fresh matched OFF/ON pair at0.20x,346.32s,offset0,25sdrain.
- Protocol: experiments/eth1_batched_full_cpk18_20261021/PROTOCOL.txt.
- Exact K17 binaries,profiles,classic temporaryCBS,all-LOCAL covariance and
  FL4e7s no-central-IMU WNOA; only duration/output identities change.
- No rebuild,tuning,exhaustive observer,saved-journal repeat or retry. Existing
  ordinary startup/coverage/lifecycle/LOCAL payload checks precede ON.
- Known Kimera source-history/receiver-retention mismatch remains untouched;
  stop on failure and preserve evidence. No full-sequence success claimed yet.
- Save nativeGLIM/nativeKimera/ONLINEfusion vs IE-TC,DELAYED separate,
  full-saved/common-mask coverage,ATE/1sRPE/paths,covariance/revision/timing logs.
- Restore stopped containers after owned work exits; no promotion or push.
- OFF launched ONCE~00:32 local; effective settings/library checks pass before
  playback.2167protected hashes and original profiles preserved. ON unlaunched;
  requires successful OFF and three ordinary saved-data checks. No repeat replay.
- OFF FAILED307.231672s~00:58 local: Kimera1459->1460rev24/update1484 after
  rev23 absorption.11.099945s source history+33.391ms freshness versus7s
  receiver;preceding keyframe gap5.099973s. Same mechanism/edge as CP-K14.
- GLIM sustained batching over prefix: last20s scanmedian61.664ms,p9582.624ms,
  MAINmedian32.551ms;4,076,638prepared MAIN consumptions,zeroMAINsync,
  no aborted MAIN updates,internal queue max1. Not a full-sequence certificate.
- Failed prefix preserved; no ON,validation/scoring,source retry or model edit.
 2167protected hashes unchanged. Owned jobs exited,containers restored stopped.
 Follow-up remains PAUSED: user approved proceeding but failure was discovered
 before activation. See REPORT.txt/failure_evidence.json/failure_closeout.json.

### CP-K19 — explicit bounded receiver finalization

- 2026-10-21 user "Proceed" after the explicit bounded-finalization plan.
- PASSED focused lifecycle and saved failed-prefix gate. Isolated opt-in receiver
  policy; strict existing mode remains. No new live/full-sequence certificate.
- Single hypothesis: a known absorbed ID can be audited as expired without
  solver mutation or reinsertion; revisions of active IDs remain unchanged.
- Preserve FL4e7s WNOA, priors, covariance, independent clock, source estimators,
  MAIN batching and classic temporary CBS. No source run, tuning or promotion.
- Focused lifecycle/rollback tests then saved CP-K18 failed journal only.
  Do not call its truncated capture a full experiment. Unknown expired first
  arrivals and malformed/conflicting messages remain errors. Late corrections
  are intentionally forgone, not lossless source preservation.
- Evidence/protocol: experiments/receiver_finalization_cpk19_20261021/.
- Stop and report before any fresh live run. No automatic retry or push.
- Strict suite50818 checks and focused182 Release/182ASan+UBSan checks PASS.
  No-op full state,linearized objective and joint covariance exact; future
  filtered-event oracle exact. Known-ID validation and transactional rollback
  preserved. Unknown expired first arrivals still fail.
- One retained CP-K18 replay214546events+explicit syntheticEOF passes in150.049s.
  All186856original commit records through214526 byte-identical; original
  ONLINE/finalized prefixes exact.19known Kimera IDs expire at old failure;
  1tail active revision replaced.71supports/174slots; complete conservation.
- Forgone relative mean maxtranslation0.071454mm,maxrotation3.25241e-5rad;
  covariance relative Frobenius median4.0614%,max16.8714%. Not GT errors or a
  negligible-impact/full-tail claim: only20retained events from failure onward.
- Candidate SHA4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e.
  Original FL4e/source builds and2169protected files unchanged. No estimator,
  covariance,lag,Qc,prior,source-CBS,automation or installed/default edit.
- Next fresh matched harness must explicitly account for expired versions and
  pin opt-in policy equally; not executed. Stop before source experiment.

### CP-K20 — full ETH1 matched bounded-finalization live comparison

- 2026-10-21 user "Proceed" authorizes fresh full OFF/ON at0.20x after CP-K19.
- PREPARED, BLOCKED BEFORE LAUNCH on an existing coverage-contract mismatch.
  No new capture started. OFF must pass agreed gates before ON. No retries.
- Exact CP-K17/K18 source builds/profiles/MAIN batching and FL4e7s WNOA;
  only receiver opts into tested CP-K19 seal_at_absorption policy, equally.
- Known absorbed-ID corrections are explicitly expired, not accepted or
  reinserted. Active replacement unchanged. Unknown old first arrivals and
  malformed/conflicting payloads remain errors; all coverage guards retained.
- Protocol: experiments/eth1_finalization_full_cpk20_20261021/PROTOCOL.txt.
- Save native GLIM/native Kimera/ONLINE fusion full-saved/common-mask metrics,
  DELAYED separate; report expired IDs/versions and forgone mean/cov changes.
- No rebuild,tuning,exhaustive observer,saved-journal solver repeat,promotion,
  commit/push or other sequence. Restore containers after owned work finishes.
- Preparation passes2249protected hashes,unchanged profiles/source binaries;
  expired accounting tested read-only on saved K19 archive (19versions/IDs).
- K18 native Kimera has5.099973s gap atseq1484 (~307s), but raw front cameras
  continuous75frames each over7.5s. Existing post-startup1s gap gate and
  contiguous common-mask assertion would reject. Not a K19 receiver problem.
  No gate relaxed or pose invented. Asked user whether to run/report gaps
  explicitly with gap-aware scoring or keep strict coverage and stop.

### CP-K21 — standalone Kimera full ETH1 at0.50x

- 2026-10-21 user explicitly requests only Kimera,full sequence,0.5x.
- COMPLETED ONCE; effective startup checks pass before playback. Supersedes
  executing the pending CP-K20 pair for this turn.
- One capture with exact current CP-K9 Kimera/private profile/CPT7/native IMU
  path and OFF settings. No GLIM,gnssFGO,GNSS,external CBS inputs or rebuild.
- Preserve outgoing diagnostic work for comparison; record lightweight sensor
  topic header receipts plus native odometry/frontend stats and timing logs.
- Question: does the native5.1s gap near307s recur without other estimators?
  This changes load AND rate, so not a unique-cause attribution by itself.
- Full playback completion and actual pose coverage reported separately;
  gaps are the diagnostic outcome, not interpolated away or hidden by retry.
- Protocol: experiments/kimera_only_full_rate050_cpk21_20261021/PROTOCOL.txt.
- No tuning,production edits,automatic retries,default promotion or push.

CP-K21 closeout2026-10-21: full346.32s playback at0.50x completes cleanly;
1492native poses,28prepared hashes unchanged. Original5.099973322s gap repeats
at EXACT timestamps/seq1483->1484 while50images per camera and510IMUs arrive.
Frontend frame counters continue; keyframe/backend counts stay fixed; bounding
backend updates67.69/65.31ms;LOW_DISPARITY/no-motion logs and~1mm IE-TC movement.
Configured5s keyframe maximum,slightly-under0.1s frame period and keyframe-only
backend explain~5.1s sparse outputs. Eight intervals occur in stationary tail.
This is not missing-camera data or a5s solver stall; receiver retention and
coverage assumptions must account for native keyframe selection. CP-K20 stays
unlaunched; no validation gate silently changed or new run authorized.
Whole backend median74.20ms,p9593.19ms,max141.76ms including outgoing work.
All1492poses scored over342.598s saved span:ATE4.496229m,tRPE.440356m,
rRPE.567768deg,path196.007m;shared-CPT7 IE-TC. Sparse gaps/startup/tail disclosed,
no dense346s coverage claim. Standalone success != full-system0.5x or accuracy.
Raw capture preserved; separate evaluator strips only2EOF rostopic warning
lines,not poses. No source rerun. Both containers stopped after owned jobs.
Evidence: experiments/kimera_only_full_rate050_cpk21_20261021/REPORT.txt;
run root analysis.json,capture/effective.json,capture/capture_complete.json.

### CP-K20 continuation decision — expired revisions, 2026-10-21

- User explicitly selects ignoring expired GLIM/Kimera revisions while keeping
  the existing source/receiver windows and all other model settings.
- Verified existing CP-K19 expireSealed implementation and prepared CP-K20
  wrapper already implement this for both sources/modes. No new solver patch,
  build, source run or test needed. Active revisions still replace; absorbed
  factors retain last accepted information in the prior; late revisions are
  logged/expired without solver mutation. Unrelated invalid-input guards stay.
- Saved decision: GNSSFGO_EXPIRED_REVISION_POLICY_20261021.txt, linked at the
  repository entry point. Candidate remains opt-in, not installed-default
  promotion or a claim of completed full-system validation.
- CP-K21 established normal sparse keyframe behavior. CP-K20's distinct
  coverage/evaluation preparation remains to be updated explicitly for those
  semantics, not silently skipped. No capture launched or protected harness
  overwritten during this decision-recording step.

### CP-K22 — native-keyframe-aware full comparison, 2026-10-21

- User Proceed after selected expired-revision policy. PREPARING fresh full
  OFF/ON at0.20x; no change to source estimators or receiver mathematics.
- Preserve unlaunched CP-K20. Only update harness assumptions: native Kimera
  keyframe gaps bounded by configured5s+image period, actual frontend/backend
  progress, explicit gap-aware scoring. No invented poses or dense coverage.
- Existing K19 seal_at_absorption remains pinned equally OFF/ON; active
  replacement and all unrelated input/solver guards unchanged.
- Protocol: experiments/eth1_keyframe_finalization_full_cpk22_20261021/PROTOCOL.txt.
  Preparation PASS:2320protected files,unchanged builds/profiles/model; saved
  CP-K21 fixture1492poses/3460frontend updates passes,seven negative coverage
  cases fail as expected. No source or solver replay for these tests.
  Full OFF launched ONCE about10:55local,2026-10-21;startup settings pass.
  Native sources and WNOA co-running at0.20x;no final result yet. Check owned
  controller before action;three ordinary OFF gates required before ON.
  Follow-up approval requested,not enabled. No retry/rebuild/tuning/push.

CP-K22 status2026-10-21~11:25local: OFF stopped11:14:27 after documented host
suspend11:11:19--11:14:25(lid close). Event139001 is first Kimera972->973rev0,
arrival+210.708s,first endpoint13.009s old versus7s retained window. Known-ID
expired handling works in prior commit139000; first old arrivals still error.
Measured adjacent Kimera backend141/148ms versus186s host sleep,so do not blame
this interruption on optimization load or claim a full-run outcome. No ON,
score,retry or policy change. Evidence and exact epochs in CP-K22 REPORT.txt.
Owned jobs exited; restoring originally-stopped containers. Follow-up never
activated. No automatic retry; future run needs awake-machine protection and
explicit authorization,not a wider window or relaxed first-arrival guard.
Closeout verifies2320protected hashes unchanged,ON absent,both containers exited.

### CP-K23 — awake-machine reattempt, 2026-10-21

- User explicitly reauthorizes full matched OFF/ON after CP-K22 host sleep.
- COMPLETED: same frozen CP-K22 settings and coverage/scoring checks; new
  roots/port11473 only. Temporary sleep/idle/lid-switch inhibitor held by a
  finite OFF/gates/ON/gates/scoring controller. No persistent power change.
- Protocol: experiments/eth1_awake_full_cpk23_20261021/PROTOCOL.txt.
  Reuse prior saved-data checks, no extra solver/source tests or rebuild.
- No automatic retry; failure stops pair. Keep old capture; no tuning,
  guard relaxation, covariance/lag change, promotion or push.
- Preparation PASS2391protected hashes,identical source/model/profiles/coverage.
  One-shot controller launched ONCE11:37:49local,PID105779,session5247; verified
  temporary systemd inhibitor PID105777 blocked sleep/idle/lid handling.
- Both full346.32s captures,three gates each,evaluation and summary passed once.
  Native GLIM ATE OFF/ON.039683/.037421m; native Kimera4.496229/.340930m;
  fusion ONLINE.166126/.103489m. DELAYED fixed-lag finalized.155965/.098779m.
  Full saved spans345.400/342.598/345.500s; common mask296.700s covered across
  298.300s envelope due native Kimera gaps. Shared-CPT7 IE-TC reference.
- Classic ON injected1473factors each direction;1460Kimera retirements/refreshes.
  All LOCAL covariance payloads exact; receiver expired131OFF/130ON known old
  versions under selected policy; pending/unsupported end0. Bounded71supports.
  MAIN prepared consumptions4,909,738OFF/5,292,978ON,MAIN synchronous calls0.
- Controller cleanup failed on docker top comm,args format after results passed.
  Original failure preserved; manual authorized cleanup2026-10-21~13:58 verified
  no jobs,restored both containers exited,released inhibitor and rehashed2391
  protected files unchanged. See closeout.json; no source/validation/evaluation
  repeated. No tuning,new test,default promotion,push or next sequence.

### CP-K24 — full EIG-2 matched comparison, 2026-10-21

- Authorization: user requests saving the ETH1 report and the same full run/report
  for EIG-2. ETH1 reference saved as GNSSFGO_ETH1_FULL_CPK23_REFERENCE_REPORT_20261021.md.
- Preparing one OFF/ON pair, unchanged CP-K23 binaries, model, covariance,
  batching, temporary-CBS and absorbed-revision policy. No tuning/rebuild/retry.
- EIG-2 saved sensor profile; established5.7s offset through final Hesai,
  355.02s at0.20x (not historical305s crop), port11474. Bag/stereo/reference
  preflight passed. See experiments/eig2_full_cpk24_20261021/PROTOCOL.txt.
- Same ordinary capture/LOCAL/lifecycle/receiver gates and gap-aware scoring.
  Window-specific counts only; fix prior Docker top cleanup format in new
  controller. Temporary inhibitor required; restore originally-stopped containers.
- Preparation PASSED2464protected files,syntax/XML,reviewed harness diff and
  unchanged normalized models/binaries/profiles. Finite controller launched ONCE
  14:34:19local,PID133089,session79125,inhibitorPID133087. OFF startup/effective
  checks passed and all three estimators publish; no completed result yet.
  Controller owns OFF/gates/ON/gates/evaluation/summary/cleanup; do not duplicate.
  No push/default promotion/next experiment authorized.

CP-K24 follow-up: both355.02s captures finished; OFF all3gates pass. ON source
coverage/LOCAL/lifecycle pass; receiver validation failed15:36:21 at strict
unsupported_end==0 gate. Exactly1final Kimera1759->1760rev0 endpoint is11.7451ms
beyond newest100ms support; next bracket after last input clock. Existing EOF
exclusion,not an estimator crash. Full raw trajectories/covariances retained;
scoring/summary not run, no validated OFF/ON accuracy result yet. Cleanup passed
15:36:38,both containers exited,2464protected hashes intact. No retry, gate
relaxation or implementation changes. Resolve saved-data end-boundary first.

### CP-K24A — saved-data terminal boundary and scoring

- User authorizes proceeding after CP-K24's EOF validation blocker. COMPLETED
  under explicit finite-recording contract; original strict ON failure retained.
- Existing Receiver.h::end explicitly excludes future-bracket pending factors;
  legacy_test.cpp already covers that behavior. Original validator assumed0.
- Add a separate finite-recording audit, same OFF/ON predicate: only the next
  unsupported100ms state beyond final clock, measured endpoint still<=clock,
  start retained, exact EOF classification/conservation. Keep original failure
  and every other check. No source/solver rerun, fabricated support or gate edit.
- If supplemental checks pass, score unchanged full-saved/common trajectories
  and produce the requested report with explicit terminal omission. No tuning.
  Protocol: experiments/eig2_full_cpk24_20261021/eof_review/PROTOCOL.txt.
- Both supplemental validations pass, every remaining original receiver check
  unchanged; OFF0/ON1terminal omissions, no pending/expired. All2464protected
  files and16saved output/input hashes unchanged. No source/solver replay.
- Existing evaluator and supplemental summarizer ran once. Full-saved ATE
  OFF/ON: native GLIM.051085/.044555m,Kimera1.882644/.688834m,ONLINEfusion
  .167508/.135765m. DELAYED.166193/.132314m separate. Identical common mask
 3526timestamps/352.299664covered seconds confirms direction; fullsaved354s+.
- Report GNSSFGO_EIG2_FULL_CPK24_REFERENCE_REPORT_20261021.md records settings,
  metrics,paths,covariance,timing and terminal limitation. IE-TC shares CPT7.
  Original status remains failed; qualified results are results_eof_review.json.
  Both containers stopped; eof_review/closeout.json verifies completion.
  Missing factor not recovered; no zero-factor-loss, nonlinear stationarity,
  exhaustive batching or universal EOF-policy certification. No next run/push.

### CP-K25 — full usable SPX-3 matched comparison, 2026-10-21

- User explicitly requests SPX-3 similarly after completed ETH1/EIG-2. OFF
  launched ONCE16:53:28+03:00, one fresh OFF/ON pair at0.20x with unchanged
  CP-K23/24 binaries/model. Finite controller PID157459/tool session49183 owns
  sequential capture/gates/scoring/cleanup; refuse duplicate stages.
- SPX-3 saved CP-R5 profiles and32s start; original bags establish194.02s
  through last Hesai, not older134s crop. Four bag hashes,1941Hesai/19402IMU/
  1940synchronized stereo pairs and whole-window IE-TC coverage pass. Same
  native/ONLINE/delayed scoring and covariance/revision telemetry.
- Preparation2538 protected hashes/model/profile equivalence pass; OFF
  effective settings passed before playback, all three trajectories publish.
  Startup evidence is not completed-run validation. Scoped SPX3-CPK25 sleep
  inhibitor, no persistent follow-up. Read current status.json before acting.
- Predeclare CP-K24A finite-recording terminal-cell audit equally OFF/ON; do not
  fabricate supports. Keep all other gates, report any omitted terminal factors.
- No rebuild/tuning/retry/push or unrelated method. Fresh directory:
  experiments/spx3_full_cpk25_20261021. OFF must pass all gates before ON.

### CP-K26 — fresh full SPX-3 restart after host suspend, 2026-10-21

- User explicitly requested full restart. CP-K25 OFF completed/passed; ON
  stopped after host suspend17:14:12--17:34:44,first Kimera167->168 arrived
  9.0119003s old beyond7s retention. Prior evidence preserved; cleanup passed.
- Prepare one fresh194.02s OFF/ON pair at0.20x,offset32s,port11476. Same
  frozen K25 binaries/profiles,classic temporaryCBS,all-LOCAL,WNOA7s/no central
  IMU,weights,retirement/EOF/coverage gates. No model/settings changes.
- Scoped GNOME suspend/idle inhibition supplements system sleep/idle/lid lock.
  Probe passed; no permanent policy changes or claim of immunity to forced
  suspend. K25 inhibitor bypass's exact cause is still unverified.
- Preparation passes2613 protected hashes/four freshly rehashed bags,identical
  input window/profiles/binaries/gates. Reviewed harness diff; only identities,
  port11476 and additional GNOME inhibition assertion change. Before launch,
  corrected a copied prose port11475 to11476 and re-froze those3doc/preparer
  hashes; actual controllers/config already11476. No estimator/gate change.
- OFF launched ONCE19:41:05+03:00,PID178759/tool session51733. Both scoped
  inhibitors verified; effective settings pass,GLIM/Kimera/fusion outputs advance,
  queue0 at~4.9s. Startup only,not final validation. Current state:
  experiments/spx3_awake_full_cpk26_20261021/status.json. Sequential finite
  controller owns OFF/gates/ON/gates/scoring/cleanup. No automatic retry,
  duplicate run, source code edit, rebuild, tuning or push.

- COMPLETED2026-10-21T20:16:36+03:00; reference report finalized2026-10-22.
  Both194.02s captures,three gates per mode,scoring and closeout passed. No
  controller/cleanup error; both containers stopped,2613protected files intact.
- Full-saved ATE OFF/ON(m): original GLIM.016897/.016885,original Kimera
  .552484/.120067,fusion ONLINE.058603/.044991. DELAYED finalized.051392/.039281
  separate. Fullsaved193s+ spans,common190.899818s covered confirms direction.
  IE-TC shares CPT7; existing covariance-alignment approximation unchanged.
- One final Kimera revision0 omitted in each mode under predeclared same EOF
  predicate: endpoint within recording but future100ms support beyond clock.
  Exact conservation and every other gate pass; impact of omission unknown.
- ON injections922G2K/933K2G;898Kimera retirements/priorrefreshes. GLIM MAIN
  prepared hits890922/897958,sync0/0,queue max1. Scan medians37.653/36.818ms;
  fusion support43.688/43.989ms; each capture~16m56s. No1x speed certification.
- Saved GNSSFGO_SPX3_FULL_CPK26_REFERENCE_REPORT_20261021.md and results.json
  in experiments/spx3_awake_full_cpk26_20261021. No stage repeated,solver replay,
  exhaustive batching/stationarity claim,tuning,push or default promotion.

### Subsequent task template

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
