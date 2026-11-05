# GrandTour/Newer College CBS and gnssFGO work

**CP-V112 attempt1 LEICA-2 corrected fresh pair (2026-11-05; COMPLETE, post-audited):**
H=experiments/leica2_split_imu_live_pair_cpv112_20261105_attempt1. Both fresh
428.75s captures and all four gates per arm passed; saved-only evaluation and
report verification completed03:14:32local. Read RESULTS_REPORT.txt/results.json,
complete.json,report_verification.json and post_closeout.json. REPORT.txt is a
sealed historical startup snapshot, not current status; NEVER overwrite it.
Original failed CP112 H/P remain unchanged and are not reused as the OFF control.
The sole operational correction was selected6.0s -> rosbag6.099999100s; intended
absolute bounds, scientific profiles/guards, corrected985cKimera both arms,
CP82 GLIM and CP-K19 seven-second receiver remained unchanged. Actual loaded
DSO/maps and OFFfalse/ONtrue checks passed before each playback.

Method1 EVO1.36.3 SE3/no-scale versusIE-TC at the calibrated AP20 physical point,
NOT independently measured prismGT. Six-curve common100ms mask:3844rows,
384.0s/428.75s(89.5627%),378reference-distance0.5m pairs; reference205.8974m.
Common ATE OFF->ON: GLIM.498242->.501302m(-.61%),Kimera.836987->.481459m
(+42.48%),ONLINEfusion.552047->.501756m(+9.11%). tRPE.019023->.016755 /
.067940->.020546 / .031803->.029873m; rRPE.389261->.327163 /
.469991->.486466 /1.475251->1.468669deg. Estimatedpaths205.7319->205.5826 /
211.3859->205.1373 /223.3019->218.5818m. Fusion improves OFF->ON but does not
beat either ON native estimator on common-mask ATE; Kimera rRPE worsens slightly.
Separate7s-finalized common ATE.551374->.501167m(+9.11%),tRPE.017100->.016153,
rRPE.263808->.245387deg,path205.1476->205.5315m; NOT ONLINE/full-history.
Full-native ATE separately: GLIM.572734->.574095,Kimera.841780->.486306,
ONLINE.630471->.576433,7s-finalized.629670->.575966m. Scoredrows GLIM4280both,
Kimera1938->1933,fusion4282both; supported427.8995 /387.5961->385.1961 /428.1s.
Kimera11->15gaps>0.5s,max5.099946s retained. Common segments0.684928–57.584928,
60.484928–195.384928,196.684928–304.184928,305.684928–390.384928s; last38.365072s
not common-scored. This is a full capture, not dense six-curve coverage. No filling.

Both arms exact42875CPT7/85688Alpha receipts and4287matchedstereo;1camera pair
preinitialization each,0terminalcamera omissions. Native GLIM4281rows each;
one row excluded by scoring support. Receiver252537->253503versions,
GLIM204882->206014,Kimera47655->47489; replacements246129->247116.
50preinit exclusions each;171expiredKimera versions/24IDs OFF ->156/25 ON,
seal_at_absorption preserved;0unsupportedEOF/duplicate/stale/pending. Omitted
expired corrections are not proven harmless. Fullmean/covariancepayload parity
passes,error0covariance and <3.1e-14mean roundtrip. Framewire4278GLIMboth,
1937->1932Kimera,0unmatched;ON1907K2G/1888G2K persistent insertions checked.
Kimera native mediantranslationtrace9.713685e-7->6.993648e-7m2; transported
9.949962e-7->7.156456e-7m2;rotation3.856615e-8->3.384317e-8rad2. GLIMtransported
mediantranslation1.781438e-6->1.784170e-6m2. Unpaired relative-belief distributions,
not calibrated global uncertainty. Kimera optimize median201.193->213.286ms;
receiver processingp9518.740->18.337ms,supportcyclep9566.237->63.613ms;
previouspose revisionmax.769022->.033547m. Actual order/arrivals/revisions retained.
Supplemental postaudit03:17:51 passes273236pins/647sealedoutputs/Hesai667,
original failed262969regularfiles+2specialmetadata(noFIFOreads),24exactownedjobs
exited and both initially-stopped containers restored stopped. Exactly2source
wrappers; no post-audit source runs, rescoring, retry or further experiment.
Scoped heartbeat PAUSED after terminal notification. Persistent
mixedprior/native-onlyoutgoingFALSE and experimentalAlpha remain; IE-TCsharesCPT7,
paths are estimated polylines, parity is not stationarity/calibration/health.

Historical preparation/launch snapshot below is superseded by this closeout:

Latest user proceed authorizes ONE fresh OFF/ON after the original coverage
failure below. H=experiments/leica2_split_imu_live_pair_cpv112_20261105_attempt1;
P resolved from H/prepared.json. Read PROTOCOL, PRELAUNCH_REVIEW,
PREPARATION_REPAIR, REPORT and current receipts. Do not restart original CP112.
Same intended428.75s absolute window1732549034915071665..1732549463665071665ns,
selected-topic offset6.0s/.20x/25sdrain. Correct rosbag all-bag-relative start is
6.099999100s, recorded separately as CBSMS_BAG_PLAYBACK_START_SEC; original
CBSMS_BAG_START_SEC remains6.0. No sensor/reference epoch shift or gate relaxation.
Host original-bag check matches ALL five unchanged expected timestamp sequences;
negative old6.0 rejects allfive with1/10/20/1/1early/missingtail messages.
Preparation port substitution accidentally touched copied JSON digits; host
tests rejected it before launch. Rejected copies/diff preserved, sevenJSON
restored byte-identical; final parity supersedes initial copy receipt. All
host30fixtures/40geometry/14negative schemas/5EOF/5syntheticEVO checks pass.
No new scientific scores. Same corrected985cKimera both arms, CP82 GLIM and
CP-K19 seven-second fusion, exact scientific profiles/math unchanged.
Fresh11653/11654ports and leica2_cpv112a1_{off,on}_20261105 sessions.
Admission01:54:47 passes273236pins/Hesai667, preserving original10253 and
entire failed H/P262969regular files+2special metadata; NO FIFO reads.
Initial174311362560Bfree>25GiBrequired;20GiBoutput+5GiBreserve,5GiBruntimefloor.
Started ONCE01:55:06local: launcher86072 exited, supervisor86073/start1938945,
controller86075/start1938946 exact host commands/identities verified. Both
containers initially stopped. Actual Noetic30adapterfixture passed01:55:45;
allfive-stream exact window check passed01:55:52. OFF capture86777/start1943545.
OFF effective preplayback checks passed:985cDSO/maps/body/TF/native_local,
both receivefalse and unchanged Alpha params. playback_offsets.txt records6.0
selected/6.099999100rosbag. Actual first CPT7stamp1732549034919515000 and
Alphastamp1732549034918935177 match expected. Native GLIM/Kimera outputs and
livefusion confirmed:57ONLINEposes/1730belief versions,queue0/max26 at startup.
ON pending; no final scientific results. Read current receipts, not old status.
Existing scoped heartbeat ACTIVE for attempt1 ONLY; quiet during normalprogress.
Sole controller owns OFF+fourgates->ON+fourgates->cleanup/restore->savedEVO/verify.
No duplicate capture/validation/scoring, automatic retry, tuning/build/install/
delete/protected edit or other sequence. Afterterminal/exactexit run postaudit
once ifabsent, preserve immutable artifacts, update finaldocs and pause monitor.
Native-onlyoutgoingFALSE/peer correlation and experimental Alpha model remain;
IE-TCsharesCPT7, no prismGT independence or stationarity/health certification.

**CP-V112 LEICA-2 FULL LIVE PAIR (2026-11-05; FAILED OFF coverage, post-audited):**
Terminal failure01:31:55local; ON NEVER STARTED. Read H/FAILURE_ANALYSIS.txt,
failed.json,capture_off.log and supplemental post_closeout.json. No full-pair
results/EVO scores. Original terminal artifacts and captures remain unchanged.
The OFF source completed rosbag playback, then coverage_control.py:12 rejected
exact IMU timestamps. This is a playback-window origin mismatch in the harness:
audit used first SELECTED-topic epoch1732549028915071665ns, but rosbag playback
with --start=6.0 followed all-bag epoch1732549028815072565ns. Difference99999100ns
(99.9991ms). Captured window1732549034815072565..1732549463565072565ns, not the
intended1732549034915071665..1732549463665071665ns. Read-only original-bag diagnosis
confirms BOTH saved IMU header sequences exactly equal the all-bag-origin window.
Counts alone matched42875CPT7/85688Alpha, but10CPT7/20Alpha early messages replace
the same number of intended tail messages. No guard relaxation or relabeling.
Prelaunch selected-topic adaptation did not translate the launcher's offset;
earlier claims that unused bag beginnings cannot affect playback were incorrect.
This failure is not evidence of CBS/covariance failure or random estimator crash.
Receiver saved4282ONLINEposes/253227belief versions(205622GLIM/47605Kimera),
queue0/max46 and a receiver complete marker; these do NOT qualify the rejected
source or a complete pair. Four saved-data gates/scoring were not reached.
One-time postaudit01:34:12 passes10253pins/Hesai667 and9exact owned process exits,
both initially-stopped containers restored stopped; no complete.json exists.
CP111/earlier protected evidence unchanged. No retry, replay, tuning or deletion.
Heartbeat PAUSED after failure notification. Future recovery needs a fresh
bounded protocol and playback-origin correction; do not edit/restart this run.

Historical preparation/launch record below is retained, superseded by the
terminal status above (not an authorization to resume):
Latest user proceed authorizes next full usable LEICA2 OFF/ON with GLIM,
corrected-private Kimera and concurrent gnssFGO fusion in BOTH arms.
H=experiments/leica2_split_imu_live_pair_cpv112_20261105. Read PROTOCOL,
PRELAUNCH_REVIEW,PREPARATION_REPAIR,REPORT,prepared and exact launch/status
receipts before acting. One428.75s offset6s quiet-start/.20x/25sdrain pair;
historical CP-K29 window, not rawbagzero. Same CP111/108 scientific profiles,
private binaries and receiver policy; corrected985c115dKimera both modes.
No tuning/covariancefreeze/seedchange/build/install/delete/retry/othersequence.
Original missing Alpha bag downloaded via existing authenticated Klein CLI,
6607426bytes,sha ba392c29402e102bb469bf68a19d8580d4d7ac86a99fe74ef542870670f2065e.
Raw audit passes4288Hesai/42875CPT7/85688Alpha/4287matchedstereo, quiet and
missionTF/intrinsics<1e-9. Selected-topic epoch1732549028915071665 retained;
start1732549034915071665,end1732549463665071665ns. No input omissions.
Host30adapterfixtures/40geometrycases/14negative schemas/5EOFcontrols and
5syntheticEVOtests pass; actual Noetic/runtime effective checks still mandatory.
Final admission00:53:33local passes10253protected hashes/Hesai667, preserving
CP1119721pins/415sealedoutputs/postaudit and earlier evidence.20GiBoutput+
5GiBreserve,5GiBruntimefloor; ports11651/11652;178098688000bytes initiallyfree.
Started ONCE00:54:25local: launcher63091 exited,supervisor63092/start1574815,
controller63094/start1574815 exact hostidentity verified. Both containers were
initially stopped. Actual Noetic30fixture bitparity passed; OFFpreplayback
effective checks passed985cDSO/body/topics/TF/receivefalse. Sensorplayback and
both native estimators plus live fusion confirmed:51ONLINEposes,1371belief
versions,queue0/max14 in startup snapshot. ON not yet started; no finalresults.
Existing scoped heartbeat reactivated forCP112 only; quiet duringnormalprogress.
Controller alone owns OFF+4gates->ON+4gates->cleanup/restore->savedEVO/verify.
No competing validation/scoring. REPORT immutable when sealed. Afterterminal
and exactjobs exit,post_closeout.py once ifabsent; then finaldocs andpausemonitor.
IE-TCsharesCPT7/notindependentprismGT; native-onlyoutgoingFALSE/mixedpriorpeer
history remain. Alpha model experimental; paritynotstationarity/health.

**CP-V111 SPX-3 FULL LIVE PAIR (2026-11-04; completed and post-audited):**
User explicitly authorized next full SPX3 and automatic approval handling.
H=experiments/spx3_split_imu_live_pair_cpv111_20261104. Read PROTOCOL,
PRELAUNCH_REVIEW,PREPARATION_REPAIR,REPORT and current receipts. One fresh
197.02s quiet-start offset29s/.20x/25sdrain OFF/ON pair, GLIMHesai+CPT7,
corrected-private KimeraAlphaIMU and concurrent CP-K19 7sWNOA in BOTH.
Same CP108 profiles/binaries/scientific settings; no CP106/110 covariancefreeze.
Original missing Alpha bag retrieved through existing authenticated CLI,
3380171bytes,sha f5c304ac0437ea8f1d5c67c34f465839019b77bbfdd25363e4558c4469b7b24f.
Raw1971Hesai/19702CPT7/39370Alpha/1970matchedstereo; quiet/calibration checkspass.
Preparation-only CLI-order/raw-count repairs preserved; no source failure.
Final admission passed9721protected hashes/Hesai667;181559201792bytes free
against21GiB reserve. Started once20:45:43local: launcher16179 exited,
supervisor16180/start82628,controller16182/start82629 verified on host.
Both containers initially stopped; actual Noetic30fixture bit-parity passed.
Both effective checks passed before playback with corrected DSO985c115d,
split topics/body/TF and OFFfalse/ONtrue. Both197.02s captures, all4gates each,
cleanup/restore and48EVO archive checks completed21:22:39local. Read sealed
RESULTS_REPORT.txt/results.json/report_verification.json/complete.json and
supplemental post_closeout.json(21:27:34). REPORT.txt is immutable, do not edit.
Method1 EVO1.36.3 SE3/no-scale vsIE-TC, six-curve common100ms1931rows,
192.7s(97.807%)/167reference-distance0.5m pairs; reference path90.7671m.
ATE OFF->ON GLIM.016738->.017135m(-2.37%),Kimera.642416->.105379m(+83.60%),
ONLINEfusion.127351->.047204m(+62.93%). tRPE .006773->.007751 /
.052750->.021985 / .026023->.027637m; rRPE .256731->.285556 /
.818359->.581779 /1.420132->1.367180deg. Paths90.9522->91.0552 /
91.8220->90.5480 /93.8367->93.9612m. Better fusionATE not better tRPE.
Separate7s-finalized ATE.126740->.042982m(+66.09%),tRPE.009511->.009792,
rRPE.463332->.410165deg,path90.5633->90.5379m; NOT ONLINE/full-history.
Full-native scoredrows GLIM1962both/196.099s;Kimera958->957/193.498->194.198s,
4->3gaps>0.5s;fusion1965both/196.4s. Full-native ATE GLIM.016766->.017149,
Kimera.651979->.108232,ONLINE.132543->.050728,finalized.131348->.045225m.
All1970stereo pairs retained;1preinitcamera each, no terminalcamera loss.
Receiver116645->116666versions(93645->93791GLIM,23000->22875Kimera),
1965ONLINE and finalized each;25preinit and1unsupportedterminalKimera each,
0expired/duplicate/stale/pending. Full mean/covariance payload parity passes,
covarianceerror0; meanroundtrip<2e-14. Framewire1960GLIMboth,957->956Kimera;
ON935K2G/925G2K persistentinsertions checked; no temporaryretirement.
Kimera native median translationtrace3.72006e-6->2.13112e-6m2;
CPT7-transported3.81099e-6->2.16039e-6m2;rotation5.74082e-8->3.89937e-8rad2.
GLIM transportedmediantranslation1.66588e-6->1.67757e-6m2. These are unpaired
relative-belief distributions, not calibrated global uncertainty/independence.
Kimera optimize median82.747->85.037ms; receiverprocessing p9522.736->22.260ms,
supportcycle p9569.015->68.239ms; previouspose revisionmax.33100->.05399m.
Postaudit verifies9721pins/415immutableoutputs/Hesai667,23exactownedjobs exited,
both initially-stopped containers restored stopped; no supplemental source runs.
Old CP108383/CP10914/CP110327 evidence preserved. Scoped heartbeat paused.
No retry/tuning/install/build/delete/old-run overwrite. LEICA2 not started.
IE-TCsharesCPT7, not independent prismGT; paths estimated polylines not travel;
incremental parity not nonlinear stationarity/health. Mixedprior/native-onlyFALSE
and experimental Alpha noise model remain, not repaired. No further run authorized.

**CP-V110 EIG-2 SAVED REGIONAL INTERVENTION (2026-11-04; completed, NOT a fix):**
H=experiments/eig2_fusion_intervention_cpv110_20261104; read REPORT.txt,
results.json,validation.json,report_verification.json,closeout.json,complete.json
and regional_intervention.png. Four receiver-only replays, zero source runs.
Unchanged CP108ON control is bitexact CSV/TUM/summary/all221071archivefiles/
249253logicalprogress rows; measured changes are conditional saved-input effects.
273Kimera IDs with both endpoints50–70s or275–310s; freeze afterage2.5s at
last accepted anchor: hold_cov/hold_mean/hold_both,3393late rows each. Same
CP-K19 binary/wrapper/runtime/7sWNOA/settings and recorded order/arrivals.
Consumed mean/full6x6 payload and logical lifecycle parity pass; preserve
50preinit/0expired/1unsupportedEOF,3545ONLINE/finalized publications each.
Method1/EVO1.36.3 SE3/no-scale versusIE-TC, original common3532rows/349pairs:
ONLINE ATE baseline.13465044 -> cov.13355237(+.8155%),mean.14017844(-4.1054%),
both.13824787(-2.6717%)m. Separate7s-finalized.13230966 -> .13107693/
.13785753/.13576609m. Full-native3545rows separately in report, no gap filling.
Fixed baseline alignment ONLINE whole RMS.13465044 -> .13473989/.14667914/
.14199616m: tiny covariance-only ATE gain is alignment-sensitive. Fixed-fit
298–304s RMS.213534 -> cov.200232m, but own-fit.213558m; peak not removed.
Local covariance benefit redistributes error; mean freezing helps some early
regions but hurts late/overall. Neither intervention solves ON degradation.
Earlier weights/means and source feedback history remain untested; do not
declare covariance/correlation innocent or deploy a blanket freeze. Held
covariance is not uniform inflation. IE-TCsharesCPT7; paritynotstationarity.
48EVOarchives verified. Closeout8873pins/CP108383/CP109seal/original221071files/
Hesai667 pass; four221071-file archive manifests sealed with327outputs.
Exact owned jobs exited, both containers restored stopped. All original
evidence preserved. No further trial or production-policy change authorized;
suggest saved-data young-belief/directional-weight diagnosis. Heartbeat paused.

**CP-V109 EIG-2 FUSION DISCREPANCY LOCALIZATION (2026-11-04; completed):**
Read experiments/eig2_fusion_localization_cpv109_20261104/REPORT.txt,findings.json,
localization.json,belief_analysis.json,verification/complete.json and twoPNGplots.
Saved CP108 only; no capture/replay/EVO/tuning/build/deletion/settings change.
Exact3532-row saved error arrays/alignments reused. Same-initial-frame ONLINE
OFF/ON separation continuously>5cm from53.136s, not a sudden causal onset.
Global-fit window RMS OFF->ON:50–70s .086577->.126583m;170–178s
.068831->.181654;298–304s .079719->.213534. Finalized discrepancy persists;
280s-to-end39.51%of net extra ONLINE squared error. OFF has larger single-point
spike.4051m@144.136s vsON.3126@143.036; not the cause of ON penalty.
1615same Kimera physical intervals: terminal ON/OFF mediantranslationtrace
.59549; withinON age2.5s->terminal pairedmedian.53754 versusOFF.98525.
Ratio of mean tracesON.38553 while reference residualRMS7.699->7.968mm.
All6directions contract late62.35%ON. GLIM terminalratio1.00214/late~1.
At50–70s Kimera final relative discrepancy2.821->4.612mm despite slightly
highertraceON; localmeans matter too. At275–290s discrepancy2.943->4.190mm
whilemeantrace.4964->.3338mm2;290–310s5.057->5.325mm and.6421->.3754mm2.
GLIM improves in these windows. No largequeue/expiry/tailgap at mainpeaks.
Suggests localmeanbias plus misweighting/correlation; does NOT prove persistent
feedback causality. IE-TCsharesCPT7; residualnotNEES/NIS,5sdiagnosticnot0.5mRPE.
All383CP108sealed outputs/10406read inputs unchanged;1600trajectorychecks,
40520SPDmatrixchecks pass. No newexperiment/replay authorized; heartbeatpaused.

**CP-V108 EIG-2 LIVE PAIR (2026-11-04; COMPLETED, audited):**
H=experiments/eig2_split_imu_live_pair_cpv108_20261104. Read PROTOCOL plus
PRELAUNCH_REVIEW/PREPARATION_REPAIR, RESULTS_REPORT.txt,results.json,
report_verification.json,frame_off/on.json,receiver_validation/off/on/validation,
complete.json and supplemental post_closeout.json. Do NOT rerun/overwrite.
Both fresh full355.02s/offset5.7s/.20x/25sdrain captures and all gates passed;
controller completed14:40:51local, post_closeout14:55:11. GLIMHesai+CPT7,
KimeraAlpha stereo+built-inIMU, concurrent CP-K19 7sWNOA both arms. Corrected
CP102985c115d Kimera BOTH modes, persistent/Gaussian1, receivefalseOFF/trueON.
No covariance freeze/tuning/observer/nativeIMU override or centralIMU/GNSS.
Method1/EVO1.36.3/SE3no-scale versusIE-TC, reference-distance0.5m RPE:
six-curve common3532rows/353.0s(99.431%)/349pairs,refpath188.0137m.
GLIM ATE.046328->.047447m(-2.42%),tRPE.019707->.017653m,rRPE.638466->.634358deg,
path189.7889->189.7217m. Kimera3.189958->.366042m(+88.53%),.358022->.043514m,
.907520->.868306deg,path237.8221->187.4987m. ONLINEfusion.083176->.134650m
(-61.89%),.039580->.041280m,1.996855->2.018519deg,path201.2903->198.5043m.
Separate7s-finalized.078442->.132310m(-68.67%),.018001->.018698m,
.671155->.663871deg,path186.7931->187.0396m; not ONLINE/full-history.
Full-native separate: GLIM3543rows/354.1993s,Kimera1763/1762rows and
353.8972/353.7972supportedseconds, fusion3545rows/354.4s each. Full-native ATE
GLIM.046492/.047602,Kimera3.197040/.365911,ONLINE.085702/.135921,
finalized.078444/.132332; native path/RPE/support differ, see sealed report.
All3550raw stereo pairs matched;1pre-init pair each. Kimera native gaps.8/1.0s
retained,no interpolation>0.5s;10common queries excluded.50pre-init belief
versions each;0expired. ON1terminal Kimera revision lacks future100ms support,
explicitly omitted under frozen EOF contract; not zero-loss,clock not extended.
213742/213750fusion versions,208405/208415replacements,71retainedsupports;
support-cycle median51.34/51.93ms,p95~72ms. Sourcewire3541GLIM each/1762->1761
Kimera;ON1742G2K/1748K2G insertions. All payload parity error0,frame gatespass.
Kimera previous-pose revision>0.5m27->0;max5.626->.269m. Optimize median
248.0->271.1ms. Pooled all-version transported Kimera translationtrace mean
4.029e-6->1.204e-6m2,butmedian5.220e-7->5.826e-7; native mean4.007e-6->1.189e-6.
GLIM transported median1.860e-6->1.880e-6. Not paired-age calibration/causality.
All8873protected hashes,383sealed outputs,Hesai667copies/originals verified;
all exact owned jobs exited,both initially-stopped containers restoredstopped.
CP1038120pins/CP103-107terminal evidence unchanged. No deletion/build/retry.
REPORT.txt immutable; later notes/audits supplemental only. Scoped heartbeat
PAUSED after completion notification; no SPX3/LEICA2 or other run started.
IE-TC sharesCPT7,not independent prismGT; paths estimatedpolylines not travel.
Mixed-prior/native-onlyFALSE and experimentalAlpha calibration remain;
incremental parity is not nonlinear stationarity/health certification.

**CP-V107 COVARIANCE MATURATION / PROVENANCE (2026-11-04; completed analysis):**
Read experiments/eth1_covariance_provenance_cpv107_20261104/REPORT.txt,analysis.json,
insertions.json,verification.json,code_review.json,closeout/complete.json and plot.
Saved CP95OFF/CP103ON only; no capture/replay/EVO/build/tuning/deletion/change.
Same1454physical Kimera intervals: paired median age2.5->terminal translation
variance ratio.267863ON/.920147OFF; native Alpha.268708/.921628, so not body
conversion artifact. All36778OFF/36746ON native->received6x6 matrices match to
6.10e-16relative. Last0.4s lifetime per-revision mediantrace ratio.835611ON/
.996299OFF; sustained late contraction, not a handful of extreme drops.
All6generalized covariance directions contract late for83.77%ON/36.93%OFF.
ON reportedtranslationnorm1.929->.846mm while IE discrepancy5.131->5.142mm;
meanvariance-80.76%, distinct from paired median-73.21%. OFF also uncalibrated.
1472actualsource insertions eachdirection finite/SPD/exactpayload; these young
wire factors are distinct from mature fusion revisions. Classic source CBS
does NOT forward every diagnostic revision. No stacking of same-ID revisions
found: receiver replaces oldslot; inter-ID/source correlations remain unmodeled.
Frozen currentcode confirms persistent absorption then explicit LOCAL filtering
cannot remove absorbed peer history. Prior CP67 fixture demonstrates mechanism;
CP98 separately shows state-dependent native visual information before marginal.
Exact contribution to this CP103 late contraction NOT isolated: after8.499951s
every source update retires a pose, so retirement correlation is confounded.
Separate IMUs/persistent labels do not establish uncertainty independence.
Next proposal ONLY: qualify positively admitted native-only marginal history at
fixed measurements/states and compare before/after absorption; missing historical
graphs may require separately authorized short capture. No automatic replay/fix.
Initial parser ordinal/update-ID mistake preserved; diagnostics-only repair.
26544independent matrix checks pass;8120pins,CP10362/104318/10512/106186/95426
sealed outputs and Hesai667copies+originals pass; bothcontainers stopped.
14newoutputs sealed; heartbeat unchangedpaused. CP106 freeze stays diagnostic.

**CP-V106 KIMERA COVARIANCE INTERVENTION (2026-11-04; completed):**
User approved CP105's parity-first saved ON test. Read experiments/
eth1_covariance_freeze_cpv106_20261104/REPORT.txt,PROTOCOL.txt,prepared.json,
baseline_parity.json,validation.json,results.json,report_verification.json,
closeout.json,complete.json. Exactly2receiver-only replays,0source runs;
same CP103 ON journal/CP-K19 binary/settings. Baseline trajectories/state CSV/
summary/all211422archive files bit-identical;237614logical progress rows equal.
Variant holds1472Kimera IDs' full6x6 covariance at first actually accepted
matrix, preserving all mean revisions/endpoints/arrivals/order/GLIM/clock/EOF.
Only35249covariance rows differ; consumed payload error0; all lifecycle counts
and3456ONLINE/finalized timestamps unchanged (25pre-init/134expired retained).
DirectEVO1.36.3/Method1/SE3no-scale/IE-TC, same2966row/365distance0.5m mask:
ONLINE ATE.094335->.052415m (44.44%better); separate7s-finalized.091336->.043808
(52.04%). ONLINE tRPE.036492->.036603m,rRPE1.744175->1.745136deg,path214.146->
214.050m/ref201.374; DELAYED tRPE.012826->.013278,rRPE.323322->.322830,
path201.970->201.971. Full-native separately ATE.095034->.056967/.091602->.048724.
Covariance schedule materially affects ATE in this fixed stream; not random
source outcome, but NOT proof of source-side mechanism/calibration or a
qualified production freeze. Short-scale RPE/ONLINE path inflation persists.
No new live OFF/ON, source behavior change, build/install/tuning sweep/deletion.
Host ldd preflight rejected missingBoost1.74 before any run; original Ubuntu
runtime used, no install; source container stayed stopped, Ubuntu restored.
24EVOCLI archives/API parity0;8120pins,CP10362/CP104318/CP10512terminal outputs,
original211422archivefiles,Hesai667copies+originals preserved; exact jobs exited.
Seal186top-level/evaluation files plus two211422-file archive manifests. Do not
rerun/overwrite; next work supplemental and separately authorized. Heartbeatpaused.

**CP-V105 SAVED FUSION DIAGNOSIS (2026-11-04; completed, causal test pending):**
User asked why fusion worsens. Read experiments/eth1_fusion_diagnosis_cpv105_20261104/
REPORT.txt,localization.json,belief_analysis.json,verification.json,closeout/complete.
No capture, receiver replay, EVO call, tuning, build, deletion or deployment.
CP95 historical OFF versus corrected CP103 ON; not a new same-binary pair.
Existing common-mask ONLINE ATE.061504->.094335m and separate7s-finalized
.056428->.091336m: publication jitter does not explain all degradation.
69.43% of extra aligned ONLINE squared error lies before90s, not sparse tail.
Actual received/last-accepted full6x6 matrices,1454same physical Kimera intervals:
ON age2.45->4.85s mean translation trace falls80.76%, while relative IE-TC
RMS discrepancy5.131->5.142mm stays flat; reported norm1.929->.846mm.
Paired median ON/OFF trace first1.0126,last.2122; GLIM last1.0045.
Initial source-wire covariance alone misses this revised-belief tightening.
Reference discrepancy is NOT true NEES/NIS/calibration or trajectory0.5mRPE;
IE-TC sharesCPT7. Receiver configs/wrappers/adapter math match, timing similar,
frame/payload gates pass. Full matrices feed factors directly; separate factors
lack cross-source/overlap covariance. Correlated revised overconfidence is a
candidate mechanism, NOT demonstrated ATE causality or a qualified fix.
Proposed only: parity-first saved ON replay, then hold Kimera covariance at
first accepted matrix with mean revisions/order retained and frame checked.
Requires separate approval; no counterfactual optimization started.
Independent35392matrix checks pass;8120protected,CP104318/CP10362/CP95426
terminal outputs and Hesai667copies+originals preserved. CP105 sealed12outputs;
do not rerun or overwrite. Heartbeat remains paused.

**CP-V104 SAVED CORRECTED-ON ACCURACY (2026-11-04; completed):**
User authorized scoring saved CP103, not another capture. Read experiments/
eth1_prior_fix_evaluation_cpv104_20261104/INTERPRETATION.txt,RESULTS_REPORT.txt,
results.json,report_verification.json,coverage_review.json,closeout/complete.json.
Method1 EVO1.36.3 direct CLI54calls, SE3/no-scale, IE-TC at same calibrated prism
point; not Leica scoring. Eleven primary curves (CP95 OFF/ON+CP103 ON GLIM/
Kimera/ONLINE and CP91 source-only ON) share2966samples/296.3s/85.56%,365distance
0.5m RPE pairs. Three separate7s-finalized curves use same mask. No gap filling.
ATE CP95OFF->CP95ON->CP103ON: GLIM.039497->.053071->.038280m;
Kimera.737605->.198843->.192637; ONLINE.061504->.118626->.094335;
separate DELAYED.056428->.116118->.091336. CP91 ON same-mask GLIM.041514,
Kimera.209101; no CP91fusion. GLIM deterioration did not recur, but changed
backend/new realization cannot isolate fix causality or prove randomness.
ONLINE still worse historicalOFF; Kimera rotation RPE.527284->.658833deg worsens.
CP103 common paths GLIM202.743/Kimera202.069/ONLINE214.146/DELAYED201.970m
versus201.374m reference. Estimated polylines, not physical travel.
Full-saved CP103 ATE .039995/.192608/.095034/.091602, separate cadences/support;
CP95/91 full-saved archives reused unchanged. New mask omits3 CP103 Kimera
unsupported queries from old2969mask; ends301.744604s,44.575396s tail omitted.
All3454 first-published GLIM CSV poses scored; native shutdown dump has3455,
one extra final pose at346.219259s not in CSV. Do not substitute revised dump
means or call native-dump paths equal to prism-point/first-published scores.
Missingright203.898841154s,Kimera12gaps>0.5s,25pre-init/134expired fusion
versions preserved;0unsupportedEOF. No new covariance-calibration claim.
Final seal318outputs;8120pins,CP103all62/CP95all426/CP91all221 terminal outputs,
Hesai667copies+originals unchanged; both containers stopped, old identities
exited. No source/receiver replay/build/tuning/deletion/global deployment.
Do not rerun evaluator/verification/closeout or edit sealed CP104 evidence;
any later analysis is a fresh supplement. Scoped heartbeat remains paused.

**CP-V103 PRIOR-FIX LIVE QUALIFICATION (2026-11-04; PASSED ON-only):**
User authorized continued solution/validation while away, preserving settings
and old evidence. Read experiments/eth1_prior_fix_live_cpv103_20261103/PROTOCOL.txt,
PRELAUNCH_REVIEW.txt,PREPARATION_REPAIR.txt,REPORT.txt,copy_parity.json and current
prepared/launch/start/status/controller receipts. Fresh W/runs/same-name/prepared;
ON-only CLI/port11622. One full346.32s ETH1,offset0,0.20x,25sdrain, GLIM+Kimera+
concurrent CP-K19 fusion. Exact CP99 scientific settings; only private Kimera
985c115d correction from CP102 and failure-only archive differ. CBS/GLIM/receiver
DSOs, profiles/noise/priors/calibration/frames/weights/7sWNOA Qcx1 unchanged.
NOT identical-binary repeat/new matched OFF/ON. Do not repeat saved CP99 OFF.
CPV102_PRIOR_FAILURE_DIR only on Kimera -> source/prior_failures; guard stillfatal.
Initial host preflight missing OFF config descriptor was repaired before any
container/source launch; shared adapter fixture unchanged, OFF playback disabled.
Admission8120pins/Hesai667 and host fixtures pass. Started ONCE23:37:10local:
launcher104397exited,supervisor104398/start1346319,controller104400/start1346319.
Actual Noetic fixture and preplayback new DSO hash/maps/archive checks pass.
Completed00:08:14local: full346.32s input and all4source/payload/receiver/frame
gates pass,1492Kimera updates,1472persistent injections eachdirection,0failure
archives.202995fusion versions;4924inserts/197912replacements/25before-init/
134expired accounted,0unsupportedEOF/pending;3456ONLINE+3456separate7s-finalized.
Covariance payload error0; sparse Kimera5.10s gaps/2.42s tail and missingright
203.898841154s retained. No EVO score, new OFF or same-binary repeat claim.
Read new supplemental MORNING_REPORT.txt,qualification.json,complete.json and
post_closeout.json. Audit00:11:18local passes8120pins/62sealedoutputs/Hesai667,
all13owned identities exited,both containers restoredstopped. Do NOT rerun the
already-completed post_closeout.py or edit sealed REPORT.txt/terminal evidence.
Completed scoped heartbeat PAUSED; any future audit/report must be a new supplement.
No further capture/scoring/global deployment started; discuss next scope first.
Native-onlyFALSE, mixed-prior feedback and experimental Alpha calibration remain.
Even a full pass cannot recover CP99's unrecorded failed prior/theta or prove ATE
causality, covariance calibration, nonlinear stationarity or estimator health.

**CP-V102 PRIVATE BACKEND INTEGRATION (2026-11-03; qualified, not deployed globally):**
Read experiments/prior_anchor_integration_cpv102_20261103/REPORT.txt,complete.json.
Frozen CP76 baseline rebuild is byte-identical to successful b7d6829d DSO;
CP101 exact two-header correction integrated in new private Kimera985c115d DSO.
CBS82f85c2a reused byte-identical, GLIM/fusion/settings/guard unchanged. Both
baseline/candidate pass inherited CP76 synthetic compatibility; all48 logged
metrics exactly equal. Persistent6replacement/9absorption/9rollback checks pass.
Out-of-line backend recorder passes real marginal-prior/cache-corruption test:
3unique archives, all3exact matrices/typed-anchor readbacks, repeat nonoverwrite,
success no-write, disabled/invalid/relative roots retain original fatal exception,
native graph/locks/cache unchanged and persistent smoother transaction restored.
Configure CPV102_PRIOR_FAILURE_DIR to existing absolute directory for any new
validation; complete.marker last, native-x86_64 format, no durability guarantee.
Early test request rejected before launch by predecessor-exit guard; preserved
tests/PRELAUNCH_REJECTION.txt; authoritative tests_v2 passed after cleanup.
Final23:30:57local:7354pins/original6425,CP101/100/98/95/91/CP99failure,Hesai667
preserved; exact jobs exited,both containers stopped. No CP102 sensor run,
installation/global replacement, deletion or scoring. Actual CP99 failed prior
was not recorded: exact historic cause remains unproven; no GLIM ATE claim.
User subsequently authorized continued solution/validation while away. Next
fresh CP103 ON-only qualification may use corrected private DSO+failure recorder
with unchanged scientific settings; do not overwrite CP99 or repeat saved OFF.

**CP-V101 PRIVATE EXACT-ANCHOR CANDIDATE (2026-11-03; offline qualified, NOT deployed):**
User approved isolated correction/regression after CP100. Read experiments/
prior_anchor_candidate_cpv101_20261103/REPORT.txt,analysis.json,complete.json;
qualification_v3 is authoritative successful stage. Candidate headers only in
W/runs/same-name/candidate. Validated exact-anchor Gaussian clone; no epsilon,
anchor reset, guard/weight/settings change. Moved conversion code byte-identical;
both original/cache and converted/cache checks remain at1e-8, transaction intact.
1100cases: frozen126false rejections -> candidate0, coefficient error0; all1098
CP100 errors reproduced exactly.66660moved checks including1e-12 have matrix and
objective differences0;15negative controls reject with pointers/locks unchanged.
Four synthetic iSAM2 lifecycle checks pass15marginalizations each, finite C and
proper retained keys; baseline/candidate near-origin pose difference3.42e-49.
No actual CP99 failed matrix/theta was recorded; its cause remains unproven.
Separate failure summary errors/slot/keys and typed matrices/anchor implemented;
production matrix-export handler NOT installed. No estimator DSO rebuild,
replacement, live run, saved-trajectory optimization, settings change or deletion.
Initial raw-matrix equality assertion and synthetic fixture crash preserved;
baseline-only trace crashed before any marginal prior. Corrected ONLY harness
ordering to0oldest/1rest, with explicit leaf checks; absentGDB/inconclusiveASAN
receipts preserved too. Do not confuse root initial failed.json with finalv3.
Final22:46:26local:7052inputpins/original6425,CP100/98/95/91terminal evidence,
CP99failure,Hesai667copies/originals pass; ownedjobs exited,both containers stopped.
Next separate stage: private Kimera/CBS integration build plus typed failure
archive and bounded regressions before deployment; no automatic live ON/OFF.
Immutable CP101 evidence preserved; existing experiment heartbeat stayspaused.

**CP-V100 OFFLINE PRIOR CONVERSION (2026-11-03; numerical false rejection shown):**
User authorized the offline-first test after CP99 failure/settings audit. Read
experiments/prior_conversion_offline_cpv100_20261103/REPORT.txt,analysis.json,
complete.json. Two standalone diagnostic builds use exact frozen CP76 headers
and existing GTSAM; no estimator build, sensor/receiver/optimizer run, settings
change or deletion. Original/cache error exactly0 but finite-difference wrapped
prior error can exceed unchanged1e-8 guard. Synthetic SPD prior at last published
CP99 pose1475 gives1.50812974694e-8 and the original exception; exact unchanged-
anchor Gaussian clone gives0. This is NOT the unrecorded failed theta/prior.
1098cases including196initial and902nearby: original35saved CP98 priors allpass;
nearby translated saved0/738reject,synthetic56/164reject. Large-offset stress
cases separately recorded. Deliberate cache corruption still rejected; failure
leaves graph pointers/locks unchanged. No production patch applied. Actual CP99
failed branch/matrices were not logged, so its cause is not proven; do not label
this solved randomness, covariance calibration, or explanation of GLIM ATE.
Next proposed private fix: exact Gaussian clone at exactly unchanged anchor,
retain guard/original-cache check and moved-anchor behavior; separate failure
branch diagnostics and regression tests before deployment. No fresh live ON
authorized. Verification6642/6776inputpins,original6425,CP91/95/98 and CP99failure,
Hesai667copies/originals pass; owned jobs exited,both containers restoredstopped.
Immutable CP100 terminal evidence preserved; completion heartbeat remainspaused.

**CP-V99 IDENTICAL-SETTINGS REPEAT (2026-11-03; FAILED ON, FULL OFF SAVED):**
User requested one additional full ETH1 live OFF/ON pair with all3estimators
to assess repeatability before more debugging. Read experiments/
eth1_split_imu_repeat_cpv99_20261103/PROTOCOL.txt,PRELAUNCH_REVIEW.txt and current
prepared/launch/start/status/controller receipts. Fresh W/runs/same-name/prepared;
ports11611/11612. EXACT CP95 split-IMU bothpersistent settings,346.32s/offset0/
0.20x/25sdrain; GLIM Hesai+CPT7,Kimera stereo+Alpha with experimental biasRW,
CP-K19 7s WNOA Qcx1 concurrent fusion eacharm. No CP98 observer/seedchanges,
tuning/build/install/deletion. Source driver/profile/DSO/transport unchanged
except freshpaths/ports/sessionlabels; use corrected CP95continuation gates.
Admission6425protected/Hesai667/109staticfile parity passed; source schemas
CP89/91+14negatives and5synthetic EVO/report tests pass. Missing80KBfixture
restored before launch; preserve PREPARATION_REPAIR/initialparityreceipt.
Started ONCE20:10:06local:launcher19093exited,supervisor19094/start103915,
controller19096/start103916,captureOFF19803. Exact hostcommand/start verified.
Actualcontainer30fixturebitparity and sourcepreplaybackeffectivechecks pass;
Full346.32s OFF completed and all4gates passed20:40:50local. ON failed at
Kimera update1476/currentstate299.498319976s: `CPK9 promotion changes cached prior`.
See new FAILURE_REVIEW.txt and failure_post_closeout.json; originals untouched.
CP76 promote() checks original/wrapped augmented information against cached
Gaussian with normalized threshold1e-8; actualerror/slot/failedbranch unlogged.
Not yet distinguished numerical-tolerance failure from substantive inconsistency.
Source coverage assertion is secondary. No EVO/repeatcomparison/fullpair result.
Closeout21:07:04local; supplemental audit17ownedhostidentities exited,containers
restoredstopped,6425pins/426CP95/38CP98terminalfiles/Hesai667originals pass.
FullOFF and partialON preserved; no retry/scoring/tuning/deletion. Experiment
heartbeat PAUSED after failure notification; no further run authorized.
No duplicate/retry/extra sequence; preserve CP95/CP98 and all old evidence.
Cross-repeat common100ms mask must be separate from per-pair/common/fullnative;
ONLINE primary,DELAYED7s separate. Two realizations are not a variance estimate
or proof of randomness/no bug. Native-onlyFALSE/persistentcorrelation/Alpha
calibration/IE-TC limitations unchanged. Success-only post_closeout.py not run.

**CP-V98 NATIVE VISUAL GATING TEST (2026-11-03; MECHANISM SHOWN, ATE CAUSE OPEN):**
Read experiments/eth1_early_native_capture_cpv98_20261103/REPORT.txt,analysis.json,
verification.json,complete.json. User authorized one12s ETH1 source-only ON;
unchanged CP91 split-IMU/both-persistent profiles/private binaries. Existing
CP66 recorder enabled only in Kimera,168native snapshots/44updates,40insertions
each direction. No source retry/full run/fusion/tuning/estimator build/deletion.
Offline fixed SAME845smart-factor measurement set at7.899955s: CP91 poses give
16nonzero endpoint contributors,CP95 poses66(outof287touching);52OUTLIER->VALID,
2losevalidity. Globalvalid136->190; visualHessiantrace2.63143x. Native3pixel
MAX reprojection gate/zero-on-invalid makes information state-dependent even
with identical measurements. Historical measurement sets unavailable; not a
complete CP91/95 causal decomposition or a calibration/whole-run ATE proof.
Captured translationtrace5.2712e-7m2 vsold9.1424e-6/new6.5940e-7. OfflineLOCAL
full6x6 reproduced all23premarginal updates to2.56e-13relative. Cache reset at
sameeventchangesC0.04564%,max7.1573%over23updates;not order-of-magnitude cause
atcaptured event. Initial->optimized C not monotonically smaller. PIMbiasHat
exactlymatchesMAIN k-2 for21checkedintervals;frontendper-calltraceunavailable,
not labelledbug. Observer6.963s extra aggregate time; executionnotidentical.
All773admissionpins/CP91/95terminaloutputs/CP96/97/Hesai667preserved,containers
restoredstopped. Preserve prepassertion/offlinefixturecompile failures;analysis_v2
fixes only diagnostic constructor type. Nextcheck: newlyacceptedtrack/belief
consistency before changingweights/threshold. No furthercapture authorized here.

**CP-V97 EARLY KIMERA COVARIANCE DIAGNOSIS (2026-11-03; ATE CAUSE OPEN):**
Read experiments/eth1_kimera_early_covariance_cpv97_20261103/REPORT.txt,
analysis.json,source_review.json,closeout.json. Saved-only CP91/95 ON comparison.
First native covariance difference4.099975s; live LOCAL count differs4.499973s
(424/419); translationtrace first<half at5.499967s. First pose retirement8.499951s.
At7.899955s x22->x23: native LOCAL translationtrace9.1424e-6->6.5940e-7m2,
13.86x lower; transported13.58xlower. Explicit-peer-excluded graph audit matches
first exported full6x6 exactly; no marginal_prior yet. Global smart count827->886,
endpoint nonzero Hessian contributors20->70 (touching307->287), visual Hessian
trace2.8843x. Native IMU23/bias23/explicitpeer20 each; same initial prior traces.
Direct joint-information propagation matches within3.48e-14 relative atLOCAL
event;36755queries each allok/tmp_marginals/Schur/no jitter. Numerical parity is
not calibration/stationarity. LOCAL uses CBS-influenced current states; native
visual information can change even with explicit peer factors excluded.
Actual landmark measurements/triangulation reasons/frontend bias-used trace are
not saved: cannot isolate changed visual support versus linearization or prove
whole-run ATE causality. Event mean error improves too,not proof of overconfidence.
Pinned BPSAM confirms LOCAL count is nonnull live factors; source_review resolves
initial conservative graph-slot caveat in analysis.json without rewriting it.
Next proposal only: short~12s full graph/visual/bias passive capture then frozen
factor/state counterfactual. No capture/receiver/optimizer/tuning/build/deletion
launched.221CP91/426CP95 hashes and3CP96 manifests pass; preserve all evidence.

**CP-V96 SAVED GLIM ON REGRESSION DIAGNOSIS (2026-11-03; CAUSE OPEN):**
User requested investigating CP95ON ATE .053058m vsCP91ON .041500m rather than
calling it small randomness. Read experiments/eth1_glim_on_regression_cpv96_20261103/
REPORT.txt,analysis.json,supplement.json,covariance_timeline.json and verification
receipts. Same2969timestamps/365pairs; broad aligned shape change,not uniformly
worse local motion:medianAPE .034745->.052468m,max .135183->.092266m;0.5/5mRPE
better,10/20m worse,50m better(3pairs). No mask or scientific-setting change.
Actual1473K2G means slightly better on average,raw .007856->.007648m/ aligned
.009836->.009609m translationRMSE versusIE-TC at exact respectiveintervals.
IMPORTANT whole-run median covariance stability hides event-specific changes:
91matched translationtraces<half old,two<tenth. At5.699967s sameg46->48 trace
9.88xlower;7.899955s sameg68->70 trace13.58xlower. Incoming/inserted matrices
match exactly;no insertion multiplier bug demonstrated. Full-matrix generalized
ratios .030707–8.601074,median .998566;not proof of calibration or ATEcausality.
GLIM differs2.367576mm alreadyat.919669590s beforeCBS. FirstK2Graw mean/C exact;
firstG2K differs,returnedK2GC differsat4.099975s;large changesprecedefirstdifferent
binding8.299952s.15binding differences remain possible contributors,not proven.
HistoricalK16e sampler schedule-dependence remains relevant but doesnotexplain
THISATEdelta byproof. Next proposal: nativeKimera information/prior/visualsupport
over5–10s,then controlled frozen-input single-variable test if needed;no launch.
All221CP91/426CP95terminalhashes and29coreinputs pass,threeanalysisverification
receipts pass. No source/receiver/optimizer run,tuning,build,deletion or old
artifact edit. Preserve CP96 supplemental evidence; completionheartbeatpaused.

**CP-V95 AUTHORIZED SUPPLEMENTAL CONTINUATION (2026-11-03; FULL LIVE PAIR COMPLETE):**
User approved validator repair, saved OFF gates, then only the unstarted ON.
Read experiments/eth1_split_imu_live_pair_cpv95_20261103_continuation1/PROTOCOL.txt.
Reuse frozen attempt3/prepared and recorded full OFF unchanged; no OFF rerun.
Original failed controller/receipts remain immutable. Supplemental code/outputs
live in continuation1. No estimator/build/configuration changes or deletion.
Strict mode-specific injection schema passes full saved CP89OFF/CP91ON source
gates and14negative cases. Finish all saved OFF integrity gates before ON.
All saved OFF source/payload/receiver/frame gates passed:202999versions,
3456ONLINE,zero covariance payload error,137expired revisions accounted,
zero unsupportedEOF. Report parsing contracts and5synthetic EVO CLI tests pass.
Admission5849protected (original5744),Hesai667,32profile/buildfiles eacharm
unchanged;212291savedOFF regular files plus symlink/FIFO metadata tree-pinned.
223.04GBfree>15.51GBreserved. No changes to original recordings/failure artifacts.
Started ONCE15:59:56local:launcher171099exited,supervisor171100/start1698543,
controller171102/start1698544. Only previously-unstarted ON in originalP3
full_on_* and port11602. Container fixture and actual effective split-IMU,
bothpersistent/receivetrue gate passed before playback. Completed16:32:57local;
both fresh346.32s arms include live GLIM/Kimera/concurrentfusion. Original full
OFF reused unchanged; continuation captured only ON. No OFF rerun/receiver replay.
Authoritative continuation1/results.json,RESULTS_REPORT.txt,report_verification.json,
complete.json and supplemental post_closeout.json all pass.48direct EVO1.36.3 CLI
archives; Method1 first absolute publications,SE3/no-scale,reference-distance0.5mRPE.
Six-curve common100ms mask:2969rows,296.6/346.32s(85.643%),365RPEpairs,
reference201.3745m. OFF->ON ATE/tRPE/rRPE/path:
GLIM .039574->.053058m/.018377->.014303m/.352970->.329290deg/202.9502->202.4156m;
Kimera .738453->.198778m/.085253->.035072m/.619426->.527284deg/211.0250->201.8619m;
ONLINE .061500->.118581m/.037904->.035882m/1.823789->1.729998deg/216.1807->213.5990m.
ATE improvement GLIM-34.07%,Kimera+73.08%,ONLINE-92.82%; no universal gain.
DELAYED7s-finalized separately:ATE .056414->.116078,tRPE .013384->.011685,
rRPE .337977->.318803deg,path201.4344->201.8687m; not ONLINE/full-history.
Full-saved native ATE GLIM .038510->.051503,Kimera .744750->.198362,
ONLINE .059788->.113615,DELAYED .053513->.110691m; do not mix with common-mask.
Native rows3454GLIM/1493->1492Kimera/3456fusion;11Kimera gaps>0.5s retained,
common mask ends301.9446s,44.3754s tail unsupported. Missingright203.898841154s
unchanged; no gap filling. Full-native paths/reference/support details in report.
ON1473persistent insertions eachdirection. Receiver202999->202925versions,
197911->197837replacements,137->138expired revisions/25preinit each omitted,
zero unsupportedEOF/pending/duplicate;3456ONLINE+finalized eacharm.
Payload covariance error0; frame joins3452GLIM and1492->1491Kimera wiremessages,
1473ONreceive means eachside; Kimera wirecov relativeerror<=5.48e-16.
Kimera previous-state revision p95 .113872->.022887m,max .748554->.162806m;
>0.5m revisions3->0. Receiver support-cycle p95 .074191->.073848s at0.20x;
Kimera optimize p95 371.135->387.936ms. Incremental parity is not stationarity.
Wire median rotation/translation traces:GLIM9.550e-9->9.473e-9rad2/
3.439e-6->3.421e-6m2;Kimera6.953e-8->5.759e-8rad2/1.095e-5->1.235e-5m2.
Native Kimera all-version mediantranslation4.209e-6->2.045e-6m2;
transported4.263e-6->2.065e-6m2. Different populations,not matched-age proof
of overconfidence. GLIM native matrix parity separately unlogged. Persistent
peer correlation/native-only provenance FALSE and experimentalAlpha noise remain.
IE-TC sharesCPT7,not independentprismGT; estimatedpolyline is not physicaltravel.
Post-closeout verifies5849pins/original5744,212291OFFregularfiles+specialmetadata,
426terminaloutputs/45reportinputs,CP89all148/CP91all221/CP84all187,Hesai667copies/
originals and preserved failures. Exact owned jobs/controller/supervisor exited;
both originally-stopped containers restoredstopped. Completion heartbeat PAUSED.
Do not modify immutable terminal files,includingREPORT.txt,or original failure
evidence. Later audits need new supplements. No new experiment is authorized.

CP-V95 vs CP-V91 read-only repeatability check (2026-11-03): user queried
GLIM ON ATE .041500->.053058m. Saved common reference.tum and pair_indices.json
are byte-identical; all2969curve timestamps identical, OFFKimera TUM identical.
Actual ON16GLIM+14Kimera profile files and2privateKimera DSOs match byte-for-byte;
effective scientific parameters match and GLIM runtime private-library paths match.
Nevertheless GLIM OFF .038062->.039574m and ON estimates differ. ON native GLIM
already differs at second saved pose before first peer insertion. Both ON logs
contain1473insertions eachdirection, but15K2G sender intervals bind to different
GLIM state pairs; first source k24->k25 maps g73->g75(old) vs g72->g74(new),
~8.2s,~100ms shift. G2K endpoint IDs all match,5application-update headers differ.
This establishes non-identical execution,not the cause/size of the ATE change.
Concurrent fusion is downstream only; scheduling/load/numerical repeatability
remain hypotheses. No capture,rescoring,parameter change or terminal-artifact edit.

**CP-V95 FRESH ETH1 LIVE OFF/ON + FUSION (2026-11-03; ATTEMPT3 VALIDATOR FAILURE, OFF CAPTURE SAVED):**
User selected fresh full live all-three OFF/ON, not reconstructed CP94 replay.
Read experiments/eth1_split_imu_live_pair_cpv95_20261103/PROTOCOL.txt.
CP89/91 split-IMU profiles/private builds unchanged; BOTH persistent in ON,
both receivefalse in OFF.346.32s offset0 at0.20x/25sdrain. Receiver CP-K19
7s source_wnoa,Qc x1,independent100ms supports,no central IMU/GNSS unchanged.
Qualify CP94 Alpha-native->CPT7 full mean/covariance conversion in actual
private live adapter,never double-transform wire beliefs. Preserve real arrivals,
full revisions,payload/coverage/numerical/storage guards and protected evidence.
Sequential OFF gates then ON; no retries/tuning/build/deletion/extra sequences.
Original startup14:38:26 stopped BEFORE any master/source/receiver because a
transient oh-my-zsh curl update tripped the immediate container-idle gate.
Both containers restored stopped,5285protected/Hesai667passed,exactsupervisor
126690/controller126692exited. Preserve original failed evidence,no raw capture.
Fresh H/P ends_attempt1;ports11597/11598,only bounded30s startup-idle wait added
(same allowed process set,curl not whitelisted),fresh outputs;no model change.
Attempt1 also stopped pre-master/source/receiver:Noetic lacked SciPy for private
transport geometry import.5432protected/Hesai667passed,bothcontainers restored,
supervisor131402/controller131404exited. Its pre-admission metadata-copy repair
is documented;original data unchanged. Preserve BOTH failed infrastructure roots.
Fresh H/P ends_attempt2,ports11599/11600:equivalent adjoint mean/fullcov conversion
uses fixed scalar sums,no SciPy,with full saved-version/wire and actual-container
fixture parity before any source/receiver. No estimator/model/weight changes.
Attempt2 passed Noetic fixtures but host receiver --check lacked NumPy; only
owned ROS master ran/exited0, no source/receiver started.5581pins/Hesai667passed,
containers stopped;supervisor135545/controller135547exited. Preserve all3failures.
Attempt3 fresh paths/ports11601/11602 uses standard-library-only frame adapter;
require exact host system-Python fixture/CLI imports plus actual Noetic fixture
before permitting source/receiver. Same equivalent frame math, no model changes.
Read attempt3/PROTOCOL.txt,PRELAUNCH_REVIEW.txt,REPORT.txt,prepared/status receipts.
Record Method1 EVO ATE/reference-distance0.5mRPE/path,full-native vs common-mask,
DELAYED separately,covariance/revisions/timing/omissions. Peer feedback remains
correlated; native-only information FALSE. Attempt3 admission passed5744protected,
Hesai667,all frozen profiles/builds,exact host CLI/fixture and Noetic fixture.
Started ONCE15:07:01local:launcher140870exited,supervisor140871/start1381076,
controller140873/start1381077. OFF actual sensor playback and GLIM/Kimera/fusion
verified; source effective split-IMU/receivefalse gate passed before playback.
Receiver is consuming BOTH native streams and publishing online poses; early
queue drained to0. This is ongoing, not a result. OFF gates must pass beforeON.
Controller owns all capture/validation/evaluation; do not duplicate stages or
modify frozen files. Completion-only heartbeat monitors attempt3 and pauses
after terminal notification. Prior3failures remain immutable. No further retry.
Terminal15:37:49:full346.32s OFF source+receiver capture exited0; source coverage
passed,all34618CPT7/69186Alpha/3463stereopairs accounted,3455GLIM/1493Kimera rows.
Receiver completed202999beliefversions(166221GLIM/36778Kimera),3456ONLINE poses,
duplicate0,maxqueue47. Full pair is NOT complete and no scientific scores exist.
validate_off_source.py line41 failed: external_injections=0 is an int in OFF,
but validator called .values(). This is a harness schema bug,not evidence of
an estimator crash. No ON launched; remaining payload/frame/receiver gates and
EVO did not run. Preserve saved OFF and failed.json/log/receipts unchanged.
Cleanup and independent read-only audit:5744pins,CP89all148/CP91all221/CP84all187
terminal hashes,Hesai667copies/originals pass; exact owned jobs/supervisor exited,
both containers restored stopped. See attempt3/MONITOR_FAILURE_REVIEW.txt.
Heartbeat paused after failure notification. No repair/revalidation/replay made
by monitor. Proposed next step ONLY: supplemental validator schema repair on
saved OFF,remaining gates,then new ON continuation if authorized;no OFF rerun.

**CP-C7 PRE-REFRESH OUTPUT CLEANUP (2026-11-03; COMPLETE, 30 PERMISSION LEFTOVERS):**
User authorized deleting obsolete runs before the Kimera linearization refresh.
Boundary CP-V38/39/40; CP-V35+ fix evidence and all later runs are excluded.
Read experiments/pre_refresh_cleanup_cpc7_20261103/REPORT.txt,completion.json.
Removed10,292,155generated files/10,217,254empty dirs across214old roots;
file allocation170.059GiB,net filesystem gain208.704GiB,213.741GiBfree atclose.
Permanent deletion,notTrash/no newbackup. Compact reports/trajectories,all
datasets/Livox,code/build/runtime,current pins and later runs retained.
Checks:328905retained identities,7644dataset entries,6273protected hashes,
Hesai667copies/originals,CP89all148/CP91all221immutable terminal hashes passed.
First removal permission failure preserved; supplemental resume_permissions/
reconciled every prior absence and removed only remaining writable targets.
30files/131.082MiB remain unchanged,listed in BLOCKED_FILES.txt;no ownership/
permission changes or administrator workaround. all_planned_files_deleted=false.
ExactcleanupPID84337/start696018and4workers exited;host audit no visible
experiment/open old-run handles,both containers stopped(58processesinaccessible).
214root DECOMMISSIONED_CPC7.txt notices and checkpoint supplement supersede
old complete-bulk-retention claims; historical reports themselves unchanged.
Never restart either cleanup executor. No source/fusion/build/settings change.
CP-V94 storage shortfall resolved;its timing/method choice remains pending.

**CP-V94 FUSION REPLAY PREFLIGHT (2026-11-02/03; NO REPLAY LAUNCHED):**
User approved saved CP89OFF/CP91ON fusion-only replay with recorded timing.
Read experiments/eth1_split_imu_fusion_cpv94_20261102/REPORT.txt,preflight.json.
Standalone Alpha-native->CPT7 mean/full6x6 converter testedall36778OFF/36755ON
Kimera versions;actual same-version wire parity1492OFF/1491ON<=5.48e-16cov,
2.33e-16mean. Not installed/default-promoted. GLIMnative/wire alreadyCPT7.
Timing blocker: sources-only capture has no receiver events.csv/receipts.jsonl;
recorded CBS wire publications(allrev0,includingrepeatedintervals) omit most
full native revisions. Exact receiver arrivals/merged order were never recorded.
Cannot call invented times an exact causal replay. Need user choice before
fresh matched live source+fusion runs,approximate reconstructed-timing replay,
or native-wire-only changed input policy. Nothing launched.
Original storage shortfall7.123GiBfree vs10.787GiBreceiver-pairforecast+5GiB
reserve was resolved by separately authorized CP-C7 cleanup above;recheck
space before any future launch. Timing/method choice still requires direction.
No guard relaxation or archive suppression.148OFF+221ONterminal hashes intact.
Preflight serialization failure preserved;no source/receiver retry or results.

**CP-V93 ACTUAL INSERTED EXTERNAL COVARIANCE (2026-11-02; COMPLETE):**
User corrected focus to external factor matrices inserted on BOTH sides,not
CP92's later native revisions. Read experiments/eth1_inserted_covariance_cpv93_20261102/
REPORT.txt,analysis.json,verification.json and trace/ratio PNGs. SavedCP91ON/
CP89OFF only,no source/optimizer/container/build/tuning/deletion/rescoring.
All1473actual matrices each direction:finite/SPD,equalalignment/frozen/transport;
source wire->applied after body conversion maxrelative6.38e-16G2K/0K2G.
Scale1/healthawareOFF;40G2Kinformationinversechecks9.87e-15,noneavailableK2G.
No extra insertion shrinking found;this does NOT certify source calibration.
Actual medianrotation/translationG2K1.00866e-8rad2/3.59946e-6m2,K2G5.77873e-8/
1.20222e-5. Relative~0.20s motion,NOT global pose/trajectory covariance.
Matched1473G2K/1466K2GOFFexports (OFF has no insertions);6ambiguousOFFversions+
1missingK2G excluded only fromcomparison. Median ON/OFFrotation/translation
ratiosG2K.999472/1.007178,K2G.813271/1.019091. No universal collapse.13K2G
translationratios<.1 at16.6,183.6--184.5,256.6--259.3s;worst.024676at184.2196s
amidOFFspike. Smallertrace alone not overconfidence. Actualinsertions~3.8--302s,
no fabricated tail. Persistent correlation/mean-only timestamp covariance
approximation unresolved. CP92 native-revision conclusions are separate.
19input/148OFF+221ONterminal hashes unchanged;independent rawlog/Cholesky/trace
checks pass. Next possible saved diagnostic: actual aligned-belief errors in
listed intervals;no weights/new capture authorized by this pointer.

**CP-V92 SAVED ETH1 COVARIANCE AUDIT (2026-11-02; COMPLETE):**
Read experiments/eth1_covariance_audit_cpv92_20261102/REPORT.txt,analysis.json,
verification.json before interpreting covariance shrinkage in CP89OFF/CP91ON.
No source/optimizer/container/build/tuning/trajectory-rescoring; originals intact.
Matched wire3452GLIM/1473Kimera physical edges;fixed-age3411/1435same edges and
actual update times. Kimera17repeated wire edges each mode retained for parity,
first cohort deduplicated;162OFF/185ONunknown-age revision rows excluded from
age selection,not silently interpolated. Sparse tail has no new short beliefs.
First paired median covariance ON/OFF GLIMt/r1.00459/1.00134,Kimera1.01789/.81396.
At~4s GLIM1.00466/1.00132,Kimera.36995/.66020: no universal transmitted collapse.
Kimera ON same-cohort first->~4.2s mean translation variance-93.69%,squared
IE-TC residual-58.59%;ratio worsens6.56x. OFF-79.57%/-40.02%,2.94x. ON rotation
variance-43.73%,squared error+8.62%. Strong plateau~2.2->4.2s:ONtranslation
variance-68.37%,error2-0.020%,EVOrelativeRMS5.1150->5.1145mm. Revised covariance
looks overconfident versus IE-TC,not proof of exact true-covariance error or cause.
Nominal95%marginal inclusion mature KimeraOFF/ONtranslation23.14/11.50%,rotation
8.50/2.58%. Unknown IE-TC uncertainty/correlation and dependent edges mean these
are normalized discrepancies,not true NEES/NIS or calibrated scales. Relative
belief errors use EVO1.36.3;covariance whitening separate,not trajectory0.5mRPE.
13input/148OFF/221ONterminal hashes unchanged;independent whitening max3.4e-12.
Next proposed ONLY:localize mature covariance drop vs marginalization/peer-prior
absorption/provenance. Do not tune weights or launch another capture by default.

**CP-V91 FULL ETH1 SPLIT-IMU CBS-ON (2026-11-02; COMPLETE):**
User authorized CBS-ON with the CP89 setup. Read experiments/
eth1_split_imu_persistent_on_cpv91_20261102_attempt1/PROTOCOL.txt. One full346.32s
source-only live bidirectional capture: GLIM Hesai+CPT7/HG4930, Kimera stereo+
Alpha IMU, BOTH persistent. Preserve CP89 profiles/binaries/noise/calibration;
no fusion, tuning, builds, deletion, source retry or additional sequence.
Qualify existing nonidentity Alpha/CPT7 belief conversion before launch; same
physical prism for saved Method1 EVO comparison to CP89 OFF. Native-only peer
information provenance remains false; separate IMUs do not remove feedback.
Require unchanged10GiB initial/5GiB runtime reserve and protected/Hesai hashes.
One finite controller owns capture/gates/cleanup/report. Do not duplicate stages.
Original attempt stopped pre-playback18:18:00 due copied CBS_E1B_PROFILE_PATH
off_source rather than on_source. Cleanup18:18:30passed4789pins/Hesai667,
both containers restored;supervisor139781/controller139783 exited. No sensor
playback/recorders/native initialized states. Evidence preserved and pinned.
Fresh attempt1 corrects only diagnostic path and new paths/port11592. Launched
ONCE18:24:53local:launcher142174 exited,supervisor142175,controller142177
proc_start3120104. Admission4940protected/Hesai667,10.897GBfree>10GiB,
3.830GBforecast passed. Actual preplayback topics/body/TF/private DSOs and both
persistent receive=true passed before sensor playback.
Saved1492Kimera/3452GLIM payload means and same-version Kimera covariance frame
conversion passed preflight. Full capture/gates/frame audit/EVO completed18:55:39;
once-only supplemental post_closeout18:57:19passed. Exact owned jobs/supervisor
exited;both containers restored stopped;4940protected/221terminal/33report-input
hashes and Hesai667copies+originals pass. Attempt0 and CP89/90 pins unchanged.
Method1 EVO1.36.3 SE3/no-scale vs IE-TC,reference-distance0.5m RPE: common100ms
2969queries/365pairs,296.6s=85.643% support,reference201.374540m. GLIM OFF->ON:
ATE.038062->.041500m(-9.03%),t/rRPE.016331->.015195m/.336865->.333948deg,
path202.879510->202.624335m. Kimera ATE.738453->.209054m(+71.69%),
t/rRPE.085253->.032740m/.619426->.587296deg,path211.024963->202.001747m.
OFF is saved matched-settings CP89 rescored on the four-curve mask,not new OFF;
one fewer query than original CP89. Full-native support separately GLIM99.705%
both arms,Kimera85.816%OFF/85.787%ON,11gaps>0.5s each;no gap bridging.
Live1473persistent insertions each direction;all3452GLIM/1491Kimera wire means
match native endpoints;Kimera same-version full6x6 covariance relative error
4.70e-16. Incoming1473each raw mean checks pass;unlogged final native factor
covariances and mean-only timestamp-alignment approximation remain limitations.
Wire covariance median rotation/translation traces GLIM9.4246e-9->9.4730e-9rad2/
3.4415e-6->3.4563e-6m2,Kimera6.9534e-8->5.7549e-8rad2/1.09494e-5->1.15966e-5m2.
Kimera all-native-revision medians fall31.3%rotation/47.5%translation;do not
conflate these with published covariances or certify uncertainty calibration.
Kimera retained translation revisionp95/max.113872/.748554->.022934/.151270m;
optimizationmean/p95187.03/353.72->200.85/372.45ms,at0.20x,not1xqualification.
One missing right image203.898841154s remains;allraw34618CPT7/69186Alpha/
3462Hesai and3463stereo pairs accounted for. IE-TC sharesCPT7,not independent
prismGT;estimated polyline not physical travel;increment parity not stationarity.
Read RESULTS_REPORT.txt,results.json,frame_validation.json,post_closeout.json.
Completion heartbeat paused. No fusion,next sequence,retry or tuning authorized.

**CP-V90 ETH1 AP20 SAVED POSITION CHECK (2026-11-02; COMPLETE):**
User approved additional independent AP20 position evaluation of completed CP89.
Read experiments/eth1_ap20_saved_comparison_cpv90_20261102/PROTOCOL.txt.
Saved data only;no source/ROS/container launch,build,tuning,CBS/fusion or next
sequence. Preserve CP89 terminal outputs and IE-TC tables. Common raw AP20
timestamps,<=0.5s estimator interpolation,SE3/no-scale EVO1.36.3 APE only;
no orientation metric or gap filling. Include same-mask IE-TC reference check,
coverage/segmented path and immutable input/output hashes. No automatic rerun.
Completed saved-only with5EVOCLI/API-parity comparisons;34terminal files pinned
in complete.json. Same3873rawAP20 timestamps: GLIM ATE.031474m,Kimera.684862m;
same-mask IE-TC scores.032814/.686583m. IE-TC vsAP20.010988m. This gap persists
against external Leica positions;no proof of its cause or uncertainty calibration.
Common supported194.514120s/346.32=56.166008%,13segments;rawAP20 support65.924865%.
Common segmented paths AP20/GLIM/Kimera/IE152.455646/153.293176/156.854371/
152.383621m. Not full-route physical travel.674omitted rawrows:72startup,602tail;
Kimera sparse native tail gaps not bridged. Native3454GLIM/1493Kimera rows retain
99.705%/85.816% support,not AP20 coverage. No orientation/RPE fromAP20.
RawIE-prism reference reconstruction matches CP89 within1.5e-14m. Pre/post
3716protected/148terminal/28report-input hashes,Hesai667copies+originals pass;
both containers remain stopped. Original IE-TC tables unchanged. Read REPORT.txt,
results.json,verification.json,supplemental_verification.json. No new run authorized.

**CP-V89 FULL ETH1 ALPHA CBS-OFF (2026-11-02; COMPLETE):**
User approved one full346.32s validation after CP88,not more tuning/ON/fusion.
Read experiments/eth1_alpha_bias_rw_full_off_cpv89_20261102/PROTOCOL.txt,
PRELAUNCH_REVIEW.txt and receipts. All CP88 profiles/binaries/settings unchanged;
GLIM Hesai+CPT7 / Kimera Alphasense,biasRW.0043/.000266,bothreceiveOFF.
Offset0,0.20x,25sdrain;port11590,one exclusive controller,no duplicate stages.
Raw full stereo3464left/3463right,one missing right203.898841154s;3463matched
pairs agrees with historical CP-K23. Inputs unchanged; omission explicit.
Storage>10GiB and forecast+5GiB/runtime5GiB guards;no deletion or source retry.
Launched ONCE15:58:08local,supervisor45570/controller45572 proc_start2239673;
launcher45569 exited. Verify exact identities from launch/start/stage receipts.
Admission3716protected/Hesai667,12.417GBfree,3.545GBforecast passed. Startup
effective profile/body/topics/private-build checks passed BEFORE playback.
Completed16:28:43local; supplemental post_closeout16:29:51 passed. All owned
jobs/supervisor exited,both containers restored stopped;3716protected,
148terminal/28report-input hashes,Hesai667copies/originals passed. CP88 pinned.
Method1 EVO1.36.3 SE3/no-scale,reference-distance0.5m: common100ms mask2970rows,
296.7s/346.32s (85.672%),365pairs,reference201.375536m. GLIM ATE.038060m,
t/rRPE.016331m/.336865deg,path202.881834m;Kimera ATE.738754m,
t/rRPE.085253m/.619426deg,path211.027943m. Native-cadence saved GLIM3454poses
99.705% support:ATE.037891m,path205.236377vs201.475912mreference. Kimera1493
poses85.816% support:ATE.744750m,path214.718296vs201.020155m;11gaps>0.5s,
sparse ~5.1s keyframes after302s,not dense/full-time output. Do not bridge gaps.
Exact CP88 prefix561queries/46pairs reproduces ATE.144141m and all pilot scores;
full-window and pilot-only alignment differ. Kimera revisionp95/max.113872/
.748554m,3>0.5m,0>1m;optimizationmean/p95187.025/353.724ms. Logged36778
6x6covariances finite/symmetric/SPD,median rot/trans traces5.433613e-8rad2/
4.208529e-6m2;not uncertainty calibration. One early GLIM native pose is absent
from saved ROS CSV (3455native coverage vs3454saved);no source rerun.
3463stereo pairs=3preinit+3460processed+0EOF;raw unmatched-left retained.
Run/report pass is not estimator-health certification. No matched full CPT7
Kimera control,actual bias/residual history or qualified split-body CBS/fusion
adapter. IE-TC sharesCPT7,not independent prismGT;path not physical travel;
increment parity not nonlinear stationarity. Experiment-only heartbeat PAUSED
after terminal notification. No new run/default promotion. See mutable REPORT.txt
and immutable results.json/RESULTS_REPORT.txt/post_closeout.json.

**CP-V88 ETH1 ALPHA BIAS-RW ABLATION (2026-11-02; COMPLETE):**
User approved the one60s CBS-OFF experiment proposed by CP87. Read experiments/
eth1_alpha_bias_rw_off60_cpv88_20261102/PROTOCOL.txt and PRELAUNCH_REVIEW.txt.
Only Alpha accelerometer_random_walk=.0043 and gyroscope_random_walk=.000266
change from CP85; unchanged profiles/binaries/raw inputs/frames/white noise,
initializer/priors/rate/timing/backend. GLIM Hesai+CPT7 co-runs with receiveOFF;
Kimera Alpha receiveOFF; no fusion/ON/full sequence/sweep/build/default promotion.
Fresh roots/port11589,10GiB initial/5GiB runtime storage guards. No deletion.
One exclusive controller owns capture,gates,cleanup,EVO. Never duplicate stages.
Started ONCE15:24:18 local: supervisor37766,controller37768 proc_start2036654.
Completed15:30:56,post_closeout15:31:57 passed:all owned jobs exited,both
containers stopped,3368protected/104terminal/34report-input hashes,Hesai667.
Common Method1 EVO1.36.3:561queries/56s,46reference0.5m pairs,sameCP85/86mask.
Alpha ATE23.192259->0.144141m,tRPE13.281672->0.074174m,path1323.428703->26.190106m
vs25.304035mreference. RotRPE0.932099->1.002433deg did NOT improve. CPT7 control
ATE0.047226m remains better. Fresh GLIM ATE0.030953m with unchanged settings.
Alpha previous-state revisionp95/max37.122535/75.824811->0.054173/0.359724m;
zero>0.5m vs128before. All raw input stamps conserved;601=3+597+1EOFcamera.
Native Alpha284poses ATE0.144665m,path26.742070vs25.334488mreference,
56.399676s support;startup3.199979s gap not bridged. Full60s capture,not100%
scored coverage. Results/complete immutable; REPORT.txt explains all caveats.
Strong evidence of bias-model sensitivity,NOT certified Alpha calibration or
proof all timing/extrinsic/hardware issues excluded. CP85/86/87 remain immutable.
No defaults promoted. Next PROPOSED ONLY longer/full ETH1 CBS-OFF; no next run
authorized by this pointer. Split-body belief/fusion adapter still unqualified.

**CP-V87 SPLIT-IMU SAVED DIAGNOSIS (2026-11-02; ANALYSIS COMPLETE):**
Read experiments/split_imu_diagnostic_cpv87_20261102/REPORT.txt,analysis.json,
supplement_2.json and verification.json. No estimator run/build/parameter change
or rescoring. CP85/86 originals preserved. Alpha native Kimera failure remains
UNSOLVED; intended GLIM/HG4930 + Kimera/Alphasense is not campaign-qualified.
Correct actual Alpha subscription/body, mission-TF profile composition, input
units/header timing and noise mapping checked; no obvious axis/rate conversion
bug. Physical small timing/extrinsic errors NOT excluded. Zero CBS injections.
First Alpha previous-state revision>0.1m at18.899898s,>0.5m20.899888s,>1m22.899877s;
CPT7 never>0.1m. Native weighted IMU objective rises early6.5s; at20.9s14907/15300
total. Not physical residuals, identical-noise comparison or nonlinear stationarity.
Leading testable hypothesis: Alpha bias random walks constrain successive bias
changes9919x(accel)/20294x(gyro) more strongly in information than CPT7 control;
white measurement noise is larger, initial bias priors unchanged. Not proof of
wrong calibration or a hard lock of all bias values to initialization.
Initializer near-aligned identity branch even round0 affects both arms; raw
Alpha apparent-39.2m/s2 saturation follows initial divergence. Range unconfirmed;
supplement_2 supersedes broad signed16 quantization interpretation in supplement_1.
Lever-arm-corrected raw acceleration still disagrees; no fitted scale/time/frame
correction applied. Bias trajectories and individual physical residuals missing.
Next PROPOSED ONLY: one60s Alpha CBS-OFF bias-RW-only ablation to existing CPT7
values(.0043accel/.000266gyro), all other settings fixed. No run authorized by
this pointer; no full sequence/CBS/fusion or sweep. Split-body adapter still
requires separate qualification before live beliefs/fusion. Separate IMUs do
not remove peer-feedback correlation; IE-TC sharesCPT7,not independent prismGT.
Final verification PASS:39 analysis-manifest entries,268 CP85/104 CP86 immutable
terminal output hashes and all667 Hesai checkpoint copies/originals unchanged.

**CP-V86 ETH1 CPT7 CONTROL (2026-11-02; COMPLETE, CPT7 HEALTHY):**
User approved one matched60s CBS-OFF current-binary CPT7 Kimera control against
completed CP85 Alphasense. Read experiments/eth1_cpt7_control_off60_cpv86_20261102/
PROTOCOL.txt, PRELAUNCH_REVIEW.txt and receipts before acting. Same offset0,
0.20x/drain25s, CP82 GLIM and CP76/CP40 Kimera; no fusion/ON/full sequence.
Only CPT7-specific topic/body, camera T_BS and rate/noise restored from CP-K23.
All other profiles/binaries unchanged; no fitted2.4deg/3ms corrections applied.
Alpha bag still played/recorded for workload parity but not consumed by Kimera.
Single fresh capture; preserve CP85 and Hesai checkpoint. No automatic retry,
deletion, builds, tuning or next experiment. Both containers must be restored.
Current matched control tests integration bundle, not hardware quality alone.
Launched11:15:36local supervisor10073/controller10075(proc_start544448).
3017protected/Hesai667 admission PASS;13.066GBfree>10GiB, both initiallystopped.
Startup confirms current private binaries and both receive=false before playback.
Finite controller exclusively owns capture->coverage->cleanup->saved EVO; do not
duplicate stages or edit protected files. Read status/complete/failed receipts.
Completed11:22:13; post_closeout11:22:58PASS,3017pins/104outputs/29reportinputs/
Hesai667, all owned jobs exited, both containers restored stopped. Do not rerun.
Authority H/RESULTS_REPORT.txt,results.json,complete.json,post_closeout.json.
Method1 EVO identical CP85common100ms mask56.0s,561queries/46reference0.5m pairs:
Kimera CPT7/new vsAlpha/saved ATE.047226/23.192259m,translationRPE.025292/
13.281672m,rotationRPE.665586/.932099deg,path25.219278/1323.428703m versus
25.304035m reference. NewGLIM ATE.045248m/path25.788650m. CP85scores reused.
Full native Kimera ATE.047371m/path25.489573m vs25.334488m reference;284rows,
56.399676s no-gap support;startup3.199979s gap explicit, not input loss.
Previous-state revisionp95.017472m/max.078017m vsAlpha37.122535/75.824811m.
All601camera pairs conserved,zero CBS injections. EVOCLI/API error0.
Localizes failure to Alpha integration bundle/interaction, NOT hardware proven
bad or a specific extrinsic/noise/initialization cause. No next run authorized.

**CP-V85 ETH1 SPLIT-IMU PILOT (2026-11-01; FINISHED, KIMERA UNHEALTHY):**
User approved trying GLIM+CPT7 / Kimera+Alphasense. Read experiments/
eth1_split_imu_off60_cpv85_20261101/PROTOCOL.txt and REPORT.txt.
First gate is ONE60s source-only CBS-OFF capture at0.20x/offset0/drain25s,
current CP82/CP76-CP40 binaries, ETH1 profiles with author Alphasense extrinsics
and published Allan IMU parameters. No fusion/ON/full campaign, sweep or retry.
Separate raw IMUs do NOT remove peer-feedback correlation. Current fuser log
adapter needs explicit body conversion before use; never mislabel Alpha as CPT7.
Existing CP84 and Hesai checkpoint immutable; no deletion/default promotion.
Original launch18:57:09 failed BEFORE sensor playback: copied OFF selector
observe_only unsupported by GLIM shared_native provider. No sensor/quality result.
Supervisor210675/controller210678(start10058425) exited;19:02:24closeout passes
2544pins/Hesai667 and both containers restored stopped. Failed evidence retained.
Fresh experiments/eth1_split_imu_off60_cpv85_20261101_attempt1 restores intended
inject_persistent selector with BOTH receive=false; unique paths/port11587 only.
Profiles/binaries/weights/calibration unchanged; not a numerical/source retry.
Read its LAUNCH_REPAIR.txt and receipts. No automatic retries, ON/fusion/full run.
Attempt1 launched19:09:17local, supervisor223871/controller223873(start10131236),
2671protected pins/Hesai667 and12.60GBfree>10GiB guard passed. One actual60s
playback finished; containers restored19:15:45. Original frontend guard failed:
597processed vs598minimum. saved_report_1/source_coverage.json reconciles exactly
601received=3before initialization+597processed+1EOF camera1.4386ms after lastIMU.
Both IMU header streams match every raw sample; no source retry/guard erasure.
Final authority: *_attempt1/report_completion_1/RESULTS_REPORT.txt,results.json,
report_verification.json,post_closeout.json,complete.json. All12EVO CLI archives
scored once; inherited post-score diagnostic parser assumed CPT7, fixed only in
supplemental report by parsing actual Alphasense body; no CLI recoring/source rerun.
Method1 common100ms56.0s/60: GLIM/Kimera ATE.047968/23.192259m,
0.5m tRPE.032890/13.281672m,rRPE.545846/.932099deg, estimated paths
25.843993/1323.428703m vs25.304035m IE-TC. Full-saved native ATE.047135/
24.126003m, paths26.237749/1704.070396m,591/284rows; support58.999369/
56.399676s, startup Kimera3.199979s keyframe gap excluded from common mask.
Previous-state Kimera revisionp9537.122535m/max75.824811m; source/export parity
1.44e-15 and EVO CLI/API error0. This is NOT a healthy split-IMU baseline.
No CBS injections/ON/fusion/full run; diagnose native Alpha configuration first.
Separate raw IMUs do not prove independence; IE-TC sharesCPT7, not prismGT;
estimated polyline is not physical travel. Preserve all failed and final artifacts.
Initial prelaunch profiler-path correction also preserved before any source launch.

**CP-V84 BOTH-PERSISTENT FULL (2026-11-01; COMPLETE, ATTEMPT1 VERIFIED):**
Terminal authority: experiments/eig2_symmetric_persistent_full_cpv84_20261101_attempt1
and matching W/runs/.../prepared. Read RESULTS_REPORT.txt, results.json,
complete.json, report_verification.json and supplemental post_closeout.json.
One 355.02s live ON capture completed; launched 15:22:15, completed 15:54:07 local.
GLIM CP82 release_4 inject_persistent AND Kimera CP76 persistent, live
bidirectional exchange plus CP-K19 concurrent fusion, WNOA Qc x1.0 unchanged.
No fresh OFF control; not a new matched OFF/ON comparison or default promotion.
Method1 EVO common 100ms mask: 353.0s/355.02s (99.431%), 3532 queries,
349 reference-distance 0.5m RPE pairs; IE-TC reference path 188.013683m.
GLIM/Kimera/ONLINE: ATE .047487/.330296/.228121m; tRPE .017034/.037836/
.039718m; rRPE .633874/.845110/2.293056deg; paths 189.9177/188.3852/201.0671m.
Full-saved native cadence: ATE .047655/.330924/.229055m; no-gap paths
193.0726/189.8885/224.1500m vs reference 188.0976/186.6062/188.1258m;
3543/1761/3545 rows, supported 354.1993/353.7972/354.4s. Kimera has one
approximately 1s gap; common-mask and native sampling must not be interchanged.
DELAYED 7s fixed-lag-finalized: common ATE .226980m, path 187.0592m,
tRPE .016307m/rRPE .670052deg; native ATE .227062m/path 188.7955m.
Not ONLINE/full-history. Estimated polyline is not physical travel; IE-TC
shares CPT7 and is not independent prism GT. No nonlinear-stationarity claim.
1741 GLIM-to-Kimera and 1747 Kimera-to-GLIM injections; 213827 exact transported
versions, 5283 inserts/208493 replacements/50 pre-init exclusions/1 terminal
Kimera EOF omission/0 pending, stale, duplicate or expired receiver versions.
Kimera previous-state revision p95 .031962m/max .221563m, 1745 paired steps,
15 missing diagnostic step IDs; optimize mean249.298ms/p95467.336ms at0.20x.
Receiver support p95 73.217ms; receipt processing p95 21.591ms. Covariance
histories retained: median GLIM/Kimera rotation trace 6.74994e-9/1.10845e-8rad2,
translation trace 1.86421e-6/6.02616e-7m2. Mixed-prior Kimera outgoing information
remains user-accepted, NOT native-only, independent or calibrated uncertainty.
16:01:45 supplemental audit PASS: 2304 protected files, 187 terminal outputs,
75 report inputs, CP82/80/78 (176/192/218) and CP83 (8) outputs, Hesai all667.
All16 owned identities/commands exited, including controller177714 proc_start
8768965 and supervisor177712; both originally-stopped containers restored stopped.
Experiment-only heartbeat PAUSED. No further experiment authorized. Preserve
immutable terminal artifacts; post_closeout already exists, do not run it again.
Original CP84 startup15:18:44 failed before receiver/sensor capture: copied
wrapper mode664 instead of755. Only ROS master started/exited0; containers
restored and2220pins/Hesai667 passed. Original failure/logs/receipts preserved
and now pinned by attempt1. Fresh attempt restores executable permission and
fresh paths/port11585/session ONLY; no estimator/model/calibration change.
Use completed attempt1 receipts, not original failed status, for final evidence.
The following records describe preceding admission and cleanup, not active state:
Storage blocker resolved by specific user approval: pip cache1433files/~4.78GB
cleared,16.05GBfree afterward; installed packages/datasets/evidence untouched.
Fresh full harness under experiment/runs roots now preparing; no launch yet.
Conservative1.5x full output forecast,8GiBprelaunch/5GiBruntime reserves retained.
Read PROTOCOL.txt/PRELAUNCH_REVIEW.txt then require passed prepared.json.
The earlier blocked admission record follows for historical context:
User "Proceed to full" authorizes one355.02s EIG2Hesai live CBS-ON after CP83,
GLIM CP82 inject_persistent + Kimera CP76 persistent + concurrent CP-K19 WNOA
Qc x1.0, unchanged profiles/weights/calibration,0.20x/25sdrain. No new OFF,
other sequence, tuning/build/default promotion or deletion. Read experiments/
eig2_symmetric_persistent_full_cpv84_20261101/REPORT.txt. Host15:10:10 free11.29GB
is below existing full-run forecast12.64GB (CP49 bytes*1.25 +8GiB +512MiB).
Both containers stopped; no CP84 preparation/capture/controller or scoring
launched. Do not lower storage guard or delete pinned evidence. Potential
4.5GiB pip download cache identified read-only; subsequent approval resolved it.
Recheck capacity/full preflight before launch. Accepted mixed-prior limitation
unchanged. CP83 complete evidence and original reporting failures preserved.

**CP-V83 BOTH-PERSISTENT LIVE60 (2026-11-01; COMPLETE VIA SAVED-REPORT REPAIRS):**
User-approved conditional60s after CP82 private tests. Read experiments/
eig2_symmetric_persistent60_cpv83_20261101/PROTOCOL.txt,PRELAUNCH_REVIEW.txt,
prepared.json,status.json,launch/start receipts. Launched ONCE14:02:21local,
supervisor166061/controller166063, controller proc_start8289596. Do not duplicate
capture/validation/scoring, change settings, or edit protected files while active.
GLIM CP82 inject_persistent AND Kimera CP76 persistent, live bidirectional and
concurrent CP-K19 WNOA Qc x1.0. Same EIG2Hesai/Alphasense/CPT7 profile,60s after
offset5.7s,0.20x/25sdrain. Not full355s, not a fresh OFF/ON pair, not saved replay.
1989protected pins/Hesai667/preflight passed;12.06GBfree>9.84GBforecast. Finite
controller owns capture->gates->restore->EVO/report; fail closed/no retry. Both
containers initially stopped. Mixed-prior Kimera outgoing information limitation
remains useraccepted, NOT native-only or calibrated covariance. No full/campaign
or default promotion authorized. On terminal completion verify receipts/hashes,
owned jobs exit/containers stopped; preserve immutable output evidence.
Capture completed ONCE14:08:28; all60s,292G2K/293K2G persistent injections.
Original controller status remains FAILED solely because added GLIM injection
assertion read launcher/Kimera logs but omitted GLIM stdout. Do NOT rerun capture.
saved_validation_repair_1 added correct log in a new validator; all source and
receiver gates passed. EVO24CLI archives scored once, then report's inherited
>250s diagnostic failed on60s input. Both original failures are preserved.
Final authoritative results: report_completion_1/RESULTS_REPORT.txt,results.json,
complete.json,report_verification.json,post_closeout.json. Only inapplicable
>250s diagnostic omitted; existing24EVO archives independently verified, no
CLI re-scoring/source/receiver replay or guard relaxation. Method1 COMMON59.0s,
591queries/98.333%,reference34.754858m: GLIM/Kimera/ONLINE ATE.022821/.062271/
.059850m,0.5m tRPE.015044/.029827/.038867m,rRPE.790456/1.056253/2.534410deg,
paths35.0632/34.5645/37.0947m. Full-saved native ATE.023262/.062517/.063801m,
paths35.7594/34.8636/41.2382m vs34.8501/34.4625/34.9403m reference;592/298/595rows.
Keep native/common cadence distinct. DELAYED7s native ATE.053265m,path34.8718m,
notONLINE/fullhistory. Kimera previous-state revisionp95.028878m,max.084349m,
296pairedsteps/1missingID; optimize mean98.957ms,p95133.221ms at0.20x,notrealtime.
34552exact payload versions,884inserts/33618replacements/50preinit exclusions,
0EOFomissions/pending/expired. Covariance histories retained, calibration NOT
certified; mixed-prior limit unchanged. Host14:15:07audit:1989protected,151report
inputs,8terminalreport products,176CP82/192CP80/218CP78 outputs,Hesai667PASS;
16ownedprocess identities/commands absent,bothcontainers stopped. No full355s
sequence, fresh OFF control, automatic next campaign or default promotion.

**CP-V82 GLIM PERSISTENT RESTORE (2026-11-01; PRIVATE TESTS PASS):**
Read experiments/glim_persistent_compat_cpv82_20261101/REPORT.txt,verification.json,
complete.json. Existing persistent path restored for shared_native/native_local
by private opt-in CPV82_GLIM_PERSISTENT_COMPAT=1; no factor/covariance math,
weights, matching, calibration or default changes. Release_4 DSO aecc729f...e918.
29/29 release and UBSan tests; actual bridge insertion/dedup/retirement, native
history/covariance checks pass. Temporary mode exact original-DSO parity. GPU/IMU
72updates/13peer retirements/555queries:51,343records,2,590,537numeric entries
batching OFF/ON identical. No reset/fallback. Vendor libs not wholly instrumented,
no ASan/calibration/nonlinear-stationarity claim. Three harness failures retained
(link overlays, ROS node lifetime, missing fixture IMU); no sensor numerical retry.
1617protected/192CP80/218CP78 outputs/Hesai667 unchanged; containers stopped.
Conditional CP83 single60s bothpersistent live integration completed under
experiments/eig2_symmetric_persistent60_cpv83_20261101; read its protocol/receipts
before action. No full sequence/campaign/default promotion. Mixed-prior Kimera
information limit remains useraccepted, not repaired by this compatibility patch.

**CP-V81 SYMMETRIC PERSISTENT ADMISSION (2026-11-01; AUDIT COMPLETE, LIVE BLOCKED):**
User now requires persistent reception in BOTH GLIM and Kimera. Read experiments/
glim_persistent_admission_cpv81_20261101/PROTOCOL.txt,REPORT.txt,audit.json.
The conditional60s EIG-2 Hesai live test has NOT launched. Frozen CP-I1 GLIM's
constructor at cbs_bridge.cpp:685 rejects InjectPersistent with shared_native,
which is required by native_local revisions. Static source/build/binary evidence;
not a new runtime test or proof of numerical incompatibility. Persistent append
exists, native admission precedes peers, and old S4 wrapper tests covered retained
peers across retirement. Current bridge plus CP-K16 GPU integration still needs
private compatibility validation; do not bypass guard, change covariance routing,
use old binaries, or silently run asymmetric modes. Separate patch/tests decision
before conditional60s; no full campaign automatically authorized. No estimator
source/config/build/run/container changes.12:11:58 audit verifies1617protected,
192CP80terminal,218CP78terminal hashes and all667Hesai originals; containers stopped.
Audit evidence PASS, live admission FALSE. Keep CP80 accepted mixed-prior provenance
limitation explicit; it is NOT the reason this new configuration is blocked.
Historical follow-up confirmed user recollection: read CP81/HISTORICAL_COMPATIBILITY.txt
(CP81=experiments/glim_persistent_admission_cpv81_20261101). Sep21 EIG2 saved
inject_persistent/persistent completed305s, actual1519 K2G/1517 G2K rows; ETH1
and SPX3 also retain persistent settings. Exact Oct12 S4 provider build.py adds
the temporary-only guard; Oct19 I1 inherits it. Treat this as loss of supported
configuration in the newer private provider, NOT demonstrated persistent numeric
failure or need for a new CBS algorithm. Restore compatibility minimally after
focused tests; no implementation or run performed in this historical follow-up.

**CP-V80 FULL LIVE PERSISTENT (2026-11-01; COMPLETE/PASS WITH PROVENANCE LIMIT):**
User explicitly requested full live all estimators and accepted not excluding
absorbed peer information ("It's ok if we don't exclude it ... Just run it").
Read experiments/eig2_persistent_live_cpv80_20261101/PROTOCOL.txt,REPORT.txt,
PRELAUNCH_REVIEW.txt,prepared.json then launch/start/status receipts. One fresh
355.02s EIG-2 Hesai CBS-ON,0.20x/25sdrain: GLIM temporary, CP76 generalpersistent
Kimera, concurrent CP-K19 seven-second WNOA Qc x1.0. Actual live bidirectional
exchange; NOT CP78 saved-input replay, NOT FL4m RETAIN, no fresh OFF control.
CP79 admission blocker superseded ONLY by explicit accepted provenance scope:
Kimera mixed-prior LOCAL is NOT native-only; feedback/correlation may remain.
No weights/covariance/calibration/build/default changes.1617protected identities,
profiles/receiver exact,19.89GBfree, Hesai667 pass, bothcontainers stopped.
Finite controller owns capture->gates->cleanup->EVO->verification. Never duplicate
stages, edit protected files, rerun a numeric failure, or call a prefix full.
Launched ONCE02:23:06 local: supervisor128696/controller128698; launcher128695
exited normally. Actual startup gates passed before playback, all3 live and
both directions verified02:24:32; GLIM129798/Kimera129801/receiver129583 hostPIDs.
Closed02:54:46 local; full capture, route/lifecycle/receiver gates and24direct EVO
archives PASS. Read RESULTS_REPORT.txt,results.json,report_verification.json,
FINAL_INTERPRETATION.txt and supplemental post_closeout.json. Method1 full-saved
GLIM/Kimera/ONLINE ATE.048002/.321950/.212946m;0.5m tRPE.018141/.044498/.052010m,
rRPE.574969/.733095/3.151577deg; estimatedpaths193.1909/190.0336/224.3963m versus
native-cadence reference188.0976/186.6052/188.1258m. DELAYED7s-finalized separate:
ATE.211210m,path188.8831m,tRPE.016378m,rRPE.667703deg; notONLINE/fullhistory.
Common3526queries/352.3s(99.234%) ATE.047793/.321577/.212067m,paths189.9111/
188.5196/201.1290m vs188.0112m. Keep common/native masks and sampling distinct.
Kimera previous-state revisionp95.031687m,max.216949m,no>0.5m in1748pairedsteps;
13missingstepIDs. Mean optimize249.56ms,p95480.47ms at0.20x; no real-time claim.
Live1744injections each direction; no temporary Kimera retirements. All213753
transported payloads exact;5286inserts/208416replacements,50pre-init exclusions,
1terminal Kimera omission,0pending/stale/duplicate/expired. Not zero factor loss.
Independent host audit03:01:14:1617protected/192terminal/218CP78output hashes and
Hesai667 PASS; all20checked ownedprocesses absent,bothcontainers restored stopped.
Immutable complete/REPORT/results preserved; supplements only. Kimera live result
consistent with CP78 persistent replay, not a newly matched OFF/ON improvement.
ONLINE fusion still has19.28% native-path excess; do not call all issues solved.
Experiment-only heartbeat paused after terminal notification; no next experiment,
retry, tuning, build, or default promotion authorized by this completion.

**CP-V79 FULL LIVE ADMISSION REVIEW (2026-11-01; BLOCKED, NOT LAUNCHED):**
Historical review; user explicitly accepted its provenance limitation for CP80.
User requested full live all-estimator run after CP78. Read experiments/
persistent_live_preflight_cpv79_20261101/REPORT.txt. Intended EIG-2 Hesai
GLIM/Kimera/fusion; CP67 mixed-prior LOCAL provenance still fails with persistent
Kimera. CP76 compiled relative export still selects MAIN's LOCAL filter,
not the separate covariance sidecar. CP78 had no return consumer and did not
close this gate. No source/container/build/tuning/scoring change performed.
Need explicit scope decision for outgoing peer-free marginal-history repair
and tests, or a clearly qualified one-way live diagnostic. Do not silently
change direction, GLIM factor mode, weights or waive native-information claims.
All prior evidence preserved; no live controller or new monitor started.

**CP-V78 FULL NATIVE-MATCHING PERSISTENT (2026-11-01; COMPLETE/PASS):**
User explicitly approved full sequence after CP77. Read experiments/
persistent_native_full_cpv78_20261101/PROTOCOL.txt,PRELAUNCH_REVIEW.txt,REPORT.txt,
then launch.json/status.json once launched. Same matched Kimera-only temporary
vs generalpersistent, full355.02s EIG-2 Hesai, complete3541 CP49 raw input stream,
native matching/mean alignment, same CP76 private binary/weights/profile/0.20x.
No GLIM/fusion/return peer, forced CP70 IDs, CP73 waits or build/tuning/default.
Controller owns exclusive pair->gates->EVO->cleanup; stop on failure/no retry.
Prepared812pins/3541exact raw beliefs; launched ONCE00:45:57 local with sleep
inhibitorPID85846/controllerPID85848; closed01:47:00 local, no retry. Read
RESULTS_REPORT.txt,results.json,report_verification.json,FINAL_INTERPRETATION.txt
and post_closeout.json. Both full captures/gates and12EVOarchives PASS.
Full-saved Method1 EVO temporary->persistent ATE.887076->.319811m (63.95%better),
0.5m tRPE.340378->.043441m,rRPE2.116129->.716961deg; estimatedpath334.3011->
189.8771m vs186.6050/186.6120m reference at each native cadence. Common3531mask
ATE.882818->.319614m,path299.6635->188.4087m vs188.0133m;99.375%time coverage.
Native1761/1763poses; temporary0.6/1.0s gaps,persistent0.7s gap retained/excluded
from cross-gap scoring. All3541raw inputs exact,1741/1743native admissions;
1768/1767drops,32/29last retries,persistent2withoutmatchrow. Not zero loss.
Whole previous-pose revisionp95.429936->.031256m (92.73%less); final60s
.391833->.025961m. Optimize mean109.75->279.41ms (2.55x); no realtime claim.
812protected/218terminaloutput hashes PASS; Hesai667PASS; ownedjobs exited and
bothcontainers stopped, independently observed01:48:39. No stronger weights,
smoothing or refined-state scoring. Strong policy-comparison evidence, NOT a
retention-only isolation or new live bidirectional test; admissions/means evolve.
Preserve immutable complete/REPORT/results and CP77/earlier failures. CP67 mixed-
prior LOCAL provenance remains failed; no default promotion or automatic next run.

**CP-V77 NATIVE-MATCHING PERSISTENT60 (2026-11-01; COMPLETE/PASS):**
User approved fresh60s EIG-2 Hesai Kimera-only temporary/remove vs general
persistent. Read experiments/persistent_native60_cpv77_20261101/PROTOCOL.txt,
REPORT.txt,PRELAUNCH_REVIEW.txt,status.json once launched.590 complete raw CP49
beliefs, native matching and original mean alignment; no forced CP70 update IDs,
no CP73 readiness. Same CP76 private binary; no weights/build/default changes.
Exclusive controller owns pair/gates/EVO/cleanup. Never duplicate an active stage
or edit pinned files. Both CBS-ON inputs, not OFF/ON; no GLIM/fusion/return peer.
Saved ON feedback history and CP67 mixed-prior LOCAL limitation remain.
Launched ONCE00:27:12 local, controllerPID72061; closed00:38:57, no retry.
Read INTERPRETATION.txt,RESULTS_REPORT.txt,results.json,post_closeout.json.
Both298 identical native stamps/292 identical native admission IDs, updates and
endpoint times,590 exact raw inputs.272 native drops/24last retries/2received
without match row; no zero-loss admission claim. Full-saved Method1 EVO ATE
.093192->.057444m (38.36%better),0.5m tRPE.113062->.031230m,rRPE2.49514->.88971deg;
path41.41095->34.93514m vs34.46253reference. Common596 ATE.084683->.057071m.
Late revision p95.297709->.042279m (85.80%less); optimize mean74.84->110.67ms.
No stronger weights/smoothing. Supports retirement contribution, not isolated
retention-only/full/live proof; aligned means evolve with each arm's state.
601pins/212outputs/12EVOarchives/Hesai667 PASS; ownedjobs exited,bothcontainers
restored stopped, source trees clean. No automatic next/full run authorized.

**CP-V76 GENERAL PERSISTENT COMPATIBILITY (2026-11-01; PRIVATE TESTS PASS):**
Read experiments/persistent_compatibility_cpv76_20261101/REPORT.txt,
PROTOCOL.txt,build_fixture1/,verification_fixture1.json before a later trial.
Existing general persistent now supported by fresh private CPV76 opt-in with
corrected CP-K9/CP40; NOT active-window FL4m RETAIN or historical binary replay.
Native admission/replacement unchanged; mode-aware slot/payload/native-factor,
refresh and exactly-once marginal absorption checks; original rollback retained.
Release/ASan+UBSan/inherited corrected OFF/REMOVE/RETAIN and disabled exact
state/cache/tree parity PASS. Six replacement cases, nine marginal boundaries,
nine injected rollback cases; Schur1.503e-16, prior-function error0. Vendor DSOs
not instrumented; generic faults not exhaustive internal recovery certification.
Private libkimera b7d6829d...fa1cee8, unchanged copied libcbs82f85c2a...712476.
Initial test-only compilation failure preserved.41inputs/544CP73outputs/Hesai667
verify; production source clean, ownedjobs exited, bothcontainers stopped.
No sensor replay, GLIM/fusion run, weights/default change, deletion or push.
Implementation/testing complete. Proposed separate60s temporary vs persistent
must use complete original wire input/native matching, NOT accepted-only frozen
input or old forced update IDs. No returned peer consumer; CP67 mixed-prior LOCAL
limitation remains. No live-bidirectional/native-only/accuracy/full-run certificate.

**CP-V75 GENERAL PERSISTENT COMPATIBILITY (2026-10-31; AUDIT COMPLETE, NOT RUN-READY):**
Read experiments/persistent_mode_compatibility_cpv75_20261031/REPORT.txt,
audit.json and verification.json. Actual old general persistent selector exists,
but corrected CP73 CP-K9/CP40 requires ActiveWindowTemporary before first solve.
Changing only the flag is rejected; removing the enum check is insufficient:
post-refresh ownership assumes active-window maps absent in persistent mode.
Existing RETAIN tests use active-window params, not general persistent;
October15 persistent fixture predates the corrected backend. No mathematical
incompatibility claimed. Need private mode-aware compatibility + native tests,
not disabling corrections or guards; no implementation/build/replay authorized
by this pointer. Future native-matching test must use complete original wire
input, not CP73 accepted-only pre-aligned1743 of3541 messages (1798omitted).
Current persistent selector is not certified exact September binary behavior.
19static checks;25compiled-source identities/544CP73outputs/25inputs unchanged.
Source repos clean. No run/solver/build/container/settings/weights changes.
CP67 mixed-prior LOCAL limitation remains; no live/default/full-run promotion.

**CP-V74 RETAIN TIMESTAMP/RETRY DIAGNOSIS (2026-10-31; COMPLETE):**
Read experiments/retention_timestamp_diagnosis_cpv74_20261031/REPORT.txt,
analysis.json,verification.json before changing retention or using oldpersistent.
Saved-only CP73 audit: first RETAIN native stamp divergence1554/311.197445s
expected vs311.297445s actual; all prior stamps exact. Frozen3103->3105 at1555
fails100ms endpoint error versus50ms gate. It was released, NOT inserted;
native internal retry reenters same queue and triggers private gate's duplicate
already-released assertion at1556. Wire1551unique/exact,1549admissions exact;
NO duplicate wire/factor proved. FullREMOVE1762stamps/1743admissions exact.
Raw stereo has both expected and later frames; frontend selection is adaptive,
with backend bias feedback. Per-frame decisions/receipt unsaved: exact trigger
NOT proven. Frozen belief schedule did not freeze frontend keyframe decisions.
Do NOT relax matching or allow retry and call comparison controlled. CP73 stays
failed/unscored; CP70 short evidence unchanged. No newrun/build/solver/scoring,
settings/weights/production change.544CP73outputs/20inputs unchanged. This is
existing FL4m RETAIN plus new replay machinery, NOT old generalpersistent mode.
CP67 mixed-prior LOCAL provenance remains failed; no live/default promotion.

**CP-V73 READINESS-GATED REPLAY (2026-10-31; FAILED/CLOSED):**
User approved CP72 handshake and fresh conditional full comparison after gates.
Read experiments/retention_readiness_cpv73_20261031/PROTOCOL.txt,REPORT.txt,
build_initial/ and, once launched, status.json/launch.json/replay_attempt/.
Private opt-in2000ms real-queue readiness; original payload/update/timestamps
unchanged, all missing/late/duplicate/unknown/timeout guards fail closed. Release,
sanitizer, concurrent/negative fixtures and inherited native/disabled parity
precede full355.02s REMOVE then strict validation then RETAIN. CP71 frozen1743
inputs reused byte-exact; no new freeze/GLIM/fusion/returned-peer consumer.
Both arms ON inputs, NOT OFF/ON. Preserve CP71 failure and CP70 success. No
retry/tuning/default promotion; CP67 LOCAL provenance remains failed. Never
duplicate an active stage or edit its pinned files. New results pending.
All build gates passed; backend141474ec...8169d1. Prepared9558 pins verified.
Launched ONCE18:22:36 local: supervisor45229/controller45231. The finite
controller owns REMOVE->validation->RETAIN->validation->EVO->cleanup. Do not
start another capture/check/scoring stage while it runs. Consult status.json.
Closed19:19:41 FAILED at RETAIN update1556; last pose311.497443105s (1556poses).
REMOVE completed355.02s and strict1743-input/1762-pose validation passed.
RETAIN timestamp mismatch then duplicate internal-retry guard, diagnosedCP74;
no full pair/EVO scoring or automatic retry.9558pins/Hesai667PASS, cleanup and
integrity errors null, both containers restored stopped, ownedjobs exited.
Preserve original complete/status/REPORT and all pinned outputs unchanged.

**CP-V72 DELIVERY DIAGNOSIS (2026-10-31; COMPLETE, NO RETRY):**
Read experiments/retention_delivery_diagnosis_cpv72_20261031/REPORT.txt and
analysis.json. CP71 failure is replay synchronization, not retention failure:
missing685->687 was actually recorded byte-exact;341received,340admitted with
exact original IDs/covariance,346original native timestamps through345. It was
recorded10.546610ms after target346 camera stamp; gate holds early but cannot
wait for due callback/20Hz wrapper forwarding. Exact callback/flush wall times
not saved; do not claim precise delay attribution. Old FL4m450inputs all preceded
target stamp by>=41.026628ms, both exact-update validation gates genuinelypass.
CP70 short pair still valid, not scheduling-independent certification. Proposed
next ONLY: private bounded callback-readiness acknowledgement before solve,
preserve availability/payload/update/binding and fail on timeout. No arbitrary
sleep/backdating/skipping/guard relaxation. No implementation/build/replay done.
CP71 pinned outputs unchanged, source and containers untouched; CP67 separate.

**CP-V71 FULL FROZEN RETENTION (2026-10-31; FAILED/CLOSED, DO NOT RETRY):**
User chose full355.02s conditional test first, NOT live bidirectional replay.
Read experiments/retention_full_frozen_cpv71_20261031/PROTOCOL.txt,REPORT.txt,
status.json,launch.json and replay_attempt/*. Exact controllerPID32252,
suspend-inhibiting supervisor32250 launched ONCE16:52:16 local.
Full CP49 saved ON aligned stream frozen:1743 accepted beliefs, exact original
update/receiver timestamp joins to1762 original poses; original3541 wire beliefs
include1798 not in accepted set, listed explicitly. Original native gaps .6/1.0s
near tail preserved as coverage gates, not zero-loss claims.
9038 original/static pins verified; exact CP70 backend reused (6e620d0f...634ce),
no build/weights/production changes. REMOVE stopped69.599331232s/update346 on
CPV70_DELIVERY_FAILURE missing685->687. RETAIN NOT STARTED, no scoring/full
comparison.16:58:50closeout9041pins/Hesai667PASS; ownedjobs exited, bothcontainers
restored stopped. Read immutable status/complete receipts and separateCP72audit.
Fresh roots only; do not modify pinned scripts, launches or profiles. Failure
stops pipeline without retry. Both arms consume ON inputs, not OFF/ON. No live
GLIM/fusion/returned-peer consumer. CP67 LOCAL provenance failure remains.
No retries or completed full-result claim. Failed evidence preserved.

**CP-V70 RECORDED-UPDATE RETENTION TEST (2026-10-31; COMPLETE, SUPPORTED):**
Read experiments/retention_update_gate_cpv70_20261031/REPORT.txt,results.json,
report_verification.json,verification.json,complete.json. Fresh Kimera-only60s
REMOVE/RETAIN pair, same292 frozen CP66 ON beliefs; original update IDs,
physical timestamps/endpoints, payloads/covariance exact. No new peer feedback.
Both298poses, first30exact;268 retained marginal absorptions; all gates PASS.
Gate held sender405->407 from206 to207 in BOTH arms; CP69 failures preserved.
Method1 full-saved EVO ATE .097695->.062660m (35.86%better),0.5m tRPE
.111920->.033203m, rRPE2.403280->.854870deg; path41.7278->34.8232m vs
34.4625m reference. Common100ms ATE .088985->.062449m; keep masks separate.
Late40..60s previous-pose revision p95 .285475->.037104m (87%smaller),
max .639667->.113423m; native movement mean .092180->.091968m unchanged-ish.
Strong sustained evidence of retirement contribution here, NOT universal/full
mission fix. IMU cost not uniformly improved. Method1, not incremental/finalized.
Private delivery fixtures/Release/san, CP69 compatibility/regressions pass;
backend6e620d0f24aed055ee8f9f2f638af1b59881ff09e23ebe561614162a93d634ce.
No weights/model/production/default changes. Both arms CBS-ON inputs, NOT OFF/ON.
Conditional saved input includes original feedback history; IE-TC shares CPT7.
CP67 live bidirectional LOCAL provenance STILL FAILED: retained mixed marginal
history cannot be exported as native-only. Next requires separately authorized
provenance-safe MAIN/LOCAL design, no full/live bidirectional run launched.
Final preservation/owned jobs/both stopped containers/Hesai667 receipts retained.

**CP-V69 RETAIN COMPATIBILITY (2026-10-31; TESTS PASS, REPLAY GATE FAILED):**
Read experiments/retention_compatibility_cpv69_20261031/REPORT.txt and
replay_diagnosis.json. Existing FL4m retain privately supports corrected K9/CP40:
legacy/Release/sanitizer/exact-remove parity pass, nine retained boundaries,
dense Schur<=1.34e-16, continuity<=1.75e-18, immutable prior function exact;
native smart/slots/bounds/rollback preserved. No production/weights change.
60s frozen-input REMOVE completed298poses/292beliefs/268retirements, but one
belief405->407 on receiver204->205 entered update206(41.5996s) rather than
original207(41.7996s). Payload/order/endpoints/covariance exact;291/292 update
IDs match. STRICT replay timing gate FAILED; RETAIN NOT RUN, no EVO/pair result.
Initial equivalent-path preflight failure preserved; fresh pathcheck attempt
fixed only Path.resolve before first sensor replay. No retry after timing failure.
Proposed next ONLY: recorded-update delivery gate with causal availability,
fixtures then fresh bounded pair. No such change/run launched. CP67 bidirectional
LOCAL gate stays failed; retained prior is mixed, not native-only certification.
Final preservation/containers/Hesai667: verification.json,complete.json.

**CP-V68 EXISTING RETENTION RECONCILIATION (2026-10-31; COMPLETE AUDIT):**
Read experiments/retention_reconciliation_cpv68_20261031/REPORT.txt,audit.json,
complete.json before further retention work. Do NOT reinvent permanent factors:
cbs_odom_factor_mode=persistent and FL4M_RETIREMENT=retain already exist. FL4m
selector header byte-identical in CP66, scheduling branch unchanged; Oct15
94s ETH1 retain already reduced late instability on the pre-CP-K9/CP40 backend.
General persistent changes admission/replacement/recovery/force-solve too;
targeted FL4m retain preserves active-window admission. CP66 adds corrected-
backend immediate causal evidence; CP67 one-boundary/provenance, not new mode.
Current CP-K9/CP40 guards support remove only: need mode-aware compatibility
tests, not flag flip/guard removal/disabling the validated refresh. Reuse old
code/tests. Proposed next: private compatibility fixture, then separately
scoped60s EACH frozen-input Kimera-only remove/retain; no NEW return feedback,
but saved ON stream retains its original history. Not independent peer input,
live bidirectional safety or native-only LOCAL certification. CP67 failed gate
stays failed; no sidecar redesign required merely for this diagnostic question.
292 post-alignment rows and wire bag inventoried; joins/admission NOT validated.
17static checks/33pins; no native solve/build/source/covariance/production change.
Only audit documentation; no next replay/implementation authorized by pointer.

**CP-V67 MARGINALIZATION HANDOVER (2026-10-31; COMPLETE; LIVE GATE FAILED):**
experiments/eig2_marginal_handover_cpv67_20261031/REPORT.txt,verification.json,
complete.json. Saved CP66update250/50.4s only. Native Gaussian reconstruction
reproduces recorded postmarginalization H/RHS EXACT; survivor norm3.66e-17.
CP66retained B reproduced; oldfactor absorbed once; boundary survivorchange
3.96e-17, priorfunctions unchanged/22owned locks released in adapter copies.
NOT forward replay or safepermanentretention. Real BPSAM LOCALfixture excludes
explicitpeer exactly BEFORE absorption but retains its mixed-prior information
AFTER (normalized difference.342872); real common-chart peer-stripped probe
difference3.95349e-5. Relative CBS export uses MAIN smoother LOCALfilter, not
separate posecovariance sidecar. Currentfilter cannot subtract absorbedpeer.
Naive retain policy FAILS predeclaredprovenance gate; NO60s/forward/EVO run.
This is candidatepolicy risk, NOT proof currentremovebaseline already leaks.
Release/san fixtures;11independent denseSchur checks<=2.87e-15;9611pins and
Hesai667PASS; jobs exited/bothcontainers stopped. Initial harness FastMap
compilefailure preserved; syntaxonly freshattempt2 passes. No weight/covariance
scaling/production/defaultchange/deletion/push. Next design only proposed:
separate MAIN/LOCAL marginal histories or verified exclusion; evaluate existing
sidecar suitability, do not silently repurpose it or bypass CP-K9/CP40 guards.

**CP-V66 RETIREMENT CAUSAL TEST (2026-10-31; COMPLETE, MATERIAL EFFECT):**
experiments/eig2_retirement_capture_cpv66_20261031/REPORT.txt,complete.json,
capture_analysis.json,causal_analysis.json,verification.json. One matched60s
OFF/ON pair from original EIG-2 Hesai start; no source retries/settings changes.
Private passive prior serializer/observer: Release/san repeated-marginalization
nonmutation and disabledCP40parity PASS;848four-phase snapshots. Initial missing
dependency/fixture-presence-check build failures preserved; phase2passed.
60s reproduces ON.639667m previous-pose correction at59.39942s vs OFFmax.037687m.
Eleven native event2x2 tests: controls28,29; removals30..35; largestlate295,296,250.
88real Gaussian steps, exactlyordinary+refresh each; fresh factualH/RHS exact,
state reproduction<=3.45e-7/7.57e-7mixed norm. AllApass beforeB/C/D.
Oldremoved/newincluded A vs oldretained-one-update/newincluded B:
50.3995s .335969->.062328m (81.45%smaller previous-pose revision);
59.3994s .639667->.107762m (83.15%);59.5994s .449398->.019526m (95.66%).
Corresponding rotation3.88593->.52156 /2.61133->.64478 /2.37556->.20352deg.
Native within-solve movement nearlyunchanged. New-only omission does NOT remove
these big jumps; controlsno-removal exactzero. This is direct immediate causal
evidence for retirement at selected lateevents, not allzigzags/fulltrajectoryfix.
B/C stop before marginalization; each starts from ORIGINAL history. Never stitch
counterfactualstates or promote permanentretention. Nativecost/bias notuniformly
improved (295gyro-bias almostunchanged;250IMU/biascostworse). NoATE/RPEclaim,
weight/covariance/production/defaultchanges/deletion/push or next run authorized.
01:10:43final528independent scalar/quaternionchecks<=4.15e-14;11realarchive
roundtrips/33restoredbranchbytechecks,7754pins/Hesai667PASS. Ownedjobs exited,
bothcontainers stopped. Preserve original captures. Next only a proposed safe
lifecycle-design test avoiding abrupt loss AND unaccounted peer recirculation.

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
User explicitly parked suspect1 and authorized2; older do-not-advance notes
superseded, not universal exclusion of1. Original CP51update6 initial/refresh/
final geometry, noCP61candidate.1014smart factor-state instances:818accepted,
196exact-zero rejected;213missing-right rows exactlyzero. Native derivatives
vsFD at1e-5/1e-6 <=7.58e-9relative; independent analyticF/E<=9.37e-14,
RHS<=4.77e-11;whitening<=1.67e-16. Native H/g vs independentSVD/normal Schur
<=9.63e-10relative; all818landmark Jacobians rank3. Native unreduced b'b
constant difference explicitly accounted; not a pose-update error.
Original ordinary/exact-refresh matrices exact;state<=3.34e-10. Independently
assembled system produces same first increment to2.60e-9deg/2.93e-13m,
refresh<=1.78e-10deg/4.52e-14m. Two native saved linear solves, no forward
nonlinear replay. GlobalHcondition~1.006e10; stable backward residuals, NOT
nonlinear stationarity or physical observability proof. Release/san fixtures
and wrong-chain/missing-mask negatives pass. No tested visual math bug found;
not proof CBS healthy/later marginalization/nonlinear update correct.
Original analyze.py/partialanalysis.json retained after NumPy-bool JSON failure;
report-only analyze_v2 succeeds without formula/threshold or native rerun.
21:04closeout10711pins/Hesai667PASS,ownedjobs exited/bothcontainers stopped.
No source/fullrun/production/covariance/weight/default changes/deletion/push.
Next proposed ONLY: suspect3 directional factor/cost response along actual
rotation/bias update, fixed weights and explicit accepted-set accounting.
No suspect3 test launched or authorized by this pointer.

**CP-V61 TRACK-LOCAL TRIANGULATION (2026-10-30; COMPLETE, NO MATERIAL RESPONSE FIX):**
experiments/eig2_local_frame_cpv61_20261030/REPORT.txt,analysis.json,complete.json.
Private CP51update6 original vs first-left-camera-local native DLT vs shifted
candidate. Same cache freshness/guards/weights/Jacobians/Schur;6saved GNsteps.
2028cold coordinate comparisons pass: same statuses, point<=1.15e-12m,
Gaussian<=8.37e-10relative. Independent1014classifications/815points<=1.05e-12m,
440state metrics<=2.78e-16. Release/san rejection/cache/coordinate fixtures pass;
original equations exact/state<=3.34e-10. No failures/retries.
Ordinary A/B EXACTLY identical,285support,0candidate retriangulations. Refresh
338retriangulations but same266accepted tracks;7already-invalid labels change.
Net maxoldrotation .322580686->.322537187deg (0.0135%reduction),gyro-bias
.007460941->.007460878rad/s. Final A/B<=4.62846micrometers/.000122236deg.
Candidate translated-back B/C<=3.48e-13m/7.02e-10deg; origin dependence removed
for tested cases, NOT fullzigzag cause/fix. Large ordinary rotation precedes
refresh support loss. Original warm history preserved; cold initial282valid is
NOT used285support and candidate-from-startup history remains untested.
20:06closeout10641pins/Hesai667PASS,ownedjobs exited/bothcontainers stopped.
No source/fullrun/production/covariance changes, no promotion/deletion/push.
Suspect1 remains open; do not jump to2 or repeat origin shifts. No next test
authorized by this pointer; report historical-cache limitation before decision.

**CP-V60 WORLD-ORIGIN ABLATION (2026-10-30; COMPLETE, DEPENDENCE CONFIRMED):**
experiments/eig2_world_origin_cpv60_20261030/REPORT.txt,analysis.json,complete.json.
Same CP51update6 physical graph translated[3,-2,1]m; all poses/pose-prior means/
cached landmarks/cameras transformed, native cache rules/functions/weights intact.
1068coordinate controls pass: nonvisual equations<=1.65e-16, fixed-point all
equations<=9.31e-10 relative.3138independent geometry checks<=1.99e-15m;
1352post-step native classifications reproduce CP56;250state metrics<=2.22e-16.
A equations exact/state<=3.34e-10; four native GNsteps after Release/san fixtures.
Ordinary unchanged (A/Brotation<=4.75e-9deg),285visual factors both. Refresh
support266->257 solely from origin representation:6valid->rankfailure,
3valid->outlier,18total status changes at same physical postordinary state.
Final A/Bmaxpose difference.158546mm/.00565807deg; net oldrotation revision
.322581->.325879deg,gyrobias.00746094->.00749091rad/s. Real native triangulation/
rejection coordinate-consistency problem, NOT fullzigzag rootcause or frame/
extrinsic mistake; large initial disturbance remains. No productionfix/replay/
weights/guards/suspect2.19:33verification10552pins/Hesai667PASS,jobs exited,
bothcontainers stopped. Suspect1 remains open. Next only proposed: deterministic
track-local triangulation candidate with unchanged physical model and explicit
native tests, coordinate-invariance gates before saved-event response; not
implemented/authorized by this pointer. No favorable-origin tuning/fullrun.

**CP-V59 INITIAL CACHE571 ABLATION (2026-10-30; COMPLETE, NEGLIGIBLE EFFECT):**
experiments/eig2_initial_cache_cpv59_20261030/REPORT.txt,analysis.json,complete.json.
CP51update6 original ordinary+refresh vs identical graph with ONLY571cache cleared
once before ordinary. Native rules/functions/weights unchanged; no refinement.
Initial support285->284 (571valid->rank-degenerate); refresh BOTH266 and571valid.
Postordinary/final native classifications identical across338smart factors.
Net maxoldrotation.322581->.322388deg;gyro-bias.007460941->.007461002rad/s;
position5.721914->5.725811mm. Final A/B maxallstate5.173micrometers/.000218871deg.
Not a material explanation of first-event disturbance; no global cache all-clear.
A equations exact/state<=3.34e-10;355untouched Gaussians exact; restoring only571
recovers original snapshot bytes. Four real GNsteps, Release/san fixtures pass;
250independent scalar checks <=2.22e-16. Original exact-zero tangent harness
failure retained (zero saved solves): byte-identical inputs yield2.676e-19 group
roundoff. Fresh attempt_exact_values uses strict serialized identity, no guard
relaxation. Final19:20verification10432pins/Hesai667PASS; ownedjobs exited,
bothcontainers stopped. No source/covariance/production work or suspect2.
Suspect1 remains open: separate CP56world-origin sensitivity's causal influence
on graph response untested. Coordinate-equivalent A/B only proposed, not launched
or authorized by pointer; do not jump to another suspect or full replay.

**CP-V58 SUSPECT1 REFINEMENT-ONLY REFRESH (2026-10-30; COMPLETE, NO FIX):**
experiments/eig2_refinement_refresh_cpv58_20261030/REPORT.txt,analysis.json,
complete.json. Same CP51update6 postordinary state: original A refresh vs B
restoring only newly-outlier landmark fits that pass unchanged native rank/
depth/10m/3px gates.7of25restore(5geometries,allleft-only);266->273visual factors.
Aequations exact/state1.05e-10;349untouched Gaussians exact; restoring7caches
makes snapshot byte-identical. Native smart functions/elimination/priors/weights
unchanged.2real GNsteps, no source replay/production modification/covariance work.
Net old rotation maximum.322581->.324613deg;gyro-bias.00746094->.00746486rad/s;
position5.72191->5.33937mm, accel-bias.0228479->.0241224m/s2;IMUcost1.25375->1.38487.
Mixed small response, NOT cure/accuracy/root-cause proof;no full replay justified.
One-time point-cache counterfactual, not general triangulator or propagated history.
Release/san fixtures pass;19:05verification10347pins/Hesai667PASS,jobs exited,
bothcontainers stopped. Suspect1 remains open. Next only proposed: isolate initial
cache571 separately BEFORE ordinary solve; do not combine fixes or advance to2.
No additional graph/source test authorized by this pointer.

**CP-V57 SUSPECT1 LANDMARK FEASIBILITY (2026-10-30; DIAGNOSTIC COMPLETE, CAUSE OPEN):**
experiments/eig2_landmark_feasibility_cpv57_20261030/REPORT.txt, findings.json,
certificate_verification.json and complete.json. User explicitly requires staying
on suspect1; CP56's proposed move to suspect2 below is SUPERSEDED. CP51update6
saved geometry only,33factor instances x3states x3representations; no graph/source
run, covariance work, guards/weights/production change. Afterordinary25outlier
losses:13geometrically feasible/12numerically incompatible at unchanged3px;
original smart-L2 refinement recovers7instances(5distinct geometries). Final27
outliers:14feasible/13incompatible, plainL2recovers3plus2with10m-constrained fit.
Feasibility/minimax is NOT nativeL2 or a ready fix. Four/three additional rank
rejections remain excluded despite feasible points; no observability conclusion.
Pure coordinate transforms preserve geometric feasibility; chosen artificial
translation restores NONE of the actual post-step rejected factors to nativevalid.
Initial571cache effect remains unisolated; CP56 world-origin gate stays FAILED.
2544independent point checks/39native cost matches<=8.17e-13;75numerical dual
certificates/300necessary halfspaces verified. Boundary-search/consolidation
roundoff failures preserved; stricter inner witnesses, never relaxed acceptance.
18:53verification10308pins/Hesai667PASS; jobs exited/bothcontainers stopped.
Next ONLY proposed within suspect1: same-postordinary-state original vs isolated
landmark-refinement-before-rejection refresh, measure pose/bias response, retain
all guards; cache-only ablation separate, propagation needed before root-cause
claim. No new graph ablation/full rerun launched or authorized by this pointer.

**CP-V56 FIRST-INSERTION VISUAL SUPPORT (2026-10-30; DIAGNOSIS COMPLETE):**
experiments/eig2_visual_support_cpv56_20261030/REPORT.txt, analysis.json,
independent_dlt.json, verification.json and complete.json. Saved CP51 update6
only; zero source replay/graph solves/weight or production changes. Native
nonzero visual factors OFF285->285, ON285->266 at ordinary/refresh; final ON
post-evaluation267.25valid->outlier before refresh,27by final; max3px rejection
zeros whole track.16of27 lost tracks have full stereo,11left-only. Warm/cold
post-step ON support and equations agree; initial cache571 differs, not ruled out.
STRICT world-origin invariance gate FAILED, retained. Fresh measurement (not
relaxed acceptance) and rotation/translation decomposition: pure rotation zero
classification changes; artificial [3,-2,1]m world translation changes6/18/11
track statuses at initial/postordinary/final states. Native algebraic DLT/rank
sensitivity independently reproduced by NumPy:4056status matches,3677point
matches <=6.34e-11m. Not a wrong CBS body frame or proven full-zigzag cause.
Compile name collision/fixture-mode failure retained; Release+san fixtures pass.
18:24 verification10192pins/Hesai667 originals pass, owned stages exited, both
containers stopped. Next isolated suspect2 visual Jacobian/elimination check
proposed, not launched. No tuning, disabled guards, fix/promotion/deletion/push.

**CP-V55 FRAME/EXTRINSIC AUDIT (2026-10-30; COMPLETE, NO CBS FRAME FIX):**
experiments/eig2_frame_extrinsic_cpv55_20261030/REPORT.txt and complete.json.
EIG-2 author TF matches actual camera/Hesai/Velodyne profiles (max matrix3e-10);
both graph bodies CPT7, not camera vsLiDAR. Full Hesai1743G2K/1748K2G and
Velodyne1744G2K/1757K2G raw-payload joins pass; G2K aligned/frozen exact.
NativeCP51 OFF/ON291smart factors each match rectified-left extrinsic/calibration
exactly. Native1000nonidentity frame fixtures pass Release+ASan/UBSan, all wrong
inverse/missing/double/world-conversion negatives detected. No extra camera-LiDAR
transform justified. Last historical K2G native factor matrices absent; retain
that limitation and source/binary provenance qualifications. Side finding:
right-camera visualization metadata uses left body pose; correct left smart
factor/stereo baseline and CBS path unaffected, no production fix applied.
Separate GLIM mixed-OpenCV dependency observation is not a demonstrated cause.
Standalone build/reader failures preserved; no source run or weight changes.
17:34 verification9952protected/10037inputs/Hesai667originals PASS; containers
stopped, zero optimizer/source calls. No further capture authorized by pointer.

**CP-V54 GLIM FIRST-EDGE PASSIVE CAPTURE (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_glim_first_edge_cpv54_20261030/REPORT.txt,analysis.json,
verification.json,complete.json. One10s EIG-2 Hesai OFF/ON with unchanged CP51
settings; GLIM observer updates0..10,22readouts/mode, native bias/PIM/pre/post.
GLIM5->7 joint rotation vs OWN current-bias IMU prediction .119420degOFF/
.115709degON; native adjacent residuals .051560/.067944OFF,.046728/.069177ON.
Raw native RIGHT-support PIM reproduction <=1.67e-17deg, current-bias correction
vs reintegration <=3.37e-14deg. Bias choice changes gyro prediction .022960deg,
does not explain away mismatch. Raw gyro NOT GT; joint correction NOT standalone
LiDAR-only pose/residual. Does not prove GLIMwrong/codingbug/fullzigzagrootcause.
First ON admission still update6 at1.599933183s, GLIM5->7/Kimera4->5; prior K2G
x4->x6 already exists. Native export differs CP51 .001375/.000765degOFF/ON;
not bitwise replay. Kimera bias reference is CP51, explicitly qualified.
Private bridge d64bbb84f3bb2c7e...; no estimator/model/core change. Release/san
nonmutation/4-step parity pass; strict-zero-PIM equality and test Boostserialization
failures preserved, fixture-only repairs. All source/receiver/observer gates pass.
Final16:51:43:9952protected/9059retained/Hesai667originals pass, owned jobs exited,
both containers stopped. No fullrun/scoring/tuning/covariance study/promotion/push.
Remaining question: why Kimera amplifies a genuine small rotational conflict;
no further test/fix launched or authorized by this pointer.

**CP-V53 FIRST-EDGE RAW GYRO/ALIGNMENT AUDIT (2026-10-30; COMPLETE, CAUSE OPEN):**
experiments/eig2_first_rotation_cpv53_20261030/REPORT.txt,analysis.json,
support_audit.json; complete.json final authority. Saved CP-V51 first admission6
at1.599933s, GLIM5->7/Kimera x4->x5 only. Native/export/incoming/actualfactor
mean chain passes. Raw CPT7/native camera-border support reproduces saved PIM
rotation1.025e-17deg and native endpoint bridge6.441e-16deg; current-bias native
PIM vs reintegration1.509e-9deg.16.49/16.46ms endpoint offsets explicitlyhandled.
GLIM native vs same-time gyro with Kimera pre-update bias .099529deg ON/
.102394deg OFF; ON/OFF native mean difference.003943deg. AlignedGLIM-vs-Kimera
.099531deg; Kimera-vs-current-bias gyro .000003073deg. Mismatch already in
nativeGLIM, not created by checked transport/time bridge. Gyro NOTindependentGT;
GLIM contemporaneous bias/PIM absent, so no verdict GLIMwrong/codingbug/fullrootcause.
GLIM ON already had K2G x4->x6 before sending this edge; OFF mismatch alsoexists.
Header map is worldlabel, both configuredbodyframes CPT7. Four explanatorysource
hashes pinned; five not historicallysource-pinned, qualified in supportaudit.
Private saved-PIM extraction only, zero graphoptimizer/sourcecalls/covariancework;
Release+sanfixtures/100SO3fixtures pass. Three report-only failures preserved,
analyze_v4.py succeeds without changing numericalgates. Final16:05:21 checks:
9861protected/30analysis-support inputs/Hesai667originals pass; ownedjobs exited,
bothcontainers stopped. No productionfix/tuning/fullrun/deletion/push.
Next ONLY proposed: GLIM's contemporaneous bias/PIM and IMU-vs-LiDAR rotation
residual at sameedge, requiring short unchanged-settings passive observation if
approved; not launched. Do not substitute finalmap bias or infer calibratedconfidence.

**CP-V52 FIRST-ADMISSION ABLATION (2026-10-30; SAVED TEST COMPLETE):**
experiments/eig2_first_factor_cpv52_20261030/REPORT.txt,analysis.json,
visual_support.json; complete.json final closeout authority. CP-V51 update6 only,
1.599933s: same native graph/state with vs without exact CBS929 x4->x5. Four
GNsteps; A original ordinary/refresh reproduced <=3.34e-10 mixed tangent norm,
equations exact. Native355functions/initial state match actualOFF (smart caches
excluded from function comparison). B returns within.0665mm/.001272deg ofOFF.
Old-window tilt B/A .02019/.07859deg; maxoldrotation .02728/.32258deg;
gyro-bias revision .000343/.007461rad/s; IMUcost .05256/1.25375. Final cloned
smart-factor evaluation285valid B vs267A;27valid->outlier,3->degenerate,
12degenerate->valid. Not frontend feature-loss/exact live active-row counts.
First factor causally induces coupled response; NOT proof of wrong belief,
full-run root cause, nonlinear stationarity or a coding fix. No covariance work.
Initial compile name collision preserved; fresh attempt_compile1 repair only.
Release/sanfixtures pass; final15:36:07verification9084pins/732analysis inputs/
Hesai667originals PASS; owned jobs exited, both containers stopped. No source
run/tuning/default change. Next ONLY proposed: raw CPT7 gyro and native/aligned mean audit of this
one edge's rotation/time/body/bias semantics. Not launched.

**CP-V51 HESAI INSERTION PHASE OBSERVER (2026-10-30; COMPLETE, ROOT CAUSE OPEN):**
experiments/eig2_hesai_insertion_cpv51_20261030/REPORT.txt, analysis.json,
factor_audit.json,verification.json,complete.json. ONE10s OFF/ON with unchanged
CP-V49 settings; launcher112849/controller112856,14:46:18--14:50:39local.
Release+san BOTH-mode nonmutation/disabled parity passed;120native phase
snapshots6--25. OFF/ON windows1--5exact; first admission6/1.599933s, x4->x5.
Actual native means match frozen aligned matrices <=1.445e-13; one CBS factor
per observed ON update, noneOFF; no removal before26. Four native prior functions
(x0/v0/b0 plus zero-velocityv3) byte-identical across120snapshots. Native no-motion
x2->x3 is NOTCBS; old inspection family labels are qualified in factor_audit.
ON fresh H/RHS equal cache exactly in all60phases; tree H/RHS numerical parity,
independent native linear solution <=8.89e-9combined norm. Not stationarity.
At25/5.3999s ON ordinary common tilt2.7366deg, refresh.1070deg, net2.8245deg
vsOFF.3271deg; accel-bias change.47136m/s2 follows gravity-compensating direction.
Refresh reduces IMU weightedcost1873.29->319.70, OFFfinal7.84. Early response
predates marginalization26/5.5999s andCBSretirement30/6.3999s. No cache/removal/
prior-function bug established for this prefix; semantic frame/time remains open.
OFF920active-pose comparisons exact CP-V49; ON not exact (max.03163m/1.24129deg),
firstmean differs1.48mm/.00681deg. Early behavior reproduced, not fullzigzagscore.
Allgatespassed;7688pins/272CP49outputs/120snapshots/Hesai667originals verified;
jobs exited, both containers stopped. First supplemental factor-audit assumption
failed on nativev3prior; fresh saved-only v2 repair retained original failure.
No source retry/covariance study/tuning/fix/fullcampaign/push. Next ONLY proposed:
saved first-admission single-factor ablation with identical native graph/prior;
not launched. complete.json final authority over historical controller status.

**CP-V50 HESAI FIRST-ONSET DIAGNOSIS (2026-10-30; COMPLETE, ROOT CAUSE OPEN):**
experiments/eig2_hesai_onset_cpv50_20261030/REPORT.txt and verification.json.
Saved CP-V49 only: zero source/optimizer/build/covariance work or estimator changes.
Recorded OFF/ON active poses identical1--5; first admission/difference6 at1.59993s
after capture start (original bag+5.7s). First difference is not proof of a bug.
At25/5.39990s ON old-window tilt3.43465deg versus OFF.32711; native IMU costs
308.37versus7.84. First marginalization26/5.59990s; CBS removal30/6.39989s.
Retirement alone cannot explain early tilt. First>.5m revision295/59.39942s:
ON.63224m versus OFF.00419m; same-update motion.09351m but published step.62274m.
All69large ON revisions predominantly rigid (median99.99755% explained).
3541export/incoming means and1743aligned/frozen/pre/post joins pass; endpoint
offsets14.3--18.8ms explicitly bridged.1762-update ledger:1743insert/1720remove,
23final-active; counts/slots/edges consistent, no recovery. No blanket frame/
timing/cache correctness claim. CP-V49 lacks complete graph/theta/X/V/b/PIM
and intermediate whole-state snapshots; cannot isolate ordinary versus refresh.
42inputs/272CP49outputs/7593pins/88report inputs/Hesai667copied+originals pass;
100SE3fixtures;containers stopped/owned jobs exited. Next ONLY proposed:
short identical-settings observation prefix capturing first admission6 and
pre-marginalization21--25 phases, cached vs fresh equations before any fix.
No new capture/build/optimizer test launched or authorized by this pointer.

**CP-V49 FRESH CORRECTED HESAI EIG-2 PAIR (2026-10-30; COMPLETE, ON STILL UNHEALTHY):**
User explicitly requested full Hesai EIG-2 OFF/ON with all three estimators to
isolate ON behavior from the less accurate Velodyne source. New experiments/
eig2_hesai_refresh_full_cpv49_20261030/PROTOCOL.txt. One pair only, original
CP-K24 355.02s/offset5.7s profile and Qc x1.0, tested CP-V40 refresh in BOTH
modes. No CP-V44 observer/CP-V46 Jacobian/CP-V48 seed. OFF gates then saved
Method1 EVO/revision screen before ON; stop if >0.5m previous-state revisions
reappear. Original files immutable, no tuning/retries/builds/default promotion.
Check launch/start/status receipts before any action; never duplicate stages.
Low Hesai ATE is not proof of calibrated covariance. No covariance investigation.
Launched ONCE12:47:53local, launcher66944/controller66951; start12:48:01,
Both captures and gates complete;7593pins/bag hashes/Hesai667passed before launch;
33.0GiB free versus11.98GiB conservative pair forecast. Do not duplicate.
RESULTS_REPORT.txt/results.json and complete.json: final verification14:00:52.
Both containers restored/jobs exited;7593pins/88report inputs/Hesai667originals
pass. status.json is the earlier reporting-stage snapshot; complete.json is final.
Method1 common352.2s99.206%,reference188.0097m,348EVO0.5m RPE pairs:
GLIM ATE.046029->.048301m; Kimera.515214->.862981; fusionONLINE.086494->.179195.
Kimera common path186.9171->296.6668m; RPEtranslation.030823->.228248m.
OFF previous-state revisions>.5m0/1745; ON69/1747,max2.736418m. OFF fix
reproduces; ON remains poor despite low Hesai GLIM error. Velodyne degradation
is not necessary for this failure; root cause/covariance calibration NOT proved.
Full-saved paths/coverage and DELAYED output separate; ON1EOF omission/OFF0,
both50pre-init exclusions.48matched EVO CLI calls, parity0. No further run started.

**CP-V48 ROTATION-SEED TEST (2026-10-30; COMPLETE, CANDIDATE REJECTED):**
experiments/eig2_rotation_seed_cpv48_20261030/REPORT.txt,analysis.json,complete.json.
User authorized five saved CP-V44 windows21--25, original A versus only newest
orientation corrected using latest-bias IMU prediction B; stereo translation,
old states/velocity/bias/native observations/factors/weights unchanged. Original
ordinary AND chained refresh reproduced before any B. Native equations exact;
ordinary/final state error<=5.925e-8/5.880e-8 mixed-unit tangent norm.
Net old-window tilt A->B degrees:21 1.550510->1.550486;22 4.692479->4.691204;
23 2.540166->2.540160;24 9.508939->9.511847;25 9.350214->9.353689.
Initial rotation residual<=4.45e-16deg but large tilts remain. Final actual
B-vs-A old-state orientation<=.06084deg/position<=1.047mm. Candidate fails
predeclared severe-window improvement screen; no deployment/full rerun justified.
20native GN steps, five INDEPENDENT ON-conditioned windows, not propagated
sequence/accuracy/convergence proof. Earlier frontend matching/cumulative
history untested. No covariance work, source replay or production changes.
Release/sanitizer fixtures,10snapshot nonmutation,7519pins/Hesai667originals
pass;owned jobs exited/both containers restored. Root cause still open.

**CP-V47 EARLY ON STATE/IMU AUDIT (2026-10-30; COMPLETE, CAUSE STILL OPEN):**
experiments/eig2_early_consistency_cpv47_20261030/REPORT.txt,analysis.json,
early_analysis.json and complete.json. Saved-only CP-V42/44 diagnosis, zero
source/optimizer calls; no covariance investigation or production changes.
CP-V42 OFF/ON active poses identical1--5; first difference6 at first CBS
admission x4->x5, final IMU cost.05314 OFF/1.34648 ON. Full snapshots only21--25.
Old revised states/new bias/current-bias velocity prediction all pass. New pose
rotation instead equals raw PIM rotation at older biasHat to4.32e-15deg;
current-bias mismatch.03253--.12153deg, stereo/IMU position.01057--.06678m.
At22--25 biasHat equals previous k-2 published bias exactly. Normal integration
reference ordering; actual IMU factor/predict DOES correct bias. Mixed-reference
initialization is a lead, NOT proof of the~10deg tilt cause or a lost callback.
25PIMs/315repeated byte checks/455factor rows pass, deltaT error<=1.67e-16s.
No raw reintegration/early complete visual history; current frontend source is
separate code-inspection evidence. CP-V44 differs from CP-V42 at6; no exact
reproduction claim. API-only compile repair retained in fresh attempt_api1;
Release/sanitizer fixtures,15snapshot nonmutation,7406pins/Hesai667pass;
owned jobs exited/both containers restored. No new accuracy or fix claim.
Next proposed ONLY: saved initialization ablation changing just new pose
rotation to current-bias prediction, keeping stereo translation/native graph/
weights unchanged. Compare subsequent tilt, not initial-cost reduction alone;
reject if tilt persists. Not launched; no full rerun authorized by this pointer.

**CP-V46 CBS EQUATION ABLATION (2026-10-30; COMPLETE, NOT A LIVE FIX):**
experiments/eig2_cbs_ablation_cpv46_20261030/REPORT.txt,analysis.json,complete.json.
Five independent CP-V44 pre-ordinary checkpoints21--25; one GN step each of
A current CBS removed/B original/C exact CBS Jacobian. All B cached equations
exact and captured state<=5.925e-8 tangent norm. At25 tilt15.258/10.212/10.182deg;
C-vs-B common rotation<=.05081deg and old-position<=2.974mm across all5.
Jacobian-only candidate does NOT solve early tilt; no full rerun/promotion.
Removing CBS increases tilt magnitude/native remaining cost in all5, but A is
ON-conditioned, NOT a fresh OFF run or proof historical CBS is innocent.
Native smart/initial priors/weights unchanged.3420FD checks, Release/sanitizer
fixtures,5snapshot nonmutation,7296pins/Hesai667pass;containers restored/jobs
exited. No source replay, covariance work or trajectory accuracy claim.
Proposed next only: first ON tilt/bias transition and initialization/IMU bias
reference/visual-activity consistency audit; not a new run or authorized fix.

**CP-V45 ON CONVERGENCE ADMISSION (2026-10-30; SAVED DIAGNOSIS COMPLETE):**
experiments/eig2_on_convergence_cpv45_20261029/REPORT.txt,analysis.json and
complete.json. Five CP-V44 final graphs, unchanged native factors/weights.
All5fail exact cost-derivative gate; zero iterative optimizations. CBS native
BetweenFactor omits Local chain-rule Jacobian (95/95affected); exact chain
passes3420finite-difference checks. Stacked gradient discrepancy .137--.150%
only: confirmed approximation, NOT proven cause of large tilt or a live fix.
Initial pose-prior identity Jacobian and known smart cost/Schur mismatch also
fail; native IMU/bias/other priors pass.13fixed one-GN-direction fractions per
window; full step cost improves4/5but net tilt increases5/5 (24:9.51->9.81deg,
IMU6191.63->285.79). No convergence/accuracy/covariance claim. Release/ASan/
UBSan fixtures,5exact snapshot nonmutation,7090pins/Hesai667pass;containers
restored. No source run, covariance work, production patch or promotion.
Next proposed only: isolate exact CBS Jacobian in saved equation comparison,
same residual/measurement/weight and no simultaneous prior/smart change.

**CP-V44 ON PHASE CHECKPOINT (2026-10-29; COMPLETE, MECHANISM NOT ROOT-CAUSE FIX):**
experiments/eig2_on_checkpoint_cpv44_20261029/REPORT.txt,analysis.json and
gravity_coupling.json. ONE10s original-start ON observer run, frozen CP-V42/
CP-V40/Qc x1.0 settings; no covariance investigation.15full native snapshots
at21--25 before/after ordinary solve and added refresh. Ordinary tilt5.25/
5.68/10.21deg at22/24/25 accompanies median accel-bias changes.895/.973/1.750
m/s^2, matching gravity-projection compensation. Extra refresh mostly repairs
IMU residual but adds3.94deg tilt at24. Fresh/cache equations exact; independent
linear solutions reproduce states<=5.93e-8 tangent norm. Ordinary theta already
current: not the same stale-linearization mechanism as OFF in these windows.
Not nonlinear stationarity, a proven code bug, physical flight or a full result.
Native prefix differs from CP-V42 starting6 (before observer21); no retry.
48Kimera outputs/42G2K/43K2G, all capture gates pass; both containers restored.
6590pins/Hesai667 verified. Final preservation in final_verification.json.
Next proposed only: saved-graph nonlinear derivative/convergence diagnostic,
same native factors/priors/weights. No full rerun, tuning or default promotion.

**CP-V43 ON MECHANISM DIAGNOSIS (2026-10-29; SAVED-ONLY, CAUSE STILL OPEN):**
User excludes covariance work. experiments/eig2_on_mechanism_cpv43_20261029/
REPORT.txt and analysis.json trace CP-V42: ON tilt11.13deg by update25/5.4s,
before first marginalization26/peer retirement30.3540export/transport means and
1744exact endpoint joins pass; no inversion/frame/duplicate admission found.
Ordinary-vs-final peer residual medians barely change with extra refresh; missing
intermediate whole-state/IMU cost prevents phase attribution. Four same-graph
uphill updates, worst1016 objective621025->3423990 despite better peer fit.
Large revisions mostly rigid window motion (472/475events,median99.9703%).
No covariance calculations, weighting, source run, build, scoring or live fix.
Next proposed only: one~10s original-start ON observer capture at21--25 with
complete native graph/PIM/X/V/b and before/after both solves, unchanged settings.
Original CP-V42 artifacts immutable; no full rerun or default promotion.

**CP-V42 FULL CORRECTED EIG-2 PAIR (2026-10-29; COMPLETE, ON STILL UNHEALTHY):**
User renewed CBS-ON authorization after cleanup. New experiments/
eig2_refresh_full_cpv42_20261029/PROTOCOL.txt and REPORT.txt; never duplicate.
Full355.02s offset5.7s0.20x/25s, tested CP-V40 backend in BOTH OFF/ON modes,
original Qc x1.0/all other frozen settings. OFF gates precede ON. CP-V39 was
Kimera-only OFF, not the new matched three-estimator baseline. Fresh storage
forecast required; CP-V41's old shortage is superseded by CP-C5/6 cleanup.
Launched ONCE20:13local: launcher139842/controller139849; both exited after
all OFF/ON gates and cleanup21:16:56.6253pins/Hesai667pass, containers stopped.
RESULTS_REPORT.txt/results.json: direct EVO Method1 common ATE OFF->ON GLIM
.390391->.384648m; Kimera.515187->2.073638m; fusionONLINE.518175->.962769m.
Common paths204.14->204.20/186.92->791.88/209.46->212.92m; reference188.01m,
352.3s99.234%support/348reference0.5m RPE pairs.48EVO CLI calls/API parity0.
Full-native ONLINE path294.43->259.57m differs from interpolated common-grid
length; both retained, not physical travel distance. Delayed fusion separate.
Kimera revision max.288->7.271m; >.5m0->475; ON IMU cost spikes remain. OFF
improvement replicated, ON NOT fixed by this refresh. No cause inferred solely
from lifecycle pass. Proposed ON-specific saved diagnosis NOT started.
No retries/default promotion/tuning/deletion/other sequence.
Final complete.json verifies capture+EVO report,6253pins/88report inputs/Hesai667;
status.json retains the controller's earlier reporting-pending snapshot only.

**CP-C6 HISTORICAL RECEIVER CLEANUP (2026-10-29; COMPLETED):**
User approved pruning old blue-selected /results experiments. Of30explicit
visible roots,22had intermediate numeric snapshot/commit histories removed:
1,226,682files/1,190,714leaf directories,10.91GiB reclaimed;~37.61GiB free.
Read experiments/historical_receiver_cleanup_cpc6_20261029/REPORT.txt and
plan.json/completion.json for exact scope. Reports/final trajectories/code/
builds/tests/identified fixtures/current campaigns/Method1&2RRDs/all datasets
retained, including all9Livox bags.46,590retained entries/6,184current protected
hashes/Hesai667verify. No Trash/content backup. Historical exhaustive snapshot
audits are decommissioned; do not rerun old controllers to recreate them.
22roots have DECOMMISSIONED_CPC6.txt. Eight reviewed roots without recognized
snapshot histories were not pruned. Containers stayed stopped; no new run.

**CP-C5 TARGETED CLEANUP (2026-10-29; COMPLETED):**
User explicitly approved ONLY three September ETH1 presentation RRD exports
and the original GEODE Stairs bag; all nine Livox bags explicitly retained.
See experiments/targeted_cleanup_cpc5_20261029/REPORT.txt for exact paths.
Four files permanently removed,7.32GiB reclaimed,26.93GiB free immediately
afterward. Surrounding reports/run data/current Method1/2 RRDs/builds retained;
63 retained-entry metadata comparisons and Hesai667-file before/after checks
pass. No Trash/backup; GEODE reuse requires another copy or reacquisition.
This supersedes CP-C3/CP-C4's historical GEODE-bag retention statement.
CP-V41's earlier storage shortage is historical; recheck before any launch.
Cleanup itself launched no experiment and authorizes no additional deletion.

**CP-V41 BOUNDED EIG-2 ON INTEGRATION (2026-10-29; PASSED, FULL PAIR STORAGE-BLOCKED):**
Fresh experiments/eig2_on_refresh_smoke_cpv41_20261029/PROTOCOL.txt,
prepared.json/launch.json/start.json identify this30s original-start5.7s ON
engineering check. Private tested CP-V40 Kimera, original Velodyne/Alphasense/
CPT7/GLIM/7s WNOA Qc x1.0,0.20x/25s drain. Launcher PID120021; controller
and exact owned jobs in receipts; now all exited, both containers stopped.
REPORT.txt/complete.json/closeout.json: sensor/LOCAL/lifecycle/receiver gates
all pass;148Kimera/291GLIM/295ONLINE outputs,142G2K/143K2G injections,
118retirements,16249exact belief versions,50initial exclusions/0EOF omissions.
6184pins/Hesai667verify. No ATE/RPE or full ON accuracy claim. Do not rerun.
Full matched pair NOT launched:9.84GiB free versus11.33GiB forecast; even no
growth leaves7.34GiB before ON (<8GiB gate). Need>=2GiB more (3recommended)
or user-selected output filesystem. Do not delete evidence or relax guards.

**CP-V40 CBS-ON REFRESH ADMISSION (2026-10-29; FOCUSED GATES PASSED):**
Read experiments/kimera_on_incremental_refresh_cpv40_20261029/REPORT.txt,
PROTOCOL.txt and closeout.json. Private CPV40_REFRESH=1 extends CP-V38's native
refresh to temporary relative ON; original prior function/smart factors/weights/
libcbs unchanged. Existing residual/ownership guards rechecked, no duplicate
BPSAM processing, whole-transaction rollback. Disabled OFF/ON and enabled OFF
parity exact; deterministic Release, ASan/UBSan, nine-retirement lifecycle and
first/mature/post-marg rollback pass. LOCAL12x12 error<=2.619e-12; not nonlinear
stationarity or real-data ON evidence. Backend SHA750b8c8ffed61d186ec8f071d4f58f068c9b63baccefcc9fc37803d77a118f06.
Two failed fixture-stage attempts retained; solver/tolerances unchanged.
5966pins/Hesai667verify, jobs exited/containers stopped. User authorized bounded
ON then full matched EIG-2 only on safety/storage success. No source run yet,
no defaults promoted. CP-V38/39/originals immutable.

**CP-V39-VIS METHOD1 RERUN VIEW (2026-10-29; COMPLETE):**
New full EIG-2 OFF Kimera candidate/reference and original baseline overlay:
/home/yeranis/results/eig2_off_refresh_method1_cpv39_20261029/.
Portable eig2_off_refresh_method1.rrd, open_rerun.sh, README/report/verification.
1761absolute poses each,71004reference,3518447saved Velodyne map points;
exact common-mask EVO fits, native gaps, no smoothing. Every RRD vertex/edge
and input hash verifies; default preview inspected, normal viewer opened.
Screenshot-only shutdown102 retained separately. No estimator/scoring rerun,
new GLIM/fusion/ON result or changes to completed evidence.

**CP-V39 FULL EIG-2 OFF REFRESH (2026-10-29; POSITIVE, PRIVATE ONLY):**
Read experiments/eig2_off_refresh_full_cpv39_20261029/REPORT.txt, results.json,
REPORTING_REVIEW.txt and final closeout.json. ONE full355.02s original-start5.7s
0.20x Kimera-only CBS-OFF capture with CP-V38 refresh; no GLIM/fusion/ON/weights.
1761native outputs, complete camera/CPT7 receipts; actual private DSO/env passed.
Strict timestamp parity FAILED:126late IDs have changed stamps,76different
selected frames. Preserved failed reporting script; no source retry. Predeclared
same-support fallback excludes7old queries,352.3s/99.23% support;348reference
0.5m pairs (old349), gap rule unchanged. Both sides rescored directly with EVO.
Common ATE M1 2.309533->.515187m; M2 1.880779->.482241m. M1 path290.30->186.92m
(ref188.01m); full-native M1 path332.02->188.05m is a different cadence/product.
Translation RPE .452902->.030823m M1, .088765->.017921m M2; M2 rotation essentially
unchanged/slightly worse. Max prior-state revision4.85->.288m; >.5m112->0 observed.
Native IMU cost spike disappears; original-prior-only counters omit adapter,
NOT proof of removed prior. Retained-linearization mechanism strongly supported
in OUR modified baseline; no general upstream bug/ON/all-sequence certification.
No production/default promotion, source retries, deletion or push. Originally
stopped containers restored; final protected/Hesai/RRD verification in closeout.

**CP-V38 PRIVATE OFF INCREMENTAL REFRESH (2026-10-29; FOCUSED GATES PASSED):**
Read experiments/kimera_off_incremental_refresh_cpv38_20261029/REPORT.txt,
PROTOCOL.txt and final closeout.json. Isolated CP-K9-derived headers implement
ONE extra native current-state GN refresh before each OFF marginalization;
immutable original-prior function/noise/native smart factors retained. Explicit
CPV38_OFF_REFRESH=1 withCPK9=1/FL4Q0/remove; pending/incoming peers reject.
No source replay, CBS-ON, real-data trajectory result, weights or default change.
Disabled old/new pose/covariance dumps identical. Release twice deterministic,
ASan/UBSan pass; nine-retirement lifecycle, original prior energy0difference,
dense Schur~1.38e-14, LOCAL joint covariance<=6.25e-12, exact injected rollback.
Vendor DSOs uninstrumented. Small synthetic change is NOT accuracy improvement.
Private backend built/ldd-r passes, SHA560ad4e82323e91db43eae606bafcd3ecec6ccbd85dcf271fb84a7f50658ecf4.
Preparation/compile/test-accounting failures retained; candidate code and numeric
tolerances unchanged after first compile. Report explains adapter absorption.
Both containers restored stopped. Original5613pins/Hesai667/RRD verification in
closeout.json. Next bounded300s original-start EIG-2 Kimera-only OFF gate is
PROPOSED ONLY; no replay launch authorized by this pointer. No production fix.

**CP-V37 EIG-2 OFF NATIVE REFRESH (2026-10-29; COMPLETE, NO LIVE FIX):**
Read experiments/eig2_native_refresh_cpv37_20261029/REPORT.txt, results.json and
closeout.json. Seven existing CP-V36 checkpoints only; ONE undamped current-
state native-factor step each, original prior function/anchor/weights preserved.
No explicit landmarks, damping, nonlinear iterations, source run or CBS-ON.
All native smart factors/guards retained. All seven IMU position residual maxima
improve; peak267.8s .827053->.027510m,268.0s .983587->.061230m, NOT ATE/RPE.
Re-solving old equations leaves behavior unchanged (maxpose difference36.7um).
42prior-function checks, Release/sanitizer native fixtures, exact saved-input
nonmutation pass. Native smart validity changes0--71; visual cost worsens4/7.
Late corrections up to3.175m/19.10deg; residuals remain, no nonlinear stationarity
or trajectory fix. Retained-linearization hypothesis strongly supported, exact
live integration/origin open. Private incremental OFF integration/lifecycle tests
are proposed only, not implemented/launched. Initial unused-libcbs dependency
preflight failure preserved; continue_saved.py ran each never-started checkpoint
once with unchanged executable. Actual pinned GTSAM dependencies pass. Containers
restored stopped; originals/Hesai checkpoint/RRDs unchanged. No promotion/push.

**CP-V36 EIG-2 OFF COUNTERFACTUAL (2026-10-29; COMPLETE, NO LIVE FIX):**
Read experiments/eig2_counterfactual_cpv36_20261029/REPORT.txt,
capture_verification.json, diagnostic_results_v2.json, equation_audit.json and
closeout.json. ONE300s0.20x Kimera-only OFF replay:1498native pose/cost rows and
37100MAIN payload rows EXACT original parity. Seven complete-factor checkpoints
roundtrip with zero cost/linearization difference. Release/sanitizer observer
and offline fixtures pass. No GLIM/fusion/incoming CBS/ON/weights/live change.
Cache=fresh native equations at STORED theta; cache/tree H and rhs agree~1e-16,
scaled linear residual<1e-6. Yet nonlinear IMU residual is large:267.8s .827m.
Conditional joint-landmark comparator reduces that to1.05e-6m with unchanged
weights/original-chart prior, NOT trajectory RMSE. All8457 joint derivative
directions pass; native derivative failures map only to smart stereo. Of14
starts,5meet gradient gate,9hit30iterations; ALL peak starts hit cap. Native
activity changes0--275 and2--23 joint landmarks exceed native distance policy;
NOT an all-native-domain optimum, full trajectory repair or production fix.
Original-native reevaluated candidate costs also fall dramatically, but exact
cause remains open. CP-K9 prior promotion activates only on peer retirement,
not OFF. Next native-smart relinearization-only saved-checkpoint ablation is
PROPOSED, not executed/authorized by this pointer. No source/solver retries.
5131original/40private pins and667Hesai files pass; original RRDs preserved;
owned stages exited, containers restored stopped. Failed preflight/report
attempts retained; partial diagnostic_results.json is NOT authoritative.

**CP-V35 EIG-2 KIMERA OFF DIAGNOSIS (2026-10-29; COMPLETE, CAUSE/FIX OPEN):**
Read experiments/eig2_kimera_off_cpv35_20261029/REPORT.txt, verification.json,
diagnosis_summary.json, off_diagnosis_v2.png and closeout.json. ONE300s
Kimera-only original0.20x/start/profile replay completed; no GLIM/fusion/
incoming CBS/ON/tuning. All1498 native pose/cost rows and37100 MAIN revision
payload rows exactly match original OFF. Private observer release/sanitizer
gates pass;61updates/183stage snapshots. At267.8/268.0s native IMU position
residual maximum .827/.984m, NOT ATE. Large change occurs in solve, zero saved
surviving-state change at subsequent marginalization for all61inspected updates.
25/27pose anchors fixed near peak; fixed-anchor departure up to19.46deg at1336,
not fixed estimates. Retained-prior common-motion energy softens but no proof
that weak priors alone cause this or that tuning fixes it. Leading investigation:
linearization/chart/cache/one-step solver consistency in our modified baseline,
not verdict on untouched upstream Kimera. No production repair. Full nonlinear
counterfactual checkpoint/solve is only proposed, NOT launched or authorized.
Snapshots lack full factor/PIM/cache serialization. Original OFF objective.csv
IS Kimera FL4M (1761identical H/V rows); CP-V34 objective-absence wording corrected.
2871original pins/38private pins and667-file Hesai checkpoint pass; original
RRDs preserved. Owned jobs exited; both containers restored stopped. Two
rostopic drain-warning lines handled by strict report-only parser repair;
initial faulty preflight fixtures retained. No source retry or score change.

**CP-V34 EIG-2 KIMERA SHAKING DIAGNOSIS (2026-10-29; COMPLETE, CAUSE OPEN):**
Read experiments/eig2_kimera_jitter_cpv34_20261029/REPORT.txt, verification.json,
window_mechanism.png and closeout.json. Saved-data-only original CP-V4/Qc1.0.
Kimera OFF absolute arrays exactly equal for Hesai/Velodyne. VelodyneON spikes
predominantly coherent whole-window corrections: at219.798s previous pose moves
6.343m versus .164m same-update motion;25shared poses move almost rigidly,
max residual2.24cm. Not primarily scale pumping and not physical travel; OFF
also has real nonrigid deformation. Shared CPT7/cpt7_imu execution confirmed;
3540GLIM exports match incoming bag;1744V/1740H candidate transport/frame checks
pass. Endpoint offsets~41msV/~17msH; temporal mean corrected but covariance
unchanged, known approximation remains. No proven IMU/frame mismatch or exact
optimizer bug yet. Prior/global anchoring/linearization and tight peer-factor
interaction are next suspects; strong initial priors do not settle marginalized
prior behavior. Do not fix by changing WNOA or silently weakening constraints.
Proposed300s original-start Kimera-only diagnostic with saved peer input,
parity-first and staged prior/solver/IMU audits is NOT launched by this pointer.
All new mechanism checks are distinct from ATE/RPE; existing direct-evo CP-V32
scores retained. No source/receiver replay, build, tuning, deletion or promotion.

**CP-V33 ALL-SEQUENCE METHOD1 RERUN FILES (2026-10-29; COMPLETED):**
Seven portable RRDs in /home/yeranis/results/rerun_method1_all_sequences_20261029/,
with README/open_rerun.sh/SHA256SUMS. Read experiments/
all_method1_rerun_cpv33_20261029/REPORT.txt, verification.json and closeout.json.
Original CP-V27 seven OFF/ON pairs/Qc1.0: full first-published absolute GLIM,
recovered absolute Kimera BACKEND (not native incremental ROS), fusion ONLINE.
Exact CP-V32 evo SE3 fits; common ATE/0.5m RPE shown in results tab; full-native
paths separate. All113732native estimator points,17534190cloud points retained;
reference/cloud identical to CP-V30. No smoothing/thinning/integration or gap
bridging. All42common samples and evo error arrays exact; full paths reproduce.
Every RRD vertex/edge/cloud and132pins pass; all10Method2 output files unchanged;
Hesai667original files pass. Seven default OFF renders inspected; ON exact
readback, no separate interactive-tab audit. Known screenshot-only shutdown
panic102 after saved images preserved; all owned screenshot jobs exited.
236772334RRD bytes. No source/receiver run, build/container/tuning/default/
deletion/push. Method1 absolute publication jumps are not physical motion.

**CP-V32 ALL-SEQUENCE EVO TABLES (2026-10-29; COMPLETED):**
User requested0.5m distance RPE columns for all seven pairs, method1 and method2.
Read experiments/all_evo_tables_cpv32_20261029/REPORT.txt, METHOD1_TABLE.txt,
METHOD2_TABLE.txt, results.json and closeout.json. Both new42-row tables combined
use evo1.36.3 DIRECTLY for all84ATE and168RPE entries; original CP-V27/28 grids,
masks/prism poses/Qc1.0 retained. Whole-common-mask SE3/no-scale APE; RPE uses
reference-distance consecutive pairs with delta0.5m, gap segments separate,
actual step lengths disclosed. ETH1 CP-V31 RPE24entries reused unchanged.
Pair counts H ETH1/EIG2/SPX3/Leica2=365/348/167/378; V SPX3/EIG2/Leica2=167/349/378.
All372newCLI runs passed/API errors exactly equal;516newevoZIP readbacks pass.
225pins unchanged;84ATE old-value max difference1.159e-14m; paths exact; original
Hesai667files pass. Estimated path and ATE use common-mask coverage, not all
native timestamps; RPE subset counts/coverage explicit. Method1 Kimera is
absolute backend, not native incremental ROS; method2 uses native Kimera.
IE-TC shared CPT7, no independent GT or significance claim. All original reports
preserved. No source/receiver run, build, container, tuning/default/deletion/push.

**EVALUATION REQUIREMENT / CP-V31 ETH1 EVO RPE (2026-10-29; COMPLETED):**
User now requires reported accuracy metrics computed directly by evo, not
custom ATE/RPE formulas with cross-checks. Historical reports remain untouched
and must not be relabelled evo-only; no automatic all-sequence rescoring.
Read experiments/eth1_evo_rpe_distance_cpv31_20261029/REPORT.txt. Original Hesai
ETH1 OFF/ON, separate CP-V27 method1/CP-V28 method2,Qc1.0. evo1.36.3 CLI RPE
delta0.5m consecutive pairs selected on REFERENCE,365same pairs for all rows;
actual distances0.500561--0.593297m, unchanged common mask/no gap bridging.
Method2 translation OFF/ON metres: GLIM .020438/.018545, Kimera .330896/.016697,
fusion .064150/.039518; rotation degrees .370255/.360012, .448391/.461237,
2.375301/2.219676 respectively. Method1 Kimera absolute backend is NOT native
incremental ROS. All24CLI runs pass/API error arrays identical;48evo ZIPs,
39TUMs, pair IDs/config/commands retained.54pins unchanged; Hesai667files pass.
Only287.2s common segment produces distance pairs; full capture346.32s is NOT
fully scored. IE-TC shared-CPT7 reference, not independent GT. No source/receiver
run, build, tuning, container, deletion, default promotion or push.

**CP-V30 ALL-SEQUENCE METHOD2 RERUN FILES (2026-10-29; COMPLETED):**
User requested visual inspection before further receiver diagnosis. Seven
self-contained RRDs in /home/yeranis/results/rerun_method2_all_sequences_20261029/,
with README.txt/open_rerun.sh. Read experiments/all_method2_rerun_cpv30_20261029/
REPORT.txt and verification.json. Original CP-V28 seven OFF/ON pairs/Qc1.0,
not fresh CP-V29 OFF mixed with old ON. Four FULL method2/reference paths per
tab, same fixed common-mask SE3 fits, all113732native estimator points,
17534190saved ON cloud points. Grey corrected GLIM map uses original CP-V10
fit into shared ENU, NOT method2 fit; context only, not independent GT.
No smoothing/resampling/cropping; original publication gaps remain unconnected.
All42ATE/path reproductions, exact RRD vertices/edges/cloud and819pins pass;
Hesai667-file checkpoint intact. Seven OFF renders inspected; ON data exact
readback, no separate interactive-tab audit. Screenshot-only known shutdown
panic retained after saved images. No source/receiver replay/build/container/
tuning/default/deletion/push. Next proposed receiver diagnosis NOT started.

**CP-V29 FRESH ETH1 LIVE GLIM INCREMENTAL OUTPUT (2026-10-29; COMPLETED):**
Read experiments/eth1_live_incremental_cpv29_20261028/REPORT.txt,results.json,
verification.json and final_closeout.json. Single full346.32s Hesai CBS-OFF
capture, unchanged CP-K23/Qc1.0. Method2 full-saved ATE/path: GLIM .324791m/
204.521962m; native Kimera4.496229m/196.006964m; fusion2.383948m/199.278569m.
Old common-mask296.7s retained: ATE .338390/4.500022/2.488463m respectively.
Same-run GLIM absolute .046827m; live chain equals independent reconstructed
chain within2.193e-10, actual ROS equality exact. Relocating chaining into GLIM
does not restore absolute-pose accuracy. No output feedback/covariance invented.
All original source/receiver gates passed. Reporting precheck originally used
host instead of Humble compiler paths; same pinned replay later ran once in
Humble, all1678pins and byte/event/commit parity pass. Original strict scorer
then exposed2extra shutdown-only observer rows:3456paired vs3454ROS/bag IDs,
both GLIM topics0..3453, no interior omissions. Supplemental actual-ROS scoring
preserves these2rows separately; never labels them online. Full receipt audit
and failed scripts/logs retained. Original status.json remains historical FAILED
reporting; final_closeout.json is authoritative. No source retry/rebuild/tuning.
Seven output readbacks PASS;2499protected files/667Hesai checkpoint files intact.
Both containers stopped, owned jobs exited. No ON campaign/default promotion.

CP-V29 historical launch details:
Explicit user request for one from-scratch full ETH1 Hesai run. Read
experiments/eth1_live_incremental_cpv29_20261028/PROTOCOL.txt, REPORT.txt,
status.json and controller.log. ONE CBS-OFF capture, CP-K23 settings/Qc1.0,
0.20x346.32s/25sdrain, port11529. Private opt-in GLIM observer publishes an
additional causal same-update incremental PoseStamped/paired CSV; original
absolute odometry, graphs, beliefs and covariance remain unchanged. Normal
and sanitizer tests each pass11921checks/3454saved rows,maxerror1.927e-11.
Finite launch PID140594 under ETH1-CPV29 sleep inhibitor; controller receipts
identify owned processes. First tool approval timed out BEFORE execution;
launch/start/status were absent, then the single authorized retry launched.
Never duplicate run/stage. Original LOCAL/lifecycle/receiver gates precede one
CP-V23 saved-journal reporting-only observer replay with strict byte/event/
commit parity, then all-three method1/method2/full/common scoring. No ON run,
tuning/default promotion/deletion/push. Inspect actual status, not this label.
Restore originally stopped containers only after owned jobs exit; preserve all
old Hesai/GrandTour artifacts. Capture confirmed23:57local: effective settings
PASS, native GLIM/new incremental ROS rows matched, native Kimera and concurrent
fusion publishing, transport queued0. ControllerPID140596 owns later stages.

**CP-C4 NON-GRANDTOUR CLEANUP (2026-10-28; COMPLETED):**
Explicit user request removed obsolete S3E/Boreas/M3DGR/GEODE/Newer College
run/check outputs and recordings. Read experiments/non_grandtour_cleanup_cpc4_20261028/
REPORT.txt and both completion JSONs. 30,669 files permanently deleted;
15.44GB reclaimed;22.04GB available at closeout. No Trash/new bulk backup.
All3,922 protected GrandTour/reference/runtime files unchanged; Hesai667-file
checkpoint passes. Raw datasets, code, source/safety snapshots, shared/private
builds and every GrandTour experiment retained. Old stage source/report remnants
are intentional. Deleted generated M3DGR diagnostic bags are not original data.
CP-V29's storage blocker is cleared; cleanup itself launched no run/build.

**CP-V28 ALL SEVEN METHOD2 PAIRS (2026-10-28; COMPLETED):**
Read experiments/all_incremental_comparison_cpv28_20261028/REPORT.txt,
summary.json and closeout.json. This fulfills the explicit request for METHOD2
for all three estimators: accumulated SAME-update relative motions, used for
BOTH path and RMSE. GLIM recovered from saved MAIN active rows; Kimera native
incremental ROS CSV unchanged, never double-integrated; fusion twelve original
saved-journal observer replays plus reused CP-V23 Leica-2 Velodyne OFF/ON.
Original Qc1.0 (not0.25/.1), no source run/build/model change. All twelve replay
ONLINE/finalized outputs byte-identical, summaries/deterministic events and
publication commits identical. Read back92124chained poses;157host input pins
and1779replay/dependency pins verified. CP-V10 common masks retained; full-saved
own-cadence scores and1/10/20s RPE are separate. Common ATE winners: Kimera8/14,
GLIM6/14,fusion0/14. CBS improves common ATE GLIM2/7,Kimera6/7,fusion4/7.
Leica-2 Velodyne common OFF/ON: GLIM10.106881/3.331124m, Kimera1.094234/
2.585334m, fusion3.482370/5.331684m; full-saved fusion3.786712/6.080933m.
No interpretation as GLIM authors' standard output or equal latency/cadence.
IE-TC shares CPT7, not independent prism GT; original EOF omissions unchanged.
Storage-only stop after3successes preserved;9NEVER-started cases continued in
fresh RAM root with corrected output-filesystem/projected-capacity checks.
No duplicate replay or numerical/parity guard change. All new bulky evidence
retained in verified compressed archives; temporary duplicates removed only
after verified copies; staged products restored. Both containers exited, owned
jobs exited, four-sequence Hesai checkpoint PASS4cases/667files. No production
promotion. CP-V27 below is METHOD1, not the chosen method2 table; preserve both.

**CP-V27 SEVEN-PAIR NEWEST ABSOLUTE OUTPUTS (2026-10-28; COMPLETED):**
Read experiments/newest_absolute_comparison_cpv27_20261028/REPORT.txt and
summary.json. Four Hesai/three Velodyne OFF/ON pairs, saved data only,Qc1.0.
GLIM first-published/fusion ONLINE unchanged; recover21608Kimera newest
absolute backend poses, not its actual incremental ROS publication. Exact
CSV timestamps/keyframe IDs/every paired incremental pose verify; original
six-significant-digit graph pose rounding bound<=0.504mm disclosed.
All42old product full/common ATE values reproduce exactly; CP-V10 masks kept.
Absolute-policy common Kimera ATE OFF/ON: Hesai ETH1 4.209184/1.606029,
EIG-2 2.310051/1.101293,SPX-3 .361816/.221140,Leica-2 1.005786/.773176;
Velodyne SPX-3 .361645/.271351,EIG-2 2.307834/1.801042,Leica-2
1.005375/1.532446m. GLIM lowest13/14 modes,Kimera1,fusion0. Absolute
Kimera path can be hugely inflated (ETH1 ON782.32m common); connected
publication jumps, not physical movement or proof of internal trajectory.
Native incremental, newest absolute and delayed products must remain distinct;
one common rule is not equal latency/cadence/evidence age. CP-V26 chaining is
a separate output-policy ablation, not the authors' universal evaluation rule.
109input pins and independent geometry/evo/RPE pass; Hesai667files verify.
No rerun/replay/build/container/settings/default/deletion/commit/push.

**CP-V26 ALL-THREE METHOD2 LEICA-2 OUTPUTS (2026-10-28; COMPLETED):**
User selects incremental chaining as common comparison policy. Read
experiments/leica2_all_incremental_cpv26_20261028/REPORT.txt/results.json.
Saved GLIM same-update snapshots reconstruct4280poses/mode, exact publication
parity; retain native incremental Kimera/reuse CP-V23 fusion chains,Qc1.0.
Common ATE OFF/ON: GLIM10.106881/3.331124,Kimera1.094234/2.585334,
fusion3.482370/5.331684m. Full-saved GLIM11.931732/3.867642m vs original
absolute2.829284/.863203m. Thus GLIM chaining loses substantial global accuracy
here; Kimera lowest method2 ATE both modes. Not a universal sequence conclusion.
All-three publication rule now matched, not update cadence/evidence age/latency.
Original method1/refined scores remain separate valid product measurements.
Independent chain, exact source publication, geometry/ATE/pin checks pass;
no rerun/replay/build/container/settings/default/deletion/push. No new output
covariance/velocity assigned. Only Leica-2 Velodyne converted by this task;
other sequences need their own saved-data reconstruction/coverage verification.

**CP-V25 LEICA-2 OFF SOURCE-SUBSET RECEIVER CONTROLS (2026-10-27; COMPLETED):**
Read experiments/leica2_single_source_controls_cpv25_20261027/REPORT.txt,
results.json and closeout.json. One full saved GLIM-only and Kimera-only replay
each; original CP-K19/Qc1.0/7s/100ms. Not native estimators, no source run/build.
Common ATE nativeGLIM2.782169 /receiverGLIM-only2.807562m; nativeKimera1.094234
/receiverKimera-only1.323540m; BOTH2.006010m,51.56% worse than Kimera-only.
Single-source newest ONLINE rotation RPE/path inflated too; finalized repairs
local motion, not global ATE. Therefore combining sources worsens this OFF case,
but is not sole jitter cause. Input subsequences byte-equal, all retained
payload/lifecycle actions preserved,4282ONLINE/finalized timestamps each pass.
Initial failed.txt is prelaunch Python3.10 hashing incompatibility, NOT receiver
failure; run_compat.py/start_compat.json and replay_complete.json are authoritative.
Each receiver launched once, no numerical retry.23input/16runtime pins verified;
containers restored stopped, no source/default/tuning/deletion/push. Proposed
bounded Kimera-only newest-state prediction trace NOT authorized/run. Preserve
original BOTH and native records; do not select health weights from ATE alone.

**CP-V24 LEICA-2 FUSION GAP (2026-10-27; COMPLETED DIAGNOSIS, NO FIX):**
Read experiments/leica2_fusion_gap_diagnosis_cpv24_20261027/REPORT.txt,
results.json, frontier_cohorts_complete.json. Original-Qc1.0 fusion ATE lies
between native sources; its ONLINE1s rotation RPE is worse than both. Chaining
is a separate additional drift problem, not an improvement in global accuracy.
Saved full journal audit: latest GLIM endpoint median~153ms behind publication;
neither source directly touches newest support~55.45% full/~50.6% inside common
intervals. WNOA propagates earlier information; not zero information/disconnection.
Sparse Kimera belief-tail ages are not transport delay or missing camera data.
Exact saved-pair chain decomposition passes2.15e-14m; useful previous-point
corrections discarded. Finalized reduces excess path, not original global ATE.
No uniquely proved cause of original ATE gap; conditional covariance/mixed-source
correlation and factor representation are unisolated concerns. Next proposed
OFF saved receiver GLIM-only/Kimera-only controls are NOT run/authorized; do not
confuse them with native source results. No replay/build/container/model edits,
defaults/deletion/commit/push.16combined pins unchanged; original reports remain intact.

**CP-V23 FULL LEICA-2 INCREMENTAL OUTPUT (2026-10-27; COMPLETED, NOT PROMOTED):**
Read experiments/leica2_incremental_full_cpv23_20261027/REPORT.txt,
results.json and closeout.json. One full saved Velodyne OFF/ON receiver replay
each; ORIGINAL Qc, not0.25/.1. Only observer reads same-publication previous/
current poses; all original ONLINE/finalized CSV/TUM byte-identical, summaries,
296400/296470 deterministic events and4282 publication commits per mode match.
Chained O[k]=O[k-1] inv(X[k-1|k]) X[k|k] reduces path280.154->216.954m OFF,
235.137->209.649m ON (reference205.929m), but full independently fitted ATE
worsens2.000424->3.786712m and1.828658->6.080933m.1s RPE also worsens;100ms
RPE improves. Smoother is NOT a global accuracy fix: useful corrections are
discarded. Graph-to-odometry correction retained; reconstructs original graph
within1.57e-12. No chained covariance/velocity assigned, no production change.
Full Rerun OFF/ON tabs preserve original fixed alignment on BOTH fusion paths,
plus optional native controls and776491 saved map points. open_rerun.sh reopens
leica2_incremental_comparison.rrd; comparison.png shows jitter/path/drift tradeoff.
Native Kimera already incremental, GLIM first-published; DELAYED finalized
separate. Full4282GT-supported rows, common3850rows/384.6s; gaps in report.
Both containers restored stopped; no source rerun/tuning/default change.
Recommend distinct local odometry/corrected graph outputs, not replacing graph
with pure chaining; production frame/covariance/velocity design not implemented.
No new experiment authorized by this pointer. Preserve all original evidence.

**CP-V22 / CP-V22A PAIRED LEICA-2 RECEIVER DIAGNOSIS (2026-10-27; COMPLETED):**
Read experiments/leica2_paired_receiver_trace_cpv22a_20261027/REPORT.txt and
closeout.json. Original-Qc saved Hesai/Velodyne OFF replays reproduce all full
ONLINE/finalized CSV/TUM byte-for-byte; deterministic events/trace commits match.
Initial CP-V22 observer failure preserved; repaired only its generic GTSAM
Jacobian output-vector allocation in fresh A roots, not the original model.
First-minute new GLIM newest-point shift p95:3.085/6.589cm; identical Kimera
beliefs cause4.243/9.973cm. Newest pose/velocity sensitivity dominates; immediate
retirement p95 only0.084/0.130mm, older active states much steadier.42024tested
Jacobian block/step comparisons pass; endpoint checks pass. Five frozen linear
systems each full rank with sub-mm remaining newest linear corrections; not
global nonlinear stationarity or comprehensive bug-free proof. No estimator
fix, tuning, source rerun, default promotion or next experiment. Age0.2s state
readback reduces Velo step RMS5.057->1.376cm but is delayed, NOT current-time
ONLINE. User then recalled Kimera incremental-pose output: saved effective
no_incremental_pose=false and pinned implementation confirm Kimera chains
same-update relative motions, whereas fusion writes absolute graph poses.
First-minute saved-snapshot chaining reduces Velo path41.249->33.759m and step
RMS5.015->2.465cm, but common-start endpoint error1.544->3.405m. No extra delay
or solve. See REPORT section11 / incremental_publication_check.json/.png.
These causal outputs have different semantics; smoothness is not optimizer
quality. Proposed next: separate continuous odometry/corrected graph products
and audit error/correction/frame consistency before further tuning; not done.
Both containers stopped after owned jobs exit; original
pins and Hesai checkpoint retained. CP-V21's 'not run' proposal below is now
historical. Existing original evidence and failed observer roots must remain.

**CP-V21 LEICA-2 HESAI/VELODYNE OFF JITTER AUDIT (2026-10-27; COMPLETED):**
Read experiments/leica2_sensor_jitter_audit_cpv21_20261027/REPORT.txt.
Saved-only CP-K29/CP-V4 diagnosis, no fix/replay/tuning. First60s ONLINE paths
34.743/41.249m versus32.608m reference;100ms vector RMS2.594/5.015cm.
Same receiver/model; Kimera1,937native poses/47,605belief payloads identical.
Copied extrinsics match raw TF; first-belief log->event payload/time equality
and sampled raw point-time normalization pass. No confirmed frame/time-unit
bug. GLIM endpoint phase differs~14ms; Velodyne first60s relative translation
RMS2.66x Hesai, median translation covariance trace1.37x. Not a NEES/weight
certificate or proof poor hardware alone explains the fusion response.
Newest-state interpolation/update/Jacobian sensitivity remains to isolate;
paired unchanged-model diagnostic trace/numerical checks proposed, NOT run.
113pins and Hesai667file checkpoint pass. Use full_range.png plot (original
plot retained but clips a few peaks). ARC-3 and earlier Qc candidates untouched.

**CP-V20 HESAI LEICA-2 RERUN VIEW (2026-10-27; COMPLETED):**
User left ARC-3 aside. Saved CP-K29 full OFF/ON overlay in
experiments/leica2_hesai_rerun_cpv20_20261027/leica2_hesai_full_scene.rrd;
REPORT.txt and open_rerun.sh beside it. Same CP-V11 presentation: full four
paths per OFF/ON tab, GLIM first-published/Kimera ROS/fusion ONLINE/IE-TC,
1,070,784saved Hesai ON map points. Original Qc, not a Velodyne ablation.
CP-V10 common-mask rigid fits applied to ALL native points, no smoothing,
resampling or crop.20996estimator/85750reference points; gaps preserved.
Exact RRD vertices/edges/cloud and63pins pass; Hesai667file checkpoint verifies.
Native screenshot inspected; normal viewer opened14:35+02. Screenshot-only
shutdown panic same as CP-V11, separate from normal operation. No estimator
rerun, container launch, source/settings/baseline edit, deletion or push.
Grey map is reconstructed GLIM context; IE-TC shares CPT7, not independentGT.

**CP-V19 ARC-3 GLIM DIAGNOSIS / CP-V18 STATUS CORRECTION (2026-10-27):**
CP-V18 STOPPED/FAILED, not running; controller cleanup13:07:27+02 passed,
2959protected hashes intact, both containers stopped/PID0. Read
experiments/arc3_glim_failure_cpv19_20261027/REPORT.txt and JSON audits.
Saved-only investigation: recorded lidar gaps leave3--4active scans; no new
usable Kimera->GLIM factor for10.899s, explicit active CBS factors fall to0.
Existing-state timestamp/duration matching prevents peer rescue across sparse
GLIM states. Kimera stays locally healthier; GLIM CURRENT revised window has
30.777m displacement versus0.688m IE-TC, so not just ONLINE stitching.
Recovery includes a69.411m publication correction, NOT physical movement.
At513.5675s LOCAL provider guard fails extraction/chart consistency, then
invalidates history. Exact numerical branch/matrices unrecorded; large
post-gap cancellation suggests sensitivity but is not a proved false positive.
No CUDA/throughput trigger; failing scan12.773ms,MAIN7.452ms. Diagnosis only:
no replay/build/settings/guard edit/retry. Further gap-support design and
isolated numerical reproduction need a separate decision. No full ARC-3 pair.
Historical CP-V18 launch entry below is superseded by this status correction.

**CP-V18 ARC-3 STANDALONE ON (2026-10-27; RUNNING):**
Launched ONCE12:22:57+02;launcher62623/controller62630, capture12:23:12.
Startup12:24:38 effective source/sensor gates and actual CP-V15 Qc0.25exe pass;
both source beliefs and109ONLINE outputs,queue0/max23. Startup only, no full
validation/result yet.2959pins verified; Hesai4cases/667files unchanged.
Check current status before acting; never duplicate. Finite controller owns
remaining ON/gates/scoring/cleanup, no persistent automation enabled.
User explicitly requests CBS ON despite CP-V17 OFF failure. Read
experiments/arc3_velodyne_on_cpv18_20261027/PROTOCOL.txt, REPORT.txt and actual
launch/status receipts before acting. One fresh ON only, same726.38s window,
0.20x,CP-V15 Qc0.25receiver and frozen CP-V17 sources/profiles/guards. Separate
diagnostic admission; old OFF-gate failure is not relabelled or bypassed in its
original harness. OFF numerical failure around305.93s occurred amid explosive
Kimera motion, while known >1s raw lidar gaps begin around501.86s; no dropout
at the failure time. Exact solver conditioning cause remains unproved.
No OFF retry or new model; do not presume CBS rescue. If ON passes, full ON
metrics separate from shared supported failed-OFF-prefix comparison. Primary
first-published GLIM/Kimera/ONLINE, refined/finalized secondary. Finite owned
ON/gates/saved-scoring/cleanup controller; no persistent automation, retries,
guard relaxation, tuning, default promotion, deletion, commit or push.

**CP-V17 STATUS CORRECTION (2026-10-27):** OFF STOPPED/FAILED, not running.
Authoritative arc3/status.json failed capture_off; receiver indeterminate linear
system near x3029, transport Broken pipe secondary. Prefix ~306/726.38s, not
a full run. ON never launched. Cleanup passed23:43:33+02, both containers
stopped and2874protected hashes unchanged. Original chronology below retained.

**CP-V17 ARC-3 VELODYNE Qc0.25 (2026-10-26; CBS OFF RUNNING):**
Launched ONCE23:16:17+02:00; launcherPID19368/controllerPID19375. Actual
source/effective/sensor gates and Qc0.25executable passed startup; both source
beliefs and fusion ONLINE observed23:17:45. No completed-run validation yet.
Read experiments/arc3_velodyne_qcquarter_cpv17_20261026/REPORT.txt and current
launch.json/arc3/status.json before acting. User approved explicit gap-aware
full as-recorded OFF/ON. Raw admission v2 passes726.38s, offset0.2s,7028scans/
7262available stereo pairs; all interior gaps/70partial scans retained, no
fabricated samples. Native output checks every available interior raw scan,
not the old clean-input<=1s bound; runtime/numerical/lifecycle guards unchanged.
Exact CP-V15 Qc0.25receiver both modes, frozen CP-V2 sources/profiles,0.20x.
First-published GLIM/Kimera/ONLINE primary, gap-aware common masks; refined
GLIM/finalized fusion secondary. One finite controller owns OFF/gates/ON only
on pass/gates/scoring/cleanup; never duplicate or retry. No persistent follow-up.
Original input failures preserved; admission endpoint adds10ms for the last
scan's recorded future IMU bracket. No builds/defaults/deletion/push authorized.
Read fresh launch/status receipts; no numerical results yet. Never duplicate.

**CP-V16A LEICA-2 TENTH-Qc (2026-10-26; COMPLETED, NOT PROMOTED):**
Read experiments/leica2_motion_prior_tenth_cpv16a_20261026/REPORT.txt.
User requested0.1times ORIGINAL Qc:10x original motion information, not0.1times
CP-V15. One full saved OFF/ON receiver replay each; unchanged source payloads,
timing, coverage and other settings. Math/lifecycle/payload checks pass.
ONLINE100ms vector RMS2.8728/2.5505cm improves45.98%/17.69% vsoriginal,
12.60%/1.70% vs0.25. Paths227.867/223.687m vs205.929m reference still10.65%/
8.62% excessive. ATE2.013129/1.830556m:12.705/1.897mm worse thanoriginal;
practically small, but strict no-increase screen FAIL. Compared with0.25,
1s translation RPE worsens both; ON speed-lag scan shifts+.16->+.18s versus
+.14s original. Diagnostic only, not proven latency. Diminishing returns,
especially ON;0.25 remains a more conservative working tradeoff, not promoted.
DELAYED finalized translation worsens vs0.25; rotation improves. Not a global
drift fix or proof of initial/marginal-prior weakness. Initial CP-V16 setup-only
undefined-variable failure occurred before build/replay; preserved separately.
Fresh CP-V16A repairs only host pin setup, not a numerical/source retry.
Both containers stopped after successful owned jobs; originals/0.5/0.25 intact.
No source capture, next weight, default promotion, deletion, commit or push.

**CP-V15 LEICA-2 QUARTER-Qc (2026-10-26; COMPLETED, NOT PROMOTED):**
Read experiments/leica2_motion_prior_quarter_cpv15_20261026/REPORT.txt.
User separately requested0.25times ORIGINAL Qc:4x original motion information,
not0.25timesCP-V14. One full saved OFF/ON receiver replay each; source beliefs/
covariances/order/coverage unchanged. Math/payload/lifecycle checks pass.
100ms vector error RMS improves38.20%OFF/16.27%ON vsoriginal and18.25%/6.05%
vs0.5. ONLINE paths235.806/225.060m vs205.929m reference: still14.51%/9.29%
excess. ATE2.007044/1.829250m worsens6.620/.592mm vsoriginal; strict frozen
screen FAIL. Moving/turning and1s RPE improve. Response scan minima move one
20ms bin (speed+.14->+.16s,angular+.08->+.10s), not proven flight latency.
Original and0.5results preserved/reproduced; original default retained.
Both containers stopped, no source capture/sweep/retry/deletion/push. No next
weight authorized. Partial jitter reduction, not global drift or prior fix.

**CP-V14 LEICA-2 MOTION-PRIOR STRENGTH (2026-10-26; COMPLETED, NOT PROMOTED):**
Read experiments/leica2_motion_prior_cpv14_20261026/REPORT.txt and closeout.json.
One full saved CP-V4 OFF/ON receiver replay each; single predeclared Qc*0.5,
same source means/covariances/order and all other receiver settings. Private
one-line delta;100-case interpolation/Jacobian/noise/motion-cost audit passes.
All296400/296470event metadata and255857/255928source payload commits match.
Age-zero100ms vector error RMS improves24.4%/10.9%; ONLINE path280.154->250.947m
OFF,235.137->228.563m ON versus205.929m reference. Moving/turning and1s RPE
improve; lag-scan minima unchanged. Global ATE worsens2.000424->2.003606m OFF,
1.828658->1.829293m ON: small, but fails frozen no-worsening screen. Partial
jitter benefit supports ongoing motion/source-balance sensitivity, not a weak
initial/marginal prior diagnosis or full fix. Original default retained.
Original evidence intact, both containers stopped, no source capture/sweep/
retry/deletion/push. Further Qc candidates or live tests need a new decision.

**CP-V13 LEICA-2 RECEIVER UPDATE TRACE (2026-10-26; COMPLETED):**
Read experiments/leica2_receiver_trace_cpv13_20261026/REPORT.txt. One unchanged-
model saved-journal replay per original CP-V4 OFF/ON, observer-only private
build: ONLINE/finalized CSV/TUM byte-identical;296400/296470event metadata
rows and selected commits match. No source rerun/tuning. Selected380--390s
large newest-state shifts occur at new beliefs: median GLIM3.27/2.08cm,
Kimera6.46/3.72cm OFF/ON, not retirement (p95~0.009cm). Older window much more
stable; 0.2s-old estimates already remove much of local jitter, but are delayed,
not current-time output or a fix. Initial/marginal-prior weakness NOT established;
ongoing WNOA/source balance remains a hypothesis. Detailed traces/active-window
snapshots/plots/pins saved. Both containers stopped; original evidence preserved.
Next proposed single receiver-only motion-prior-strength ablation is NOT run
or authorized by this pointer. No promotion, sensor capture, deletion or push.

**CP-V12 LEICA-2 ONLINE JITTER AUDIT (2026-10-26; COMPLETED, CAUSAL TRACE OPEN):**
Read experiments/leica2_fusion_jitter_cpv12_20261026/REPORT.txt. Saved CP-V4
Velodyne same-time ONLINE paths280.154/235.137m become delayed finalized
204.797/205.890m vs205.929m IE-TC. Global ATE remains~2.000/1.83m. Selected
380--390s update cohorts alternate GLIM-only/GLIM+Kimera every100ms; OFF
phase-mean offset7.06cm,ON1.53cm. Association, not isolated source causality.
12311inputs unchanged; independent path/ATE and SE(3) prediction checks pass.
No per-belief fused pre/post states or simultaneous windows were saved, so
weak-prior/marginal-conditioning causality is unverified. Proposed unchanged-
model instrumented receiver-only replay needs a new decision; NOT run here.
No source/solver replay, build, tuning, containers, promotion or baseline edit.

**CP-V11 LEICA-2 VELODYNE RERUN VIEW (2026-10-26; COMPLETED):**
User-corrected presentation: full_overlay/leica2_full_scene.rrd under the
folder below. ONE3D scene per OFF/ON tab with all four FULL paths superimposed
and the same776,491point saved Velodyne map. No trajectory smoothing/cropping;
all points/edges/cloud readback and47pins pass. Map is context, not independent
GT. full_overlay/REPORT.txt supersedes the earlier short-window default view.
experiments/leica2_velodyne_rerun_cpv11_20261026/REPORT.txt and
leica2_velodyne_first_published.rrd visualize original CP-V4 OFF/ON GLIM first-
published/Kimera ROS/fusion ONLINE against shared-CPT7 IE-TC. Every native
sample retained, CP-V10 common-mask SE(3) alignment, no smoothing/resampling.
Full3D, selected380--390s XY/XZ, scrub-able10s paths and100ms step charts.
Exact RRD point/segment readback and15input pins pass. Selected window visibly
jagged, especially fusionOFF; this concerns connected first publications, NOT
proof of revised graph or physical drone zigzag. Worst-window selection clear.
No source rerun/settings/container change. Native viewer left open; report
documents screenshot-shutdown and CLI verification quirks. Old results intact.

**CP-V10 FIRST-PUBLISHED OUTPUT REEVALUATION (2026-10-26; COMPLETED):**
Read experiments/first_published_comparison_cpv10_20261026/REPORT.txt before
cross-estimator online-accuracy claims. All seven baseline OFF/ON pairs rescored
from the 14 saved glim_odometry.csv files, alongside unchanged Kimera ROS
odometry and fusion ONLINE. Earlier odom_imu.txt GLIM scores are revised
fixed-lag exports, NOT first publication; preserve them as separate products.
Same-time maturity audit: refinement improves GLIM ATE 0.030--7.090% in 13/14,
worsens one by 0.184%; all 14 estimator ATE rankings unchanged (GLIM lowest 12,
Kimera 2, fusion 0). GLIM 1s translation RPE benefits more: 5.13--19.58%.
Primary first-publication CBS ATE improves GLIM 3/7, Kimera 6/7, fusion 6/7.
42 historical scores reproduce within 1.56e-8m; independent evo/matrix checks
pass; 136 pinned inputs unchanged. Common masks, paths, RPE and figures saved.
First-publication does not certify equal latency: Kimera's saved incremental
output chains optimized relative motions, unlike newest absolute-state outputs.
IE-TC shares CPT7; sparse keyframe gaps and mean-only covariance alignment
limitations remain. No rerun/replay/container/build/tuning/deletion/push.
Old reports stay intact; CP-V10 supersedes their mixed-maturity online ranking.

**CP-C3 OLD NON-GRANDTOUR STAGE CLEANUP (2026-10-26; COMPLETED):**
User-approved ten stage2a/stage2c/stage3a/stage16--18 roots pruned,not GrandTour.
39,985files/links removed permanently;56.21GiB reclaimed during deletion,
55.55GiB net after668MiB retained archive;68.07GiB free.12,922retained entries
and3,558current protected hashes verify; all14liveGLIM odometry inputs and
Hesai667file checkpoint unchanged. Original GEODE Stairs bag/source copies/
safety patches/reports retained. See experiments/artifact_cleanup_cpc3_20261026/
REPORT.txt,completion.json and retained_compact_archive.tar.gz. Deleted bulk
captures/builds are not backed up and old stage replay may require reconstruction.
No new run,scoring,model/build change or push; containers stayed stopped.

**CP-V9 G2K GRAVITY-PROJECTION EXPERIMENT (2026-10-26; COMPLETED, REJECTED):**
Full428.75s candidate and saved-data scoring finished. Authoritative closeout:
experiments/leica2_gravity_projection_cpv9_20261026/report_repair/final.json.
Original report-only raw matrix vs Exp(Log) failure remains preserved; bounded
polar comparison with original equality tolerance resolves tiny nonorthogonality.
All1908actual4D G2K factors/Jacobians and1917unchangedK2G factors validate;
3027protectedfiles unchanged,no source rerun,both containers stopped.
Full-saved ATE originalON->candidate: GLIM.861349->.724307,
Kimera2.584159->3.949346,ONLINE1.828658->2.613159m. Common384.6s confirms:
KimeraATE+52.32%,20s total rotationRPE-24.78%,but gravity-axisRPE+38.47%.
Scientific screen FAIL; do not promote. DECISION_REPORT.txt and REPORT.txt
document limits. Fixed projection is NOT degeneracy detection. No new run,
sweep,retry/default promotion/push authorized. Original chronology follows.
User explicitly approved CP-V8's single full Leica-2 Velodyne ON ablation.
Read experiments/leica2_gravity_projection_cpv9_20261026/PROTOCOL.txt, REPORT.txt
and status.json when present before any action. Private immutable projected
BetweenFactor: fixed gravity-axis angular residual +3translation, dense MCM^T;
gravity from native IMU/receiver pre-state.1000geometry checks and actual
insertion/LOCAL-exclusion/retirement/CP-K9 rollback/default-parity tests pass.
ONE428.75s ON at0.20x,fresh port11487; saved CP-V4 OFF/original ON controls.
No detector, extra captures, auto-retry, default promotion or push. Finite
controller owns capture/gates/scoring/decision/cleanup; never duplicate stages.
Inspect launch/status before assuming running/completed. Preserve all baselines.
Launched ONCE12:21:13local(+02:00),launcher25937/controller25944. Effective
startup passed; first10actual4D factors verify MCM^T/native gravity.3027pins.
Final scientific decision is DECISION_REPORT.txt/decision.json, separate from
operational results.json; stop after this one test. No heartbeat was enabled.

**CP-V8 CROSS-SEQUENCE DECISION REPORT (2026-10-26; COMPLETED):** Read
experiments/cross_sequence_decision_cpv8_20261026/REPORT.txt and PROPOSAL.txt.
Saved data only: seven completed Hesai/Velodyne baseline pairs; ARC-5 failure
and rejected CP-V6A separate. All42native/ONLINE ATE values reproduced within
1.56e-8m. Kimera20s rotation RPE worsens6/7 despite positionATE improving6/7.
10609actual G2K factors/pre-states verified: incoming tilt RMS worse6/7,
heading better5/7; Hesai EIG-2 is a counterexample, not universal tilt causality.
Four figures/common masks/1-10-20s metrics/covariances/timing retained. Proposal:
parameter-free gravity-separated G2K4D marginal ablation, NOT implemented or
authorized to run. No coefficient/gate/default promotion or new experiment.
Baseline unchanged; closeout.json records original-input hashes and review.

**CP-V7 LEICA-2 drift attribution (2026-10-25; COMPLETED):** Read
experiments/leica2_drift_attribution_cpv7_20261025/REPORT.txt.
Saved-data only, no estimator/container/settings changes. Variance10 reduces
Kimera tilt but increases accumulated heading drift: at common390.385s heading
OFF/originalON/candidate -13.352/-21.310/-36.542deg. Candidate1s rotation RPE
improves,10/20s worsens; same-start cohort confirms.1905/1908actual G2K factors
and pre-states verified;1384matched intervals show GLIM heading RMS better
than Kimera pre-state(.0579/.0789deg original) but tilt worse(.3183/.1161deg).
Matched-factor coverage lacks210--300s bins; no isolated causal attribution.
All nine full-saved ATE values reproduced within7.01e-9m;19inputs unchanged.
This supports a directional tradeoff, NOT a calibrated policy or gyro-bias
diagnosis. Baseline preserved,variance10 not promoted,no new test authorized.

**CP-V6/CP-V6A LEICA-2 rotational weighting (2026-10-24; COMPLETED):**
Full428.75s candidate capture/four original gates/corrected actual-factor gate
and scoring completed14:53local. report_repair/final.json passed; both containers
stopped,3002protected hashes unchanged,no source retry/terminal omissions.
All1908G2K matrices confirm rotation variance10, unchanged translation/cross
blocks;1919K2G matrices unchanged. Original report-only failure preserved.
Full-saved ATE originalON->candidate(m): GLIM.861349->.713107,
Kimera2.584159->4.430475,ONLINEfusion1.828658->2.934918. Kimera1s translation/
rotation RPE improves17.87%/11.76%, but global errors worsen; common mask
confirms direction. Candidate NOT promoted; no further experiment authorized.
See RESULTS_REPORT.txt and report_repair/comparison.json. Chronology follows.
User approved
one reduced G2K rotational-influence test after CP-V5. Read
experiments/leica2_rotation_weight_cpv6a_20261024/PROTOCOL.txt, REPORT.txt and
status.json. Initial CP-V6 harness failed receiver execute-bit preflight BEFORE
any source/receiver capture; its cleanup passed. CP-V6A preserves execute bits
in a fresh root; same single candidate, no model/source retry. Unmodified CBS
rebuild byte-identical to baseline;1000dense matrix/default/isolation tests pass.
Private receiver-only rotation variance10, translation/cross blocks unchanged;
same full428.75s LEICA-2 Velodyne ON compared to saved CP-V4 OFF/ON, no sweep,
duplicate OFF, retry/default promotion/push. Finite candidate controller handles
capture/gates/scoring/cleanup; do not compete or relaunch. No heartbeat enabled.
Check saved status before action. Earlier no-new-test pointers are historical.
CP-V6A launched14:13local/controller530448; effective startup passed. First
156G2K/158K2G actual factor matrices verify requested treatment exactly. Original
weight reporter misidentified upstream FL4M_FROZEN_FACTOR_ROW as applied; source
inspection and actual factor-matrix logs resolve this without runtime edits.
Pinned validator remains; finite report_repair/finish_saved_report.py will
wait for its expected report-only failure/cleanup, then validate actual factors
and score saved data ONLY after all four original gates pass. Check
report_repair/final.json for authoritative final reporting/cleanup status;
original status.json will retain that report-gate failure. No source retry.

**CP-V5 LEICA-2 Velodyne belief audit (2026-10-24; COMPLETED):** Read
experiments/leica2_velodyne_belief_audit_cpv5_20261024/REPORT.txt. Saved-data only,
no rerun/replay/tuning.507118versions checked,22160matched first/near4s records;
27input hashes unchanged.1905actualG2K factors and every Kimera pre-state
verified;1913K2G factors joined (1905rev0+8rev1),no K2G pre-state claim.
Covariance ranks translation usefully, but incoming G2K rotation rank~.047;
GLIM rotation worse than Kimera pre-state83.67%. Offline lower-trace choice
reduces translation RMS9.4% but raises rotation RMS26.0%; NOT a fusion run.
Post-update rotation worsens59.06%; all-factor update,not isolated causality.
Known mean-only alignment/shared-CPT7 limitations unchanged. Kimera matched
cohort has200..300s gaps,not missing camera data. Use translation_time_coverage.png,
not superseded translation_time.png. No coefficient/new experiment authorized.

**CP-V4 overnight Velodyne campaign (2026-10-24; FINISHED WITH FAILURE):**
Latest: SPX-3/EIG-2/LEICA-2 full matched pairs completed/all24gates/scoring pass.
ARC-5 OFF stopped around380.6/401.29s on fusion GTSAM indeterminate linear
system near x3758; ON never launched. Numerical cause unresolved; no retry or
guard/setting change. SNOW-3 startup blocked; ETH1/SPX-2/CON-1 missing inputs.
Read experiments/velodyne_full_campaign_cpv4_20261024/RESULTS_REPORT.txt and
FINAL_AUDIT.txt. All owned controllers exited, both containers stopped,
protected hashes unchanged, Hesai667file checkpoint verifies. Follow-up PAUSED;
do not restart completed/failed controllers. No further capture authorized.
Chronological campaign record below. User
requests full available-sequence matched OFF/ON, with routine problem handling
while asleep. Read experiments/velodyne_full_campaign_cpv4_20261024/PROTOCOL.txt,
REPORT.txt, campaign.json and status.json when present. SPX-3/EIG-2/LEICA-2 keep
their exact established full usable windows; ARC-5 raw quiet-start preflight
passes at bag3.2s for401.29s. SNOW-3 blocked: no verified quiet initialization
interval; ETH1/SPX-2/CON-1 missing required bags. No silent sensor/model swap.
Finite sequential controller runs OFF->four gates->ON->four gates->score->cleanup,
one attempt per mode, unique roots/ports11481..11485; refuse duplicates.
Freeze CP-V2 private builds/profiles, rear Velodyne/front stereo/CPT7,0.20x,
all-LOCAL covariance/corrected MAIN means, temporary classic CBS,7s WNOA/no
central IMU/GNSS,Gaussian scale1,seal_at_absorption,unchanged EOF/alignment.
No tuning/builds/guard relaxation/deletion/push. Failed cases retain evidence;
continue independent cases only after safe cleanup. This supersedes unlaunched
CP-V3, not completed Hesai/CP-V2 outputs. Check live status before any action.
Use experiment-only heartbeat; disable after completion/no safe work remains.
Campaign PID441787/launcher441780 started00:07:39local; SPX-3 OFF00:07:55.
Use launch_cli_repair.json/campaign_cli_repair.log. Earlier launcher failed
before any captures/containers due only to GNOME CLI syntax; evidence kept.
Do not mistake old launch.json/campaign.log for the active process or retry it.
00:28local milestone: SPX-3 full197.02s OFF and all four gates passed; ON
started00:26:24 by the existing controller. Do not relaunch/redo these stages.
Read live status for subsequent progress; no accuracy comparison yet.
00:45:58local SPX-3 full197.02s pair completed, eight gates/scoring/cleanup pass;
2847protected hashes unchanged. Full-saved ATE OFF/ON nativeGLIM.050405/.053729,
nativeKimera.483414/.376635,ONLINEfusion.128743/.112111m; one OFF terminal
Kimera factor omitted/zero ON. DELAYED finalized separate; common-mask confirms
ATE direction, but ONLINE translation RPE worsens. See spx3/RESULTS_REPORT.txt.
Campaign continues remaining missions; never repeat completed SPX-3 stages.
01:19local: EIG-2 full355.02s OFF and four gates passed; ON started01:18:31
by existing controller, startup verified. Do not repeat completed OFF stages.
01:50:50 EIG-2 pair completed/eight gates/scoring/cleanup passed,2847hashes
unchanged,zero terminal omissions. ATE OFF/ON nativeGLIM.283734/.330320,
nativeKimera1.882644/1.802880,ONLINEfusion.534328/.886053m. DELAYED separate;
ONLINE path lengths299.811/255.326m vs~188.126m reference retained as evidence.
LEICA-2 OFF launched01:51:04; do not rerun EIG-2 because of unfavorable accuracy.
02:29local: LEICA-2 full428.75s OFF and all four gates passed02:28:31; existing
controller launched ON02:28:44, effective-mode/sensor startup checks pass.
Both sources/fusion advancing; matched accuracy pending. ARC-5 remains queued.
Do not repeat OFF validation or launch another ON capture.
03:07:15 LEICA-2 full428.75s pair/eight gates/scoring/closeout completed,
2850protected files unchanged,zero terminal omissions. Full-saved ATE OFF/ON:
nativeGLIM2.828135/.861349,nativeKimera1.097081/2.584159,ONLINEfusion
2.000424/1.828658m; DELAYED separate. Common384.599633s confirms direction;
sparse native stationary-tail gaps disclosed. See leica2/RESULTS_REPORT.txt.
ARC-5 OFF launched03:07:27/PID494891 by existing controller;startup passes.
SNOW-3 remains blocked; never rerun completed stages or tune from these results.

**CP-C2 obsolete trial payload cleanup (2026-10-23; COMPLETED):** User requested
broader deletion. Removed35598snapshotdirectories+637debugCSVs/153285files
from stationary_consensus_20261006,local_prediction_20261007,serial_qr_20261007,
bounded_polish_20261007,exact_accel_20261007 under /home/yeranis/results.
Retained compact outcomes/final trajectories/code/builds and two saved-graph
fixtures;~335MiB remains across these five roots. Recovered21.32GiB additional,
26.75GiB together with CP-C1;free35.41GiB.1380retained and2770current protected
hashes pass; four-sequence Hesai checkpoint667files verifies against originals.
Current Hesai/Velodyne captures/datasets/runtime unchanged. Old bulk trials are
intentionally decommissioned; do NOT expect deleted histories or recreate them
automatically. Read experiments/artifact_cleanup_cpc2_20261023/REPORT.txt.
Deletion permanent; no new run/push. CP-C1 below describes the earlier pass.

**CP-C1 historical repeat cleanup (2026-10-23; COMPLETED):** User authorized
targeted deletion. Only9847 intermediate numeric snapshot directories inside
bounded_polish_20261007/serial_qr_20261007 repeat cases removed; last/final
snapshots, all original cases/builds/reports/trajectories retained.45147 retained
file hashes and2770current protected files unchanged. Recovered5.43GiB,free
~14.15GiB. Exact manifest/report: experiments/artifact_cleanup_cpc1_20261023/.
Permanent deletion; old scripts needing those repeat histories cannot rerun
as-is. No current Hesai/Velodyne artifact removed; no new run/push.

**CP-V3 full SPX-3 Velodyne (2026-10-23; STORAGE RESOLVED, NOT LAUNCHED):**
User approved full197.02s matched OFF/ON after CP-V2. Read
`experiments/velodyne_spx3_full_cpv3_20261023/REPORT.txt`. Only8.85GiB free;
full OFF projected~1.31GiB would leave less than unchanged8GiB ON launch gate.
Need >=12GiB free or authorized suitable output storage before preparation.
No containers/captures started, no guard edits/deletions/relocations. Preserve
CP-V2 and Hesai artifacts; intended settings are exactly CP-V2,0.20x with the
CP-K28 full197.02s physical journey. No other sequence or tuning authorized.
Above8.85GiB blocker is historical: CP-C1 cleanup now leaves~14.15GiB free.
Fresh setup/preflight remain; no full capture started during cleanup.

**CP-V2 rear Velodyne SPX-3 smoke (2026-10-23; COMPLETED):** Read
`experiments/velodyne_spx3_smoke_cpv2_20261023/REPORT.txt`,RESULTS_REPORT.txt,
PROTOCOL.txt,status.json and closeout.json. Both authorized30s OFF/ON captures
and all four gates per mode passed; no terminal omissions. Full-saved native
GLIM ATE OFF/ON0.067795/0.055356m, Kimera0.096045/0.175519m, ONLINE fusion
0.064922/0.085510m vs shared-CPT7 IE-TC (29.00/29.60/29.40s saved spans).
Exact quiet bag29..59,0.20x,port11480; same current algorithms,only rear LiDAR
input/calibration.2770protected hashes unchanged; controller exited and both
containers restored stopped. Hesai checkpoint667files matches originals.
No tuning/full-sequence follow-up/retry authorized. This is integration success,
not universal accuracy improvement or0.5x validation. Preserve completed outputs.

**Four-sequence Hesai checkpoint and next sensor plan (2026-10-23):** User has
parked Hesai experiments; read `experiments/checkpoints/20261023_hesai_four_sequences/README.txt`.
Local compact667file snapshot/hash manifest preserves CP-K23 ETH1,CP-K24A
EIG-2,corrected CP-K28 SPX-3,CP-K29 LEICA-2 and CP-T1 timing. Large original
captures remain untouched; this is not an offsite backup/GitHub push. No0.5x
run/audit removal/tuning authorized. CP-V1 plan:
`experiments/velodyne_campaign_cpv1_20261023/PLAN.txt`. Rear Velodyne topic and
old sensor overlay exist for EIG-2/SPX-3/LEICA-2; ETH1 bag not found. Preserve
current camera/CPT7/CBS/WNOA/private builds,0.20x and physical windows; fresh
sensor-specific harness only after approval. Plan is NOT a launch authorization.
Only~9.5GiB free at planning time; reserve/storage gate before any new capture.

**CP-K29 full usable LEICA-2 matched comparison (2026-10-23; COMPLETED):** Read
`experiments/leica2_full_cpk29_20261023/PROTOCOL.txt`, REPORT.txt and status.json.
User authorized one fresh full OFF/ON pair. Both captures,threegates per mode,
scoring and closeout passed17:58:19local. Do NOT duplicate completed stages.
Mission2024-11-25-16-36-19,bag6..434.75,428.75s through lastHesai,0.20x,
port11479. Startup raw-camera/IMU quiet pre-roll independently checked; initial
6s settling and trailing camera/IMU-only data excluded. Mission CameraInfo/TF
match frozen CP-K28 profiles to numerical precision. Same private binaries,
all-LOCAL covariance,classic temporaryCBS,7sWNOA/no centralIMU,Gaussian scale1.
Six input hashes/2837protectedfiles. Three gates OFF before ON, same three
ON gates then scoring/summary. Scoped SystemD+GNOME inhibitors; both containers
initially stopped and restored only after owned jobs exit. No persistent
automation, retry, tuning, rebuild, default promotion or push. Both containers
restored stopped;2837protectedfiles unchanged. Results in RESULTS_REPORT.txt
and results.json: full-saved ATE OFF/ON(m),nativeGLIM.569231/.570179,
nativeKimera1.097081/.480025,fusionONLINE.696970/.565647. DELAYEDfinalized
.696285/.565905 separately. No EOF omissions; all saved poses scored, actual
427--428s spans; common384.9996s supported/420.8996s envelope. No stationarity
or higher-rate certificate. Full source covariance/revision/timing data retained.
Run root /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_full_cpk29_20261023/prepared.
Report full-saved original/nativeGLIM,original/nativeKimeraROSodom,ONLINEfusion
OFF/ON plus common mask; DELAYED finalized separate. IE-TC shares CPT7;
independentprism covers only part, mean-only covariance alignment unchanged.

**CP-K28 full SPX-3 quiet-start rerun (2026-10-23; COMPLETED):** Read
`experiments/spx3_stationary_full_cpk28_20261023/REPORT.txt`, RESULTS_REPORT.txt
and results.json. Both197.02s OFF/ON captures, three gates per mode, scoring
and closeout passed. Completed15:32:04local; both containers restored stopped,
2763protectedhashes unchanged. Do NOT duplicate completed stages.
Windowbag29..226.02,0.20x,port11478; quiet pre-roll through lastHesai; same
CP-K27 binaries/profiles/covariance/model, no tuning/rebuild. Full-saved ATE
OFF/ON(m): nativeGLIM.016576/.016559, nativeKimera.483414/.110169,
fusionONLINE.044145/.040815; DELAYEDfinalized.037843/.034675 separately.
Actual spans196.199/196.798/196.400s; every saved pose scored. Common mask
191.799817s supported in195.299814s envelope;1923queries,39excluded. One final
Kimera revision0 omitted EACH mode under predeclared final-partial-cell rule;
not zero factor loss. All covariance payload errors0; MAIN sync calls0.
Run root: /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_stationary_full_cpk28_20261023/prepared.
IE-TC sharesCPT7,not independentGT; mean-only covariance alignment unchanged.
No fresh replay/exhaustive batching/nonlinear-stationarity claim. Scoped
inhibitors released; no retries/default promotion/commit/push. Full covariance,
revision,payload,timing histories retained. Earlier CP-K26/27 preserved.

**CP-K27 bounded SPX-3 startup repair (2026-10-23; COMPLETED):** Read
`experiments/spx3_stationary_preroll_cpk27_20261023/REPORT.txt`. One28s OFF/ON
pair starts atbag29,3s before CP-K26's movingbag32 crop, with ALL estimator
binaries/profiles/covariance/model settings unchanged. Same three startup
relative intervals' revised translation errors fall43--95mm to1.4--2.4mm.
99matched intervals confirm large translation improvement; rotation uncertainty
is NOT thereby calibrated. This is a launch/startup-condition correction, not
an algorithm patch or universal covariance tune. ETH1/EIG-2 saved first-pose
reference speeds are0,not the~.39m/s SPX-3 moving-start violation. Use verified
quiet pre-roll for stationary-only initialization; do not copy this mission's
29s offset blindly. CP-K26 full results below remain unchanged historical
captures; corrected full rerun is now separately authorized as CP-K28 above.
New28s full-saved ATE OFF/ON:
GLIM.006806/.006487m,Kimera.097535/.032109m,ONLINEfusion.024035/.024241m;
DELAYEDfinalized.010748/.012002m separate. Both captures/gates pass, containers
restoredstopped; no defaults promoted, replay, commit or push. Do not rerun.

**CP-K26 full usable SPX-3 restart (2026-10-21; COMPLETED AND CLOSED OUT):**
Read GNSSFGO_SPX3_FULL_CPK26_REFERENCE_REPORT_20261021.md and
experiments/spx3_awake_full_cpk26_20261021/REPORT.txt. Fresh194.02s OFF/ON pair,
offset32s,rate0.20x,port11476 completed; all three gates per mode and scoring
passed. Same K25 frozen binaries/profiles/model; no tuning/rebuild. Full-saved
ATE OFF/ON(m): native GLIM.016897/.016885, native Kimera.552484/.120067,
fusion ONLINE.058603/.044991. DELAYED fixed-lag-finalized.051392/.039281 separate.
Native/online saved spans193.20/193.80/193.40s, all saved poses scored;
common190.90s contiguous coverage in193.10s envelope confirms direction.
One terminal Kimera rev0 omitted in EACH mode under the predeclared finite-
recording predicate: future100ms support beyond recorded clock. Explicitly
counted, not zero factor loss; every other gate passed. IE-TC shares CPT7,
not independent prismGT. All-LOCAL covariances and mean-only alignment
approximation retained. Results in RESULTS_REPORT.txt/results.json.
Closeout20:16:36local passes: both containers stopped,2613 protected hashes
unchanged, no controller/cleanup failure. No permanent power-policy change.
No fresh deterministic replay/exhaustive batching/stationarity certificate,
automatic retry,push or default promotion. Never duplicate completed stages.
Run root:
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared.

**CP-K25 full usable SPX-3 comparison (2026-10-21; OFF PASSED, ON INTERRUPTED):**
Read experiments/spx3_full_cpk25_20261021/PROTOCOL.txt and REPORT.txt, then
status.json before acting. User authorized one matched OFF/ON pair,194.02s
at0.20x from established32s offset through lastHesai (not old134s crop),port11475.
Same CP-K23/24 frozen binaries/model/temporary CBS/all-LOCAL covariances and
absorbed-revision policy; SPX-3's saved CP-R5 sensor profile. No tuning/build.
Preflight2538 protected hashes and four input hashes pass. OFF launched ONCE
16:53local; effective settings pass and native GLIM/Kimera/ONLINE fusion publish.
Controller exited; OFF complete194.02s and all gates pass. ON interrupted by
host suspend17:14:12--17:34:44 despite scoped inhibition. After wake new Kimera
167->168rev0 arrives9.0119003s old beyond7s; receiver stops on retired support.
Not a completed pair or evidence of intrinsic compute overload. Closeout passes,
containers stopped,2538 protected hashes unchanged. Do NOT duplicate any stage.
Exact CP-K24A terminal-cell EOF audit predeclared equally OFF/ON, with omitted
terminal factors explicitly counted; no fake supports or other gate relaxation.
No fresh solver replay/exhaustive batching certificate, tuning, push or default
promotion. IE-TC shares CPT7. Run root
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_full_cpk25_20261021/prepared.
No matched results.json; preserve the failed prefix. Separate CP-K26 restart
explicitly authorized above; exact inhibitor-bypass cause remains unverified.

**CP-K24 full EIG-2 comparison (2026-10-21; SCORED WITH EXPLICIT EOF OMISSION):**
Read GNSSFGO_EIG2_FULL_CPK24_REFERENCE_REPORT_20261021.md and
experiments/eig2_full_cpk24_20261021/REPORT.txt. Both355.02s captures complete at
0.20x,established5.7s offset through lastHesai (not old305s crop),port11474.
Same CP-K23 frozen binaries/model/temporaryCBS/all-LOCAL covariance and selected
absorbed-revision policy. Native GLIM ATE OFF/ON.051085/.044555m; nativeKimera
1.882644/.688834m;fusion ONLINE.167508/.135765m. DELAYED.166193/.132314m separate.
All saved poses scored (354s+spans); common352.299664s confirms direction.
Reference is shared-CPT7 IE-TC,not independent prismGT.
Original ON zero-unsupported gate failed on1final Kimera1759->1760rev0 needing
a future100ms bracket beyond endclock. Original status.json/failedlog preserved.
Authorized CP-K24A saved-data audit applied SAME finite-recording predicate to
OFF/ON; both pass plus every remaining original check. Factor remains omitted;
counterfactual impact unknown. No source/solver rerun, output change, protected
gate edit or tuning. See eof_review/PROTOCOL.txt and closeout.json. Qualified
metrics are results_eof_review.json/RESULTS_REPORT_EOF_REVIEW.txt, not results.json.
All2464protected files and16saved input hashes unchanged; both containers stopped.
No default promotion,universal EOF acceptance,exhaustive batching/stationarity
certificate,push or next experiment. Do NOT duplicate capture/score/validation.
Run root /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eig2_full_cpk24_20261021/prepared.

**User-designated reference report (2026-10-21):**
`GNSSFGO_ETH1_FULL_CPK23_REFERENCE_REPORT_20261021.md` preserves the full
native GLIM/native Kimera/ONLINE fusion OFF/ON results, delayed outputs,
coverage, covariance, timing and exact frozen settings. Use it when comparing
future runs. CP-K23 captured346.32s at0.20x and is fully closed out.

**CP-K23 full matched comparison (2026-10-21; COMPLETED AND CLOSED OUT):**
Read experiments/eth1_awake_full_cpk23_20261021/PROTOCOL.txt and REPORT.txt.
Both captures and three gates per mode passed; scoring completed once.
Full-saved ATE OFF/ON(m): native GLIM.039683/.037421,native Kimera4.496229/.340930,
fusion ONLINE.166126/.103489. DELAYED finalized.155965/.098779 separate.
Saved spans345.400/342.598/345.500s; native Kimera5.1s stationary-tail keyframe
gaps disclosed. Common mask296.700s covered/298.300s envelope confirms direction.
Reference IE-TC shares CPT7. See RESULTS_REPORT.txt/results.json for all metrics.
Controller cleanup-only Docker top error preserved in cleanup_failed.json;
manual closeout verified no owned jobs,containers stopped,inhibitor released,
all2391protected hashes unchanged. No stage repeated. Do NOT duplicate captures.
Same CP-K22 algorithm/profiles/coverage/expiry policy,0.20x346.32s,port11473.
Known absorbed-ID revisions expired131OFF/130ON; no pending/unsupported end.
Old sleep-interrupted CP-K22 preserved,no automatic
retry,tuning/build/covariance/lag/default promotion/push. Run root
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared`.

**CP-K22 full comparison (2026-10-21; OFF STOPPED AFTER HOST SUSPEND):** read
`experiments/eth1_keyframe_finalization_full_cpk22_20261021/PROTOCOL.txt` and
REPORT.txt. User Proceed selects fresh matched346.32s ETH1 OFF/ON at0.20x,
same K20 sources/model with K19 seal_at_absorption equally. Only harness
coverage now recognizes native5s keyframes with frontend/backend conservation;
gap-aware scoring retains all actual poses,never fills long holes.2320protected
hashes and saved1492pose coverage fixture/seven negatives pass. No source or
solver diagnostic rerun/build/tuning. Fresh run root
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_keyframe_finalization_full_cpk22_20261021/prepared`.
OFF launched ONCE about10:55local;startup checks pass. STOPPED11:14:27 after
lid-close host suspend11:11:19--11:14:25. First Kimera972->973rev0 arrives13.009s
old,violates7s retention;error new belief references retired support. Prior known
expired revisions were correctly audited. Not a completed full result or proof
of intrinsic optimization slowness. No ON/retry; preserve evidence. Persistent
follow-up was not enabled. No repeat journal,
automatic retry,old-controller reuse/default promotion/push. Keep CP-K20
unlaunched. Read REPORT for current status before acting; refuse duplicates.

**Selected expired-revision policy (2026-10-21, user confirmed):** read
`GNSSFGO_EXPIRED_REVISION_POLICY_20261021.txt`. Keep current25-keyframe Kimera,
5sGLIM,7sWNOA receiver and other settings. Use existing CP-K19
`--absorbed-revision-policy seal_at_absorption` equally OFF/ON: active stable-ID
revisions replace; known absorbed-ID revisions expire/audit without solver
mutation or stopping. Last accepted information remains in the prior. Unknown
old first arrivals/malformed/conflicting inputs retain existing errors. This
does not alter classic source CBS or imply full live validation. Prepared
CP-K20 wrapper already enables the policy; its separate native-keyframe-aware
coverage/reporting preparation remains, and no new capture started at this
decision. Do not silently relax gates or claim all revisions are preserved.

**CP-K21 standalone Kimera (2026-10-21; COMPLETED ONCE):** read
`experiments/kimera_only_full_rate050_cpk21_20261021/PROTOCOL.txt` and REPORT.txt.
User requests full ETH1 at0.50x, only Kimera, no GLIM/fusion. Exact current
CP-K9 build/profile/flags, receiving OFF, outgoing diagnostics retained.
Full346.32s playback at0.50x completes1492native poses; do not duplicate. Capture root
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_eth1_full_rate050_cpk21_20261021/capture`.
Original5.099973s gap reproduces at EXACT same epochs/seq1483->1484,with50raw
frames per camera inside,frontend processing continuing,backend65--68ms and
LOW_DISPARITY/no-motion logs. Configured5s max keyframe spacing explains it;
not a5s compute stall/camera outage. Eight such stationary-tail intervals.
All1492poses scored:ATE4.496229m,tRPE.440356m,rRPE.567768deg,path196.007m;
savedspan342.598s; shared-CPT7 IE-TC,not independentGT. Whole backend median
74.20ms,p9593.19ms,max141.76ms. Full playback != dense pose coverage;startup/
tail/gaps disclosed. No model edits,retry or automatic follow-on pair. CP-K20
still unlaunched; its coverage/reporting contract must recognize keyframe
semantics explicitly,not silently relax a gate. Noetic/Humble restored stopped.

**CP-K20 full matched finalization run (2026-10-21; prepared, NOT launched):**
Read `experiments/eth1_finalization_full_cpk20_20261021/REPORT.txt` and PROTOCOL.txt.
User authorized full OFF/ON at0.20x with K19 opt-in equally, unchanged K18 sources.
Fresh preparation and expired-accounting archive test pass. BEFORE launch found
K18 native Kimera5.1s pose gap atseq1484 despite continuous raw front cameras.
Unchanged post-startup1s coverage gate/contiguous scoring would reject it.
Asked user whether to authorize explicit-gap complete captures and gap-aware
scoring or retain strict coverage and stop. No gate changes/source run yet.
Do not execute prepared controllers until this choice is resolved. No retry,
model/covariance changes or silent interpolation across missing poses.

**CP-K19 bounded finalization candidate (2026-10-21; saved-prefix gate PASS):**
Read `experiments/receiver_finalization_cpk19_20261021/REPORT.txt` and PROTOCOL.txt.
User authorized an isolated explicit receiver finalization policy after CP-K18:
replace active versions; seal last accepted factor at absorption; audit known
late versions as expired without solver mutation/reinsertion. Default is strict;
opt-in `--absorbed-revision-policy seal_at_absorption`. This intentionally loses
late corrections, NOT lossless full-history/source revision preservation.
Strict50818checks,focused182Release+182ASan/UBSan pass; state/full objective/joint
covariance no-op exact. One saved CP-K18 replay passes214546events+syntheticEOF,
all186856original pre-failure commit records/ONLINE prefix byte-identical.
19lateKimera IDs expire,1active tail revision replaced;71supports/174slots.
Forgone mean max0.071454mm/3.25241e-5rad; covariance relative change max16.87%.
Only20events retained from failure onward; NOT full346s/ON/accuracy validation.
Private binary `/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live`
SHA4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e.
No source/model/covariance/lag/batching changes, live run, push or promotion.
Fresh matched harness needs explicit expired accounting and same opt-in flag
both modes; existing strict guards must not be silently bypassed. Unknown first
beliefs on retired supports and malformed/conflicting messages still fail.
See closeout.json for protected files and stopped-container verification.

**CP-K18 full batched ETH1 (STOPPED/FAILED OFF2026-10-21):** read
`experiments/eth1_batched_full_cpk18_20261021/PROTOCOL.txt` and REPORT.txt.
OFF launched ONCE~00:32 local; startup pass,stopped307.231672s~00:58local.
Same Kimera1459->1460rev24/update1484 after absorption as CP-K14.11.099945s
source history+33.391ms freshness exceeds receiver7s;preceding gap5.099973s.
Not GLIM backlog: last20s scanmedian61.664ms,p9582.624ms,MAIN32.551ms;
4,076,638prepared MAIN consumptions,zeroMAINsync,queue max1 over prefix.
Not a complete346.32s result or OFF/ON accuracy comparison. No ON or retry.
Do not duplicate or use old full scripts.
Exact completed K17 builds/profiles/model: MAIN batching,unchanged CP-K9 Kimera,
all-LOCAL covariance,classic temporaryCBS,FL4e7s no-central-IMU WNOA. Only
duration/output identities change:346.32s,0.20x,headless25sdrain,port11469.
Run root `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared`.
No exhaustive observer,saved-journal repeat,rebuild,tuning or guard relaxation.
2167protected hashes unchanged. Owned jobs exited,containers restored stopped;
follow-up remains PAUSED because failure discovered before activation.
Kimera11.10s/7s retention blocker remains; no repair or next run authorized.
No push/default promotion/other sequence.

**CP-K17 ordinary batched ETH1 matched60s (COMPLETED2026-10-21):** read
`experiments/eth1_batched_live60_cpk17_20261021/REPORT.txt` and PROTOCOL.txt.
User explicitly stopped exhaustive batching validation and requested ONE matched
60s OFF/ON performance run. Both complete at0.20x with frozen K16a MAIN batching,
CP-K9 Kimera,all-LOCAL covariance,classic temporary CBS and unchanged FL4e7s
no-central-IMU WNOA. No paired observer, rebuild, tuning or saved-journal rerun.
NativeGLIM/nativeKimera/ONLINEfusion ATE OFF->ON:.045338->.026153,
.084842->.092451,.057810->.050247m. Same direction on55.90s common mask.
GLIM/fusion ATE improve,Kimera worsens;fusion translation RPE worsens.
All saved591/284/593poses each scored,58.999/59.600/59.200s spans with retained
3.2s initial Kimera gap. IE-TC shares CPT7,not independent prism ground truth.
DELAYED finalized.048819/.034744m separate. Ordinary coverage/lifecycle/LOCAL
payload conservation pass; covariance transport difference0, no pending factors.
MAINmedian10.909/14.505ms;scan46.598/48.205ms;zeroMAINsync. ON2scanoutliers>500ms
did not truncate capture. Not1x/full-sequence or complete math-preservation proof.
K16 incomplete gates and separate Kimera11.10s/7s retention issue remain open.
All owned jobs exited,containers restored stopped,2097protected hashes unchanged.
No default promotion,push,automatic retry or next sequence/run authorized.

**CP-K16h paced correctness gate (2026-10-20; INCOMPLETE):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16H.txt`.
One60s same-prepared-input GLIM-only diagnostic stopped deliberately for
checker cost, not a numerical failure. No retry. Unchanged K16g observer
and K16a/CP-I1/CP-I3 libraries.254scans processed,247native poses spanning
24.60s without gaps; requested34.58s tail missing. NOT a60s pass.
495updates/181retirements match: p/v/bias/equations/covariances0,rotation1.40e-17.
Last20scan waits average30.15s, projecting173more minutes beyond1hwatchdog.
Exact observer substage still unlocalized; do not claim production slowdown.
Private direct-input adapter reuses native scan/IMU processing and idle barrier.
Preflight planned598Hesai/5986IMU with one raw scan buffered; the planned final
20.906ms tail would use original EOS drain, but execution never reached it.
Processed cross-stream insertion matches the paced plan; within-stream records
are intact through the saved prefix. Read-only preflight corrections preserved.
No estimator/settings/math changes or timing/default-promotion claim.
All jobs exited;2026protected/7pinned/56prepared/five source fingerprints
unchanged. Containers restored stopped; see CLOSEOUT_K16H.txt. No further
source run, weakened gate, promotion or push authorized by this pointer.
Kimera11.10s/7s revision-retention blocker remains separate and unresolved.

**CP-K16g sixty-second same-input check (2026-10-20; NOT COMPLETED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16G.txt`.
One capture in k16g_source_1 ended on45s graceful-shutdown timeout (TERM-15).
Exhaustive paired diagnostic MAIN last20median3.046s versus.5s scan budget;
input waits grow, only211native poses with gaps,14.18s tail missing. Separate
observer598Hesai/5986IMU receipt conservation is NOT source processing coverage.
434completed comparisons/140retirements still match: p/v/bias/equations/
covariances0,rotation1.43e-17rad. No arithmetic failure observed, but NO full60s
pass or throughput claim. Exact expensive diagnostic substage not retained.
Original estimator/settings unchanged;2026protected/7pinned/31prepared files
and five source fingerprints pass. Both containers restored stopped; see
CLOSEOUT_K16G.txt in the experiment directory.
No retry/promotion/push. Next proposed correction is paced lossless diagnostic
input with same checks, not more fixed-rate/drain guessing; unexecuted.
Separate Kimera11.10s/7s retention blocker unchanged. Preserve K16f success.

**CP-K16f exact real history (2026-10-20; BOUNDED GATE PASSED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16F.txt`.
After explicit user "Yes", corrected k16f_source_2 eight-second GLIM-only
capture completes143wrapper updates including20retirement updates; first100.
Same independently copied prepared inputs, separate persistent MAIN/LOCAL
histories; unchanged K16a DSO original-disabled versus enabled routes.
Position/velocity/bias/Gaussian/full covariance differences0; rotation roundoff
9.82e-18rad.3266sync->3266prepared requests,106286Gaussian and13476covariance
checks; exact IDs/retirement/inlier/cache state.78Hesai/786IMU,71native outputs
spanning7.00s after startup,69first/2128revised beliefs, allLOCAL provenance.
Not a timing benchmark/full-sequence/CBS-ON certificate or default promotion.
Failed source_1 observer equals(...,0) assertion and K16d fresh-capture failed
equality gate remain preserved. No repeat/overwrite/next run authorized.
2026protected/7pinned/28prepared files,five source fingerprints unchanged.
No estimator/settings/sampling change; both containers restored stopped
(CLOSEOUT_K16F.txt in the experiment directory).
Separate Kimera11.10s/7s revision-retention blocker remains unresolved.

**CP-K16e first real GPU update (2026-10-20; DIAGNOSIS COMPLETE):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16E.txt`.
Two2sGLIM-only captures: first x0/x1 Values exact, but only61.8% of prepared
source points identical. Four independent exact-device sync/batch/actual
equation checks differ0; inlier/cache states exact. Original two-thread
randomgrid_sampling is schedule-dependent (worker RNG/guided assignment);
same scan/resetRNG produced2sets across6calls,one-thread diagnostic1set.
No production preprocessing/model/thread/covariance changes. This localizes
the first live discrepancy upstream, NOT a full60s preservation certificate;
K16d failed gate remains, candidate not promoted. Future exact comparisons
must freeze prepared inputs,not demand equality of fresh stochastic captures.
2026protected/five source fingerprints unchanged;containers stopped,no push.
Separate Kimera11.10s/7s revision-retention blocker remains. No automatic run.

**CP-K16d real60s MAIN timing (2026-10-20; PRESERVATION NOT VALIDATED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16D.txt`.
Both authorized GLIM-only ETH1 captures completed at0.20x, unchanged CP-K14
profiles/LOCAL exports. Scanmedian190.733->45.215ms,p95364.072->169.160ms;
MAINmedian142.633->13.531ms. Actual candidate294972prepared hits,0MAINsync.
BUT pose difference starts at first optimized frame1(1.83mm), RMS.07287m,
max.13881m; candidate52extra revisions. Paired preservation gate FAILED/open;
not a production promotion or proof of a batching bug/nondeterminism cause.
Initialization exact, raw counts/timestamps match; live post-preprocessing GPU
state not frozen. Next proposed exact real FIRST-update comparison needs
authorization, not another full replay.2026protected/five repo fingerprints
unchanged; owned jobs exited,containers stopped,no rebuild/push/default edit.
Separate Kimera11.10s source-revision history/7s receiver mismatch remains.

**CP-K16c MAIN timing (2026-10-20; SYNTHETIC GATE PASSED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16C.txt`.
Unchanged K16a candidate / original CP-I3, six alternating matched timing pairs.
Filled50-pose/630GPU-factor synthetic cycle median332.019->24.867ms,
p95340.159->26.228ms; same derivative work, tested state/equation/covariance
differences0. Three actual MAIN updates/cycle, not an ETH1 scan. Original
sync-warning cost retained; not pure kernel or whole-GLIM speed certification.
2026protected hashes/five source fingerprints unchanged; containers stopped.
No dataset run/library rebuild/default promotion/push. Next proposed matched
60sGLIM-only real-source acceptance needs authorization. Separate Kimera
revision-history/7s receiver mismatch remains; full-sequence readiness unproven.

**CP-K16b MAIN lifecycle (2026-10-20; correctness gate PASSED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16B.txt`.
Unchanged K16a private candidate matches original CP-I3 through actual MAIN
updates/removals/replacements/retirements, partial-clique marginalization and
mixed cached/fresh incremental processing.77wrapper updates,278first/later
LOCAL pair samples; complete state/Gaussian/joint/relative covariance
differences0. Exact disabled control and hostUBSan pass.564sync factor calls
become564prepared consumptions, same derivative requests; NOT a timing result.
2026protected files/five source fingerprints unchanged; containers stopped.
Harness corrections are documented; no solver/library change in K16b.
K16c synthetic timing subsequently passed (above). No dataset
run, default promotion, push or receiver-retention change. Separate Kimera
revision-history/7s retention blocker remains; full-sequence readiness NOT proven.

**CP-K16a MAIN GPU batching (2026-10-20; factor gate PASSED):** read
`experiments/main_gpu_batching_cpk16_20261020/REPORT_K16A.txt` and PLAN.txt.
Private instance-scoped MAIN dispatcher reuses synchronous-compatible VGICP;
no global factory/flag flip or default edit.20Release checks+repeat6,20hostUBSan,
7CPUASan/UBSan pass; tested equations/errors/joint/relative covariance difference0,
exact inlier/cache state. Unchanged GLIM bridge loads the private DSO.2026
protected hashes/five source fingerprints unchanged; both containers stopped.
K16b actual MAIN update/removal/marginalization and CP-I1/CP-I3 provider
parity and K16c synthetic timing subsequently passed (see above).
Stop/report between gates. No dataset, speed claim, promotion or push.
Kimera revision-history versus receiver retention remains separately unresolved.
This pointer does not authorize later gates, source runs or full-sequence retry.

**CP-K15 latency diagnosis (COMPLETED2026-10-20; no repair/run):** read
`experiments/eth1_latency_cpk15_20261020/REPORT.txt` and `evidence_v2.json`.
CP-K14 is NOT evidence that0.10x is required/sufficient: Kimera update1484
follows a5.09997s physical keyframe gap. Its revision history is11.09994s,
freshness.03018s; backend64.08ms. Count-based25-state Kimera versus7s
time-based receiver is the demonstrated retention-contract mismatch.
CP-K13 separately has real GLIM backlog: last10 scans average556ms against
333ms available. MAIN synchronous GPU linearization consumes95% of its
measured smoother time. Loaded bridge's reused glim_ros.cpp.o lacks
BUILD_GTSAM_POINTS_GPU/register_hook; unconditional startup message is
misleading. LOCAL shared-Gaussian/covariance caching remains in use, no
independent LOCAL optimizer. Receiver/transport timings are not the bottleneck.
Read supplement.json for archived build/symbol/runtime-map evidence.
Next proposals are explicit bounded revision-retention contract and isolated
MAIN batching integration with equation/inlier/cache parity checks; ordinary
native GPU hook may change inlier bookkeeping, so no blind global flag flip.
No0.10x retry, lag/guard/model/settings edit, build, promotion or push authorized
by this pointer. Containers remain stopped; heartbeat not reactivated.

**CP-K14 full ETH1 at0.20x (STOPPED/FAILED OFF2026-10-20):** read
`experiments/eth1_full_rate020_cpk14_20261020/PROTOCOL.txt` and `REPORT.txt`.
Explicitly authorized slower-rate test after CP-K13's failure;preserve CP-K13.
OFF launched ONCE~14:42 local;startup effective checks passed. Do not duplicate.
Stopped307.228s on revision after absorption:now KIMERA1459->1460rev24,
sourceupdate1484. First endpoint11.130s old,4.130s before oldest retained
support;rev23 already absorbed. Source timing decomposition unavailable;
do not attribute this to GLIM or assume0.20x solves full-sequence retention.
Queue0/max49,71supports/174slots. No ON/full validation/scoring/retry.
Only playback0.30->0.20,deadline/output identities change;exact CP-K11/K12/K13
binaries/profiles/model,CP-K9 refresh1/FL4q0/remove,all-LOCAL covariance,
classic temporaryCBS,FL4e7s no-central-IMU WNOA unchanged. Full346.32s,
headless25sdrain. Four OFF gates before ON;no retries/guard edits/tuning.
2026protected entries unchanged;follow-up PAUSED,owned jobs exited,containers
restored stopped. Exact evidence/closeout retained. No completed full result,
build/push/promotion/other mission or automatic next experiment.

**CP-K13 full ETH1 at0.30x (STOPPED/FAILED OFF2026-10-20):** read
`experiments/eth1_full_rate030_cpk13_20261020/PROTOCOL.txt` and `REPORT.txt`.
OFF stopped136.449s on `FL4A_STREAM_FAILURE revision after absorption`.
GLIM1286->1288rev47 arrived after rev46 absorption:4.900s source history+
2.129s producing-update delay=7.029s age,exceeding7s receiver retention.
Last sampled queue0,max40;not receiver queue overflow. No ON/retry/scoring.
1956protected hashes unchanged;failure_evidence.json retains exact event.
Follow-up PAUSED;containers restored stopped after owned cleanup.
Startup effective checks passed. Do not duplicate it. Exact selected-topic
window346.32s includes last Hesai record (old346.22 label used different epoch).
Same CP-K11/K12 binaries/profiles/model,prior-refresh1,FL4q0,all-LOCAL covariance,
classic temporaryCBS,7s no-central-IMU WNOA,headless0.30x/25sdrain.
No automatic retry,guard/model change,tuning or other mission.
No completed full result or accuracy claim. Need a separate next decision.

**CP-K12 60s speed gate (PASS2026-10-20):** read
`experiments/eth1_speed60_cpk12_20261020/REPORT.txt` and `PROTOCOL.txt`.
One explicitly requested ETH1 ON at0.30x,identical CP-K11 builds/model/profiles,
headless60s from0,25sdrain. Completed245.656wallseconds,591GLIM/284Kimera/
593ONLINE outputs;all four validations pass. GLIM scanmedian193/p95366ms,
Kimera backend73/93ms,receiver cycle50/83ms. GLIMqueue max2,receiverqueue
max42 then0;oldest GLIMrevision endpointage5.332s<7s. No recovery/lost factors.
Same actual-node-log correction included before launch,no numerical relaxation.
1886protected hashes unchanged;saved journal35521archive files+5products exact.
Not a full-sequence/1x/accuracy comparison or default promotion. Future OFF/ON
must use matched rate;no automatic next run. Latest accuracy pair remains CP-K11.

**CP-K11 matched live120 comparison (COMPLETED2026-10-20):** read
`experiments/kimera_prior_matched_live120_cpk11_20261020/REPORT.txt` and
`PROTOCOL.txt` and `VALIDATION_BLOCKER.txt` there before acting. BOTH captures
completed;do not duplicate them. Original ON checker failed on wrong-log routing;
user-requested postprocessing correction preserves it and uses actual node log
with unchanged thresholds. Both lifecycle checks and saved-journal repeats PASS.
ON556refreshes match556retirements,maxcacheerror1.65e-9,no recovery/errors.
Full-saved nativeGLIM ATE OFF/ON.028690/.034553m,nativeKimera.463011/.156114m,
fusionONLINE.047754/.078585m. DELAYEDfinalized.041371/.073055m is separate.
Kimera ATE/tRPE improve,GLIM ATE worsens although both RPE improve;fusionONLINE
ATE/rRPE worsen. Same ATE direction on common mask. ALL saved poses scored
with shared-CPT7 IE-TC;~119s output spans,existing3.2sKimera startup gap disclosed.
Not full346s/1x/stationarity or calibrated/independent covariance certification.
CP-K9 private Kimera in BOTH modes,prior-refresh1,FL4q0,
remove; CP-R6 all-LOCAL GLIM/ETH1 profiles and FL4e7s no-central-IMU WNOA
unchanged. Native GLIM,Kimera and fuser run concurrently,120s at0.10x.
No automatic retries,tuning,full sequence or promotion. Follow-up remains paused;
1825protected hashes unchanged. See final_checks.json for stopped-container
closeout. No new estimator run or settings change during postprocessing.

**CP-K10 bounded real-data prior gate (completed2026-10-20):** read
`experiments/kimera_incremental_prior_live_cpk10_20261019/REPORT.txt` first.
One111s Kimera-only ON with frozen CP-K5 GLIM messages completes539outputs,
511temporary retirements, no recovery/rollback/FL4q. CP-K9 private candidate,
CPK9_PRIOR_REFRESH=1,FL4Q=0,remove; same profiles/weights/guards. First28objective
rows identical to CP-K5; change starts at first retirement. Prior-cache parity
max5.34e-10; old internal IMU-error runaway not reproduced. All-saved native
Kimera OFF/conditionalON ATE.306224/.129063m;translation RPE improves,rotation
RPE slightly worsens.110.599s saved span with existing3.2sstartup gap; IE-TC
shares CPT7. NOT a fresh live bidirectional or fusion comparison/full certificate.
Original post-check's1100expected was a100ms query-start error: actual1099all
exact. Separate exact-window checker passed; original/capture preserved. No
source retry/tuning/default promotion/push. Both containers stopped. Preserve
the isolated candidate for separately authorized next work; no automatic run.

**CP-K9 incremental prior candidate (2026-10-19):** read
`experiments/kimera_incremental_prior_cpk9_20261019/REPORT.txt` before follow-up.
Isolated private Kimera build passes actual BPSAM incremental/removal/marginal
integration, original-prior energy/cache/tree consistency, exact synthetic OFF
parity, smart-cache/LOCAL transaction rollback, repeat and focused sanitizers.
At real temporary-CBS retirement only, wraps native Gaussian marginals in the
existing immutable OriginalChartPrior and releases verified owning fixed keys;
ordinary iSAM2 then refreshes. No FL4q global LM/core rebuild or guard removal.
Opt-in CPK9_PRIOR_REFRESH=1 requires explicit FL4Q_ENABLE=0,retirement=remove;
CP-K8 common settings unchanged. Installed/default and prior private builds
untouched. NO real-data validation or full-run admission: next proposed gate
is saved-input Kimera-only111s ON past CP-K5's102.199s stop, not automatic full
retry. No source/covariance tuning, launch/controller edit, commit or push.

**CP-K8 next-experiment settings lock (2026-10-19):** read
`experiments/next_experiment_settings_cpk8_20261019/REPORT.txt` and
`settings.lock.json` there before preparing another launch.30ETH1 profile files
match CP-R6 OFF/ON and CP-R7; effective source settings and loaded private paths
match between modes. Pins all-LOCAL CP-I1/CP-I3, temporary classic CBS, Gaussian1,
FL4e7s no-central-IMU WNOA,100ms supports and headless0.10x. Nested launch sets
FL4Q explicitly: parent-shell override is insufficient. Temporary-linear flag
false is intentional, not a missing temporary-mode switch. FULL-RUN ADMISSION
REMAINS BLOCKED: neither existing FL4Q setting validates both modes;0 is only
the ordinary diagnostic reference,1 retains the known longer-OFF failure.
No asymmetric OFF/ON backend, persistent-mode substitution, tuning or guard
bypass. Read-only verify_settings.py checks hashes; --require-ready exits2.
Settings preparation did not run or fix an estimator. Preserve prior evidence.

**CP-K7 existing-fix reconciliation (2026-10-19):** read
`experiments/kimera_fix_lineage_cpk7_20261019/REPORT.txt` FIRST before repair.
June19 temporary-factor rollback/retry is still present and executed in CP-K5;
the controller stopped on that recovery log, not a Kimera process crash.
Large prior IMU error prevents calling this healthy or simply bypassing guard.
July diagnosed related fixed-lag behavior even stock OFF; no general production
solver repair was selected. FL4q already implements original-prior-preserving
current-state LM/core rebuild: enabled in FL4v/CP-R6/R7, deliberately disabled
in CP-K4b/K5. CP-K6 reuses its exact prior helper; do not reinvent this operation
or imply it is entirely absent from existing work. Unresolved: a validated
integration/acceptance policy compatible with native smart factors, without
silently changing ordinary OFF. CP-K2 remains on hold. Documentation-only audit;
no new repair/replay authorized, no guard or covariance changes.

**CP-K6 bounded prior/native-refresh gate (2026-10-19):** read
`experiments/kimera_prior_refresh_cpk6_20261019/REPORT.txt` FIRST before repair.
Small actual iSAM2 removal/marginalization + Pose3/velocity/bias/IMU fixture
passes dense Schur, immutable original-prior energy, fresh derivative, repeat
and focused sanitizer checks. Reuses FL4p original-chart reference helper.
This is NOT integrated in Kimera and NOT a convergence/full-ETH1 certificate.
No source code/settings/covariance changes or dataset replay. Separate narrow
incremental theta/delta/cache/prior-lifecycle integration remains necessary;
do not blindly reset prior anchors/unfix keys or revive FL4q's global wrapper.
No automatic run or production promotion is authorized by this pointer.

**CP-K5 ordinary temporary-ON diagnostic (2026-10-19):** read
`experiments/kimera_incremental_on_cpk5_20261019/REPORT.txt` FIRST.
Same ordinary FL4Q=0 backend as completed CP-K4b OFF; one111s-bounded Kimera-only
run with saved FL4v incoming GLIM payloads, not live bidirectional exchange.
Stopped at102.199s/update495 on existing active_window_rollback:
staged edge residual .149932->.538649 exceeded .5. No FL4q call/process crash;
IMU error had grown severely beforehand. No completed111s result or ATE table.
Exact wire prefix and dense covariance provenance checked. No retry, tuning,
guard edits, production/default promotion or full experiment. Both containers
stopped. Ordinary OFF success does not certify temporary ON; preserve evidence.

**CP-K4 ordinary-incremental OFF control (2026-10-19):** read
`experiments/kimera_incremental_control_cpk4_20261019/RUN_REPORT.txt` FIRST.
After a separately reauthorized launch,111s ETH1 Kimera-only OFF completes
with existing FL4Q_ENABLE=0 and539 finite native outputs. First44 messages are
byte-identical to CP-K1; divergence44 is the first prior wrapper intervention.
Same-mask ATE oldFL4q/ordinary .687725/.288268m; full-saved ordinary .306224m.
It passes old stop531, but does NOT certify temporary-ON or full ETH1 repair.
No estimator rebuild, tuning, ON/fusion or default promotion. CP-K2 remains on
hold. Prior harness-only failed preflight stays in REPORT.txt. Preserve both
records; do not infer authorization for another run or asymmetric OFF/ON solvers.

**CP-K3 backend-semantics audit (2026-10-19):** read
`experiments/kimera_classic_backend_audit_cpk3_20261019/REPORT.txt` FIRST before
further Kimera repair. The user paused landmark-solver redesign to check classic
behavior. Historical/current ETH1 BackendParams are byte-identical, but FL4q
replaced17 OFF updates starting44 with ZERO CBS retirements before stop531.
This is an added backend policy, not unchanged historical Kimera. Separate its
smart-factor acceptance mismatch from FL4m/FL4p's real temporary-ON old-anchor
instability. CP-K2's proposed joint-landmark follow-up below is ON HOLD.
Next proposed control: bounded ordinary-incremental OFF via existing disabled
selector, not a new solver, full retry or production guard bypass. No run or
backend edit is authorized by this pointer; no repair has been certified.

**CP-K2 proposed Kimera repair gate (2026-10-19):** read
`experiments/kimera_consistent_landmark_cpk2_20261019/REPORT.txt`. The standalone
conditional-landmark candidate corrects derivatives on171/173 valid saved
factors but fails on two tracks;106675's conditional solve escapes toward infinity.
Do not install it, relax its gates or retry full ETH1. No production code or
source run was changed. The next bounded candidate is joint pose/temporary
landmark fallback consistency, not a presumed finite conditional minimum.
This pointer is not an authorization for integration or replay.

**CP-K1 Kimera OFF stop localization (2026-10-19):** read
`experiments/kimera_failed_factor_cpk1_20261019/REPORT.txt` before retrying
CP-R7 or changing FL4q stopping. One bounded diagnostic reproduced all531
native messages exactly. The derivative mismatch is the visual smart-factor
held-landmark error versus Schur-eliminated landmark model, not IMU/bias/prior;
fresh triangulation alone does not fix it. Production repair is NOT implemented
or certified. Do not repeat the same diagnostic, relax the guard, tune covariance
or launch full OFF/ON from this pointer. Saved visual factors support follow-up.

**CP-A2 actual peer alignment audit (2026-10-19):** read
`experiments/aligned_peer_covariance_cpa2_20261019/REPORT.txt`. EIG-2 CP-R3
292G2K/293K2G installed matrices equal first-source LOCAL covariances exactly.
Native mean alignment LZR leaves covariance unchanged: no Ad(R^-1) source
transport or bridge-uncertainty propagation. This confirms the existing
mean-only approximation, not a new covariance substitution/cache regression.
Alignment helps versus using raw means at wrong receiver timestamps. Separate
real L/R/joint uncertainty are not directly logged; magnitude/accuracy causality
is unestablished. No fix/tuning/rerun performed or authorized by this pointer.
Keep peer alignment distinct from gnssFGO's original-endpoint CT source factors.

**CP-A1 saved-data covariance/error audit (2026-10-19):** read
`experiments/local_covariance_error_cpa1_20261019/REPORT.txt` before interpreting
the new all-LOCAL CP-R4/R3,CP-R5,CP-R6 uncertainty. Matched physical endpoints,
first/latest-common/exact-update-age cohorts;206133versions checked, inputs
unchanged. No universal CBS-ON collapse: Kimera rho trace increases on EIG-2/
ETH1, decreases on SPX-3; GLIM changes little. Calibration/ranking limitations
depend on sequence,block,revision age. Shared-CPT7 IE-TC discrepancies are not
true NEES/NIS. No tuning/optimizer run or scale selected. Actual aligned peer
factor consistency (CP2) and receiver information competition remain separate
gates; this pointer does not authorize execution or covariance changes.

**CP-R6 matched ETH1 all-LOCAL60s (2026-10-19):** read
`experiments/local_covariance_eth1_cpr6_20261019/REPORT.txt`. Same cachedLOCAL
CP-I1/CP-I3, FL4q Kimera, FL4e7s no-central-IMU WNOA; ETH1 archived profiles,
bag0--60s,headless0.10x. Both captures/provenance/conservation/replay pass.
Full-saved ATE OFF/ON originalGLIM.046001/.044886m,originalKimera.138290/.107645m,
fusionONLINE.049713/.066941m. Kimera ATE improves but1sRPE worsens; fusion
ATE/RPE worsen. GLIM small ATE gain reverses on common mask. Known3.2sKimera
startup gap disclosed; all saved poses scored with IE-TC. Delayedfinalized
separate. No tuning, default promotion, independence/1x certification or new run.

**CP-R5 matched SPX-3 all-LOCAL60s (2026-10-19):** read
`experiments/local_covariance_spx3_cpr5_20261019/REPORT.txt`. Same cachedLOCAL
CP-I1/CP-I3, FL4q Kimera, FL4e7s no-central-IMU WNOA as CP-R4/R3; SPX-3's
saved profile, bag32--92s, headless0.10x. Both captures/provenance/conservation/
replay pass. Full-saved ATE OFF/ON originalGLIM.010246/.011237m, originalKimera
.530451/.070118m, fusionONLINE.054648/.035018m. Delayed finalized separate.
No tuning, default promotion, calibration/independence/1x certification or
authorization for another run. Preserve EIG-2's different CP-R4 result.

**2026-10-19 design decision (private GLIM candidate tested; not promoted):** read
`GNSSFGO_LOCAL_COVARIANCE_POLICY_20261019.txt` before covariance-policy work.
The user selected LOCAL covariance for every outgoing belief version, including
downstream revisions, while preserving corrected MAIN means. Do not claim the
frozen FL4v/FL4w/FL4x captures already satisfy this: GLIM revisions used MAIN
covariance. The isolated CP-I1 candidate/tests are documented in
`experiments/local_revision_covariance_cpi1_20261019/REPORT.txt`. CP-I2 closes
Kimera's explicit native-information prior gate for temporary/remove mode:
`experiments/kimera_native_prior_cpi2_20261019/REPORT.txt`. It is not an
independence certificate. CP-R1 is the newly authorized sequential60s campaign;
read `experiments/local_covariance_cpr1_20261019/REPORT.txt` for actual status.
Do not duplicate an active capture or promote a baseline before live validation.
CP-I3's isolated per-epoch query-cache repair passes tests and saved-equation
timing: `experiments/native_covariance_cache_cpi3_20261019/REPORT.txt`.
CP-R2 now passes one fresh30s EIG-2 OFF live capture with this cache:
`experiments/local_covariance_live_cpr2_20261019/REPORT.txt`. Exact LOCAL payload
conservation and deterministic replay passed; matched revision median1.024s
fell to.086s, no late-absorption failure. This does NOT certify60s/ON/1x operation
or promote defaults; no matched campaign was resumed in this gate.
CP-R3 subsequently passes one explicitly requested standalone60s EIG-2 ON:
`experiments/local_covariance_on60_cpr3_20261019/REPORT.txt`. All-LOCAL payloads,
both CBS directions, retention and byte-identical replay pass; no matched60s
cached-policy OFF exists yet. Full saved native GLIM/Kimera/ONLINE fusion ATE
is.025337/.138205/.060148m against shared-CPT7 IE-TC; delayed finalized.045736m
is distinct. No tuning/default promotion/1x or general latency certification.
This is not permission for a new run or a claim of statistical independence.

**CP-R4 matched EIG-2 all-LOCAL60s result (2026-10-19):** read
`experiments/local_covariance_off60_cpr4_20261019/REPORT.txt`. A fresh60s OFF
now matches CP-R3 ON's builds/profiles/covariance policy; ON was not repeated.
Full-saved ATE OFF/ON: original GLIM.020797/.025337m, original Kimera
.081560/.138205m, fusion ONLINE.043669/.060148m. Delayed finalized is separate.
Coverage, provenance, conservation and deterministic replay passed. Use this
pair for the new policy, not old mixed-policy OFF. No default/1x promotion,
new run authorization, calibration claim or tuning follows from this result.

**Latest saved launch/settings (2026-10-17):** read
`GNSSFGO_GRANDTOUR_FL4V_LAUNCH_20261017.txt` FIRST for current ETH1 work, plus
`experiments/checkpoints/20261017_fl4v/README.txt`. This freezes the actual
FL4v private builds: temporary classic CBS, shared-native GLIM covariance,
FL4q Kimera, FL4e 7-second fixed-lag WNOA without central IMU, scales1, 0.10x.
It supersedes older configuration pointers below for this experiment, NOT
their historical evidence. The120s ON completed; the OFF reference stopped
early and only a105.4s common prefix is compared. No automatic retry, new run,
default promotion or claim that the OFF failure is fixed is authorized.

For the current incremental runtime engineering work, first read and update
`GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md`. Identify its task ID before
editing, and record evidence/status after each task. Do one hypothesis per patch;
do not silently execute the whole plan. Recovery sources/build instructions are
in `experiments/checkpoints/20261011/README.md`. This latest snapshot uses classic
`active_window_temporary` sources and source-driven WNOA without central IMU
factors; the older baseline descriptions below are distinct configurations.

For the latest bounded ETH1 fixed-lag live OFF/ON milestone, read
`experiments/fixed_lag_live_fl4f_20261014/MILESTONE.md` and its report before
reusing settings. It archives the exact source/receiver profiles and compact
validation evidence. The FL4e receiver repair remains an isolated candidate,
not the default build; the 30 s result does not certify longer or 1x operation.

Before running or interpreting CBS-ON GLIM/Kimera source experiments, read
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md`.
It is the current operational reference for the shared classic CBS protocol
and the saved sequence-specific profiles. Do not silently substitute the
experimental source-to-source `live_revision` protocol.

Keep source CBS exchange separate from downstream gnssFGO stable-ID revision
replacement. For OFF/ON reporting, the primary comparison is always the
original/native GLIM trajectory, original/native Kimera ROS-odometry
trajectory, and fused gnssFGO trajectory in each mode. gnssFGO GLIM-only and
Kimera-only graphs are diagnostic receiver controls, not native trajectories.
Distinguish the receiver's causal as-played output from its final
retrospectively revised output, and never label offline replay as a live
co-running receiver.

Verify effective launch parameters, copied profiles, input identities, and
evaluation masks for each sequence. This instruction does not authorize a
new run, protocol change, or covariance tuning.

For the 2026-10-07 source-preservation checkpoint and latest experimental WNOA
build, read `experiments/checkpoints/20261007/README.md`. That archive preserves
the actual diagnostic build separately from the default CMake target, including
the live-run profiles and validation limits. Do not infer the latest experiment
from the older WNOJ/default-build paragraphs below.

For live/asynchronous GrandTour receiver work, read
`GNSSFGO_GRANDTOUR_CLASSIC_LIVE_BASELINE_20260929.md` and
`GNSSFGO_GRANDTOUR_LIVE_RECEIVER_RUNBOOK.md`, then use the saved
`config/grandtour_live_receiver.json` template. Source count is configurable,
not fixed at the synthetic test's four LiDAR/three camera example. Keep the
independent IMU-driven support clock separate from source arrival times.
The current live adapter observes full diagnostic revision logs while classic
source CBS remains unchanged. The matched SPX-3 120-second OFF/ON run validated
co-running sources and receiver at 0.10x with graph inspectors disabled; it is
not evidence of 0.3x, 0.5x, or 1x capacity or a universal sequence profile.
Do not call final retrospective paths live.

An opt-in source-driven WNOJ receiver is documented in
`GNSSFGO_SOURCE_WNOJ_IMPLEMENTATION_20261005.md`. It estimates acceleration
and omits central raw-IMU factors/initialization; it is only synthetically
validated, not a replacement for the frozen live baseline. Default mode remains
`imu_wnoj`. Do not enable `source_wnoj` in an experiment without authorization,
or mistake the older `--audit-use-imu false` control for this new model.
