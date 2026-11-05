"""Generate the CP-V8 evidence tables and static figures; no estimator calls."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import analyze as v

H=v.H;O=v.OUT;E=v.E


def pct(old,new):return 100*(new/old-1)


def table(headers, rows):
    rows=[[str(x) for x in r] for r in rows]
    widths=[max(len(str(h)),max((len(r[i]) for r in rows),default=0)) for i,h in enumerate(headers)]
    fmt=lambda r:'  '.join(str(s).ljust(w) for s,w in zip(r,widths))
    return '\n'.join([fmt(headers),fmt(['-'*w for w in widths])]+[fmt(r) for r in rows])+'\n'


def design_check():
    rng=np.random.default_rng(20261026)
    maxenergy=0.;maxsym=0.;mineig=1.
    for _ in range(1000):
        A=rng.normal(size=(6,6));C=A@A.T+np.eye(6)*.2
        S=np.diag(np.exp(rng.uniform(-3,0,6)));C=S@C@S
        g=rng.normal(size=3);g/=np.linalg.norm(g)
        M=np.zeros((4,6));M[0,:3]=g;M[1:,3:]=np.eye(3)
        Cy=M@C@M.T;W=np.linalg.inv(C);Wy=np.linalg.inv(Cy)
        assert np.linalg.eigvalsh(Cy).min()>0
        D=W-M.T@Wy@M
        e=np.linalg.eigvalsh((D+D.T)/2).min()/np.linalg.norm(W,2)
        mineig=min(mineig,e);assert e>-1e-10
        _,_,VT=np.linalg.svd(M);N=VT[4:].T
        y=rng.normal(size=4);r0=M.T@y
        z=-np.linalg.solve(N.T@W@N,N.T@W@r0)
        r=r0+N@z
        q=float(y@Wy@y);delta=abs(float(r@W@r)-q)/(1+q)
        maxenergy=max(maxenergy,delta);assert delta<1e-9
        maxsym=max(maxsym,float(np.max(np.abs(Cy-Cy.T))))
    return dict(passed=True,dense_random_cases=1000,
                maximum_relative_projected_vs_minimized_cost_error=maxenergy,
                minimum_normalized_information_difference_eigenvalue=mineig,
                maximum_covariance_symmetry_error=maxsym,
                scope='Only proposed fixed-projection Gaussian algebra, NOT real factor Jacobians, chart mapping, runtime lifecycle, implementation or nonlinear validation.')


def main():
    assert not (H/'REPORT.txt').exists(),'Refuse report overwrite'
    v.begin();v.a.pin(Path(__file__));v.a.pin(H/'PROPOSAL.txt')
    s=json.loads(v.a.pin(O/'summary.json').read_text());cases=s['cases']
    timing=json.loads(v.a.pin(E/'four_sequence_timing_cpt1_20261023/timing_v2.json').read_text())
    cpv7=json.loads(v.a.pin(E/'leica2_drift_attribution_cpv7_20261025/outputs/metrics.json').read_text())
    v.a.pin(E/'leica2_drift_attribution_cpv7_20261025/REPORT.txt')
    v.a.pin(E/'velodyne_full_campaign_cpv4_20261024/FINAL_AUDIT.txt')
    labels=[f'{r[1]} {r[2]}' for r in v.ROWS]
    ordered=[cases[r[0]] for r in v.ROWS]
    parts=[]
    def add(x):parts.append(x.strip()+'\n')
    add('''CP-V8 — HESAI / VELODYNE CROSS-SEQUENCE DECISION REPORT
Completed 2026-10-26. Saved-data analysis; no new estimator experiment.
====================================================================

EXECUTIVE DECISION
The system demonstrates substantial collaborative position-estimation gains,
but is NOT yet a robust, calibrated uncertainty-sharing method. Preserve the
baseline; do not promote the rejected whole-rotation variance10 candidate.
Do not start another broad dataset campaign or covariance-scale sweep.

The important NEW finding is that Kimera's20s rotation RPE worsens with CBS ON
in SIX of SEVEN completed pairs, although position ATE improves in six. Only
Hesai ETH1 improves both Kimera position ATE and20s rotation RPE. Several
apparently successful position results therefore hide an orientation tradeoff.
This is a descriptive result from these captures, not a significance test.

Actual incoming GLIM->Kimera factors show worse tilt RMS than the verified
Kimera pre-update state in SIX pairs, but better heading RMS in FIVE. These
sets are not identical; Hesai EIG-2 is an important counterexample. The
evidence supports investigating directional information exchange, not a
universal claim that LiDAR is always bad at tilt or good at heading.

Recommended next intervention: ONE parameter-free, gravity-separated G2K
information ablation that retains gravity-axis rotation and translation but
marginalizes the two tilt residual directions. Keep K2G and fusion unchanged.
This is a causal diagnostic proposal, NOT an implemented fix or final adaptive
policy. It avoids inventing a trust threshold from unreliable covariance ranks.
The full mathematical specification, stop rules and future acceptance contract
are appended in Section10 and also saved as PROPOSAL.txt. Nothing was launched.

1. POPULATION, FAIRNESS AND WHAT 'WORKS' MEANS
---------------------------------------------
Seven completed baseline OFF/ON pairs: four Hesai and three rear-Velodyne.
Four distinct missions, not seven independent environments or repetitions.
SPX-3 uses corrected quiet-start CP-K28, NOT obsolete moving-start CP-K26.
EIG-2 Hesai uses CP-K24A's qualified EOF-reviewed scores, not a zero-loss claim.
The existing CP-V6A Leica variance10 intervention is separate negative evidence.

Freeze: 0.20x, MAIN batching, GLIM5s/all-LOCAL dense covariance every version,
corrected MAIN means, CP-K9 Kimera25states/prior-refresh1/FL4Q0/remove,
classic active_window_temporary CBS at Gaussian scale1, seven-second WNOA
fusion/100ms supports/no central IMU or GNSS, seal_at_absorption and explicit
EOF handling. Source-to-source exchange is not downstream revision replacement.

Success is measured OFF->ON improvement with acceptable native-source harm,
long-horizon accuracy, coverage and safe completion. Fusion need not beat the
best native estimator in every run. Conversely, better GLIM cannot cancel a
large Kimera regression without an explicitly different application objective.
Engineering correctness, calibrated uncertainty, accuracy and throughput are
separate questions. Passed payload/lifecycle checks are not an accuracy proof.

Reference: shared-CPT7 IE-TC GNSS/INS, not independent/noiseless prism GT.
Trajectories are evaluated at the established prism lever arm, with rigid
SE(3) no-scale best-fit ATE. RPE is relative motion, not globally aligned ATE.
Rear Velodyne changes mounting/viewpoint/density as well as the sensor; this
is not an isolated experiment varying only 'LiDAR accuracy'.

The three cross-sensor Kimera OFF trajectory files are BYTE-IDENTICAL. That
supports the unchanged camera/IMU baseline, but also prohibits counting them
as independent repetitions. No standard errors/p-values are inferred from
thousands of overlapping time windows or repeated covariance revisions.

2. POSITION ACCURACY — ORIGINAL FULL-SAVED RESULTS
-------------------------------------------------
ATE RMSE metres, OFF -> ON. These reproduce the historical report values.
Every saved pose is scored; streams can have different timestamps and spans.
Change is100*(ON/OFF-1): negative improves. Tiny changes are not proof of gain.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        ev=c['trajectories']['original_evaluation']['results'];r=[label]
        for role in v.ROLES:
            x=ev['off_'+role]['full_saved']['ATE_RMSE_m'];y=ev['on_'+role]['full_saved']['ATE_RMSE_m']
            r.append(f'{x:.4f} -> {y:.4f} ({pct(x,y):+.1f}%)')
        rows.append(r)
    add(table(['Pair','Native GLIM','Native Kimera','Fusion ONLINE'],rows))
    add('''Hesai: Kimera and ONLINE fusion position ATE improve4/4. Velodyne: each
improves2/3, but on different subsets. Do not collapse this to a success rate.
Leica Velodyne GLIM gains while Kimera is damaged; EIG Velodyne fusion worsens.
Hesai SPX-3's0.000017m GLIM ATE change is effectively unchanged at this evidence
level. No repeated-trial variance estimate supports a claim about such a gain.

New CP-V8 same-time common-mask ATE, metres (native G/K/ONLINE all share mask):''')
    rows=[]
    for label,c in zip(labels,ordered):
        m=c['trajectories']['new_common_metrics']
        rows.append([label]+[f"{m['off_'+r]['common_ATE_m']:.4f} -> {m['on_'+r]['common_ATE_m']:.4f}" for r in v.ROLES])
    add(table(['Pair','GLIM','Kimera','Fusion ONLINE'],rows))
    add('''All Kimera and ONLINE fusion ATE direction conclusions survive matched
coverage. GLIM Hesai SPX-3 is effectively unchanged: its tiny full-saved gain
reverses to a tiny common-mask increase, so it is NOT counted as a robust gain.
Absolute full-saved
and common-mask ATE differ because samples and fitted alignment differ; these
are not rerun trajectories. An integer-nanosecond grid avoids the old float
arange endpoint drift. One or two boundary queries differ in some cases;
original scores/masks remain preserved alongside this newly declared analysis.

3. THE MISSING PART OF THE VERDICT: MULTI-HORIZON MOTION ERROR
------------------------------------------------------------
Kimera rotation RPE RMS degrees, OFF -> ON; complete common support throughout
each interval. No interval crosses a missing/unsupported sample. The20s result
worsens in6/7pairs, including three Hesai position-success cases.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        m=c['trajectories']['new_common_metrics'];o=m['off_native_kimera'];n=m['on_native_kimera']
        rows.append([label]+[f"{o['horizons'][str(h)]['rotation_RMS_deg']:.3f} -> {n['horizons'][str(h)]['rotation_RMS_deg']:.3f}" for h in (1,10,20)]+[f"{pct(o['horizons']['20']['rotation_RMS_deg'],n['horizons']['20']['rotation_RMS_deg']):+.1f}%"])
    add(table(['Pair','1s','10s','20s','20s change'],rows))
    add('''The same-start sensitivity check fixes each pair's start cohort to those
supporting an uninterrupted20s interval, then evaluates1/10/20s on those
EXACT starts. Six-of-seven20s deterioration is unchanged by construction;
the shorter-horizon same-cohort scores are retained in metrics. Counts below.
Do not interpret changing horizon cohorts as a precise crossover time.

Full multi-horizon table below: translation metres / rotation degrees.
These are new common-mask RPE, not original per-stream1s scores.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        m=c['trajectories']['new_common_metrics']
        for role in v.ROLES:
            for mode in v.MODES:
                d=m[mode+'_'+role]
                rows.append([label,role,mode]+[f"{d['horizons'][str(h)]['translation_RMS_m']:.4f} / {d['horizons'][str(h)]['rotation_RMS_deg']:.4f}" for h in (1,10,20)])
    add(table(['Pair','Output','CBS','1s m/deg','10s m/deg','20s m/deg'],rows))
    add('''Sequence decisions:
- Hesai ETH1: strongest positive evidence; large Kimera translation and
  position gains, plus lower20s rotation error. Preserve as a useful baseline.
- Hesai EIG-2: large position/translation gains, but Kimera20s rotation rises
  ~19%. Incoming tilt is slightly healthier here; simple tilt harm cannot
  explain every orientation regression.
- Hesai SPX-3 and Leica-2: strong Kimera position gains with ~22% worse20s
  rotation. Collaboration is useful, but not an across-the-board improvement.
- Velodyne SPX-3: modest position gains; Kimera20s rotation more than doubles,
  and ONLINE fusion20s translation worsens. Classify mixed, not clean success.
- Velodyne EIG-2: small Kimera ATE gain hides20s rotation rising1.875->4.835deg;
  fusion ATE and20s translation worsen. Clear rejection as robust improvement.
- Velodyne Leica-2: substantial GLIM recovery but Kimera deteriorates in ATE,
  short/long relative motion. ONLINE ATE modestly improves. Harmful transfer
  is plausible, but bidirectional feedback prevents isolated causal attribution.

4. ACTUAL FACTOR UNCERTAINTY VERSUS ACCURACY
------------------------------------------
10,609 actual G2K factors joined to their installed dense covariance and exact
Kimera same-edge pre/post snapshots. All covariance differences are0; all
pre-states reproduce the logged residual/whitened diagnostic within tolerance.
This is not an asynchronous-odometry proxy or covariance interpolation.
Kimera pre-state already contains past collaboration/native factors/prior;
it is NOT a counterfactual CBS-OFF estimate. Post-update changes include ALL
factors, retirement and feedback, not just the newly injected belief.

Angular error RMS degrees at each receiver interval, incoming GLIM / Kimera
PRE-update. Heading is projected CPT7-X yaw; tilt is horizontal ENU angular-log
norm. No per-belief re-alignment to make either source look better.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        d=c['peers']['all'];inc=d['incoming'];pre=d['pre']
        rows.append([label,d['n'],f"{inc['heading_deg_rms']:.4f} / {pre['heading_deg_rms']:.4f}",f"{inc['tilt_deg_rms']:.4f} / {pre['tilt_deg_rms']:.4f}",f"{100*d['incoming_better_fraction']['heading_deg']:.1f}%",f"{100*d['incoming_better_fraction']['tilt_deg']:.1f}%"])
    add(table(['Pair','Factors','Heading G/Kpre','Tilt G/Kpre','G heading better','G tilt better'],rows))
    add('''The distinction generalizes beyond Leica-2: incoming tilt RMS is worse6/7,
heading RMS better5/7. EIG-2 heading is similar or worse, so no universal
'trust LiDAR heading' rule follows. EIG-2 also has larger platform pitch;
Euler heading and the vertical angular-log component differ there. Both are
retained separately; the covariance projection below uses the angular-log axes.

Actual angular marginal uncertainty, degrees, G/Kpre, expressed in nominal
predicted ENU axes. Vertical is sqrt(Czz); tilt norm is sqrt(Cxx+Cyy).
This projection is NOT a complete residual/bridge covariance propagation.
The last two columns are within-run Spearman ranks between incoming uncertainty
and actual angular-component error. They describe ranking, not calibration.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        d=c['peers']['all'];x=d['incoming'];y=d['pre']
        rows.append([label,f"{x['vertical_std_deg_rms']:.5f} / {y['vertical_std_deg_rms']:.5f}",f"{x['tilt_norm_deg_rms']:.5f} / {y['tilt_norm_deg_rms']:.5f}",f"{x['vertical_deg_covariance_rank']:.3f}",f"{x['tilt_deg_covariance_rank']:.3f}"])
    add(table(['Pair','Vertical uncertainty G/Kpre','Tilt uncertainty G/Kpre','G vertical rank','G tilt rank'],rows))
    add('''A smaller covariance is not a reliable source selector. For Velodyne EIG-2,
the lower-tilt-uncertainty choice selects incoming GLIM on~81% of intervals,
but GLIM tilt is more accurate on only~21%. The selected tilt RMS is0.339deg
versus0.214deg for always using Kimera's PRE state. This is an offline selector
diagnostic, NOT a fused trajectory or evidence that an executable gate helps.
For Leica Velodyne, the corresponding selected tilt RMS is0.174deg versus
Kimera PRE0.116deg. SPX-3 has substantially better covariance/error ranking;
there is no universal conversion from a covariance trace to estimator health.

Same physical receiver-endpoint sensitivity across sensors:
- SPX-3:881 matched pairs (46Hesai/49Velodyne excluded).
- EIG-2:1,637 matched pairs (103/107 excluded).
- Leica-2:1,458 matched pairs (432/447 excluded).
On matched Leica intervals, heading G/Kpre is0.0497/0.0768deg with Hesai and
0.0575/0.0776deg with Velodyne; tilt is0.2067/0.0941 and0.3240/0.1147deg.
Thus the sensor-cohort check preserves useful-heading/harmful-tilt ordering.
Leica matched cohort has NO330--390s factors and only12in300--330s. Do not
use it to explain late drift; separate per-run30s bins retain coverage there.
This is a different matched cohort from CP-V7's original/candidate comparison.

For continuity with earlier reports, outgoing revision-MIXED median traces:
translation m^2 and rotation rad^2, OFF -> ON. These are sender publications,
not what the other sender receives, nor equal-age/equal-endpoint statistics.
Revisions are repeated versions of correlated edges, not independent samples.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        cv=c['trajectories']['original_evaluation']['covariance']
        for sender in ('glim','kimera'):
            names=[x for x in cv['off'] if x.startswith(sender)];assert len(names)==1
            name=names[0];x=cv['off'][name];y=cv['on'][name]
            rows.append([label,sender,f"{x['translation_trace_m2']['median']:.3e} -> {y['translation_trace_m2']['median']:.3e}",f"{x['rotation_trace_rad2']['median']:.3e} -> {y['rotation_trace_rad2']['median']:.3e}"])
    add(table(['Pair','Sender','Translation trace','Rotation trace'],rows))
    add('''5. ONLINE QUALITY AND DELAYED OUTPUT MUST STAY SEPARATE
-------------------------------------------------------
Full-saved ONLINE vs DELAYED ATE and path length, plus reference path length
sampled at ONLINE timestamps. Length is at prism, not the CPT7 body origin.
Native path lengths and original1s RPE are in AppendixA below. Polylines through
sparse native samples can shortcut motion; gaps are not fabricated motion.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        ev=c['trajectories']['original_evaluation']['results']
        for mode in v.MODES:
            on=ev[mode+'_fused_online']['full_saved'];de=ev[mode+'_fused_finalized']['full_saved']
            rows.append([label,mode,f"{on['ATE_RMSE_m']:.4f} / {de['ATE_RMSE_m']:.4f}",f"{on['path_length_prism_m']:.2f} / {de['path_length_prism_m']:.2f}",f"{on['reference_path_length_prism_m']:.2f}",f"{on['path_length_prism_m']/on['reference_path_length_prism_m']:.3f}"])
    add(table(['Pair','CBS','ATE ONLINE/DELAYED','Length ONLINE/DELAYED','Reference length','ONLINE/ref'],rows))
    add('''EIG-2 Velodyne ONLINE path is299.81m OFF /255.33m ON versus~188.13m
reference. DELAYED paths191.05/189.44m are much smoother, but their ATE still
worsens OFF->ON. Therefore neither jitter alone nor swapping in finalized
output removes the EIG-2 global accuracy regression. The two issues differ.
Do not describe finalized output as ONLINE or full-history smoothing.

6. COVERAGE, COMPLETION AND FAILURE ACCOUNTING
---------------------------------------------
New CP-V8 common grid: valid/excluded queries; supported seconds versus
envelope; last supported time is relative to capture start, not raw bag start.
An envelope can contain holes. RPE counts require the whole interval supported.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        d=c['trajectories']['new_common_coverage'];m=c['trajectories']['new_common_metrics']['off_native_kimera']
        rows.append([label,f"{d['capture_s']:.2f}",f"{d['valid_queries']}/{d['excluded_queries']}",f"{d['supported_s']:.1f}/{d['envelope_s']:.1f}",f"{d['first_s']:.3f}..{d['last_s']:.3f}",'/'.join(str(m['horizons'][str(h)]['n']) for h in (1,10,20))])
    add(table(['Pair','Capture s','Valid/excluded','Supported/envelope s','Common first..last s','RPE n1/10/20'],rows))
    add('''Native sparse stationary-tail keyframes cause long gaps, especially ETH1 and
Leica-2. All full-saved poses remain in their original ATE evaluation; the
matched-mask metrics make NO dense full-recording coverage claim. First/tail
offsets and each supported segment are retained in outputs/summary.json.
Window crops: ETH1 offset0, EIG-2 offset5.7, SPX-3 offset29, Leica-2 offset6s;
later sensor-only tails excluded by the established physical-window policy.

EOF omissions (OFF/ON): Hesai ETH1 0/0, EIG-2 0/1, SPX-3 1/1,
Leica-2 0/0; Velodyne SPX-3 1/0, EIG-2 0/0, Leica-2 0/0. These omissions
are not repaired here; their counterfactual accuracy impact is unknown.
Known post-absorption revisions expire under the recorded seal policy; last
accepted information remains in the prior. This is not lossless full history.

Reliability inventory:7completed pairs out of8attempted final-baseline pair
cases (Hesai4/4, Velodyne3/4), not a population reliability estimate. ARC-5
Velodyne OFF failed near380.6/401.29s with fusion GTSAM indeterminate linear
system near x3758. ON was not launched. Numerical root cause is unresolved;
exclude its failed prefix from full accuracy tables, not from failure counts.
CBS was OFF: changing G2K cannot establish that this numerical failure is fixed.
SNOW-3 quiet initialization blocked; ETH1 Velodyne input missing; SPX-2/CON-1
required inputs missing. These are unavailable cases, not successful runs or
numeric failures. Earlier superseded engineering attempts remain historical
evidence; the denominator above is the designated final-baseline campaign.

7. TIMING AND PRACTICAL DEPLOYMENT
---------------------------------
Milliseconds median/p95. GLIM per scan; Kimera per keyframe backend; fusion
per100ms support cycle INCLUDING intervening belief revisions. Do not add
parallel pipeline times or equate one factor solve with a complete cycle.
Hesai CP-T1 excludes first5s for these timers; Velodyne source timers exclude
first5s, fusion timers are whole-capture. Reuse original measurements, not
a fresh benchmark or evidence that CBS causally speeds up computation.
''')
    rows=[]
    for row,label,c in zip(v.ROWS,labels,ordered):
        original=json.loads(v.a.pin(E/row[3]/row[4]).read_text())
        for mode in v.MODES:
            if row[1]=='Hesai':
                t=timing['cases'][row[2]+' '+mode]
                fields=[t['glim']['steady']['scan_ms'],t['kimera']['steady']['backend_ms'],t['fusion']['steady']['support_service_ms']]
            else:
                t=original['cases'][mode];f=t['checks']['receiver']['support_cycle_seconds']
                fields=[t['glim_timing']['after5s']['scan_ms'],t['kimera_timing_after5s']['backend_ms'],{k:f[k]*1000 for k in ('median','p95')}]
            rows.append([label,mode]+[f"{x['median']:.1f}/{x['p95']:.1f}" for x in fields])
    add(table(['Pair','CBS','GLIM scan','Kimera keyframe','Fusion cycle'],rows))
    add('''Current evidence supports these completed0.20x captures, not0.5x operation.
At0.5x, a10Hz scan has200ms wall budget; Velodyne GLIM p95 exceeds it in
five of six cases. Queuing can absorb occasional overruns, so this is a risk
indicator, not proof of inevitable failure. Do not mix speed changes into the
next accuracy test. Sustained belief/covariance export remains expensive;
MAIN batching is active. No new optimization work is justified by this report.

8. WHAT WE CAN AND CANNOT CONCLUDE
---------------------------------
Supported:
- Collaboration often greatly improves position when LiDAR is strong.
- Switching to rear Velodyne reveals harmful tradeoffs hidden by good ATE.
- Rotation is not one trust number: direction-dependent error relationships
  recur across missions and sensors, and six Kimera long-horizon regressions
  deserve more weight than six position-ATE gains alone.
- Payload covariance use is consistent in the inspected factors. The problem
  is not a reporting-only substitution of the actual applied matrices here.

Not established:
- That smaller covariance identifies the healthier estimator, or any scalar
  multiplier/gate is calibrated; rank usefulness is sequence/direction-specific.
- That G2K tilt alone caused all drift. EIG-2 Hesai is a counterexample, and
  pre/post updates are not an isolated causal ablation. Mean alignment, past
  feedback, priors, native factors and shared IMU remain confounded.
- Independence of source beliefs or reference. All-LOCAL removes the intended
  explicit external-factor contribution, not every possible correlation.
- Statistical significance, universal success, nonlinear stationarity,
  arbitrary-duration numerical reliability, or0.5x/real-time readiness.

Known mathematical approximation: source peer means are temporally aligned
while covariance retains the existing mean-only treatment. Bridge uncertainty
and full transport are not reconstructed here. It remains a caveat, not a
newly proven explanation or a fix silently bundled into this proposal.
gnssFGO's original-endpoint source factors are a different path; do not assume
the source peer alignment approximation directly applies to them identically.

9. PREVIOUS NEGATIVE INTERVENTION: KEEP IT IN THE DECISION
--------------------------------------------------------
CP-V6A inflated all G2K rotational variance10x, translation/cross blocks
unchanged. Full-saved originalON->candidate ATE:
GLIM0.861349->0.713107m; Kimera2.584159->4.430475m;
ONLINE fusion1.828658->2.934918m. Reject as overall improvement.
CP-V7 showed lower Kimera1s rotation error but worse10/20s error and much
larger accumulated heading drift. At common390.385s, heading original/candidate
-21.310/-36.542deg; tilt decreased. This motivates separating directions,
not weakening all rotation again or claiming that untouched cross blocks mean
unchanged translational optimizer influence. The candidate was not promoted.

10. JUSTIFIED PROPOSAL AND EXPLICIT STOP RULE
-------------------------------------------
''')
    add((H/'PROPOSAL.txt').read_text())
    add('''APPENDIX A — ORIGINAL PER-STREAM FULL-SAVED METRICS
---------------------------------------------------
ATE m;1s translation RPE m;1s rotation RPE degrees; prism path m; saved span s.
These are historical scores, not newly harmonized RPE. DELAYED is separate.
''')
    rows=[]
    for label,c in zip(labels,ordered):
        ev=c['trajectories']['original_evaluation']['results']
        for role in (*v.ROLES,'fused_finalized'):
            for mode in v.MODES:
                d=ev[mode+'_'+role]['full_saved']
                rows.append([label,role,mode]+[f'{d[k]:.6f}' for k in ('ATE_RMSE_m','translation_RPE_1s_RMSE_m','rotation_RPE_1s_RMSE_deg','path_length_prism_m','span_s')])
    add(table(['Pair','Output','CBS','ATE','tRPE1s','rRPE1s','Path','Span'],rows))
    add('''APPENDIX B — EVIDENCE / REPRODUCIBILITY / READING GUIDE
------------------------------------------------------
REPORT.txt is this consolidated decision report. PROPOSAL.txt is the standalone
recommendation; no future run is authorized. analyze.py/report.py reproduce
the NEW analysis in fresh outputs; refuse overwrites. PROTOCOL.txt freezes scope.
outputs/summary.json includes original scores, new masks/horizons, directional
summaries, input hashes and cross-sensor cohort/identity checks.
Per-case*_series.npz retains timestamps, validity, fixed-start position and
orientation series;*_peer_records.jsonl.gz retains exact means/full matrices;
*_directions.jsonl.gz and*_peers.json retain directions and30s coverage bins.
No original source, receiver, profile, covariance or run evidence was modified.
''')
    maxdelta=max(x['full_saved_ATE_reproduction_error_m'] for c in ordered for x in c['trajectories']['new_common_metrics'].values())
    maxpre=max(c['peers']['checks']['max_pre_check'] for c in ordered)
    add(f'All42native/ONLINE full-saved ATE values independently reproduced within{maxdelta:.3e}m.\nAll10609pre-state joins pass; maximum logged-check discrepancy{maxpre:.3e}.\nEvery actual applied G2K covariance matches the baseline raw covariance exactly.\nSynthetic SE(3), rigid-transform invariance, known yaw drift, exact timestamp\nand gap-exclusion checks pass. Design algebra check is distinct from software\nvalidation. closeout.json records input hashes and final checks.\n')
    add('Figures:\n01_decision_scorecard.png — negative percentages/blue improve, positive/red worsen;\nposition-ATE gains coexist with Kimera long-horizon rotational regressions.\n02_kimera_rotation_horizons.png — actual OFF->ON degrees at1/10/20s.\n03_peer_directional_tradeoff.png — incoming G/Kpre error ratios; above1 is\nworse incoming error. Heading and tilt frequently lie on opposite sides.\n04_velodyne_orientation.png — fixed-start Kimera heading/tilt error over time;\nbroken lines mark excluded coverage. Not best-fit ATE or causal attribution.\n')
    add('Canonical source reports/results:\n'+'\n'.join(str(E/r[3]/r[4]) for r in v.ROWS))
    text='\n'.join(parts)
    with (H/'REPORT.txt').open('x') as f:f.write(text)
    design=design_check();v.a.save(O/'design_check.json',design)
    render(s,ordered,labels)
    # Explicit evidence counts, not unsupported inference from file names.
    k20=sum(c['trajectories']['new_common_metrics']['on_native_kimera']['horizons']['20']['rotation_RMS_deg']>c['trajectories']['new_common_metrics']['off_native_kimera']['horizons']['20']['rotation_RMS_deg'] for c in ordered)
    tilt=sum(c['peers']['all']['incoming']['tilt_deg_rms']>c['peers']['all']['pre']['tilt_deg_rms'] for c in ordered)
    heading=sum(c['peers']['all']['incoming']['heading_deg_rms']<c['peers']['all']['pre']['heading_deg_rms'] for c in ordered)
    total=sum(c['peers']['all']['n'] for c in ordered)
    assert (k20,tilt,heading,total)==(6,6,5,10609)
    assert all(x['byte_identical'] for x in s['kimera_off_cross_sensor_identity'].values())
    v.finish(O/'report_checks.json',dict(passed=True,kimera20s_rotation_regressions=k20,incoming_tilt_worse_pairs=tilt,
                incoming_heading_better_pairs=heading,actual_factors=total,ate_reproduction_max_error_m=maxdelta,
                pre_state_max_error=maxpre,projection_design_check=design,no_estimator_execution=True))
    print('Report and four figures generated; all consistency checks passed',flush=True)


def render(s,ordered,labels):
    plt.rcParams.update({'font.size':10,'savefig.facecolor':'white'})
    z=[];kh=[];ktr=[]
    for c in ordered:
        m=c['trajectories']['new_common_metrics']
        z.append([pct(m['off_'+r]['common_ATE_m'],m['on_'+r]['common_ATE_m']) for r in v.ROLES]+[pct(m['off_native_kimera']['horizons'][str(h)]['rotation_RMS_deg'],m['on_native_kimera']['horizons'][str(h)]['rotation_RMS_deg']) for h in (1,10,20)])
        kh.append([pct(m['off_native_kimera']['horizons'][str(h)]['rotation_RMS_deg'],m['on_native_kimera']['horizons'][str(h)]['rotation_RMS_deg']) for h in (1,10,20)])
        ktr.append([(m['off_native_kimera']['horizons'][str(h)]['rotation_RMS_deg'],m['on_native_kimera']['horizons'][str(h)]['rotation_RMS_deg']) for h in (1,10,20)])
    fig,ax=plt.subplots(figsize=(12,6.5));im=ax.imshow(z,cmap='RdBu_r',norm=TwoSlopeNorm(vmin=-100,vcenter=0,vmax=100),aspect='auto')
    for i,row in enumerate(z):
        for j,x in enumerate(row):ax.text(j,i,f'{x:+.1f}%',ha='center',va='center',color='white' if abs(x)>60 else 'black')
    ax.set_yticks(range(7),labels);ax.set_xticks(range(6),['GLIM\nposition ATE','Kimera\nposition ATE','ONLINE fusion\nposition ATE','Kimera\nrotation RPE 1s','Kimera\nrotation RPE 10s','Kimera\nrotation RPE 20s'])
    ax.axvline(2.5,color='white',lw=3);ax.axhline(3.5,color='black',lw=1)
    fig.colorbar(im,ax=ax,label='ON vs OFF change (%) — negative is better',shrink=.8,extend='both')
    ax.set_title('Position improvement does not imply rotational improvement',pad=15)
    fig.text(.06,.025,'Per-pair common masks; shared-CPT7 reference. Color clipped at ±100%; labels retain actual changes. No repeated-trial significance claim.',fontsize=8.5)
    fig.tight_layout(rect=(0,.065,1,1));fig.savefig(O/'01_decision_scorecard.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,6.5));im=ax.imshow(kh,cmap='RdBu_r',norm=TwoSlopeNorm(vmin=-100,vcenter=0,vmax=100),aspect='auto')
    for i,row in enumerate(ktr):
        for j,(x,y) in enumerate(row):ax.text(j,i,f'{x:.3f} → {y:.3f}°',ha='center',va='center',color='white' if abs(kh[i][j])>60 else 'black')
    ax.set_yticks(range(7),labels);ax.set_xticks(range(3),['1-second interval','10-second interval','20-second interval'])
    ax.set_title('Kimera rotational RPE: OFF → ON (degrees)\nSix of seven pairs worsen at 20 seconds',pad=14)
    ax.axhline(3.5,color='black',lw=1)
    fig.text(.06,.025,'No interval crosses an unsupported gap. Same-start cohort controls are saved. Blue improves; red worsens; color clipped at ±100%.',fontsize=8.5)
    fig.tight_layout(rect=(0,.065,1,1));fig.savefig(O/'02_kimera_rotation_horizons.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(11,6));x=np.arange(7)
    for off,field,label,color in [(-.18,'heading_deg_rms','Heading error ratio','#3478a9'),(.18,'tilt_deg_rms','Tilt error ratio','#d88b35')]:
        vals=[c['peers']['all']['incoming'][field]/c['peers']['all']['pre'][field] for c in ordered]
        b=ax.bar(x+off,vals,.35,label=label,color=color);ax.bar_label(b,fmt='%.2f',padding=3,fontsize=9)
    ax.axhline(1,color='black',ls='--',lw=1);ax.set_ylabel('Incoming GLIM RMS / Kimera pre-update RMS')
    ax.set_xticks(x,[x.replace(' ','\n',1) for x in labels]);ax.set_ylim(0,3.25);ax.legend(loc='upper left')
    ax.set_title('Incoming GLIM can be more accurate in one direction and less in another\nAbove 1: incoming error is larger; below 1: smaller',fontsize=12);ax.grid(axis='y',alpha=.2)
    fig.text(.06,.025,'10,609 exact installed factors/pre-states; per-run cohorts. These are reference errors, not online decision signals or isolated causal effects.',fontsize=8.5)
    fig.tight_layout(rect=(0,.065,1,1));fig.savefig(O/'03_peer_directional_tradeoff.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(12,10))
    for rr,slug in enumerate(('spx3','eig2','leica2')):
        z=np.load(v.a.pin(O/('velodyne_'+slug+'_series.npz')));valid=z['valid'];t=z['time_s']
        for mode,col in [('off','#327eaa'),('on','#d98030')]:
            angles=z[mode+'_native_kimera_rpy_error_deg'].copy();angles[~valid]=np.nan
            axes[rr,0].plot(t,angles[:,2],color=col,label='CBS '+mode.upper(),lw=1.2)
            axes[rr,1].plot(t,np.linalg.norm(angles[:,:2],axis=1),color=col,label='CBS '+mode.upper(),lw=1.2)
        axes[rr,0].set_ylabel(slug.upper()+' Kimera (deg)')
        for ax in axes[rr]:ax.grid(alpha=.2);ax.set_xlabel('Seconds after capture start')
    axes[0,0].set_title('Heading discrepancy');axes[0,1].set_title('Roll/pitch discrepancy norm')
    axes[0,0].legend();fig.suptitle('Velodyne: accumulated orientation error after one fixed start alignment')
    fig.text(.05,.012,'Euler-angle diagnostics in CPT7 axes; not ATE. Gaps are not interpolated; no alignment reset. No claim of isolated G2K causality.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,.97));fig.savefig(O/'04_velodyne_orientation.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
