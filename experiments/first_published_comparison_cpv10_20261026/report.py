"""Render CP-V10 results without touching any historical artifacts."""
import importlib.util
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('rescore',H/'analyze.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
LABELS={'glim_initial':'GLIM first-published','kimera':'Kimera published','fusion_online':'Fusion ONLINE'}


def table(headers, rows):
    return ['| '+' | '.join(headers)+' |','|'+'|'.join(['---']*len(headers))+'|']+[
        '| '+' | '.join(map(str,row))+' |' for row in rows]+['']


def pair(d, role, field='ATE_RMSE_m', digits=4, block='common'):
    return f"{d[block]['off_'+role][field]:.{digits}f} / {d[block]['on_'+role][field]:.{digits}f}"


def main():
    summary=H/'outputs/summary.json';data=json.loads(summary.read_text());cases=data['cases']
    dest=H/'REPORT.txt';figdir=H/'figures'
    assert not dest.exists() and not figdir.exists(), 'Refuse report overwrite'
    for p,h in data['input_sha256'].items():assert s.a.g.sha(Path(p))==h,p
    assert all(json.loads(Path(d['original_report']).read_text())['passed'] for d in cases.values())
    checks=data['verification'];names=[d['sensor']+' '+d['sequence'] for d in cases.values()]
    rank=[];initial_wins=0;kimera_wins=0;fusion_wins=0
    for d in cases.values():
        for mode in ('off','on'):
            current=sorted(s.ROLES,key=lambda r:d['maturity'][mode+'_'+r]['ATE_RMSE_m'])
            old=sorted(s.ROLES,key=lambda r:d['maturity'][mode+'_'+('glim_refined' if r=='glim_initial' else r)]['ATE_RMSE_m'])
            rank.append(current==old)
            initial_wins+=current[0]=='glim_initial';kimera_wins+=current[0]=='kimera';fusion_wins+=current[0]=='fusion_online'
    assert all(rank)
    effects=[v for d in cases.values() for v in d['maturity_effect'].values()]
    rpe_effects=[s.improve(v['initial_RPE1']['translation_RMSE_m'],v['refined_RPE1']['translation_RMSE_m']) for v in effects]
    lines=['CP-V10 — FIRST-PUBLISHED OUTPUT COMPARISON','Completed 2026-10-26; saved-data rescoring only.','',
        'EXECUTIVE CONCLUSION','',
        'The earlier cross-estimator comparison mixed revised GLIM poses with first-published Kimera and fusion ONLINE. That was a reporting mismatch. This report corrects it without rerunning any estimator.',
        f"However, refinement does NOT account for the large previous ATE advantage: on identical timestamps it improves GLIM ATE in13/14 cases by0.030% to7.090%, and worsens one case by0.184%. All14 within-mode estimator ATE rankings are unchanged. GLIM is lowest in{initial_wins}/14, Kimera in{kimera_wins}/14, fusion ONLINE in{fusion_wins}/14.",
        f"Refinement matters more to local smoothness: GLIM1s translation RPE improves by{min(rpe_effects):.2f}--{max(rpe_effects):.2f}% across the14 cases. On Velodyne EIG-2 it removes10.42m OFF/9.54m ON from the matched10Hz path, while ATE improves only2.22%/0.92%.",
        'CBS remains mixed: first-published GLIM ATE improves3/7 pairs; Kimera and fusion ONLINE improve6/7 each. Fusion ONLINE still never has the lowest ATE among the three on a matched baseline. Leica-2 Velodyne OFF remains a real Kimera-better-than-GLIM case. Its CBS ON improves GLIM strongly but damages Kimera.',
        'Use this first-publication table for online-output accuracy claims. Preserve the old values as explicitly labelled revised-GLIM results, not discarded/invalid numerical measurements. Do not claim that this study measures equal latency or proves fusion cannot be useful.',
        '', '1. EXACTLY WHAT IS COMPARED','',
        'GLIM: /glim/cbs/odometry in glim_odometry.csv. First optimized MAIN estimate for the newest scan after that update and current temporary CBS application. Not an unoptimized initial guess, not LOCAL-belief means, and not odom_imu.txt.',
        'Kimera: /kimera_vio_ros/odometry in kimera_odometry.csv, unchanged. Saved --no_incremental_pose=false means its published pose chains optimized consecutive-keyframe increments. It is not the absolute newest smoother pose, nor the final revision of every state.',
        'Fusion ONLINE: online_trajectory.tum, unchanged. The newest100ms support is saved once per support cycle; later updates do not replace old file rows. This remains a valid first-publication product, not a snapshot of the fully revised active window.',
        'Supplement only: GLIM odom_imu.txt retains revised active-window poses at retirement; remaining active tail is flushed at EOF. It is neither first publication nor full-history smoothing. Fusion FINALIZED is not included in the primary comparison.',
        '', '2. PRIMARY COMMON-MASK POSITION ATE','',
        'All values in meters; each cell is CBS OFF / ON. Same reference times for all six products within each sequence. Rigid SE(3) no-scale best-fit alignment is estimated separately for each trajectory. Lowest is better. This is reference-aligned trajectory error, not raw absolute navigation error.', '']
    lines+=table(['Sensor / sequence','GLIM first','Kimera published','Fusion ONLINE'],[
        [name,pair(d,'glim_initial'),pair(d,'kimera'),pair(d,'fusion_online')] for name,d in zip(names,cases.values())])
    lines+=['CBS percentage improvement =100*(OFF-ON)/OFF. Positive is better; negative is worse. Descriptive single-run differences, not statistical significance.','']
    lines+=table(['Sensor / sequence','GLIM ATE','Kimera ATE','Fusion ATE'],[
        [name,*[f"{d['off_on'][r]['ATE_improvement_percent']:+.2f}%" for r in s.ROLES]] for name,d in zip(names,cases.values())])
    lines+=['Important small change: Hesai SPX-3 GLIM was essentially tied using revised poses (full-saved0.016576/0.016559m). First-published common ATE is0.016652/0.017117m: ON is2.79% worse, an absolute difference of0.465mm. Do not inflate that tiny difference into a major failure.','',
        '3. HOW MUCH DID GLIM REFINEMENT HELP?','',
        'Identical eight-product maturity mask for each row; positive benefit means refined ATE is lower. The primary and maturity masks are identical except one terminal100ms query removed for Velodyne SPX-3 and EIG-2. This avoids comparing different start/end coverage. Millimeter differences below are ATE differences, not maximum point corrections.','']
    lines+=table(['Sensor / sequence','CBS','First ATE m','Refined ATE m','Benefit %','ATE reduction mm','1s translation-RPE benefit %'],[
        [name,mode.upper(),f"{v['initial_ATE_m']:.6f}",f"{v['refined_ATE_m']:.6f}",
         f"{v['refinement_ATE_improvement_percent']:+.3f}",f"{1000*v['initial_minus_refined_ATE_m']:+.3f}",
         f"{s.improve(v['initial_RPE1']['translation_RMSE_m'],v['refined_RPE1']['translation_RMSE_m']):+.2f}"]
        for name,d in zip(names,cases.values()) for mode,v in d['maturity_effect'].items()])
    lines+=['Conclusion: the mismatch understated GLIM short-horizon error and roughness more than it understated global ATE. It does not reverse the main estimator ranking in these saved runs.','',
        '4. COMMON-MASK ONE-SECOND RELATIVE MOTION ERROR','',
        'Each cell is OFF / ON. Translation is meters; rotation is degrees. Every intervening100ms point must be supported, so no one-second interval crosses a masked gap. Orientation is measured independently of whole-path ATE alignment.','']
    for field,label in [('translation_RMSE_m','Translation RPE RMSE (m)'),('rotation_RMSE_deg','Rotation RPE RMSE (deg)')]:
        lines+=[label,'']
        lines+=table(['Sensor / sequence','GLIM first','Kimera published','Fusion ONLINE'],[
            [name,*[f"{d['common']['off_'+r]['horizons']['1'][field]:.5f} / {d['common']['on_'+r]['horizons']['1'][field]:.5f}" for r in s.ROLES]]
            for name,d in zip(names,cases.values())])
    lines+=['Longer-horizon results are also retained at10s/20s in each case JSON and summary.json; identical supported start/end pairs across the six products per horizon. They are overlapping intervals, not independent trials.','',
        '5. MATCHED PATH LENGTHS AND COVERAGE','',
        'Meters, OFF / ON, same10Hz grid and supported segments as Section2. Reference path is identical for all six products. Gaps are not joined by a chord. These are reconstructed path lengths, not measured physical distance flown; shorter is not automatically more accurate.','']
    lines+=table(['Sensor / sequence','Reference m','GLIM first m','Kimera m','Fusion ONLINE m'],[
        [name,f"{d['common']['off_glim_initial']['reference_path_length_m']:.2f}",
         *[pair(d,r,'path_length_m',2) for r in s.ROLES]] for name,d in zip(names,cases.values())])
    lines+=['This comparison also controls sampling rate, but interpolation onto10Hz can suppress variations between those grid times. Therefore do not read a smaller common-grid length versus native length as an estimator improvement. Section6 retains the original sampling-rate polylines. Connecting first publications can include estimator correction jumps; it does not establish that the latest revised trajectory, or a physical drone, zigzags.','']
    lines+=table(['Sensor / sequence','Capture s','Supported s (%)','First / last s','Common queries','Excluded grid queries'],[
        [name,f"{d['capture_s']:.2f}",f"{c['supported_s']:.1f} ({c['supported_capture_percent']:.1f}%)",
         f"{c['first_s']:.3f} / {c['last_s']:.3f}",c['valid_queries'],c['excluded_queries']]
        for name,d in zip(names,cases.values()) for c in [d['common_coverage']]])
    lines+=['Times are seconds after each frozen capture start. Supported duration sums contiguous grid segments, not last-minus-first. Full captures and full-saved evaluation remain separate from this stricter common mask.',
        'ETH1 common coverage stops near302.045s despite later saved poses, because sparse Kimera keyframe intervals exceed the unchanged500ms interpolation bound. Leica-2 has analogous gaps, including long sparse late segments. This is an evaluation-support limit, not evidence of missing camera measurements or a failed full capture. Detailed segments are saved in JSON.',
        '', '6. FULL-SAVED OUTPUT SCORES AND PATHS (SECONDARY)','',
        'Every actual saved pose has a reference bracket. These rows use each product\'s own timestamps, span, and sampling rate; they are not matched-time cross-estimator rankings. Native polyline lengths can bridge long saved-sample gaps. ATE is meters; path meters. Refined GLIM is explicitly separate.','']
    lines+=table(['Sensor / sequence','Product','ATE OFF / ON m','Native path OFF / ON m','Rows OFF / ON','Span OFF / ON s'],[
        [name,LABELS.get(r,'GLIM refined (supplement)'),pair(d,r,digits=6,block='full_saved'),
         pair(d,r,'native_polyline_length_m',2,'full_saved'),
         f"{d['full_saved']['off_'+r]['rows']} / {d['full_saved']['on_'+r]['rows']}",
         pair(d,r,'span_s',3,'full_saved')]
        for name,d in zip(names,cases.values()) for r in (*s.ROLES,'glim_refined')])
    lines+=['Full-saved starts/ends, reference row support and gap-chord counts are in the per-case JSON. GLIM CSV and refined EOF exports can differ by one final scan, another reason to use Section3\'s exact matched mask for refinement attribution.','',
        '7. INTERPRETATION AND NEXT DECISION','',
        'A. Replace mixed-maturity tables in future supervisor/online-performance discussions with Section2. Cite the old GLIM numbers only as fixed-lag-refined export results. The original reports are preserved unchanged; this report supersedes their cross-estimator first-publication interpretation.',
        'B. Do not rerun these seven pairs merely to fix the scoring mismatch: all14 live GLIM streams were already saved. The corrected ATE rankings do not support attributing the large GLIM/Kimera difference primarily to later refinement.',
        'C. The fusion question remains: on these baselines its first-published ATE never beats the best native estimator. Its output convention and arrival timing can add correction discontinuities. A future implementation-level analysis should distinguish newest-state publication, corrected active-window trajectory, and age-matched delayed products; it must not silently substitute one for another.',
        'D. Leica-2 Velodyne remains the strongest counterexample to universally trusting GLIM: Kimera OFF1.094m versus GLIM OFF2.782m on common timestamps. CBS flips this to GLIM0.849m and Kimera2.585m; the trust/rotation problem was not an artifact of refined GLIM scoring.',
        'E. No covariance coefficient or new model is selected here. Covariance-belief diagnostics remain relative-edge quantities with their own timestamps/maturity, not these absolute output ATEs; this rescoring does not recalibrate them.',
        '', '8. VALIDATION, PROVENANCE AND LIMITS','',
        f"All14 GLIM CSVs have unique strictly increasing header stamps, finite poses and map/cpt7_imu frames. All42 historical ATE values reproduce; maximum discrepancy{checks['historical_ATE_max_difference_m']:.3g}m. Independent evo no-scale alignment agrees within{checks['evo_ATE_max_difference_m']:.3g}m. Homogeneous-matrix RPE checks agree within{checks['direct_RPE_max_difference']:.3g} (translation meters / rotation degrees). Existing identity/global-transform/known-yaw/gap/exact-sample tests pass.",
        'All frozen input SHA256 values are rechecked before report generation and in closeout. Source code, recorded runner/topic and effective Kimera flags are included in input provenance. Header stamp, not simulated ROS logging time, is used. No new source playback, solver replay, container start, build or tuning occurred.',
        'Implementation evidence: GLIM cbs_bridge.cpp onSmootherUpdateFinish calls applyTemporaryFactors before publishOdometryAndRerun; publisher reads calculateEstimate(last_pose_key_). GLIM odometry_estimation_imu.cpp updates retained frames on subsequent updates; global_mapping.cpp writes retained odometry frames. Kimera VioBackend.cpp updateStates chains optimized between-keyframe motions. CP-K19 Receiver.h publish appends only s.newest to online_incremental.',
        'First-published products do not have proven equal wall-clock latency, identical update frequencies, or identical absolute-vs-incremental output conventions. The10Hz interpolation is an offline scoring operation, not a newly causal online output. Best-fit whole-path alignment uses future reference information; no absolute real-time control-accuracy claim.',
        'IE-TC GNSS/INS shares CPT7 IMU; not independent prism ground truth. The same fixed CPT7-to-prism transform is applied to estimate and reference. Mean-only source covariance alignment approximation is unchanged. No nonlinear-stationarity claim.',
        'Seven pairs cover four missions. SPX-3/EIG-2/Leica-2 Kimera OFF streams are byte-identical between Hesai and Velodyne, not independent repeats. Sensor replacement also changes viewpoint. No claim of statistical significance, population-wide generalization or validated0.5x operation.',
        'Preserved terminal factor omissions: Hesai EIG-2 ON1 (OFF0), Hesai SPX-3 OFF1/ON1, Velodyne SPX-3 OFF1/ON0; other successful baseline pairs0. ARC-5 Velodyne OFF failed numerically before full duration and ON was not run; excluded from accuracy tables, not from the reliability record. ETH1 Velodyne unavailable locally. Rejected CP-V6A/CP-V9 candidates are not baseline pairs.',
        '', 'FILES','',str(summary),str(H/'PROTOCOL.txt'),str(H/'analyze.py'),
        'outputs/<case>.json: full/common/maturity scores, gaps, inputs, validation.',
        'outputs/<case>.npz: integer timestamps, both masks, prism positions/rotations, common aligned errors.',
        'figures/01_first_published_ate.png: matched first-publication ATE; OFF blue, ON orange, log scale.',
        'figures/02_glim_refinement_effect.png: same-time refinement benefit for ATE and1s translation RPE.',
        'closeout.json: final source/output hash checks. Historical reports/captures remain unchanged.','']
    figdir.mkdir()
    plt.rcParams.update({'font.size':10,'savefig.facecolor':'white'})
    xx=np.arange(len(cases));colors={'off':'#2878b5','on':'#e68626'}
    fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
    for ax,role in zip(axes,s.ROLES):
        for mode,offset in [('off',-.19),('on',.19)]:
            yy=[d['common'][mode+'_'+role]['ATE_RMSE_m'] for d in cases.values()]
            ax.bar(xx+offset,yy,width=.36,color=colors[mode],label='CBS '+mode.upper())
            for x,y in zip(xx+offset,yy):ax.text(x,y*1.07,f'{y:.3f}',ha='center',va='bottom',fontsize=8)
        ax.set_yscale('log');ax.set_ylim(.01,8);ax.grid(axis='y',alpha=.22)
        ax.set_ylabel(LABELS[role]+'\nATE RMSE (m)')
    axes[0].legend(loc='upper right');axes[-1].set_xticks(xx,[n.replace(' ','\n',1) for n in names])
    fig.suptitle('First-published trajectories: matched timestamps, CBS OFF versus ON')
    fig.text(.04,.014,'Lower is better. Logarithmic y-axis. Each sequence has its own disclosed common mask; IE-TC shares CPT7.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,.96));fig.savefig(figdir/'01_first_published_ate.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,7.5),sharex=True)
    for mode,offset in [('off',-.19),('on',.19)]:
        ee=[d['maturity_effect'][mode] for d in cases.values()]
        values=[[v['refinement_ATE_improvement_percent'] for v in ee],
                [s.improve(v['initial_RPE1']['translation_RMSE_m'],v['refined_RPE1']['translation_RMSE_m']) for v in ee]]
        for ax,yy in zip(axes,values):
            ax.bar(xx+offset,yy,width=.36,color=colors[mode],label='CBS '+mode.upper())
            for x,y in zip(xx+offset,yy):ax.text(x,y+.25,f'{y:.2f}',ha='center',va='bottom',fontsize=8)
    for ax in axes:ax.axhline(0,color='black',lw=.7);ax.grid(axis='y',alpha=.22)
    axes[0].set_ylim(-.8,8.5);axes[1].set_ylim(0,23)
    axes[0].set_ylabel('ATE reduction (%)');axes[1].set_ylabel('1s translation RPE reduction (%)')
    axes[0].legend();axes[-1].set_xticks(xx,[n.replace(' ','\n',1) for n in names])
    fig.suptitle('How much does later GLIM refinement help?')
    fig.text(.04,.015,'Same timestamps for first and refined GLIM. Positive = refined is better; negative = refined is worse.',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(figdir/'02_glim_refinement_effect.png',dpi=160);plt.close(fig)
    with dest.open('x') as f:f.write('\n'.join(lines))
    inputs=dict(data['input_sha256']);inputs[str(summary)]=s.a.g.sha(summary)
    inputs[str(Path(__file__))]=s.a.g.sha(Path(__file__))
    assert all(s.a.g.sha(Path(p))==sha for p,sha in inputs.items())
    outputs={str(p):s.a.g.sha(p) for p in [dest,*sorted(figdir.glob('*.png'))]}
    s.a.save(H/'closeout.json',dict(passed=True,completed='2026-10-26',
        estimator_ranking_unchanged_all14=True,first_publication_ATE_lowest_counts=dict(glim=initial_wins,kimera=kimera_wins,fusion=fusion_wins),
        verification=checks,input_sha256=inputs,output_sha256=outputs,input_hashes_unchanged=True,
        no_source_rerun=True,no_solver_replay=True,no_historical_output_overwrite=True))
    print(json.dumps(dict(report=str(dest),input_hashes=len(inputs),output_hashes=len(outputs),rankings_unchanged=True),indent=2))


if __name__=='__main__':main()
