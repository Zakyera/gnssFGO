"""Integrity, descriptive summaries and plots only; never invokes an estimator."""
import csv, hashlib, json, subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import ROOT, REPO, sha
from beliefs import stat

manifest=json.loads((ROOT/'manifest.json').read_text())
changed=[p for p,h in manifest['protected'].items() if sha(p)!=h]
assert not changed, changed
head=subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip()
diff=hashlib.sha256(subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])).hexdigest()
assert head==manifest['head'] and diff==manifest['tracked_diff_sha256']
beliefs=list(csv.DictReader((ROOT/'belief_comparison.csv').open()))
checks=[];batch=[];counts={};intervals={};paired={}
for mode in ['off','on']:
    ledger=list(csv.DictReader((ROOT/(mode+'_wnoa/snapshot_final/ledger.csv')).open()))
    intervals[mode]={(r['source'],r['from_ns'],r['to_ns']) for r in ledger}
    analyzed={(r['source'],r['from_ns'],r['to_ns']) for r in beliefs if r['mode']==mode}
    assert intervals[mode]==analyzed and len(ledger)==len(analyzed)==1770
    counts[mode]={s:sum(r['source']==s for r in ledger) for s in {r['source'] for r in ledger}}
    for label in ['294','594','894','1192','final']:
        folder=ROOT/(mode+'_wnoa')/('snapshot_'+label)
        original=np.loadtxt(folder/'incremental_states.csv',delimiter=',',skiprows=1)
        for variant in ['batch','polished','full_backsub']:
            a=np.loadtxt(folder/(variant+'_states.csv'),delimiter=',',skiprows=1)
            assert np.array_equal(a[:,:2],original[:,:2])
            difference={'position_m':float(np.max(np.linalg.norm(a[:,2:5]-original[:,2:5],axis=1))),
                'rotation_rad':float(np.max((Rotation.from_quat(a[:,5:9]).inv()*Rotation.from_quat(original[:,5:9])).magnitude())),
                'velocity_mps':float(np.max(np.linalg.norm(a[:,9:12]-original[:,9:12],axis=1))),
                'omega_radps':float(np.max(np.linalg.norm(a[:,12:15]-original[:,12:15],axis=1)))}
            assert max(difference.values())<1e-5,(mode,label,variant,difference)
            batch.append(dict(mode=mode,snapshot=label,variant=variant,**difference))
    for source in counts[mode]:
        rows=[r for r in beliefs if r['mode']==mode and r['source']==source and r['age']=='cutoff']
        assert len(rows)==counts[mode][source]
for p in sorted((ROOT/'interaction').glob('*_check.csv')):
    r=next(csv.DictReader(p.open()));r={k:float(v) for k,v in r.items()}
    assert r['cost_relative_error']<1e-7
    # -1 explicitly denotes a snapshot without the optional original payload probe.
    assert r['probe_cov_error']<1e-10 and r['probe_residual_error']<1e-9
    checks.append(dict(snapshot=p.stem,**r))
assert intervals['off']==intervals['on']
for source in counts['off']:
    for age in ['first','1','2','cutoff']:
        a={(r['from_ns'],r['to_ns']):r for r in beliefs if r['mode']=='off' and r['source']==source and r['age']==age}
        b={(r['from_ns'],r['to_ns']):r for r in beliefs if r['mode']=='on' and r['source']==source and r['age']==age}
        assert set(a)==set(b)
        paired[source+'_'+age]={}
        for name in ['rho_error_m','rotation_error_deg']:
            off=np.array([float(a[k][name]) for k in a]);on=np.array([float(b[k][name]) for k in a])
            paired[source+'_'+age][name]={'on_better_fraction':float(np.mean(on<off)),
                                         'on_minus_off':stat(on-off)}
linear=json.loads((ROOT/'linear_summary.json').read_text())
assert len(linear)==18
event_rows=list(csv.DictReader((ROOT/'largest_event_beliefs.csv').open()))
events={}
for mode in ['off','on']:
    for source in counts[mode]:
        rr=[r for r in event_rows if r['mode']==mode and r['source']==source and r['kind']=='revision']
        a=np.array([float(r['old_rho_reference_error_m']) for r in rr])
        b=np.array([float(r['new_rho_reference_error_m']) for r in rr])
        events[mode+'_'+source]={'revisions':len(rr),'old_rho':stat(a),'new_rho':stat(b),'rho_improved_fraction':float(np.mean(b<a))}
report={'passed':True,'protected_files':len(manifest['protected']),'protected_hashes_unchanged':True,
        'head_unchanged':head,'tracked_diff_unchanged':True,'accepted_intervals_equal':True,'counts':counts,
        'snapshot_checks':checks,'copied_solution_checks':batch,'paired_reference_discrepancies':paired,
        'largest_batch_paired_revision_errors':events,
        'max_fresh_normalized_gradient':max(v['fresh_normalized_gradient'] for v in linear.values()),
        'max_linear_solve_relative_residual':max(f['linear_solve_relative_residual'] for v in linear.values() for f in v['families'].values()),
        'max_sensitivity_finite_difference_relative_error':max(f.get('finite_difference_relative_error',0) for v in linear.values() for f in v['families'].values())}
with (ROOT/'audit_validation.json').open('x') as f:json.dump(report,f,indent=2)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
changes=list(csv.DictReader((ROOT/'changes.csv').open()))
fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
for mode,color in [('off','tab:blue'),('on','tab:orange')]:
    rr=[r for r in changes if r['mode']==mode and r['zone']=='current']
    t=[(int(r['timestamp_ns'])-manifest['first_imu_ns'])*1e-9 for r in rr]
    axes[0].plot(t,[100*float(r['position_m']) for r in rr],label=mode.upper(),color=color,lw=.7,alpha=.85)
    axes[1].plot(t,[float(r['velocity_mps']) for r in rr],label=mode.upper(),color=color,lw=.7,alpha=.85)
axes[0].set_ylabel('Same-time position revision (cm)');axes[0].legend()
axes[1].set_ylabel('Same-time velocity revision (m/s)');axes[1].set_xlabel('Seconds from first IMU timestamp')
fig.suptitle('Old current state revised by the next publication — NOT ground-truth errors')
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(ROOT/'same_time_revisions.png',dpi=160);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(10,4))
metric=json.loads((ROOT/'metrics.json').read_text())
for ax,key,title in zip(axes,['ate_m','translation_rpe_m'],['Prism ATE (62.1 s GT coverage)','1-second translational RPE']):
    for x,mode,color in [(0,'off','tab:blue'),(1,'on','tab:orange')]:
        vals=[metric[mode+'_fused_'+kind][key] for kind in ['online','final']]
        ax.bar(np.arange(2)+(x-.5)*.32,vals,width=.32,label=mode.upper(),color=color)
    ax.set_xticks([0,1],['Online/as-played','Final retrospective']);ax.set_ylabel('m');ax.set_title(title);ax.grid(axis='y',alpha=.2)
axes[0].legend();fig.suptitle('Identical common grid and arrival cutoff; unchanged WNOA receiver')
fig.tight_layout();fig.savefig(ROOT/'matched_fusion_metrics.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in report.items() if k not in ['snapshot_checks','copied_solution_checks','paired_reference_discrepancies','largest_batch_paired_revision_errors']},indent=2))
