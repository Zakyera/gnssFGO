"""Additional saved-trace analysis; no solver or source replay."""
import csv, hashlib, json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_receiver_trace_cpv13_20261026')
V12=H.parent/'leica2_fusion_jitter_cpv12_20261026'
V11=H.parent/'leica2_velodyne_rerun_cpv11_20261026'
norm=lambda x:np.linalg.norm(x,axis=-1)
cache=np.load(V12/'timeseries.npz');fits=json.loads((V11/'data_checks.json').read_text())
out={};fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
for col,mode in enumerate(['off','on']):
    assert json.loads((H/f'{mode}_parity.json').read_text())['pass_']
    with open(R/f'{mode}_trace'/'trace_meta.csv') as f: meta=list(csv.DictReader(f))
    pubs={int(m['snapshot']):int(m['newest']) for m in meta if m['stage']=='publication'}
    # Only publication rows needed; avoid retaining all before/after snapshots.
    rows=[]
    with open(R/f'{mode}_trace'/'trace_states.csv') as f:
        reader=csv.reader(f);next(reader)
        for row in reader:
            if int(row[0]) in pubs: rows.append([float(x) for x in row])
    a=np.array(rows);Q=np.array(fits['products'][mode+'_fusion_online']['alignment_rotation'])
    tr=np.array(fits['products'][mode+'_fusion_online']['alignment_translation_m'])-np.array(fits['origin_reference_enu_m'])
    points=(a[:,2:5]+Rotation.from_quat(a[:,5:9]).apply([.308837802,-.038312997,-.234944116]))@Q.T+tr
    poses={(pubs[int(row[0])],int(row[1])):p for row,p in zip(a,points)}
    t=cache[mode+'_time_s'];op=cache[mode+'_online'];fp=cache[mode+'_finalized'];gp=cache[mode+'_reference']
    ages=[0,1,2,3,5,10]
    indices=np.array([i for i in range(len(t)) if 380<t[i]<=389 and all((i+age,i) in poses for age in ages)])
    assert len(indices)>80 and np.all(np.diff(indices)==1)
    report=dict(physical_start_s=float(t[indices[0]]),physical_end_s=float(t[indices[-1]]),rows=len(indices),
        reference_path_m=float(norm(np.diff(gp[indices],axis=0)).sum()),products={})
    for age in ages:
        x=np.array([poses[(i+age,i)] for i in indices])
        if age==0: assert np.array_equal(x,op[indices])
        diff=np.diff(x,axis=0);dr=np.diff(gp[indices],axis=0)
        report['products'][str(age/10)]=dict(path_m=float(norm(diff).sum()),
            step_vector_error_rms_m=float(np.sqrt(np.mean(norm(diff-dr)**2))),
            position_rmse_fixed_original_fit_m=float(np.sqrt(np.mean(norm(x-gp[indices])**2))),
            same_state_shift_from_first_median_m=float(np.median(norm(x-op[indices]))),
            saved_finalized_remaining_shift_median_m=float(np.median(norm(fp[indices]-x))))
        if age in [0,1,2,10]:
            axes[0,col].plot(t[indices][1:],100*norm(diff),label=f'State age {age/10:g}s',lw=1)
    axes[0,col].plot(t[indices][1:],100*norm(np.diff(gp[indices],axis=0)),color='black',label='IE-TC reference',lw=1.4)
    axes[0,col].set(title=f'CBS {mode.upper()}: same physical states, different publication ages',ylabel='100ms step length (cm)',xlabel='Physical time after capture start (s)')
    axes[0,col].legend(fontsize=8);axes[0,col].grid(alpha=.2)
    for label,x in [('finalized',fp[indices])]:
        report['products'][label]=dict(path_m=float(norm(np.diff(x,axis=0)).sum()),
            step_vector_error_rms_m=float(np.sqrt(np.mean(norm(np.diff(x,axis=0)-np.diff(gp[indices],axis=0))**2))))
    cycles=json.loads((H/f'{mode}_cycles.json').read_text())
    both=[c for c in cycles if c['counts'].get('kimera_front_insert',0)]
    cos=[]
    for c in both:
        g=np.array(c['vectors']['glim_velodyne_insert']);k=np.array(c['vectors']['kimera_front_insert'])
        cos.append(float(g@k/(norm(g)*norm(k))))
    report['new_glim_vs_new_kimera_same_cycle_cosine']=dict(n=len(cos),median=float(np.median(cos)),opposing_fraction=float(np.mean(np.array(cos)<0)))
    # Select previously discussed alternating 383.0--383.6s cycles, no new search.
    report['example_cycles']=[]
    for c in cycles:
        if 383<c['time_s']<383.6:
            report['example_cycles'].append(dict(time_s=c['time_s'],state_index=c['state_index'],
                online_step_cm=float(100*norm(c['actual'])),reference_step_cm=float(100*norm(c['reference'])),
                component_cm={k:float(100*norm(v)) for k,v in c['vectors'].items()},counts=c['counts']))
    analyses=json.loads((H/f'{mode}_analysis.json').read_text())
    ag=[0,.1,1,3,6]
    for group,label in [('glim_velodyne_insert','New GLIM belief'),('kimera_front_insert','New Kimera belief')]:
        vals=[1000*analyses['groups'][group]['age_shift_m'][str(float(v))]['p95'] for v in ag]
        axes[1,col].plot(ag,vals,'o-',label=label)
    axes[1,col].set(xlabel='Age of affected state behind newest (s)',ylabel='p95 same-state movement per new belief (mm)',yscale='symlog')
    axes[1,col].grid(alpha=.2);axes[1,col].legend(fontsize=8)
    out[mode]=report
with open(H/'maturity.json','x') as f: json.dump(out,f,indent=2)
fig.suptitle('Diagnostic only: delayed trajectories are NOT current-time outputs or a fix\nSelected interval; original alignment, no smoothing, unchanged receiver mathematics')
fig.savefig(H/'state_age.png',dpi=170)
print(json.dumps(out,indent=2))
