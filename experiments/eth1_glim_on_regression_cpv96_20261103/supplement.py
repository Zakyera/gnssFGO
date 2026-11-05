"""Read-only follow-up; original analysis and terminal reports remain immutable."""
import csv, json
from pathlib import Path
import analyze as a
import numpy as np
from scipy.spatial.transform import Rotation

H=a.H
assert not (H/'supplement.json').exists()
a.pin(Path(__file__))
core=json.loads(a.pin(H/'verification.json').read_text())
for p,d in core['output_sha256'].items():assert a.sha(p)==d
x=json.loads(a.pin(H/'analysis.json').read_text())
events=json.loads(a.pin(H/'belief_events.json').read_text())
rows={k:a.parse_source(s) for k,s in a.SOURCES.items()}
def quant(z):
    z=np.asarray(z,float)
    return dict(n=z.size,**{n:float(v) for n,v in zip(['min','p1','p5','median','p95','p99','max'],np.percentile(z,[0,1,5,50,95,99,100]))})
ratios=[];spectra=[]
for old,new in zip(rows['old'],rows['new']):
    L=np.linalg.cholesky(old['C']);tmp=np.linalg.solve(L,new['C'])
    relative=np.linalg.solve(L,tmp.T).T
    eig=np.linalg.eigvalsh((relative+relative.T)/2)
    assert eig.min()>0
    spectra.append(eig.tolist())
    ratios.append([np.trace(new['C'][:3,:3])/np.trace(old['C'][:3,:3]),np.trace(new['C'][3:,3:])/np.trace(old['C'][3:,3:])])
ratios=np.asarray(ratios)
groups={}
for changed in [False,True]:
    ids=[i for i,z in enumerate(events) if z['binding_changed']==changed]
    groups[str(changed)]={label:{kind:{met:a.stats([events[i][label][kind][met] for i in ids]) for met in ['translation_m','rotation_deg']} for kind in ['raw_error','aligned_error']} for label in rows}
native={}
for label,folder in a.SOURCES.items():
    with a.pin(folder/'glim_odometry.csv').open() as f: data=list(csv.DictReader(f))
    ts=np.asarray([int(r['field.header.stamp']) for r in data],dtype=np.int64)
    poses=np.repeat(np.eye(4)[None,:,:],len(data),axis=0)
    poses[:,:3,3]=[[float(r['field.pose.pose.position.'+k]) for k in 'xyz'] for r in data]
    poses[:,:3,:3]=Rotation.from_quat([[float(r['field.pose.pose.orientation.'+k]) for k in 'xyzw'] for r in data]).as_matrix()
    native[label]=(ts,poses)
assert np.array_equal(native['old'][0],native['new'][0])
assert np.array_equal(native['old'][1][0],native['new'][1][0])
diff=a.pose_errors(native['old'][1],native['new'][1])
first=int(np.flatnonzero(diff['translation_m']>1e-12)[0])
update=events[0]['old']['merge']['update'];assert update==events[0]['new']['merge']['update']==30
initial=dict(first_different_index=first,first_different_time_s=float((native['old'][0][first]-a.EPOCH)*1e-9),
    first_difference_translation_m=float(diff['translation_m'][first]),first_difference_rotation_deg=float(diff['rotation_deg'][first]),
    before_first_injected_update={k:a.stats(v[:update]) for k,v in diff.items()},
    first_injected_update=update,first_injected_state_time_s=float((native['old'][0][update]-a.EPOCH)*1e-9),
    first_peer_raw_mean_max_difference=float(np.max(abs(rows['old'][0]['raw']-rows['new'][0]['raw']))),
    first_peer_covariance_max_difference=float(np.max(abs(rows['old'][0]['C']-rows['new'][0]['C']))))
axes={}
for label,root,stem in [('old',a.OLD,'ON_GLIM'),('new',a.NEW,'on_glim')]:
    folder=root/'evaluation/common'
    ref=a.file_interface.read_tum_trajectory_file(a.pin(folder/'reference.tum'))
    est=a.file_interface.read_tum_trajectory_file(a.pin(folder/(stem+'.tum')))
    ar=a.ape_archive(folder/(stem+'_ape.zip'));est.transform(ar['alignment_transformation_sim3'])
    v=est.positions_xyz-ref.positions_xyz
    axes[label]=dict(squared_error_by_reference_axis_m2=np.sum(v*v,axis=0).tolist())
    p=ref.positions_xyz[ref.timestamps<30];singular=np.linalg.svd(p-p.mean(axis=0),compute_uv=False)
    axes[label]['early30_reference_position_singular_values']=singular.tolist()
for p in [a.E/'main_gpu_batching_cpk16_20261020/REPORT_K16E.txt',a.W/'src/glim_ros1/src/glim_ros/cbs_bridge.cpp',
          a.W/'src/gtsam_points/src/gtsam_points/types/point_cloud_cpu_funcs.cpp']:
    a.pin(p)
for folder in a.SOURCES.values():a.pin(folder/'glim_config/config_preprocess.json')
original_delta=x['original_scores']['new_on_glim']['sse']-x['original_scores']['old_on_glim']['sse']
windows=[b for b in x['time_bins'] if b['start_s'] in [30,60,150,180,210]]
out=dict(passed=True,paired_covariance_rotation_trace_ratio=quant(ratios[:,0]),paired_covariance_translation_trace_ratio=quant(ratios[:,1]),
    paired_full_covariance_generalized_eigenvalues=quant(spectra),groups_by_binding_changed=groups,initial_divergence=initial,
    error_axes=axes,extra_sse_windows=dict(windows_s=[[b['start_s'],b['end_s']] for b in windows],
       total_extra_sse_m2=original_delta,selected_extra_sse_m2=sum(b['GLIM_ON_delta_sse'] for b in windows),
       fraction=sum(b['GLIM_ON_delta_sse'] for b in windows)/original_delta),
    caveats=['Pairwise covariance ratios do not certify calibration or independent peer information.',
       'Pre-CBS divergence is not proof of the cause or magnitude of later ATE regression.',
       'Historical sampler test demonstrates a mechanism, not prepared-cloud identity/differences in CP91/95.',
       'Early-position-only alignment sensitivity is not a causal drift onset test.'])
for p,d in a.PINS.items():assert a.sha(p)==d,p
for p,d in core['output_sha256'].items():assert a.sha(p)==d,p
a.save('supplement.json',out);a.save('supplement_inputs.json',a.PINS)
a.save('supplement_verification.json',dict(passed=True,core_outputs_unchanged=True,source_runs=0,optimizer_runs=0,
    output_sha256={str(H/n):a.sha(H/n) for n in ['supplement.json','supplement_inputs.json']}))
print(json.dumps({k:v for k,v in out.items() if k!='groups_by_binding_changed'},indent=2))
