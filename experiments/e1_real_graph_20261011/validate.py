#!/usr/bin/env python3
"""Independent post-capture matrix and preservation check; never changes gates."""
import hashlib
import json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/glim_e1_real_graph_20261011/prepared'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for c in iter(lambda:f.read(1048576),b''):h.update(c)
    return h.hexdigest()
assert not (ROOT/'audit/FAILED.txt').exists()
assert json.loads((ROOT/'control/complete.json').read_text())['passed']
assert (ROOT/'e1_source/complete.marker').exists()
effective=json.loads((ROOT/'e1_source/effective_mode_check.json').read_text())
assert effective['passed'] and effective['checked_before_playback']
coverage=json.loads((ROOT/'e1_source/source_coverage.json').read_text()); assert coverage['passed']
rows=[]; matrices={}; first_pose=[]
for n in [10,30,60,100,150,180]:
    d=ROOT/'audit'/('sample_'+str(n)); summary=json.loads((d/'summary.json').read_text())
    assert summary['passed'] and summary['values_unchanged'] and summary['gpu_factors']>0
    assert (d/'means_unchanged.marker').exists()
    keys=(d/'structure.txt').read_text().splitlines()[1].split()[1:]
    first_pose.append(min(int(k[1:]) for k in keys if k.startswith('x')))
    checks={}
    for stem,atol in [('hessian',1e-8),('covariance',1e-10)]:
        a=np.loadtxt(d/(stem+'_A.txt')); assert np.isfinite(a).all()
        for suffix in ['B','A2']:
            b=np.loadtxt(d/(stem+'_'+suffix+'.txt'))
            assert a.shape==b.shape and np.isfinite(b).all()
            delta=float(np.max(np.abs(a-b))); bound=atol+1e-6*float(np.max(np.abs(a)))
            assert delta<=bound,(n,stem,suffix,delta,bound)
            checks[stem+'_'+suffix]={'max_abs':delta,'bound':bound}
    for i in range(summary['pairs']):
        a=np.loadtxt(d/('relative_A_%d.txt'%i)); b=np.loadtxt(d/('relative_B_%d.txt'%i))
        assert a.shape==b.shape==(6,6) and np.isfinite(a).all() and np.isfinite(b).all()
        delta=float(abs(a-b).max()); bound=1e-10+1e-6*float(abs(a).max())
        assert delta<=bound
        checks['relative_'+str(i)]={'max_abs':delta,'bound':bound}
    summary['independent_checks']=checks; summary['oldest_pose_index']=first_pose[-1]
    rows.append(summary)
    for p in d.iterdir():
        if p.is_file():matrices[str(p)]=sha(p)
assert first_pose[-1]>first_pose[0],first_pose
# Preserve copied sensor profile bytes, not just selected parameter values.
baseline=WS/'runs/grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
for tree in ['glim_config','kimera_profile']:
    before={str(p.relative_to(baseline/tree)):sha(p) for p in (baseline/tree).rglob('*') if p.is_file()}
    after={str(p.relative_to(ROOT/'e1_source'/tree)):sha(p) for p in (ROOT/'e1_source'/tree).rglob('*') if p.is_file()}
    assert before==after,(tree,'profile changed')
preflight=json.loads((ROOT/'preflight.json').read_text())
for path,h in preflight['prepared_hashes'].items(): assert sha(path)==h,path
prior=json.loads((HERE.parent/'e1_sync_compatible_batching_20261011/validation_manifest.json').read_text())
protected=json.loads((WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/protected_manifest.json').read_text())
changed={p:{'baseline':v,'candidate':sha(p)} for p,v in protected.items() if sha(p)!=v}
assert changed==prior['intentional_source_changes']
for p,h in prior['prior_candidate_and_baseline_binary_checks'].items():assert sha(p)==h,p
counts={k:0 for k in ['GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_remove_committed',
                      'GLIM_CBS_ODOM_INJECT_ROW','active_window_temporary_odom_applied']}
for path in (ROOT/'e1_source/ros_logs').rglob('*stdout.log'):
    if path.name.startswith('rosout'):continue
    with path.open(errors='replace') as f:
        for line in f:
            for token in counts:
                if token in line: counts[token]+=1
assert all(v>0 for v in counts.values()),counts
result={'passed':True,'samples':rows,'coverage':coverage,'source_lifecycle_log_counts':counts,
        'unchanged_protected_entries':len(protected)-len(changed),'new_protected_changes':[],
        'matrix_and_sample_hashes':matrices,'no_receiver':True,'not_live_throughput_acceptance':True}
with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='matrix_and_sample_hashes'},indent=2))
