"""Read-only numerical analysis and validation; never invokes an optimizer."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import ROOT,PRIOR,BASE,CASES,sha,rows,git_state

manifest=json.loads((ROOT/'manifest.json').read_text())
assert json.loads((ROOT/'complete.json').read_text())=={'passed':True,'new_solves':2,'original_graph_unchanged':True}
for p,h in manifest['protected'].items():assert sha(p)==h,p
assert all(git_state()[k]==manifest[k] for k in git_state())
expected={'glim_hesai':1189,'kimera_front':581,'wnoa':1192,'prior':3}
baseline=np.loadtxt(BASE/'on_wnoa/snapshot_final/incremental_states.csv',delimiter=',',skiprows=1)
original=rows(PRIOR/'unchanged_control/initial_factors.csv')
checks={};own_families={};fixed_families={};raw={}

def family_summaries(rr):
    assert len(rr)==2965
    result={}
    for kind in expected:
        ff=[r for r in rr if r['kind']==kind];assert len(ff)==expected[kind]
        costs=np.array([float(r['half_squared_whitened_error']) for r in ff]);assert np.isfinite(costs).all()
        result[kind]={'count':len(ff),'cost':float(costs.sum()),'mean_squared_whitened_residual':float(2*costs.mean())}
    return result

for label,selected in [('glim_off_cov','glim_hesai'),('kimera_off_cov','kimera_front')]:
    p=ROOT/label;meta=json.loads((p/'optimizer.json').read_text());valid=json.loads((ROOT/(label+'_validation.json')).read_text())
    assert valid['passed'] and valid['changed_factors']==expected[selected] and valid['unchanged_non_source_factors']==1195
    assert valid['unchanged_source_factors']+valid['changed_factors']==1770
    assert valid['raw_error']==valid['jacobian_error']==valid['original_probe_cov_error']==0
    assert valid['changed_covariance_error']<1e-12 and valid['original_probe_residual_error']<1e-9
    assert meta['normalized_gradient']<=1e-5 and meta['batch_iterations']<500 and meta['polish_iterations']<100
    init=rows(p/'initial_factors.csv');assert len(init)==len(original)
    for a,b in zip(original,init):
        assert all(a[k]==b[k] for k in ['slot','kind','dimension','keys'])
        if a['kind']!=selected:assert a==b
    own_families[label]={stage:family_summaries(rows(p/(stage+'_factors.csv'))) for stage in ['initial','final']}
    states=np.loadtxt(p/'states.csv',delimiter=',',skiprows=1)
    assert np.array_equal(states[:,:2],baseline[:,:2]) and np.isfinite(states).all()
    assert np.max(abs(np.linalg.norm(states[:,5:9],axis=1)-1))<1e-12
    pos=np.linalg.norm(states[:,2:5]-baseline[:,2:5],axis=1)
    angle=(Rotation.from_quat(baseline[:,5:9]).inv()*Rotation.from_quat(states[:,5:9])).magnitude()
    checks[label]={'position_rms_m':float(np.sqrt(np.mean(pos**2))),'position_max_m':float(pos.max()),
                   'rotation_rms_deg':float(np.sqrt(np.mean(angle**2))*180/np.pi),'optimizer':meta,'identity':valid}

for label in ['original_on',*CASES,'both_off_cov']:
    fixed_families[label]=family_summaries(rows(ROOT/'fixed_on_noise'/(label+'_factors.csv')))
    rr=rows(ROOT/'fixed_on_noise'/(label+'_source_raw.csv'));assert len(rr)==1770
    raw[label]={}
    for source in ['glim_hesai','kimera_front']:
        r=np.array([[float(row[f'r{i}']) for i in range(6)] for row in rr if row['source']==source]);assert r.shape==(expected[source],6)
        raw[label][source]={}
        for channel,part,scale in [('phi_deg',r[:,:3],180/np.pi),('rho_m',r[:,3:],1)]:
            n=np.linalg.norm(part,axis=1)*scale
            raw[label][source][channel]={'rms':float(np.sqrt(np.mean(n*n))),'median':float(np.median(n)),'p95':float(np.quantile(n,.95)),'max':float(n.max())}
    if label=='original_on':
        for a,b in zip(original,rows(ROOT/'fixed_on_noise'/(label+'_factors.csv'))):
            assert all(a[k]==b[k] for k in ['slot','kind','dimension','keys'])
            assert abs(float(a['half_squared_whitened_error'])-float(b['half_squared_whitened_error']))<1e-8

metrics=json.loads((ROOT/'metrics.json').read_text())
prior_metrics=json.loads((PRIOR/'metrics.json').read_text())
assert all(metrics[k]==v for k,v in prior_metrics.items())
effects={}
for metric in ['ate_m','translation_rpe_m','rotation_rpe_deg']:
    base=metrics['on_fused_final'][metric];g=metrics['glim_off_cov'][metric];k=metrics['kimera_off_cov'][metric];both=metrics['on_means_off_cov'][metric]
    effects[metric]={'original_on':base,'glim_only':g,'kimera_only':k,'both':both,
                    'glim_change':g-base,'kimera_change':k-base,'both_change':both-base,
                    'finite_interaction_both_minus_glim_minus_kimera_plus_on':both-g-k+base,
                    'glim_relative_change':g/base-1,'kimera_relative_change':k/base-1}
result={'passed':True,'new_solves':2,'protected_files_unchanged':len(manifest['protected']),**git_state(),
        'state_checks':checks,'own_noise_families':own_families,'fixed_on_noise_families':fixed_families,
        'raw_source_residuals':raw,'metric_effects':effects,
        'diagnostic_binary_sha256':sha(ROOT/'isolation'),
        'artifact_hashes':{str(p.relative_to(ROOT)):sha(p) for p in ROOT.rglob('*') if p.is_file() and p.suffix in ['.cpp','.py','.json','.tum','.csv']}}
with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['artifact_hashes','own_noise_families','fixed_on_noise_families','raw_source_residuals']},indent=2))
