"""Independent output arithmetic and original-input immutability checks."""
import datetime,json
import numpy as np
from scipy.spatial.transform import Rotation
from localize import H,S,P,read,sha,save
def main():
    assert not (H/'verification.json').exists()
    manifests=[read(H/f) for f in ('localization.json','belief_analysis.json','findings.json')];pins={}
    for m in manifests:
        for p,h in m['input_sha256'].items():
            assert pins.setdefault(p,h)==h
    seal=read(S/'complete.json')['output_sha256']
    for p,h in seal.items():assert sha(p)==h,p
    for p,h in pins.items():assert sha(p)==h,p
    a=np.load(H/'localized.npz');z=np.load(S/'evaluation/comparison.npz');b=np.load(H/'belief_cohorts.npz')
    valid=a['valid'];t=a['time_s'];assert valid.sum()==3532 and np.array_equal(valid,z['common_valid'])
    r=read(S/'results.json');rng=np.random.default_rng(109);checks=0;relative_max=0.;roll_max=0.
    for k in r['common']:
        n=a[k+'_norm'];assert np.all(np.isnan(n[~valid]))
        assert abs(np.sqrt(np.nanmean(n*n))-r['common'][k]['ATE_RMSE_m'])<1e-12
        for i in rng.choice(np.flatnonzero(np.isfinite(a[k+'_rms5'])),100,replace=False):
            assert valid[i-25:i+26].all();err=abs(np.sqrt(np.mean(n[i-25:i+26]**2))-a[k+'_rms5'][i]);roll_max=max(roll_max,err);assert err<1e-12;checks+=1
        rot=Rotation.from_rotvec([.2,-.3,.4]).as_matrix();offset=np.array([7.,-8.,9.])
        p=z[k+'_position']@rot.T+offset;rr=rot@z[k+'_rotation'];gp=z['reference_position'];gr=z['reference_rotation']
        for i in rng.choice(np.flatnonzero(np.isfinite(a[k+'_relative5_m'])),100,replace=False):
            j=i-50;local=rr[j].T@(p[i]-p[j]);ref=gr[j].T@(gp[i]-gp[j]);err=abs(np.linalg.norm(local-ref)-a[k+'_relative5_m'][i]);relative_max=max(relative_max,err);assert err<1e-9;checks+=1
    matrices=0
    for role in ('glim_hesai','kimera_front'):
        endpoints=b[role+'_endpoints_ns'];assert np.all(endpoints[:,1]>endpoints[:,0]);assert len(np.unique(endpoints,axis=0))==len(endpoints)
        for mode in ('off','on'):
            for stage in ('first','age2.5','age4.5','terminal'):
                C=b[f'{role}_{mode}_{stage}_covariance'];e=b[f'{role}_{mode}_{stage}_residual'];assert len(C)==len(endpoints)
                assert np.max(abs(C-C.transpose(0,2,1)))<1e-12 and np.linalg.eigvalsh(C).min()>0
                q1=np.einsum('ni,ni->n',e,np.linalg.solve(C,e[...,None])[...,0]);q2=np.sum(np.linalg.solve(np.linalg.cholesky(C),e[...,None])[...,0]**2,axis=1)
                assert np.max(abs(q1-q2)/(1+q1))<1e-8;matrices+=len(C)
    f=manifests[2];lo=f['onset_same_initial_frame']['fusion_online']['0.05'][0]['start_s'];assert abs(lo-53.136453966)<1e-8
    for m in ('off','on'):
        sums=manifests[1]['accounting'][m]['versions'];assert sum(sums.values())==r['diagnostics'][m]['receiver']['summary']['received']
        assert sum(manifests[1]['accounting'][m]['terminal_payload_matches'].values())==r['diagnostics'][m]['receiver']['summary']['absorbed']
    v=dict(passed=True,checked_at=datetime.datetime.now().astimezone().isoformat(),CP108_sealed_outputs_unchanged=len(seal),all_read_inputs_unchanged=len(pins),trajectory_diagnostic_checks=checks,max_rolling_error=roll_max,max_global_gauge_invariance_error=relative_max,SPD_full6_matrices=matrices,received_and_absorbed_counts_exact=True,source_runs=0,receiver_replays=0,new_evo_calls=0,old_artifacts_changed=False)
    save(H/'verification.json',v)
    # Seal only this flat supplemental directory, never original artifacts.
    outputs={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file() and p.name!='complete.json'}
    save(H/'complete.json',dict(passed=True,finished=v['checked_at'],scope='saved-data CP108 diagnosis only',output_sha256=outputs,verification=v))
    print(json.dumps(v,indent=2))
if __name__=='__main__':main()
