"""Qualify actual private live parser on saved data; no receiver/source execution."""
import hashlib,importlib.util,json,sys
from collections import defaultdict
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';pins={}
sys.dont_write_bytecode=True;sys.path.insert(0,str(P/'transport_tools'))
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
import live_receiver_protocol as live
import frame_adapter as a
def pin(p):
    p=Path(p)
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
def main():
    assert not (H/'live_adapter_test.json').exists()
    for p in (P/'transport_tools').glob('*'):
        if p.is_file():pin(p)
    audit=json.loads(pin(P/'input_audit.json').read_text());X=np.asarray(audit['T_alphasense_cpt7']);Y=a.geo.inv(X)
    assert np.array_equal(X,np.asarray(live.FRAME_CALIBRATION['T_alphasense_cpt7']))
    for mode in ('off','on'):live.validate_config(json.loads(pin(P/f'full_{mode}.json').read_text()))
    old={'off':'eth1_alpha_bias_rw_full_off_cpv89_20261102','on':'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'}
    results={};J=a.adjoint(Y)
    for mode,name in old.items():
        S=W/'runs'/name/'prepared'/(mode+'_source');exports=defaultdict(list);n=0;max_mean=max_cov=0.
        for line in pin(S/'estimators.log').open():
            b=live.parse_native_belief('kimera',line)
            out=live.parse_belief('kimera',line)
            if b is None:assert out is None;continue
            assert all(b[k]==out[k] for k in b if k not in ('mean','covariance'))
            M=a.geo.exp(np.array(b['mean']));N=a.geo.exp(np.array(out['mean']))
            C=np.array(b['covariance']).reshape(6,6);D=np.array(out['covariance']).reshape(6,6)
            np.linalg.cholesky(D);assert np.isfinite(D).all()
            max_mean=max(max_mean,float(np.max(abs(N-Y@M@X))))
            max_cov=max(max_cov,float(np.max(abs(D-J@C@J.T))));n+=1
            exports[b['i'],b['j']].append((N,D))
        gpaths=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(gpaths)==1
        ng=0
        for line in pin(gpaths[0]).open():
            b=live.parse_native_belief('glim',line);out=live.parse_belief('glim',line)
            assert b==out
            if b is not None:ng+=1
        meanerr=coverr=0.;nw=0
        with AnyReader([pin(S/'recordings/cbs.bag')]) as reader:
            for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_out']):
                for z in reader.deserialize(raw,c.msgtype).beliefs:
                    assert z.source_agent==ord('k')
                    M=a.geo.exp(np.array(z.relative_mu,copy=True));C=np.array(z.covariance).reshape(6,6)
                    errors=[(float(np.max(abs(M-N))),float(np.max(abs(C-D))/np.max(abs(C)))) for N,D in exports[int(z.from_pose_index),int(z.to_pose_index)]]
                    em,ec=min(errors,key=lambda v:max(v[0]/1e-9,v[1]/1e-6))
                    meanerr=max(meanerr,em);coverr=max(coverr,ec);nw+=1
        assert n>36000 and ng>100000 and nw>1400
        assert max_mean<1e-9 and max_cov==0 and meanerr<1e-9 and coverr<1e-6
        results[mode]=dict(native_kimera_versions=n,native_glim_passthrough_versions=ng,wire_same_version_matches=nw,
            expected_conjugation_max=max_mean,expected_covariance_max=max_cov,wire_mean_max=meanerr,wire_covariance_relative_max=coverr)
        print(mode,results[mode],flush=True)
    # Wrong identity/direction must be distinguishable; Jacobian retains cross terms.
    rng=np.random.default_rng(95);jac=[];wrong=[]
    for _ in range(30):
        M=a.geo.exp(rng.normal(size=6)*.2);N=Y@M@X
        numeric=np.column_stack([a.geo.log(a.geo.inv(N)@(Y@(M@a.geo.exp(np.eye(6)[j]*1e-7))@X))/1e-7 for j in range(6)])
        jac.append(float(np.max(abs(numeric-J))));wrong.append(float(np.max(abs(N-M))))
    assert max(jac)<1e-7 and min(wrong)>1e-3
    # Version IDs, observation order, endpoint timestamps, changed payload replacement unchanged.
    v=live.Versions();b=dict(i=1,j=2,ti=100,tj=200,mean=[0.]*6,covariance=np.eye(6).ravel().tolist())
    first=v.accept('kimera_front','test',b,300);assert first[5]==0
    assert v.accept('kimera_front','test',b,301) is None
    b['mean'][3]=.1;second=v.accept('kimera_front','test',b,302)
    assert second[4]==first[4] and second[5]==1 and second[8:10]==[100,200]
    result=dict(passed=True,source_captures=0,receiver_runs=0,results=results,jacobian_max=max(jac),
        identity_negative_control_min=min(wrong),metadata_and_version_policy_unchanged=True,input_sha256=pins)
    with (H/'live_adapter_test.json').open('x') as f:json.dump(result,f,indent=2)
if __name__=='__main__':main()
