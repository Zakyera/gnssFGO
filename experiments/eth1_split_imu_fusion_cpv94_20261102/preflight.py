"""Read-only input tests plus fresh preflight receipt; never launches fusion."""
import hashlib, importlib.util, json, shutil, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader

H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
pins={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p);pins[str(p)]=sha(p);return p
def module(name,p):
    s=importlib.util.spec_from_file_location(name,pin(p));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def main():
    assert not (H/'preflight.json').exists()
    pin(__file__);pin(H/'PROTOCOL.txt')
    a=module('cp94_adapter',H/'frame_adapter.py')
    pin(a._geometry)
    protocol=module('cp94_protocol',E.parent/'tools/live_receiver_protocol.py')
    names={'OFF':'eth1_alpha_bias_rw_full_off_cpv89_20261102','ON':'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'}
    frame=json.loads(pin(E/names['ON']/'frame_validation.json').read_text());assert frame['passed']
    X=np.array(frame['T_alphasense_cpt7']);Y=a.geo.inv(X);AX=a.adjoint(X)
    results={}
    for mode,name in names.items():
        source=W/'runs'/name/'prepared'/(mode.lower()+'_source')
        n=0;meanerr=coverr=0.;exports=defaultdict(list)
        for line in pin(source/'estimators.log').open():
            if 'KIMERA_CBS_REVISION_EDGE_ROW,' not in line:continue
            b=protocol.parse_belief('kimera',line);out=a.native_to_cpt7('kimera',b,X)
            assert all(out[k]==b[k] for k in b if k not in ('mean','covariance'))
            M=a.geo.exp(np.array(b['mean']));N=a.geo.exp(np.array(out['mean']))
            C=np.array(b['covariance']).reshape(6,6);D=np.array(out['covariance']).reshape(6,6)
            np.linalg.cholesky(D)
            meanerr=max(meanerr,float(np.max(abs(X@N@Y-M))))
            coverr=max(coverr,float(np.max(abs(AX@D@AX.T-C))/np.max(abs(C))))
            exports[(b['i'],b['j'])].append((N,D))
            n+=1
        topics=[];counts={'g':0,'k':0};revisions=set();wiremean=wirecov=0.
        with AnyReader([pin(source/'recordings/cbs.bag')]) as r:
            topics=[dict(topic=c.topic,count=c.msgcount) for c in r.connections]
            for c,t,raw in r.messages(connections=[c for c in r.connections if '/cbs/odom_belief_' in c.topic]):
                for z in r.deserialize(raw,c.msgtype).beliefs:
                    counts[chr(z.source_agent)]+=1;revisions.add(int(z.revision))
                    if z.source_agent==ord('k'):
                        M=a.geo.exp(np.array(z.relative_mu,copy=True));C=np.array(z.covariance).reshape(6,6)
                        options=exports[(int(z.from_pose_index),int(z.to_pose_index))];assert options
                        errors=[(float(np.max(abs(M-N))),float(np.max(abs(C-D))/np.max(abs(C)))) for N,D in options]
                        em,ec=min(errors,key=lambda v:max(v[0]/1e-9,v[1]/1e-6))
                        wiremean=max(wiremean,em);wirecov=max(wirecov,ec)
        # Exact named transport records are absent from source and prepared roots.
        journals=[str(p) for root in (source.parent,E/name) for pat in ('**/events.csv','**/receipts.jsonl','**/transport_complete.json') for p in root.glob(pat)]
        results[mode]=dict(native_Kimera_versions_converted=n,mean_roundtrip_max=meanerr,
            covariance_relative_roundtrip_max=coverr,body_test_pass=bool(meanerr<1e-9 and coverr<1e-12 and wiremean<1e-9 and wirecov<1e-6),
            actual_wire_same_version_mean_max=wiremean,actual_wire_same_version_covariance_relative_max=wirecov,
            bag_topics=topics,wire_belief_counts=counts,wire_source_revision_values=sorted(revisions),
            receiver_journals_or_receipts=journals)
        assert results[mode]['body_test_pass'] and not journals
        print(mode,results[mode],flush=True)
    jac=[];rng=np.random.default_rng(94)
    for _ in range(30):
        M=a.geo.exp(rng.normal(size=6)*.2);N=Y@M@X
        J=np.column_stack([a.geo.log(a.geo.inv(N)@(Y@(M@a.geo.exp(np.eye(6)[j]*1e-7))@X))/1e-7 for j in range(6)])
        jac.append(float(np.max(abs(J-a.adjoint(Y)))))
    assert max(jac)<1e-7
    free=shutil.disk_usage(H).free
    historical_allocated=2123153408 # du -s -B1 of completed CP84 full_on_receiver
    forecast=2*1.5*historical_allocated*(346.32/355.02)
    required=max(8*2**30,forecast+5*2**30)
    terminal={}
    for mode,name in names.items():
        done=json.loads(pin(E/name/'complete.json').read_text())
        for p,d in done['output_sha256'].items():assert sha(p)==d,p
        terminal[mode]=len(done['output_sha256'])
    assert all(sha(p)==d for p,d in pins.items())
    data=dict(ready_to_launch=False,frame_tests_pass=True,frame_conversion_candidate_not_installed=True,
        source_checks=results,synthetic_adjoint_jacobian_error_max=max(jac),
        full_revision_recorded_receiver_timing_available=False,
        storage=dict(free_bytes=free,receiver_pair_forecast_bytes=forecast,required_bytes=required,
            shortfall_bytes=max(0,required-free),passes=free>=required,historical_allocated_receiver_bytes=historical_allocated,
            forecast_basis='2 arms x 1.5 margin x CP84 allocated receiver bytes x duration ratio; planning estimate, not guaranteed bound'),
        blockers=['CP89/91 did not run fusion transport; full revision receiver receipt times and merged event order were not recorded.',
        'Recorded ROS CBS bags contain the first-publication stream, not every revision consumed by the standard fusion adapter.',
        'Available disk is below preserved launch/forecast/reserve admission. No deletion authorized.'],
        receiver_runs_started=0,source_runs_started=0,input_sha256=pins,terminal_hash_counts=terminal)
    with (H/'preflight.json').open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in data.items() if k not in ('source_checks','input_sha256')},indent=2))

if __name__=='__main__':main()
