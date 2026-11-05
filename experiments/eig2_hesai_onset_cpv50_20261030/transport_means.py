"""Check saved mean direction, identity and time; intentionally ignore covariance."""
import json
import sys
from collections import Counter
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation
sys.dont_write_bytecode=True
import analyze as a

def main():
    assert not (a.H/'transport_means.json').exists()
    a.pin(__file__); a.pin(a.__file__)
    s=a.P/'full_on_source'
    logs=[p for p in (s/'ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest']
    assert len(logs)==1
    exports={}; errors=[]
    for line in a.pin(logs[0]).open():
        if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' not in line:continue
        f=a.ANSI.sub('',line.split('GLIM_CBS_EXPORT_ENDPOINT_ROW,',1)[1]).strip().split(',')
        assert len(f)==22
        i,j=map(int,f[3:5]); pair=[]
        for offset in [7,14]:
            T=np.eye(4); T[:3,3]=np.array(f[offset:offset+3],float)
            T[:3,:3]=Rotation.from_quat(np.array(f[offset+3:offset+7],float)).as_matrix();pair.append(T)
        mean=np.array(f[21].split(';'),float)
        errors.append(a.diff(a.inv(pair[0])@pair[1],a.exp(mean)))
        exports.setdefault((i,j),[]).append(mean)
    bag=a.pin(s/'recordings/cbs.bag'); incoming={}; frames=set(); agents=set(); export_error=[]
    with AnyReader([bag]) as reader:
        connections=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_in']
        for c,t,raw in reader.messages(connections=connections):
            msg=reader.deserialize(raw,c.msgtype)
            for b in msg.beliefs:
                key=(int(b.from_pose_index),int(b.to_pose_index)); mean=np.array(b.relative_mu)
                export_error.append(min(float(np.max(abs(x-mean))) for x in exports[key]))
                incoming.setdefault(key,[]).append((mean,round(b.from_stamp_sec*1e9),round(b.to_stamp_sec*1e9)))
                frames.add(b.header.frame_id);agents.add(int(b.source_agent))
    mu_error=[];stamp_error=[];raw=[]; counts=Counter()
    for line in a.pin(s/'estimators.log').open():
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' not in line:continue
        f=a.ANSI.sub('',line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1]).strip().split(',')
        key=tuple(map(int,f[1:3])); mean=np.array(f[10].split(';'),float)
        candidates=[x for x in incoming[key] if x[1]==int(f[5]) and x[2]==int(f[6])]
        assert candidates
        mu_error.append(min(float(np.max(abs(x[0]-mean))) for x in candidates))
        stamp_error.append(0);counts[int(f[0])]+=1
    profile_checks=[];effective={}
    for mode in ['off','on']:
        root=a.P/f'full_{mode}_source';d=json.loads(a.pin(root/'effective_mode_check.json').read_text())
        assert 'imu:=/boxi/cpt7/imu' in d['kimera_argv']
        effective[mode]={k:d[k] for k in ['kimera_candidate_env','kimera','glim']}
        for relative in ['kimera_profile/ImuParams.yaml','kimera_profile/BackendParams.yaml',
                         'kimera_profile/LeftCameraParams.yaml','kimera_profile/RightCameraParams.yaml',
                         'glim_config/config_ros.json','glim_config/config_sensors.json']:
            p=a.pin(root/relative)
            if mode=='on':
                q=a.P/'full_off_source'/relative
                assert a.PINS[str(p)]==a.PINS[str(q.resolve())]
                profile_checks.append(relative)
    build=json.loads(a.pin(a.C/'backend_initial/sources.json').read_text())
    build_checks=[]
    for p,expected in build.items():
        local=Path(p.replace('/workspace/cbs_gtsam4.3',str(a.W)))
        if local.name in ['VioBackend.cpp','CbsNativeImuMeanBridge.h','BpsamOnRefresh.inc','incremental_fixed_lag_bpsam_smoother.h','bpsam.cpp']:
            a.pin(local); assert a.sha(local)==expected;build_checks.append(str(local))
    edges=json.loads(a.pin(a.H/'edge_events.json').read_text())
    summary=dict(scope='No covariance fields inspected or calculated; no source replay',
       endpoint_exports=len(errors),incoming_messages=len(export_error),aligned_candidates=len(mu_error),
       export_endpoint_translation_m=max(x['translation_m'] for x in errors),
       export_endpoint_rotation_deg=max(x['rotation_deg'] for x in errors),
       export_to_incoming_mean_component_error=max(export_error),incoming_to_raw_mean_component_error=max(mu_error),
       stamp_error_ns=max(stamp_error),sender_frames=sorted(frames),sender_agents=sorted(agents),
       updates_with_two_distinct_edges=[k for k,c in counts.items() if c==2],
       maximum_candidates_per_update=max(counts.values()),identical_profile_files=profile_checks,
       compiled_source_hash_matches=build_checks,effective=effective,
       alignment_correction={c:a.stat([x['alignment_correction'][c] for x in edges]) for c in ['rotation_deg','translation_m']},
       corrected_off_same_edge={c:a.stat([x['corrected_off_same_edge'][c] for x in edges if 'corrected_off_same_edge' in x]) for c in ['rotation_deg','translation_m']},
       input_sha256=a.PINS)
    assert summary['export_endpoint_translation_m']<1e-10
    assert summary['export_endpoint_rotation_deg']<1e-8
    assert max(export_error)<1e-12 and max(mu_error)<1e-12
    assert all(a.sha(p)==v for p,v in a.PINS.items())
    summary['inputs_unchanged']=True;a.save('transport_means.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k not in ['input_sha256','effective']},indent=2))

if __name__=='__main__':main()
