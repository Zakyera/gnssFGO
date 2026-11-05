"""Supplemental saved-message/frame/timing and coherent-window audit only."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from rosbags.highlevel import AnyReader
import analyze_windows as w

H=w.H

def vec(token): return np.array([float(x) for x in token.replace(':',';').split(';')])
def skew(a):
    x,y,z=a
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def exp(x):
    phi=x[:3];th=np.linalg.norm(phi);k=skew(phi)
    if th<1e-7: J=np.eye(3)+k/2+k@k/6
    else: J=np.eye(3)+(1-np.cos(th))/th**2*k+(th-np.sin(th))/th**3*k@k
    return J@x[3:],R.from_rotvec(phi).as_matrix()
def between(a,b): return a[1].T@(b[0]-a[0]),a[1].T@b[1]
def difference(a,b):
    return float(np.linalg.norm(a[0]-b[0])),float(np.rad2deg(R.from_matrix(a[1].T@b[1]).magnitude()))

def analyze(case):
    d=w.load(H/(case+'_on_summary.json'));p=Path(d['source'])
    windows,factors,alignments,merges,lines,counts=w.parse(p/'estimators.log')
    bag=w.pin(p/'recordings/cbs.bag'); payloads={};frames=set();agents=set();received=0
    with AnyReader([bag]) as reader:
        cs=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_in']
        for c,ts,data in reader.messages(connections=cs):
            msg=reader.deserialize(data,c.msgtype)
            for b in msg.beliefs:
                key=(b.from_pose_index,b.to_pose_index)
                payloads.setdefault(key,[]).append((np.array(b.relative_mu),np.array(b.covariance),b.from_stamp_sec,b.to_stamp_sec))
                frames.add(b.header.frame_id);agents.add(int(b.source_agent));received+=1
    events=w.load(H/(case+'_on_events.json'));by_k={e['k']:e for e in events}
    records=[];rotations=[];max_mu=0.;max_cov=0.;max_frozen_mu=0.;max_frozen_cov=0.;max_stamp=0
    for k,rows in alignments.items():
        for row in rows:
            sf,st,rf,rt=map(int,row[1:5]);ss,ts,rs,rts,causal=map(int,row[5:10])
            raw=vec(row[10]);aligned=vec(row[11]);cov=vec(row[12]);candidates=payloads[(sf,st)]
            b=min(candidates,key=lambda b:np.linalg.norm(b[0]-raw)+np.linalg.norm(b[1]-cov))
            max_mu=max(max_mu,float(np.max(abs(b[0]-raw))));max_cov=max(max_cov,float(np.max(abs(b[1]-cov))))
            max_stamp=max(max_stamp,abs(round(b[2]*1e9)-ss),abs(round(b[3]*1e9)-ts))
            fs=[f for f in factors[k] if list(map(int,f[1:5]))==[sf,st,rf,rt]]
            assert len(fs)==1
            max_frozen_mu=max(max_frozen_mu,float(np.max(abs(vec(fs[0][5])-aligned))))
            max_frozen_cov=max(max_frozen_cov,float(np.max(abs(vec(fs[0][6])-cov))))
            raw_pose=exp(raw);aligned_pose=exp(aligned)
            changed=difference(raw_pose,aligned_pose)
            out=dict(k=k,sender_ids=[sf,st],receiver_ids=[rf,rt],
                     from_offset_ms=(ss-rs)*1e-6,to_offset_ms=(ts-rts)*1e-6,
                     sender_duration_ms=(ts-ss)*1e-6,receiver_duration_ms=(rts-rs)*1e-6,
                     causal_margin_ms=(causal-max(ss,ts,rs,rts))*1e-6,
                     raw_translation_m=float(np.linalg.norm(raw_pose[0])),
                     aligned_translation_m=float(np.linalg.norm(aligned_pose[0])),
                     temporal_alignment_translation_change_m=changed[0],
                     temporal_alignment_rotation_change_deg=changed[1],
                     raw_rotation_deg=float(np.rad2deg(np.linalg.norm(raw[:3]))),
                     rotation_marginal_sigma_deg=np.rad2deg(np.sqrt(cov.reshape(6,6).diagonal()[:3])).tolist(),
                     translation_marginal_sigma_m=np.sqrt(cov.reshape(6,6).diagonal()[3:]).tolist())
            for suffix,win in [('before',windows.get(k-1,{})),('after',windows.get(k,{}))]:
                if rf in win and rt in win:
                    error=difference(aligned_pose,between(win[rf],win[rt]))
                    out[suffix+'_factor_translation_disagreement_m']=error[0]
                    out[suffix+'_factor_rotation_disagreement_deg']=error[1]
            records.append(out)
    for e in events:
        k=e['k']
        if 'window' not in e: continue
        ids=sorted(set(windows[k])&set(windows[k-1]))
        rold=np.array([windows[k-1][i][1] for i in ids]);rnew=np.array([windows[k][i][1] for i in ids])
        rot=R.from_matrix(rnew@np.swapaxes(rold,1,2)).mean()
        rm=rot.as_matrix();xyz=rot.as_euler('xyz',degrees=True)
        tilt=np.rad2deg(np.arccos(np.clip(rm[2,2],-1,1)))
        out=dict(k=k,time_s=e['time_s'],previous_revision_m=e['previous_revision_m'],
                 coherent_roll_deg=float(xyz[0]),coherent_pitch_deg=float(xyz[1]),coherent_yaw_deg=float(xyz[2]),
                 coherent_tilt_deg=float(tilt),coherent_rotation_deg=float(rot.magnitude()*180/np.pi),
                 old_extent_m=e['window']['old_extent_m'])
        rotations.append(out)
    cols=['from_offset_ms','to_offset_ms','sender_duration_ms','receiver_duration_ms','causal_margin_ms',
          'raw_translation_m','aligned_translation_m','raw_rotation_deg',
          'temporal_alignment_translation_change_m','temporal_alignment_rotation_change_deg',
          'before_factor_translation_disagreement_m','before_factor_rotation_disagreement_deg',
          'after_factor_translation_disagreement_m','after_factor_rotation_disagreement_deg']
    large=[e for e in rotations if e['previous_revision_m']>.5 and e['old_extent_m']>.5]
    summary=dict(case=case,source=str(p),incoming_beliefs=received,aligned_candidates=len(records),
                 sender_frame_ids=sorted(frames),sender_agents=sorted(agents),
                 incoming_vs_pre_alignment_mu_max_abs=max_mu,incoming_vs_pre_alignment_cov_max_abs=max_cov,
                 source_vs_log_stamp_max_ns=max_stamp,frozen_vs_aligned_mu_max_abs=max_frozen_mu,
                 frozen_vs_aligned_cov_max_abs=max_frozen_cov,
                 statistics={c:w.stats([x[c] for x in records if c in x]) for c in cols},
                 large_window_yaw_abs_deg=w.stats([abs(x['coherent_yaw_deg']) for x in large]),
                 large_window_tilt_deg=w.stats([x['coherent_tilt_deg'] for x in large]),
                 largest_window_events=[dict(event=e,rotation=next((r for r in rotations if r['k']==e['k']),None),
                                           candidates=[x for x in records if x['k']==e['k']]) for e in sorted(events,key=lambda e:e['previous_revision_m'],reverse=True)[:12]])
    assert max_mu<1e-12 and max_cov<1e-12 and max_stamp==0
    assert max_frozen_mu==0 and max_frozen_cov==0
    assert all(x['causal_margin_ms']>=0 for x in records)
    w.save(H/(case+'_input_events.json'),records)
    w.save(H/(case+'_rotation_events.json'),rotations)
    w.save(H/(case+'_input_summary.json'),summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='largest_window_events'},indent=2),flush=True)
    return summary

if __name__=='__main__':
    w.pin(__file__);w.pin(w.__file__)
    summaries=[analyze(c) for c in ['velodyne_eig2','hesai_eig2']]
    assert all(w.sha(p)==s for p,s in w.PINS.items())
    w.save(H/'input_results.json',dict(passed=True,cases=summaries,input_sha256=w.PINS,inputs_unchanged=True))
