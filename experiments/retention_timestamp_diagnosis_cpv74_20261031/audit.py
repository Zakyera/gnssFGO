"""Saved-only CP73 event audit; no optimizer, playback or accuracy scoring."""
import collections, datetime, hashlib, json, re, struct
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
old=E/'retention_readiness_cpv73_20261031';K=W/'runs'/old.name;P=K/'replay'
epoch=1730638816563546034;pins={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p);pins[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def save(name,obj):
    with (H/name).open('x') as f:json.dump(obj,f,indent=2)
complete=read(old/'complete.json')
assert not complete['passed'] and not complete['scored']
assert complete['cleanup_error'] is None and complete['integrity_error'] is None
for p,v in complete['output_sha256'].items():assert sha(p)==v,p
frozen=read(P/'frozen_inputs.json');expected=frozen['rows']
original=frozen['original_pose_timestamps_ns'];assert len(original)==1762
def bag(p):
    poses=[];incoming=[]
    with Reader(pin(p)) as r:
        for c,t,raw in r.messages():
            if c.topic=='/kimera/cbs/odom_belief_in':
                incoming.append(dict(record_ns=int(t),sha256=hashlib.sha256(raw).hexdigest()))
            elif c.topic=='/kimera_vio_ros/odometry':
                assert c.msgtype=='nav_msgs/msg/Odometry'
                seq,sec,nsec=struct.unpack_from('<III',raw)
                poses.append(dict(seq=seq,stamp_ns=sec*10**9+nsec,record_ns=int(t)))
    return poses,incoming
_,inputs=bag(P/'aligned_inputs.bag');assert len(inputs)==1743
assert len({r['sha256'] for r in inputs})==len(inputs)
ansi=re.compile(r'\x1b\[[0-9;]*m')
def values(l,marker):return ansi.sub('',l.split(marker,1)[1]).strip().split(',')
results={};logs={};allposes={}
for mode in ('remove','retain'):
    D=P/(mode+'_capture');poses,received=bag(D/'outputs.bag');allposes[mode]=poses
    lines=pin(D/'kimera.log').read_text().splitlines();logs[mode]=lines
    bound=[];releases=[];ready=[];published=[];retries=[];match_fail=[];failures=[];low=[]
    last_update=None;pending_low=[]
    for number,l in enumerate(lines,1):
        if 'FL4M_FROZEN_FACTOR_ROW,' in l:
            v=values(l,'FL4M_FROZEN_FACTOR_ROW,')
            bound.append(dict(ids=list(map(int,v[:5])),mu=list(map(float,v[5].split(';'))),cov=list(map(float,re.split('[;:]',v[6])))))
        if 'CPV70_DELIVERY_ROW,' in l:releases.append(list(map(int,values(l,'CPV70_DELIVERY_ROW,'))))
        if 'CPV73_READINESS_ROW,' in l:ready.append(list(map(float,values(l,'CPV73_READINESS_ROW,'))))
        if 'KIMERA_BACKEND_POSE_PATH_ROW,' in l:
            v=values(l,'KIMERA_BACKEND_POSE_PATH_ROW,');last_update=int(v[1]);published.append(v)
            if pending_low:low.append(dict(update=last_update,lines=pending_low));pending_low=[]
        if 'Low mono disparity.' in l or 'Low disparity: adding zero velocity' in l:
            pending_low.append(dict(line=number,text=l))
        if 'CBS_ODOM_RETRY_ROW_G2K,' in l:retries.append(dict(line=number,text=l))
        if 'retry_timestamp_match' in l:match_fail.append(dict(line=number,text=l))
        if 'CPV70_DELIVERY_FAILURE:' in l or 'CPV73_READINESS_FAILURE:' in l or 'CPV40_FATAL' in l or 'active_window_rollback' in l:
            failures.append(dict(line=number,text=l))
    assert len(poses)==len(published) and [int(v[1]) for v in published]==list(range(len(poses)))
    assert [p['seq'] for p in poses]==list(range(len(poses)))
    assert all(a['stamp_ns']<b['stamp_ns'] for a,b in zip(poses,poses[1:]))
    assert [p['sha256'] for p in received]==[p['sha256'] for p in inputs[:len(received)]]
    assert len({p['sha256'] for p in received})==len(received)
    maxmean=0.
    for b,r in zip(bound,expected):
        assert b['ids']==[r['update']]+r['sender']+r['receiver']
        assert b['cov']==r['cov'];maxmean=max(maxmean,float(np.max(abs(np.array(b['mu'])-r['aligned']))))
    assert maxmean<1e-12
    for release,r in zip(releases,expected):assert release[:3]==[r['update']]+r['sender'] and release[3]<=release[0]
    differences=[dict(update=i,original_ns=original[i],actual_ns=p['stamp_ns'],delta_ms=(p['stamp_ns']-original[i])/1e6,
        original_selected_s=(original[i]-epoch)/1e9,actual_selected_s=(p['stamp_ns']-epoch)/1e9)
        for i,p in enumerate(poses) if p['stamp_ns']!=original[i]]
    cost=np.loadtxt(pin(D/'objective.csv'),delimiter=',');assert np.isfinite(cost).all()
    results[mode]=dict(pose_count=len(poses),last_pose=poses[-1],last_selected_s=(poses[-1]['stamp_ns']-epoch)/1e9,
        physical_timestamp_mismatches=differences,exact_received_prefix=len(received),no_duplicate_wire=True,
        exact_admitted_prefix=len(bound),exact_admitted_covariance=True,max_mean_roundtrip_error=maxmean,
        last_admitted=bound[-1]['ids'],released_count=len(releases),last_releases=releases[-3:],
        readiness_waited=[r for r in ready if r[2]>0],finite_objective_rows=len(cost),
        final_cost_row=cost[-1].tolist(),retirements=int(cost[:,2].sum()),retries=retries,match_failures=match_fail,
        failures=failures,low_disparity_before_stop=[l for l in low if l['update']>=1545],
        visual_health_logged=any('KIMERA_VISUAL_HEALTH_FRAME_ROW,' in l for l in lines),
        keyframe_reason_logged=any('Keyframe reason:' in l for l in lines))
    # Useful short exact log excerpts, not copies of all trajectories/covariances.
    selected=[]
    keep=('CPV73_READINESS_ROW,','CPV70_DELIVERY_ROW,','CBS_ODOM_RETRY_ROW_G2K,','CBS_ODOM_MATCH_ROW_G2K,',
          'KIMERA_BACKEND_SPINONCE_TIMING_ROW,','KIMERA_BACKEND_CALLBACK_TIMING_ROW,','KIMERA_BACKEND_POSE_PATH_ROW,',
          'Low mono disparity.','Low disparity: adding zero velocity','CPV70_DELIVERY_FAILURE:')
    first=next(i for i,l in enumerate(lines) if 'CPV73_READINESS_ROW,1551,' in l)
    end=next((i for i,l in enumerate(lines) if 'CPV73_READINESS_ROW,1558,' in l),len(lines))
    for i in range(first,end):
        if any(s in lines[i] for s in keep):selected.append(f'{i+1}: {lines[i]}')
    with (H/(mode+'_event_excerpt.txt')).open('x') as f:f.write('\n'.join(selected)+'\n')
assert not results['remove']['physical_timestamp_mismatches']
assert results['remove']['exact_admitted_prefix']==1743
assert results['retain']['physical_timestamp_mismatches']
first=results['retain']['physical_timestamp_mismatches'][0]['update']
table=[]
for i in range(first-3,min(first+3,len(allposes['retain']))):
    table.append(dict(update=i,original_s=(original[i]-epoch)/1e9,
        remove_s=(allposes['remove'][i]['stamp_ns']-epoch)/1e9,
        retain_s=(allposes['retain'][i]['stamp_ns']-epoch)/1e9))
target=next(r for r in expected if r['sender']==[3103,3105])
assert target['update']==1555
target_hash=inputs[expected.index(target)]['sha256']
assert sum(p['sha256']==target_hash for p in bag(P/'retain_capture/outputs.bag')[1])==1
# Indexed source header availability only: does not prove runtime subscriber receipt.
camera=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_alphasense.bag'
camera_rows=[]
with Reader(camera) as r:
    cs=[c for c in r.connections if c.topic in ['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']]
    for c,t,raw in r.messages(connections=cs,start=original[first-3]-1,stop=original[first+3]+1):
        seq,sec,nsec=struct.unpack_from('<III',raw)
        camera_rows.append(dict(topic=c.topic,record_ns=int(t),header_ns=sec*10**9+nsec,seq=seq))
assert all(r['record_ns']==r['header_ns'] for r in camera_rows)
for stamp in (original[first],allposes['retain'][first]['stamp_ns']):
    assert len([r for r in camera_rows if r['header_ns']==stamp])==2
sources={}
for relative in ['src/Kimera-VIO/src/frontend/VisionImuFrontend.cpp',
    'src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp',
    'src/Kimera-VIO/src/pipeline/StereoImuPipeline.cpp',
    'src/Kimera-VIO/include/kimera-vio/imu-frontend/ImuFrontend.h',
    'src/Kimera-VIO/include/kimera-vio/frontend/VisualHealthAudit.h']:
    sources[relative]=pin(W/relative).read_text()
for relative in ['VioBackend.cpp','RecordedUpdateGate.h','ReplayReadiness.h','replay/run_case.py',
    'replay/kimera_profile/FrontendParams.yaml','replay/kimera_profile/PipelineParams.yaml']:
    sources[relative]=pin(K/relative).read_text()
frontend=sources['src/Kimera-VIO/src/frontend/VisionImuFrontend.cpp'];backend=sources['VioBackend.cpp'];gate=sources['RecordedUpdateGate.h']
assert 'max_time_elapsed || max_disparity_reached ||' in frontend and 'disparity_flipped || nr_features_low ||' in frontend
assert backend.index('deliveryGate().select(cur_id')<backend.index('"retry_timestamp_match"')<backend.index('requeuePendingExternalOdometryBeliefs(retry_beliefs);')
assert 'released_.insert(key); out.released.push_back(p.second);' in gate and 'duplicate already-released edge' in gate
out=dict(audit_passed=True,experiment_passed=False,results=results,first_physical_divergence_update=first,
    event_table=table,target=dict(**target,payload_sha256=target_hash,wire_occurrences_in_retain=1),
    raw_stereo_headers=camera_rows,source_checks=dict(adaptive_keyframes=True,backend_bias_feedback_to_frontend=True,
        release_before_actual_admission=True,internal_retry_uses_same_pending_queue=True,duplicate_guard_has_no_internal_retry_distinction=True),
    no_run=True,no_build=True,no_solver=True,no_scoring=True,
    limitations=['Raw bag availability is not evidence of exact per-image runtime subscriber receipt.',
        'Keyframe decision values and per-frame frontend audit were not logged; cannot uniquely distinguish adaptive selection from runtime image loss at this event.',
        'Backend bias feedback provides a mechanism for different frontend tracking, not proof it triggered this exact decision.',
        'Simply allowing retry would not recover a missing historical keyframe or satisfy the fixed-endpoint experiment contract.'])
save('analysis.json',out)
for p,v in pins.items():assert sha(p)==v,p
for p,v in complete['output_sha256'].items():assert sha(p)==v,p
save('input_sha256.json',pins)
save('verification.json',dict(passed=True,CP73_outputs_verified=len(complete['output_sha256']),audited_inputs_unchanged=len(pins),
    completed=datetime.datetime.now().astimezone().isoformat()))
print(json.dumps({k:out[k] for k in ('results','first_physical_divergence_update','event_table')},indent=2))
