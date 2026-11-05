"""Saved-data/source diagnosis only. No ROS process, solver or scoring invocation."""
import datetime, hashlib, json, re, struct, subprocess
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
H=Path(__file__).resolve().parent; E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3'); runs=W/'runs'
H71=E/'retention_full_frozen_cpv71_20261031'; P=runs/H71.name/'replay'; D=P/'remove_capture'
S=runs/'eig2_hesai_refresh_full_cpv49_20261030/prepared/full_on_source'
G=runs/'retention_update_gate_cpv70_20261031'
old=runs/'fixed_lag_retirement_fl4m_20261015'
epoch=1730638816563546034; pins={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p);pins[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def text(p):return pin(p).read_text()
def save(name,obj):
    with (H/name).open('x') as f:json.dump(obj,f,indent=2)
complete=read(H71/'complete.json');assert not complete['passed'] and not complete['scored']
assert complete['cleanup_error'] is None and complete['integrity_error'] is None
for p,v in complete['output_sha256'].items():assert sha(p)==v,p
frozen=read(P/'frozen_inputs.json');rows=frozen['rows'];assert len(rows)==1743
target=next(r for r in rows if r['sender']==[685,687]);assert target['update']==346
lines=text(D/'kimera.log').splitlines();original=text(S/'estimators.log').splitlines()
ansi=re.compile(r'\x1b\[[0-9;]*m')
def values(line,marker):return ansi.sub('',line.split(marker,1)[1]).strip().split(',')
bound=[];deliveries=[];poses=[]
for line in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v=values(line,'FL4M_FROZEN_FACTOR_ROW,')
        bound.append(dict(ids=list(map(int,v[:5])),mu=list(map(float,v[5].split(';'))),cov=list(map(float,re.split('[;:]',v[6])))))
    if 'CPV70_DELIVERY_ROW,' in line:deliveries.append(list(map(int,values(line,'CPV70_DELIVERY_ROW,'))))
    if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:poses.append(list(map(float,values(line,'KIMERA_BACKEND_POSE_PATH_ROW,'))))
assert len(bound)==len(deliveries)==340
maxmean=0.
for got,release,r in zip(bound,deliveries,rows):
    assert got['ids']==[r['update']]+r['sender']+r['receiver']
    assert release[:3]==[r['update']]+r['sender'] and release[3]<=release[0]
    assert got['cov']==r['cov'];maxmean=max(maxmean,float(np.max(abs(np.array(got['mu'])-r['aligned']))))
assert maxmean<1e-12
failures=[l for l in lines if 'CPV70_DELIVERY_FAILURE:' in l]
assert len(failures)==3 and all('missing at target update 346 sender 685->687' in l for l in failures)
assert not any('active_window_rollback' in l or 'CPV40_FATAL' in l for l in lines)
def incoming(p):
    with Reader(pin(p)) as reader:
        return [(int(t),bytes(raw)) for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_in'])]
inputs=incoming(P/'aligned_inputs.bag');received=incoming(D/'outputs.bag')
assert len(inputs)==len(rows)==1743 and len(received)==341
assert [raw for _,raw in received]==[raw for _,raw in inputs[:341]]
assert rows[340] is target and inputs[340][0]==target['arrival_ns']
native=[]
with Reader(D/'outputs.bag') as reader:
    for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/kimera_vio_ros/odometry']):
        assert c.msgtype=='nav_msgs/msg/Odometry'
        seq,sec,nsec=struct.unpack_from('<III',raw)
        native.append(dict(seq=seq,stamp_ns=sec*10**9+nsec,record_ns=int(t)))
assert len(native)==len(poses)==346 and [int(v[1]) for v in poses]==list(range(346))
assert [v['stamp_ns'] for v in native]==frozen['original_pose_timestamps_ns'][:346]
camera=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_alphasense.bag'
# Do not re-read ten GiB to decode two headers; read-only indexed message range.
camera_rows=[]
with Reader(camera) as reader:
    conns=[c for c in reader.connections if c.topic in ['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']]
    for c,t,raw in reader.messages(connections=conns,start=target['times'][4]-1000000,stop=target['times'][4]+1000000):
        assert c.msgtype=='sensor_msgs/msg/CompressedImage'
        seq,sec,nsec=struct.unpack_from('<III',raw)
        camera_rows.append(dict(topic=c.topic,record_ns=int(t),header_ns=sec*10**9+nsec))
assert len(camera_rows)==2 and all(r['header_ns']==target['times'][4] for r in camera_rows)
margins={}
for label,p,rate in [('FL4m_94s',old/'frozen_inputs.json',.10),('CP70_60s',G/'replay/frozen_inputs.json',.20),('CP71_full',P/'frozen_inputs.json',.20)]:
    rs=read(p)['rows'];delta=np.array([r['arrival_ns']-r['times'][4] for r in rs])/1e6
    margins[label]=dict(rows=len(rs),playback_rate=rate,record_arrival_minus_target_sensor_ms=dict(minimum=float(delta.min()),median=float(np.median(delta)),maximum=float(delta.max())),
        later_than_target_sensor_stamp=int(sum(delta>0)),positive_rows=[dict(update=r['update'],sender=r['sender'],delta_ms=float(d)) for r,d in zip(rs,delta) if d>0])
assert margins['FL4m_94s']['later_than_target_sensor_stamp']==0
for mode in ('remove','retain'):
    v=read(old/(mode+'_capture')/'validation.json')
    assert v['valid'] and v['exact_backend_update_ids'] and v['incoming_factor_count']==450
oldrun=text(old/'run_case.py');oldvalidate=text(old/'validate.py')
assert '--rate=.10' in oldrun and 'CPV70_RECORDED_UPDATE_SCHEDULE' not in oldrun
assert "ids==[r['update']]+r['sender']+r['receiver']" in oldvalidate
gate=text(G/'RecordedUpdateGate.h');backend=text(G/'VioBackend.cpp')
assert 'update < r.update' in gate and 'out.held.push_back(b)' in gate
assert 'missing at target update' in gate and 'condition_variable' not in gate and 'sleep' not in gate
start=backend.index('  auto pending_odom_beliefs = popPendingExternalOdometryBeliefs();')
assert backend.index('deliveryGate().select(cur_id, pending_odom_beliefs)',start)>start
wrapper=text(W/'src/Kimera-VIO-ROS/src/KimeraVioRos.cpp')
assert 'ros::WallRate rate(20)' in wrapper
assert 'flushExternalOdometryBeliefsToPipeline();' in wrapper
assert 'bufferExternalOdometryBeliefs(converted_beliefs);' in wrapper
preflight=read(D/'preflight.json');original_mode=read(S/'effective_mode_check.json')
assert '--cbs_experimental_native_imu_mean_alignment=false' in preflight['argv']
lib=W/'devel/lib/libkimera_vio_ros.so';binary=G/'private_build/libkimera_vio.so'
manifest=read(H71/'protected_before.json')
assert sha(lib)==manifest[str(lib)]
assert sha(D.parents[1]/'private_build/libkimera_vio.so')==sha(binary)
pin(lib);pin(binary)
# The original successful event and its unchanged incoming physical endpoints.
original_event=[l for l in original if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,346,685,687,' in l]
assert len(original_event)==1
cost=np.loadtxt(pin(D/'objective.csv'),delimiter=',');assert np.isfinite(cost).all()
assert len(cost)==346 and int(cost[-1,0])==345 and set(cost[:,1])=={0}
evidence=dict(passed=True,diagnosis_not_experiment_pass=True,
    cause='Recorded wall-paced input replay plus a hold-early-only exact-update gate does not guarantee backend queue readiness at the original update; gate aborts before optimizer use.',
    target=dict(update=346,sender=target['sender'],receiver=target['receiver'],capture_time_s=(target['times'][4]-epoch)*1e-9,
        physical_state_ns=target['times'][4],source_to_ns=target['times'][1],recorded_input_ns=target['arrival_ns'],
        recorded_input_after_state_ms=(target['arrival_ns']-target['times'][4])/1e6,
        recorder_received_ns=received[340][0],recorder_minus_input_record_ms=(received[340][0]-inputs[340][0])/1e6,
        recorder_payload_sha256=hashlib.sha256(received[340][1]).hexdigest(),exact_payload_present=True,
        camera_headers=camera_rows,original_successful_alignment_log=original_event[0],failure_logs=failures),
    prefix=dict(exact_received_wire_messages=341,exact_admitted_scheduled_beliefs=340,exact_native_timestamps=346,
        max_admitted_mean_roundtrip_error=maxmean,all_covariance_entries_exact=True,
        final_successful_update=345,finite_objective_rows=len(cost),retirements_before_stop=int(cost[:,2].sum())),
    comparison=margins,older_FL4m_both_validated=True,older_test_exact_updates_checked_after_capture=True,
    runtime=dict(current_backend_sha256=sha(binary),wrapper_sha256=sha(lib),parallel_run=True,
        callback_wrapper_flush_nominal_wall_hz=20,production_wrapper_unmodified=True),
    limitations=['Recorder reception is not an acknowledgement that the Kimera callback/20Hz wrapper flush/backend enqueue completed.',
        'Exact original/new callback receive, forwarding and backend-pop wall timestamps were not all saved; cannot allocate precise lateness to these stages.',
        'ROS recording times use simulated clock; millisecond differences do not prove negative wall transport latency.',
        'The failure precedes optimization at346 and RETAIN was not launched; no new retention-performance conclusion.'],
    proposed_next='Diagnostic-only bounded readiness/acknowledgement barrier for original scheduled inputs before solve, preserving message availability/order/payload/update/endpoints; do not skip/retimestamp/relax guards. Fixtures before any separately authorized fresh capture.',
    no_capture=True,no_solver=True,no_build=True,no_scoring=True,no_retry=True)
save('analysis.json',evidence)
for p,v in pins.items():assert sha(p)==v,p
for p,v in complete['output_sha256'].items():assert sha(p)==v,p
save('input_sha256.json',pins)
save('verification.json',dict(passed=True,CP71_immutable_outputs_verified=len(complete['output_sha256']),inputs_unchanged=len(pins),completed=datetime.datetime.now().astimezone().isoformat()))
print(json.dumps({k:evidence[k] for k in ('cause','target','prefix','comparison')},indent=2))
