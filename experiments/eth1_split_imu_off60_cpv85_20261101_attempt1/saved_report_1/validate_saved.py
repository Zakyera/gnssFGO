"""Exact endpoint reconciliation; no sensor or estimator execution."""
import csv,hashlib,importlib.util,json,re,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
Q=Path(__file__).resolve().parent;H=Q.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';S=P/'off_source'
def read(p):return json.loads(p.read_text())
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert read(H/'closeout.json')['passed'] and (H/'failed.json').exists()
assert not (H/'complete.json').exists() and not (H/'evaluate.log').exists()
a=read(P/'input_audit.json');epoch=a['epoch_ns'];end=a['end_ns'];streams={}
for suffix,topics in [('cpt7_imu',['/boxi/cpt7/imu']),('alphasense_imu',['/boxi/alphasense/imu']),
    ('alphasense',['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed'])]:
    path=Path(a['bag_files'][suffix]['path']);assert sha(path)==a['bag_files'][suffix]['sha256']
    with AnyReader([path]) as r:
        for c,t,raw in r.messages(connections=[c for c in r.connections if c.topic in topics],start=epoch,stop=end):
            m=r.deserialize(raw,c.msgtype);ts=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            streams.setdefault(c.topic,[]).append(ts)
imu={}
for topic,file in [('/boxi/cpt7/imu','cpt7_headers.csv'),('/boxi/alphasense/imu','alphasense_headers.csv')]:
    with (S/file).open() as f:rows=list(csv.DictReader(f))
    key=next(k for k in rows[0] if k.endswith('.stamp'))
    got=[int(r[key]) for r in rows];assert got==streams[topic]
    assert np.all(np.diff(got)>0)
    imu[topic]=dict(rows=len(got),first_ns=got[0],last_ns=got[-1],all_raw_stamps_equal=True)
cam=np.asarray(streams['/boxi/alphasense/front_left/image_raw/compressed'],np.int64)
assert np.array_equal(cam,streams['/boxi/alphasense/front_right/image_raw/compressed']) and len(cam)==601
with (S/'kimera_odometry.csv').open() as f:rows=list(csv.DictReader(f))
kt=np.asarray([int(r['field.header.stamp']) for r in rows],np.int64)
assert [int(r['field.header.seq']) for r in rows]==list(range(len(rows)))
first=int(np.flatnonzero(cam==kt[0])[0]);last=int(np.flatnonzero(cam==kt[-1])[0])
lastimu=imu['/boxi/alphasense/imu']['last_ns'];processable=(cam>=kt[0]) & (cam<=lastimu)
assert first==3 and last==599 and len(cam[cam>lastimu])==1
assert cam[-1]-lastimu==1438600
text=(S/'estimators.log').read_text()
front=[int(v) for v in re.findall(r'VioFrontend \[ms\]\s+(\d+)',text)]
left=[int(v) for v in re.findall(r'data_provider_left_frame_queue Size \[#\]\s+(\d+)',text)]
right=[int(v) for v in re.findall(r'data_provider_right_frame_queue Size \[#\]\s+(\d+)',text)]
assert max(left)==max(right)==len(cam) and max(front)==int(processable.sum())==597
assert int(re.search(r'Initialize Pipeline: timestamp = (\d+)',text)[1])==kt[0]
spec=importlib.util.spec_from_file_location('cov85',P/'coverage_checks.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
coverage={}
for role,file,topic in [('native_glim','odom_imu.txt','/boxi/cpt7/imu'),('native_kimera','kimera_odometry.csv','/boxi/alphasense/imu')]:
    coverage[role]=m.check(*m.load(S/file,role=='native_kimera'),60,imu[topic]['first_ns']*1e-9,imu[topic]['last_ns']*1e-9,role)
assert coverage['native_kimera']['span_s']>=54 and coverage['native_glim']['rows']>=580
updates=set(map(int,re.findall(r'KIMERA_BACKEND_SPINONCE_TIMING_ROW,(\d+),',text)))
assert sorted(updates)==list(range(len(rows)))
glimlogs=list(S.glob('ros_logs/*/glim*stdout.log'));assert len(glimlogs)==1
alltext=text+'\n'+glimlogs[0].read_text()
assert not re.search(r'IndeterminantLinearSystem|terminate called|Assertion.*failed|local_update_failure|\[FATAL\]|CPV40.*FAIL',alltext)
assert not any(v in alltext for v in ('persistent_odom_applied','injected_persistent','injected_active_window','sender_odom_queued'))
effective=read(S/'effective_mode_check.json');assert effective['passed'] and not effective['glim']['glim_cbs_receive_external_beliefs']
assert read(S/'hesai_startup.json')['passed']
assert 'Done.' in (S/'rosbag.log').read_text()
result=dict(passed=True,original_strict_guard_passed=False,requested_s=60,source_replayed=False,
    input_receipts=imu,coverage=coverage,frontend=dict(received_pairs=601,initialized_camera_index=first,
    preinitialization_pairs=first,terminal_unsupported_pairs=1,terminal_camera_after_last_imu_ms=(int(cam[-1])-lastimu)*1e-6,
    processed=597,expected_from_saved_support=597,last_backend_camera_index=last),
    both_receivers_off=True,external_injections=0,backend_rows=len(rows),
    limits='Counter conservation plus initialization/backend timestamps; no per-frame branch trace was enabled.')
save(Q/'source_coverage.json',result)
save(Q/'capture_complete.json',dict(passed=True,source_capture_finished=True,supplemental_endpoint_audit=True,
    original_strict_guard_passed=False,source_retries=0,duration_s=60,CBS_ON=False,fusion=False))
save(Q/'closeout.json',read(H/'closeout.json'))
print(json.dumps(result,indent=2))
