"""Fresh mechanically rebased harness and exact raw 12s input audit. No launch."""
import ast,difflib,hashlib,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'eth1_split_imu_persistent_on_cpv91_20261102_attempt1';Q=W/'runs'/OLD.name/'prepared'
P=W/'runs'/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not P.exists();P.mkdir(parents=True)
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11610))
diff=[]
for name in ('source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py','map_guard.py','coverage_checks.py','coverage_control.py','active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch'):
    old=(Q/name).read_text();new=old.replace(OLD.name,H.name).replace('11592','11610').replace('346.32','12.0').replace('CP-V91','CP-V98').replace('cpv91_frame','cpv98_frame')
    if name=='kimera_vio_ros.launch':
        anchor='    <env name="CPV76_PERSISTENT_COMPAT" value="1"/>'
        assert new.count(anchor)==1
        new=new.replace(anchor,anchor+'\n    <env name="CPV66_AUDIT_DIR" value="$(env ROS_LOG_DIR)/../checkpoints"/>\n    <env name="CPV66_TEST_ALL" value="1"/>')
    if name=='verify_effective_on.py':
        anchor="assert env(pid)['CPV76_PERSISTENT_COMPAT']=='1' and env(pid).get('CPV69_RETAIN_COMPAT','0')=='0'"
        assert new.count(anchor)==1
        new=new.replace(anchor,anchor+"\nassert Path(env(pid)['CPV66_AUDIT_DIR']).resolve()==source/'checkpoints'\nassert env(pid)['CPV66_TEST_ALL']=='1'\nassert 'CPV66_AUDIT_DIR' not in env(glim_pid) and 'CPV66_TEST_ALL' not in env(glim_pid)")
        new=new.replace("kimera_body='alphasense_imu'","observer=dict(root=env(pid)['CPV66_AUDIT_DIR'],test_all=True,clones_only=True),kimera_body='alphasense_imu'")
    with (P/name).open('x') as f:f.write(new)
    (P/name).chmod((Q/name).stat().st_mode&0o777)
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(Q/name),tofile=str(P/name)))
for name in ('input_profiles','private_build'):shutil.copytree(Q/name,P/name)
with (H/'harness.diff').open('x') as f:f.writelines(diff)
parity=[]
for name in ('input_profiles','private_build'):
    for src in sorted((Q/name).rglob('*')):
        if src.is_file():
            dst=P/name/src.relative_to(Q/name);assert sha(src)==sha(dst)
            parity.append(dict(old=str(src),new=str(dst),sha256=sha(src)))
save(H/'profile_parity.json',dict(passed=True,files=parity))
audit=read(Q/'input_audit.json');oldheaders=read(Q/'expected_headers.json');epoch=audit['epoch_ns'];end=epoch+12000000000
streams={};expected={}
topics={'hesai':['/boxi/hesai/points'],'cpt7_imu':['/boxi/cpt7/imu'],'alphasense_imu':['/boxi/alphasense/imu'],
        'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']}
for suffix,wanted in topics.items():
    info=audit['bag_files'][suffix];path=Path(info['path']);assert sha(path)==info['sha256']
    with AnyReader([path]) as reader:
        assert reader.start_time==info['start_ns'] and reader.end_time==info['end_ns']
        for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted],start=epoch,stop=end):
            m=reader.deserialize(raw,c.msgtype);ts=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            expected.setdefault(c.topic,[]).append(ts);s=streams.setdefault(c.topic,dict(frames=set(),delta=[]))
            s['frames'].add(m.header.frame_id);s['delta'].append(t-ts)
            if c.topic.endswith('/imu'):
                assert np.isfinite([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]).all()
                if suffix=='alphasense_imu':
                    for field,k in [('angular_velocity_covariance','gyroscope_noise_density'),('linear_acceleration_covariance','accelerometer_noise_density')]:
                        assert np.max(abs(np.asarray(getattr(m,field)).reshape(3,3)-np.eye(3)*audit['noise'][k]**2*200))<1e-15
    print('Audited raw prefix',suffix,flush=True)
for topic,stamps in expected.items():
    t=np.asarray(stamps,np.int64);ds=np.diff(t)*1e-9;s=streams[topic]
    assert len(t)>=100 and np.all(ds>0) and (t[-1]-t[0])*1e-9>11.7
    assert oldheaders[topic][:len(stamps)]==stamps
    assert sorted(s['frames'])==audit['streams'][topic]['frames']
    s.update(frames=sorted(s['frames']),count=len(t),first_ns=int(t[0]),last_ns=int(t[-1]),median_period_s=float(np.median(ds)),max_gap_s=float(ds.max()),record_header_max_delta_ns=int(np.max(np.abs(s.pop('delta')))))
left=expected[topics['alphasense'][0]];right=expected[topics['alphasense'][1]];paired=sorted(set(left)&set(right))
assert left==right==paired
audit.pop('historical_pair_count',None);audit.pop('prefix_CP88_exact',None)
audit.update(duration_s=12.0,end_ns=end,stereo_pairs=len(paired),stereo_pair_stamps_ns=paired,
    raw_unmatched_camera_headers=dict(left_only_ns=[],right_only_ns=[]),streams=streams,
    scope='CP98 exact raw12s prefix; CP91 settings unchanged',prefix_CP91_exact=True)
save(P/'input_audit.json',audit);save(P/'expected_headers.json',expected)
pins={}
for terminal in (OLD/'complete.json',E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1/complete.json'):
    d=read(terminal);assert d['passed'];pins[str(terminal)]=sha(terminal)
    for p,h in d['output_sha256'].items():assert sha(p)==h,p;pins[p]=h
for root in (E/'eth1_kimera_early_covariance_cpv97_20261103',E/'eth1_glim_on_regression_cpv96_20261103'):
    for p in root.iterdir():
        if p.is_file():pins[str(p)]=sha(p)
for v in audit['bag_files'].values():pins[v['path']]=v['sha256']
pins[audit['tf']['path']]=audit['tf']['sha256']
cp76=W/'runs/persistent_compatibility_cpv76_20261101'
for n in ('Checkpoint.h','Binary.h','persistent_test.cpp','OriginalChartPrior.h','include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h','release_fixture1/passed.json','release_fixture1/compatibility.log'):
    p=cp76/n;pins[str(p)]=sha(p)
for x in parity:pins[x['old']]=x['sha256']
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file():continue
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
        pins[str(p)]=sha(p)
free=shutil.disk_usage(P).free;assert free>20*1024**3
assert (P/'source_runner_on.sh').stat().st_mode&0o777==0o755
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),duration_s=12.0,containers=states,protected_files=len(pins),free_bytes=free,initial_floor_bytes=20*1024**3,runtime_floor_bytes=10*1024**3,hesai=hesai,source_binaries_unchanged=True,source_runs=0))
print('PREPARED',len(pins),json.dumps(streams),flush=True)
