"""Read raw bags and author calibration; save a new admission artifact, no replay."""
import hashlib,json,re,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';D=W/'src/datasets/GrandTour'
mission='2024-10-01-11-29-55'
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read_yaml(p):return yaml.safe_load(p.read_text().replace('%YAML:1.0',''))
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def stamp(m):return int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
tfpath=D/(mission+'_tf_minimal.bag');edges={}
with AnyReader([tfpath]) as reader:
    for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/tf_static']):
        msg=reader.deserialize(raw,c.msgtype)
        for tr in msg.transforms:
            x=tr.transform;T=np.eye(4)
            T[:3,:3]=Rotation.from_quat([x.rotation.x,x.rotation.y,x.rotation.z,x.rotation.w]).as_matrix()
            T[:3,3]=[x.translation.x,x.translation.y,x.translation.z]
            key=(tr.header.frame_id,tr.child_frame_id)
            if key in edges:assert np.max(abs(edges[key]-T))<1e-12
            edges[key]=T
        if len(edges)>=64:break
assert len(edges)==64
def transform(start,end):
    seen={start};queue=[(start,np.eye(4))]
    for node,T in queue:
        if node==end:return T
        for (parent,child),edge in edges.items():
            if parent==node and child not in seen:
                seen.add(child);queue.append((child,T@edge))
            if child==node and parent not in seen:
                seen.add(parent);queue.append((parent,T@np.linalg.inv(edge)))
    raise ValueError((start,end))
E=transform('alphasense_imu','cpt7_imu')
calibration={}
baseline=W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared/input_profiles'
for side in ('Left','Right'):
    cfg=read_yaml(P/f'input_profiles/kimera_profile/{side}CameraParams.yaml')
    old=read_yaml(baseline/f'kimera_profile/{side}CameraParams.yaml')
    raw=transform('alphasense_imu','alphasense_front_'+side.lower())
    got=np.asarray(cfg['T_BS']['data']).reshape(4,4)
    assert np.max(abs(raw-got))<1e-9
    assert np.max(abs(E@np.asarray(old['T_BS']['data']).reshape(4,4)-got))<1e-9
    a=dict(cfg);b=dict(old);a.pop('T_BS');b.pop('T_BS');assert a==b
    calibration[side]=dict(T_BS=raw.tolist(),maximum_error=float(np.max(abs(raw-got))))
for p in (P/'input_profiles/glim_config').rglob('*'):
    if p.is_file():assert p.read_bytes()==(baseline/'glim_config'/p.relative_to(P/'input_profiles/glim_config')).read_bytes()
noise=read_yaml(H/'author_alphasense_imu.yaml')
imu=read_yaml(P/'input_profiles/kimera_profile/ImuParams.yaml')
old=read_yaml(baseline/'kimera_profile/ImuParams.yaml')
for key in ('gyroscope_noise_density','accelerometer_noise_density','gyroscope_random_walk','accelerometer_random_walk'):
    assert imu[key]==noise[key];old[key]=noise[key]
old['rate_hz']=200;assert old==imu
for p in (baseline/'kimera_profile').rglob('*'):
    if p.is_file() and p.name not in ('LeftCameraParams.yaml','RightCameraParams.yaml','ImuParams.yaml'):
        assert p.read_bytes()==(P/'input_profiles/kimera_profile'/p.relative_to(baseline/'kimera_profile')).read_bytes()
bags={};start=[]
for suffix in ('hesai','cpt7_imu','alphasense','alphasense_imu'):
    path=D/(mission+'_'+suffix+'.bag')
    with AnyReader([path]) as r:
        start.append(r.start_time)
        bags[suffix]=dict(path=str(path),bytes=path.stat().st_size,mtime_ns=path.stat().st_mtime_ns,
                          sha256=sha(path),start_ns=r.start_time,end_ns=r.end_time)
epoch=min(start);end=epoch+60*10**9
streams={};arrays={}
topics={'hesai':['/boxi/hesai/points'],'cpt7_imu':['/boxi/cpt7/imu'],
        'alphasense_imu':['/boxi/alphasense/imu'],
        'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']}
for suffix,wanted in topics.items():
    with AnyReader([Path(bags[suffix]['path'])]) as reader:
        for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted],start=epoch,stop=end):
            m=reader.deserialize(raw,c.msgtype);s=streams.setdefault(c.topic,dict(stamps=[],frames=set(),record_delta=[]))
            ts=stamp(m);s['stamps'].append(ts);s['frames'].add(m.header.frame_id);s['record_delta'].append(t-ts)
            if suffix in ('cpt7_imu','alphasense_imu'):
                vals=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
                arrays.setdefault(suffix,[]).append(vals)
                if suffix=='alphasense_imu':
                    for field,k in [('angular_velocity_covariance','gyroscope_noise_density'),('linear_acceleration_covariance','accelerometer_noise_density')]:
                        assert np.max(abs(np.asarray(getattr(m,field)).reshape(3,3)-np.eye(3)*noise[k]**2*200))<1e-15
for topic,s in streams.items():
    ts=np.asarray(s.pop('stamps'),np.int64);ds=np.diff(ts)*1e-9
    assert len(ts)>500 and (ds>0).all()
    assert (ts[-1]-ts[0])*1e-9>59 and (end-ts[-1])*1e-9<.2
    s.update(count=len(ts),first_ns=int(ts[0]),last_ns=int(ts[-1]),median_period_s=float(np.median(ds)),max_gap_s=float(ds.max()),
             frames=sorted(s['frames']),record_header_max_delta_ns=int(np.max(abs(np.asarray(s.pop('record_delta'))))))
    if 'front_left' in topic:left=ts
    if 'front_right' in topic:right=ts
    if topic.endswith('/imu'):
        suffix='alphasense_imu' if 'alphasense' in topic else 'cpt7_imu';v=np.asarray(arrays[suffix]);assert np.isfinite(v).all()
        quiet=v[ts<epoch+10**9]
        assert len(quiet)>50 and np.linalg.norm(quiet[:,:3],axis=1).max()<.05
        assert np.max(np.std(quiet[:,3:],axis=0))<.3
        assert 9<np.linalg.norm(np.mean(quiet[:,3:],axis=0))<10.5
        s.update(first_second_gyro_mean=quiet[:,:3].mean(axis=0).tolist(),first_second_acc_mean=quiet[:,3:].mean(axis=0).tolist(),
                 first_second_acc_std=quiet[:,3:].std(axis=0).tolist(),finite_all=True)
assert np.array_equal(left,right)
assert len(left)>=598
result=dict(passed=True,epoch_ns=epoch,end_ns=end,duration_s=60,stereo_pairs=len(left),
            author_commit='3dcc98fc15694f9a96d7b9ecfc60097de8d1c914',noise=noise,
            noise_density_matches_raw_message_covariance=True,quiet_first_second=True,streams=streams,
            calibration=calibration,T_alphasense_cpt7=E.tolist(),T_cpt7_prism=transform('cpt7_imu','prism').tolist(),
            T_alphasense_prism=transform('alphasense_imu','prism').tolist(),bag_files=bags,
            tf=dict(path=str(tfpath),sha256=sha(tfpath),static_edges=64),only_expected_profile_fields_changed=True)
save(P/'input_audit.json',result)
print(json.dumps(result,indent=2))
