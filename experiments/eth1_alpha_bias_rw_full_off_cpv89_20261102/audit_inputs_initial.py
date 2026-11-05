"""Read full raw selected window; no ROS playback or source mutation."""
import hashlib,json,sys
from pathlib import Path
import numpy as np
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';Q=W/'runs/eth1_alpha_bias_rw_off60_cpv88_20261102/prepared'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
audit=json.loads((Q/'input_audit.json').read_text());old=json.loads((Q/'expected_headers.json').read_text())
epoch=audit['epoch_ns'];end=epoch+346320000000;streams={};expected={}
topics={'hesai':['/boxi/hesai/points'],'cpt7_imu':['/boxi/cpt7/imu'],'alphasense_imu':['/boxi/alphasense/imu'],
        'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']}
for suffix,wanted in topics.items():
    info=audit['bag_files'][suffix];path=Path(info['path']);assert sha(path)==info['sha256']
    with AnyReader([path]) as reader:
        assert reader.start_time==info['start_ns'] and reader.end_time==info['end_ns']
        for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic in wanted],start=epoch,stop=end):
            m=reader.deserialize(raw,c.msgtype);ts=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            expected.setdefault(c.topic,[]).append(ts);s=streams.setdefault(c.topic,dict(frames=set(),delta=[],finite=True))
            s['frames'].add(m.header.frame_id);s['delta'].append(t-ts)
            if c.topic.endswith('/imu'):
                vals=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
                assert np.isfinite(vals).all()
                if suffix=='alphasense_imu':
                    for field,k in [('angular_velocity_covariance','gyroscope_noise_density'),('linear_acceleration_covariance','accelerometer_noise_density')]:
                        assert np.max(abs(np.asarray(getattr(m,field)).reshape(3,3)-np.eye(3)*audit['noise'][k]**2*200))<1e-15
    print('Read full selected stream',suffix,flush=True)
for topic,stamps in expected.items():
    t=np.asarray(stamps,np.int64);ds=np.diff(t)*1e-9;s=streams[topic]
    assert len(t)>3400 and np.all(ds>0) and (t[-1]-t[0])*1e-9>345
    assert (end-t[-1])*1e-9<.2 and t[0]-epoch<10**9
    assert sorted(s['frames'])==audit['streams'][topic]['frames']
    if topic in old:assert [x for x in stamps if x<epoch+60000000000]==old[topic]
    s.update(frames=sorted(s['frames']),count=len(t),first_ns=int(t[0]),last_ns=int(t[-1]),
        median_period_s=float(np.median(ds)),max_gap_s=float(ds.max()),record_header_max_delta_ns=int(np.max(abs(s.pop('delta')))))
left=expected['/boxi/alphasense/front_left/image_raw/compressed'];right=expected['/boxi/alphasense/front_right/image_raw/compressed']
assert left==right and len(left)>3460
assert sha(audit['tf']['path'])==audit['tf']['sha256']
audit.update(duration_s=346.32,end_ns=end,stereo_pairs=len(left),streams=streams,
    scope='CP89 full ETH1 window; CP88 exact profile and input prefix retained',prefix_CP88_exact=True)
save(P/'input_audit.json',audit);save(P/'expected_headers.json',expected)
print(json.dumps(dict(passed=True,duration_s=346.32,streams=streams,CP88_prefix_exact=True),indent=2))
