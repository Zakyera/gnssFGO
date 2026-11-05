"""Exact raw SPX-3 selected-input audit; never starts an estimator."""
import datetime, hashlib, json, sys
from pathlib import Path
import numpy as np
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D=W/'src/datasets/GrandTour/SPX-3';mission='2024-11-02-17-43-10'
epoch=1730565899872888651;end=1730566096892888651
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
assert not (H/'raw_input_audit.json').exists()
prior=json.loads((H/'available_data_review.json').read_text())
topics={'hesai':['/boxi/hesai/points'],'cpt7_imu':['/boxi/cpt7/imu'],
        'alphasense_imu':['/boxi/alphasense/imu'],
        'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']}
bags={};streams={};headers={};quiet={};starts=[]
noise={'gyroscope_noise_density':0.00020202932517334086,'accelerometer_noise_density':0.0012593921763625943}
for suffix,wanted in topics.items():
    p=D/(mission+'_'+suffix+'.bag')
    with AnyReader([p]) as r:
        starts.append(r.start_time)
        bags[suffix]=dict(path=str(p),bytes=p.stat().st_size,mtime_ns=p.stat().st_mtime_ns,sha256=sha(p),start_ns=r.start_time,end_ns=r.end_time)
        for c,t,raw in r.messages(connections=[c for c in r.connections if c.topic in wanted],start=epoch,stop=end):
            m=r.deserialize(raw,c.msgtype);ts=int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            headers.setdefault(c.topic,[]).append(ts)
            s=streams.setdefault(c.topic,dict(frames=set(),delta=[]));s['frames'].add(m.header.frame_id);s['delta'].append(t-ts)
            if c.topic.endswith('/imu'):
                v=m.angular_velocity;a=m.linear_acceleration;vals=[v.x,v.y,v.z,a.x,a.y,a.z]
                assert np.isfinite(vals).all()
                if t<epoch+10**9:quiet.setdefault(suffix,[]).append(vals)
                if suffix=='alphasense_imu':
                    for field,k in [('angular_velocity_covariance','gyroscope_noise_density'),('linear_acceleration_covariance','accelerometer_noise_density')]:
                        assert np.max(abs(np.asarray(getattr(m,field)).reshape(3,3)-np.eye(3)*noise[k]**2*200))<1e-15
    print('Audited',suffix,len(headers[wanted[0]]),flush=True)
assert min(starts)+29000000000==epoch,'New IMU must not silently change rosbag playback epoch'
for topic,values in headers.items():
    ts=np.array(values,np.int64);gap=np.diff(ts)*1e-9;s=streams[topic]
    assert (gap>0).all() and (ts[-1]-ts[0])*1e-9>196
    assert (end-ts[-1])*1e-9<.2 and ts[0]-epoch<10**9
    s.update(frames=sorted(s['frames']),count=len(ts),first_ns=int(ts[0]),last_ns=int(ts[-1]),
             median_period_s=float(np.median(gap)),max_gap_s=float(gap.max()),
             record_header_max_delta_ns=int(np.max(abs(np.array(s.pop('delta'))))))
    if topic.endswith('/imu'):assert gap.max()<.03
for suffix,values in quiet.items():
    v=np.array(values);g=np.linalg.norm(v[:,:3],axis=1)
    assert len(v)>50 and g.max()<.05 and np.max(v[:,3:].std(axis=0))<.3
    assert 9<np.linalg.norm(v[:,3:].mean(axis=0))<10.5
    quiet[suffix]=dict(n=len(v),gyro_norm_max=float(g.max()),acc_mean=v[:,3:].mean(axis=0).tolist(),acc_std=v[:,3:].std(axis=0).tolist())
left=headers[topics['alphasense'][0]];right=headers[topics['alphasense'][1]];paired=sorted(set(left)&set(right))
assert len(left)==len(right)==len(paired)==1970
assert streams['/boxi/hesai/points']['count']==1970 and streams['/boxi/cpt7/imu']['count']==19702
assert abs(streams['/boxi/alphasense/imu']['median_period_s']-.005)<1e-5
tf=Path(prior['TF']['path']);assert sha(tf)==prior['TF']['sha256']
result=dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),epoch_ns=epoch,end_ns=end,
    playback_epoch_ns=min(starts),offset_s=29.0,duration_s=197.02,mission=mission,bag_files=bags,
    streams=streams,quiet_first_second=quiet,noise=noise,noise_density_matches_raw_message_covariance=True,
    stereo_pairs=len(paired),stereo_pair_stamps_ns=paired,
    raw_unmatched_camera_headers=dict(left_only_ns=sorted(set(left)-set(right)),right_only_ns=sorted(set(right)-set(left))),
    T_alphasense_cpt7=prior['T_alpha_cpt7'],T_alphasense_prism=prior['T_alpha_prism'],T_cpt7_prism=prior['T_cpt7_prism'],
    tf=dict(path=str(tf),sha256=sha(tf)),scope='Original raw SPX-3 split-IMU full selected window; not a calibration certification')
save('raw_input_audit.json',result);save('expected_headers.json',headers)
print(json.dumps(dict(passed=True,streams=streams,quiet=quiet),indent=2))
