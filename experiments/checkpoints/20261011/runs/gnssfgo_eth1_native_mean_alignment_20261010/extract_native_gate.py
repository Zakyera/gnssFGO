"""Read-only ROS capture extraction; covariance equality gate, no estimator."""
from pathlib import Path
import hashlib
import io
import json
import re
import sys
import numpy as np
import rosbag

root = Path(__file__).resolve().parent
case = sys.argv[1]
out = root/('native8_'+case)
assert (out/'capture_complete.json').exists()
text = (out/'estimators.log').read_text(errors='replace')
assert 'MEAN_ALIGNMENT_UNAVAILABLE' not in text
assert 'terminate called' not in text
hashes, counts, wire, poses, inputs = {}, {}, {}, [], {}
with rosbag.Bag(str(out/'native.bag')) as bag:
    for topic,msg,stamp in bag.read_messages():
        if topic == '/kimera/cbs/odom_belief_out': continue
        buf=io.BytesIO(); msg.serialize(buf)
        hashes.setdefault(topic,hashlib.sha256()).update(buf.getvalue())
        counts[topic]=counts.get(topic,0)+1
        if topic == '/kimera/cbs/odom_belief_in':
            for b in msg.beliefs:
                key=(b.from_pose_index,b.to_pose_index)
                assert key not in wire
                wire[key] = dict(mu=list(b.relative_mu),cov=list(b.covariance),
                    ti=b.from_stamp_sec,tj=b.to_stamp_sec,arrival_ns=stamp.to_nsec())
        elif topic == '/kimera_vio_ros/odometry':
            p=msg.pose.pose.position; q=msg.pose.pose.orientation
            poses.append([msg.header.stamp.to_nsec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w,int(msg.header.seq),stamp.to_nsec()])
        elif topic == '/boxi/cpt7/imu':
            inputs.setdefault('imu_stamps',[]).append(msg.header.stamp.to_nsec())
assert len(poses)>=20 and poses[-1][0]-poses[0][0] >= 4.5e9
# The playback window starts at the first camera record, ~146 ms before IMU.
# Verify exact prefixes against source bags rather than guessing 800 samples.
expected_prefixes={}
ws=root.parents[1]
data=ws/'src/datasets/GrandTour'
for suffix,topics in [('cpt7_imu',['/boxi/cpt7/imu']),('alphasense',[
        '/boxi/alphasense/front_left/image_raw/compressed',
        '/boxi/alphasense/front_right/image_raw/compressed'])]:
    expected={topic:hashlib.sha256() for topic in topics}; seen={topic:0 for topic in topics}
    with rosbag.Bag(str(data/('2024-10-01-11-29-55_'+suffix+'.bag'))) as bag:
        for topic,msg,stamp in bag.read_messages(topics=topics):
            if seen[topic]<counts[topic]:
                buf=io.BytesIO();msg.serialize(buf);expected[topic].update(buf.getvalue());seen[topic]+=1
            if all(seen[t]==counts[t] for t in topics):break
    for topic in topics:
        assert expected[topic].hexdigest()==hashes[topic].hexdigest(), ('raw input prefix mismatch',topic)
        expected_prefixes[topic]=True
assert inputs['imu_stamps'][-1]-inputs['imu_stamps'][0] >= 7.8e9
rows=[]; equal=0; matched=[]
def token(s): return np.array([float(x) for x in re.split('[:;]',s)])
for line in text.splitlines():
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
        v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,')[1].split(',')
        u,si,sj,ri,rj,a,b,c,d,now=map(int,v[:10]); raw=token(v[10]); mean=token(v[11]); cov=token(v[12])
        assert max(a,b,c,d)<=now
        assert np.array_equal(cov,wire[(si,sj)]['cov'])
        assert np.max(np.abs(raw-np.array(wire[(si,sj)]['mu'])))<1e-12
        rows.append(dict(update=u,si=si,sj=sj,ri=ri,rj=rj,a=a,b=b,c=c,d=d,now=now,raw=raw.tolist(),mean=mean.tolist()))
    if 'CBS_BPSAM_SOURCE_TRANSPORT_ROW,G2K,' in line:
        v=line.split('CBS_BPSAM_SOURCE_TRANSPORT_ROW,')[1].split(',')
        si,sj=int(v[1][2:]),int(v[2][2:])
        assert np.array_equal(token(v[5]),wire[(si,sj)]['cov'])
        assert np.array_equal(token(v[5]),token(v[6]))
        equal+=1
    if 'CBS_ODOM_MATCH_ROW_G2K,' in line and 'sender_odom_queued' in line:
        v=line.split('CBS_ODOM_MATCH_ROW_G2K,')[1].split(',')
        matched.append([v[0],v[3]])
if case=='aligned': assert len(rows)==20 and equal==20
elif case=='raw': assert not rows and equal==20
else: assert not rows and equal==0
result=dict(case=case,passed=True,requested_bag_seconds=8,received_counts=counts,
            raw_sensor_prefixes_exact=expected_prefixes,
            input_hashes={k:v.hexdigest() for k,v in hashes.items() if k!='/kimera_vio_ros/odometry'},
            all_36_covariance_entries_exact_at_factor_construction=True,factor_covariances_checked=equal,
            no_missing_native_bridge=True,alignment_rows=rows,matched_edges=matched,
            native_pose_count=len(poses),native_duration_s=(poses[-1][0]-poses[0][0])*1e-9,
            native_poses=poses,imu_start_ns=inputs['imu_stamps'][0],imu_end_ns=inputs['imu_stamps'][-1])
with (out/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['native_poses','alignment_rows','matched_edges','input_hashes']},indent=2))
