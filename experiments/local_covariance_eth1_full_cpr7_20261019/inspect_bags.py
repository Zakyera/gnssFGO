"""Read-only ROS1 bag/topic/reference preflight; no estimator launch."""
import json
from pathlib import Path
import rosbag

root=Path('/workspace/cbs_gtsam4.3/src/datasets/GrandTour')
mission='2024-10-01-11-29-55'
required={'hesai':['/boxi/hesai/points'], 'cpt7_imu':['/boxi/cpt7/imu'],
 'alphasense':['/boxi/alphasense/front_left/image_raw/compressed',
              '/boxi/alphasense/front_right/image_raw/compressed'],
 'cpt7_ie_tc':['/boxi/inertial_explorer/tc/odometry']}
out={}
for kind,topics in required.items():
 p=root/(mission+'_'+kind+'.bag')
 with rosbag.Bag(str(p)) as b:
  info=b.get_type_and_topic_info().topics
  assert all(t in info for t in topics),(kind,topics)
  out[kind]=dict(path=str(p),record_start=b.get_start_time(),record_end=b.get_end_time(),
    topics={t:dict(type=info[t].msg_type,count=info[t].message_count) for t in topics})
  first={};last={};count={t:0 for t in topics};maxgap={t:0 for t in topics}
  # Small IMU/GT bags permit a complete stamp check. Image/cloud payloads
  # are not decoded for this metadata-only preflight.
  if kind in ('cpt7_imu','cpt7_ie_tc'):
   for topic,msg,record in b.read_messages(topics=topics):
    stamp=msg.header.stamp.to_sec()
    if topic in last:
     assert stamp>last[topic],(kind,'non-increasing stamps')
     maxgap[topic]=max(maxgap[topic],stamp-last[topic])
    first.setdefault(topic,stamp);last[topic]=stamp;count[topic]+=1
   out[kind]['stamp_coverage']=dict(first=first,last=last,count=count,maxgap=maxgap)
epoch=min(out[k]['record_start'] for k in ('hesai','cpt7_imu','alphasense'))
start=epoch;end=start+346.22
for k in ('hesai','cpt7_imu','alphasense'):
 assert out[k]['record_start']<=start+1 and out[k]['record_end']+0.001>=end,k
gt=out['cpt7_ie_tc']['stamp_coverage'];topic=required['cpt7_ie_tc'][0]
assert gt['first'][topic]<=out['cpt7_imu']['stamp_coverage']['first']['/boxi/cpt7/imu'] and gt['last'][topic]>=end
out.update(passed=True,window=dict(record_epoch=epoch,start=start,end=end,
 seed_clock_seconds=1727775007,bag_offset=0,duration=346.22))
assert 0 <= end-out['hesai']['record_end'] < 0.001
out['window']['end_rule']='through last Hesai record, rounded up to1ms; exclude camera/IMU-only tails'
print(json.dumps(out,indent=2))
