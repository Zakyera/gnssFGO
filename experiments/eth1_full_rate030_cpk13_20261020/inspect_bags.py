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
  selected_topic,selected_message,selected_record=next(b.read_messages(topics=topics))
  out[kind]['first_selected_record_ns']=selected_record.to_nsec()
  out[kind]['first_selected_topic']=selected_topic
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
query_start_ns=min(out[k]['first_selected_record_ns'] for k in ('hesai','cpt7_imu','alphasense'))
epoch=query_start_ns*1e-9
import rospy
with rosbag.Bag(out['hesai']['path']) as b:
 last_selected_record_ns=None
 for topic,msg,record in b.read_messages(topics=required['hesai'],start_time=rospy.Time.from_sec(out['hesai']['record_end']-1)):
  last_selected_record_ns=record.to_nsec()
 assert last_selected_record_ns is not None
duration_ns=((last_selected_record_ns-query_start_ns+999999)//1000000)*1000000
assert duration_ns==346320000000,duration_ns
start=epoch;end=(query_start_ns+duration_ns)*1e-9
for k in ('hesai','cpt7_imu','alphasense'):
 assert out[k]['record_start']<=start+1 and out[k]['record_end']+0.001>=end,k
gt=out['cpt7_ie_tc']['stamp_coverage'];topic=required['cpt7_ie_tc'][0]
assert gt['first'][topic]<=out['cpt7_imu']['stamp_coverage']['first']['/boxi/cpt7/imu'] and gt['last'][topic]>=end
out.update(passed=True,window=dict(record_epoch=epoch,start=start,end=end,
 seed_clock_seconds=1727775007,bag_offset=0,duration=346.32,query_start_ns=query_start_ns,query_end_ns=query_start_ns+duration_ns,last_selected_hesai_record_ns=last_selected_record_ns,end_rule='last selected Hesai record rounded up to1ms from first selected input; no camera/IMU-only tails'))
print(json.dumps(out,indent=2))
