"""Header-only topic receipt audit; no image decoding or estimator feedback."""
import csv,json,os,struct,sys,time,threading
from pathlib import Path
import rospy
P=Path(sys.argv[1]);rospy.init_node('cpk21_receipt_observer')
topics=['/boxi/alphasense/front_left/image_raw/compressed',
 '/boxi/alphasense/front_right/image_raw/compressed',
 '/grandtour/front_left/image_raw','/grandtour/front_right/image_raw',
 '/boxi/cpt7/imu','/kimera_vio_ros/odometry']
lock=threading.Lock();state={}
f=(P/'topic_headers.csv').open('x',newline='');writer=csv.writer(f)
writer.writerow(['topic','sequence','stamp_ns','receipt_ros_ns','receipt_wall_ns'])
def callback(msg,topic):
 seq,sec,nsec=struct.unpack_from('<III',msg._buff)
 stamp=sec*1000000000+nsec
 with lock:
  writer.writerow([topic,seq,stamp,rospy.Time.now().to_nsec(),time.time_ns()])
  s=state.setdefault(topic,dict(count=0,first_ns=stamp,last_ns=stamp,max_gap_ns=0,sequence=seq))
  if s['count']:s['max_gap_ns']=max(s['max_gap_ns'],stamp-s['last_ns'])
  s.update(count=s['count']+1,last_ns=stamp,sequence=seq)
subs=[rospy.Subscriber(t,rospy.AnyMsg,callback,callback_args=t,queue_size=2000,buff_size=16777216) for t in topics]
(P/'observer_ready.marker').write_text('header subscribers registered\n')
try:
 while not rospy.is_shutdown():
  with lock:
   f.flush();temp=P/'observer_status.tmp'
   temp.write_text(json.dumps(dict(wall_ns=time.time_ns(),topics=state)))
   os.replace(str(temp),str(P/'observer_status.json'))
  time.sleep(1)
finally:f.close()
