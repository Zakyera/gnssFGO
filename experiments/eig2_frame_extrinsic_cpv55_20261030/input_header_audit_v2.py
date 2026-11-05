"""Inspect original sensor headers and camera calibration, without replay."""
from pathlib import Path
import json,hashlib
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');D=W/'src/datasets/GrandTour/EIG-2';P=W/'runs/eig2_glim_first_edge_cpv54_20261030/prepared/input_profiles'
out={};meta={}
for suffix,topics in [('cpt7_imu',['/boxi/cpt7/imu']),('hesai',['/boxi/hesai/points']),('anymal_velodyne',['/anymal/velodyne/points']),('alphasense',['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed','/boxi/alphasense/front_left/camera_info','/boxi/alphasense/front_right/camera_info'])]:
 p=D/f'2024-11-03-13-59-54_{suffix}.bag';st=p.stat();meta[str(p)]=dict(size=st.st_size,mtime_ns=st.st_mtime_ns,scope='metadata, not new full-bag checksum')
 with AnyReader([p]) as r:
  for topic in topics:
   cs=[c for c in r.connections if c.topic==topic];assert len(cs)==1
   c,t,b=next(r.messages(connections=cs));m=r.deserialize(b,c.msgtype)
   data=dict(header_frame=m.header.frame_id,connection_type=c.msgtype,first_record_ns=t)
   if topic.endswith('camera_info'):
    side='Left' if 'front_left' in topic else 'Right';q=P/f'kimera_profile/{side}CameraParams.yaml';raw=q.read_text();y=yaml.safe_load(raw.replace('%YAML:1.0',''))
    K=np.array(m.K).reshape(3,3);got=np.array([K[0,0],K[1,1],K[0,2],K[1,2]])
    data.update(intrinsics_max_error=float(np.max(abs(got-y['intrinsics']))),distortion_max_error=float(np.max(abs(np.array(m.D)-y['distortion_coefficients']))),distortion_model=m.distortion_model,resolution=[m.width,m.height],profile_sha256=hashlib.sha256(q.read_bytes()).hexdigest())
    assert data['intrinsics_max_error']<1e-8 and data['distortion_max_error']<1e-8 and [m.width,m.height]==y['resolution']
   out[topic]=data
expected={'/boxi/cpt7/imu':'cpt7_imu','/boxi/hesai/points':'hesai_lidar','/anymal/velodyne/points':'velodyne_lidar'}
for t,v in out.items():assert v['header_frame']==expected.get(t,'alphasense_front_left' if 'front_left' in t else 'alphasense_front_right')
with (H/'input_headers.json').open('x') as f:json.dump(dict(passed=True,topics=out,raw_metadata=meta,scope='First original message per topic; not a complete stream/calibration stability audit.'),f,indent=2)
print(json.dumps(out,indent=2))
