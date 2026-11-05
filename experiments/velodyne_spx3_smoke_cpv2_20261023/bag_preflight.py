"""Read-only selected-window cloud/header audit, complete static calibration and hashes."""
from pathlib import Path
import collections, hashlib, json, shutil, struct, sys
import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.highlevel import AnyReader

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D=W/'src/datasets/GrandTour/SPX-3'
MISSION='2024-11-02-17-43-10'
START=1730565899872888651; END=START+30_000_000_000
TOPICS={'anymal_velodyne':['/anymal/velodyne/points'],
 'cpt7_imu':['/boxi/cpt7/imu'],
 'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed'],
 'cpt7_ie_tc':['/boxi/inertial_explorer/tc/odometry']}
assert not (H/'bag_preflight.json').exists()
old=json.loads((H.parent/'spx3_stationary_full_cpk28_20261023/bag_preflight.json').read_text())
bags={};stamps={};headers={}
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for kind,topics in TOPICS.items():
 p=D/f'{MISSION}_{kind}.bag';st=p.stat()
 with Reader(p) as r:
  cs=[c for c in r.connections if c.topic in topics]
  assert sorted(c.topic for c in cs)==sorted(topics)
  for c in cs:
   t=[x.time for x in r.indexes[c.id]];assert t and np.all(np.diff(t)>0)
   stamps[c.topic]=t;headers[c.topic]=[]
  for c,t,raw in r.messages(connections=cs,start=START,stop=END):
   _,sec,ns=struct.unpack_from('<III',raw);h=sec*10**9+ns
   assert abs(t-h)<50_000_000,(kind,t,h)
   headers[c.topic].append(h)
 digest=sha(p)
 if kind!='anymal_velodyne':assert digest==old['bag_files'][kind]['sha256'],kind
 assert st.st_size==p.stat().st_size and st.st_mtime_ns==p.stat().st_mtime_ns
 bags[kind]=dict(path=str(p),bytes=st.st_size,mtime_ns=st.st_mtime_ns,sha256=digest)
 print('Verified '+kind,flush=True)
epoch=min(stamps[t][0] for k,ts in TOPICS.items() if k!='cpt7_ie_tc' for t in ts)
offset=(START-epoch)/1e9
assert offset==29.0,(epoch,offset,'must update prepared exact physical offset before any launch')
selected={t:[x for x in ts if START<=x<END] for t,ts in stamps.items()}
left,right=(headers[t] for t in TOPICS['alphasense'])
assert left==right and len(left)==300
imu=headers[TOPICS['cpt7_imu'][0]];assert np.all(np.diff(imu)>0) and max(np.diff(imu))<30_000_000
assert stamps[TOPICS['cpt7_ie_tc'][0]][0]<START and stamps[TOPICS['cpt7_ie_tc'][0]][-1]>=END
clouds=[]
with AnyReader([D/f'{MISSION}_anymal_velodyne.bag']) as r:
 cs=[c for c in r.connections if c.topic==TOPICS['anymal_velodyne'][0]]
 for c,t,raw in r.messages(connections=cs,start=START,stop=END):
  m=r.deserialize(raw,c.msgtype);h=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  assert h==t and m.header.frame_id=='velodyne_lidar' and m.height==1
  want=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',18,7)]
  assert [(f.name,f.offset,f.datatype) for f in m.fields]==want and all(f.count==1 for f in m.fields)
  assert m.point_step==22 and not m.is_bigendian
  dtype=np.dtype(dict(names=[x[0] for x in want],formats=['<f4']*4+['<u2','<f4'],offsets=[x[1] for x in want],itemsize=22))
  a=np.frombuffer(m.data,dtype=dtype,count=m.width)
  assert all(np.isfinite(a[f]).all() for f in ('x','y','z','time'))
  mn=float(a['time'].min());mx=float(a['time'].max())
  assert -.11<mn<0 and 0<=mx<.01 and .08<mx-mn<.11
  normalized=h/1e9+mn
  clouds.append(dict(record_ns=t,header_ns=h,normalized_stamp_s=normalized,
      min_offset_s=mn,max_offset_s=mx,count=len(a),
      captured_imu_bracketed=imu[0]/1e9<=normalized and imu[-1]/1e9>=h/1e9+mx))
assert len(clouds)==300 and np.all(np.diff([c['normalized_stamp_s'] for c in clouds])>0)
assert all(c['captured_imu_bracketed'] for c in clouds[1:])

def mat(v):
 m=np.eye(4);m[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();m[:3,3]=v[:3];return m
edges={};tf_count=0
p=D/f'{MISSION}_tf_minimal.bag'
with AnyReader([p]) as r:
 cs=[c for c in r.connections if c.topic=='/tf_static']
 for c,t,raw in r.messages(connections=cs):
  tf_count+=1
  for x in r.deserialize(raw,c.msgtype).transforms:
   v=x.transform.translation;q=x.transform.rotation;k=(x.header.frame_id,x.child_frame_id)
   v=[v.x,v.y,v.z,q.x,q.y,q.z,q.w]
   if k in edges:assert np.allclose(mat(v),mat(edges[k]),atol=1e-12,rtol=0),k
   edges[k]=v
q=collections.deque([('velodyne_lidar',np.eye(4),[])]);seen=set();found=None
while q:
 node,m,path=q.popleft()
 if node=='cpt7_imu':found=(m,path);break
 if node in seen:continue
 seen.add(node)
 for (a,b),v in edges.items():
  if a==node:q.append((b,m@mat(v),path+[[a,b]]))
  elif b==node:q.append((a,m@np.linalg.inv(mat(v)),path+[[b,a]]))
assert found is not None
extrinsic=[-.3864038,.0361,.121779,0,.999999999199,.000040026795,0]
error=float(abs(found[0]-mat(extrinsic)).max());assert error<1e-10
bags['tf_minimal']=dict(path=str(p),bytes=p.stat().st_size,sha256=sha(p))
result=dict(passed=True,mission=MISSION,bag_files=bags,
 window=dict(selected_topic_epoch_ns=epoch,start_ns=START,end_ns=END,bag_offset_s=offset,duration_s=30.0),
 selected={t:dict(count=len(ts),first_ns=ts[0],last_ns=ts[-1],max_gap_ns=int(max(np.diff(ts)))) for t,ts in selected.items()},
 stereo_pairs=len(left),stereo_headers_identical=True,reference_covers_complete_window=True,
 imu=dict(first_header_ns=imu[0],last_header_ns=imu[-1],count=len(imu),max_gap_ns=int(max(np.diff(imu)))),
 clouds=clouds,T_lidar_imu=extrinsic,tf_chain=found[1],tf_messages_checked=tf_count,tf_error=error,
 first_partial_scan_note='First raw scan precedes captured IMU; native warmup retained, require all published outputs bracketed.',
 disk_free_bytes=shutil.disk_usage(W).free)
with (H/'bag_preflight.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k!='clouds'},indent=2))
