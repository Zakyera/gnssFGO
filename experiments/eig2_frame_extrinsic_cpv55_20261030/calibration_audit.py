"""Read-only independent mission TF / saved profile comparison."""
import hashlib,json,re,sys
from pathlib import Path
from collections import defaultdict,deque
import numpy as np
from scipy.spatial.transform import Rotation as R
from rosbags.highlevel import AnyReader
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
pins={}
def read(p):
 p=Path(p);b=p.read_bytes();pins[str(p)]=hashlib.sha256(b).hexdigest();return b.decode()
def jload(p):return json.loads(re.sub(r'//[^\n]*|/\*[\s\S]*?\*/','',read(p)))
def pose(t,q):
 T=np.eye(4);T[:3,:3]=R.from_quat(q).as_matrix();T[:3,3]=t;return T
def inv(T):
 U=np.eye(4);U[:3,:3]=T[:3,:3].T;U[:3,3]=-U[:3,:3]@T[:3,3];return U
def err(A,B):return dict(rotation_deg=float(np.degrees(R.from_matrix(A[:3,:3].T@B[:3,:3]).magnitude())),translation_m=float(np.linalg.norm(A[:3,3]-B[:3,3])),max_matrix=float(np.max(abs(A-B))))
def yamlpose(p):return np.array(json.loads(re.search(r'data:\s*(\[[\s\S]*?\])',read(p))[1])).reshape(4,4)
bag=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_tf_minimal.bag'
with bag.open('rb') as f:pins[str(bag)]=hashlib.file_digest(f,'sha256').hexdigest()
edges={};counts=defaultdict(int);maxchange=0.
with AnyReader([bag]) as r:
 for c,t,b in r.messages(connections=[c for c in r.connections if c.topic=='/tf_static']):
  for z in r.deserialize(b,c.msgtype).transforms:
   v=z.transform.translation;q=z.transform.rotation;k=(z.header.frame_id,z.child_frame_id)
   T=pose([v.x,v.y,v.z],[q.x,q.y,q.z,q.w]);counts[k]+=1
   if k in edges:maxchange=max(maxchange,float(np.max(abs(edges[k]-T))))
   edges[k]=T
assert maxchange<1e-12
adj=defaultdict(list)
for (a,b),T in edges.items():adj[a].append((b,T));adj[b].append((a,inv(T)))
def lookup(a,b):
 Q=deque([(a,np.eye(4),[a])]);seen={a}
 while Q:
  u,T,path=Q.popleft()
  if u==b:return T,path
  for v,E in adj[u]:
   if v not in seen:seen.add(v);Q.append((v,T@E,path+[v]))
 raise RuntimeError(('missing static TF route',a,b,sorted(adj)))
frames=['alphasense_front_left','alphasense_front_right','hesai_lidar','velodyne','cpt7_imu']
# Obtain actual Velodyne frame from saved config, not a guessed sensor name.
Pvel=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared'
vframe=jload(Pvel/'input_profiles/glim_config/config_ros.json')['glim_ros']['lidar_frame_id']
frames[3]=vframe
transforms={};checks=[]
for f in frames:
 T,path=lookup('cpt7_imu',f);transforms[f]=dict(T_body_sensor=T.tolist(),path=path,lever_arm_m=float(np.linalg.norm(T[:3,3])))
preps={n:W/'runs'/p/'prepared' for n,p in [('Hesai_CP49','eig2_hesai_refresh_full_cpv49_20261030'),('Hesai_CP54','eig2_glim_first_edge_cpv54_20261030'),('Velodyne_CPV4','gnssfgo_eig2_velodyne_full_cpv4_20261024')]}
for name,P in preps.items():
 prof=P/'input_profiles';g=jload(prof/'glim_config/config_sensors.json')['sensors'];ros=jload(prof/'glim_config/config_ros.json')['glim_ros'];frame=ros['lidar_frame_id']
 x=g['T_lidar_imu'];T=pose(x[:3],x[3:]);expected,_=lookup(frame,'cpt7_imu')
 checks.append(dict(case=name,item='T_lidar_imu',error=err(expected,T)))
 for side in ['Left','Right']:
  T=yamlpose(prof/f'kimera_profile/{side}CameraParams.yaml');expected,_=lookup('cpt7_imu','alphasense_front_'+side.lower())
  checks.append(dict(case=name,item=side+'Camera_T_BS',error=err(expected,T)))
 checks.append(dict(case=name,item='IMU_T_BS',error=err(np.eye(4),yamlpose(prof/'kimera_profile/ImuParams.yaml'))))
 assert ros['imu_frame_id']==ros['base_frame_id']=='cpt7_imu' and ros['imu_topic']=='/boxi/cpt7/imu'
 for mode in ['off','on']:
  S=P/f'full_{mode}_source';kp=read(S/'resolved_kimera_params.yaml');gp=read(S/'resolved_glim_params.yaml')
  for text,tokens in [(kp,['base_link_frame_id: cpt7_imu','cbs_external_pose_frame_id: cpt7_imu']),(gp,['glim_cbs_child_frame_id: cpt7_imu'])]:
   assert all(t in text for t in tokens),(name,mode)
  for rel in ['kimera_profile/LeftCameraParams.yaml','kimera_profile/RightCameraParams.yaml','kimera_profile/ImuParams.yaml','glim_config/config_sensors.json','glim_config/config_ros.json']:
   assert read(S/rel)==read(prof/rel),(name,mode,rel)
for c in checks:assert c['error']['max_matrix']<1e-9,c
L=np.array(transforms['alphasense_front_left']['T_body_sensor']);Rt=np.array(transforms['alphasense_front_right']['T_body_sensor']);LR=inv(L)@Rt
cross={f:(inv(L)@np.array(transforms[f]['T_body_sensor'])).tolist() for f in ['hesai_lidar',vframe]}
out=dict(passed=True,static_edges=len(edges),static_messages_transform_observations=sum(counts.values()),static_max_change=maxchange,transforms=transforms,checks=checks,stereo_left_T_right=LR.tolist(),stereo_baseline_m=float(np.linalg.norm(LR[:3,3])),camera_T_lidar=cross,input_sha256=pins,limits=['Author static TF consistency, not independent physical calibration.','Saved input profiles and resolved runtime contract; no assertion every historical source file is binary-identical.'])
with (H/'calibration.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:out[k] for k in ['passed','static_edges','transforms','checks','stereo_baseline_m']},indent=2))
