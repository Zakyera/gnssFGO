"""Read-only mission audit; write new evidence, never modify bags or profiles."""
from pathlib import Path
import hashlib, io, json, math, re, struct, sys
sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.highlevel import AnyReader
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D=W/'src/datasets/GrandTour/LEICA-2'
S=W/'runs/gnssfgo_spx3_stationary_full_cpk28_20261023/prepared/input_profiles'
MISSION='2024-11-25-16-36-19'
TOPICS={'hesai':['/boxi/hesai/points'],'cpt7_imu':['/boxi/cpt7/imu'],
 'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed'],
 'cpt7_ie_tc':['/boxi/inertial_explorer/tc/odometry'],'tf_minimal':['/tf_static'],
 'ap20_prism_position':['/boxi/ap20/prism_position']}
assert not (H/'bag_preflight.json').exists()
bags={};stamps={}
for kind,topics in TOPICS.items():
 p=D/f'{MISSION}_{kind}.bag';before=p.stat()
 with Reader(p) as r:
  cs=[c for c in r.connections if c.topic in topics]
  assert sorted(c.topic for c in cs)==sorted(topics)
  for c in cs:
   stamps[c.topic]=[x.time for x in r.indexes[c.id]]
   assert stamps[c.topic]
   if kind!='tf_minimal':assert all(b>a for a,b in zip(stamps[c.topic],stamps[c.topic][1:])),c.topic
  info=dict(path=str(p),bytes=before.st_size,mtime_ns=before.st_mtime_ns,
   topics={c.topic:dict(type=c.msgtype,count=c.msgcount,first_record_ns=stamps[c.topic][0],last_record_ns=stamps[c.topic][-1]) for c in cs})
 with p.open('rb') as f:info['sha256']=hashlib.file_digest(f,'sha256').hexdigest()
 after=p.stat();assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns),'Input still changing'
 bags[kind]=info;print('Verified '+kind,flush=True)
epoch=min(stamps[t][0] for k in ('hesai','cpt7_imu','alphasense') for t in TOPICS[k])
start=epoch+6_000_000_000 # Raw settling/quiet-start choice, not selected by estimator ATE.
last=stamps[TOPICS['hesai'][0]][-1]
duration_ns=math.ceil((last-start+1)/10_000_000)*10_000_000;end=start+duration_ns
assert 400_000_000_000<duration_ns<450_000_000_000
selected={}
for k in ('hesai','cpt7_imu','alphasense','cpt7_ie_tc','ap20_prism_position'):
 for t in TOPICS[k]:
  a=[n for n in stamps[t] if start<=n<end];assert len(a)>100
  selected[t]=a;bags[k]['topics'][t]['selected']=dict(count=len(a),first_record_ns=a[0],last_record_ns=a[-1],maximum_record_gap_ns=max(y-x for x,y in zip(a,a[1:])))
assert selected[TOPICS['hesai'][0]][-1]==last
assert selected[TOPICS['cpt7_imu'][0]][-1]>=last-20_000_000
assert max(np.diff(selected[TOPICS['cpt7_imu'][0]]))<30_000_000
assert stamps[TOPICS['cpt7_ie_tc'][0]][0]<start and stamps[TOPICS['cpt7_ie_tc'][0]][-1]>=end

# Mission CameraInfo and static TF must agree with every used calibration.
edges={};tf_rows=0
with AnyReader([D/f'{MISSION}_tf_minimal.bag']) as r:
 for c,t,b in r.messages(connections=[c for c in r.connections if c.topic=='/tf_static']):
  for x in r.deserialize(b,c.msgtype).transforms:
   p=x.transform.translation;q=x.transform.rotation
   v=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w]);key=(x.header.frame_id,x.child_frame_id)
   if key in edges:assert np.max(abs(edges[key]-v))<1e-12,key
   edges[key]=v;tf_rows+=1
def mat(v):
 T=np.eye(4);T[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();T[:3,3]=v[:3];return T
def tf(a,b):
 todo=[(a,np.eye(4),[])];seen=set()
 while todo:
  node,T,path=todo.pop(0)
  if node==b:return T,path
  if node in seen:continue
  seen.add(node)
  for (p,c),v in edges.items():
   if p==node:todo.append((c,T@mat(v),path+[(p,c)]))
   elif c==node:todo.append((p,T@np.linalg.inv(mat(v)),path+[(c,p)]))
 raise AssertionError((a,b))
def yload(p):return yaml.safe_load('\n'.join(p.read_text().splitlines()[1:]))
cameras={};headers={t:[] for t in TOPICS['alphasense']};changes=[];prev=None
with AnyReader([D/f'{MISSION}_alphasense.bag']) as r:
 cs=[c for c in r.connections if c.topic in TOPICS['alphasense'] or c.topic in [t.replace('image_raw/compressed','camera_info') for t in TOPICS['alphasense']]]
 for c,t,b in r.messages(connections=cs):
  if c.topic.endswith('camera_info'):
   m=r.deserialize(b,c.msgtype)
   obj=dict(frame=m.header.frame_id,width=m.width,height=m.height,model=m.distortion_model,K=m.K.tolist(),D=m.D.tolist())
   if c.topic in cameras:assert cameras[c.topic]==obj,'Camera calibration changes'
   cameras[c.topic]=obj
  else:
   _,sec,ns=struct.unpack_from('<III',b);stamp=sec*10**9+ns
   assert abs(t-stamp)<50_000_000
   if start<=t<end:headers[c.topic].append(stamp)
   if c.topic==TOPICS['alphasense'][0] and epoch<=t<epoch+10_000_000_000:
    m=r.deserialize(b,c.msgtype);im=np.asarray(Image.open(io.BytesIO(bytes(m.data))).convert('L').resize((320,240)),dtype=float)
    if prev is not None:changes.append([(t-epoch)*1e-9,float(np.mean(abs(im-prev)))])
    prev=im
left,right=(headers[t] for t in TOPICS['alphasense'])
assert left==right and len(left)==len(selected[TOPICS['alphasense'][0]])
assert max(np.diff(left))<110_000_000
calibration={}
for side,name in [('left','LeftCameraParams.yaml'),('right','RightCameraParams.yaml')]:
 cfg=yload(S/'kimera_profile'/name);ci=cameras[f'/boxi/alphasense/front_{side}/camera_info']
 T,path=tf('cpt7_imu',ci['frame']);K=ci['K'];want=np.array(cfg['T_BS']['data']).reshape(4,4)
 assert cfg['resolution']==[ci['width'],ci['height']] and cfg['distortion_model']==ci['model']
 assert np.max(abs(np.array(cfg['intrinsics'])-np.array([K[0],K[4],K[2],K[5]])))<1e-10
 assert np.max(abs(np.array(cfg['distortion_coefficients'])-np.array(ci['D'])))<1e-12
 err=float(np.max(abs(T-want)));assert err<1e-8,(side,err)
 calibration[side]=dict(camera_info=ci,T_cpt7_camera=T.tolist(),tf_path=path,max_extrinsic_difference=err)
sensor=(S/'glim_config/config_sensors.json').read_text()
sensor=json.loads(re.sub(r'//[^\n]*','',re.sub(r'/\*.*?\*/','',sensor,flags=re.S)))['sensors']
T,path=tf('hesai_lidar','cpt7_imu');err=float(np.max(abs(T-mat(np.array(sensor['T_lidar_imu'])))))
assert err<1e-8;calibration['hesai']=dict(T_hesai_cpt7=T.tolist(),tf_path=path,max_difference=err)
T,path=tf('cpt7_imu','prism');want=mat(np.array([.308837802,-.038312997,-.234944116,-.005724388,-.001387943,-.004742722,.999971405]))
err=float(np.max(abs(T-want)));assert err<1e-8
calibration['prism']=dict(T_cpt7_prism=T.tolist(),tf_path=path,max_difference=err)
assert np.max(abs(np.array(yload(S/'kimera_profile/ImuParams.yaml')['T_BS']['data']).reshape(4,4)-np.eye(4)))==0

imu=[];first_imu=None
with AnyReader([D/f'{MISSION}_cpt7_imu.bag']) as r:
 for c,t,b in r.messages(connections=[c for c in r.connections if c.topic==TOPICS['cpt7_imu'][0]],stop=epoch+10_000_000_000):
  m=r.deserialize(b,c.msgtype);assert m.header.frame_id=='cpt7_imu'
  w=m.angular_velocity;a=m.linear_acceleration;imu.append([(t-epoch)*1e-9,w.x,w.y,w.z,a.x,a.y,a.z])
  if first_imu is None:first_imu=dict(frame=m.header.frame_id,gyro_covariance=m.angular_velocity_covariance.tolist(),accel_covariance=m.linear_acceleration_covariance.tolist())
xx=np.array(changes);ii=np.array(imu);startup={}
for sec in range(10):
 a=xx[(xx[:,0]>=sec)&(xx[:,0]<sec+1),1];b=ii[(ii[:,0]>=sec)&(ii[:,0]<sec+1)]
 startup[str(sec)]=dict(image_abs_difference_median=float(np.median(a)),image_abs_difference_max=float(max(a)),gyro_norm_p95=float(np.percentile(np.linalg.norm(b[:,1:4],axis=1),95)),accel_std_xyz=np.std(b[:,4:],axis=0).tolist(),accel_mean_xyz=np.mean(b[:,4:],axis=0).tolist())
assert startup['6']['gyro_norm_p95']<.01 and startup['7']['gyro_norm_p95']<.01
assert startup['6']['image_abs_difference_max']<1.5 and startup['7']['image_abs_difference_max']<1.5
assert max(startup['6']['accel_std_xyz'])<.1 and max(startup['7']['accel_std_xyz'])<.1
assert startup['6']['accel_mean_xyz'][2]<-9

# Reference checks are not fed into initialization or used to select weights.
ref=[];children=set()
with AnyReader([D/f'{MISSION}_cpt7_ie_tc.bag']) as r:
 for c,t,b in r.messages(connections=[c for c in r.connections if c.topic==TOPICS['cpt7_ie_tc'][0]],start=start-1_000_000_000,stop=start+2_000_000_000):
  m=r.deserialize(b,c.msgtype);children.add(m.child_frame_id);p=m.pose.pose.position
  ref.append([t*1e-9,p.x,p.y,p.z])
assert children=={'cpt7_imu'}
rr=np.array(ref);t0=start*1e-9
ref_displacement=float(np.linalg.norm(rr[np.argmin(abs(rr[:,0]-(t0+1))),1:]-rr[np.argmin(abs(rr[:,0]-t0)),1:]))
assert ref_displacement<.02
result=dict(passed=True,mission=MISSION,bag_files=bags,window=dict(selected_topic_epoch_ns=epoch,start_ns=start,end_ns=end,last_hesai_record_ns=last,bag_offset_s=6.0,duration_s=round(duration_ns*1e-9,2),seed_clock_seconds=start//10**9-1),stereo_pairs=len(left),frontend_min=len(left)-3,frontend_max=len(left),stereo_headers_identical=True,reference_covers_complete_window=True,source_parameters_changed=False,calibration=calibration,calibration_matches_frozen_profiles=True,tf_rows=tf_rows,imu=first_imu,startup_raw=startup,startup_reference_displacement_1s_m=ref_displacement,reference_note='Full-window IE-TC shares CPT7. AP20 prism has partial coverage; do not substitute it for full scoring.',profile_template=str(S))
with (H/'bag_preflight.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps(dict(passed=True,window=result['window'],stereo_pairs=len(left),calibration_difference={k:v.get('max_difference',v.get('max_extrinsic_difference')) for k,v in calibration.items()},startup_reference_displacement_1s_m=ref_displacement),indent=2))
