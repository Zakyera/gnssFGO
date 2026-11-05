"""CP-V17 explicitly authorized as-recorded gap-aware input admission."""
from pathlib import Path
import collections, hashlib, io, json, math, re, struct, sys
import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.highlevel import AnyReader

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared/input_profiles'
TOPICS={'anymal_velodyne':['/anymal/velodyne/points'], 'cpt7_imu':['/boxi/cpt7/imu'],
 'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed'],
 'cpt7_ie_tc':['/boxi/inertial_explorer/tc/odometry']}

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def mat(v):
 a=np.eye(4);a[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();a[:3,3]=v[:3];return a

def yload(p):return yaml.safe_load('\n'.join(p.read_text().splitlines()[1:]))

def main(slug):
 cfg=next(c for c in json.loads((H/'campaign.json').read_text())['sequences'] if c['slug']==slug)
 out=H/slug;out.mkdir(exist_ok=True)
 assert not (out/'bag_preflight.json').exists(), 'Never overwrite completed preflight'
 D=W/'src/datasets/GrandTour'/cfg['dataset'];mission=cfg['mission'];bags={};stamps={}
 for kind,topics in TOPICS.items():
  p=D/f'{mission}_{kind}.bag';before=p.stat()
  with Reader(p) as r:
   cs=[c for c in r.connections if c.topic in topics]
   assert sorted(c.topic for c in cs)==sorted(topics),(kind,'missing topic')
   for c in cs:
    a=[x.time for x in r.indexes[c.id]]
    assert a and np.all(np.diff(a)>0),(kind,'time regression')
    stamps[c.topic]=a
  digest=sha(p);after=p.stat()
  assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
  bags[kind]=dict(path=str(p),bytes=before.st_size,mtime_ns=before.st_mtime_ns,sha256=digest)
  print(slug+': verified '+kind,flush=True)
 if cfg.get('hesai_report'):
  old=json.loads((H.parent/cfg['hesai_report']/'bag_preflight.json').read_text())
  for kind in ('cpt7_imu','alphasense','cpt7_ie_tc'):
   assert bags[kind]['sha256']==old['bag_files'][kind]['sha256'],('changed mission input',kind)
 epoch=min(stamps[t][0] for kind,ts in TOPICS.items() if kind!='cpt7_ie_tc' for t in ts)
 fixed=cfg.get('start_ns');lo=fixed if fixed is not None else epoch
 hi=lo+2_000_000_000 if fixed is not None else epoch+16_000_000_000

 # Verify every CameraInfo and static transform against the frozen profile.
 edges={};tf_messages=0;p=D/f'{mission}_tf_minimal.bag';before=p.stat()
 with AnyReader([p]) as r:
  for c,t,b in r.messages(connections=[c for c in r.connections if c.topic=='/tf_static']):
   tf_messages+=1
   for x in r.deserialize(b,c.msgtype).transforms:
    v=x.transform.translation;q=x.transform.rotation;k=(x.header.frame_id,x.child_frame_id)
    a=[v.x,v.y,v.z,q.x,q.y,q.z,q.w]
    if k in edges:assert np.allclose(mat(a),mat(edges[k]),atol=1e-12,rtol=0),k
    edges[k]=a
 bags['tf_minimal']=dict(path=str(p),bytes=before.st_size,mtime_ns=before.st_mtime_ns,sha256=sha(p))
 def tf(a,b):
  todo=collections.deque([(a,np.eye(4),[])]);seen=set()
  while todo:
   node,m,path=todo.popleft()
   if node==b:return m,path
   if node in seen:continue
   seen.add(node)
   for (x,y),v in edges.items():
    if x==node:todo.append((y,m@mat(v),path+[[x,y]]))
    elif y==node:todo.append((x,m@np.linalg.inv(mat(v)),path+[[y,x]]))
  raise AssertionError(('missing TF',a,b))
 cameras={};changes=[];prev=None
 with AnyReader([D/f'{mission}_alphasense.bag']) as r:
  topics=TOPICS['alphasense'];ci_topics=[t.replace('image_raw/compressed','camera_info') for t in topics]
  cs=[c for c in r.connections if c.topic in topics+ci_topics]
  for c,t,b in r.messages(connections=cs):
   if c.topic.endswith('camera_info'):
    m=r.deserialize(b,c.msgtype)
    v=dict(frame=m.header.frame_id,width=m.width,height=m.height,model=m.distortion_model,K=m.K.tolist(),D=m.D.tolist())
    if c.topic in cameras:assert cameras[c.topic]==v,'changing camera calibration'
    cameras[c.topic]=v
   elif c.topic==topics[0] and lo<=t<hi:
    m=r.deserialize(b,c.msgtype)
    im=np.asarray(Image.open(io.BytesIO(bytes(m.data))).convert('L').resize((320,240)),dtype=float)
    if prev is not None:changes.append([t,float(np.mean(abs(im-prev)))])
    prev=im
 calibration={}
 for side,name in [('left','LeftCameraParams.yaml'),('right','RightCameraParams.yaml')]:
  p=yload(S/'kimera_profile'/name);ci=cameras[f'/boxi/alphasense/front_{side}/camera_info'];K=ci['K']
  m,path=tf('cpt7_imu',ci['frame']);want=np.array(p['T_BS']['data']).reshape(4,4)
  assert p['resolution']==[ci['width'],ci['height']] and p['distortion_model']==ci['model']
  assert np.max(abs(np.array(p['intrinsics'])-np.array([K[0],K[4],K[2],K[5]])))<1e-10
  assert np.max(abs(np.array(p['distortion_coefficients'])-np.array(ci['D'])))<1e-12
  error=float(abs(m-want).max());assert error<1e-8,(side,error)
  calibration[side]=dict(camera_info=ci,tf_path=path,max_difference=error)
 extrinsic=[-.3864038,.0361,.121779,0,.999999999199,.000040026795,0]
 m,path=tf('velodyne_lidar','cpt7_imu');error=float(abs(m-mat(extrinsic)).max());assert error<1e-10
 calibration['velodyne']=dict(tf_path=path,max_difference=error)
 m,path=tf('cpt7_imu','prism')
 want=mat([.308837802,-.038312997,-.234944116,-.005724388,-.001387943,-.004742722,.999971405])
 error=float(abs(m-want).max());assert error<1e-8
 calibration['prism']=dict(tf_path=path,max_difference=error)
 assert np.max(abs(np.array(yload(S/'kimera_profile/ImuParams.yaml')['T_BS']['data']).reshape(4,4)-np.eye(4)))==0
 imu_start=[]
 with AnyReader([D/f'{mission}_cpt7_imu.bag']) as r:
  for c,t,b in r.messages(connections=[c for c in r.connections if c.topic==TOPICS['cpt7_imu'][0]],start=lo,stop=hi):
   m=r.deserialize(b,c.msgtype);assert m.header.frame_id=='cpt7_imu'
   w=m.angular_velocity;a=m.linear_acceleration;imu_start.append([t,w.x,w.y,w.z,a.x,a.y,a.z])
 xx=np.array(changes);ii=np.array(imu_start);quiet=[]
 candidates=[fixed] if fixed is not None else [epoch+n*100_000_000 for n in range(141)]
 start=None
 for candidate in sorted(set(candidates)):
  a=xx[(xx[:,0]>=candidate)&(xx[:,0]<candidate+2e9),1]
  b=ii[(ii[:,0]>=candidate)&(ii[:,0]<candidate+2e9)]
  if len(a)<15 or len(b)<190:continue
  info=dict(start_ns=candidate,image_diff_max=float(a.max()),gyro_norm_p95=float(np.percentile(np.linalg.norm(b[:,1:4],axis=1),95)),accel_std_xyz=np.std(b[:,4:],axis=0).tolist(),accel_mean_xyz=np.mean(b[:,4:],axis=0).tolist())
  info['passed']=info['image_diff_max']<1.5 and info['gyro_norm_p95']<.01 and max(info['accel_std_xyz'])<.1 and info['accel_mean_xyz'][2]<-9
  quiet.append(info)
  # Diagnostic candidate must have all leading sensor/reference records;
  # preserve the first failed admission rather than call this a passed run.
  leading=max(stamps[t][0] for a in TOPICS.values() for t in a)
  if info['passed'] and start is None and candidate>leading:start=candidate
 # Existing matched missions keep their already validated physical start.
 # A new two-second heuristic is not a retroactive replacement of that contract.
 if fixed is not None:start=fixed
 else:
  if start is None:
   evidence=dict(passed=False,stage='quiet_initialization',mission=mission,
     bag_files=bags,calibration=calibration,quiet_start_checks=quiet,
     reason='No two-second raw-sensor quiet interval in first15s; thresholds unchanged; no capture attempted.')
   with (out/'blocked.json').open('x') as f:json.dump(evidence,f,indent=2)
   raise AssertionError(('no raw-sensor quiet initialization interval',str(out/'blocked.json')))
 raw_last=stamps[TOPICS['anymal_velodyne'][0]][-1]
 if 'end_ns' in cfg:end=cfg['end_ns']
 else:
  latest=min(raw_last,stamps[TOPICS['cpt7_imu'][0]][-1]-10_000_000,*(stamps[t][-1]+100_000_000 for t in TOPICS['alphasense']))
  last=max(t for t in stamps[TOPICS['anymal_velodyne'][0]] if t<=latest)
  end=start+math.ceil((last-start+1)/10_000_000)*10_000_000
 duration=(end-start)/1e9;offset=(start-epoch)/1e9
 assert 700<duration<730,(slug,duration)  # ARC-3 indexed full duration, not CP-V4's shorter missions
 selected={t:[v for v in a if start<=v<end] for t,a in stamps.items()}
 headers={t:[] for t in stamps}
 for kind,topics in TOPICS.items():
  with Reader(D/f'{mission}_{kind}.bag') as r:
   cs=[c for c in r.connections if c.topic in topics]
   for c,t,b in r.messages(connections=cs,start=start,stop=end):
    _,sec,ns=struct.unpack_from('<III',b);h=sec*10**9+ns
    assert abs(t-h)<50_000_000,(kind,t,h)
    headers[c.topic].append(h)
 left,right=(headers[t] for t in TOPICS['alphasense'])
 # Diagnose known stereo gap without certifying/launching the dataset.
 unmatched_left=sorted(set(left)-set(right));unmatched_right=sorted(set(right)-set(left))
 assert unmatched_left==[] and unmatched_right==[1731934279738235734]
 assert max(np.diff(left))==199998000 and max(np.diff(right))<110_000_000
 stereo_diagnostic=dict(strict_gate_passed=False,missing_left_ns=unmatched_right,
   missing_right_ns=unmatched_left,left_max_gap_ns=int(max(np.diff(left))),
   right_max_gap_ns=int(max(np.diff(right))),not_admitted=False,admission='explicit as-recorded gaps; strict clean-input gate not passed')
 imu=headers[TOPICS['cpt7_imu'][0]]
 assert np.all(np.diff(imu)>0) and max(np.diff(imu))<30_000_000
 assert stamps[TOPICS['cpt7_ie_tc'][0]][0]<start and stamps[TOPICS['cpt7_ie_tc'][0]][-1]>=end
 clouds=[]
 with AnyReader([D/f'{mission}_anymal_velodyne.bag']) as r:
  cs=[c for c in r.connections if c.topic==TOPICS['anymal_velodyne'][0]]
  for c,t,b in r.messages(connections=cs,start=start,stop=end):
   m=r.deserialize(b,c.msgtype);h=m.header.stamp.sec*10**9+m.header.stamp.nanosec
   assert h==t and m.header.frame_id=='velodyne_lidar' and m.height==1
   want=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',18,7)]
   assert [(f.name,f.offset,f.datatype) for f in m.fields]==want and all(f.count==1 for f in m.fields)
   assert m.point_step==22 and not m.is_bigendian
   dtype=np.dtype(dict(names=[v[0] for v in want],formats=['<f4']*4+['<u2','<f4'],offsets=[v[1] for v in want],itemsize=22))
   a=np.frombuffer(m.data,dtype=dtype,count=m.width)
   assert len(a)>0 and all(np.isfinite(a[f]).all() for f in ('x','y','z','time'))
   mn=float(a['time'].min());mx=float(a['time'].max())
   # Entire scans may precede the header. Native TimeKeeper shifts by min(time),
   # so max(time) need not be positive; absolute point times stay unchanged.
   assert -.11<mn<0 and mn<mx<.01 and 0<mx-mn<.11,(t,mn,mx)
   # Compare the exact audited cloud below; no blanket partial-scan exemption.
   normalized=h/1e9+mn
   clouds.append(dict(record_ns=t,header_ns=h,normalized_stamp_s=normalized,min_offset_s=mn,max_offset_s=mx,count=len(a),captured_imu_bracketed=imu[0]/1e9<=normalized and imu[-1]/1e9>=h/1e9+mx))
 assert len(clouds)==len(selected[TOPICS['anymal_velodyne'][0]])
 assert np.all(np.diff([c['normalized_stamp_s'] for c in clouds])>0)
 assert all(c['captured_imu_bracketed'] for c in clouds[1:]), dict(start=start,end=end,imu_first=imu[0],imu_last=imu[-1],unbracketed=[(i,c) for i,c in enumerate(clouds) if not c['captured_imu_bracketed']])
 diagnostic=json.loads((H/'cloud_diagnosis.json').read_text())
 audited={c['header_ns']:c for c in diagnostic['clouds']}
 for c in clouds:
  d=audited[c['header_ns']]
  assert c['min_offset_s']==d['min_offset_s'] and c['max_offset_s']==d['max_offset_s'],c
 partial=[c['header_ns'] for c in clouds if c['max_offset_s']-c['min_offset_s']<.08]
 assert len(partial)==70,('unexpected partial-scan inventory',len(partial))
 raw_gaps=[dict(before_ns=a['header_ns'],after_ns=b['header_ns'],gap_s=(b['header_ns']-a['header_ns'])/1e9) for a,b in zip(clouds,clouds[1:]) if b['header_ns']-a['header_ns']>150_000_000]
 assert sum(g['gap_s']>1 for g in raw_gaps)==6
 result=dict(passed=True,strict_clean_input_gate_passed=False,
  admission_contract='User-authorized ARC-3 as-recorded gaps and exact partial-cloud inventory; no source changes',
  known_partial_cloud_headers_ns=partial,known_raw_gaps=raw_gaps,
  stereo_diagnostic=stereo_diagnostic,other_raw_checks_passed=True,
  window_note='Earliest fully supported quiet start; complete mutually supported tail; no interior crop',
  mission=mission,dataset=cfg['dataset'],bag_files=bags,
  window=dict(selected_topic_epoch_ns=epoch,start_ns=start,end_ns=end,bag_offset_s=offset,duration_s=duration,seed_clock_seconds=start//10**9-1,last_included_scan_ns=clouds[-1]['record_ns'],last_available_scan_ns=raw_last),
  selected={t:dict(count=len(a),first_ns=a[0],last_ns=a[-1],max_gap_ns=int(max(np.diff(a)))) for t,a in selected.items()},
  stereo_pairs=len(left),frontend_min=len(left)-3,frontend_max=len(left),glim_min=len(clouds)-20,
  stereo_headers_identical=False,reference_covers_complete_window=True,
  calibration_matches_frozen_profiles=True,calibration=calibration,
  imu=dict(first_header_ns=imu[0],last_header_ns=imu[-1],count=len(imu),max_gap_ns=int(max(np.diff(imu)))),
  clouds=clouds,T_lidar_imu=extrinsic,tf_messages_checked=tf_messages,quiet_start_checks=quiet,
  first_partial_scan_note='Native warmup retained; every published pose must be IMU bracketed.',
  input_only_selection=True,hesai_window_matched=bool(cfg.get('hesai_report')))
 with (out/'bag_preflight.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
 print(json.dumps({k:result[k] for k in ('passed','dataset','window','stereo_pairs','glim_min')},indent=2),flush=True)

if __name__=='__main__':main(sys.argv[1])



