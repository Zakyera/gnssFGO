"""Summarize saved solve phases; no estimator, covariance or trajectory scoring."""
import csv,hashlib,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name;Q=W/'runs/eig2_refresh_full_cpv42_20261029/prepared'
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def vals(p):
 return {r[0]:np.array(r[1:],float) for r in csv.reader(p.open())}
def rot(v):return Rotation.from_quat(v[[4,5,6,3]]).as_matrix()
def movement(a,b):
 keys=sorted((k for k in a if k in b and k.startswith('x')),key=lambda x:int(x[1:]));assert keys
 pa=np.array([a[k][:3] for k in keys]);pb=np.array([b[k][:3] for k in keys])
 matrices=np.array([rot(b[k])@rot(a[k]).T for k in keys]);R=Rotation.from_matrix(matrices).mean().as_matrix()
 t=pb.mean(axis=0)-R@pa.mean(axis=0);res=pb-(pa@R.T+t)
 da=np.array([np.linalg.norm(b[k][0:3]-a[k][0:3]) for k in keys])
 dr=Rotation.from_matrix(matrices).magnitude()*180/np.pi
 vs=[np.linalg.norm(b[k]-a[k]) for k in a if k.startswith('v') and k in b]
 bas=[np.linalg.norm(b[k][:3]-a[k][:3]) for k in a if k.startswith('b') and k in b]
 bgs=[np.linalg.norm(b[k][3:]-a[k][3:]) for k in a if k.startswith('b') and k in b]
 return dict(poses=len(keys),max_position_m=float(max(da)),max_rotation_deg=float(max(dr)),
  previous_latest_key=keys[-1],previous_latest_position_m=float(da[-1]),previous_latest_rotation_deg=float(dr[-1]),
  common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
  common_world_yaw_deg=float(np.degrees(np.arctan2(R[1,0],R[0,0]))),
  common_shape_max_residual_m=float(max(np.linalg.norm(res,axis=1))),
  max_velocity_change_m_s=float(max(vs)),max_accel_bias_change_m_s2=float(max(bas)),max_gyro_bias_change_rad_s=float(max(bgs)))
def cost(p):
 rows=list(csv.DictReader(p.open()));families={};imu=[];smart=[]
 for row in rows:
  families[row['type']]=families.get(row['type'],0)+float(row['cost'])
  if row['type']=='imu':imu.append(row)
  if row['type']=='smart':smart.append(row)
 return dict(total=sum(families.values()),families=families,factors=len(rows),smart=len(smart),
  valid_smart=sum(int(r['valid']) for r in smart),
  max_imu_position_m=max(float(r['imu_position_m']) for r in imu),
  max_imu_rotation_deg=max(float(r['imu_rotation_deg']) for r in imu),
  max_imu_velocity_m_s=max(float(r['imu_velocity_m_s']) for r in imu))
result={'covariance_analysis':False,'accuracy_scored':False,'updates':[]}
for k in range(21,26):
 d=P/'inspection/results'/f'update_{k}'
 states={s:vals(d/(s+'_values.csv')) for s in ('before_ordinary','after_ordinary','after_refresh')}
 init=vals(d/'initialized_values.csv');ordinary=states['after_ordinary'];final=states['after_refresh']
 c={s:cost(d/('same_graph_'+s+'.csv')) for s in ('initialized','ordinary','refresh')}
 e=dict(csv.reader((d/'equations.txt').open()))
 r=dict(k=k,costs=c,equations={key:float(v) for key,v in e.items()},
  ordinary_revision=movement(states['before_ordinary'],ordinary),
  refresh_revision=movement(ordinary,final),net_revision=movement(states['before_ordinary'],final))
 result['updates'].append(r)
# Compare actual native first publications, not aligned/resampled trajectories.
def odom(p):
 rows=list(csv.DictReader(p.open()));out={}
 for r in rows:
  i=int(r['field.header.seq']);t=int(r['field.header.stamp']);v=np.array([float(r['field.pose.pose.position.'+c]) for c in 'xyz']+[float(r['field.pose.pose.orientation.'+c]) for c in 'wxyz'])
  out[i]=(t,v)
 return out
old=odom(Q/'full_on_source/kimera_odometry.csv');new=odom(P/'prepared/full_on_source/kimera_odometry.csv')
assert all(i in old for i in new)
maxpos=max(float(np.linalg.norm(v[:3]-old[i][1][:3])) for i,(t,v) in new.items())
maxrot=max(float(Rotation.from_matrix(rot(v)@rot(old[i][1]).T).magnitude()*180/np.pi) for i,(t,v) in new.items())
result['prefix_native_parity']=dict(outputs=len(new),stamps_equal=all(t==old[i][0] for i,(t,v) in new.items()),max_position_m=maxpos,max_rotation_deg=maxrot,
 all_pose_components_exact=all(np.array_equal(v,old[i][1]) for i,(t,v) in new.items()),
 first_changed_pose_id=next((i for i,(t,v) in new.items() if not np.array_equal(v,old[i][1])),None),
 first_change_over_1mm_id=next((i for i,(t,v) in new.items() if np.linalg.norm(v[:3]-old[i][1][:3])>.001),None),
 first_change_over_point01deg_id=next((i for i,(t,v) in new.items() if Rotation.from_matrix(rot(v)@rot(old[i][1]).T).magnitude()*180/np.pi>.01),None))
latencies=[];snapshots={}
for p in (P/'checkpoints').glob('update_*/*/checkpoint.bin'):
 other=P/'inspection/results'/p.parent.parent.name/(p.parent.name+'_roundtrip.bin')
 assert sha(p)==sha(other),(p,'byte roundtrip')
 snapshots[str(p)]=sha(p)
 fields=(p.parent/'complete.txt').read_text().splitlines();latencies.append(float(fields[1].split(',')[1]))
assert len(snapshots)==15
result['observer_seconds']=dict(count=15,total=sum(latencies),maximum=max(latencies),median=float(np.median(latencies)))
result['checkpoint_sha256']=snapshots
result['analysis_input_sha256']={str(p):sha(p) for p in (P/'inspection/results').rglob('*') if p.is_file()}
with (H/'analysis.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
for r in result['updates']:
 print(r['k'],'IMU',*[r['costs'][s]['families']['imu'] for s in ('initialized','ordinary','refresh')],
  'tilt ordinary/refresh/net',*[r[s]['common_world_tilt_deg'] for s in ('ordinary_revision','refresh_revision','net_revision')])
print(json.dumps({k:v for k,v in result.items() if k not in ('updates','analysis_input_sha256','checkpoint_sha256')},indent=2))
