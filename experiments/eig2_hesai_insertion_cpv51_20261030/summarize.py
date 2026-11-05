"""Native saved-state diagnosis. No EVO/trajectory score or covariance analysis."""
import csv,hashlib,importlib.util,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name;P=K/'prepared';Q=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared'
EPOCH=json.loads((P/'input_audit.json').read_text())['window']['start_ns'];pins={}
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):pins[str(p)]=sha(p);return p
def vals(p):return {r[0]:np.array(r[1:],float) for r in csv.reader(pin(p).open())}
def rot(v):return Rotation.from_quat(v[[4,5,6,3]]).as_matrix()
def movement(a,b):
 keys=sorted((k for k in a if k in b and k.startswith('x')),key=lambda x:int(x[1:]));assert keys
 pa=np.array([a[k][:3] for k in keys]);pb=np.array([b[k][:3] for k in keys])
 matrices=np.array([rot(b[k])@rot(a[k]).T for k in keys]);R=Rotation.from_matrix(matrices).mean().as_matrix()
 t=pb.mean(axis=0)-R@pa.mean(axis=0);res=pb-(pa@R.T+t)
 da=np.linalg.norm(pb-pa,axis=1);dr=Rotation.from_matrix(matrices).magnitude()*180/np.pi
 vs=[np.linalg.norm(b[k]-a[k]) for k in a if k.startswith('v') and k in b]
 bas=[np.linalg.norm(b[k][:3]-a[k][:3]) for k in a if k.startswith('b') and k in b]
 bgs=[np.linalg.norm(b[k][3:]-a[k][3:]) for k in a if k.startswith('b') and k in b]
 gg=[]
 for x in keys:
  y='b'+x[1:];Ra,Rb=rot(a[x]),rot(b[x]);measured=b[y][:3]-a[y][:3]
  predicted=Rb.T@np.array([0,0,9.81])-Ra.T@np.array([0,0,9.81])
  den=np.linalg.norm(measured)*np.linalg.norm(predicted)
  if den>1e-20:gg.append(float(measured@predicted/den))
 return dict(poses=len(keys),max_position_m=float(max(da)),max_rotation_deg=float(max(dr)),
  previous_latest_key=keys[-1],previous_latest_position_m=float(da[-1]),previous_latest_rotation_deg=float(dr[-1]),
  common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
  common_world_yaw_deg=float(np.degrees(np.arctan2(R[1,0],R[0,0]))),
  common_shape_max_residual_m=float(max(np.linalg.norm(res,axis=1))),
  max_velocity_change_m_s=float(max(vs)),max_accel_bias_change_m_s2=float(max(bas)),max_gyro_bias_change_rad_s=float(max(bgs)),
  median_bias_gravity_compensation_cosine=float(np.median(gg)) if gg else None)
def cost(p):
 rows=list(csv.DictReader(pin(p).open()));families={};imu=[];smart=[]
 for row in rows:
  families[row['type']]=families.get(row['type'],0)+float(row['cost'])
  if row['type']=='imu':imu.append(row)
  if row['type']=='smart':smart.append(row)
 return dict(total=sum(families.values()),families=families,factors=len(rows),smart=len(smart),
  valid_smart=sum(int(r['valid']) for r in smart),
  max_imu_position_m=max(float(r['imu_position_m']) for r in imu),
  max_imu_rotation_deg=max(float(r['imu_rotation_deg']) for r in imu),
  max_imu_velocity_m_s=max(float(r['imu_velocity_m_s']) for r in imu))
def odom(p):
 rows=list(csv.DictReader(pin(p).open()));out={}
 for r in rows:
  i=int(r['field.header.seq']);t=int(r['field.header.stamp']);v=np.array([float(r['field.pose.pose.position.'+c]) for c in 'xyz']+[float(r['field.pose.pose.orientation.'+c]) for c in 'wxyz'])
  out[i]=(t,v)
 return out
assert json.loads((H/'inspection_passed.json').read_text())['passed']
spec=importlib.util.spec_from_file_location('cp50',H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py')
previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
result=dict(covariance_analysis=False,accuracy_scored=False,modes={},matched={})
data={};native={};snapshots={};latencies=[]
for mode in ('off','on'):
 source=P/f'full_{mode}_source';old=odom(Q/f'full_{mode}_source/kimera_odometry.csv');new=odom(source/'kimera_odometry.csv');native[mode]=new
 data[mode]=previous.parse_source(source/'estimators.log')
 logs=[p for p in (source/'ros_logs').glob('*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log') if p.parent.name!='latest'];assert len(logs)==1
 life=previous.parse_stdout(logs[0])
 phase=[];max_export_position=max_export_rotation=0
 for k in range(6,26):
  d=K/'inspection'/mode/f'update_{k}'
  states={s:vals(d/(s+'_values.csv')) for s in ('before_ordinary','after_ordinary','after_refresh')}
  c={s:cost(d/('same_graph_'+s+'.csv')) for s in ('initialized','ordinary','refresh')}
  eq={key:float(v) for key,v in csv.reader(pin(d/'equations.txt').open())}
  final=states['after_refresh'];index='x'+str(k)
  max_export_position=max(max_export_position,float(np.linalg.norm(final[index][:3]-new[k][1][:3])))
  max_export_rotation=max(max_export_rotation,float(Rotation.from_matrix(rot(final[index])@rot(new[k][1]).T).magnitude()*180/np.pi))
  for key,v in final.items():
   if not key.startswith('x'):continue
   logged=data[mode]['windows'][k][int(key[1:])]
   assert np.linalg.norm(v[:3]-logged[:3,3])<1e-10
   assert np.linalg.norm(rot(v)-logged[:3,:3])<1e-10
  phase.append(dict(k=k,time_s=(new[k][0]-EPOCH)*1e-9,costs=c,equations=eq,
   ordinary_revision=movement(states['before_ordinary'],states['after_ordinary']),
   refresh_revision=movement(states['after_ordinary'],final),net_revision=movement(states['before_ordinary'],final)))
  for name in states:
   p=source/'checkpoints'/f'update_{k}'/name/'checkpoint.bin';other=d/(name+'_roundtrip.bin')
   assert sha(p)==sha(other),(p,'byte roundtrip');snapshots[str(p)]=sha(p)
   latencies.append(float((p.parent/'complete.txt').read_text().splitlines()[1].split(',')[1]))
 eqrows=list(csv.DictReader(pin(K/'inspection'/mode/'equations.csv').open()))
 eqmax={key:max(abs(float(r[key])) for r in eqrows) for key in eqrows[0] if key not in ('update','phase')}
 common=sorted(set(new)&set(old));assert len(common)==len(new)
 prefix=dict(outputs=len(new),stamps_equal=all(new[i][0]==old[i][0] for i in common),
  all_pose_components_exact=all(np.array_equal(new[i][1],old[i][1]) for i in common),
  first_changed_id=next((i for i in common if not np.array_equal(new[i][1],old[i][1])),None),
  max_position_m=max(float(np.linalg.norm(new[i][1][:3]-old[i][1][:3])) for i in common),
  max_rotation_deg=max(float(Rotation.from_matrix(rot(new[i][1])@rot(old[i][1]).T).magnitude()*180/np.pi) for i in common))
 olddata=previous.parse_source(Q/f'full_{mode}_source/estimators.log')
 admissions=[]
 for key,v in data[mode]['aligned'].items():
  oldv=olddata['aligned'].get(key)
  admissions.append(dict(k=key[0],receiver=list(key[1:]),ids=v['ids'],time_s=(new[key[0]][0]-EPOCH)*1e-9,
   old_same_ids=oldv is not None and oldv['ids']==v['ids'],old_same_mean=oldv is not None and np.array_equal(oldv['mean'],v['mean'])))
 result['modes'][mode]=dict(updates=phase,prefix_vs_CPV49=prefix,admissions=admissions,
  equation_maxima=eqmax,snapshot_vs_native_output_max_position_m=max_export_position,
  snapshot_vs_native_output_max_rotation_deg=max_export_rotation,
  first_marginalization=next((dict(k=k,**v) for k,v in life['life'].items() if v['marginal_keys']),None),
  first_retirement=next((dict(k=k,**v) for k,v in life['life'].items() if v['retiring_factors']),None),
  recovery_events=life['failures'])
on=data['on']['windows'];off=data['off']['windows']
result['matched']['first_changed_window']=next((k for k in sorted(on) if k in off and any(not np.array_equal(v,off[k][i]) for i,v in on[k].items())),None)
result['matched']['windows_1_to_5_exact']=all(set(on[k])==set(off[k]) and all(np.array_equal(v,off[k][i]) for i,v in on[k].items()) for k in range(1,6))
assert len(snapshots)==120
result['observer_seconds']=dict(count=len(latencies),total=sum(latencies),maximum=max(latencies),median=float(np.median(latencies)))
result['checkpoint_sha256']=snapshots;pins.update(previous.PINS)
result['analysis_input_sha256']=pins
with (H/'analysis.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
for mode,v in result['modes'].items():
 print(mode,json.dumps({k:v[k] for k in ('prefix_vs_CPV49','equation_maxima','first_marginalization','first_retirement')}))
 for r in v['updates']:
  print(mode,r['k'],r['time_s'],'IMU',*[r['costs'][s]['families']['imu'] for s in ('initialized','ordinary','refresh')],
   'tilt ordinary/refresh/net',*[r[s]['common_world_tilt_deg'] for s in ('ordinary_revision','refresh_revision','net_revision')])
print('matched',result['matched'],'observer',result['observer_seconds'])
