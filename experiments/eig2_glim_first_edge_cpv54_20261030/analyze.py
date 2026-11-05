"""Saved native first-edge readouts: bias, prediction and joint correction only."""
import hashlib,importlib.util,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as SO3
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
H53=H.parent/'eig2_first_rotation_cpv53_20261030';pins={}
def load(p):
 p=Path(p);pins[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return json.loads(p.read_text())
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def pin(p):
 pins[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();return p
spec=importlib.util.spec_from_file_location('geometry',pin(H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
def R(v):return np.asarray(v).reshape(4,4)[:3,:3]
def angle(A,B):return float(np.degrees(SO3.from_matrix(A.T@B).magnitude()))
def desc(r):return dict(matrix=r.tolist(),rotvec_deg=np.degrees(SO3.from_matrix(r).as_rotvec()).tolist())
assert load(H/'capture_complete.json')['passed']
old=load(H53/'analysis.json');raw=load(H53/'raw_samples.json')
ns=np.array([r[0] for r in raw['imu']],dtype=np.int64)
ts=(ns//10**9).astype(float)+(ns%10**9).astype(float)*1e-9
ww=np.array([r[1:] for r in raw['imu']]);kb=np.array(old['support']['gyro_bias_preupdate_b4'])
def integrate(start,end,bias,times=ts,rates=ww):
 # Native GLIM RIGHT support: each raw sample acts over its incoming interval;
 # first sample after end supplies final partial interval. No camera interpolation.
 assert times[0]<start<end<times[-1]
 r=np.eye(3);last=start
 for t,w in zip(times,rates):
  if t<=last:continue
  right=min(t,end);r=r@SO3.from_rotvec((w-bias)*(right-last)).as_matrix();last=right
  if last==end:return r
 raise AssertionError('raw coverage')
ft=np.linspace(0,1,101);fb=np.array([.01,-.02,.03]);fw=np.array([.04,.02,-.01]);fr=np.tile(fw+fb,(101,1))
assert angle(integrate(.135,.335,fb,ft,fr),SO3.from_rotvec(fw*.2).as_matrix())<1e-10
assert angle(integrate(.135,.335,fb,ft,fr),integrate(.135,.247,fb,ft,fr)@integrate(.247,.335,fb,ft,fr))<1e-10
pin(W/'src/glim/src/glim/common/imu_integration.cpp')
pin(W/'src/glim/src/glim/odometry/odometry_estimation_gpu.cpp')
out={}
for mode in ('off','on'):
 load(H/(mode+'_glim_observer.json'));S=P/f'full_{mode}_source'
 logs=[p for p in (S/'ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest'];assert len(logs)==1
 stamps={};export=None;events=[]
 for line_no,line in enumerate(pin(logs[0]).open(),1):
  if 'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,inserted,' in line:events.append(dict(line=line_no,text=a.ANSI.sub('',line.strip())))
  if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' not in line:continue
  f=a.ANSI.sub('',line.split('GLIM_CBS_EXPORT_ENDPOINT_ROW,',1)[1]).strip().split(',')
  i,j=map(int,f[3:5]);stamps[i]=float(f[5]);stamps[j]=float(f[6])
  if (i,j)==(5,7):
   assert export is None
   export=dict(line=line_no,current=int(f[1]),times=[float(f[5]),float(f[6])],mean=a.exp(f[21].split(';')))
 assert export and export['current']==7
 before=load(S/'glim_observer/7_before.json');after=load(S/'glim_observer/7_after.json')
 bs={v['id']:v for v in before['states']};states={v['id']:v for v in after['states']}
 native=R(states[5]['T_world_imu']).T@R(states[7]['T_world_imu'])
 match=angle(native,export['mean'][:3,:3]);assert match<1e-10
 pims={(f['from'],f['to']):f for f in after['imu_factors']}
 hat=np.eye(3);current=np.eye(3);raw_current=np.eye(3);raw_kimera=np.eye(3);edges=[]
 for i,j in ((5,6),(6,7)):
  f=pims[i,j];b=np.array(states[i]['bias_accel_gyro'][3:]);bh=np.array(f['bias_hat_accel_gyro'][3:])
  rhat=np.array(f['delta_R_hat']).reshape(3,3);rc=R(states[i]['T_world_imu']).T@R(f['predicted_T_world_imu'])
  actual=R(states[i]['T_world_imu']).T@R(states[j]['T_world_imu'])
  rh=integrate(stamps[i],stamps[j],bh);rr=integrate(stamps[i],stamps[j],b)
  reproduce=angle(rhat,rh)
  assert reproduce<1e-6,('native PIM reproduction',mode,i,reproduce)
  assert abs(f['dt']-(stamps[j]-stamps[i]))<1e-10
  hat=hat@rhat;current=current@rc;raw_current=raw_current@rr;raw_kimera=raw_kimera@integrate(stamps[i],stamps[j],kb)
  edges.append(dict(edge=[i,j],dt=f['dt'],bias_hat_gyro_rad_s=bh.tolist(),current_gyro_bias_rad_s=b.tolist(),
   bias_difference_from_Kimera_rad_s=(b-kb).tolist(),raw_PIM_reproduction_deg=reproduce,
   native_bias_correction_vs_raw_reintegration_deg=angle(rc,rr),native_IMU_rotation_residual_deg=angle(rc,actual),
   unwhitened_IMU_residual=f['imu_residual_rotation_position_velocity']))
 prior_events=[e for e in events if e['line']<export['line']]
 latest_correction=angle(R(bs[7]['T_world_imu']),R(states[7]['T_world_imu']))
 aligned=a.parse_source(S/'estimators.log');pins.update(a.PINS)
 first=min(aligned['aligned']) if aligned['aligned'] else None
 firstdata=aligned['aligned'][first] if first else None
 oldnative=np.array(old['exports'][mode]['endpoint_mean'])[:3,:3]
 trajectory=[]
 for k in range(1,11):
  pre=load(S/f'glim_observer/{k}_before.json');post=load(S/f'glim_observer/{k}_after.json')
  preS={x['id']:x for x in pre['states']};postS={x['id']:x for x in post['states']}
  ff=next(f for f in post['imu_factors'] if f['to']==k)
  trajectory.append(dict(update=k,gyro_bias=postS[k]['bias_accel_gyro'][3:],
    newest_joint_rotation_correction_deg=angle(R(preS[k]['T_world_imu']),R(postS[k]['T_world_imu'])),
    adjacent_IMU_rotation_residual_deg=float(np.degrees(np.linalg.norm(ff['imu_residual_rotation_position_velocity'][:3])))))
 out[mode]=dict(edge=[5,7],export_current=7,native_export_match_deg=match,CPV51_native_rotation_difference_deg=angle(oldnative,native),
   first_Kimera_admission=dict(update=first[0],ids=firstdata['ids'],times=firstdata['times']) if first else None,
   prior_K2G_events=prior_events,edges=edges,Kimera_bias_reference_rad_s=kb.tolist(),
   rotations=dict(joint_native=desc(native),IMU_biasHat_composed=desc(hat),IMU_current_Glim_bias_composed=desc(current),
      raw_gyro_Glim_bias=desc(raw_current),raw_gyro_Kimera_bias_Glim_support=desc(raw_kimera)),
   comparison_deg=dict(joint_vs_native_IMU_current_bias=angle(native,current),joint_vs_native_IMU_biasHat=angle(native,hat),
      joint_vs_raw_Kimera_bias=angle(native,raw_kimera),joint_vs_raw_Glim_bias=angle(native,raw_current),
      gyro_bias_change_effect=angle(raw_current,raw_kimera),newest_absolute_joint_correction=latest_correction),
   early_updates=trajectory)
save('analysis.json',dict(modes=out,input_sha256=pins,raw_gyro_is_ground_truth=False,
 limits=['One short event, not full-run root cause.','Joint correction is not a standalone LiDAR-only measurement.',
 'Kimera pre-update bias reference comes from CP-V51; scheduling parity explicitly reported.',
 'No covariance calibration claim.','GTSAM synthetic parity does not prove identical asynchronous live scheduling.']))
print(json.dumps({m:{k:v for k,v in d.items() if k in ('comparison_deg','edges','first_Kimera_admission','CPV51_native_rotation_difference_deg')} for m,d in out.items()},indent=2))
