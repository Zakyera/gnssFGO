"""Summarize independent saved one-step arms, not trajectory accuracy."""
import csv,hashlib,json
from collections import defaultdict
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
pins={}
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def rows(p):
 pins[str(p)]=sha(p)
 with p.open() as f:return list(csv.DictReader(f))
def vals(p):
 pins[str(p)]=sha(p)
 with p.open() as f:return {r[0]:np.array(r[1:],float) for r in csv.reader(f)}
def rot(v):
 w,x,y,z=v[3:];assert abs(w*w+x*x+y*y+z*z-1)<1e-10
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def angle(r):return float(np.degrees(np.arccos(np.clip((np.trace(r)-1)/2,-1,1))))
def movement(a,b,keys=None):
 keys=keys or [k for k in a if k in b and k.startswith('x')]
 mats=[rot(b[k])@rot(a[k]).T for k in keys]
 u,_,vt=np.linalg.svd(sum(mats));fix=np.eye(3);fix[2,2]=np.linalg.det(u@vt);R=u@fix@vt
 bias=[];pred=[];errors=[];cos=[];g=np.array([0.,0.,9.81])
 for k in keys:
  d=b['b'+k[1:]][:3]-a['b'+k[1:]][:3];p=rot(b[k]).T@g-rot(a[k]).T@g
  bias.append(float(np.linalg.norm(d)));pred.append(float(np.linalg.norm(p)));errors.append(float(np.linalg.norm(d-p)))
  cos.append(float(d@p/(np.linalg.norm(d)*np.linalg.norm(p))) if np.linalg.norm(d)*np.linalg.norm(p)>1e-20 else None)
 latest=max(keys,key=lambda k:int(k[1:]))
 return dict(poses=len(keys),common_world_rotation_matrix=R.tolist(),common_world_rotation_deg=angle(R),
  common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
  rotation_dispersion_p95_deg=float(np.percentile([angle(m@R.T) for m in mats],95)),
  maximum_orientation_change_deg=max(angle(m) for m in mats),
  median_accel_bias_change_m_s2=float(np.median(bias)),median_gravity_compensation_m_s2=float(np.median(pred)),
  median_bias_minus_gravity_compensation_m_s2=float(np.median(errors)),median_bias_gravity_cosine=float(np.median([v for v in cos if v is not None])) if any(v is not None for v in cos) else None,
  maximum_position_change_m=max(float(np.linalg.norm(b[k][:3]-a[k][:3])) for k in keys),
  latest_old_key=latest,latest_old_position_change_m=float(np.linalg.norm(b[latest][:3]-a[latest][:3])),
  latest_old_rotation_change_deg=angle(rot(b[latest])@rot(a[latest]).T))
def cost(p):
 r=rows(p);families=defaultdict(float)
 for z in r:families[z['type']]+=float(z['cost'])
 imu=[z for z in r if z['type']=='imu'];smart={z['slot']:(int(z['valid']),int(z['status'])) for z in r if z['type']=='smart'}
 return dict(total=sum(families.values()),native=sum(v for k,v in families.items() if k!='peer_pose'),families=dict(families),
  active_smart=sum(v[0] for v in smart.values()),smart_factors=len(smart),
  max_imu_position_m=max(float(z['imu_position_m']) for z in imu),
  max_imu_rotation_deg=max(float(z['imu_rotation_deg']) for z in imu),
  max_imu_velocity_m_s=max(float(z['imu_velocity_m_s']) for z in imu)),smart
assert np.allclose(rot(np.array([0,0,0,1,0,0,0.])),np.eye(3))
summary=rows(P/'results/summary.csv');parity=rows(P/'results/parity.csv');events=[]
old=json.loads((H.parent/'eig2_on_checkpoint_cpv44_20261029/analysis.json').read_text())
for k in range(21,26):
 d=P/'results'/f'update_{k}';before=vals(d/'previous_values.csv');initial=vals(d/'initial_values.csv');captured=vals(d/'captured_values.csv')
 keys=[key for key in before if key.startswith('x')];base,activity=cost(d/'initial_factors.csv')
 arms={};states={}
 for mode in 'ABC':
  states[mode]=vals(d/f'{mode}_values.csv');c,s=cost(d/f'{mode}_factors.csv')
  eq=next(r for r in summary if int(r['update'])==k and r['arm']==mode)
  arms[mode]=dict(from_previous=movement(before,states[mode]),from_initial_old=movement(initial,states[mode],keys),cost=c,
   smart_status_changes=sum(s[z]!=v for z,v in activity.items()),equations={key:float(value) for key,value in eq.items() if key not in ('update','arm')})
 # Numerical reconstruction gate, not a second optimization or accuracy metric.
 reference=movement(before,captured)
 assert abs(reference['common_world_tilt_deg']-arms['B']['from_previous']['common_world_tilt_deg'])<1e-5
 events.append(dict(update=k,initial_cost=base,initial_old_movement=movement(before,initial),
  captured_ordinary_movement=reference,arms=arms,
  A_vs_B=movement(states['B'],states['A'],keys),C_vs_B=movement(states['B'],states['C'],keys),
  parity={key:float(value) for key,value in next(r for r in parity if int(r['update'])==k).items() if key!='update'}))
 print(k,'tilt A/B/C',[round(arms[m]['from_previous']['common_world_tilt_deg'],6) for m in 'ABC'],
  'bias',[round(arms[m]['from_previous']['median_accel_bias_change_m_s2'],6) for m in 'ABC'],
  'native cost',[round(arms[m]['cost']['native'],3) for m in 'ABC'])
out=dict(complete=True,independent_updates=5,one_step_solves=15,source_replays=0,iterative_optimizations=0,
 covariance_investigated=False,accuracy_scored=False,interpretation='A is ON-conditioned, not a fresh CBS-OFF result. All arms share pre-ordinary-solve initial values; no propagation between updates.',
 updates=events,input_sha256=pins)
with (H/'analysis.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
