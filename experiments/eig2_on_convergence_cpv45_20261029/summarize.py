"""Summarize unchanged native cost probes and derivative evidence; no scoring."""
import csv,hashlib,json,math
from collections import defaultdict
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
Q=W/'runs/eig2_on_checkpoint_cpv44_20261029';pins={}
def sha(p):
 with p.open('rb') as f:return hashlib.sha256(f.read()).hexdigest()
def rows(p):
 pins[str(p)]=sha(p)
 return list(csv.DictReader(p.open()))
def vals(p):
 pins[str(p)]=sha(p)
 return {r[0]:np.array(r[1:],float) for r in csv.reader(p.open())}
def rot(v):
 w,x,y,z=v[3:];assert abs(w*w+x*x+y*y+z*z-1)<1e-10
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def movement(a,b):
 keys=[k for k in a if k in b and k.startswith('x')]
 mats=[rot(b[k])@rot(a[k]).T for k in keys]
 u,_,vt=np.linalg.svd(sum(mats));correction=np.eye(3);correction[2,2]=np.linalg.det(u@vt);R=u@correction@vt
 angles=[np.degrees(np.arccos(np.clip((np.trace(m)-1)/2,-1,1))) for m in mats]
 biases=[float(np.linalg.norm(b['b'+k[1:]][:3]-a['b'+k[1:]][:3])) for k in keys]
 return dict(poses=len(keys),common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
             maximum_orientation_change_deg=float(max(angles)),median_accel_bias_change_m_s2=float(np.median(biases)),
             maximum_position_change_m=float(max(np.linalg.norm(b[k][:3]-a[k][:3]) for k in keys)))
def cost(p):
 r=rows(p);families=defaultdict(float)
 for z in r:families[z['type']]+=float(z['cost'])
 imu=[z for z in r if z['type']=='imu'];smart={z['slot']:int(z['status']) for z in r if z['type']=='smart'}
 return dict(total=sum(families.values()),families=dict(families),
             active_smart=sum(int(z['valid']) for z in r if z['type']=='smart'),
             max_imu_position_m=max(float(z['imu_position_m']) for z in imu),
             max_imu_rotation_deg=max(float(z['imu_rotation_deg']) for z in imu),
             max_imu_velocity_m_s=max(float(z['imu_velocity_m_s']) for z in imu)),smart
assert np.allclose(rot(np.array([0,0,0,1,0,0,0.])),np.eye(3))
events=[]
old=json.loads((H.parent/Q.name/'analysis.json').read_text())
for line in rows(P/'results/summary.csv'):
 k=int(line['update']);d=P/'results'/f'update_{k}';der=rows(d/'derivatives.csv');peers=rows(d/'peer_jacobians.csv')
 counts={}
 for f in sorted(set(z['type'] for z in der)):
  r=[z for z in der if z['type']==f and float(z['epsilon'])<=1e-5]
  bad=[z for z in r if z['pass']=='0']
  counts[f]=dict(checks=len(r),failed_checks=len(bad),factors=len(set(z['slot'] for z in r)),failed_factors=len(set(z['slot'] for z in bad)))
 pe=[z for z in peers if float(z['epsilon'])==1e-5]
 native=np.array([float(z['native_cost_derivative']) for z in pe]);chain=np.array([float(z['chain_cost_derivative']) for z in pe]);fd=np.array([float(z['finite_difference_cost']) for z in pe])
 grad=dict(native_stacked_relative_l2=float(np.linalg.norm(native-fd)/np.linalg.norm(fd)),
           chain_stacked_relative_l2=float(np.linalg.norm(chain-fd)/np.linalg.norm(fd)),
           native_component_sign_disagreements=int(sum(native*fd<0)),
           chain_failed_checks=sum(z['chain_pass']=='0' for z in peers),
           checks=len(peers))
 captured=vals(d/'captured_values.csv');before=vals(Q/'inspection/results'/f'update_{k}'/'before_ordinary_values.csv')
 base,ba=cost(d/'captured_factors.csv');net=movement(before,captured)
 assert abs(net['common_world_tilt_deg']-old['updates'][k-21]['net_revision']['common_world_tilt_deg'])<1e-7
 probes=[]
 for j in range(13):
  x=vals(d/f'probe_{j}_values.csv');c,activity=cost(d/f'probe_{j}_factors.csv')
  probes.append(dict(index=j,fraction=2**(-j),cost=c,status_changes=sum(activity[s]!=v for s,v in ba.items()),
                     from_captured=movement(captured,x),from_before_ordinary=movement(before,x),
                     lower_reported_cost=c['total']<base['total']))
 event=dict(update=k,native_equations={key:float(value) for key,value in line.items() if key!='update'},
            derivative_families=counts,peer_gradient=grad,captured_cost=base,captured_net_movement=net,probes=probes)
 events.append(event)
 print(k,'native CBS gradient discrepancy%',100*grad['native_stacked_relative_l2'],
       'cost',base['total'],'->',probes[0]['cost']['total'],
       'net tilt',net['common_world_tilt_deg'],'->',probes[0]['from_before_ordinary']['common_world_tilt_deg'],
       'extra tilt',probes[0]['from_captured']['common_world_tilt_deg'])
out=dict(complete=True,native_convergence_admitted=False,iterative_optimizations=0,source_replays=0,
         covariance_investigated=False,accuracy_scored=False,interpretation='Five independent saved graphs, thirteen fractions of one native GN direction each; no propagated iteration and no stationary solution',
         updates=events,input_sha256=pins)
with (H/'analysis.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
