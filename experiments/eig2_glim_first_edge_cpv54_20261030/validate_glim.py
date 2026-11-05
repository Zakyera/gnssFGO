"""Readout completeness and native prediction/rotation consistency, no scoring."""
import json,sys
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
mode=sys.argv[1];assert mode in ('off','on');D=P/f'full_{mode}_source/glim_observer'
paths=list(D.glob('*.json'));expected={f'{k}_{s}.json' for k in range(11) for s in ('before','after')}
assert {p.name for p in paths}==expected
times=[];count=0
for p in paths:
 d=json.loads(p.read_text());k=d['update'];assert p.name==f"{k}_{d['phase']}.json"
 assert {v['id'] for v in d['states']}==set(range(k+1))
 for v in d['states']:
  T=np.array(v['T_world_imu']).reshape(4,4);R=T[:3,:3]
  assert np.all(np.isfinite(T)) and np.linalg.norm(R.T@R-np.eye(3))<1e-10 and abs(np.linalg.det(R)-1)<1e-10
 assert len(d['imu_factors'])==k
 for f in d['imu_factors']:
  assert f['to']==f['from']+1 and .09<f['dt']<.11
  if d['phase']=='before' and f['to']==k:
   # New native value must be exactly the recorded native IMU prediction.
   newest=d['states'][-1]
   assert np.max(np.abs(np.array(newest['T_world_imu'])-f['predicted_T_world_imu']))<1e-10
   assert np.linalg.norm(f['imu_residual_rotation_position_velocity'])<1e-8
  count+=1
 times.append(d['readout_us_before_write'])
result=dict(passed=True,mode=mode,readouts=len(paths),native_imu_factor_observations=count,max_readout_us=max(times),mean_readout_us=float(np.mean(times)),pure_lidar_pose_available=False)
with (H/(mode+'_glim_observer.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
