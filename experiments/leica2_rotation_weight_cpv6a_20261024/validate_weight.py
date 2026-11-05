"""Audit every actual frozen G2K factor against its aligned raw covariance."""
from pathlib import Path
import json,re,sys
import numpy as np
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_rotation_weight_cpv6a_20261024/prepared')
S=P/'full_on_source'
def fields(line,tag):return re.sub(r'\x1b\[[0-9;]*m','',line).split(tag+',',1)[1].strip().split(',')
def matrix(s):return np.array([[float(v) for v in r.split(':')] for r in s.split(';')])
aligned={}; frozen={}
for line in (S/'estimators.log').open():
 if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
  f=fields(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');key=tuple(map(int,f[3:5]))
  assert key not in aligned;aligned[key]=(matrix(f[11]).ravel(),matrix(f[12]))
 if 'FL4M_FROZEN_FACTOR_ROW,' in line:
  f=fields(line,'FL4M_FROZEN_FACTOR_ROW');key=tuple(map(int,f[3:5]))
  assert key not in frozen;frozen[key]=(matrix(f[5]).ravel(),matrix(f[6]))
assert frozen and set(frozen)<=set(aligned)
max_error=0.
for key,(mu,c) in frozen.items():
 rawmu,raw=aligned[key];want=raw.copy();want[:3,:3]*=10
 assert np.allclose(mu,rawmu,rtol=1e-10,atol=1e-12)
 assert np.allclose(c,want,rtol=1e-10,atol=1e-18),(key,c,want)
 assert np.array_equal(c[3:,:],raw[3:,:]) and np.array_equal(c[:3,3:],raw[:3,3:])
 assert np.linalg.eigvalsh(c).min()>0
 max_error=max(max_error,float(np.max(np.abs(c-want))))
logs=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(logs)==1
k2g=0
for line in logs[0].open():
 if 'CBS_BPSAM_SOURCE_TRANSPORT_ROW,K2G,' in line:
  f=fields(line,'CBS_BPSAM_SOURCE_TRANSPORT_ROW')
  assert np.array_equal(matrix(f[-2]),matrix(f[-1]));k2g+=1
assert k2g>0
r=dict(passed=True,g2k_frozen_factors=len(frozen),aligned_not_frozen=len(set(aligned)-set(frozen)),
       k2g_identity_rows=k2g,max_covariance_error=max_error,rotation_variance_multiplier=10,
       translation_and_cross_blocks_exactly_unchanged=True,mean_unchanged=True)
if '--partial' not in sys.argv:
 assert (P/'full_on_control/complete.json').exists()
 with (S/'weight_validation.json').open('x') as f:json.dump(r,f,indent=2)
print(json.dumps(r))
