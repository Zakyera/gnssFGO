"""Audit actual Gaussian factor matrices, not the upstream frozen input row."""
from pathlib import Path
from collections import defaultdict
import json,re,sys
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_rotation_weight_cpv6a_20261024/prepared')
S=P/'full_on_source'
def fields(line,tag):return re.sub(r'\x1b\[[0-9;]*m','',line).split(tag+',',1)[1].strip().split(',')
def matrix(s):return np.array([[float(v) for v in r.split(':')] for r in s.split(';')])
def same(a,b):return np.allclose(a,b,rtol=1e-10,atol=1e-18)
aligned={};frozen={}
for line in (S/'estimators.log').open():
 if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
  f=fields(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');key=tuple(map(int,f[3:5]))
  assert key not in aligned;aligned[key]=(matrix(f[11]).ravel(),matrix(f[12]))
 if 'FL4M_FROZEN_FACTOR_ROW,' in line:
  f=fields(line,'FL4M_FROZEN_FACTOR_ROW');key=tuple(map(int,f[3:5]))
  assert key not in frozen;frozen[key]=(matrix(f[5]).ravel(),matrix(f[6]))
assert frozen and set(frozen)<=set(aligned)
for key,(mu,c) in frozen.items():
 rawmu,raw=aligned[key]
 assert np.allclose(mu,rawmu,rtol=1e-10,atol=1e-12) and same(c,raw)
counts={};max_error=0.
for direction,pattern in [('G2K','kimera_vio_ros-kimera_vio_ros_node-*-stdout.log'),('K2G','glim_rosnode-*-stdout.log')]:
 paths=list(S.glob('ros_logs/*/'+pattern));assert len(paths)==1
 transport=defaultdict(list);factor_count=0;transport_count=0;capped_count=0
 for line in paths[0].open():
  if direction=='K2G' and 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
   f=fields(line,'GLIM_CBS_MEAN_ALIGNMENT_ROW');key=tuple(map(int,f[2:4]))
   transport[key].append(matrix(f[11]));transport_count+=1
  if 'CBS_BPSAM_SOURCE_TRANSPORT_ROW,'+direction+',' in line:
   f=fields(line,'CBS_BPSAM_SOURCE_TRANSPORT_ROW');key=tuple(int(x[1:]) for x in f[3:5])
   raw,applied=matrix(f[5]),matrix(f[6]);want=raw.copy()
   if direction=='G2K':
    want[:3,:3]*=10
    assert key in frozen and same(raw,frozen[key][1])
   assert same(applied,want)
   assert np.array_equal(applied[3:,:],raw[3:,:]) and np.array_equal(applied[:3,3:],raw[:3,3:])
   assert np.linalg.eigvalsh(applied).min()>0
   transport[key].append(applied);transport_count+=1
   max_error=max(max_error,float(np.max(np.abs(applied-want))))
  if 'CBS_ODOM_FACTOR_MATRIX_ROW,'+direction+',' in line:
   f=fields(line,'CBS_ODOM_FACTOR_MATRIX_ROW');key=tuple(int(x[1:]) for x in f[1:3]);c=matrix(f[4])
   assert key in transport and any(same(c,v) for v in transport[key]),(direction,key,f[3])
   factor_count+=1
  if 'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW,'+direction+',' in line:
   f=fields(line,'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW')
   if f[8]=='receiver_inserted_factor' and f[9]=='covariance':
    key=tuple(int(x[1:]) for x in f[5:7]);assert any(same(matrix(f[12]),v) for v in transport[key]);capped_count+=1
 assert factor_count>0 and transport_count>0
 counts[direction]=dict(transport=transport_count,actual_factor_matrices=factor_count,capped_inserted=capped_count)
r=dict(passed=True,counts=counts,upstream_frozen_rows=len(frozen),max_covariance_error=max_error,
       rotation_variance_multiplier=10,translation_and_cross_blocks_exactly_unchanged=True,
       mean_unchanged=True,k2g_unchanged=True,
       correction='FL4M_FROZEN_FACTOR_ROW is pre-BPSAM input. Actual CBS_ODOM_FACTOR_MATRIX_ROW is immediately used for Gaussian::Covariance.',
       original_validator_preserved=True)
if '--partial' not in sys.argv:
 assert (P/'full_on_control/complete.json').exists()
 with (S/'weight_validation.json').open('x') as f:json.dump(r,f,indent=2)
else:
 with (H/'partial_weight_check.json').open('x') as f:json.dump(r,f,indent=2)
print(json.dumps(r))
