"""Check actual 4D factors separately from retained raw6 observation diagnostics."""
from pathlib import Path
import importlib.util,json,re,sys
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared')
S=P/'full_on_source'
spec=importlib.util.spec_from_file_location('geometry',H.parent/'leica2_drift_attribution_cpv7_20261025/analyze.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
def fields(line,tag):return re.sub(r'\x1b\[[0-9;]*m','',line).split(tag+',',1)[1].strip().split(',')
def mat(s):return np.array([[float(v) for v in row.split(':')] for row in s.split(';')])
def same(x,y):return np.allclose(x,y,rtol=1e-9,atol=1e-12)
def key(xs):return tuple(int(x[1:]) for x in xs)
frozen={};aligned={}
for line in (S/'estimators.log').open():
 if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
  f=fields(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');k=tuple(map(int,f[3:5]));assert k not in aligned
  aligned[k]=(mat(f[11]).ravel(),mat(f[12]))
 if 'FL4M_FROZEN_FACTOR_ROW,' in line:
  f=fields(line,'FL4M_FROZEN_FACTOR_ROW');k=tuple(map(int,f[3:5]));assert k not in frozen
  frozen[k]=(mat(f[5]).ravel(),mat(f[6]))
assert frozen and set(frozen)<=set(aligned)
for k,v in frozen.items():assert same(v[0],aligned[k][0]) and same(v[1],aligned[k][1])
logs=list(S.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log'));assert len(logs)==1
projected={};capped=0;max_cov=0.;max_residual=0.;times=[];min_loss=0.
for line in logs[0].open():
 if 'CPV9_PROJECTED_FACTOR_ROW,' in line:
  f=fields(line,'CPV9_PROJECTED_FACTOR_ROW');assert len(f)==16
  k=key(f[:2]);assert k not in projected and k in frozen
  gravity,Xi,Xj,Z,C,M,Cy,y,Hi,Hj=map(mat,f[4:14]);gravity=gravity.ravel();y=y.ravel()
  assert C.shape==(6,6) and M.shape==(4,6) and Cy.shape==(4,4) and Hi.shape==Hj.shape==(4,6)
  assert same(C,frozen[k][1]) and same(Z,a.g.exp(frozen[k][0]))
  expected=np.zeros((4,6));expected[0,:3]=(Xi[:3,:3]@Z[:3,:3]).T@(gravity/np.linalg.norm(gravity));expected[1:,3:]=np.eye(3)
  assert same(M,expected) and same(Cy,M@C@M.T)
  assert np.linalg.eigvalsh(Cy).min()>0
  loss=np.linalg.inv(C)-M.T@np.linalg.solve(Cy,M)
  minimum=float(np.linalg.eigvalsh(loss).min()/np.linalg.norm(np.linalg.inv(C),2));min_loss=min(min_loss,minimum)
  assert minimum>-1e-9
  r=a.g.log(a.g.inv(Z)@a.g.inv(Xi)@Xj)
  assert same(y,M@r)
  # Independent numerical pose-right perturbations at actual runtime endpoints.
  numeric=[]
  for which in (0,1):
   J=np.zeros((4,6))
   for axis in range(6):
    d=np.zeros(6);d[axis]=1e-6
    def eval(sign):
     u=Xi@a.g.exp(sign*d) if which==0 else Xi
     v=Xj@a.g.exp(sign*d) if which==1 else Xj
     return M@a.g.log(a.g.inv(Z)@a.g.inv(u)@v)
    J[:,axis]=(eval(1)-eval(-1))/(2e-6)
   numeric.append(J)
  assert np.allclose(Hi,numeric[0],rtol=1e-6,atol=1e-7) and np.allclose(Hj,numeric[1],rtol=1e-6,atol=1e-7),k
  max_cov=max(max_cov,float(np.max(np.abs(Cy-M@C@M.T))))
  max_residual=max(max_residual,float(np.max(np.abs(y-M@r))))
  times.append(float(f[14]));projected[k]=(Cy,M)
 if 'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW,G2K,' in line:
  f=fields(line,'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW')
  if f[8]=='receiver_inserted_factor' and f[9]=='covariance':
   k=key(f[5:7]);assert k in projected and same(mat(f[12]),projected[k][0]);capped+=1
assert projected and capped>0
# Opposite receiving direction stays original full6 covariance.
gl=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(gl)==1
incoming={};k2g=0
for line in gl[0].open():
 assert 'CPV9_PROJECTED_FACTOR_ROW,' not in line
 if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
  f=fields(line,'GLIM_CBS_MEAN_ALIGNMENT_ROW');k=tuple(map(int,f[2:4]));incoming.setdefault(k,[]).append(mat(f[11]))
 if 'CBS_ODOM_FACTOR_MATRIX_ROW,K2G,' in line:
  f=fields(line,'CBS_ODOM_FACTOR_MATRIX_ROW');k=key(f[1:3]);C=mat(f[4])
  assert C.shape==(6,6) and any(same(C,x) for x in incoming[k]);k2g+=1
meta=(S/'run_metadata.txt').read_text()
count=int(next(x.split('=',1)[1] for x in meta.splitlines() if x.startswith('g2k_injected=')))
assert count==len(projected),(count,len(projected))
assert (P/'full_on_control/complete.json').exists()
result=dict(passed=True,actual_G2K_factors=len(projected),unchanged_K2G_factors=k2g,
 capped_actual_4D_covariances=capped,raw_upstream_rows=len(frozen),
 max_covariance_error=max_cov,max_residual_error=max_residual,minimum_normalized_information_loss_eigenvalue=min_loss,
 projection_setup_ms=a.stat(times),exact_runtime_jacobians_checked=True,
 raw6_observation_diagnostics_not_applied_noise=True,gravity_from_native_imu=True,
 alignment_approximation_unchanged=True,default_promoted=False)
with (S/'projection_validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
