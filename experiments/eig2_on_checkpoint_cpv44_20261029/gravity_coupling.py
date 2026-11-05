"""Saved X/V/b geometric diagnostic, no covariance analysis or optimization."""
import csv,json,hashlib,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def sha(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins={}
def vals(p):
 pins[str(p)]=sha(p);return {r[0]:np.array(r[1:],float) for r in csv.reader(p.open())}
def rot(v):return Rotation.from_quat(v[[4,5,6,3]]).as_matrix()
profile=P/'prepared/full_on_source/kimera_profile/ImuParams.yaml';pins[str(profile)]=sha(profile)
g=np.array(json.loads(re.search(r'^n_gravity:\s*(\[[^\n]+\])',profile.read_text(),re.M)[1]),float)
events=[]
for k in range(21,26):
 d=P/'inspection/results'/f'update_{k}'
 states={s:vals(d/(s+'_values.csv')) for s in ('before_ordinary','after_ordinary','after_refresh')}
 for label,aa,bb in [('ordinary','before_ordinary','after_ordinary'),('refresh','after_ordinary','after_refresh'),('net','before_ordinary','after_refresh')]:
  a,b=states[aa],states[bb];rows=[]
  for x in sorted((x for x in a if x.startswith('x') and x in b),key=lambda x:int(x[1:])):
   key='b'+x[1:];v='v'+x[1:];Ra,Rb=rot(a[x]),rot(b[x]);G=Rb@Ra.T
   measured=b[key][:3]-a[key][:3]
   # f = R^T(a-g)+b. Under a common world rotation of trajectory kinematics,
   # preserving the same body accelerometer signal requires this bias change.
   gravity_only=Rb.T@g-Ra.T@g
   error=measured-gravity_only;den=np.linalg.norm(measured)*np.linalg.norm(gravity_only)
   rows.append(dict(key=x,actual_bias_delta=measured.tolist(),gravity_compensating_delta=gravity_only.tolist(),
    actual_norm=float(np.linalg.norm(measured)),predicted_norm=float(np.linalg.norm(gravity_only)),
    residual_norm=float(np.linalg.norm(error)),cosine=float(measured@gravity_only/den) if den else None,
    velocity_minus_rotated_previous=float(np.linalg.norm(b[v]-G@a[v]))))
  events.append(dict(k=k,phase=label,states=len(rows),
   median_cosine=float(np.median([r['cosine'] for r in rows])),
   median_actual_norm=float(np.median([r['actual_norm'] for r in rows])),
   median_gravity_prediction_norm=float(np.median([r['predicted_norm'] for r in rows])),
   rms_residual=float(np.sqrt(np.mean([r['residual_norm']**2 for r in rows]))),
   relative_squared_residual=sum(r['residual_norm']**2 for r in rows)/sum(r['actual_norm']**2 for r in rows),
   median_velocity_nonrigid_m_s=float(np.median([r['velocity_minus_rotated_previous'] for r in rows])),details=rows))
output=dict(gravity_world=g.tolist(),equation='delta_bias ~= R_after^T g - R_before^T g, conditional on common rotation of trajectory acceleration',
 scope='Geometric mechanism diagnostic, not sensor bias truth, causation proof, stationarity or an observability eigenvalue analysis',events=events,input_sha256=pins)
with (H/'gravity_coupling.json').open('x') as f:json.dump(output,f,indent=2,allow_nan=False)
for e in events:
 if e['phase']=='ordinary':print({key:v for key,v in e.items() if key!='details'})
