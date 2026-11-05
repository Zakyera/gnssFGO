"""Read-only numerical diagnosis, NOT an estimator or new triangulation model.
Use the saved all-monocular smart factor to inspect cost versus positive depth.
At rho=0 use the analytic point-at-infinity limit for this diagnostic only.
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
lines=(R/'off_capture/factor_derivatives.txt.snapshot').read_text().splitlines()
begin=lines.index('smart,106675')
block=[]
for line in lines[begin+1:]:
    if line.startswith('smart,'):break
    block.append(line.split(','))
body=next(np.array(f[1:],float).reshape(4,4) for f in block if f[0]=='body')
point=next(np.array(f[2:],float) for f in block if f[0]=='cache_status')
sigma=next(float(f[-1]) for f in block if f[0]=='params')
measurements=[f for f in block if f[0]=='measurement']
poses={f[0][5:]:np.array(f[3:],float).reshape(4,4)@body for f in block if f[0].startswith('pose_')}
assert all(np.isnan(float(f[3])) for f in measurements)
cameras=[poses[f[1]] for f in measurements]
T0=cameras[0];p0=T0[:3,:3].T@(point-T0[:3,3]);xy=p0[:2]/p0[2]
def residual(angular,rho):
    out=[]
    for m,T in zip(measurements,cameras):
        # Homogeneous camera point; equals physical camera point *rho.
        p=T[:3,:3].T@(T0[:3,:3]@np.r_[angular,1.]+rho*(T0[:3,3]-T[:3,3]))
        assert p[2]>0
        fx,fy,skew,cx,cy,baseline=np.array(m[5:],float)
        assert skew==0
        out.extend([(fx*p[0]/p[2]+cx-float(m[2]))/sigma,(fy*p[1]/p[2]+cy-float(m[4]))/sigma])
    return np.array(out)
def jac(angular,rho):
    out=[]
    for m,T in zip(measurements,cameras):
        M=T[:3,:3].T@T0[:3,:3]
        p=M@np.r_[angular,1.]+rho*T[:3,:3].T@(T0[:3,3]-T[:3,3])
        fx,fy=float(m[5]),float(m[6]);x,y,z=p
        out.extend([(fx/z*M[0,:2]-fx*x/z**2*M[2,:2])/sigma,(fy/z*M[1,:2]-fy*y/z**2*M[2,:2])/sigma])
    return np.array(out)
raw=.5*np.sum(residual(xy,1/p0[2])**2)
assert abs(raw-.52211571334320217)<1e-10
rows=[]
for depth in (float(p0[2]),10.,20.,100.,1e3,1e4,1e6,float('inf')):
    rho=0. if not np.isfinite(depth) else 1/depth
    fit=least_squares(lambda x:residual(x,rho),xy,jac=lambda x:jac(x,rho),xtol=1e-13,ftol=1e-13,gtol=1e-12,max_nfev=100)
    assert fit.success
    rows.append({'first_camera_depth_m':depth if np.isfinite(depth) else 'infinity_limit',
                 'cost':float(fit.cost),'angular_gradient_inf':float(np.max(np.abs(jac(fit.x,rho).T@fit.fun))),
                 'angular':fit.x.tolist(),'evaluations':int(fit.nfev)})
result={'slot':106675,'measurements':len(measurements),'valid_right_observations':0,
        'same_noise_sigma':sigma,'saved_point_first_camera_depth_m':float(p0[2]),
        'saved_point_first_camera_range_m':float(np.linalg.norm(p0)),
        'saved_cost_crosscheck':raw,'profile':rows,
        'scope':'diagnostic depth profile only; no modified estimator, no proof of global optimum'}
with (H/'depth_profile.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps(result,indent=2))
