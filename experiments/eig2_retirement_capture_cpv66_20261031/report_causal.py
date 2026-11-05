"""Independent pose-vector contrasts for saved native event branches."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
D=K/'causal/results'
assert json.loads((K/'causal/passed.json').read_text())['passed']
capture=json.loads((H/'capture_analysis.json').read_text())
def read(path):
    with path.open() as f:return {r[0]:np.array(r[1:],float) for r in csv.reader(f)}
def pose(v):
    return v[:3],Rotation.from_quat(v[[4,5,6,3]]).as_matrix()
def project(e,d,floor):
    size=np.linalg.norm(d);effect=np.linalg.norm(e)
    if size<floor:return dict(observed=float(size),effect=float(effect),fraction=None,perpendicular=None)
    fraction=float(e@d/(d@d))
    return dict(observed=float(size),effect=float(effect),fraction=fraction,
                perpendicular=float(np.linalg.norm(e-fraction*d)))
def costs(path):
    sums={};valid=[]
    with path.open() as f:
        for r in csv.DictReader(f):
            key=r['family'];sums[key]=sums.get(key,0.)+float(r['cost'])
            if key=='smart' and int(r['valid'])==1:valid.append(int(r['slot']))
    return dict(sum_by_family=sums,valid_smart_slots=valid)
def geometry(previous,current,k):
    old=[key for key in previous if key.startswith('x') and key in current]
    A=np.array([pose(previous[key])[0] for key in old]);B=np.array([pose(current[key])[0] for key in old])
    p0,r0=pose(previous[f'x{k-1}']);p1,r1=pose(current[f'x{k-1}'])
    pn,rn=pose(current[f'x{k}'])
    u,_,vt=np.linalg.svd((A-A.mean(0)).T@(B-B.mean(0)))
    fix=np.eye(3);fix[-1,-1]=np.linalg.det(vt.T@u.T)
    rigid=vt.T@fix@u.T;shift=B.mean(0)-rigid@A.mean(0)
    residual=B-(A@rigid.T+shift)
    rotation=Rotation.from_matrix(r0.T@r1).as_rotvec()
    return dict(previous_translation_revision_world=(p1-p0).tolist(),
                previous_rotation_revision_chart=rotation.tolist(),
                previous_translation_revision_m=float(np.linalg.norm(p1-p0)),
                previous_rotation_revision_deg=float(np.rad2deg(np.linalg.norm(rotation))),
                published_step_m=float(np.linalg.norm(pn-p0)),
                within_solution_step_m=float(np.linalg.norm(pn-p1)),
                old_window_rigid_residual_rms_m=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))),
                max_gyro_bias_revision_rad_s=max(float(np.linalg.norm(current[key][3:]-previous[key][3:])) for key in previous if key.startswith('b') and key in current),
                max_accel_bias_revision_m_s2=max(float(np.linalg.norm(current[key][:3]-previous[key][:3])) for key in previous if key.startswith('b') and key in current),
                max_velocity_revision_m_s=max(float(np.linalg.norm(current[key]-previous[key])) for key in previous if key.startswith('v') and key in current))
events=[]
for target in capture['selected_events']:
    k=target['k'];root=D/f'update_{k}';previous=read(root/'previous.csv')
    phases={}
    for phase in ('ordinary','refresh'):
        states={arm:read(root/f'{arm}_{phase}.csv') for arm in 'ABCD'}
        metrics={arm:geometry(previous,x,k) for arm,x in states.items()}
        p0,r0=pose(previous[f'x{k-1}'])
        p={arm:pose(x[f'x{k-1}'])[0] for arm,x in states.items()}
        r={arm:Rotation.from_matrix(r0.T@pose(x[f'x{k-1}'])[1]).as_rotvec() for arm,x in states.items()}
        contrasts={}
        for label,coeff in [('removal',{'A':1,'B':-1}),('admission',{'B':1,'C':-1}),
                            ('interaction',{'A':1,'D':-1,'B':-1,'C':1})]:
            et=sum(weight*p[arm] for arm,weight in coeff.items())
            er=sum(weight*r[arm] for arm,weight in coeff.items())
            contrasts[label]=dict(translation=project(et,p['A']-p0,1e-6),rotation=project(er,r['A'],1e-6))
        if not target['removed']:
            assert contrasts['removal']['translation']['effect']<1e-9
            assert contrasts['removal']['rotation']['effect']<1e-9
        phases[phase]=dict(arms=metrics,contrasts=contrasts)
    # Independent native-log comparison: previous-revision screen and A.
    screen=target['screen']
    assert abs(phases['refresh']['arms']['A']['previous_translation_revision_m']-screen['correction_m'])<1e-5
    events.append(dict(k=k,time_s=screen['capture_time_s'],targets=target,phases=phases,
                       costs={arm:costs(root/f'{arm}_costs.csv') for arm in 'ABCD'}))
result=dict(passed=True,events=events,limits=[
    'Same native ordinary linearization charts; two steps, no forward propagation.',
    'Retained factors never enter a marginal prior in this experiment.',
    'Projection fractions can be negative or exceed one; not variance explained.',
    'Short prefix, no full-sequence efficacy or physical drone motion claim.',
    'No EVO ATE/RPE or covariance/weight tuning.'])
with (H/'causal_analysis.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print('k,time_s,observed_revision_m,removal_effect_m,projected_fraction,retained_revision_m')
for e in events:
    f=e['phases']['refresh'];c=f['contrasts']['removal']['translation']
    print(e['k'],e['time_s'],c['observed'],c['effect'],c['fraction'],f['arms']['B']['previous_translation_revision_m'],sep=',')
