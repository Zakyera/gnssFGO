"""Independent NumPy DLT/guards and SciPy pose metrics for CP61."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def load(n):return json.loads((P/n).read_text())
inputs=[Path(__file__),H/'analysis.json',P/'local_geometry.jsonl']+list(P.glob('*ordinary*.json'))+list(P.glob('*refresh*.json'))+[P/'previous.json',P/'initial.json']
pins={str(p):sha(p) for p in inputs}
analysis=json.loads((H/'analysis.json').read_text())
point_max=0.;status_count=0;point_count=0;details=[]
for row in (json.loads(l) for l in (P/'local_geometry.jsonl').open()):
    cameras=row['cameras'];A=[]
    for c in cameras:
        pose=np.array(c['pose']).reshape(4,4);R=pose[:3,:3];t=pose[:3,3];K=np.array(c['K']).reshape(3,3)
        proj=K@np.column_stack([R.T,-R.T@t]);u,v=c['z']
        A.extend([u*proj[2]-proj[0],v*proj[2]-proj[1]])
    _,singular,Vh=np.linalg.svd(np.array(A),full_matrices=True);rank=int(sum(singular>row['rank_tolerance']))
    status=1;point_error=None
    if rank>=3:
        point=Vh[-1,:3]/Vh[-1,3]
        assert np.all(np.isfinite(point))
        depth=[];distance=[];error=[]
        for c in cameras:
            pose=np.array(c['pose']).reshape(4,4);local=pose[:3,:3].T@(point-pose[:3,3])
            depth.append(local[2]);distance.append(np.linalg.norm(local))
            pix=np.array(c['K']).reshape(3,3)@local;pixel=pix[:2]/pix[2]
            error.append(np.linalg.norm(pixel-np.array(c['z'])))
        status=2 if min(depth)<=0 else 4 if max(distance)>row['max_distance'] else 3 if max(error)>row['max_reprojection'] else 0
        if status==0:
            assert row['point_local'] is not None
            point_error=float(np.linalg.norm(point-np.array(row['point_local'])))
            assert point_error<1e-7,(row['phase'],row['slot'],point_error)
            point_max=max(point_max,point_error);point_count+=1
    assert status==row['status'],(row['phase'],row['slot'],status,row['status'],singular)
    status_count+=1
    details.append(dict(phase=row['phase'],slot=row['slot'],status=status,rank=rank,point_difference_m=point_error,singular_values=singular.tolist()))
assert status_count==1014
states={n:{k:np.array(v) for k,v in load(n+'.json').items()} for n in ['previous','initial']+[a+'_'+p for a in 'ABC' for p in ['ordinary','refresh']]}
def rot(v):return Rotation.from_quat(v[[4,5,6,3]])
metric_count=0;metric_max=0.
def verify(a,b,reported):
    global metric_count,metric_max
    for k,metrics in reported['per_state'].items():
        n=k[1:]
        own=dict(position_m=float(np.linalg.norm(b[k][:3]-a[k][:3])),
                 rotation_deg=float(np.degrees((rot(b[k])*rot(a[k]).inv()).magnitude())),
                 velocity_m_s=float(np.linalg.norm(b['v'+n]-a['v'+n])),
                 accel_bias_m_s2=float(np.linalg.norm(b['b'+n][:3]-a['b'+n][:3])),
                 gyro_bias_rad_s=float(np.linalg.norm(b['b'+n][3:]-a['b'+n][3:])))
        for name,value in own.items():
            err=abs(value-metrics[name]);assert err<1e-10,(name,value,metrics[name])
            metric_count+=1;metric_max=max(metric_max,err)
for a in 'ABC':
    d=analysis['arms'][a];ordinary=states[a+'_ordinary'];final=states[a+'_refresh']
    verify(states['previous'],ordinary,d['ordinary']);verify(ordinary,final,d['refresh']);verify(states['previous'],final,d['net'])
    verify(states['initial'],ordinary,d['newest_ordinary']);verify(ordinary,final,d['newest_refresh'])
for name,a,b in [('A_to_B','A','B'),('B_to_C','B','C')]:
    for phase in ['ordinary','refresh']:verify(states[a+'_'+phase],states[b+'_'+phase],analysis[name][phase])
assert metric_count==440
for phase in ['ordinary','refresh']:
    a=states['C_'+phase];shifted=load('C_'+phase+'_shifted.json')
    for k,v in a.items():
        expected=np.array(shifted[k])
        if k.startswith('x'):expected[:3]-=[3,-2,1]
        assert np.array_equal(v,expected)
for p,h in pins.items():assert sha(p)==h,p
result=dict(passed=True,native_status_matches=status_count,valid_point_matches=point_count,max_point_difference_m=point_max,
            scalar_metric_checks=metric_count,max_metric_difference=metric_max,translated_output_checks=True,
            input_sha256=pins,records=details)
with (H/'independent_check.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ['input_sha256','records']},indent=2))

