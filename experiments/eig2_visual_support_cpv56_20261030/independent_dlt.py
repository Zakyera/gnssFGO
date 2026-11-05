"""Independent NumPy SVD of native exported camera/measurement geometry."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'gauge_decomposition/results/gauge.jsonl'
rows=[json.loads(l) for l in P.open()]
errors=[];status_differences=[];groups=collections.defaultdict(list)
for r in rows:
    A=[];cameras=[]
    for c in r['cameras']:
        pose=np.array(c['pose']).reshape(4,4);K=np.array(c['K']).reshape(3,3);z=c['z']
        for side,u in [(0,z[0]),(1,z[1])]:
            if u is None:continue
            R=pose[:3,:3];t=pose[:3,3]+R@np.array([side*c['baseline'],0,0])
            projection=K@np.column_stack((R.T,-R.T@t))
            A.extend([u*projection[2]-projection[0],z[2]*projection[2]-projection[1]])
            cameras.append((R,t,K,np.array([u,z[2]])))
    _,singular,Vh=np.linalg.svd(np.array(A),full_matrices=True)
    rank=int(np.sum(singular>r['rank_tolerance']))
    if rank<3:status=1
    else:
        point=Vh[-1,:3]/Vh[-1,3]
        depths=[];distances=[];reproj=[]
        for R,t,K,z in cameras:
            local=R.T@(point-t);depths.append(local[2]);distances.append(np.linalg.norm(point-t));uv=K@local
            reproj.append(np.linalg.norm(uv[:2]/uv[2]-z))
        status=2 if min(depths)<=0 else 4 if max(distances)>10 else 3 if max(reproj)>3 else 0
        if r['status'] in (0,3,4):
            error=np.linalg.norm(point-np.array(r['native_point']))
            errors.append(float(error));assert error<1e-6*(1+np.linalg.norm(point)),(r['phase'],r['slot'],r['transform'],error)
    if status!=r['status']:status_differences.append([r['phase'],r['slot'],r['transform'],r['status'],status])
    groups[(r['phase'],r['transform'])].append(r)
assert not status_differences,status_differences[:10]
summary={}
for (phase,transform),group in groups.items():
    baseline={r['slot']:r for r in groups[(phase,'identity')]}
    changed=[r for r in group if r['status']!=baseline[r['slot']]['status']]
    point_errors=[r['point_error_m'] for r in group if r['point_error_m']>=0]
    summary[phase+'_'+transform]=dict(tracks=len(group),status_changes=len(changed),status_transition_counts={f'{a}->{b}':n for (a,b),n in collections.Counter((baseline[r['slot']]['status'],r['status']) for r in changed).items()},valid_to_invalid=sum(baseline[r['slot']]['status']==0 and r['status']!=0 for r in group),point_error_median_m=float(np.median(point_errors)),point_error_max_m=max(point_errors))
result=dict(passed=True,independent_SVD_rows=len(rows),native_point_comparisons=len(errors),max_native_numpy_point_error_m=max(errors),native_numpy_status_differences=status_differences,summary=summary,
    status_names=['valid','degenerate','behind_camera','outlier','far_point'],
    qualification='Reproduces native algebraic DLT and native gate decisions, not independent physical truth, optimal reprojection triangulation, or a cause of the full-run instability.',
    input_sha256={str(P):hashlib.sha256(P.read_bytes()).hexdigest()})
with (H/'independent_dlt.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
