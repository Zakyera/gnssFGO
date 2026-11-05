"""Summarize existing checkpoint diagnostics only; never runs an optimizer."""
import csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
old=P.parent/'eig2_kimera_off_cpv35_20261029/capture/snapshots'
assert json.loads((H/'capture_verification.json').read_text())['passed']
finished=json.loads((H/'offline_finished.json').read_text())
costs=np.loadtxt(P/'capture/objective.csv',delimiter=',')
def kv(p):return dict(l.split(',',1) for l in p.read_text().splitlines())
def states(p):return {r[0]:np.array(r[1:],float) for r in csv.reader(p.open())}
results=[]
for receipt in finished['results']:
    k=receipt['update'];root=P/'offline'/('update_'+str(k))
    row=dict(update=k,receipt=receipt,equations={a:float(b) for a,b in kv(root/'equations.txt').items()},starts={})
    for name,ix in [('initialized',4),('incremental_result',5)]:
        d=root/name;failure=root/(name+'_failed.txt');r={}
        if failure.exists():r['failure']=failure.read_text().strip()
        if (d/'model.txt').exists():
            r['model']=kv(d/'model.txt');actual=float(r['model']['native_initial_cost']);expected=costs[k,ix]
            r['native_initial_cost_matches_capture']=abs(actual-expected)<1e-5+1e-10*abs(expected)
            r['native_initial_cost_absolute_difference']=abs(actual-expected)
            assert r['native_initial_cost_matches_capture'],(k,name,actual,expected)
        r['derivatives']={}
        for kind in ('native','joint'):
            f=d/(kind+'_derivatives.csv')
            if f.exists():
                data=list(csv.DictReader(f.open()));bad=[x for x in data if x['pass']=='0']
                r['derivatives'][kind]=dict(tested=len(data),failed=len(bad),failed_factor_ids=sorted({int(x['factor']) for x in bad}),max_absolute_difference=max(float(x['absolute_difference']) for x in data))
        if (d/'iterations.csv').exists():
            trace=[{a:float(b) for a,b in v.items()} for v in csv.DictReader((d/'iterations.csv').open())]
            assert trace
            for a,b in zip(trace,trace[1:]):assert b['cost']<=a['cost']+1e-8*(1+abs(a['cost']))
            r['initial']=trace[0];r['final']=trace[-1];r['iterations']=len(trace)-1
            expected_imu=costs[k,6 if ix==4 else 7]
            assert abs(trace[0]['imu_cost']-expected_imu)<1e-5+1e-10*abs(expected_imu)
            r['cost_reduction_ratio']=trace[0]['cost']/trace[-1]['cost'] if trace[-1]['cost'] else None
        baseline=old/('update_'+str(k))/'after_solve/state.csv'
        if baseline.exists() and (d/'final_values.csv').exists():
            a=states(baseline);b=states(d/'final_values.csv');keys=[q for q in a if q.startswith('x') and q in b]
            dp=[np.linalg.norm(b[q][:3]-a[q][:3]) for q in keys]
            angles=[np.linalg.norm((Rotation.from_quat(a[q][[4,5,6,3]]).inv()*Rotation.from_quat(b[q][[4,5,6,3]])).as_rotvec())*180/np.pi for q in keys]
            latest=max(keys,key=lambda q:int(q[1:]));r['movement_from_saved_incremental_result']=dict(pose_count=len(keys),translation_max_m=max(dp),rotation_max_deg=max(angles),latest_key=latest,latest_translation_m=float(np.linalg.norm(b[latest][:3]-a[latest][:3])))
        row['starts'][name]=r
    results.append(row)
out=dict(checkpoints=results,accuracy_metrics=False,live_fix=False,conditional_joint_landmarks=True)
encoded=json.dumps(out,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else str(x))
with (H/'diagnostic_results_v2.json').open('x') as f:f.write(encoded)
for row in results:
    print('UPDATE',row['update'],'equations',row['equations'])
    for name,r in row['starts'].items():
        print(name,json.dumps(r,default=lambda x:x.item() if isinstance(x,np.generic) else str(x)))
