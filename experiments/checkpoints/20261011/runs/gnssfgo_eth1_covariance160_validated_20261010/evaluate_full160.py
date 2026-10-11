"""Full-travel and matched-mask OFF/ON evaluation, no estimator/model changes.

Run in the existing ROS Noetic environment after both captures validate.
Uses the exact retained evaluator and body/extrinsic/alignment conventions.
"""
from pathlib import Path
import importlib.util
import json
import math
import numpy as np

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'prepared'
OFF=ROOT.parent/'gnssfgo_eth1_covariance160_20261010/prepared'
for mode in ['off','on']:
    base=OFF if mode=='off' else RUN
    assert json.loads((base/('full160_'+mode+'_validation.json')).read_text())['passed']
tool=ROOT.parent/'gnssfgo_eth1_active_window40_off_20261008/full_reference_evaluation_20261008/evaluate.py'
spec=importlib.util.spec_from_file_location('retained_full_eval',tool)
ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
out=ROOT/'evaluation';out.mkdir(exist_ok=False);ev.OUT=out
curves={};inputs={}
for mode in ['off','on']:
    base=OFF if mode=='off' else RUN
    source=base/('full160_'+mode+'_source');rec=base/('full160_'+mode+'_receiver/receiver')
    paths={'native_glim':source/'odom_imu.txt','native_kimera':source/'kimera_odometry.csv',
           'fused_online':rec/'online_trajectory.tum','fused_final_retrospective':rec/'trajectory.tum'}
    for role,path in paths.items():
        c=ev.relative.load_odometry_csv(path) if role=='native_kimera' else ev.relative.load_tum(path)
        assert np.all(np.diff(c[0])>0) and all(np.isfinite(a).all() for a in c)
        assert c[0][-1]-c[0][0]>=158
        curves[mode+'_'+role]=c;inputs[str(path)]=ev.sha(path)
rt,rp,rq,frames,children=ev.relative.load_reference_bag(ev.REFERENCE,ev.TOPIC)
assert children==['cpt7_imu']
reference=ev.relative.sort_unique(rt,rp,rq)
inputs[str(ev.REFERENCE)]=ev.sha(ev.REFERENCE);inputs[str(tool)]=ev.sha(tool)
# Grid anchored to the first observed IMU epoch, NOT selected on accuracy.
start=max(c[0][0] for c in curves.values());end=min(c[0][-1] for c in curves.values())
first=math.ceil((start-ev.FIRST_IMU)/.1)
last=math.floor((end-ev.FIRST_IMU)/.1)
candidate=ev.FIRST_IMU+np.arange(first,last+1)*.1
mask=np.array([all(ev.pose_at(c,t,.5) is not None for c in curves.values()) and
               ev.pose_at(reference,t,.05) is not None for t in candidate])
common=candidate[mask]
assert len(common)>1500 and common[-1]-common[0]>155
# Shared start/end physical intervals for1s RPE, rejecting holes on either end.
pairs=[(i,i+10) for i in range(len(candidate)-10) if mask[i] and mask[i+10]]
assert len(pairs)>1490
results={}
fixed=ev.relative.quaternion_to_matrix(ev.relative.CPT7_TO_PRISM_Q)
lever=ev.relative.CPT7_TO_PRISM_T
def at(c,t,gap):
    p,r=ev.pose_at(c,t,gap)
    return p+r@lever,r@fixed
for name,c in curves.items():
    poses=[(p,ev.relative.quaternion_to_matrix(q)) for p,q in zip(c[1],c[2])]
    v=dict(full=ev.score(name,'full',c[0],poses,reference),
           common=ev.score(name,'common',common,[ev.pose_at(c,t,.5) for t in common],reference))
    errors=[]
    for i,j in pairs:
        ep,er=ev.relative.relative_pose(at(c,candidate[i],.5),at(c,candidate[j],.5))
        gp,gr=ev.relative.relative_pose(at(reference,candidate[i],.05),at(reference,candidate[j],.05))
        errors.append([np.linalg.norm(ep-gp),math.degrees(math.acos(float(np.clip((np.trace(gr.T@er)-1)/2,-1,1))))])
    a=np.array(errors)
    v['common_RPE_1s']=dict(pairs=len(a),translation_RMSE_m=float(np.sqrt(np.mean(a[:,0]**2))),
                          rotation_RMSE_deg=float(np.sqrt(np.mean(a[:,1]**2))))
    results[name]=v
assert len({v['common']['timestamps_sha256'] for v in results.values()})==1
for path,digest in inputs.items():assert ev.sha(Path(path))==digest
summary=dict(passed=True,reference='Continuous IE-TC GNSS/INS; shares CPT7 with native sources; NOT independent prism GT',
    convention='Unchanged SE(3) unit-scale Kabsch and CPT7-to-prism lever on both sides; no extrapolation',
    inputs_sha256=inputs,results=results,
    common_mask=dict(rows=len(common),span_s=float(common[-1]-common[0]),candidate_rows=len(candidate),
       excluded_for_gaps=int((~mask).sum()),RPE_pairs=len(pairs),period_s=.1,
       rule='All eight saved trajectories and reference must support interpolation; <=.5s source gap/.05s reference gap'),
    full_coverage='Every original saved pose scored over158+seconds. Startup holes not filled.',
    online='Live concurrent fuser as published; final retrospective trajectory labeled separately.')
with (out/'summary.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
print(json.dumps({k:dict(ATE=v['full']['ATE_RMSE_m'],common_ATE=v['common']['ATE_RMSE_m'],
    duration=v['full']['timestamp_span_s'],path=v['full']['full_CPT7_polyline_path_m'],RPE=v['common_RPE_1s']) for k,v in results.items()},indent=2))
