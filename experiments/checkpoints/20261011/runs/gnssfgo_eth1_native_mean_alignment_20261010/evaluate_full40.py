"""Full saved-output and identical-mask scores, using retained IE-TC evaluator."""
from pathlib import Path
import importlib.util
import json
import math
import sys
import numpy as np

ROOT=Path(__file__).resolve().parent
RUNS=ROOT.parent
OLD_OFF=RUNS/'gnssfgo_eth1_active_window40_off_20261008'
OLD_ON=RUNS/'gnssfgo_eth1_active_window40_prepared_20261008'
NEW=ROOT/'arrival_boundary'
assert json.loads((NEW/'pilot_on_validation.json').read_text())['passed']
assert json.loads((NEW/'pilot_on_native_validation.json').read_text())['passed']
spec=importlib.util.spec_from_file_location('retained_full_eval',OLD_OFF/'full_reference_evaluation_20261008/evaluate.py')
ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
out=ROOT/'full40_evaluation';out.mkdir(exist_ok=False);ev.OUT=out
curves={}; inputs={}
for name,base,mode in [('off',OLD_OFF,'off'),('raw_on',OLD_ON,'on'),('aligned_on',NEW,'on')]:
    source=base/f'pilot_{mode}_source'; rec=base/f'pilot_{mode}_receiver/receiver'
    paths={'native_glim':source/'odom_imu.txt','native_kimera':source/'kimera_odometry.csv',
           'fused_online':rec/'online_trajectory.tum','fused_final':rec/'trajectory.tum'}
    for role,path in paths.items():
        key=name+'_'+role
        curves[key]=ev.relative.load_odometry_csv(path) if role=='native_kimera' else ev.relative.load_tum(path)
        assert np.all(np.diff(curves[key][0])>0)
        inputs[str(path)]=ev.sha(path)
reference=ev.relative.load_reference_bag(ev.REFERENCE,ev.TOPIC)
rt,rp,rq,frames,children=reference
assert children==['cpt7_imu']
reference=ev.relative.sort_unique(rt,rp,rq)
common=np.loadtxt(OLD_OFF/'comparison/common_grid/off_native_glim.tum')[:,0]
results={}
for name,curve in curves.items():
    poses=[(p,ev.relative.quaternion_to_matrix(q)) for p,q in zip(curve[1],curve[2])]
    common_poses=[ev.pose_at(curve,t,.5) for t in common]
    assert all(p is not None for p in common_poses)
    assert curve[0][-1]-curve[0][0]>38.5
    results[name]=dict(full=ev.score(name,'full',curve[0],poses,reference),
                       common=ev.score(name,'common',common,common_poses,reference))
    # Existing evaluator's one-second, body-relative error with the SAME
    # rigid CPT7-to-prism transform on estimate and reference, no global fit.
    errs=[]
    fixed=ev.relative.quaternion_to_matrix(ev.relative.CPT7_TO_PRISM_Q)
    lever=ev.relative.CPT7_TO_PRISM_T
    def at(curve,t,gap):
        p=ev.pose_at(curve,t,gap)
        assert p is not None
        return p[0]+p[1]@lever,p[1]@fixed
    for t in common:
        if t+1>common[-1]+1e-7:continue
        ep,er=ev.relative.relative_pose(at(curve,t,.5),at(curve,t+1,.5))
        gp,gr=ev.relative.relative_pose(at(reference,t,.05),at(reference,t+1,.05))
        errs.append([np.linalg.norm(ep-gp),math.degrees(math.acos(float(np.clip((np.trace(gr.T@er)-1)/2,-1,1))))])
    assert len(errs)>=350
    a=np.array(errs)
    results[name]['common_RPE_1s']=dict(pairs=len(a),translation_RMSE_m=float(np.sqrt(np.mean(a[:,0]**2))),
        rotation_RMSE_deg=float(np.sqrt(np.mean(a[:,1]**2))))
for path,digest in inputs.items():assert ev.sha(Path(path))==digest
# Frozen controls must reproduce their retained all-pose ATE exactly.
old=json.loads((OLD_OFF/'full_reference_evaluation_20261008/summary.json').read_text())
for key,v in results.items():
    if key.startswith('aligned_on_'):continue
    oldkey=key.replace('raw_on_','on_')
    assert abs(v['full']['ATE_RMSE_m']-old['results'][oldkey]['full_saved_output']['ATE_RMSE_m'])<1e-12
report=dict(passed=True,reference='Continuous IE-TC GNSS/INS, shares CPT7; NOT independent prism GT',
    old_controls='Frozen OFF and raw-mean ON; NOT rerun or retuned. New ON changes only mean alignment.',
    inputs_sha256=inputs,results=results,
    full_coverage='All original saved poses over 38.5+ seconds scored; startup gaps not filled.',
    common_mask='361 identical timestamps, retained 36-second mask after startup; 1s RPE uses 351 intervals.')
with (out/'summary.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:dict(ATE=v['full']['ATE_RMSE_m'],duration=v['full']['timestamp_span_s'],
                        path=v['full']['full_CPT7_polyline_path_m'],RPE=v['common_RPE_1s']) for k,v in results.items()},indent=2))
