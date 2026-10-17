"""Read-only comparison of FL4t OFF and FL4v ON before the OFF capture failure.

No estimator execution. Reuse FL4v's exact evaluator functions and reference.
The principal comparison uses only native outputs and fusion as published live.
"""
import importlib.util
import json
import math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('frozen_evaluator', HERE/'prepared/evaluate_fl4f.py')
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)
roots = {
    'off': HERE.parent/'gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared',
    'on': HERE/'prepared',
}
assert (roots['off']/'live120_off_control/failed.txt').is_file()
assert (roots['on']/'live120_on_control/complete.json').is_file()
curves = {}
paths = {}
for mode, root in roots.items():
    src = root/f'live120_{mode}_source'
    rec = root/f'live120_{mode}_receiver/receiver'
    for name, path, loader in [
        ('native_glim', src/'odom_imu.txt', ev.rel.load_tum),
        ('native_kimera', src/'kimera_odometry.csv', ev.rel.load_odometry_csv),
        ('fused_online', rec/'online_trajectory.tum', ev.rel.load_tum),
    ]:
        key = mode+'_'+name
        curves[key] = ev.rel.sort_unique(*loader(path))
        paths[key] = path
hashes = {key:ev.sha(p) for key,p in paths.items()}
t,p,q,frames,children = ev.rel.load_reference_bag(ev.GT_BAG, ev.GT_TOPIC)
reference = ev.rel.sort_unique(t,p,q)
assert children == ['cpt7_imu']
start = max(c[0][0] for c in [*curves.values(),reference])
end = min(c[0][-1] for c in [*curves.values(),reference])
grid = np.arange(math.ceil(start*10)/10, math.floor(end*10)/10+1e-8, .1)
times = np.asarray([stamp for stamp in grid if
    ev.rel.interpolate_pose(*reference,stamp,.05) is not None and
    all(ev.rel.interpolate_pose(*c,stamp,.5) is not None for c in curves.values())])
assert len(times)>1000 and max(np.diff(times))<=.100001
results = {key:ev.score(c,reference,times) for key,c in curves.items()}
assert hashes == {key:ev.sha(p) for key,p in paths.items()}
comparison = {}
for name in ('native_glim','native_kimera','fused_online'):
    off,on = results['off_'+name],results['on_'+name]
    comparison[name] = {metric:{'off':off[metric],'on':on[metric],
        'change_percent':100*(on[metric]/off[metric]-1)} for metric in
        ('ATE_RMSE_m','translation_RPE_1s_RMSE_m','rotation_RPE_1s_RMSE_deg')}
print(json.dumps(dict(
    claim='Descriptive identical pre-stop interval; OFF failed, NOT completed120s OFF/ON validation',
    source_runs={k:str(v) for k,v in roots.items()},
    reference=dict(path=str(ev.GT_BAG),topic=ev.GT_TOPIC,sha256=ev.sha(ev.GT_BAG),
        caveat='IE-TC GNSS/INS shares CPT7; not independent prism GT'),
    scoring='Frozen score(): rigid SE3 no scale, prism lever arm, 1s RPE; identical 10Hz grid for all six outputs',
    common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),
        span_s=float(times[-1]-times[0]),
        timestamps_sha256=ev.hashlib.sha256(times.astype('<f8').tobytes()).hexdigest()),
    saved_coverage={k:dict(rows=len(c[0]),start_s=float(c[0][0]),end_s=float(c[0][-1])) for k,c in curves.items()},
    input_paths={k:str(v) for k,v in paths.items()},input_hashes=hashes,
    results=results,comparison=comparison,
    omitted='Delayed finalized fusion: unequal end-of-run future support; not substituted for live fusion.',
    estimators_rerun=False,inputs_unchanged=True),indent=2))
