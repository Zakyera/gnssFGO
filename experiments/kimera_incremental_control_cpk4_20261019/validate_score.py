"""Validate the one CP-K4b capture and score saved native odometry, not fusion."""
import hashlib
import importlib.util
import io
import json
import math
import sys
from pathlib import Path
import numpy as np
import rosbag
import yaml

R = Path(__file__).resolve().parent
W = Path('/workspace/cbs_gtsam4.3')
K = W/'runs/kimera_failed_factor_cpk1_20261019'
D = R/'off_capture'

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()

def messages(path):
    out = []
    with rosbag.Bag(str(path)) as bag:
        for _, m, _ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):
            data = io.BytesIO()
            m.serialize(data)
            p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, m.twist.twist.angular
            nums = [p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,
                    *m.pose.covariance,*m.twist.covariance]
            assert np.isfinite(nums).all(), 'Nonfinite native output'
            out.append(dict(ns=m.header.stamp.to_nsec(), data=data.getvalue(), p=[p.x,p.y,p.z],
                            q=[q.x,q.y,q.z,q.w], frame=m.header.frame_id, child=m.child_frame_id))
    assert len(out)>44
    assert all(b['ns']>a['ns'] for a,b in zip(out,out[1:])), 'Nonmonotonic output'
    return out

def curve(rows):
    return (np.asarray([r['ns']*1e-9 for r in rows]), np.asarray([r['p'] for r in rows]),
            np.asarray([r['q'] for r in rows]))

assert not (D/'validation.json').exists() and not (D/'evaluation.json').exists(), 'Refuse overwrite'
assert json.loads((D/'capture_complete.json').read_text())['passed']
assert json.loads((D/'cleanup.json').read_text())['kimera_process_gone']
assert not (D/'failed.txt').exists() and not (D/'outputs.bag.active').exists()
assert not (D/'acceptance.csv').exists(), 'Disabled FL4q must not execute'
preflight = json.loads((D/'preflight.json').read_text())
assert preflight['passed'] and preflight['selected_env']['FL4Q_ENABLE']=='0'
params = yaml.safe_load((D/'resolved_params.yaml').read_text())
assert params['cbs_belief_receive_enable'] is False and params['cbs_belief_publish_enable'] is True
assert preflight['flags']['cbs_live_revision_enable']=='false'
assert preflight['flags']['cbs_odom_factor_mode']=='active_window_temporary'
manifest = json.loads((R/'pre_capture_receipt.json').read_text())['manifest']
for p,h in manifest.items():
    assert sha(p.replace('/home/yeranis/repos/V4RL','/workspace'))==h, p
protected = json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():
    # gnssFGO/results roots are host-only and checked separately on the host.
    if p.startswith('/home/yeranis/repos/V4RL/'):
        assert sha(p.replace('/home/yeranis/repos/V4RL','/workspace'))==h, p

actual, old = messages(D/'outputs.bag'), messages(K/'off_capture/outputs.bag')
a, b = [np.loadtxt(p,delimiter=',') for p in (D/'objective.csv',K/'off_capture/objective.csv')]
assert np.isfinite(a).all() and a.shape[1]==10
assert np.array_equal(a[:44],b[:44]), 'Objective prefix is not identical'
assert all(x['data']==y['data'] for x,y in zip(actual[:44],old[:44])), 'Native prefix changed before intervention'
assert np.all(a[:,1:3]==0), 'Unexpected retain setting or CBS retirements'
assert np.array_equal(a[:,0],np.arange(len(a))), 'Nonconsecutive objective update IDs'
assert len(a)==len(actual)>531, 'Capture did not pass old failing update'
assert actual[-1]['ns']>old[-1]['ns'], 'Capture did not pass old physical endpoint'
first_native = next((i for i,(x,y) in enumerate(zip(actual,old)) if x['data']!=y['data']),None)
first_objective = next((i for i,(x,y) in enumerate(zip(a,b)) if not np.array_equal(x,y)),None)
log = (D/'kimera.log').read_text()
bad = [s for s in ('FL4Q no decreasing','active_window_rollback','Smoother recovery',
                  'Persistent BPSAM recovery','VioBackend::optimize failed','terminate called',
                  'Segmentation fault','FL4m prior chart changed') if s in log]
assert not bad, bad
assert {m['child'] for m in actual}=={'cpt7_imu'}
qnorm = max(abs(np.linalg.norm(m['q'])-1) for m in actual)
assert qnorm<1e-10
before_after = a[:,4:6]
increases = np.flatnonzero(before_after[:,1]>before_after[:,0]+1e-10)
checkpoints = {str(i):dict(update=int(a[i,0]),before=float(a[i,4]),after=float(a[i,5]),
                         imu_before=float(a[i,6]),imu_after=float(a[i,7]),
                         prior_before=float(a[i,8]),prior_after=float(a[i,9])) for i in (44,531,len(a)-1)}
play = json.loads((D/'started.json').read_text())['play_args']
bag_paths = [Path(p) for p in play if p.endswith('.bag')]
starts=[]
for path in bag_paths:
    with rosbag.Bag(str(path)) as bag: starts.append(bag.get_start_time())
window_start = min(starts)
time_steps=np.diff([m['ns'] for m in actual])*1e-9
validation=dict(passed=True,source_playbacks=1,source_algorithm_changed_only='FL4Q_ENABLE=0',
    count=len(actual),reference_count=len(old),objectives=len(a),native_prefix_44_byte_identical=True,
    objective_prefix_44_exact=True,first_native_difference=first_native,first_objective_difference=first_objective,
    observed_cost_increases=len(increases),cost_increase_update_ids=increases.tolist(),checkpoints=checkpoints,
    fl4q_calls=0,cbs_retirements=0,revision_log_rows=log.count('KIMERA_CBS_REVISION_EDGE_ROW,'),
    finite_native_pose_twist_covariance=True,max_quaternion_norm_error=float(qnorm),
    monotonic_native_timestamps=True,max_native_gap_s=float(time_steps.max()),
    nominal_playback_start_s=window_start,nominal_playback_end_s=window_start+111,
    native_start_gap_s=actual[0]['ns']*1e-9-window_start,
    native_end_gap_s=window_start+111-actual[-1]['ns']*1e-9,
    exact_input_manifest_unchanged=True,checkpoints_are_observational_not_convergence=True,
    capture=json.loads((D/'capture_complete.json').read_text()),
    output_bag_sha256=sha(D/'outputs.bag'),reference_bag_sha256=sha(K/'off_capture/outputs.bag'))
with (D/'validation.json').open('x') as f: json.dump(validation,f,indent=2,allow_nan=False)
print(json.dumps(validation,indent=2),flush=True)

# Use existing full-saved and same-grid scoring functions without modifications.
evaluator = W/'runs/gnssfgo_eth1_local_cov_full_cpr7_20261019/prepared/evaluate_fl4f.py'
spec = importlib.util.spec_from_file_location('existing_evaluation',evaluator)
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)
t,p,q,frames,children = ev.rel.load_reference_bag(ev.GT_BAG,ev.GT_TOPIC)
assert children==['cpt7_imu']
reference=ev.rel.sort_unique(t,p,q)
curves={'ordinary_incremental':curve(actual),'saved_FL4q_failed_prefix':curve(old)}
start=max(c[0][0] for c in [*curves.values(),reference])
end=min(c[0][-1] for c in [*curves.values(),reference])
grid=np.arange(math.ceil(start*10)/10,math.floor(end*10)/10+1e-8,.1)
times=np.asarray([s for s in grid if ev.rel.interpolate_pose(*reference,s,.05) is not None and
                  all(ev.rel.interpolate_pose(*c,s,.5) is not None for c in curves.values())])
assert len(times)>500
metrics={name:dict(full_saved=ev.score_saved(c,reference),common=ev.score(c,reference,times),
                  full_saved_path_length_CPT7_m=ev.path_length(c[1])) for name,c in curves.items()}
out=dict(reference=dict(path=str(ev.GT_BAG),topic=ev.GT_TOPIC,sha256=sha(ev.GT_BAG),
    note='IE-TC shares CPT7 IMU; not independent prism ground truth.'),
    evaluation_code_sha256=sha(evaluator),metrics=metrics,
    common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),
        span_s=float(times[-1]-times[0]),excluded_grid_rows=len(grid)-len(times),
        max_gap_s=float(np.diff(times).max()),sha256=hashlib.sha256(times.astype('<f8').tobytes()).hexdigest()),
    convention='Original native ROS odometry, CPT7-to-prism lever arm, per-path SE3/no-scale ATE; 1s RPE.',
    scope='OFF backend diagnostic only, not a new CBS-OFF/ON or fusion comparison.')
with (D/'evaluation.json').open('x') as f: json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps(out,indent=2),flush=True)
