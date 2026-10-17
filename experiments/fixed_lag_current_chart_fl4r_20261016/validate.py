"""Validate the sole94s extension without rerunning optimization or scoring GT."""
from pathlib import Path
import io, json, re
import numpy as np
import rosbag

R = Path(__file__).resolve().parent
D = R / 'remove_capture'
B = R.parent / 'fixed_lag_linear_parity_fl4n_20261015/remove_capture'
Q = R.parent / 'fixed_lag_current_chart_fl4q_20261016/remove_capture'
assert (D / 'complete.json').exists() and not (D / 'failed.txt').exists()
assert not (D / 'validation.json').exists(), 'refuse overwriting validation'
def messages(path, topic):
    with rosbag.Bag(str(path)) as bag:
        result = []
        for _, m, _ in bag.read_messages(topics=[topic]):
            s = io.BytesIO(); m.serialize(s)
            result.append((m.header.stamp.to_nsec() if hasattr(m, 'header') else None, s.getvalue()))
        return result
poses = messages(D / 'outputs.bag', '/kimera_vio_ros/odometry')
baseline = messages(B / 'outputs.bag', '/kimera_vio_ros/odometry')
short = messages(Q / 'outputs.bag', '/kimera_vio_ros/odometry')
assert len(poses) == 454, ('missing native output', len(poses))
assert len(short) == 134 and poses[:134] == short, '30s native prefix differs'
assert poses[:89] == baseline[:89] and poses[89] != baseline[89]
assert [p[0] for p in poses] == [p[0] for p in baseline], 'physical output grid differs'
incoming = messages(D / 'outputs.bag', '/kimera/cbs/odom_belief_in')
frozen = messages(R / 'aligned_inputs.bag', '/kimera/cbs/odom_belief_in')
assert len(incoming) == 450 and incoming == frozen
rows = json.loads((R / 'frozen_inputs.json').read_text())['rows']
lines = (D / 'kimera.log').read_text().splitlines()
bound = []
for line in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v = line.split('FL4M_FROZEN_FACTOR_ROW,', 1)[1].split(',')
        bound.append((list(map(int, v[:5])), [float(x) for x in v[5].split(';')],
                      [float(x) for x in re.split('[;:]', v[6])]))
assert len(bound) == len(incoming)
max_mean_error = 0.0
for (ids, mu, cov), r in zip(bound, rows):
    assert ids == [r['update']] + r['sender'] + r['receiver']
    error = float(np.max(np.abs(np.array(mu) - r['aligned'])))
    max_mean_error = max(max_mean_error, error)
    assert error < 1e-12 and cov == r['cov']
markers = ['FL4Q', 'active_window_rollback', 'Smoother recovery',
           'active_window_retry_local_update_', 'Persistent BPSAM recovery']
assert not any(marker in line for line in lines for marker in markers)
a = np.loadtxt(D / 'acceptance.csv', delimiter=',')
obj = np.loadtxt(D / 'objective.csv', delimiter=',')
qa = np.loadtxt(Q / 'acceptance.csv', delimiter=',')
qobj = np.loadtxt(Q / 'objective.csv', delimiter=',')
base_obj = np.loadtxt(B / 'objective.csv', delimiter=',')
assert len(a) == len(obj) == 454 and np.all(np.isfinite(a)) and np.all(np.isfinite(obj))
assert np.array_equal(a[:, 0], np.arange(454)), 'missing/duplicate update'
assert np.array_equal(obj[:134], qobj)
non_timing = [0,1,2,3,4,5,7,8]
assert np.array_equal(a[:134, non_timing], qa[:, non_timing]), '30s acceptance prefix differs'
assert np.array_equal(obj[:89], base_obj[:89])
fallback = a[a[:, 4] == 1]
assert len(fallback) > 0 and fallback[0, 0] == 89
assert np.all(a[:, 3] <= a[:, 1] + 1e-10 * (1 + np.abs(a[:, 1])))
assert np.all(fallback[:, 3] < fallback[:, 1]) and np.max(a[:, 7:9]) <= 1e-8
assert np.all(a[:, 5] <= 10)
assert int(obj[:, 2].sum()) == 426
def stats(x):
    return dict(median=float(np.median(x)), p95=float(np.percentile(x, 95)),
                maximum=float(np.max(x)), total=float(np.sum(x)))
report = dict(valid=True, native_poses=len(poses), native_span_s=(poses[-1][0]-poses[0][0])*1e-9,
    incoming=len(incoming), wire_payloads_exact=True, update_bindings_covariances_exact=True,
    maximum_mean_logging_error=max_mean_error, physical_output_grid_matches_baseline=True,
    all_134_short_run_native_messages_byte_identical=True, short_run_objectives_acceptance_exact=True,
    first_changed_baseline_native_update=89, all_objective_steps_nonincreasing=True,
    accepted_fallback_count=len(fallback), fallback_updates=fallback[:,0].astype(int).tolist(),
    fallback_rows=fallback.tolist(), fallback_iteration_cap_hits=int(np.sum(fallback[:,5] == 10)),
    decision_seconds=stats(a[:,6]), fallback_seconds=stats(fallback[:,6]),
    max_cache_fresh_error=float(np.max(a[:,7])), max_cache_tree_error=float(np.max(a[:,8])),
    planned_retirements=int(obj[:,2].sum()), update438_acceptance=a[438].tolist(),
    update438_baseline_objective=base_obj[438].tolist(),
    complete=json.loads((D/'complete.json').read_text()), accuracy_evaluated=False,
    stationarity_certified=False, production_promoted=False)
with (D / 'validation.json').open('x') as f:
    json.dump(report, f, indent=2)
print(json.dumps(report, indent=2))
