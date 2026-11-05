"""Compare saved independent initialization arms; no trajectory accuracy score."""
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path
import numpy as np

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W / 'runs' / H.name
pins = {}


def pin(path):
    with path.open('rb') as stream:
        pins[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return path


def rows(path):
    with pin(path).open() as stream:
        return list(csv.DictReader(stream))


def vals(path):
    with pin(path).open() as stream:
        return {r[0]: np.array(r[1:], float) for r in csv.reader(stream)}


def rot(v):
    w, x, y, z = v[3:]
    assert abs(w*w + x*x + y*y + z*z - 1) < 1e-10
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def angle(r):
    sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]) / 2
    return float(np.degrees(np.arctan2(sine, (np.trace(r)-1)/2)))


def movement(a, b, keys=None):
    if keys is None:
        keys = [k for k in a if k in b and k.startswith('x')]
    mats = [rot(b[k]) @ rot(a[k]).T for k in keys]
    u, _, vt = np.linalg.svd(sum(mats))
    fix = np.eye(3)
    fix[2, 2] = np.linalg.det(u @ vt)
    rotation = u @ fix @ vt
    latest = max(keys, key=lambda k: int(k[1:]))
    return dict(poses=len(keys), common_world_rotation_matrix=rotation.tolist(),
                common_world_rotation_deg=angle(rotation),
                common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1)))),
                maximum_orientation_change_deg=max(angle(r) for r in mats),
                maximum_position_change_m=max(float(np.linalg.norm(b[k][:3]-a[k][:3])) for k in keys),
                latest_old_key=latest,
                latest_old_position_change_m=float(np.linalg.norm(b[latest][:3]-a[latest][:3])),
                latest_old_rotation_change_deg=angle(rot(b[latest]) @ rot(a[latest]).T),
                median_accel_bias_change_m_s2=float(np.median([np.linalg.norm(b['b'+k[1:]][:3]-a['b'+k[1:]][:3]) for k in keys])))


def cost(path):
    factors = rows(path)
    families = defaultdict(float)
    for row in factors:
        families[row['type']] += float(row['cost'])
    imu = [r for r in factors if r['type'] == 'imu']
    smart = {r['slot']: (int(r['valid']), int(r['status'])) for r in factors if r['type'] == 'smart'}
    return dict(total=sum(families.values()), families=dict(families),
                native=sum(v for k, v in families.items() if k != 'peer_pose'),
                factors=len(factors), smart_factors=len(smart),
                cost_evaluated_valid_smart=sum(s[0] for s in smart.values()),
                max_imu_position_m=max(float(r['imu_position_m']) for r in imu),
                max_imu_rotation_deg=max(float(r['imu_rotation_deg']) for r in imu),
                max_imu_velocity_m_s=max(float(r['imu_velocity_m_s']) for r in imu)), smart


# Independent analytic identity/rotation checks for the reporting geometry.
assert np.array_equal(rot(np.array([0, 0, 0, 1, 0, 0, 0.])), np.eye(3))
for deg in (0.01, 1, 10, 45, 100):
    v = np.array([0, 0, 0, np.cos(np.radians(deg)/2), np.sin(np.radians(deg)/2), 0, 0])
    assert abs(angle(rot(v))-deg) < 1e-12

parity = rows(P / 'results/parity.csv')
seed = rows(P / 'results/seed.csv')
steps = rows(P / 'results/steps.csv')
assert len(parity) == len(seed) == 5 and len(steps) == 10
events = []
for k in range(21, 26):
    root = P / 'results' / f'update_{k}'
    previous = vals(root / 'previous_values.csv')
    keys = [key for key in previous if key.startswith('x')]
    states = {arm: {stage: vals(root / f'{arm}_{stage}_values.csv')
                    for stage in ('initial', 'ordinary', 'refresh')} for arm in 'AB'}
    a, b = states['A']['initial'], states['B']['initial']
    assert a.keys() == b.keys()
    for key in a:
        assert np.array_equal(a[key][:3] if key == f'x{k}' else a[key],
                              b[key][:3] if key == f'x{k}' else b[key]), key
    assert angle(rot(a[f'x{k}']) @ rot(b[f'x{k}']).T) > 0.01
    for key in previous:
        assert np.allclose(previous[key], a[key], atol=1e-15, rtol=0), key
    arms = {}
    activity = {}
    for arm in 'AB':
        costs = {}
        activity[arm] = {}
        for stage in ('initial', 'ordinary', 'refresh'):
            costs[stage], activity[arm][stage] = cost(root / f'{arm}_{stage}_factors.csv')
        arms[arm] = dict(costs=costs,
                         ordinary=movement(previous, states[arm]['ordinary']),
                         refresh=movement(states[arm]['ordinary'], states[arm]['refresh']),
                         net=movement(previous, states[arm]['refresh']))
    # Same topology/factor count in both arms at all cost-evaluation points.
    assert len({c['factors'] for arm in arms.values() for c in arm['costs'].values()}) == 1
    assert len({c['smart_factors'] for arm in arms.values() for c in arm['costs'].values()}) == 1
    comparison = {stage: movement(states['A'][stage], states['B'][stage], keys)
                  for stage in ('ordinary', 'refresh')}
    activity_changes = {stage: sum(activity['A'][stage][slot] != status
                                   for slot, status in activity['B'][stage].items())
                        for stage in ('initial', 'ordinary', 'refresh')}
    capture = {}
    for stage in ('ordinary', 'refresh'):
        capture[stage] = movement(vals(root / f'captured_{stage}_values.csv'), states['A'][stage], keys)
        assert capture[stage]['maximum_orientation_change_deg'] < 1e-5
        assert capture[stage]['maximum_position_change_m'] < 1e-6
    original = arms['A']['net']['common_world_tilt_deg']
    candidate = arms['B']['net']['common_world_tilt_deg']
    events.append(dict(update=k, arms=arms, B_vs_A=comparison,
                       cost_evaluated_smart_status_differences=activity_changes,
                       captured_vs_A=capture,
                       net_tilt_reduction_percent=100*(original-candidate)/original,
                       seed={name: float(v) for name, v in next(r for r in seed if int(r['update']) == k).items() if name != 'update'},
                       parity={name: float(v) for name, v in next(r for r in parity if int(r['update']) == k).items() if name != 'update'}))
    print(k, 'net tilt A/B', original, candidate,
          'ordinary A/B', arms['A']['ordinary']['common_world_tilt_deg'], arms['B']['ordinary']['common_world_tilt_deg'],
          'final IMU A/B', arms['A']['costs']['refresh']['families']['imu'], arms['B']['costs']['refresh']['families']['imu'])
severe = [e for e in events if e['arms']['A']['net']['common_world_tilt_deg'] > 3]
screen = all(e['net_tilt_reduction_percent'] >= 50 for e in severe) and all(
    e['arms']['B']['net']['common_world_tilt_deg'] <= 3 for e in events if e not in severe)
out = dict(complete=True, independent_checkpoints=5, experiment_native_GN_steps=20,
           source_replays=0, covariance_analysis=False, accuracy_scored=False,
           practical_screen=dict(passed=screen, severe_original_updates=[e['update'] for e in severe],
                                  rule='All original net tilt>3deg windows reduce at least50%, no new>3deg window.'),
           updates=events, input_sha256=pins,
           limitations=['Independent ON-conditioned windows, not a sequence replay or OFF comparison.',
                        'No frontend rematching, raw IMU reintegration, covariance change or new accuracy score.',
                        'Smart status is evaluated on cost-reporting clones, not claimed to be solver Gaussian activity.',
                        'Two reproduced native GN steps are not a nonlinear stationarity result.'])
with (H / 'analysis.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print('Practical screen', screen)
