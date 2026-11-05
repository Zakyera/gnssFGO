"""Saved-only first-factor state response; no optimizer or trajectory scoring."""
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

H = Path(__file__).resolve().parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs') / H.name
D = P / 'attempt_compile1/results'
pins = {}

def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def pin(p):
    pins[str(p)] = sha(p)
    return p

def rows(p):
    with pin(p).open() as f:
        return list(csv.DictReader(f))

def vals(p):
    with pin(p).open() as f:
        return {r[0]: np.array(r[1:], float) for r in csv.reader(f)}

def rot(v):
    w, x, y, z = v[3:]
    assert abs(w*w + x*x + y*y + z*z - 1) < 1e-10
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def angle(r):
    sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]) / 2
    return float(np.degrees(np.arctan2(sine, (np.trace(r)-1)/2)))

def movement(a, b, keys):
    mats = [rot(b[k]) @ rot(a[k]).T for k in keys]
    u, _, vt = np.linalg.svd(sum(mats))
    fix = np.eye(3); fix[2, 2] = np.linalg.det(u @ vt)
    rotation = u @ fix @ vt
    detail = {}
    for k, rr in zip(keys, mats):
        num = k[1:]
        detail[k] = dict(position_change_m=float(np.linalg.norm(b[k][:3]-a[k][:3])),
                         rotation_change_deg=angle(rr),
                         velocity_change_m_s=float(np.linalg.norm(b['v'+num]-a['v'+num])),
                         accel_bias_change_m_s2=float(np.linalg.norm(b['b'+num][:3]-a['b'+num][:3])),
                         gyro_bias_change_rad_s=float(np.linalg.norm(b['b'+num][3:]-a['b'+num][3:])),
                         position_delta=(b[k][:3]-a[k][:3]).tolist(),
                         velocity_delta=(b['v'+num]-a['v'+num]).tolist(),
                         accel_bias_delta=(b['b'+num][:3]-a['b'+num][:3]).tolist(),
                         gyro_bias_delta=(b['b'+num][3:]-a['b'+num][3:]).tolist())
    return dict(common_world_rotation_matrix=rotation.tolist(), common_world_rotation_deg=angle(rotation),
                common_world_tilt_deg=float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1)))),
                maxima={k: max(d[k] for d in detail.values()) for k in
                        ('position_change_m', 'rotation_change_deg', 'velocity_change_m_s',
                         'accel_bias_change_m_s2', 'gyro_bias_change_rad_s')}, per_state=detail)

def cost(label):
    data = rows(D / (label + '_factors.csv'))
    families = defaultdict(float)
    for r in data:
        families[r['type']] += float(r['cost'])
    return dict(families=dict(families), native=sum(v for k, v in families.items() if k != 'CBS'),
                CBS_diagnostic=families['CBS'], smart_count=sum(r['type']=='smart' for r in data),
                smart_valid=sum(r['type']=='smart' and r['valid']=='1' for r in data),
                max_imu_rotation_residual_deg=max(float(r['imu_rotation_deg']) for r in data),
                max_imu_position_residual_m=max(float(r['imu_position_m']) for r in data),
                max_imu_velocity_residual_m_s=max(float(r['imu_velocity_m_s']) for r in data))

# Independent rotation-reporting fixtures.
for deg in (0.01, 1, 10, 45, 100):
    v = np.array([0, 0, 0, np.cos(np.radians(deg)/2), np.sin(np.radians(deg)/2), 0, 0])
    assert abs(angle(rot(v))-deg) < 1e-12
assert json.loads(pin(H / 'attempt_compile1/test_passed.json').read_text())['passed']
parity = {k: float(v) for k, v in rows(D / 'parity.csv')[0].items()}
assert parity['ordinary_equation_relative'] < 1e-12
assert parity['ordinary_captured_tangent'] < 1e-6
assert parity['exact_refresh_equation_relative'] < 1e-12
assert parity['chained_refresh_captured_tangent'] < 1e-5
previous = vals(D / 'previous_values.csv')
initial = vals(D / 'initial_values.csv')
off_initial = vals(D / 'OFF_initial_values.csv')
oldkeys = [k for k in previous if k.startswith('x')]
allkeys = [k for k in initial if k.startswith('x')]
assert set(oldkeys) == {f'x{k}' for k in range(6)}
for key in previous:
    assert np.allclose(previous[key], initial[key], atol=1e-15, rtol=0), key
states = {label: vals(D / (label + '_values.csv')) for label in
          ('A_ordinary', 'A_refresh', 'B_ordinary', 'B_refresh', 'OFF_refresh')}
signatures = {}
for mode in ('ON', 'OFF'):
    signatures[mode] = {p.stem: sha(pin(p)) for p in sorted((D / (mode+'_functions')).glob('*.bin'))}
on_native = Counter(v for k, v in signatures['ON'].items() if k != '929')
off_native = Counter(signatures['OFF'].values())
function_check = dict(ON_total=len(signatures['ON']), OFF_total=len(signatures['OFF']),
                      native_functions_equal=on_native == off_native,
                      ON_native_only=sum((on_native-off_native).values()),
                      OFF_native_only=sum((off_native-on_native).values()),
                      smart_cache_excluded=True)
residuals = {}
for r in rows(D / 'residuals.csv'):
    label = r.pop('state'); d = {k: float(v) for k, v in r.items()}
    rotation = np.array([d[k] for k in ('rx_rad', 'ry_rad', 'rz_rad')])
    translation = np.array([d[k] for k in ('tx_m', 'ty_m', 'tz_m')])
    residuals[label] = dict(**d, rotation_components_deg=np.degrees(rotation).tolist(),
                           rotation_norm_deg=float(np.degrees(np.linalg.norm(rotation))),
                           translation_components_mm=(1000*translation).tolist(),
                           translation_norm_m=float(np.linalg.norm(translation)))
arms = {}
for arm in ('A', 'B'):
    arms[arm] = dict(ordinary=movement(previous, states[arm+'_ordinary'], oldkeys),
                     refresh=movement(states[arm+'_ordinary'], states[arm+'_refresh'], oldkeys),
                     net=movement(previous, states[arm+'_refresh'], oldkeys),
                     final_costs=cost(arm+'_refresh'))
comparisons = {label: movement(states[a], states[b], allkeys) for label, a, b in
               [('A_vs_B', 'A_refresh', 'B_refresh'), ('B_vs_actual_OFF', 'B_refresh', 'OFF_refresh'),
                ('A_vs_actual_OFF', 'A_refresh', 'OFF_refresh')]}
for phase in ('ordinary', 'refresh'):
    assert sha(pin(D / (phase+'_before.bin'))) == sha(pin(D / (phase+'_after.bin')))
assert sha(pin(D / 'B_target_restored.bin')) == sha(D / 'ordinary_before.bin')
analysis = dict(update=6, capture_time_sec=1.599933183, source_replays=0, native_GN_steps=4,
                accuracy_scored=False, covariance_analysis=False, root_cause_proved=False,
                parity=parity, native_function_comparison=function_check,
                residuals=residuals, arms=arms, comparisons=comparisons,
                initial_costs=cost('initial'), actual_OFF_costs_on_shared_graph=cost('OFF_refresh'),
                OFF_initial_max_numeric_difference=max(float(np.max(abs(initial[k]-off_initial[k]))) for k in initial),
                steps=rows(D / 'steps.csv'), input_sha256=pins,
                caveats=['B CBS cost is diagnostic: that factor was NOT in the B solve.',
                         'Native function multiset comparison excludes smart triangulation caches.',
                         'First update only: no claim of later full-run root cause or nonlinear stationarity.',
                         'Tilt/bias changes are revisions, not errors relative to ground truth.'])
with (H / 'analysis.json').open('x') as f:
    json.dump(analysis, f, indent=2, allow_nan=False)
print(json.dumps({k:v for k,v in analysis.items() if k not in ('input_sha256','arms','comparisons')}, indent=2))
for arm, d in arms.items():
    print(arm, 'tilt', d['net']['common_world_tilt_deg'], 'maxima', d['net']['maxima'], 'cost', d['final_costs']['families'])
for name, d in comparisons.items():
    print(name, d['maxima'])
