"""FL4j: read retained Kimera diagnostics; never run an estimator or mutate inputs.

Output is JSON on stdout. Visual information here is a fresh diagnostic
linearization at calculateEstimate(), NOT the cached iSAM2 linear system.
"""
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/'
            'gnssfgo_eth1_fixed_lag_live120_fl4g_20261015')
SOURCE = ROOT / 'prepared/pilot120_on_source'
LOG, = (SOURCE / 'ros_logs').glob('*/*kimera*stdout.log')
DETAILS = ROOT / 'audit_fl4h/factor_details_v2.json'
TRACE = ROOT / 'audit_fl4h/mechanism_summary.json'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def stats(values):
    a = sorted(values)
    if not a:
        return None
    n = len(a)
    return {'n': n, 'mean': sum(a) / n,
            'median': (a[n // 2] + a[(n - 1) // 2]) / 2,
            'rms': math.sqrt(sum(x*x for x in a) / n), 'max': a[-1]}


before_hashes = {str(p): digest(p) for p in (LOG, DETAILS, TRACE)}
details = json.loads(DETAILS.read_text())
trace = json.loads(TRACE.read_text())
by_update = {row['update']: row for row in trace['on']['rows']}
visual = defaultdict(list)
update = None
for line_number, line in enumerate(LOG.open(), 1):
    if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,' in line:
        update = int(float(line.split(
            'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,')[1].split(',')[0]))
    if 'CBS_COVARIANCE_EXCHANGE_ACTIVE_FACTOR_ROW,G2K,' not in line:
        continue
    fields = line.split('CBS_COVARIANCE_EXCHANGE_ACTIVE_FACTOR_ROW,')[1].strip().split(',')
    assert len(fields) == 16
    if fields[9] != 'smart_stereo':
        continue
    assert update in by_update
    row = {'line': line_number, 'slot': int(fields[8]),
           'pose_key_count': len(fields[12].split('|')),
           'both_endpoints': fields[10] == '1' and fields[11] == '1',
           'error': float(fields[14]), 'endpoint_H_norm': float(fields[15])}
    assert math.isfinite(row['error']) and math.isfinite(row['endpoint_H_norm'])
    assert row['endpoint_H_norm'] >= 0
    visual[update].append(row)

samples = []
for update, factors in visual.items():
    assert len({r['slot'] for r in factors}) == len(factors)
    multi = [r for r in factors if r['pose_key_count'] > 1]
    both = [r for r in factors if r['both_endpoints']]
    zero = [r for r in multi if r['endpoint_H_norm'] == 0.0]
    assert all(r['error'] == 0.0 for r in zero)
    previous = details['sampled_existing_family_costs'][str(update)]['existing']
    prior = by_update[update]
    samples.append({
        'update': update, 'bag_clock_s': prior['time'],
        'touching_visual_count': len(factors), 'multiview_count': len(multi),
        'multiview_nonzero_H_count': len(multi)-len(zero),
        'both_endpoint_count': len(both),
        'both_endpoint_nonzero_H_count': sum(r['endpoint_H_norm'] > 0 for r in both),
        'zero_H_multiview_key_counts': dict(Counter(r['pose_key_count'] for r in zero)),
        'raw_log_line_start': factors[0]['line'],
        'raw_log_line_end': factors[-1]['line'],
        'existing_imu_factor_count': previous['imu']['count'],
        'existing_imu_error': previous['imu']['cost'],
        'existing_all_visual_count': previous['smart_stereo']['count'],
        'existing_all_visual_error': previous['smart_stereo']['cost'],
        'incoming_G_error_m': prior['aligned_error_m'],
        'K_target_error_before_m': prior['prior_injected_edge_error_m'],
        'K_target_error_after_m': prior['after_injected_edge_error_m'],
    })

dropped = {r['update'] for r in details['guards'] if 'staged_factor_dropped' in r['text']}
assert len(dropped) == 18
segments = []
for low, high in ((0, 30), (30, 75), (75, 90), (90, 100.5), (100.5, 121)):
    rows = [r for r in by_update.values() if low <= r['time'] < high
            and 'after_injected_edge_error_m' in r and r['update'] not in dropped]
    segments.append({
        'bag_clock_s': [low, high], 'retained_count': len(rows),
        'incoming_G_error_m': stats([r['aligned_error_m'] for r in rows]),
        'K_target_error_before_m': stats([r['prior_injected_edge_error_m'] for r in rows]),
        'K_target_error_after_m': stats([r['after_injected_edge_error_m'] for r in rows]),
        'target_error_increased_count': sum(r['after_injected_edge_error_m'] >
                                          r['prior_injected_edge_error_m'] for r in rows),
    })

after_hashes = {str(p): digest(p) for p in (LOG, DETAILS, TRACE)}
assert before_hashes == after_hashes
report = {
    'inputs_sha256': before_hashes,
    'limitations': [
        '30 sampled pre-insertion diagnostic states, not every update',
        'Endpoint diagonal Hessian norm freshly evaluated at current estimate; not solver cached Hessian',
        'Zero information does not identify triangulation failure reason or causal direction',
        'No matching OFF visual Hessian audit was recorded',
        'Existing target errors use correlated IE-TC reference, not independent prism truth',
        'Before/after target changes include the entire backend update, not an isolated CBS intervention',
    ],
    'visual_samples': samples, 'retained_target_error_segments': segments,
    'rollback_dropped_new_factors': len(dropped),
    'dropped_G_reference_error_m': stats([by_update[u]['aligned_error_m'] for u in dropped]),
    'frontend_stereo_warning_count': {m: len(trace[m]['stereo_failures']) for m in ('off','on')},
    'production_code_modified': False,
}
assert len(samples) == 30
print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
