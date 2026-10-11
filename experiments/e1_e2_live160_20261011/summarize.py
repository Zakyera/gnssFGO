"""Summarize the one validated capture and retained timing evidence, not accuracy."""
import csv
import importlib.util
import json
from pathlib import Path

WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS / 'runs/gnssfgo_e1_e2_live160_20261011'
ROOT = RUN / 'prepared'
TAG = 'live160_on'
T0 = 1727775008.003475
WINDOWS = [(0, 160), (0, 40), (40, 80), (80, 120), (120, 160)]
spec = importlib.util.spec_from_file_location(
    'saved_timing', WS / 'runs/glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)
stats = timing.stats


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def source_stats(data, ids):
    scan = {key: stats([data['scan'][i][key] for i in ids]) for key in
            ['pre_callback', 'smoother', 'post_callback', 'total', 'nfactors', 'active']}
    nested = {}
    for kind, columns in {
            'on_smoother_update': {'sidecar_update': 10},
            'publish_outgoing': {'local_covariance_rebuild': 17,
                                 'belief_query': 18, 'ros_publish': 20},
            'on_smoother_update_finish': {'outgoing_plus_revisions': 12}}.items():
        found = [data['cbs'][kind][i][0] for i in ids if i in data['cbs'].get(kind, {})]
        nested.update({key: stats([r[col] for r in found]) for key, col in columns.items()})
    pub = data['cbs'].get('publish_odometry', {})
    ages = stats([pub[i][1] - data['scan'][i]['stamp'] for i in ids if i in pub])
    return {'scan_ms': scan, 'nested_ms': nested, 'native_publication_age_s': ages,
            'scans_over_1000ms': sum(data['scan'][i]['total'] > 1000 for i in ids)}


def receiver_stats(root, tag, lo, hi):
    samples = [json.loads(line) for line in (root / (tag + '_control/monitor.jsonl')).open()]
    samples = [r for r in samples if r['published_poses'] > 0 and
               lo <= r['last_imu_ns'] * 1e-9 - T0 < hi]
    publications = [r for r in rows(root / (tag + '_receiver/receiver/publication_convergence.csv'))
                    if lo <= float(r['physical_ns']) * 1e-9 - T0 < hi]
    return {'sampled_queue': stats([r['queued'] for r in samples]),
            'transport_peak_to_end_of_window': max((r['max_queue'] for r in samples), default=None),
            'output_age_simulated_s': stats([r['output_age_seconds'] for r in samples]),
            'refinement_ms': stats([1000 * float(r['seconds']) for r in publications]),
            'publication_gradient': stats([float(r['gradient_after']) for r in publications]),
            'publication_count': len(publications)}


validation = json.loads((ROOT / (TAG + '_validation.json')).read_text())
assert validation['passed']
candidate = timing.read(ROOT / (TAG + '_source'))
windows = []
for lo, hi in WINDOWS:
    ids = [i for i, r in candidate['scan'].items() if lo <= r['stamp'] - T0 < hi]
    assert ids
    windows.append({'seconds': [lo, hi], 'scans': len(ids),
                    'source': source_stats(candidate, ids),
                    'receiver': receiver_stats(ROOT, TAG, lo, hi)})

comparisons = []
for name, root, tag, requested_end in [
        ('original_failed160', WS / 'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared',
         'full160_on', 160),
        ('passed_e1_e2_60', WS / 'runs/gnssfgo_e1_e2_live60_20261011/prepared', 'live60_on', 60)]:
    previous = timing.read(root / (tag + '_source'))
    ids = sorted(set(candidate['scan']) & set(previous['scan']))
    ids = [i for i in ids if 0 <= candidate['scan'][i]['stamp'] - T0 < requested_end]
    assert ids
    assert all(candidate['scan'][i]['stamp'] == previous['scan'][i]['stamp'] for i in ids)
    comparisons.append({'reference': name, 'common_scan_count': len(ids),
                        'first_last_physical_seconds': [candidate['scan'][i]['stamp'] - T0
                                                        for i in [ids[0], ids[-1]]],
                        'reference_source': source_stats(previous, ids),
                        'candidate_source': source_stats(candidate, ids)})

resources = [json.loads(line) for line in (RUN / 'resource_samples.jsonl').open()]
memory = []
for sample in resources:
    for container, processes in sample['processes'].items():
        for process in processes:
            memory.append({'sensor_seconds': sample['transport']['last_imu_ns'] * 1e-9 - T0,
                           'container': container, 'executable': process['executable'],
                           'memory': process['memory']})

result = {
    'passed_validation': True,
    'scope': 'One live160 capture, no retries or tuning, no GT scoring.',
    'timing_caveat': 'Nested timers overlap. Historical live arrivals and nonlinear Values differ; '
                     'matched physical scan times are not a controlled isolated E2 speedup test.',
    'windows': windows, 'historical_common_scan_comparisons': comparisons,
    'headroom': {str(w['seconds']): {'p95_ms': w['source']['scan_ms']['total']['p95'],
                                    'target_ms': 800,
                                    'passed': w['source']['scan_ms']['total']['p95'] < 800,
                                    'paced_interval_ms': 1000}
                 for w in [windows[0], windows[-1]]},
    'coverage': validation['coverage'],
    'final_transport': validation['final_transport_status'],
    'capture_wall_s': json.loads((ROOT / (TAG + '_control/complete.json')).read_text())['seconds'],
    'conservation': {key: validation[key] for key in
                     ['imu_samples', 'raw_source_ids', 'versions', 'active_source_ids', 'replacements']},
    'numerical_repeat': validation['numerical_repeat'],
    'cache': rows(ROOT / (TAG + '_receiver/receiver/e2_memo.csv')),
    'sampled_memory_not_full_peak_profile': memory,
    'source_log_checks': candidate['summary'],
}
with (RUN / 'engineering_summary.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps({key: result[key] for key in
                  ['headroom', 'coverage', 'final_transport', 'capture_wall_s', 'conservation']}, indent=2))
for w in windows:
    print('WINDOW', w['seconds'], 'scan', w['source']['scan_ms']['total'],
          'receiver_ms', w['receiver']['refinement_ms'], 'queue', w['receiver']['sampled_queue'])
