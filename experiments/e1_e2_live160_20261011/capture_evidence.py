"""Preserve read-only evidence after the native validation blocker; no replay."""
import ast
import collections
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import numpy as np

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS / 'runs/gnssfgo_e1_e2_live160_20261011'
ROOT = RUN / 'prepared'
SOURCE = ROOT / 'live160_on_source'
LIVE = ROOT / 'live160_on_receiver'
T0 = 1727775008.003475
spec = importlib.util.spec_from_file_location(
    'saved_timing', WS / 'runs/glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)
stats = timing.stats


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


assert (ROOT / 'live160_on_control/complete.json').is_file()
assert (RUN / 'validation_logs/failed.txt').is_file()
assert not (ROOT / 'live160_on_validation.json').exists()
assert not (ROOT / 'live160_on_numerical_repeat').exists()
manifest = json.loads((ROOT / 'protected_manifest.json').read_text())
for path, expected in manifest.items():
    assert sha(path) == expected, path

logs = [p for p in (SOURCE / 'ros_logs').glob('*/*stdout.log') if p.parent.name != 'latest']
glog, = [p for p in logs if 'glim_rosnode' in p.name]
klog, = [p for p in logs if 'kimera_vio' in p.name]
guard_records = []
guard_by_edge = {}
guard_counts = collections.Counter()
guard_actions = ['drop_before_stage', 'rollback_and_retry_without_staged_cbs',
                 'rollback_remove_active_and_retry']
for line_number, line in enumerate(klog.open(), 1):
    for action in guard_actions:
        if 'action=' + action not in line:
            continue
        edge = tuple(map(int, re.search(r',x(\d+)->x(\d+),', line).groups()))
        item = {'path': str(klog), 'line': line_number, 'action': action,
                'receiver_edge': edge, 'text': line.rstrip()}
        guard_records.append(item)
        guard_counts[action] += 1
        if action == 'drop_before_stage':
            guard_by_edge.setdefault(edge, []).append(item)

alignment = {}
for kind, path, matrix_path in [('glim', glog, glog), ('kimera', SOURCE / 'estimators.log', klog)]:
    matrices = collections.Counter()
    for line in matrix_path.open():
        if 'CBS_ODOM_FACTOR_MATRIX_ROW,' not in line:
            continue
        v = line.split('CBS_ODOM_FACTOR_MATRIX_ROW,')[1].split(',')
        if v[0] in ['K2G', 'G2K']:
            matrices[tuple(int(x[1:]) for x in v[1:3])] += 1
    missing = []
    calls = 0
    for line_number, line in enumerate(path.open(), 1):
        marker = kind.upper() + '_CBS_MEAN_ALIGNMENT_ROW,'
        if marker not in line:
            continue
        calls += 1
        v = line.split(marker)[1].split(',')
        update = int(v[0]) if kind == 'kimera' else None
        v = v[1:] if kind == 'kimera' else v
        edge = tuple(map(int, v[2:4]))
        if edge not in matrices:
            matching = guard_by_edge.get(edge, []) if kind == 'kimera' else []
            missing.append({'path': str(path), 'line': line_number, 'backend_update': update,
                            'sender_edge': list(map(int, v[:2])), 'receiver_edge': edge,
                            'receiver_endpoint_seconds': int(v[7]) * 1e-9 - T0,
                            'alignment_available_seconds': int(v[8]) * 1e-9 - T0,
                            'matching_existing_pre_stage_guard': matching})
    alignment[kind] = {'alignment_calls': calls, 'matrix_calls': sum(matrices.values()),
                       'missing_matrix_entries': missing}
assert len(alignment['kimera']['missing_matrix_entries']) == 5
assert all(len(r['matching_existing_pre_stage_guard']) == 1
           for r in alignment['kimera']['missing_matrix_entries'])
assert not alignment['glim']['missing_matrix_entries']

data = timing.read(SOURCE)
publications = rows(LIVE / 'receiver/publication_convergence.csv')
steps = rows(LIVE / 'receiver/publication_steps.csv')
monitor = [json.loads(line) for line in (ROOT / 'live160_on_control/monitor.jsonl').open()]
monitor = [r for r in monitor if r['published_poses'] > 0 and r['last_imu_ns'] > 0]
windows = []
for lo, hi in [(0, 160), (0, 40), (40, 80), (80, 120), (120, 160)]:
    ids = [i for i, r in data['scan'].items() if lo <= r['stamp'] - T0 < hi]
    nested = {}
    for kind, columns in {'on_smoother_update': {'sidecar_update': 10},
                          'publish_outgoing': {'local_covariance_rebuild': 17, 'belief_query': 18},
                          'on_smoother_update_finish': {'outgoing_plus_revisions': 12}}.items():
        found = [data['cbs'][kind][i][0] for i in ids if i in data['cbs'].get(kind, {})]
        nested.update({key: stats([r[col] for r in found]) for key, col in columns.items()})
    pub = [r for r in publications if lo <= float(r['physical_ns']) * 1e-9 - T0 < hi]
    samples = [r for r in monitor if lo <= r['last_imu_ns'] * 1e-9 - T0 < hi]
    windows.append({'seconds': [lo, hi], 'scans': len(ids),
                    'scan_ms': {key: stats([data['scan'][i][key] for i in ids]) for key in
                                ['pre_callback', 'smoother', 'post_callback', 'total']},
                    'nested_ms': nested,
                    'scans_over_1000ms': sum(data['scan'][i]['total'] > 1000 for i in ids),
                    'receiver_refinement_ms': stats([1000 * float(r['seconds']) for r in pub]),
                    'sampled_queue': stats([r['queued'] for r in samples]),
                    'output_age_simulated_s': stats([r['output_age_seconds'] for r in samples])})

coverage = json.loads((SOURCE / 'source_coverage.json').read_text())
for role, filename in [('fused_online', 'online_trajectory.tum'), ('fused_final', 'trajectory.tum')]:
    a = np.loadtxt(LIVE / 'receiver' / filename)
    coverage[role] = {'rows': len(a), 'span_s': float(a[-1, 0] - a[0, 0]),
                      'start_s': float(a[0, 0]), 'end_s': float(a[-1, 0]),
                      'path_length_m': float(np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1).sum())}

repos = {}
for name in ['gnssFGO', 'V4RL/cbs_gtsam4.3/src/glim_ros1', 'V4RL/cbs_gtsam4.3/src/cbs',
             'V4RL/cbs_gtsam4.3/src/gtsam_points', 'V4RL/cbs_gtsam4.3/src/Kimera-VIO',
             'V4RL/cbs_gtsam4.3/src/cbsms']:
    path = Path('/home/yeranis/repos') / name
    repos[name] = {'head': subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip(),
                   'status': subprocess.check_output(['git', '-C', str(path), 'status', '--short'], text=True)}
    if name != 'gnssFGO':
        assert not repos[name]['status']

processes = {}
for container in ['cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia']:
    output = subprocess.check_output(['docker', 'exec', container, 'ps', '-eo', 'pid,comm,args'], text=True)
    processes[container] = output
    assert not any(token in output for token in ['glim_rosnode', 'kimera_vio_ros_node', '/replay', 'rosmaster'])
for path in HERE.glob('*.py'):
    ast.parse(path.read_text())
cache = {r['kind']: r for r in rows(LIVE / 'receiver/e2_memo.csv')}
result = {
    'capture_completed': True, 'overall_validation_passed': False,
    'blocked_gate': 'validate_native_bidirectional.py:59, assert (ri,rj) in mats',
    'reason': 'Five aligned Kimera candidates were explicitly dropped by the unchanged '
              'pre-stage residual guard before matrix logging; the validator assumes they all reach it.',
    'no_validator_or_estimator_edit': True, 'saved_journal_replay_run': False,
    'unrun_validation_stages': ['lifecycle', 'diagnostics', 'numerical/conservation/replay'],
    'alignment_log_inventory_not_wire_revalidation': alignment,
    'existing_kimera_guard_counts': dict(guard_counts), 'existing_kimera_guard_records': guard_records,
    'capture_wall_s': json.loads((ROOT / 'live160_on_control/complete.json').read_text())['seconds'],
    'timing_windows': windows,
    'headroom_800ms_full_and_late_pass': [windows[i]['scan_ms']['total']['p95'] < 800 for i in [0, -1]],
    'timing_caveat': 'Nested timers overlap; no isolated same-input E2 speedup is inferred.',
    'coverage_observed_not_complete_validation': coverage,
    'receiver_summary_self_reported': json.loads((LIVE / 'receiver/summary.json').read_text()),
    'final_transport': json.loads((LIVE / 'transport_status.json').read_text()),
    'publication_status_counts': dict(collections.Counter(r['status'] for r in publications)),
    'max_publication_gradient': max(float(r['gradient_after']) for r in publications),
    'publication_steps': len(steps), 'recovery_steps': sum(r['recovery'] == '1' for r in steps),
    'cache_last': cache, 'protected_hashes_unchanged': len(manifest), 'repositories': repos,
    'processes_after_stop': processes,
    'journal_sha256': sha(LIVE / 'events.csv'),
    'source_log_hashes': {str(p): sha(p) for p in [glog, klog, SOURCE / 'estimators.log']},
}
with (RUN / 'capture_evidence.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps({k: result[k] for k in ['capture_completed', 'overall_validation_passed', 'blocked_gate',
                                        'existing_kimera_guard_counts', 'headroom_800ms_full_and_late_pass',
                                        'publication_status_counts', 'max_publication_gradient',
                                        'publication_steps', 'recovery_steps', 'protected_hashes_unchanged']}, indent=2))
