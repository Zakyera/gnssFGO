#!/usr/bin/env python3
"""Score retained, as-published OFF/ON prefixes; no estimator is executed.

The longer OFF final graph is deliberately excluded. Native source curves
are ROS odometry, not shutdown dumps that can contain smoother revisions.
"""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from tf.transformations import quaternion_from_matrix

root = Path(__file__).resolve().parent
tools = Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
sys.path.insert(0, str(tools))
from score_grandtour_relative_pose import load_tum, sort_unique, interpolate_pose

out = root / 'retained120_online_off_on_evaluation'
out.mkdir(exist_ok=False)
original = out / 'original'
original.mkdir()
assert json.loads((root / 'bounded120_on_validation.json').read_text())['passed']
interval = json.loads((root / 'bounded120_evaluation/common_interval.json').read_text())
reference_grid = np.loadtxt(root / 'bounded120_common_evaluation/on_fused_online.tum')[:, 0]
paths, inputs, checks = {}, {}, {}
first_imu = 1727775008003475000
for mode, tag in [('off', 'full_off'), ('on', 'bounded120_on')]:
    source = root / (tag + '_source')
    receiver = root / (tag + '_receiver')
    for name in ['glim', 'kimera']:
        label = mode + '_native_' + name
        src = source / (name + '_odometry.csv')
        dst = original / (label + '.tum')
        subprocess.run(['python3', str(tools / 'export_odometry_csv_tum.py'),
                        '--input', str(src), '--output', str(dst),
                        '--source-to-target', '0', '0', '0', '0', '0', '0', '1'], check=True)
        paths[label] = dst
        inputs[label] = {'path': str(src), 'sha256': hashlib.sha256(src.read_bytes()).hexdigest()}
    label = mode + '_fused_online'
    paths[label] = receiver / 'receiver/online_trajectory.tum'
    inputs[label] = {'path': str(paths[label]), 'sha256': hashlib.sha256(paths[label].read_bytes()).hexdigest()}
    monitor = [json.loads(line) for line in (root / (tag + '_control/monitor.jsonl')).read_text().splitlines()]
    prefix = [r for r in monitor if first_imu <= r['last_imu_ns'] <= first_imu + 120000000000]
    ages = [r['output_age_seconds'] for r in prefix if r['output_age_seconds'] is not None]
    conv = [r for r in csv.DictReader((receiver / 'receiver/publication_convergence.csv').open())
            if int(r['physical_ns']) <= first_imu + 120000000000]
    checks[mode] = {
        'prefix_monitor_rows': len(prefix), 'max_observed_queue': max(r['queued'] for r in prefix),
        'median_output_age_s': float(np.median(ages)), 'p95_output_age_s': float(np.percentile(ages, 95)),
        'max_output_age_s': max(ages), 'publication_checks': len(conv),
        'publication_status_counts': {s: sum(r['status'] == s for r in conv) for s in {r['status'] for r in conv}},
        'max_publication_gradient': max(float(r['gradient_after']) for r in conv),
        'binary_sha256': json.loads((receiver / 'manifest.json').read_text())['binary_sha256'],
    }
    assert checks[mode]['max_observed_queue'] < 1000
    assert checks[mode]['p95_output_age_s'] < 1
    assert all(r['status'] == 'stationary' for r in conv)
assert checks['off']['binary_sha256'] == checks['on']['binary_sha256']

curves = {name: sort_unique(*load_tum(path)) for name, path in paths.items()}
data = {name: [] for name in paths}
for stamp in reference_grid:
    poses = {name: interpolate_pose(*curve, stamp, .5) for name, curve in curves.items()}
    assert all(pose is not None for pose in poses.values()), ('missing common pose', stamp)
    for name, (position, rotation) in poses.items():
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        data[name].append([stamp, *position, *quaternion_from_matrix(matrix)])
for name, rows in data.items():
    np.savetxt(str(out / (name + '.tum')), rows, fmt='%.9f')

# Identical evaluators, physical interval, reference files and lever arm as
# the already validated ON scoring; substitute only estimate/output paths.
templates = json.loads((root / 'bounded120_evaluation/commands.json').read_text())
templates = [c for c in templates if Path(c[c.index('--estimate') + 1]).stem == 'on_fused_online']
assert len(templates) == 2
commands, results = [], {}
for name in paths:
    for template in templates:
        command = list(template)
        kind = 'prism' if '--aligned-csv' in command else 'rpe'
        command[command.index('--estimate') + 1] = str(out / (name + '.tum'))
        command[command.index('--output') + 1] = str(out / (name + '_' + kind + '.json'))
        if kind == 'prism':
            command[command.index('--aligned-csv') + 1] = str(out / (name + '_prism_aligned.csv'))
        with (out / (name + '_' + kind + '.log')).open('x') as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        commands.append(command)
    prism = json.loads((out / (name + '_prism.json')).read_text())
    rpe = json.loads((out / (name + '_rpe.json')).read_text())
    array = np.array(data[name])
    results[name] = {
        'ate_m': prism['ape_rmse_m'], 'translation_rpe_m': rpe['translation_error_m']['rmse'],
        'rotation_rpe_deg': rpe['rotation_error_deg']['rmse'],
        'common_grid_body_path_m': float(np.linalg.norm(np.diff(array[:, 1:4], axis=0), axis=1).sum()),
        'evaluation_prism_path_m': prism['estimate_full_path_m'],
        'matched_prism_poses': prism['matched_poses'],
        'gt_coverage_s': prism['matched_elapsed_coverage_s'], 'rpe_pairs': rpe['valid_pairs'],
    }
assert len({x['matched_prism_poses'] for x in results.values()}) == 1
assert len({x['gt_coverage_s'] for x in results.values()}) == 1
assert len({x['rpe_pairs'] for x in results.values()}) == 1
summary = {
    'description': 'First120s retained live/as-published comparison. No final graphs or shutdown GLIM dumps.',
    'off_caveat': 'OFF longer trial failed throughput later at218.570044s. This is a prefix-scoring audit, not full_OFF validation or deterministic repeat.',
    'common_times': len(reference_grid), 'start': float(reference_grid[0]), 'end': float(reference_grid[-1]),
    'common_scoring_interval': interval, 'inputs': inputs, 'prefix_checks': checks, 'results': results,
}
(out / 'commands.json').write_text(json.dumps(commands, indent=2))
(out / 'comparison.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
