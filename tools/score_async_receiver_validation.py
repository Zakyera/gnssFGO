#!/usr/bin/env python3
"""Resample only for a common scoring mask, then call the unchanged scorers."""
import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--products', type=Path, required=True)
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--tools', type=Path, required=True)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--output-subdir', default='evaluation')
    args = p.parse_args()
    spec = importlib.util.spec_from_file_location('relative_score', args.tools/'score_grandtour_relative_pose.py')
    scorer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scorer)
    products = {name: scorer.load_tum(args.products/folder/file) for name, folder, file in [
        ('legacy_final', 'legacy', 'trajectory.tum'),
        ('legacy_online', 'legacy', 'online_trajectory.tum'),
        ('independent_final', 'uniform', 'trajectory.tum'),
        ('independent_online', 'uniform', 'online_trajectory.tum')]}
    # Match the existing viewer/evaluation: GLIM's saved native trajectory,
    # Kimera's native ROS odometry (not Kimera's smoother-state trajectory).
    products['native_glim'] = scorer.sort_unique(*scorer.load_tum(args.capture/'odom_imu.txt'))
    products['native_kimera'] = scorer.sort_unique(*scorer.load_odometry_csv(
        args.capture/'kimera_odometry.csv'))
    start = max(x[0][0] for x in products.values())
    end = min(x[0][-1] for x in products.values())
    times = start + np.arange(int(np.floor((end-start)/.1))+1)*.1
    # Only retain common sample times with interpolation support in every product.
    valid = np.ones(len(times), dtype=bool)
    for t, _, _ in products.values():
        right = np.searchsorted(t, times)
        valid &= (right > 0) & (right < len(t))
        indices = np.clip(right, 1, len(t)-1)
        valid &= t[indices]-t[indices-1] <= .5
    times = times[valid]
    out = args.products/args.output_subdir
    out.mkdir(exist_ok=False)
    results = {}
    for name, (t, xyz, q) in products.items():
        output = []
        for stamp in times:
            right = int(np.searchsorted(t, stamp))
            left = right-1
            fraction = float((stamp-t[left])/(t[right]-t[left]))
            pos = xyz[left]*(1-fraction)+xyz[right]*fraction
            quat = scorer.slerp(q[left], q[right], fraction)
            output.append(np.r_[stamp, pos, quat])
        tum = out/(name+'.tum')
        np.savetxt(tum, output, fmt='%.12f')
        common = ['--estimate', str(tum), '--estimate-format', 'tum',
                  '--start-time-sec', str(times[0]), '--end-time-sec', str(times[-1])]
        for kind, script, bag, extra in [
            ('prism', 'score_grandtour_prism.py', 'ap20_prism_position', ['--source-frame', 'cpt7_imu']),
            ('rpe', 'score_grandtour_relative_pose.py', 'cpt7_ie_tc', ['--delta-sec', '1.0'])]:
            target = out/(name+'_'+kind+'.json')
            command = [sys.executable, str(args.tools/script), *common, *extra,
                       '--reference-bag', str(args.dataset/('2024-11-02-17-43-10_'+bag+'.bag')),
                       '--output', str(target)]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            (out/(name+'_'+kind+'.log')).write_text(result.stdout+result.stderr)
        results[name] = {kind: json.loads((out/(name+'_'+kind+'.json')).read_text())
                         for kind in ['prism', 'rpe']}
    assert len({v['prism']['matched_poses'] for v in results.values()}) == 1
    report = dict(start=float(times[0]), end=float(times[-1]), common_samples=len(times),
                  note='Scoring-only common resampling using existing slerp; no estimator rerun.',
                  results=results)
    (out/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
