#!/usr/bin/env python3
"""Synthetic-only integration and preserved-default checks; no dataset access."""
import argparse
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path
from test_stream_receiver import events


def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--baseline-binary', required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    commands, checks = [], {}
    common = ['--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
              '--support-policy', 'uniform_imu', '--event-stream', '-']
    base, start = 1000000000, 1580000000
    imu = [dict(timestamp_ns=base+i*10000000, record_timestamp_ns=base+i*10000000,
                ax=0, ay=0, az=9.81, gx=0, gy=0, gz=0) for i in range(301)]

    def journal(rows, readings=imu):
        return ''.join(events(readings, rows)+[f'E,{base+3000000000}\n'])

    def run(name, content, mode='source_wnoj', binary=None, extra=(), success=True):
        directory = args.out/name
        command = [binary or args.binary, *common, '--out', str(directory), *extra]
        if mode:
            command += ['--receiver-model', mode]
        (args.out/(name+'.journal')).write_text(content)
        result = subprocess.run(command, input=content, text=True, capture_output=True, timeout=120)
        (args.out/(name+'.log')).write_text(result.stdout+result.stderr)
        commands.append(dict(name=name, command=command, returncode=result.returncode))
        (args.out/'commands.json').write_text(json.dumps(commands, indent=2)+'\n')
        assert (result.returncode == 0) == success, (name, result.stderr)
        return directory

    def equal(a, b):
        files = ['trajectory.tum', 'online_trajectory.tum', 'states.csv', 'online_states.csv',
                 'support_clock.csv', 'support_accelerations.csv', 'belief_residuals.csv',
                 'source_model_covariance.csv', 'model_covariance.csv', 'causal_dispatch.csv']
        for filename in files:
            assert (a/filename).read_bytes() == (b/filename).read_bytes(), (a, b, filename)
        return {f: hashlib.sha256((a/f).read_bytes()).hexdigest() for f in files}

    for scenario, speed, accel, spin in [('stationary', 0, 0, 0), ('velocity', .5, 0, .1),
                                        ('acceleration', .3, .2, .1)]:
        rows = []
        for sensor, offset in [('lidar', 0), ('camera', 30000000)]:
            for n in range(10):
                # Partial-support endpoints, adjacent/shared support intervals;
                # no measurements during the last ~0.4 s (prediction gap).
                ti = start+offset+n*200000000
                tj = ti+170000000
                a, b = (ti-start)*1e-9, (tj-start)*1e-9
                mu = [spin*(b-a), 0, 0, speed*(b-a)+.5*accel*(b*b-a*a), 0, 0]
                row = dict(ordinal=len(rows), source_agent=sensor, session_id='synthetic', revision=0,
                           from_index=n, to_index=n+1, from_timestamp_ns=ti, to_timestamp_ns=tj,
                           record_timestamp_ns=tj+50000000+(n % 3)*50000000)
                row.update({f'relative_mu_{i}': mu[i] for i in range(6)})
                # Dense SPD source covariance; unchanged by receiver.
                row.update({f'covariance_{i}_{j}': 1e-5 if i == j else 1e-7
                            for i in range(6) for j in range(6)})
                rows.append(row)
        # Late correction of an existing ID, not a second active factor.
        rows.append(dict(rows[1], revision=1, record_timestamp_ns=base+2800000000))
        content = journal(rows)
        output = run(scenario, content, extra=['--graph-snapshots-out', str(args.out/(scenario+'_graph'))])
        repeat = run(scenario+'_repeat', content)
        hashes = equal(output, repeat)
        mutated = [dict(r, ax=17+math.sin(i), ay=-23, az=0, gx=9, gy=-4, gz=.3*i)
                   for i, r in enumerate(imu)]
        altered = run(scenario+'_changed_all_imu_values', journal(rows, mutated))
        equal(output, altered)
        summary = json.loads((output/'summary.json').read_text())
        assert summary['imu_factor_count'] == summary['bias_variable_count'] == summary['startup_imu_samples'] == 0
        assert summary['gravity'] is None and summary['initial_gyroscope_bias'] is None
        assert summary['beliefs_accepted'] == 20 and summary['revision_removals'] == 1
        assert summary['active_factor_count'] == 4+summary['support_states']-1+20
        states = read(output/'states.csv')
        assert not any(k.startswith(('bg', 'ba')) for k in states[0])
        assert all(math.isfinite(float(v)) for r in states for v in r.values())
        pose_error = max(abs(float(r['px'])-(speed*t+.5*accel*t*t))
                         for r in states for t in [(int(r['timestamp_ns'])-start)*1e-9])
        assert pose_error < .02, (scenario, pose_error)
        assert max(float(r['mahalanobis_squared']) for r in read(output/'belief_residuals.csv')) < .01
        keys = [r['keys'].split(';') for r in read(args.out/(scenario+'_graph')/'factor_snapshots.csv')]
        assert all(len(k) == len(set(k)) for k in keys)
        assert any(any(k.startswith('a') for k in group) for group in keys)
        assert not any(any(k.startswith('b') for k in group) for group in keys)
        checks[scenario] = dict(max_position_error_m=pose_error, repeat_and_imu_invariant_hashes=hashes,
                                summary=summary)
        archived = run(scenario+'_archived_default', content, mode=None, binary=args.baseline_binary)
        default = run(scenario+'_new_default', content, mode=None)
        explicit = run(scenario+'_new_explicit_default', content, mode='imu_wnoj')
        checks[scenario+'_default_preserved'] = equal(archived, default)
        equal(default, explicit)
    # A clock-only stream must be well posed before any relative factor arrives.
    empty = run('no_sources_yet', journal([]))
    assert json.loads((empty/'summary.json').read_text())['beliefs_accepted'] == 0
    for invalid in ['bad_model']:
        run(invalid, journal([]), mode=invalid, success=False)
    run('mixed_legacy_ablation', journal([]), extra=['--audit-use-imu', 'false'], success=False)
    run('forbidden_scaling', journal([]), extra=['--source-model-noise', 'lidar,0.1,0.1'], success=False)
    checks['startup_without_beliefs'] = True
    (args.out/'validation.json').write_text(json.dumps(dict(passed=True, checks=checks), indent=2)+'\n')
    print(json.dumps({k: v.get('max_position_error_m') if isinstance(v,dict) else v
                      for k,v in checks.items()}, indent=2))


if __name__ == '__main__':
    main()
