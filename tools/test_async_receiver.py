#!/usr/bin/env python3
"""Bounded synthetic integration checks for the existing gnssFGO receiver."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--receiver-model', choices=['imu_wnoj', 'source_wnoj'], default='imu_wnoj')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    base = 1_000_000_000
    imu = [dict(timestamp_ns=base + i * 10_000_000, ax=0, ay=0, az=9.81,
                gx=0, gy=0, gz=0, record_timestamp_ns=base+i*10_000_000)
           for i in range(351)]
    # One out-of-order receipt: later physical samples arrive before this one.
    imu[88]['record_timestamp_ns'] += 60_000_000
    write_csv(args.out/'imu.csv', imu)
    rows = []
    for n, source in enumerate(['lidar0', 'lidar1', 'lidar2', 'lidar3', 'camera0', 'camera1', 'camera2']):
        a, b = [(680, 880), (630, 830), (680, 830), (630, 880),
                (630, 660), (730, 930), (680, 880)][n]
        row = dict(ordinal=n, source_agent=source, session_id='one', revision=0,
                   from_index=0, to_index=1, from_timestamp_ns=base+a*1_000_000,
                   to_timestamp_ns=base+b*1_000_000,
                   record_timestamp_ns=base+(b+1+n)*1_000_000)
        row.update({f'relative_mu_{i}': 0 for i in range(6)})
        row.update({f'covariance_{i}_{j}': 0.01 if i == j else 0.0001
                    for i in range(6) for j in range(6)})
        rows.append(row)
    rows += [dict(rows[0], revision=2, relative_mu_3=0.001,
                  record_timestamp_ns=base+1_250_000_000),
             dict(rows[0], revision=1, record_timestamp_ns=base+1_300_000_000),
             dict(rows[0], revision=2, relative_mu_3=0.001,
                  record_timestamp_ns=base+1_350_000_000),
             dict(rows[0], session_id='restart', record_timestamp_ns=base+1_400_000_000),
             dict(rows[1], ordinal=99, from_index=5, to_index=6, revision=3,
                  from_timestamp_ns=base+1_830_000_000, to_timestamp_ns=base+2_130_000_000,
                  record_timestamp_ns=base+2_230_000_000)]

    def run(name, beliefs, expect_success=True, imu_rows=None):
        input_path = args.out/(name+'.csv')
        write_csv(input_path, beliefs)
        imu_path = args.out/'imu.csv'
        if imu_rows is not None:
            imu_path = args.out/(name+'_imu.csv')
            write_csv(imu_path, imu_rows)
        output = args.out/name
        command = [args.binary, '--imu', str(imu_path), '--revision-beliefs', str(input_path),
                   '--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
                   '--support-policy', 'uniform_imu', '--receiver-duration-ns', '3000000000',
                   '--out', str(output)]
        if args.receiver_model != 'imu_wnoj':
            command += ['--receiver-model', args.receiver_model]
        result = subprocess.run(command, capture_output=True, text=True)
        (args.out/(name+'.log')).write_text(result.stdout+result.stderr)
        assert (result.returncode == 0) == expect_success, result.stderr
        return output

    primary = run('primary', rows)
    repeat = run('repeat', rows)
    for filename in ['states.csv', 'online_states.csv', 'support_clock.csv', 'causal_dispatch.csv',
                     'support_accelerations.csv', 'trajectory.tum']:
        assert (primary/filename).read_bytes() == (repeat/filename).read_bytes(), filename
    summary = json.loads((primary/'summary.json').read_text())
    assert summary['beliefs_accepted'] == 9
    assert summary['revision_removals'] == 1
    assert summary['revision_stale'] == 1
    assert summary['revision_duplicates'] == 1
    assert all(summary['factor_kinds'][key] > 0 for key in
               ['exact', 'single_first', 'single_second', 'same_support', 'double'])
    assert len(read_csv(primary/'source_accounting.csv')) == 7
    for event in read_csv(primary/'causal_dispatch.csv'):
        assert int(event['insertion_ns']) >= int(event['arrival_ns'])
        assert int(event['created_support']) >= int(event['required_support'])
    # Change all future payloads and physical timestamps; prefix must not change.
    suffix = [dict(r, relative_mu_3=0.03, to_timestamp_ns=base+2_200_000_000)
              if int(r['record_timestamp_ns']) > base+2_000_000_000 else r for r in rows]
    changed = run('changed_future', suffix)
    for filename, stamp in [('online_states.csv', 'timestamp_ns'),
                            ('support_accelerations.csv', 'timestamp_ns')]:
        prefix = lambda p: [r for r in read_csv(p/filename) if int(r[stamp]) < base+2_000_000_000]
        assert prefix(primary) == prefix(changed), filename
    assert (primary/'support_clock.csv').read_bytes() == (changed/'support_clock.csv').read_bytes()
    changed_imu = [dict(r, ax=0.1, record_timestamp_ns=int(r['record_timestamp_ns'])+50_000_000)
                   if int(r['timestamp_ns']) > base+2_000_000_000 else r for r in imu]
    future_imu = run('changed_future_imu', rows, imu_rows=changed_imu)
    for filename, stamp in [('online_states.csv', 'timestamp_ns'),
                            ('support_accelerations.csv', 'timestamp_ns'),
                            ('support_clock.csv', 'physical_timestamp_ns')]:
        prefix = lambda p: [r for r in read_csv(p/filename) if int(r[stamp]) < base+2_000_000_000]
        assert prefix(primary) == prefix(future_imu), filename
    # Source dropout changes the solution, never the receiver clock.
    dropout = run('dropout', [r for r in rows if r['source_agent'] == 'lidar0'])
    assert (primary/'support_clock.csv').read_bytes() == (dropout/'support_clock.csv').read_bytes()
    # Physical-file order is irrelevant: arrival and deterministic tie ordering decide.
    reordered = run('reordered', list(reversed(rows)))
    assert (primary/'states.csv').read_bytes() == (reordered/'states.csv').read_bytes()
    conflict = rows + [dict(rows[0], revision=2, relative_mu_3=0.1,
                            record_timestamp_ns=base+1_500_000_000)]
    run('conflicting_revision', conflict, False)
    run('changed_endpoint', rows+[dict(rows[0], revision=4,
        to_timestamp_ns=base+890_000_000, record_timestamp_ns=base+1_550_000_000)], False)
    run('prearrival', [dict(rows[0], record_timestamp_ns=base+800_000_000)], False)
    result = dict(passed=True, synthetic_sources=7, unique_ids=9,
                  tested=['five endpoint patterns', 'delayed IMU receipt',
                          'clock only' if args.receiver_model == 'source_wnoj' else 'fractional ALL_RIGHT',
                          'replacement', 'duplicate', 'stale', 'first seen revision greater than zero',
                          'session restart', 'future belief and IMU suffix invariance', 'source dropout',
                          'CSV permutation', 'deterministic repeat', 'conflict rejection'],
                  summary=summary,
                  trajectory_sha256=hashlib.sha256((primary/'trajectory.tum').read_bytes()).hexdigest())
    (args.out/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
