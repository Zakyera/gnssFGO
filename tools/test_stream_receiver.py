#!/usr/bin/env python3
"""Compare a genuinely incremental pipe with the existing offline receiver."""
import argparse
import csv
import json
import subprocess
from pathlib import Path


def read(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def events(imu, beliefs):
    output = []
    for r in imu:
        arrival = int(r.get('record_timestamp_ns', r['timestamp_ns']))
        output.append((arrival, 0, '', 0, ['I', arrival, r['timestamp_ns'],
                       *[r[k] for k in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']]]))
    for r in beliefs:
        arrival = int(r['record_timestamp_ns'])
        source = {'g': 'glim', 'k': 'kimera'}.get(r['source_agent'], r['source_agent'])
        output.append((arrival, 1, source, int(r['ordinal']), ['B', arrival, source,
                       r.get('session_id', 'default'), r['ordinal'], r.get('revision', 0),
                       r['from_index'], r['to_index'], r['from_timestamp_ns'], r['to_timestamp_ns'],
                       *[r['relative_mu_'+str(i)] for i in range(6)],
                       *[r[f'covariance_{i}_{j}'] for i in range(6) for j in range(6)]]))
    output.sort(key=lambda r: r[:4])
    return [','.join(map(str, r[4]))+'\n' for r in output]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--receiver-model', choices=['imu_wnoj', 'source_wnoj', 'source_wnoa'], default='imu_wnoj')
    p.add_argument('--imu', type=Path, required=True)
    p.add_argument('--beliefs', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=3.0)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    imu = read(args.imu)
    end = int(imu[0]['timestamp_ns'])+round(args.seconds*1e9)
    imu = [r for r in imu if int(r['timestamp_ns']) <= end]
    # Same finite information set, including late-arriving revisions if their
    # endpoints have support. A canonical journal is retained for reproduction.
    beliefs = [r for r in read(args.beliefs) if int(r['record_timestamp_ns']) <= end]
    for name, data in [('imu', imu), ('beliefs', beliefs)]:
        with (args.out/(name+'.csv')).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, list(data[0]))
            writer.writeheader(); writer.writerows(data)
    common = [args.binary, '--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
              '--support-policy', 'uniform_imu', '--receiver-duration-ns', str(round(args.seconds*1e9))]
    if args.receiver_model != 'imu_wnoj':
        common += ['--receiver-model', args.receiver_model]
    subprocess.run(common+['--imu', str(args.out/'imu.csv'), '--revision-beliefs',
                   str(args.out/'beliefs.csv'), '--out', str(args.out/'offline')], check=True)
    journal = events(imu, beliefs)
    journal.append('E,'+str(end)+'\n')
    (args.out/'events.csv').write_text(''.join(journal))
    # Write one event at a time to a running process: no sensor CSV is passed.
    proc = subprocess.Popen(common+['--event-stream', '-', '--out', str(args.out/'stream')],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True)
    for line in journal:
        proc.stdin.write(line)
        proc.stdin.flush()
    proc.stdin.close()
    proc.wait()
    stdout, stderr = proc.stdout.read(), proc.stderr.read()
    (args.out/'stream.log').write_text(stdout+stderr)
    assert proc.returncode == 0, stderr
    checks = {}
    for name in ['trajectory.tum', 'states.csv', 'online_trajectory.tum', 'online_states.csv',
                 'support_accelerations.csv', 'support_clock.csv', 'model_covariance.csv']:
        checks[name] = (args.out/'offline'/name).read_bytes() == (args.out/'stream'/name).read_bytes()
        assert checks[name], name
    summary = json.loads((args.out/'stream/summary.json').read_text())
    assert summary['streaming'] is True
    (args.out/'validation.json').write_text(json.dumps(dict(passed=True, byte_identity=checks,
                                                          summary=summary), indent=2)+'\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
