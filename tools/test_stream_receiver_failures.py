#!/usr/bin/env python3
"""Negative streaming checks plus explicit end-of-stream support accounting."""
import argparse
import json
import subprocess
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', required=True)
    p.add_argument('--receiver-model', choices=['imu_wnoj', 'source_wnoj', 'source_wnoa'], default='imu_wnoj')
    p.add_argument('--journal', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    lines = args.journal.read_text().splitlines()
    imu = [r for r in lines if r.startswith('I,')][:60]
    end = int(imu[-1].split(',')[1])
    b = next(r.split(',') for r in lines if r.startswith('B,'))
    b[1] = str(end)
    b[8] = str(int(imu[-2].split(',')[2])+1)
    b[9] = imu[-1].split(',')[2]
    base = [args.binary, '--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
            '--support-policy', 'uniform_imu', '--event-stream', '-']
    if args.receiver_model != 'imu_wnoj':
        base += ['--receiver-model', args.receiver_model]
    checks = {}

    def run(name, events, good):
        out = args.out/name
        result = subprocess.run(base+['--out', str(out)], input='\n'.join(events)+'\n',
                                text=True, capture_output=True, timeout=30)
        (args.out/(name+'.log')).write_text(result.stdout+result.stderr)
        assert (result.returncode == 0) == good, result.stdout+result.stderr
        assert (out/'summary.json').exists() == good
        checks[name] = True
        return out

    run('disconnect_without_end', imu, False)
    run('arrival_clock_backwards', imu+[imu[0]], False)
    bad = list(b); bad[16] = '-1000'
    run('invalid_covariance', imu+[','.join(bad), 'E,'+str(end)], False)
    run('before_initialization_end', imu[:3]+['E,'+str(end)], False)
    out = run('missing_right_support_is_not_extrapolated', imu+[','.join(b), 'E,'+str(end)], True)
    summary = json.loads((out/'summary.json').read_text())
    assert summary['pending_at_end'] == 1 and summary['beliefs_accepted'] == 0
    assert 'missing_right_support_at_end' in (out/'unsupported_beliefs.csv').read_text()
    (args.out/'validation.json').write_text(json.dumps(checks, indent=2)+'\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    main()
