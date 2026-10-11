"""Bounded unchanged-receiver diagnostics; never launch source estimators."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parent
CAPTURE = ROOT.parent / 'source_huber_eth1_30s_opencv42_20261008'
BINARY = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay')
ENV = json.loads((CAPTURE/'on_huber_receiver/resolved_config.json').read_text())['receiver_environment']

def main():
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('--launch', choices=['validator', 'live'], required=True)
    p.add_argument('--pipe', action='store_true')
    a = p.parse_args()
    assert a.name.replace('_', '').isalnum()
    run = ROOT/a.name
    run.mkdir(exist_ok=False)
    assert hashlib.sha256(BINARY.read_bytes()).hexdigest() == 'fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
    journal = CAPTURE/'on_huber_receiver/events.csv'
    event_path = run/'events.fifo' if a.pipe else journal
    if a.pipe:
        os.mkfifo(event_path, 0o600)
    common = ['--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
              '--support-policy', 'uniform_imu', '--support-period-ns', '100000000',
              '--receiver-startup-samples', '59']
    if a.launch == 'live':
        cmd = ['env', *[k+'='+v for k,v in sorted(ENV.items())], str(BINARY),
               '--event-stream', str(event_path), '--out', str(run/'receiver'),
               *common, '--receiver-model', 'source_wnoa']
        command = ['docker', 'exec', 'ubuntu-jammy-humble-nvidia', 'bash', '-lc',
                   'set -e; source /opt/ros/humble/setup.bash; exec '+shlex.join(cmd)]
    else:
        command = ['docker', 'exec', 'ubuntu-jammy-humble-nvidia', 'env',
                   *[k+'='+v for k,v in ENV.items()], str(BINARY),
                   '--receiver-model', 'source_wnoa', *common,
                   '--event-stream', str(event_path), '--out', str(run/'receiver')]
    evidence = dict(command=command, journal=str(journal),
                    journal_sha256=hashlib.sha256(journal.read_bytes()).hexdigest(),
                    binary_sha256=hashlib.sha256(BINARY.read_bytes()).hexdigest())
    (run/'command.json').write_text(json.dumps(evidence, indent=2))
    start=time.monotonic()
    with (run/'receiver.log').open('x') as log:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        errors=[]
        def feed():
            try:
                with event_path.open('wb') as out, journal.open('rb') as inp:
                    for line in inp:
                        out.write(line)
                        out.flush()
            except Exception as ex:
                errors.append(repr(ex))
        writer=threading.Thread(target=feed, daemon=True) if a.pipe else None
        if writer: writer.start()
        rc=proc.wait(timeout=180)
        if writer: writer.join(timeout=5)
    evidence.update(returncode=rc,seconds=time.monotonic()-start,pipe_errors=errors)
    (run/'completion.json').write_text(json.dumps(evidence,indent=2))
    assert rc==0 and not errors and (not writer or not writer.is_alive()), evidence
    print(json.dumps(dict(name=a.name,returncode=rc,seconds=evidence['seconds'])))

if __name__=='__main__': main()
