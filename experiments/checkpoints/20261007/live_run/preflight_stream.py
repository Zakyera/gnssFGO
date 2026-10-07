#!/usr/bin/env python3
"""Bounded same-journal file/pipe equivalence, with deep audits disabled."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from prepare import ROOT, BINARY, ENV

out = ROOT/'preflight'
out.mkdir(exist_ok=False)
source = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/inputs/off_events.csv')
rows = source.read_text().splitlines()
first = int(next(r for r in rows if r.startswith('I,')).split(',')[2])
end = first + 8_000_000_000
rows = [r for r in rows if r[0] in ('I','B') and int(r.split(',')[1]) <= end]
rows.append('E,'+str(end))
journal = '\n'.join(rows)+'\n'
(out/'events.csv').write_text(journal)
environment = os.environ.copy()
environment.update(ENV)
for name in ['AUDIT_RECORD_ALL','WNOJ_AUDIT_INDICES']:
    environment.pop(name,None)
base = [str(BINARY), '--receiver-model','source_wnoa','--mode','sources','--timeline','record',
        '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
        '--receiver-startup-samples','59']
for mode in ['file','pipe']:
    command = base + ['--event-stream',str(out/'events.csv') if mode=='file' else '-', '--out',str(out/mode)]
    start = time.monotonic()
    result = subprocess.run(command, input=journal if mode=='pipe' else None, text=True,
                            capture_output=True, env=environment, timeout=120)
    (out/f'{mode}.log').write_text(result.stdout+result.stderr)
    assert result.returncode == 0, result.stderr
    (out/f'{mode}_command.json').write_text(json.dumps(dict(command=command, seconds=time.monotonic()-start),indent=2))
products = ['trajectory.tum','states.csv','online_trajectory.tum','online_states.csv',
            'support_clock.csv','support_accelerations.csv','model_covariance.csv','source_model_covariance.csv']
identity = {}
for name in products:
    payload = (out/'file'/name).read_bytes()
    assert payload == (out/'pipe'/name).read_bytes(), name
    identity[name] = hashlib.sha256(payload).hexdigest()
# Check that removing expensive diagnostic snapshots did not change previously
# validated online publications. The final retrospective prefix is not compared
# against a future-informed full-run final trajectory.
for name in ['online_trajectory.tum','online_states.csv']:
    got = (out/'file'/name).read_bytes().splitlines()
    previous = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/off_wnoa')/name
    assert got == previous.read_bytes().splitlines()[:len(got)], name+' prior online prefix'
summary = json.loads((out/'file/summary.json').read_text())
assert summary['imu_factor_count']==0 and summary['receiver_model']=='source_wnoa'
assert summary['pending_at_end']==0
(out/'validation.json').write_text(json.dumps(dict(passed=True, byte_identity=identity,
    previous_online_prefix_identical=True, summary=summary),indent=2)+'\n')
print('PASS: file/pipe and previously validated online prefix are byte-identical')
