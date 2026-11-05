"""Verify preserved inputs and close out CP56, retaining its failed strict gate."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'measure_gate_failure/results'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins=json.loads((H/'measure_gate_failure/protected_manifest.json').read_text())
for p,h in pins.items():assert sha(p)==h,p
measurement=json.loads((H/'measure_gate_failure/measurement_completed.json').read_text())
assert measurement['completed'] and measurement['snapshot_nonmutation']==4
summary=json.loads((P/'summary.json').read_text())
analysis=json.loads((H/'analysis.json').read_text())
assert summary['completed'] and analysis['completed']
assert summary['passed']==analysis['strict_gauge_passed']==measurement['strict_gauge_passed']
for p,h in analysis['inputs'].items():assert sha(p)==h,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',containers
cleanup=json.loads((H/'measure_gate_failure/cleanup.json').read_text())
assert all(row.split()[1] in ('tail','zsh') for row in cleanup['jobs'].splitlines()[1:] if row.strip())
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
verification=dict(preservation_passed=True,diagnostic_completed=True,strict_gauge_passed=summary['passed'],protected_files=len(pins),containers=containers,owned_container_jobs_exited=True,hesai=hesai,source_replays=0,graph_optimizer_calls=0,production_changes=0,time=datetime.datetime.now().astimezone().isoformat())
with (H/'verification.json').open('x') as f:json.dump(verification,f,indent=2)
print(json.dumps(verification,indent=2))
