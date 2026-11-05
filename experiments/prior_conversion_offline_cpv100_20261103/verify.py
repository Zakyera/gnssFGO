"""Read-only integrity/exit audit, writes new CP100 terminal receipt only."""
import hashlib
import json
from pathlib import Path
import subprocess
import datetime
H=Path(__file__).resolve().parent
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def verify(pins):
    for p,h in pins.items():assert sha(p)==h,p
    return len(pins)
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'complete.json').exists()
counts=[verify(read(d/'inputs.json')) for d in (H,H/'nearby_stage1')]
analysis=read(H/'analysis.json');verify(analysis['diagnostic_artifact_sha256'])
jobs=[]
for d in (H,H/'nearby_stage1'):
    for p in [d/'start.json',*sorted(d.glob('*_process.json'))]:
        r=read(p);assert identity(r['pid'])!=r['proc_start'],p
        jobs.append(dict(receipt=str(p),pid=r['pid'],recorded_start=r['proc_start'],exited=True))
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
pins={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file()}
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
         finding='Unchanged-anchor numerical conversion can falsely reject identical prior/cache.',
         actual_cp99_cause_proven=False,production_fix_applied=False,source_runs=0,
         optimizer_runs=0,estimator_builds=0,standalone_diagnostic_builds=2,
         original_protected_files=6425,input_pin_counts=counts,
         all_owned_jobs_exited=True,owned_jobs=jobs,containers=containers,hesai=hesai,
         output_sha256=pins)
with (H/'complete.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('owned_jobs','output_sha256')},indent=2))
