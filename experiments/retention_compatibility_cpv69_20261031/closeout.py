"""Read-only preservation/owned-process/container closeout, writes new receipt."""
import datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(name,v):
 with (H/name).open('x') as f:json.dump(v,f,indent=2)
assert not (H/'complete.json').exists()
pins={}
for p in H.glob('*/protected_before.json'):pins.update(json.loads(p.read_text()))
for p,v in pins.items():assert sha(p)==v,p
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
owned=[]
for p in H.glob('*/*_process.json'):
 d=json.loads(p.read_text());pid=d['pid'];proc=Path('/proc')/str(pid)/'cmdline'
 cmd=proc.read_bytes().replace(b'\0',b' ').decode(errors='replace') if proc.exists() else ''
 # A recycled PID is not our process: check its concrete command identity.
 assert not (H.name in cmd and ('docker exec' in cmd or 'build.py' in cmd or 'run_case.py' in cmd)),(pid,cmd)
 owned.append(dict(receipt=str(p),pid=pid,original_job_exited=True,pid_reused=bool(cmd)))
checkpoint=R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
with (H/'hesai_checkpoint.log').open('x') as f:
 p=subprocess.run(['python3','-B',str(checkpoint),'--originals'],stdout=f,stderr=subprocess.STDOUT)
assert p.returncode==0
check=(H/'hesai_checkpoint.log').read_text();assert 'PASS: 4 cases, 667 saved files; originals=True' in check
results=json.loads((H/'results.json').read_text()) if (H/'results.json').exists() else None
receipt=dict(passed=True,phase1_compatibility_passed=True,phase2_comparison_passed=bool(results and results['passed']),
 source_failures_preserved=True,protected_files=len(pins),containers=states,owned_jobs=owned,hesai_checkpoint=check.strip(),
 backend_sha256=sha(K/'private_build/libkimera_vio.so'),production_promotion=False,
 completed=datetime.datetime.now().astimezone().isoformat())
save('verification.json',receipt)
save('complete.json',dict(**receipt,output_sha256={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file()}))
print(json.dumps({k:v for k,v in receipt.items() if k!='owned_jobs'},indent=2))
