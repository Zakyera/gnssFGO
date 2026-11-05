"""Read-only integrity/owned-job closeout; write only new verification receipts."""
import datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,d):
 with (H/n).open('x') as f:json.dump(d,f,indent=2)
assert json.loads((H/'capture_complete.json').read_text())['passed']
assert json.loads((H/'closeout.json').read_text())['passed']
pins=json.loads((P/'protected_manifest.json').read_text())
for p,w in pins.items():assert sha(p)==w,p
analysis=json.loads((H/'analysis.json').read_text())
for p,w in analysis['input_sha256'].items():assert sha(p)==w,p
checks=[]
for mode in ('off','on'):
 for p in [H/(mode+'_glim_observer.json'),H/(mode+'_snapshots.json'),H/(mode+'_validated.json')]:assert json.loads(p.read_text())['passed']
 d=P/f'full_{mode}_receiver';t=json.loads((d/'transport_status.json').read_text())
 assert (d/'complete.marker').exists() and t['queued']==0 and t['duplicates']==0
 assert not (P/f'full_{mode}_control/failed.txt').exists()
 checks.append(dict(mode=mode,transport=t))
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
jobs=[]
for p in [H/'launch.json',H/'start.json',*H.glob('*_process.json')]:
 d=json.loads(p.read_text());pid=d.get('pid');assert pid
 proc=Path(f'/proc/{pid}/cmdline');text=proc.read_bytes().replace(b'\0',b' ').decode(errors='replace') if proc.exists() else ''
 active=bool(text) and (H.name in text or 'docker exec' in text)
 jobs.append(dict(receipt=str(p),pid=pid,present=proc.exists(),active_owned=active,current_cmdline=text))
 assert not active,(p,pid,text)
hesai=subprocess.check_output(['python3',str(H.parents[1]/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
retained={}
for root in (P,Path('/home/yeranis/results')/H.name):
 for p in root.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts and not p.is_symlink():retained[str(p)]=sha(p)
for p in H.iterdir():
 if p.is_file() and p.name not in ('controller.log','status.json'):retained[str(p)]=sha(p)
save('retained_manifest.json',retained)
stamp=datetime.datetime.now().astimezone().isoformat()
save('verification.json',dict(passed=True,time=stamp,protected_files=len(pins),analysis_inputs=len(analysis['input_sha256']),retained_files=len(retained),containers=containers,jobs=jobs,hesai=hesai,modes=checks))
save('complete.json',dict(passed=True,finished=stamp,scope='One10s OFF/ON passive native GLIM first-edge observation; no full-sequence scoring',analysis='analysis.json',report='REPORT.txt',verification='verification.json',production_promoted=False))
print(json.dumps(dict(passed=True,time=stamp,protected_files=len(pins),retained_files=len(retained),containers=containers,hesai=hesai),indent=2))
