"""One supplemental terminal audit after exact controller/supervisor exit."""
import datetime,hashlib,json,subprocess
from pathlib import Path
from paths import H,E,W,P,FIX_H
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def identity(pid):
    p=Path(f'/proc/{pid}/stat');return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'post_closeout.json').exists()
assert (H/'complete.json').exists() or (H/'failed.json').exists() or (H/'report_failed.json').exists()
jobs=[]
for p in [H/'start.json',*H.glob('*_process.json'),*P.glob('full_*_control/*_process.json')]:
    r=read(p);assert identity(r['pid'])!=str(r['proc_start']),p
    jobs.append(dict(receipt=str(p),pid=r['pid'],proc_start=r['proc_start'],args=r.get('args',r.get('command')),exited=True))
r=read(H/'launch.json');assert identity(r['owned_supervisor_pid'])!=str(r['supervisor_proc_start'])
jobs.append(dict(receipt=str(H/'launch.json'),pid=r['owned_supervisor_pid'],proc_start=r['supervisor_proc_start'],args=r['args'],exited=True))
from preserve_failed import verify_special
verify_special()
pins=read(P/'protected_manifest.json')
for p,v in pins.items():assert sha(p)==v,p
sealed=0
if (H/'complete.json').exists():
    entries=read(H/'complete.json')['output_sha256'];sealed=len(entries)
    for p,v in entries.items():assert sha(p)==v,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
         experiment_passed=(H/'complete.json').exists(),protected_files=len(pins),sealed_outputs_verified=sealed,
         containers=containers,hesai=hesai,owned_jobs=jobs,all_owned_jobs_exited=True,
         source_wrappers_started=sum((P/f'full_{mode}_control/source_process.json').exists() for mode in ('off','on')),
         post_audit_source_runs=0)
with (H/'post_closeout.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='owned_jobs'},indent=2))
