"""Supplemental terminal audit; never rewrites completed evidence."""
import datetime,hashlib,json,subprocess
from pathlib import Path
from paths import H,E,P,BASE_H
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def verify(pins):
    for p,d in pins.items():assert sha(p)==d,p
    return len(pins)
assert not (H/'post_closeout.json').exists() and read(H/'complete.json')['passed']
out=dict(passed=True,checked_at=datetime.datetime.now().astimezone().isoformat())
out['protected_files']=verify(read(P/'protected_manifest.json'))
out['terminal_files']=verify(read(H/'complete.json')['output_sha256'])
out['original_terminal_files']=verify(read(BASE_H/'complete.json')['output_sha256'])
out['report_inputs']=verify(read(H/'evaluation/input_sha256.json'))
out['repeat_inputs']=verify(read(H/'repeat_evaluation/input_sha256.json'))
jobs=[]
for f in [H/'start.json',*H.glob('*_process.json'),*(P/'full_off_control').glob('*_process.json'),*(P/'full_on_control').glob('*_process.json')]:
    z=read(f);stat=Path('/proc')/str(z['pid'])/'stat'
    live=stat.exists() and stat.read_text().split()[21]==str(z['proc_start'])
    assert not live,f;jobs.append(dict(receipt=str(f),pid=z['pid'],original_identity_alive=False))
launch=read(H/'launch.json');pid=launch['owned_supervisor_pid'];stat=Path('/proc')/str(pid)/'stat'
assert not stat.exists() or stat.read_text().split()[21]!=str(launch['supervisor_proc_start'])
out['owned_jobs']=jobs;out['supervisor_exited']=pid
out['containers']=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert out['containers']==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
out['hesai']=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
with (H/'post_closeout.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
