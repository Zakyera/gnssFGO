"""Supplemental terminal audit only, once controller and supervisor have exited."""
import datetime,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not (H/'post_closeout.json').exists()
assert read(H/'complete.json')['passed']
exits=[]
for file in [H/'start.json',*H.glob('*_process.json')]:
    v=read(file);p=Path('/proc')/str(v['pid'])/'stat'
    same=p.exists() and p.read_text().split()[21]==v['proc_start'];assert not same,v
    exits.append(dict(receipt=str(file),pid=v['pid'],proc_start=v['proc_start'],same_identity_present=False))
launch=read(H/'launch.json');pid=launch['supervisor_pid'];cmd=Path('/proc')/str(pid)/'cmdline'
assert not cmd.exists() or str(H/'controller.py').encode() not in cmd.read_bytes()
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
counts={}
for name,pins in [('protected',read(P/'protected_manifest.json')),('outputs',read(H/'complete.json')['output_sha256']),('report_inputs',read(H/'evaluation/input_sha256.json'))]:
    for p,d in pins.items():assert sha(p)==d,p
    counts[name]=len(pins)
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
v=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),counts=counts,containers=states,owned_exits=exits,
       supervisor_pid=pid,supervisor_command_exited=True,hesai=hesai,single_new_source_capture=True)
with (H/'post_closeout.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v,indent=2))
