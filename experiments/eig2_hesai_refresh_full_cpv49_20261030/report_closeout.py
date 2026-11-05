"""Read-only evidence verification followed by exclusive final report receipt."""
import datetime,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not (H/'complete.json').exists()
assert read(H/'results.json')['passed'] and read(H/'capture_complete.json')['passed']
assert read(H/'closeout.json')['passed']
assert not (H/'failed.json').exists() and not (H/'cleanup_failed.json').exists()
protected=read(P/'protected_manifest.json');inputs=read(H/'evaluation/input_sha256.json')
for mapping in (protected,inputs):
    for p,want in mapping.items():assert sha(p)==want,p
for c in read(H/'evaluation/commands.json'):assert c['exit']==0
for p in H.glob('*_receipt.json'):assert read(p)['exit']==0,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert all(s.endswith(' exited') for s in containers)
for receipt in ('launch.json','start.json'):
    pid=read(H/receipt)['pid'];proc=Path(f'/proc/{pid}/cmdline')
    assert not proc.exists() or str(H).encode() not in proc.read_bytes(),pid
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
outputs={str(p):sha(p) for p in (H/'results.json',H/'RESULTS_REPORT.txt',H/'REPORT.txt',H/'evaluate.py',Path(__file__))}
for p in (H/'evaluation').rglob('*'):
    if p.is_file():outputs[str(p)]=sha(p)
final=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
    captures_complete=True,all_gates_passed=True,accuracy_scored=True,
    conclusion='Fresh corrected Hesai matched pair scored; see results.json for measured OFF/ON outcomes. No default promotion.',
    containers=containers,owned_launcher_and_controller_exited=True,
    protected_files_verified=len(protected),report_inputs_verified=len(inputs),hesai=hesai,
    evo_cli_calls=len(read(H/'evaluation/commands.json')),output_sha256=outputs)
with (H/'complete.json').open('x') as f:json.dump(final,f,indent=2)
print(json.dumps({k:v for k,v in final.items() if k!='output_sha256'},indent=2))
