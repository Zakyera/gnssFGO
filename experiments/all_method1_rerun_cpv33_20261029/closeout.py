"""Final delivery hashes, preserved Method2 collection and original checkpoint."""
import hashlib
import json
import subprocess
from datetime import datetime
from pathlib import Path

H = Path(__file__).resolve().parent
OUT = Path('/home/yeranis/results/rerun_method1_all_sequences_20261029')
def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while data := f.read(8*1024*1024): h.update(data)
    return h.hexdigest()

v = json.loads((H/'verification.json').read_text())
inputs = json.loads((H/'input_checks.json').read_text())
renders = json.loads((H/'render_receipts_host.json').read_text())
assert v['passed'] and len(v['cases']) == len(renders) == 7
assert all(sha(p)==digest for p,digest in inputs['input_sha256'].items())
assert all(sha(p)==digest for p,digest in inputs['method2_outputs'].items())
lines = []
for case in v['cases']:
    path = Path(case['rrd'])
    assert sha(path) == case['rrd_sha256']
    lines.append(f"{case['rrd_sha256']}  {path.name}\n")
for case in renders:
    assert case['returncode'] == 102
    assert Path(case['screenshot']).stat().st_size == case['image_bytes'] > 10000
    log = (H/(case['case']+'_screenshot_host.log')).read_text()
    assert 'Failed to take store hub from the Viewer' in log
with (OUT/'SHA256SUMS').open('x') as f: f.writelines(lines)
subprocess.run(['bash','-n',str(OUT/'open_rerun.sh')],check=True)
checkpoint = subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True).strip()
assert checkpoint.startswith('PASS:')
receipt = dict(completed=datetime.now().astimezone().isoformat(),passed=True,
    recordings=7,rrd_bytes=sum(x['rrd_bytes'] for x in v['cases']),
    hesai_checkpoint=checkpoint,
    output_sha256={str(p):sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
    evidence_sha256={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()},
    inputs_unchanged=True,input_count=len(inputs['input_sha256']),
    method2_outputs_unchanged=True,method2_output_count=len(inputs['method2_outputs']),
    source_or_receiver_runs=0,default_or_model_changes=False,
    visual_check='Seven native default OFF renders inspected; ON geometry exact readback.',
    screenshot_limit='All seven screenshots saved then known shutdown panic102; receipts retained.',
    owned_screenshot_subprocesses_exited=True)
with (H/'closeout.json').open('x') as f: json.dump(receipt,f,indent=2)
print(json.dumps({k:val for k,val in receipt.items() if not k.endswith('sha256')},indent=2))
