"""Verify delivered recordings and write portable checksums and final receipt."""
import hashlib
import json
import subprocess
from datetime import datetime
from pathlib import Path

H = Path(__file__).resolve().parent
OUT = Path('/home/yeranis/results/rerun_method2_all_sequences_20261029')
def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        while data := f.read(8*1024*1024): h.update(data)
    return h.hexdigest()

v = json.loads((H/'verification.json').read_text())
assert v['passed'] and len(v['cases']) == 7
lines = []
for case in v['cases']:
    path = Path(case['rrd'])
    assert sha(path) == case['rrd_sha256']
    lines.append(f"{case['rrd_sha256']}  {path.name}\n")
with (OUT/'SHA256SUMS').open('x') as f: f.writelines(lines)
subprocess.run(['bash','-n',str(OUT/'open_rerun.sh')],check=True)
checkpoint = subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True).strip()
assert checkpoint.startswith('PASS:')
receipt = dict(completed=datetime.now().astimezone().isoformat(),passed=True,
    recordings=7,rrd_bytes=sum(x['rrd_bytes'] for x in v['cases']),
    hesai_checkpoint=checkpoint,outputs={str(p):sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
    inputs_unchanged=v['all_inputs_unchanged'],input_count=v['input_count'],
    source_or_receiver_runs=0,default_or_model_changes=False,
    visual_check='Seven native default OFF renders inspected; ON geometry exact readback.',
    screenshot_limit='Known Rerun screenshot shutdown panic after images saved; receipts retained.')
with (H/'closeout.json').open('x') as f: json.dump(receipt,f,indent=2)
print(json.dumps({k:val for k,val in receipt.items() if k!='outputs'},indent=2))
