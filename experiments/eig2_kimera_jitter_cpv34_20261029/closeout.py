"""Finalize read-only diagnosis integrity, without starting any experiment."""
import ast
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

H=Path(__file__).resolve().parent
E=H.parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
v=json.loads((H/'verification.json').read_text())
for p,s in v['input_sha256'].items():assert sha(p)==s,p
assert sha(H/'window_mechanism.png')==v['figure_sha256']
for p in H.glob('*.py'):ast.parse(p.read_text(),filename=str(p))
checkpoint=subprocess.run(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],check=True,text=True,capture_output=True)
outputs={}
for old in ['all_method1_rerun_cpv33_20261029','all_method2_rerun_cpv30_20261029']:
    c=json.loads((E/old/'closeout.json').read_text())
    hashes=c['output_sha256'] if 'output_sha256' in c else c['outputs']
    for p,s in hashes.items():assert sha(p)==s,p
    outputs[old]=len(hashes)
subprocess.run(['git','diff','--check','--','AGENTS.md','GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md'],cwd=E.parent,check=True)
receipt=dict(completed_utc=datetime.now(timezone.utc).isoformat(),passed=True,
 status='Saved-data mechanism diagnosis complete; exact root cause unresolved; proposed replay not launched.',
 inputs_unchanged=True,input_count=len(v['input_sha256']),
 hesai_checkpoint=checkpoint.stdout.strip(),original_rerun_outputs_unchanged=outputs,
 source_or_receiver_runs=0,builds=0,parameter_or_model_changes=False,
 visual_check='window_mechanism.png inspected; diagnostic rigid overlay is not a scoring alignment.',
 evidence_sha256={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()},
 documentation_sha256={str(p):sha(p) for p in [E.parent/'AGENTS.md',E.parent/'GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md']})
with (H/'closeout.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if not k.endswith('sha256')},indent=2))
