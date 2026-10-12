"""Verify protected bytes, timer-only transformation, and archived results."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_timing_20261012'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
manifest=json.loads((REPO/'experiments/shared_linearization_s3_capture_20261012/evidence/capture/protected_manifest.json').read_text())
for p,digest in manifest.items():assert sha(p)==digest,p
original=(REPO/'experiments/shared_linearization_s2_20261012/NativeGaussianHistory.cpp').read_text()
instrumented=(ROOT/'release/instrumented.cpp').read_text()
stripped=''.join(line for line in instrumented.splitlines(keepends=True) if line!='#include "Timing.h"\n' and 'diagnostic_timing::Span timing(' not in line and 'timing.mark(' not in line)
assert stripped==original,'non-timer source change'
for p in HERE.glob('*.py'):ast.parse(p.read_text())
for p in HERE.rglob('*.json'):json.loads(p.read_text())
identity=json.loads((HERE/'hashes.json').read_text())
for p,digest in identity.items():assert sha(p)==digest,p
shutil.copy2(ROOT/'environment.json',HERE/'evidence/environment.json')
result=dict(passed=True,protected_files=len(manifest),source_transformation='timer lines only; stripped source byte-identical',python_syntax=True,json_parse=True,identity_files=len(identity),no_source_run=True)
with (HERE/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
