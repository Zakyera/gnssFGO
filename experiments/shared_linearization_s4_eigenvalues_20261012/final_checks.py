"""Final archive/format checks; numerical gates live in run.py and analyze.py."""
from pathlib import Path
import ast
import hashlib
import json
import re

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s4_eigenvalues_20261012')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p in HERE.glob('*.py'):ast.parse(p.read_text())
for p in HERE.rglob('*.json'):json.loads(p.read_text())
for p,digest in json.loads((HERE/'hashes.json').read_text()).items():assert sha(p)==digest,p
for name in ['build.py','run.py','rank_tests.cpp','environment.py']:
    assert (HERE/name).read_bytes()==(ROOT/'driver'/name).read_bytes(),name
for name in ['NativeGaussianHistory.cpp','NativeGaussianHistory.h','test_native_gaussian.cpp']:
    src=HERE.parent/'shared_linearization_s2_20261012'/name
    assert src.read_bytes()==(ROOT/'driver'/name).read_bytes(),name
for link in re.findall(r'\]\(([^)]+)\)',(HERE/'REPORT.md').read_text()):
    assert (HERE/link).exists(),link
result=dict(passed=True,python_syntax=True,json_parse=True,identity_pins=True,
            archived_driver_matches=True,unchanged_s2_sources=True,report_links_exist=True)
with (HERE/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
