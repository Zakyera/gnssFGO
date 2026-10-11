"""Frozen validator logic, with only explicit result/replay path redirection.

Save the transformed code and diff for audit. Original inputs stay read-only.
"""
import argparse
import difflib
import fcntl
import hashlib
import json
from pathlib import Path
import sys

CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011')
OUT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_validation_20261011')
p=argparse.ArgumentParser();p.add_argument('gate',choices=['lifecycle','diagnostics','numerical']);a=p.parse_args()
lock=(CAPTURE/'prepared/controller.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
path=CAPTURE/({'lifecycle':'check_lifecycle.py','diagnostics':'validate_diagnostics.py',
               'numerical':'prepared/validate_numerical_case.py'}[a.gate])
original=path.read_text();code=original
replacements={}
if a.gate in ['lifecycle','diagnostics']:
    token="ROOT/(tag+'_"+a.gate+"_validation.json')"
    replacements[token]="OUT/(tag+'_"+a.gate+"_validation.json')"
else:
    for suffix in ['native_validation.json','lifecycle_validation.json','diagnostics_validation.json','numerical_repeat','numerical_repeat.log']:
        replacements["ROOT/(tag+'_"+suffix+"')"]="OUT/(tag+'_"+suffix+"')"
    replacements["ROOT/(phase+'_'+mode+'_validation.json')"]="OUT/(phase+'_'+mode+'_validation.json')"
    for stage in ['native','lifecycle','diagnostics']:
        assert json.loads((OUT/('live160_on_'+stage+'_validation.json')).read_text())['passed']
for x,y in replacements.items():
    assert x in code,x
    code=code.replace(x,y)
manifest=dict(original=str(path),original_sha256=hashlib.sha256(original.encode()).hexdigest(),
              transformed_sha256=hashlib.sha256(code.encode()).hexdigest(),replacements=replacements)
for suffix,data in [('py',code),('diff',''.join(difflib.unified_diff(original.splitlines(True),code.splitlines(True)))),
                    ('json',json.dumps(manifest,indent=2))]:
    with (OUT/(a.gate+'_retained.'+suffix)).open('x') as f:f.write(data)
sys.path.insert(0,str(path.parent));sys.argv=[str(path),'live160_on']
exec(compile(code,str(path),'exec'),dict(__name__='__main__',__file__=str(path),OUT=OUT))
