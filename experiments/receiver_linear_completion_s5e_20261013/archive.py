"""Retain small evidence plus hashes; never overwrite source artifacts."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_linear_completion_s5e_20261013')
BASE=HERE.parent/'receiver_endpoint_reuse_e2b_20261013'
for action in ['before','unit_consistent','sanitize_consistent','prefix_off','prefix_on','full_on','after','after_final']:
    assert json.loads((ROOT/(action+'.json')).read_text())['passed'],action
assert json.loads((ROOT/'analysis.json').read_text())['passed']
OUT=HERE/'evidence';OUT.mkdir(exist_ok=True)
for p in ROOT.iterdir():
    if p.suffix in ['.json','.xml','.log']:
        if (OUT/p.name).exists():assert (OUT/p.name).read_bytes()==p.read_bytes(),p
        else:shutil.copy2(p,OUT/p.name)
patch=[]
for name in ['DiagnosticReplay.cpp','SourceWnoaModel.h','SnapshotAudit.h','test_endpoints.cpp']:
    patch.extend(difflib.unified_diff((BASE/name).read_text().splitlines(True),(HERE/name).read_text().splitlines(True),str(BASE/name),str(HERE/name)))
if (OUT/'private_changes.diff').exists():assert (OUT/'private_changes.diff').read_text()==''.join(patch)
else:
    with (OUT/'private_changes.diff').open('x') as f:f.writelines(patch)
manifest={}
for p in sorted(ROOT.rglob('*')):
    if p.is_file() and p.name!='controller.lock':
        h=hashlib.sha256()
        with p.open('rb') as f:
            for b in iter(lambda:f.read(1048576),b''):h.update(b)
        manifest[str(p)]=dict(bytes=p.stat().st_size,sha256=h.hexdigest())
with (OUT/'artifact_manifest_final.json').open('x') as f:json.dump(manifest,f,indent=2)
print('Archived',len(manifest),'files by hash')
