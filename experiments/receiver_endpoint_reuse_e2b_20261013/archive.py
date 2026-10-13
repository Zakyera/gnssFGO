"""Archive small evidence, identity and patch; never overwrite original results."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013')
OLD=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate')
OUT=HERE/'evidence';OUT.mkdir()
for p in ROOT.iterdir():
    if p.suffix in ['.json','.xml','.log']:shutil.copy2(p,OUT/p.name)
patch=[]
for name in ['SourceWnoaModel.h','DiagnosticReplay.cpp','test_twist.cpp']:
    patch.extend(difflib.unified_diff((OLD/name).read_text().splitlines(True),
                 (HERE/name).read_text().splitlines(True),str(OLD/name),str(HERE/name)))
with (OUT/'private_changes.diff').open('x') as f:f.writelines(patch)
hashes={}
for path in sorted(ROOT.rglob('*')):
    if path.is_file() and path.name!='controller.lock':
        h=hashlib.sha256()
        with path.open('rb') as f:
            for chunk in iter(lambda:f.read(1<<20),b''):h.update(chunk)
        hashes[str(path)]=dict(bytes=path.stat().st_size,sha256=h.hexdigest())
with (OUT/'artifact_manifest.json').open('x') as f:json.dump(hashes,f,indent=2)
print('Archived evidence;',len(hashes),'files hashed')
