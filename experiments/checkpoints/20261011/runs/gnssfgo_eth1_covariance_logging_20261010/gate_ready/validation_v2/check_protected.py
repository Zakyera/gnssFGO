import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
manifest=ROOT/'protected_manifest.json'
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
entries=json.loads(manifest.read_text())
for path,expected in entries.items():assert sha(path)==expected,path
out=dict(passed=True,entries=len(entries),manifest_sha256=sha(manifest))
with (ROOT/'protected_validation.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out))
