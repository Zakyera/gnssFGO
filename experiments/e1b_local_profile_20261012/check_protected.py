#!/usr/bin/env python3
"""Verify build inputs in their owning container filesystem, read-only."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

result={}
for name in ('release_2','ubsan_1'):
    protected=json.loads((ROOT/name/'protected.json').read_text())
    mismatch=[p for p,h in protected.items() if sha(p)!=h]
    assert not mismatch,mismatch
    result[name]=dict(inputs=len(protected),mismatches=mismatch,
                      manifest_sha256=sha(ROOT/name/'protected.json'))
with (ROOT/'protected_verified.json').open('x') as f:
    json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result,indent=2))
