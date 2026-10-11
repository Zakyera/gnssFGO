#!/usr/bin/env python3
"""Archive scoped, small E4 evidence; never overwrite earlier evidence."""
import hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent
DEST=Path('/home/yeranis/repos/gnssFGO/experiments/e4_selective_refinement_20261012')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
paths=[p for p in ROOT.iterdir() if p.is_file() and p.suffix in ['.json','.xml','.log','.py']]
paths += [p for p in (ROOT/'build').rglob('*') if p.is_file() and p.suffix in ['.h','.cpp']]
for tag in ['reference','candidate','candidate_repeat']:
    if not (ROOT/tag).exists():continue
    for name in ['summary.json','publication_convergence.csv','publication_steps.csv','e4_refinement.csv']:
        p=ROOT/tag/name
        if p.exists():paths.append(p)
manifest=[]
for p in sorted(paths):
    target=DEST/'evidence'/p.relative_to(ROOT)
    assert not target.exists(),target
    assert p.stat().st_size<3_000_000,p
    target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,target)
    assert sha(target)==sha(p)
    manifest.append(dict(original=str(p),archive=str(target.relative_to(DEST)),sha256=sha(p),bytes=p.stat().st_size))
with (DEST/'ARCHIVE_MANIFEST.json').open('x') as f:json.dump(manifest,f,indent=2)
print('Archived',len(manifest),'files;',sum(r['bytes'] for r in manifest),'bytes')
