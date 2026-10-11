"""Archive small evidence without changing frozen inputs or replay outputs."""
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
OUT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_validation_20261011')
assert json.loads((OUT/'final_checks.json').read_text())['passed']
dest=HERE/'evidence';dest.mkdir(exist_ok=False)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
copied={}
for p in sorted(OUT.iterdir()):
    if not p.is_file() or p.suffix not in ['.json','.txt','.log','.xml','.diff','.py']:continue
    assert p.stat().st_size<5_000_000,(p,p.stat().st_size)
    target=dest/p.name;shutil.copy2(p,target)
    assert sha(target)==sha(p)
    copied[str(target.relative_to(HERE))]=dict(original=str(p),bytes=p.stat().st_size,sha256=sha(p))
rt=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate')
manifest=dict(status='FUNCTIONAL_VALIDATION_PASSED_HEADROOM_MISSED',copied=copied,
              tested_binary_sha256={str(rt/n):sha(rt/n) for n in ['replay','unit','sanitize']},
              large_artifacts='Original live capture and single saved-journal replay remain at run paths; no binaries/bags/large trajectories in this archive.')
with (HERE/'ARCHIVE_MANIFEST.json').open('x') as f:json.dump(manifest,f,indent=2)
print('Archived',len(copied),'files,',sum(r['bytes'] for r in copied.values()),'bytes, byte-identical.')
