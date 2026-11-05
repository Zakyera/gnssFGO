"""Record detected full-window count leftover before any launch; preserve old pins."""
import difflib,hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
assert not (H/'start.json').exists() and not (H/'launch.json').exists()
assert not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
old=json.loads((H/'prelaunch_original/protected_manifest.json').read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
allowed={str(P/'validate_local.py'),str(H/'prepare.py')};changes={}
for p,want in old.items():
 got=sha(p)
 if got!=want:
  assert p in allowed,p;changes[p]=dict(before=want,after=got)
assert set(changes)==allowed
for p,v in changes.items():old[p]=v['after']
old[str(H/'prelaunch_finalize.py')]=sha(H/'prelaunch_finalize.py')
with (H/'prelaunch_count_correction.diff').open('x') as f:
 for p in sorted(allowed):
  before=H/'prelaunch_original'/Path(p).name
  f.writelines(difflib.unified_diff(before.read_text().splitlines(True),Path(p).read_text().splitlines(True),fromfile=str(before),tofile=p))
(P/'protected_manifest.json').write_text(json.dumps(old,indent=2))
with (H/'prelaunch_review.json').open('x') as f:json.dump(dict(passed=True,no_source_started=True,
 reason='Copied LOCAL validator retained full 355s count3530. Prefix has100 raw scans, CP-V49 first10s native93; require90 native outputs and90 first exports, preserving all provenance/lifecycle/coverage tests.',
 changes=changes,protected_files=len(old)),f,indent=2)
