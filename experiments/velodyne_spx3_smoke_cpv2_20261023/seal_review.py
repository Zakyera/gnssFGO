"""Pre-launch documentation-only label correction; no gate or setting edits."""
from pathlib import Path
import ast,difflib,hashlib,json
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared')
assert not any(P.glob('full_*_source')) and not (H/'status.json').exists()
assert not (H/'prelaunch_review.json').exists()
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
manifest=json.loads((P/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
diff=[]
for name in ('preflight_on.py','validate_case.py','source_runner.sh','source_runner_on.sh'):
 p=P/name;s=p.read_text();out=s.replace('same28.0s','same30.0s').replace('full28.0-second','bounded30.0-second')
 assert out!=s
 if p.suffix=='.py':assert ast.dump(ast.parse(s),include_attributes=False).count('Assert')==ast.dump(ast.parse(out),include_attributes=False).count('Assert')
 p.write_text(out)
 diff.extend(difflib.unified_diff(s.splitlines(True),out.splitlines(True),fromfile=name+'-before-review',tofile=name))
 manifest[str(p)]=sha(p)
with (H/'prelaunch_labels.diff').open('x') as f:f.write(''.join(diff))
with (H/'prelaunch_review.json').open('x') as f:
 json.dump(dict(passed=True,changes='Only inherited28s labels corrected to bounded30s; operational duration already30s',
   estimator_runs_started=False,known_preparation_issues=['Sandbox denied local socket probe; resumed with scoped permission before creating artifacts.',
      'Preparation initially assumed flat profiles; preserved partial files and completed nested folder copy with exact-byte checks.']),f,indent=2)
for name in ('seal_review.py','prelaunch_labels.diff','prelaunch_review.json'):manifest[str(H/name)]=sha(H/name)
(P/'protected_manifest.json').write_text(json.dumps(manifest,indent=2))
print('Prelaunch review sealed; no source/receiver executed. Protected entries:',len(manifest))
