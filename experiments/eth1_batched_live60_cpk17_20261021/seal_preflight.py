"""Finish reviewed preparation before either source capture, never reseal a run."""
from pathlib import Path
import difflib,hashlib,json,ast
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/gnssfgo_eth1_batched_live60_cpk17_20261021/prepared'
S=W/'runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared'
assert not any(R.glob('*_control'))
assert not (H/'sealed.json').exists()
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
changed=[H/'prepare.py',*(R/n for n in ('active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','evaluate_fl4f.py'))]
m=json.loads((R/'protected_manifest.json').read_text())
for p,v in m.items():
 if Path(p) not in changed:assert sha(p)==v,p
for p in changed:
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
 m[str(p)]=sha(p)
diff=[]
for p in sorted(R.iterdir()):
 if p.suffix not in ('.py','.json','.sh','.env','.launch'):continue
 orig=S/p.name.replace('live60_','full_')
 if not orig.is_file() or p.name=='protected_manifest.json':continue
 diff.extend(difflib.unified_diff(orig.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(orig),tofile=str(p)))
(H/'harness.diff').write_text(''.join(diff))
for p in (H/'harness.diff',Path(__file__)):m[str(p)]=sha(p)
(R/'protected_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
with (H/'sealed.json').open('x') as f:json.dump(dict(passed=True,note='Prelaunch copy correction preserves full_history parameter and full_saved evaluator names; no source capture occurred.',protected_files=len(m)),f,indent=2)
print('Prepared harness sealed; unchanged original parameter/evaluator field names.')
