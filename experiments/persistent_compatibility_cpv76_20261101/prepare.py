"""Exclusive mechanical source copy and immutable predecessor manifest."""
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
A=W/'runs/retention_readiness_cpv73_20261031';P=W/'runs'/H.name
P.mkdir(exist_ok=False)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins=json.loads((H.parent/'persistent_mode_compatibility_cpv75_20261031/input_sha256.json').read_text())
sources=[*A.glob('*.h'),*A.glob('*.inc'),*A.glob('*.cpp'),*A.glob('include/**/*.h'),A/'build.py']
for p in sources:
 pins[str(p)]=sha(p);q=P/p.relative_to(A);q.parent.mkdir(parents=True,exist_ok=True)
 q.write_text(p.read_text().replace(str(A),str(P)).replace('/workspace/cbs_gtsam4.3/runs/'+A.name,'/workspace/cbs_gtsam4.3/runs/'+P.name))
shutil.copy2(P/'test.cpp',P/'legacy_test.inc')
(P/'private_build').mkdir()
shutil.copy2(A/'private_build/libcbs.so',P/'private_build/libcbs.so')
pins[str(A/'private_build/libcbs.so')]=sha(A/'private_build/libcbs.so')
with (H/'protected_before.json').open('x') as f:json.dump(pins,f,indent=2)
print(P,len(pins))
