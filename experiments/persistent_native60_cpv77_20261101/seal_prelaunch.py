"""Correct diagnostic decision-column index BEFORE first replay; preserve v0."""
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
assert not (H/'replay_attempt').exists()
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins=json.loads((H/'protected_before.json').read_text())
assert all(sha(p)==v for p,v in pins.items())
D=H/'prelaunch_version0';D.mkdir()
for name,p in [('host_validate.py',H/'validate.py'),('replay_validate.py',K/'replay/validate.py'),('controller.py',H/'controller.py')]:
    shutil.copyfile(p,D/name)
    s=p.read_text()
    if 'validate' in name:
        assert "decisions[v[12]]+=1" in s;s=s.replace("decisions[v[12]]+=1","decisions[v[11]]+=1")
    else:s=s.replace("H/'protected_before.json'","H/'protected_prelaunch.json'")
    p.write_text(s);pins[str(p)]=sha(p);pins[str(D/name)]=sha(D/name)
pins[str(H/'protected_before.json')]=sha(H/'protected_before.json')
pins[str(Path(__file__).resolve())]=sha(__file__)
with (H/'protected_prelaunch.json').open('x') as f:json.dump(pins,f,indent=2)
with (H/'PRELAUNCH_REVIEW.txt').open('x') as f:
    f.write('CP-V77 prelaunch reviewed before any capture. Complete raw590 beliefs.\n'
      'v0 diagnostic parser used sender duration column instead of match-decision label;\n'
      'corrected11 index, old files preserved in prelaunch_version0; no solver/guard change.\n'
      'Controller pins protected_prelaunch.json. Original protected_before.json retained.\n'
      'Private CP76 binaries unchanged; bothcontainers stopped after preparation.\n'
      'CP70/CP73 replay gates unset; original native IMU mean alignment true.\n'
      'Only intended mode/compatibility flag differs, no outgoing peer consumer.\n'
      'No accepted-ID equality gate: native admissions and keyframes may differ.\n')
print('SEALED',len(pins),'prelaunch pins; no replay launched.')
