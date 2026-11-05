"""Read-only original/candidate integrity checks after the bounded capture."""
import hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
groups={'protected':json.loads((R/'protected_before.json').read_text()),'candidate':json.loads((R/'pre_capture_receipt.json').read_text())['diagnostic_manifest']}
out={}
for name,manifest in groups.items():
    changed=[p for p,h in manifest.items() if sha(p)!=h]
    out[name]={'files':len(manifest),'changed':changed};assert not changed,(name,changed)
with (H/'preservation.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out))
