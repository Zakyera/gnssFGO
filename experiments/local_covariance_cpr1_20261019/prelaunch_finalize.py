"""Pin reviewed validation script before any capture, with explicit audit receipt."""
from pathlib import Path
import ast,hashlib,json
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared')
assert not list(R.glob('live60_*_control'))
assert not (R/'preflight/finalize.json').exists()
manifest=json.loads((R/'protected_manifest.json').read_text());changes={}
assert (H/'validate_local.py').read_bytes()==(R/'validate_local.py').read_bytes()
for p in (H/'validate_local.py',R/'validate_local.py'):
 ast.parse(p.read_text());new=hashlib.sha256(p.read_bytes()).hexdigest()
 changes[str(p)]=dict(old=manifest[str(p)],new=new);manifest[str(p)]=new
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with (R/'preflight/finalize.json').open('x') as f:
 json.dump(dict(changes=changes,reason='Review: provider matrix dump exists only for first publications; revision route has explicit per-version provenance, synthetic exact-equality tests and pinned bridge. No capture or estimator changed.'),f,indent=2)
print('Reviewed validator pinned before capture')
