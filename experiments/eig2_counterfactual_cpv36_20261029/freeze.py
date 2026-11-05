"""Finalize diagnostics and private runtime identities before the one replay."""
import ast,hashlib,json,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for gate in ('release_native2','san_native2','backend','offline_4','offline_4san'):assert (P/('build_'+gate)/'passed.json').exists()
assert not (P/'capture').exists()
for p,s in json.loads((H/'protected_before.json').read_text()).items():assert sha(p)==s,p
for name in ('run_inside.py','PROTOCOL.txt'):shutil.copy2(H/name,P/name)
for p in P.glob('*.py'):ast.parse(p.read_text(),filename=str(p))
files=[p for p in P.rglob('*') if p.is_file() and not any(x.startswith('build_') for x in p.relative_to(P).parts) and p.suffix!='.o']
manifest={str(p.relative_to(P)):sha(p) for p in files}
with (P/'prepared_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
with (H/'ready.json').open('x') as f:json.dump(dict(passed=True,private_files=len(manifest),libkimera_sha256=sha(P/'private_build/libkimera_vio.so'),libcbs_sha256=sha(P/'private_build/libcbs.so'),source_settings_changed=False,observer_only=True),f,indent=2)
print('CPV36 ready; private hashes frozen; no replay yet')
