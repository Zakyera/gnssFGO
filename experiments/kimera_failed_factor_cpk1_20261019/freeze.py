"""Freeze only this diagnostic after all tests; never replace a receipt."""
import hashlib,json
from pathlib import Path
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not (R/'off_capture').exists()
for n in ('test_passed.json','sanitizer_passed.json','localization_tests_passed.json'):
    assert json.loads((R/n).read_text())['passed'],n
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
files=[p for d in ('include','private_build','kimera_profile') for p in (R/d).rglob('*') if p.is_file()]
files += [p for p in R.iterdir() if p.suffix in ('.cpp','.h','.py','.launch') or p.name in ('PROTOCOL.txt','build_commands.json','actual_test_commands.json')]
manifest={str(p):sha(p) for p in files}
with (R/'pre_capture_receipt.json').open('x') as f:json.dump({'diagnostic_manifest':manifest,'protected_count':len(protected),'protected_unchanged':True,'one_capture_only':True,'solver_unchanged':True},f,indent=2)
print(json.dumps({'frozen_files':len(manifest),'protected_files':len(protected)}))
