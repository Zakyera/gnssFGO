"""Pre-capture receipt. Does not alter a source profile or production binary."""
from pathlib import Path
import hashlib,json,shutil
E=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_step_acceptance_fl4o_20261016')
assert not (R/'remove_capture').exists()
assert json.loads((R/'test_passed.json').read_text())['passed']
assert json.loads((R/'sanitizer_passed.json').read_text())['passed']
def digest(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
assert all(digest(Path(p))==h for p,h in protected.items())
for n in ['run_case.py','validate.py','build.py']:
    shutil.copy2(R/n,E/n)
paths=[R/n for n in ['test.cpp','StepAcceptance.h','test','test_san','PROTOCOL.txt','run_case.py','validate.py','build.py','sanitize.py','aligned_inputs.bag','frozen_inputs.json','private_build/libkimera_vio.so','private_build/libcbs.so']]
paths+=list((R/'include').rglob('*.h'))+list((R/'kimera_profile').rglob('*'))
receipt=dict(protected_paths=len(protected),protected_unchanged=True,container_initial_status='exited',candidate={str(p):digest(p) for p in paths if p.is_file()})
with (R/'pre_capture_receipt.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k!='candidate'}))
