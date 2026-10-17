from pathlib import Path
import json,hashlib,shutil
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016')
assert not (R/'remove_capture').exists()
assert json.loads((R/'test_passed.json').read_text())['passed']
assert json.loads((R/'sanitizer_passed.json').read_text())['passed']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
N=R.parent/'fixed_lag_linear_parity_fl4n_20261015'
for p in (N/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(R/'kimera_profile'/p.relative_to(N/'kimera_profile'))
for n in ['VioBackend.cpp','aligned_inputs.bag']:assert sha(N/n)==sha(R/n)
shutil.copy2(H/'validate.py',R/'validate.py')
assert '--duration=30' in (R/'run_case.py').read_text()
files=[p for p in R.rglob('*') if p.is_file() and (p.suffix in ['.h','.cpp','.py','.launch','.so'] or 'kimera_profile' in p.parts or p.name in ['aligned_inputs.bag','frozen_inputs.json'])]
receipt=dict(protected_count=len(protected),protected_unchanged=True,profile_exact=True,
    inputs_exact=True,backend_translation_unit_exact=True,library_sha=sha(R/'private_build/libkimera_vio.so'),
    files={str(p):sha(p) for p in files})
with (R/'pre_capture_receipt.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k!='files'},indent=2))
