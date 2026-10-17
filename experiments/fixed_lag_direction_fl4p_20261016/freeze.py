from pathlib import Path
import hashlib,json,shutil
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_direction_fl4p_20261016')
assert not (R/'remove_capture').exists()
assert (R/'test_passed.json').exists() and (R/'sanitizer_passed.json').exists()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
base=R.parent/'fixed_lag_linear_parity_fl4n_20261015'
for p in (base/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(R/'kimera_profile'/p.relative_to(base/'kimera_profile'))
assert sha(base/'VioBackend.cpp')==sha(R/'VioBackend.cpp')
assert sha(base/'aligned_inputs.bag')==sha(R/'aligned_inputs.bag')
shutil.copy2(H/'validate.py',R/'validate.py')
assert '--duration=23' in (R/'run_case.py').read_text()
files=[p for p in R.rglob('*') if p.is_file() and (p.suffix in ['.h','.cpp','.py','.launch','.so'] or p.parent.name=='kimera_profile' or p.name in ['aligned_inputs.bag','frozen_inputs.json'])]
receipt=dict(protected_count=len(protected),all_protected_unchanged=True,profile_exact=True,
    aligned_inputs_exact=True,backend_tu_exact=True,library_sha=sha(R/'private_build/libkimera_vio.so'),
    files={str(p):sha(p) for p in files})
with (R/'pre_capture_receipt.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k!='files'},indent=2))
