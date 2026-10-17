"""Read-only checks plus immutable diagnostic manifest; no estimator launch."""
import ast,difflib,hashlib,json,time
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/fixed_lag_stopping_fl4u_20261017'
Q=W/'runs/fixed_lag_current_chart_fl4q_20261016'
T=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
assert not (R/'off_capture').exists()
assert json.loads((R/'test_passed.json').read_text())['passed']
assert json.loads((R/'sanitizer_passed.json').read_text())['passed']
profile={str(p.relative_to(R/'kimera_profile')):sha(p) for p in (R/'kimera_profile').rglob('*') if p.is_file()}
original={str(p.relative_to(T/'live120_off_source/kimera_profile')):sha(p) for p in (T/'live120_off_source/kimera_profile').rglob('*') if p.is_file()}
assert profile==original
for p in R.glob('*.py'):ast.parse(p.read_text())
for n in ['VioBackend.cpp','OriginalChartPrior.h','RetirementAudit.h','LinearParityAudit.h','test.cpp']:
    assert sha(R/n)==sha(Q/n),n
for p in (R/'private_build').rglob('*'):
    if p.is_file() and p.name!='libkimera_vio.so':assert sha(p)==sha(Q/'private_build'/p.relative_to(R/'private_build')),p
delta=''.join(difflib.unified_diff((Q/'CurrentChartFallback.h').read_text().splitlines(True),(R/'CurrentChartFallback.h').read_text().splitlines(True),fromfile='FL4q original',tofile='FL4u logging only final'))
with (H/'instrumentation_final.diff').open('x') as f:f.write(delta)
selected=[p for p in R.rglob('*') if p.is_file() and (p.suffix in ['.h','.cpp','.py','.launch','.json','.so'] or 'include' in p.parts or 'kimera_profile' in p.parts)]
with (R/'pre_capture_receipt.json').open('x') as f:
    json.dump(dict(time=time.time(),protected_count=len(protected),protected_unchanged=True,profile=profile,
        diagnostic_manifest={str(p):sha(p) for p in selected},release_passed=True,focused_sanitizer_passed=True,
        diagnostic_only=True,initial_container_state={'cbsms_ws_gpu':'stopped','ubuntu-jammy-humble-nvidia':'stopped'}),f,indent=2)
print('PRE-CAPTURE PASS',len(protected),'protected files;',len(profile),'identical profile files')
