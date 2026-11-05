"""Read-only baseline verification and compact evidence collection; no runs."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import subprocess
HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/local_revision_covariance_cpi1_20261019'
BASE=WS/'runs/shared_linearization_s4_provider_20261012/release_2'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

protected=json.loads((ROOT/'protected.json').read_text())
protected[str(BASE/'libglim_ros.so')]='0114b435785ae85f58a69f1156abd1a2a320dc7138ca0b2dd4bfd56715d4ebea'
protected[str(BASE/'libgtsam_points.so.1.2.1')]='42fb1e8a931ebc86a06ae41a2eaf6ff39b9efbe5a1a4fd9737ee06edb2274228'
protected[str(WS/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/private_build/libkimera_vio.so')]='6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
protected['/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live']='53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
for p,h in protected.items():assert sha(p)==h,p
release=(ROOT/'release_v2_revision_test.log').read_text()
repeat=(ROOT/'release_v2_revision_repeat.log').read_text()
san=(ROOT/'san_revision_test.log').read_text()
for text in [release,repeat,san]:assert '[  PASSED  ] 50 tests.' in text
numeric=lambda s:[line for line in s.splitlines() if line.startswith(('NUMERIC ','CONDITION ','BOUND '))]
assert numeric(release)==numeric(repeat),'numeric repeat differs'
assert json.loads((ROOT/'san_completion_passed.json').read_text())['passed']
for name in ['release_v2_kimera_test.log','san_kimera_completion_test.log']:
    assert 'pairs=24 max_covariance_difference=0' in (ROOT/name).read_text()
for p in HERE.glob('*.py'):ast.parse(p.read_text())
for name in ['RevisionCovariance.h','test_revision.cpp','test_kimera.cpp','build.py']:
    assert sha(HERE/name)==sha(ROOT/name),name
evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
for p in ROOT.glob('*.log'):shutil.copy2(p,evidence/p.name)
for p in ROOT.glob('*.json'):shutil.copy2(p,evidence/p.name)
diff=subprocess.run(['diff','-u',str(BASE/'overlay/cbs_bridge.cpp'),str(ROOT/'cbs_bridge.cpp')],text=True,capture_output=True)
assert diff.returncode==1
(evidence/'cbs_bridge.patch').write_text(diff.stdout)
artifact_hashes={str(p):sha(p) for p in ROOT.iterdir() if p.is_file() and p.suffix in ['.h','.cpp','.py','.so']}
result=dict(protected_hashes=protected,protected_unchanged=True,
    candidate_hashes=artifact_hashes,release_tests=50,sanitizer_tests=50,
    kimera_fixture_pairs=24,kimera_covariance_difference=0,
    exact_numeric_repeat=True,dataset_runs=0,defaults_promoted=False,
    strict_all_local_certified=False,
    remaining='Kimera has filtered MAIN marginal history, not a separate native-history provider; live correctness and query throughput untested')
with (HERE/'verification.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(dict(protected=len(protected),tests=50,kimera_pairs=24,
    candidate_sha256=sha(ROOT/'libglim_ros.so'),status='isolated GLIM implementation passed; no all-source promotion')))
