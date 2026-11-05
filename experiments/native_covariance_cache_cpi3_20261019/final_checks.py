from pathlib import Path
import ast
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

for p in HERE.glob('*.py'):ast.parse(p.read_text())
for name in ['test_cache.cpp','benchmark.cpp','build.py']:
    assert (HERE/name).read_bytes()==(ROOT/name).read_bytes()
for mode in ['release_v2','san']:
    def numeric(name):
        return [l for l in (ROOT/mode/name).read_text().splitlines()
                if l.startswith(('NUMERIC ','CONDITION ','LIFECYCLE ','BOUND '))]
    assert numeric('tests.log')==numeric('repeat.log')
load=json.loads((ROOT/'abi_load.json').read_text());assert load['returncode']==0
verification=json.loads((HERE/'verification.json').read_text())
assert sha(ROOT/'release_v2/libgtsam_points.so.1.2.1')==verification['private_provider_sha256']
for p,h in verification['protected'].items():assert sha(Path(p))==h,p
source=HERE/'evidence/source';source.mkdir()
for name in ['NativeGaussianHistory.cpp','NativeGaussianHistory.h']:
    shutil.copy2(ROOT/'source'/name,source/name)
shutil.copy2(ROOT/'abi_load.json',HERE/'evidence/abi_load.json')
receipt=HERE/'final_checks.json';assert not receipt.exists()
receipt.write_text(json.dumps(dict(passed=True,numeric_repeats_identical=True,
    abi_load_passed=True,private_sources={p.name:sha(p) for p in source.iterdir()},
    scripts={p.name:sha(p) for p in HERE.glob('*.py')}),indent=2)+'\n')
print('syntax/source identity, numeric repeats, ABI and protected hashes pass')
