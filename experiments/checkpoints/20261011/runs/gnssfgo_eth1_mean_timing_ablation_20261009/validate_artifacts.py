"""Persist exact build/test provenance and frozen covariance/input checks."""
import ast
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess

from prepare import ROOT, sha, write_json

assert not (ROOT/'validation.json').exists()
for name in ['prepare.py','run_adapter.py','score.py','validate_artifacts.py']:
    ast.parse((ROOT/name).read_text(),filename=name)
environment=dict(os.environ,ASAN_OPTIONS='detect_leaks=0')
with (ROOT/'sanitizer_self_test.log').open('x') as stream:
    check=subprocess.run([str(ROOT/'mean_adapter_asan'),'--self-test'],
                         stdout=stream,stderr=subprocess.STDOUT,env=environment)
assert check.returncode==0
summary=json.loads((ROOT/'summary.json').read_text())
manifest=json.loads((ROOT/'manifest.json').read_text())
assert all(sha(Path(p))==h for p,h in manifest['inputs'].items())
metadata=json.loads((ROOT/'metadata.json').read_text())
payloads=json.loads((ROOT/'aligned_payloads.json').read_text())
assert len(metadata)==len(payloads)==175
original=b''.join(struct.pack('<36d',*m['original']['covariance']) for m in metadata)
aligned=b''.join(struct.pack('<36d',*p['covariance']) for p in payloads)
assert original==aligned
assert (ROOT/'aligned.txt').read_bytes()==(ROOT/'aligned_repeat.txt').read_bytes()
prefix='/home/yeranis/repos/ethzasl_msf/msf_core/include'
eigen='/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/gtsam/gtsam/3rdparty/Eigen'
write_json(ROOT/'validation.json',dict(
    release_build_command=['g++','-std=c++17','-O2','-DNDEBUG','-I'+prefix,'-I'+eigen,
                           str(ROOT/'mean_adapter.cc'),'-o',str(ROOT/'mean_adapter')],
    sanitizer_build_command=['g++','-std=c++17','-O1','-g1','-fsanitize=address,undefined',
                             '-fno-omit-frame-pointer','-I'+prefix,'-I'+eigen,
                             str(ROOT/'mean_adapter.cc'),'-o',str(ROOT/'mean_adapter_asan')],
    compiler=subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
    compiled_component='Standalone numerical diagnostic, no ROS estimator/library build.',
    sanitizer_self_test_returncode=check.returncode,
    leak_sanitizer=False,
    covariance_count=175,covariance_entries=175*36,
    unchanged_covariance_sha256=hashlib.sha256(original).hexdigest(),
    deterministic_byte_identical=True,maximum_causal_prefix_difference=summary['maximum_prefix_difference'],
    protected_input_hashes_unchanged=True,
    files={p.name:sha(p) for p in ROOT.iterdir() if p.is_file()},
    initial_development_failures=[
        'First compilation: int64_t/std::max template argument mismatch in test; corrected locally.',
        'Initial synthetic test exposed Query partition-dependent translation; retained aligned.log and failed_execution.json. Adapter now uses StateAt/Between on one history. No shared helper changed.'],
    no_native_estimator_or_fuser_executed=True))
print('PASS: frozen covariance, deterministic means, causal prefixes, ASan/UBSan, input hashes.')
