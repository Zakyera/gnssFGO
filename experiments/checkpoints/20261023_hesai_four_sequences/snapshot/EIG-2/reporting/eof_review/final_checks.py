"""Verify saved-data closeout; no container, source or solver mutation."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess

H = Path('/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021')
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eig2_full_cpk24_20261021/prepared')
V = Path('/home/yeranis/results/gnssfgo_eig2_full_cpk24_20261021/eof_review')
D = H / 'eof_review'
REPORT = Path('/home/yeranis/repos/gnssFGO/GNSSFGO_EIG2_FULL_CPK24_REFERENCE_REPORT_20261021.md')

def read(path):
    return json.loads(path.read_text())

def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

assert not (D / 'closeout.json').exists(), 'Do not overwrite closeout'
manifest = read(P / 'protected_manifest.json')
saved = read(D / 'validation_inputs.json')
for group in (manifest, saved['input_sha256'], saved['files']):
    for path, expected in group.items():
        assert sha(path) == expected, path
assert read(H / 'status.json')['state'] == 'failed'
assert read(H / 'status.json')['stage'] == 'validate_on_receiver'
assert (H / 'failed.json').exists()
assert 'unsupported' in (H / 'validate_on_receiver.log').read_text()
results = read(H / 'results_eof_review.json')
assert results['passed'] and not results['original_strict_on_gate_passed']
assert results['validation_contract'] == 'finite_recording_eof_cpk24a'
for mode, omitted in (('off', 0), ('on', 1)):
    gate = read(V / mode / 'validation.json')
    assert gate['passed']
    assert gate['terminal_boundary']['excluded_versions'] == omitted
    assert gate['summary']['pending_at_end'] == 0
    assert (P / f'full_{mode}_control/complete.json').exists()
    assert not (P / f'full_{mode}_control/failed.txt').exists()
assert results['evaluation'] == read(P / 'evaluation_full/summary.json')
assert REPORT.is_file()

names = ['cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia']
containers = json.loads(subprocess.check_output(['docker', 'inspect', *names], text=True))
states = {c['Name'].lstrip('/'): c['State']['Status'] for c in containers}
assert set(states) == set(names)
assert all(state == 'exited' for state in states.values()), states

# Exclude this read-only checker itself; reject exact runnable owned script argv.
owned_names = {'run_pair.py', 'run_case.py', 'run_case_on.py', 'validate_case.py',
               'validate_local.py', 'validate_candidate.py', 'validate_case_eof.py',
               'evaluate_fl4f.py', 'summarize_eof.py'}
owned = []
for row in subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines():
    tokens = row.split()
    if any((str(H) in t or str(P) in t or 'gnssfgo_eig2_full_cpk24_20261021/prepared/' in t)
           and Path(t).name in owned_names for t in tokens):
        owned.append(row)
assert not owned, owned

products = [REPORT, H/'REPORT.txt', H/'results_eof_review.json',
            H/'RESULTS_REPORT_EOF_REVIEW.txt', P/'evaluation_full/summary.json',
            H/'status.json', H/'failed.json', H/'validate_on_receiver.log',
            D/'final_checks.py', D/'validation_inputs.json',
            V/'off/validation.json', V/'on/validation.json']
output = dict(passed=True, at=datetime.datetime.now().astimezone().isoformat(),
              validation_contract='finite_recording_eof_cpk24a',
              original_strict_on_failure_preserved=True,
              terminal_omissions={'off': 0, 'on': 1},
              protected_unchanged=len(manifest),
              saved_inputs_unchanged=len(saved['input_sha256']),
              supplemental_files_unchanged=len(saved['files']),
              containers=states, owned_jobs=[], source_or_solver_rerun=False,
              products_sha256={str(p):sha(p) for p in products})
with (D/'closeout.json').open('x') as f:
    json.dump(output, f, indent=2)
    f.write('\n')
print(json.dumps({k:v for k,v in output.items() if k != 'products_sha256'}, indent=2))
