"""Mechanical preparation only; reuse the exact tested candidate, no build."""
from pathlib import Path
import difflib, hashlib, json, shutil

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
Q = W / 'runs/fixed_lag_current_chart_fl4q_20261016'
R = W / 'runs/fixed_lag_current_chart_fl4r_20261016'
def sha(p):
    return hashlib.file_digest(p.open('rb'), 'sha256').hexdigest()
protected = json.loads((Q / 'protected_before.json').read_text())
old = json.loads((Q / 'pre_capture_receipt.json').read_text())
for p, digest in {**protected, **old['files']}.items():
    assert sha(Path(p)) == digest, p
assert old['library_sha'] == '6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
for n in ['test_passed.json', 'sanitizer_passed.json']:
    assert json.loads((Q / n).read_text())['passed']
assert len(json.loads((Q / 'frozen_inputs.json').read_text())['rows']) == 450
R.mkdir()
for n in ['private_build', 'kimera_profile']:
    shutil.copytree(Q / n, R / n)
for n in ['aligned_inputs.bag', 'frozen_inputs.json', 'test_passed.json', 'sanitizer_passed.json']:
    shutil.copy2(Q / n, R / n)
for p in Q.glob('*.launch'):
    # These launch files use $(dirname), not hard-coded candidate directories.
    assert 'fixed_lag_current_chart_fl4q' not in p.read_text()
    shutil.copy2(p, R / p.name)
original = (Q / 'run_case.py').read_text()
s = original.replace('11439', '11440')
for a, b in [('--duration=30', '--duration=94'), ('duration=30', 'duration=94'), ('+460', '+1150')]:
    assert s.count(a) == 1, a
    s = s.replace(a, b)
(R / 'run_case.py').write_text(s)
(H / 'runner.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True), s.splitlines(True))))
for n in ['PROTOCOL.txt', 'validate.py']:
    shutil.copy2(H / n, R / n)
files = {str(p): sha(p) for p in R.rglob('*') if p.is_file()}
receipt = dict(protected_count=len(protected), fl4q_candidate_count=len(old['files']),
    protected_unchanged=True, fl4q_unchanged=True, library_sha=old['library_sha'],
    release_sanitizer_reused_not_rerun=True, files=files)
with (R / 'pre_capture_receipt.json').open('x') as f:
    json.dump(receipt, f, indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k != 'files'}, indent=2))
