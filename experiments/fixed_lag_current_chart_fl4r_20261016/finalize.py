"""Preservation and outcome receipt, including failures; never relaunch."""
from pathlib import Path
import hashlib, json, shutil, subprocess

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
Q = W / 'runs/fixed_lag_current_chart_fl4q_20261016'
R = W / 'runs/fixed_lag_current_chart_fl4r_20261016'
D = R / 'remove_capture'
def sha(p):
    with p.open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
baseline = json.loads((Q/'protected_before.json').read_text())
old = json.loads((Q/'pre_capture_receipt.json').read_text())
new = json.loads((R/'pre_capture_receipt.json').read_text())
for p, h in {**baseline, **old['files'], **new['files']}.items():
    assert sha(Path(p)) == h, p
assert (D/'complete.json').exists() or (D/'failed.txt').exists(), 'capture unfinished'
receipt = dict(protected_files_unchanged=len(baseline), fl4q_files_unchanged=len(old['files']),
    fl4r_files_unchanged=len(new['files']), library_sha=new['library_sha'],
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=H.parent.parent,text=True).strip(),
    release_sanitizer_reused_not_rerun=True, production_modified=False, candidate_promoted=False)
for n in ['validation.json', 'complete.json', 'failed.txt']:
    if (D/n).exists():
        receipt[n] = json.loads((D/n).read_text()) if n.endswith('.json') else (D/n).read_text()
with (R/'final_receipt.json').open('x') as f: json.dump(receipt,f,indent=2)
with (H/'receipt.json').open('x') as f: json.dump(receipt,f,indent=2)
assert not (H/'archived_run_case.py').exists()
shutil.copy2(R/'run_case.py',H/'archived_run_case.py')
print(json.dumps({k:v for k,v in receipt.items() if k != 'validation.json'},indent=2))
