"""Read-only protected-file check; save this task's bounded completion evidence."""
import hashlib
import json
from pathlib import Path

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_incremental_control_cpk4b_20261019')
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()

assert not (H/'run_integrity.json').exists(), 'Refuse overwrite'
protected = json.loads((R/'protected_before.json').read_text())
assert all(sha(p)==h for p,h in protected.items())
manifest = json.loads((R/'pre_capture_receipt.json').read_text())['manifest']
assert all(sha(p)==h for p,h in manifest.items())
assert json.loads((R/'off_capture/validation.json').read_text())['passed']
assert json.loads((R/'off_capture/cleanup.json').read_text())['kimera_process_gone']
assert (R/'off_capture/evaluation.json').is_file()
assert (R.parent/'kimera_incremental_control_cpk4_20261019/off_capture/failed.txt').is_file()
files = [R/n for n in ('pre_capture_receipt.json','PROTOCOL.txt','run_case.py','validate_score.py')]
files += [R/'off_capture'/n for n in ('started.json','preflight.json','resolved_params.yaml',
    'capture_complete.json','cleanup.json','validation.json','evaluation.json','objective.csv',
    'outputs.bag','kimera.log','native.log','play.log')]
out = dict(passed=True,protected_unchanged=len(protected),prepared_manifest_unchanged=len(manifest),
    one_completed_playback=True,prior_failed_preflight_preserved=True,
    result_manifest={str(p):sha(p) for p in files},
    scope='Bounded ordinary-incremental OFF validation, not full ETH1 or temporary ON certification.')
with (H/'run_integrity.json').open('x') as f: json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='result_manifest'},indent=2))
