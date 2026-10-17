"""Final hash checks; does not run or modify an estimator."""
from pathlib import Path
import hashlib,json,subprocess,time
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_stopping_fl4u_20261017')
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
frozen=json.loads((R/'pre_capture_receipt.json').read_text())['diagnostic_manifest']
changed_protected=[p for p,h in protected.items() if sha(Path(p))!=h]
changed_diagnostic=[p for p,h in frozen.items() if sha(Path(p))!=h]
result=dict(time=time.time(),protected_count=len(protected),diagnostic_count=len(frozen),
    changed_protected=changed_protected,changed_diagnostic=changed_diagnostic,
    head=subprocess.check_output(['git','-C',str(H.parents[1]),'rev-parse','HEAD'],text=True).strip(),
    valid=not changed_protected and not changed_diagnostic)
with (H/'preservation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
assert result['valid']
