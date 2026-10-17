from pathlib import Path
import hashlib,json,subprocess
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_direction_fl4p_20261016')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
frozen=json.loads((R/'pre_capture_receipt.json').read_text())
for p,h in frozen['files'].items():assert sha(Path(p))==h,p
report=dict(protected_paths=len(protected),all_protected_unchanged=True,
    frozen_candidate_paths=len(frozen['files']),all_candidate_unchanged=True,
    validation=json.loads((R/'remove_capture/validation.json').read_text()),
    release=json.loads((R/'test_passed.json').read_text()),
    sanitizer=json.loads((R/'sanitizer_passed.json').read_text()),
    production_promoted=False,estimator_changed=False,run_root=str(R),
    head=subprocess.check_output(['git','-C',str(H.parent.parent),'rev-parse','HEAD'],text=True).strip())
with (R/'final_receipt.json').open('x') as f:json.dump(report,f,indent=2)
with (H/'receipt.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
