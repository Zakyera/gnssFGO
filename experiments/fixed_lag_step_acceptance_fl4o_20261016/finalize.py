"""Persist failure/protection receipt without changing any scientific input."""
from pathlib import Path
import hashlib,json,subprocess
E=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_step_acceptance_fl4o_20261016')
def digest(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pre=json.loads((R/'pre_capture_receipt.json').read_text())
protected=json.loads((R/'protected_before.json').read_text())
assert all(digest(p)==h for p,h in protected.items())
assert all(digest(p)==h for p,h in pre['candidate'].items())
result=json.loads((R/'failure_analysis.json').read_text())
assert not result['completed'] and not result['candidate_validated']
report=dict(status='BOUNDED_REPLAY_FAILED_REPAIR_INCOMPLETE',result=result,
    protected_paths_unchanged=len(protected),frozen_candidate_paths_unchanged=len(pre['candidate']),
    release_tests_passed=True,sanitizer=json.loads((R/'sanitizer_passed.json').read_text()),
    container_restored='exited',gnssfgo_head=subprocess.check_output(['git','-C',str(E.parents[1]),'rev-parse','HEAD'],text=True).strip(),
    no_production_promotion=True,no_followup_run=True,
    evidence_sha256={str(p):digest(p) for p in [R/'failure_analysis.json',R/'pre_capture_receipt.json',R/'compile5_test_run.log',R/'abi_matched_san_run.log',R/'remove_capture/kimera.log',R/'remove_capture/outputs.bag',E/'REPORT.txt']})
for p in [R/'final_receipt.json',E/'receipt.json']:
    with p.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k not in ['result','evidence_sha256']}))
