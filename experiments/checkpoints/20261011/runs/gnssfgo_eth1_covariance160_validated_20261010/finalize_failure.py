"""Record failure honestly; never convert a partial capture into a pass."""
from pathlib import Path
import hashlib
import json
import subprocess
ROOT=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
protected=json.loads((ROOT/'prepared/protected_manifest.json').read_text())
protected.update(json.loads((ROOT/'additional_protected_evidence.json').read_text()))
for p,h in protected.items():assert sha(p)==h,p
f=json.loads((ROOT/'failure_evidence.json').read_text())
j=json.loads((ROOT/'partial_journal_validation.json').read_text())
assert not f['passed'] and j['captured_payloads_and_order_conserved']
assert not f['source_timing']['native_alignment_failures']
assert f['receiver_failure']['convergence_row']['status']=='budget_exhausted'
assert not f['receiver_failure']['failed_support_present_in_online']
result=dict(passed=False,classification=f['classification'],
    source_diagnostic_fix_observed_effective=True,full160_complete=False,
    protected_files_unchanged=len(protected),captured_revision_versions=j['journal_versions'],
    report_sha256=sha(ROOT/'REPORT.txt'),failure_evidence_sha256=sha(ROOT/'failure_evidence.json'),
    no_new_accuracy_scores=True,no_retries=True,no_covariance_or_solver_tuning=True,
    prior_off_validation_passed=f['prior_off_passed'],
    owned_process_cleanup='Verified both containers show only their PID1 shell/tail plus the diagnostic ps process after failure.',
    heads={str(p):subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
        for p in [ROOT.parents[1]/'src/glim_ros1',Path('/home/yeranis/repos/gnssFGO')]})
with (ROOT/'FINAL_VALIDATION.json').open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result,indent=2))
