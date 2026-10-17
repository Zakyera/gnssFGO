"""Record actual run status and preservation; never turn a failure into a pass."""
from pathlib import Path
import hashlib,json,subprocess,shutil
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((R/'protected_before.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
receipt=json.loads((R/'pre_capture_receipt.json').read_text())
for p,h in receipt['files'].items():assert sha(Path(p))==h,p
D=R/'remove_capture'
report=dict(protected_files_unchanged=len(protected),candidate_files_unchanged=len(receipt['files']),
    private_library_sha=receipt['library_sha'],head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=H.parent.parent,text=True).strip(),
    release_passed=json.loads((R/'test_passed.json').read_text()),
    sanitizer=json.loads((R/'sanitizer_passed.json').read_text()),
    candidate_promoted=False,production_modified=False)
for n in ['validation.json','complete.json','failed.txt']:
    p=D/n
    if p.exists():report[n]=json.loads(p.read_text()) if n.endswith('.json') else p.read_text()
with (R/'final_receipt.json').open('x') as f:json.dump(report,f,indent=2)
for n in ['run_case.py','build.py','sanitize.py','wrapper.diff','OriginalChartPrior.h','build_commands.json','actual_test_commands.json']:
    dest=H/('archived_'+n if n.endswith('.py') else n)
    assert not dest.exists();shutil.copy2(R/n,dest)
(H/'receipt.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
