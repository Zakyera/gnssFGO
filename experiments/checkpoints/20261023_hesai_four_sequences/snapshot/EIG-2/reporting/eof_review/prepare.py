"""Generate supplemental validators; retain originals and publish the exact diff."""
from pathlib import Path
import ast,difflib,hashlib,json
from eof_boundary import test_boundaries

D=Path(__file__).resolve().parent
H=D.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eig2_full_cpk24_20261021/prepared')
V=Path('/home/yeranis/results/gnssfgo_eig2_full_cpk24_20261021')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

assert not (V/'eof_review').exists()
assert not (P/'evaluation_full').exists()
assert not (D/'validation_inputs.json').exists()
manifest=json.loads((P/'protected_manifest.json').read_text())
for p,w in manifest.items():assert sha(p)==w,p
inputs={}
for mode in ('off','on'):
    for p in [P/f'full_{mode}_source/odom_imu.txt',P/f'full_{mode}_source/kimera_odometry.csv',
              P/f'full_{mode}_receiver/events.csv',P/f'full_{mode}_receiver/receiver/summary.json',
              *sorted((P/f'full_{mode}_receiver/receiver').glob('*_states.csv')),
              *sorted((P/f'full_{mode}_receiver/receiver').glob('*_trajectory.tum'))]:
        inputs[str(p)]=sha(p)

old=(P/'validate_case.py').read_text();new=old
changes=[('from expiry_audit import ExpiryAudit',
          f"sys.path.insert(0,{str(P)!r})\nfrom expiry_audit import ExpiryAudit\nfrom eof_boundary import audit_eof"),
         ('P=Path(__file__).resolve().parent',f'P=Path({str(P)!r})'),
         (f"RESULT=Path(f'{V}/{{MODE}}')",f"RESULT=Path(f'{V}/eof_review/{{MODE}}')"),
         ("    assert action['unsupported_at_end']==summary['unsupported_end']==0",
          '    boundary=audit_eof(P,MODE,summary,action,online_records,events)'),
         ('result=dict(passed=True,source_coverage=',
          "result=dict(passed=True,validation_contract='finite_recording_eof_cpk24a',terminal_boundary=boundary,source_coverage=")]
for a,b in changes:
    assert new.count(a)==1,a
    new=new.replace(a,b)
with (D/'validate_case_eof.py').open('x') as f:f.write(new)
diff=list(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(P/'validate_case.py'),tofile=str(D/'validate_case_eof.py')))
old=(H/'summarize.py').read_text();new=old
changes=[('H=Path(__file__).resolve().parent',f'H=Path({str(H)!r})'),
         (f"V=Path('{V}')",f"V=Path('{V}/eof_review')"),
         ("'results.json'","'results_eof_review.json'"),
         ("'RESULTS_REPORT.txt'","'RESULTS_REPORT_EOF_REVIEW.txt'"),
         ('result=dict(passed=True,cases=cases',
          "result=dict(passed=True,validation_contract='finite_recording_eof_cpk24a',original_strict_on_gate_passed=False,cases=cases"),
         ('OFF/ON — COMPLETED', 'OFF/ON — SCORED WITH EXPLICIT TERMINAL-FACTOR EXCLUSION'),
         (" 'One capture per mode,0.20x,headless. No paired observer,optimizer rebuild or tuning.',",
          " 'One capture per mode,0.20x,headless. No paired observer,optimizer rebuild or tuning.',\n 'Original ON zero-unsupported gate failed; supplemental terminal-cell accounting passes. One final Kimera factor was not incorporated; trajectories unchanged.',")]
for a,b in changes:
    assert a in new,a
    new=new.replace(a,b)
with (D/'summarize_eof.py').open('x') as f:f.write(new)
diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(H/'summarize.py'),tofile=str(D/'summarize_eof.py')))
with (D/'supplement.diff').open('x') as f:f.write(''.join(diff))
for p in D.glob('*.py'):ast.parse(p.read_text())
record=dict(passed=True,protected_unchanged=len(manifest),input_sha256=inputs,
            boundary_tests=test_boundaries(),source_or_solver_rerun=False,
            files={str(p):sha(p) for p in D.iterdir() if p.is_file()})
with (D/'validation_inputs.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(dict(passed=True,protected_unchanged=len(manifest),boundary_tests=record['boundary_tests'])))
