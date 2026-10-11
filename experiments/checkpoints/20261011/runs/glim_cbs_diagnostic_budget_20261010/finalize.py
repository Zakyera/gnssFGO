"""Summarize completed validation without erasing the initial checker failure."""
from pathlib import Path
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
PREP=ROOT/'prepared'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

protected=json.loads((PREP/'protected_manifest.json').read_text())
for path,value in protected.items():assert sha(path)==value,path
v=json.loads((PREP/'perf100_on_validation.json').read_text())
assert v['passed'] and v['native_alignment']['passed'] and v['diagnostics']['passed'] and v['lifecycle']['passed']
assert all(r['byte_identical'] for r in v['repeat_files'].values())
assert len(v['unsupported'])==25
assert {r['reason'] for r in v['unsupported']}=={'before_initialization'}
assert len({(r['source'],r['ordinal']) for r in v['unsupported']})==1
tests={}
for name,n in [('budget_test.xml',6),('budget_test_sanitized.xml',6),
               ('dcreg_regression.xml',31),('native_bridge_regression.xml',5)]:
    r=ET.parse(ROOT/name).getroot()
    assert int(r.attrib['tests'])==n and int(r.attrib['failures'])==0
    tests[name]=n
scope=subprocess.run(['python3',str(ROOT/'test_change_scope.py'),'-v'],capture_output=True,text=True)
assert scope.returncode==0
timing=json.loads((ROOT/'timing_comparison_verified.json').read_text())
assert timing['cases']['new_on']['active_diagnostic_rows']==0
assert not timing['cases']['new_on']['native_alignment_failures']
repo=ROOT.parents[1]/'src/glim_ros1'
result=dict(passed=True,scope='100 s engineering retest, not full160 or accuracy certification',
    protected_entries_checked=len(protected),tests=tests,scope_tests=scope.stderr,
    numerical_validation=str(PREP/'perf100_on_validation.json'),
    initial_checker_failure_preserved=str(ROOT/'validation_logs/failed.txt'),
    resolution='Supplemental lifecycle checker follows logged BPSAM snapshot/rollback/retry; no estimator change or source retry',
    existing_kimera_guard_rollbacks=len(v['lifecycle']['kimera']['rollbacks']),
    revision_versions_conserved=v['versions'],byte_identical_repeat_files=len(v['repeat_files']),
    report_sha256=sha(ROOT/'REPORT.txt'),
    head=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
    worktree_status=subprocess.check_output(['git','-C',str(repo),'status','--short'],text=True))
with (ROOT/'FINAL_VALIDATION.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:result[k] for k in ['passed','protected_entries_checked','existing_kimera_guard_rollbacks',
                                    'revision_versions_conserved','byte_identical_repeat_files']},indent=2))
