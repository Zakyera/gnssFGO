"""Read-only acceptance checks; writes only one new FL3b audit product."""
import csv,importlib.util,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('driver',HERE/'driver.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
root=d.ROOT
saved=json.loads((root/'protected.json').read_text())
assert saved==d.protect()
summary=json.loads((root/'fixture_outputs/summary.json').read_text())
with (root/'fixture_outputs/trials.csv').open() as f:rows=list(csv.DictReader(f))
assert len(rows)==summary['attempts']==20
assert summary['accepted']==20 and summary['rejected']==0 and not summary['passed']
assert summary['stop']=='budget_exhausted' and summary['original_objective_unchanged']
assert summary['initial_cost']==2794137.3456364814
assert summary['initial_gradient']==1.5863889609968425
last=summary['initial_cost']
for i,row in enumerate(rows,1):
    assert int(row['attempt'])==int(row['accepted_iterations'])==int(row['library_inner_iterations'])==i
    assert int(row['accepted'])==1
    assert float(row['cost_before'])==last
    assert float(row['cost_after'])<last
    # Match the installed policy's multiplication, not a differently rounded division.
    assert float(row['lambda_after'])==float(row['lambda_before'])*(1.0/3.0)
    last=float(row['cost_after'])
assert last==summary['final_cost'] and float(rows[-1]['gradient'])==summary['final_gradient']
for name in ('test_release2','test_sanitize'):
    assert json.loads((root/(name+'.json')).read_text())['returncode']==0
    assert (root/(name+'.log')).read_text()=='PASS 42 checks; nonlinear attempts=11 accepted=6 rejected=5\n'
for name in ('test_release2','test_sanitize','fixture'):
    entry=json.loads((root/(name+'.json')).read_text())
    assert entry['protected_unchanged']
    assert entry['code']['BoundedAdaptive.h']==d.sha(HERE/'BoundedAdaptive.h')
entry=json.loads((root/'fixture.json').read_text())
build=json.loads((root/'build_fixture.json').read_text())
assert entry['binary_sha256']==build['binary_sha256']==d.sha(root/'build_fixture')
for name in ('BoundedAdaptive.h','Fixture.cpp'):
    assert entry['code'][name]==build['code'][name]==d.sha(HERE/name)
result=dict(diagnostic_valid=True,candidate_passed=False,protected_files=len(saved),
    normalized_gradient_limit=1e-5,gradient_times_limit=summary['final_gradient']/1e-5,
    gradient_ratio_vs_GN=summary['final_gradient']/0.00054760810443800164,
    cost_reduction=summary['initial_cost']-summary['final_cost'],
    all_trials_accepted=True,all_costs_decreased=True,production_modified=False,
    release_checks=42,asan_ubsan_checks=42,fixture_sha256=entry['binary_sha256'])
with (root/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
