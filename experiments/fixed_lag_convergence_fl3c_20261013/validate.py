"""Validate the retained early abort; do not mislabel it a GN200 result."""
import csv,importlib.util,json,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('driver',HERE/'driver.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
root=d.ROOT
saved=json.loads((root/'protected.json').read_text());assert saved==d.protect()
summary=json.loads((root/'fixture_outputs/summary.json').read_text())
with (root/'fixture_outputs/first20_equivalence.csv').open() as f:rows=list(csv.DictReader(f))
assert len(rows)==summary['attempts']==2 and summary['accepted']==2
assert not summary['passed'] and summary['stop']=='exception: first20 GN equivalence failed'
assert summary['equivalent_first_steps']==1 and summary['original_objective_unchanged']
assert summary['initial_cost']==2794137.3456364814 and summary['initial_gradient']==1.5863889609968425
assert float(rows[0]['max_local_coordinate_difference'])<=1e-8
assert float(rows[1]['max_local_coordinate_difference'])>1e-8
assert float(rows[1]['cost_difference'])>1e-7 and float(rows[1]['gradient_difference'])>1e-7
for name in ('test_release','test_sanitize'):
    assert json.loads((root/(name+'.json')).read_text())['returncode']==0
    assert (root/(name+'.log')).read_text()=='PASS 22 checks; nonlinear attempts=4\n'
entry=json.loads((root/'fixture.json').read_text());build=json.loads((root/'build_fixture.json').read_text())
assert entry['returncode']==1 and entry['protected_unchanged']
assert build['binary_sha256']==d.sha(root/'build_fixture')
for name in ('BoundedGN.h','Fixture.cpp'):
    assert entry['code'][name]==build['code'][name]==d.sha(HERE/name)
def values(path):
    result={}
    for line in path.read_text().splitlines():
        a=line.split();result[a[0]+a[1]]=list(map(float,a[2:]))
    return result
a=values(root/'fixture_outputs/final_diagnostic_only.values');b=values(d.SNAPSHOT/'trial_2.values')
assert a.keys()==b.keys()
norm=lambda v:math.sqrt(sum(x*x for x in v))
groups={k:[] for k in ('position_m','orientation_rad','velocity_mps','angular_rate_radps')}
worst_vector=(0,None,None)
for key,x in a.items():
    y=b[key]
    if key[0]=='x':
        groups['position_m'].append(norm([x[j]-y[j] for j in (3,7,11)]))
        R=[[sum(x[4*k+i]*y[4*k+j] for k in range(3)) for j in range(3)] for i in range(3)]
        sinangle=0.5*norm([R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]])
        cosangle=(sum(R[j][j] for j in range(3))-1)*0.5
        groups['orientation_rad'].append(math.atan2(sinangle,cosangle))
    else:
        delta=[x[j]-y[j] for j in range(3)]
        groups['velocity_mps' if key[0]=='v' else 'angular_rate_radps'].append(norm(delta))
        for j,v in enumerate(delta):
            if abs(v)>worst_vector[0]:worst_vector=(abs(v),key,j)
result=dict(early_abort_valid=True,GN200_convergence_result_available=False,protected_files=len(saved),
    release_checks=22,asan_ubsan_checks=22,actual_solver_attempts=2,
    step2_state_difference={key:dict(rms=norm(v)/math.sqrt(len(v)),maximum=max(v)) for key,v in groups.items()},
    step2_largest_vector_coordinate=worst_vector,fixture_sha256=build['binary_sha256'])
with (root/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
