"""Validate observing-only replay and report exact reuse opportunity."""
import csv,hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
BASE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011/prepared/live60_on_receiver/receiver')
OUT=ROOT/'probe_live60'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
exact={}
for name in ['online_states.csv','states.csv','online_trajectory.tum','trajectory.tum',
             'support_clock.csv','source_model_covariance.csv','model_covariance.csv',
             'causal_dispatch.csv','measurement_events.csv','source_accounting.csv','unsupported_beliefs.csv']:
    exact[name]=sha(BASE/name)==sha(OUT/name)
    assert exact[name],name
for name,skip in [('publication_convergence.csv',{'seconds'}),('publication_steps.csv',set()),
                  ('revision_events.csv',{'update_seconds'})]:
    a=[{k:v for k,v in r.items() if k not in skip} for r in rows(BASE/name)]
    b=[{k:v for k,v in r.items() if k not in skip} for r in rows(OUT/name)]
    assert a==b,name
    exact[name+' non-timing']=True
probe=rows(OUT/'e2_probe.csv');start=int(probe[0]['physical_ns'])
stats={}
for window,lo,hi in [('all',-1,1000),('early',-1,20),('middle',20,40),('late',40,1000)]:
    stats[window]={}
    for kind,label in [('0','wnoa'),('1','relative')]:
        selected=[r for r in probe if r['kind']==kind and lo <= (int(r['physical_ns'])-start)/1e9 < hi]
        sums={k:sum(int(r[k]) for r in selected) for k in ['calls','repeats','compute_ns','repeat_compute_ns','signature_ns']}
        sums['repeat_call_fraction']=sums['repeats']/max(1,sums['calls'])
        sums['repeat_compute_fraction']=sums['repeat_compute_ns']/max(1,sums['compute_ns'])
        stats[window][label]=sums
total=sum(s['compute_ns'] for s in stats['all'].values())
repeat=sum(s['repeat_compute_ns'] for s in stats['all'].values())
result=dict(valid=True,exact=exact,stats=stats,potential_repeated_linearization_fraction=repeat/total,
            worthwhile_gate_passed=repeat/total>=.05,
            limitation='Observing-only timings; every computation still executed. Not an implemented speedup.')
with (ROOT/'probe_analysis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
