"""Exact output/decision validation; only explicit runtime clocks are excluded."""
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
RETAINED = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/receiver')
EXACT = ['trajectory.tum', 'states.csv', 'online_trajectory.tum', 'online_states.csv',
         'support_clock.csv', 'source_model_covariance.csv', 'model_covariance.csv',
         'causal_dispatch.csv', 'measurement_events.csv', 'source_accounting.csv',
         'unsupported_beliefs.csv', 'support_accelerations.csv', 'belief_residuals.csv',
         'publication_steps.csv', 'e2_memo.csv']


def normalized(path, ignore):
    with path.open() as f:
        return [{k:v for k,v in r.items() if k not in ignore} for r in csv.DictReader(f)]


def pair(a,b):
    checks = {}
    for name in EXACT:
        x = (a/name).read_bytes(); y = (b/name).read_bytes()
        checks[name] = dict(equal=x==y, a_sha256=hashlib.sha256(x).hexdigest(), b_sha256=hashlib.sha256(y).hexdigest())
    for name, ignore in [('publication_convergence.csv', {'seconds'}),
                         ('revision_events.csv', {'update_seconds'}),
                         ('stream_progress.csv', {'wall_unix_ns'})]:
        x = normalized(a/name, ignore); y = normalized(b/name, ignore)
        first = next((i for i,(u,v) in enumerate(zip(x,y)) if u!=v), None)
        checks[name] = dict(equal=x==y, rows=[len(x),len(y)], ignored=sorted(ignore), first_difference=first)
        if first is not None: checks[name]['example']=[x[first],y[first]]
    x = json.loads((a/'summary.json').read_text()); y = json.loads((b/'summary.json').read_text())
    for value in [x,y]:
        for key in ['wall_seconds','revision_update_seconds']: value.pop(key)
    checks['summary.json'] = dict(equal=x==y)
    return dict(a=str(a),b=str(b),checks=checks,passed=all(c['equal'] for c in checks.values()))


def main():
    stage = sys.argv[1]
    output = ROOT/(stage+'_validation.json')
    assert not output.exists(), 'Refuse overwrite'
    if stage=='prefix':
        pairs=[pair(ROOT/'prefix_reference2',ROOT/'prefix_off2'), pair(ROOT/'prefix_off2',ROOT/'prefix_on2')]
    elif stage=='full_reference': pairs=[pair(RETAINED,ROOT/'full_off')]
    elif stage=='full': pairs=[pair(ROOT/'full_off',ROOT/'full_on')]
    else: raise ValueError(stage)
    result = dict(passed=all(p['passed'] for p in pairs), comparisons=pairs)
    with output.open('x') as f: json.dump(result,f,indent=2)
    print(stage, 'PASS' if result['passed'] else 'FAIL')
    for p in pairs:
        for name,c in p['checks'].items():
            if not c['equal']: print(name, c)
    if not result['passed']: raise SystemExit(1)


if __name__=='__main__': main()
