"""Read-only numerical comparison after the exact profiling gate failed."""
import csv
import json
import math
from pathlib import Path
from collections import Counter

ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
REF = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/receiver')
NEW = ROOT/'full_off'


def rows(directory, name):
    with (directory/name).open() as f:
        return list(csv.DictReader(f))


def stats(values):
    return dict(max=max(values), rms=math.sqrt(math.fsum(x*x for x in values)/len(values)))


def orientation(a, b):
    qa = [float(a[k]) for k in ['qw','qx','qy','qz']]
    qb = [float(b[k]) for k in ['qw','qx','qy','qz']]
    qa = [x/math.hypot(*qa) for x in qa]
    qb = [x/math.hypot(*qb) for x in qb]
    # Shortest quaternion chord -> rotation angle; stable near zero.
    chord = min(math.dist(qa, qb), math.dist(qa, [-x for x in qb]))
    return 4*math.asin(min(1, chord/2))


def state_comparison(name):
    a, b = rows(REF, name), rows(NEW, name)
    assert len(a)==len(b)
    for x,y in zip(a,b):
        assert (x['state_index'],x['timestamp_ns'])==(y['state_index'],y['timestamp_ns'])
    result = dict(rows=len(a))
    for label, keys in [('position_m',['px','py','pz']), ('velocity_m_s',['vx','vy','vz']), ('omega_rad_s',['wx','wy','wz'])]:
        result[label]=stats([math.dist([float(x[k]) for k in keys], [float(y[k]) for k in keys]) for x,y in zip(a,b)])
    result['orientation_rad']=stats([orientation(x,y) for x,y in zip(a,b)])
    return result


def decision_comparison():
    a,b=rows(REF,'publication_convergence.csv'),rows(NEW,'publication_convergence.csv')
    assert len(a)==len(b)
    keys=['phase','index','physical_ns','attempted','accepted','status']
    different=[dict(row=i,reference={k:x[k] for k in keys},private={k:y[k] for k in keys}) for i,(x,y) in enumerate(zip(a,b)) if any(x[k]!=y[k] for k in keys)]
    result=dict(rows=len(a), differing_decision_rows=len(different), first_difference=different[0] if different else None)
    for label, data in [('reference',a),('private',b)]:
        steps=rows(REF if label=='reference' else NEW,'publication_steps.csv')
        result[label]=dict(status_counts=dict(Counter(r['status'] for r in data)),
            maximum_final_gradient=max(float(r['gradient_after']) for r in data),
            total_attempted=sum(int(r['attempted']) for r in data),
            total_accepted=sum(int(r['accepted']) for r in data),
            step_rows=len(steps), recovery_rows=sum(int(r['recovery'])!=0 for r in steps))
    return result


def revisions():
    a,b=rows(REF,'revision_events.csv'),rows(NEW,'revision_events.csv')
    assert len(a)==len(b)
    ignore={'update_seconds','current_position_correction_m','endpoint_position_correction_m'}
    equal=all({k:v for k,v in x.items() if k not in ignore}=={k:v for k,v in y.items() if k not in ignore} for x,y in zip(a,b))
    return dict(rows=len(a),identities_slots_outcomes_equal=equal)


def main():
    output=ROOT/'mismatch_analysis.json'
    assert not output.exists(), 'Refuse overwrite'
    result=dict(reference=str(REF),private=str(NEW),
        final_states=state_comparison('states.csv'),
        online_states=state_comparison('online_states.csv'),
        publication_decisions=decision_comparison(),revision_identity=revisions(),
        gate_remains_failed=True,
        inference='Rebuilt diagnostic path is not exact; no full timing attribution is accepted.')
    with output.open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__': main()
