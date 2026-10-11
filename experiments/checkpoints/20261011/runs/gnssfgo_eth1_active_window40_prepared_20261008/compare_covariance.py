"""Bounded, read-only comparison of existing exact physical source intervals.

No estimator execution, tuning, new model, or final-120s trajectory comparison.
"""
import csv
import importlib.util
import json
from pathlib import Path
from collections import defaultdict
import sys
sys.dont_write_bytecode = True
import numpy as np

ROOT = Path(__file__).resolve().parent
OLD = ROOT.parent/'gnssfgo_eth1_live_wnoa_20261007'
PARSER = Path('/home/yeranis/repos/gnssFGO/experiments/covariance_contraction_events_eth1_20261008/analyze.py')
spec = importlib.util.spec_from_file_location('prior_audit', PARSER)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
q = audit.q
MODES = dict(off=OLD/'full_off', persistent_on=OLD/'bounded120_on', temporary_on=ROOT/'pilot_on')
new_events = ROOT/'pilot_on_receiver/events.csv'
clocks = [int(r[1]) for r in csv.reader(new_events.open()) if r[0]=='I']
start, cut = min(clocks), max(clocks)
H = {}
retired = {}
protected = {str(PARSER):q.sha(PARSER), str(Path(__file__)):q.sha(Path(__file__))}
for mode,base in MODES.items():
    journal = Path(str(base)+'_receiver/events.csv')
    source = Path(str(base)+'_source')
    protected[str(journal)] = q.sha(journal)
    history = defaultdict(list)
    for r in csv.reader(journal.open()):
        if r[0]!='B' or not (start <= int(r[8]) < int(r[9]) <= int(r[1]) <= cut):
            continue
        key=(r[2],int(r[8]),int(r[9]))
        x=dict(arrival=int(r[1]),revision=int(r[5]),i=int(r[6]),j=int(r[7]),
               mu=np.array(r[10:16],float),C=np.array(r[16:],float).reshape(6,6))
        if history[key]:
            assert x['revision']>history[key][-1]['revision']
        history[key].append(x)
    H[mode] = history
    protected[str(source/'estimators.log')]=q.sha(source/'estimators.log')
    protected[str(source/'kimera_odometry.csv')]=q.sha(source/'kimera_odometry.csv')
    raw, _, retires, _, _, _ = audit.parse_main(source/'estimators.log')
    times = {int(r['field.header.seq']):int(r['field.header.stamp'])
             for r in csv.DictReader((source/'kimera_odometry.csv').open())}
    valid=set()
    for key,h in history.items():
        if key[0]!='kimera_front': continue
        edge=(h[0]['i'],h[0]['j'])
        rr = retires.get(edge)
        if rr is None or times.get(rr['u'],cut+1)>cut: continue
        # Prove the end-of-life record is the same payload received downstream.
        r=raw[edge][-1]
        assert rr['u']==r['u']+1
        if not np.array_equal(r['C'],h[-1]['C']) or not np.array_equal(r['mu'],h[-1]['mu']):
            continue
        valid.add(key)
    retired[mode]=valid

common=set.intersection(*(set(h) for h in H.values()))
result=dict(start_ns=start,cutoff_ns=cut,duration_s=(cut-start)*1e-9,
    interval_match='exact source + physical from/to nanoseconds, never nearest index',
    reported_uncertainty='sqrt(trace) of 3x3 marginal; radians for rotation, metres for rho; not per-axis sigma',
    reference='IE-TC same existing SE(3) discrepancy helper; shares CPT7, no independent reference covariance',
    sources={},fully_retired_kimera={})
for source in ['glim_hesai','kimera_front']:
    keys=sorted(k for k in common if k[0]==source and all(H[m][k][0]['arrival']+2_000_000_000<=cut for m in H))
    rows={}
    for mode,hist in H.items():
        parts={}
        for part,sl in audit.PARTS.items():
            values=defaultdict(list)
            for k in keys:
                h=hist[k]
                selected=dict(first=h[0],age2=next(r for r in reversed(h) if r['arrival']<=h[0]['arrival']+2_000_000_000),last_at_cut=h[-1])
                sig={age:np.sqrt(np.trace(r['C'][sl,sl])) for age,r in selected.items()}
                for age,x in sig.items(): values[age+'_sqrt_trace'].append(x)
                values['last_over_age2'].append(sig['last_at_cut']/sig['age2'])
                ref=q.ref.relative_reference(k[1],k[2])
                if ref is not None:
                    for age,r in selected.items():
                        e,cart=q.ref.discrepancy(r['mu'],ref)
                        values[age+'_reference_log_error'].append(np.linalg.norm(e[sl]))
            parts[part]={name:q.stats(v) for name,v in values.items()}
        rows[mode]=parts
    result['sources'][source]=dict(matched_edges=len(keys),modes=rows,
        note='last_at_cut is not necessarily last jointly valid; right-censored windows retained explicitly')

keys=sorted(set.intersection(*retired.values()))
result['fully_retired_kimera']['matched_edges']=len(keys)
for mode in H:
    stats={}
    for part,sl in audit.PARTS.items():
        values=defaultdict(list)
        for key in keys:
            h=H[mode][key]
            assert len(h)>6
            sig=np.array([np.sqrt(np.trace(r['C'][sl,sl])) for r in h])
            a2=max(n for n,r in enumerate(h) if r['arrival']<=h[0]['arrival']+2_000_000_000)
            values['last_over_age2'].append(sig[-1]/sig[a2])
            values['last_step_ratio'].append(sig[-1]/sig[-2])
            values['last6_steps_ratio'].append(sig[-1]/sig[-7])
            values['last_trace'].append(sig[-1]**2)
            values['last_sqrt_trace'].append(sig[-1])
            values['version_count'].append(len(h))
            ref=q.ref.relative_reference(key[1],key[2])
            if ref is not None:
                for name,r in [('first',h[0]),('age2',h[a2]),('last',h[-1])]:
                    e,_=q.ref.discrepancy(r['mu'],ref)
                    values[name+'_reference_log_error'].append(np.linalg.norm(e[sl]))
        stats[part]={name:q.stats(v) for name,v in values.items()}
    result['fully_retired_kimera'][mode]=stats
for p,sha in protected.items(): assert q.sha(p)==sha, p
with (ROOT/'covariance_comparison.json').open('x') as f:
    json.dump(result,f,indent=2,allow_nan=False)
with (ROOT/'covariance_input_hashes.json').open('x') as f:
    json.dump(protected,f,indent=2)
print(json.dumps(result['fully_retired_kimera'],indent=2))
