"""Compare frozen and cache-only receiver outputs, not trajectory accuracy."""
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parent
ORIGINAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_covariance160_validated_20261010/prepared/full160_on_receiver/receiver')
T0=1727775008003475000
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(values):
    v=sorted(map(float,values))
    if not v:return dict(n=0)
    k=(len(v)-1)*.95;i=int(k)
    return dict(n=len(v),sum=sum(v),mean=statistics.mean(v),median=statistics.median(v),
                p95=v[i]+(v[min(i+1,len(v)-1)]-v[i])*(k-i),maximum=max(v))

def compare(a,b):
    result={}
    excluded={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},
              'stream_progress.csv':{'wall_unix_ns'}}
    for p in sorted(a.iterdir()):
        if not p.is_file():continue
        q=b/p.name
        if not q.is_file():result[p.name]=dict(missing=True);continue
        entry=dict(a_sha256=sha(p),b_sha256=sha(q),byte_identical=sha(p)==sha(q))
        if p.suffix=='.csv':
            ra,rb=rows(p),rows(q);drop=excluded.get(p.name,set())
            clean=lambda x:{k:v for k,v in x.items() if k not in drop}
            entry.update(a_rows=len(ra),b_rows=len(rb),excluded=sorted(drop),
                         semantic_identical=[clean(x) for x in ra]==[clean(x) for x in rb])
            if not entry['semantic_identical']:
                for i,(x,y) in enumerate(zip(ra,rb)):
                    if clean(x)!=clean(y):
                        entry['first_difference']=dict(row=i,a=clean(x),b=clean(y));break
        else:entry['semantic_identical']=entry['byte_identical']
        result[p.name]=entry
    return result

def timing(case):
    c=rows(ROOT/case/'publication_convergence.csv');r=rows(ROOT/case/'revision_events.csv')
    s=rows(ROOT/case/'publication_steps.csv')
    polished={x['index'] for x in s if x['local_prediction']=='1'}
    windows=[]
    for lo,hi in [(0,60),(60,100),(100,115),(115,120),(120,125),(125,127.8)]:
        selected=[x for x in c if lo<=(int(x['physical_ns'])-T0)/1e9<hi]
        rev=[x for x in r if lo<=(int(x['record_timestamp_ns'])-T0)/1e9<hi]
        windows.append(dict(elapsed=[lo,hi],publication=stats(x['seconds'] for x in selected),
                            revisions=stats(x['update_seconds'] for x in rev)))
    failure=[x for x in s if x['index']==c[-1]['index']]
    passes=len([x for x in failure if x['local_prediction']=='1' and x['recovery']=='1'])
    return dict(convergence_rows=len(c),successful=sum(x['status']=='stationary' for x in c),
                failed=c[-1],published_rows=len(rows(ROOT/case/'online_states.csv')),
                attempted_gradient_limit=1e-5,publication_total=stats(x['seconds'] for x in c),
                private_polish_only=stats(x['seconds'] for x in c if x['index'] in polished),
                unaffected_supports=stats(x['seconds'] for x in c if x['index'] not in polished),
                polished_supports=len(polished),windows=windows,failed_inner_passes=passes,
                removed_cost_linearizations_at_failure=passes*3,
                revision_events=len(r),revision_outcomes={k:sum(x['outcome']==k for x in r) for k in sorted({x['outcome'] for x in r})})

result=dict(original_vs_baseline=compare(ORIGINAL,ROOT/'baseline'),
            baseline_vs_cached=compare(ROOT/'baseline',ROOT/'cached'),
            baseline=timing('baseline'),cached=timing('cached'))
result['all_semantics_equal']=all(x.get('semantic_identical',False)
                                for name in ('original_vs_baseline','baseline_vs_cached') for x in result[name].values())
probe=ROOT/'cached/failure_probe'
if probe.exists():
    p=rows(probe/'gradient_step_sensitivity.csv');groups={}
    for row in p:groups.setdefault(row['rank'],[]).append(row)
    result['failure_probe']={rank:dict(key=g[0]['key'],column=g[0]['column'],
        native_normalized=float(g[0]['native_normalized']),
        step_series=[{k:float(x[k]) for k in ('step','numeric_gradient','numeric_diagonal','numeric_normalized','max_whitened_jacobian_difference')} for x in g])
        for rank,g in groups.items()}
    con=rows(probe/'native_gradient_contributions.csv');by_rank={}
    for row in con:
        d=by_rank.setdefault(row['rank'],{})
        typ='WNOA' if 'GPWNOAPrior' in row['type'] else 'source' if 'WnoaRelativeFactor' in row['type'] else row['type']
        bucket=d.setdefault(typ,dict(gradient=0.,diagonal=0.,factors=0))
        bucket['gradient']+=float(row['gradient_contribution']);bucket['diagonal']+=float(row['diagonal_contribution']);bucket['factors']+=1
    result['failure_probe_contributors']=by_rank
with (ROOT/'comparison.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:result[k] for k in ('all_semantics_equal','baseline','cached')},indent=2))
assert result['all_semantics_equal'],'STOP: replay differs beyond elapsed times'
