"""Matched saved-journal timing only after every exact validation gate passes."""
import csv
import json
from pathlib import Path
import statistics

ROOT=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013')
ORIGIN=1727775008003475000
def stats(x):
    x=sorted(x)
    if not x:return dict(n=0)
    def percentile(q):
        p=(len(x)-1)*q;i=int(p);return x[i]+(x[min(i+1,len(x)-1)]-x[i])*(p-i)
    return dict(n=len(x),mean=statistics.mean(x),median=statistics.median(x),
                p95=percentile(.95),max=max(x),sum=sum(x))
def read(path):
    with path.open() as f:return list(csv.DictReader(f))
result=dict(cases={},pairs=[],scope='Existing148.58s saved journal; NOT live throughput certification')
result['endpoint_counters_repeat_exact']=(ROOT/'full_on1/e2b_endpoints.csv').read_bytes()==(ROOT/'full_on2/e2b_endpoints.csv').read_bytes()
assert result['endpoint_counters_repeat_exact']
for tag in ['full_off1','full_on1','full_on2','full_off2']:
    run=json.loads((ROOT/(tag+'.json')).read_text());assert run['passed']
    pubs=read(ROOT/tag/'publication_convergence.csv')
    steps=read(ROOT/tag/'publication_steps.csv')
    recovered={int(r['index']) for r in steps if r['phase']=='support' and r['recovery']=='1'}
    # Wall interval between consecutive support-completion records includes
    # intervening saved-stream dispatch/revision work. Offline service cost,
    # not live arrival-to-publication latency or a queue measurement.
    progress=read(ROOT/tag/'stream_progress.csv'); completions=[];previous=None
    for r in progress:
        if previous is None:
            previous=r;continue
        if int(r['created_support'])>int(previous['created_support']):
            completions.append(((int(r['arrival_ns'])-ORIGIN)/1e9,
                (int(r['wall_unix_ns'])-int(previous['wall_unix_ns']))/1e6))
            previous=r
    local=dict(seconds=run['seconds'],max_rss_kib=run['max_rss_kib'],cohorts={})
    cache_path=ROOT/tag/'e2b_endpoints.csv'
    cache=read(cache_path) if cache_path.exists() else []
    for label,lo,hi in [('all',0,1e9),('0-40s',0,40),('60-80s',60,80),('120-140s',120,140),('140s-end',140,1e9)]:
        rows=[r for r in pubs if r['phase']=='support' and lo<=(int(r['physical_ns'])-ORIGIN)/1e9<hi]
        cohort=dict(publication_ms=stats([1000*float(r['seconds']) for r in rows]),
                    support_completion_interval_ms=stats([ms for t,ms in completions if lo<=t<hi]),
                    nonrecovery_ms=stats([1000*float(r['seconds']) for r in rows if int(r['index']) not in recovered]),
                    attempts=stats([int(r['attempted']) for r in rows]),
                    over_1000ms=sum(float(r['seconds'])>1 for r in rows),
                    recovery_publications=sum(int(r['index']) in recovered for r in rows))
        previous={key:0 for key in ['calls','hits','misses','evaluations','failures','nonfinite','evictions']};totals=previous.copy()
        for r in cache:
            current={k:int(r[k]) for k in previous}
            if r['phase']=='support' and lo<=(int(r['physical_ns'])-ORIGIN)/1e9<hi:
                for k in totals:totals[k]+=current[k]-previous[k]
            previous=current
        if cache:
            cohort['cache']=totals
            cohort['cache']['hit_fraction']=totals['hits']/totals['calls'] if totals['calls'] else 0
        local['cohorts'][label]=cohort
    if cache:
        local['final_cache']=cache[-1]
        assert int(cache[-1]['peak_entries'])<=8192
        assert int(cache[-1]['failures'])==int(cache[-1]['nonfinite'])==0
    result['cases'][tag]=local
for off,on in [('full_off1','full_on1'),('full_off2','full_on2')]:
    a,b=result['cases'][off],result['cases'][on]
    pair=dict(off=off,on=on,wall_reduction=1-b['seconds']/a['seconds'],late={})
    for cohort in ['120-140s','140s-end']:
        pair['late'][cohort]={stat:1-b['cohorts'][cohort]['publication_ms'][stat]/a['cohorts'][cohort]['publication_ms'][stat] for stat in ['median','p95']}
    pair['gain_gate']=pair['wall_reduction']>0 and all(c['p95']>0 for c in pair['late'].values())
    result['pairs'].append(pair)
result['gain_gate']=all(p['gain_gate'] for p in result['pairs'])
with (ROOT/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
