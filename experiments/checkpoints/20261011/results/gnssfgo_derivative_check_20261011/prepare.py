"""Extract only factors touching the previously identified top-20 coordinates."""
from pathlib import Path
import csv,json,hashlib
ROOT=Path(__file__).resolve().parent
OLD=Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/cached')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_covariance160_validated_20261010/prepared/full160_on_receiver/events.csv')
def rows(p):return list(csv.DictReader(p.open()))
targets=rows(OLD/'failure_probe/gradient_step_sensitivity.csv')
unique={int(x['rank']):(x['key'],int(x['column'])) for x in targets}
keys={k for k,c in unique.values()}
lines=(OLD/'failure_probe/factor_probe.txt').read_text().splitlines();factors={};i=0
while i<len(lines):
    slot,typ=lines[i].split(' ',1);ks=lines[i+1].split();assert lines[i+2]=='covariance'
    n=len(lines[i+3].split());cov=[list(map(float,lines[i+3+j].split())) for j in range(n)]
    assert lines[i+3+n]=='residual'
    residual=[float(lines[i+4+n+j]) for j in range(n)]
    if keys.intersection(ks):factors[int(slot)]=dict(type=typ,keys=ks,covariance=cov,residual=residual)
    i+=4+2*n
dispatch=rows(OLD/'causal_dispatch.csv');revisions=rows(OLD/'revision_events.csv');assert len(dispatch)==len(revisions)
active={}
for d,r in zip(dispatch,revisions):
    assert all(d[k]==r[k] for k in ('from_index','to_index','revision','outcome'))
    assert d['arrival_ns']==r['record_timestamp_ns']
    active[int(r['new_factor_index'])]=(d['source'],d['session'],d['from_index'],d['to_index'],d['revision'])
needed={active[s]:s for s,x in factors.items() if 'WnoaRelativeFactor' in x['type']}
with JOURNAL.open() as f:
    for row in csv.reader(f):
        if row[0]!='B':continue
        key=(row[2],row[3],row[6],row[7],row[5])
        if key in needed:
            slot=needed[key];assert 'event' not in factors[slot]
            factors[slot]['event']=row
assert all('event' in x or 'GPWNOAPrior' in x['type'] for x in factors.values())
with (ROOT/'selected_factors.csv').open('x') as f:
    w=csv.writer(f)
    for s,x in sorted(factors.items()):
        if 'event' in x:w.writerow([s,'source',*x['event'][8:]])
        else:
            a=int(x['keys'][0][1:]);b=int(x['keys'][3][1:]);w.writerow([s,'wnoa',a,b])
with (ROOT/'expected.csv').open('x') as f:
    w=csv.writer(f)
    for s,x in sorted(factors.items()):w.writerow([s,len(x['residual']),*x['residual'],*(v for row in x['covariance'] for v in row)])
with (ROOT/'targets.csv').open('x') as f:
    csv.writer(f).writerows((rank,k,c) for rank,(k,c) in sorted(unique.items()))
inputs=[JOURNAL,OLD/'failure_probe/simultaneous_values.csv',OLD/'failure_probe/factor_probe.txt',OLD/'causal_dispatch.csv',OLD/'revision_events.csv']
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
out=dict(selected_factors=factors,targets=unique,input_hashes={str(p):sha(p) for p in inputs})
with (ROOT/'selection.json').open('x') as f:json.dump(out,f,indent=2)
print('Selected',len(factors),'factors for',len(unique),'coordinates; no optimization or replay.')
