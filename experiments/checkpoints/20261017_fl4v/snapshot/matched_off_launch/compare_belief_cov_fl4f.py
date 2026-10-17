"""Match final observed versions by physical source edge; compare trace, no tuning."""
import collections
import csv
import json
import statistics
from pathlib import Path

P=Path(__file__).resolve().parent
OUT=P/'evaluation_full/belief_covariance_matched.json'
assert not OUT.exists()

def last(mode):
    path=P/f'live120_{mode}_receiver/events.csv'
    result={}
    counts=collections.Counter()
    with path.open(newline='') as f:
        for r in csv.reader(f):
            if r[0]!='B':continue
            key=(r[2],r[6],r[7])
            cov=list(map(float,r[16:]))
            rotation=cov[0]+cov[7]+cov[14]
            translation=cov[21]+cov[28]+cov[35]
            result[key]=dict(start_ns=r[8],end_ns=r[9],rotation_trace=rotation,
                             translation_trace=translation)
            counts[key]+=1
    return result,counts

def summarize(x):
    a=sorted(x)
    return dict(n=len(a),mean=statistics.fmean(a),median=statistics.median(a),
                p95=a[min(len(a)-1,int(.95*len(a)))],minimum=a[0],maximum=a[-1],
                fraction_below_one=sum(v<1 for v in a)/len(a))

off,oc=last('off');on,nc=last('on')
assert set(off)==set(on)
assert len(off)>=1700
report={}
for source in ('glim_hesai','kimera_front'):
    keys=[key for key in off if key[0]==source]
    assert all(off[k]['start_ns']==on[k]['start_ns'] and off[k]['end_ns']==on[k]['end_ns'] for k in keys)
    report[source]=dict(matched_edges=len(keys),
        off_versions=sum(oc[k] for k in keys),on_versions=sum(nc[k] for k in keys),
        final_rotation_trace_on_over_off=summarize([on[k]['rotation_trace']/off[k]['rotation_trace'] for k in keys]),
        final_translation_trace_on_over_off=summarize([on[k]['translation_trace']/off[k]['translation_trace'] for k in keys]))
with OUT.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
