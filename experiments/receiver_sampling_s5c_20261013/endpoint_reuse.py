"""Static endpoint reuse opportunity; not measured savings or a cache patch."""
from collections import Counter,defaultdict
import csv,json
from pathlib import Path
from symbolize import ROOT
CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver')
active={};received={}
with (ROOT/'reference/causal_dispatch.csv').open() as f:
    for row in csv.DictReader(f):
        if row['outcome'] in ['insert','replace']:
            active[(row['source'],row['session'],row['from_index'],row['to_index'])]=row['revision']
with (CAPTURE/'events.csv').open() as f:
    for r in csv.reader(f):
        if r[0]=='B':
            key=(r[2],r[3],r[6],r[7])
            if active.get(key)==r[5]:received[key]=(int(r[8]),int(r[9]))
assert len(received)==2201
sources=defaultdict(list)
for key,pair in received.items():sources[key[0]].extend(pair)
result={}
for source,endpoints in sources.items():
    count=Counter(endpoints)
    result[source]=dict(active_edges=len(endpoints)//2,endpoint_uses=len(endpoints),
        unique_endpoint_times=len(count),multiplicity_histogram=dict(Counter(count.values())),
        duplicate_uses=len(endpoints)-len(count))
result['scope']='Static final-graph endpoint identity only. No timing speedup, cache-hit rate or reuse across changed Values is inferred.'
output=ROOT/'endpoint_reuse_opportunity.json';assert not output.exists()
with output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
