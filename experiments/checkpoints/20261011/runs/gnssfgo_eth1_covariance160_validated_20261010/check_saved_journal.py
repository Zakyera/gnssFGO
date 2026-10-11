"""Conservation of the captured partial input, not successful graph completion."""
import csv
import json
from pathlib import Path
import sys
sys.path.insert(0,'/home/yeranis/repos/gnssFGO/tools')
from live_receiver_protocol import parse_belief
ROOT=Path(__file__).resolve().parent
P=ROOT/'prepared';S=P/'full160_on_source';L=P/'full160_on_receiver'
glog,=[p for p in (S/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
expected={}
for kind,sid,path in [('glim','glim_hesai',glog),('kimera','kimera_front',S/'estimators.log')]:
    for line in path.open():
        b=parse_belief(kind,line)
        if b is None:continue
        key=(sid,b['i'],b['j']);payload=(b['ti'],b['tj'],*b['mean'],*b['covariance'])
        h=expected.setdefault(key,[])
        if not h or h[-1]!=payload:h.append(payload)
received={}
for r in csv.reader((L/'events.csv').open()):
    if r[0]!='B':continue
    key=(r[2],int(r[6]),int(r[7]));h=received.setdefault(key,[])
    assert int(r[5])==len(h)
    h.append((int(r[8]),int(r[9]),*map(float,r[10:])))
dispatch=list(csv.DictReader((L/'receiver/causal_dispatch.csv').open()))
unsupported=list(csv.DictReader((L/'receiver/unsupported_beliefs.csv').open()))
result=dict(captured_payloads_and_order_conserved=(received==expected),
    exported_versions=sum(map(len,expected.values())),journal_versions=sum(map(len,received.values())),
    stable_ids_in_journal=len(received),graph_dispatch_records=len(dispatch),unsupported_records=len(unsupported),
    unsupported_reasons=sorted({r['reason'] for r in unsupported}),
    graph_completed=False,
    note='Partial capture ended by controlled stop; receiver failed independently on strict publication. Journal conservation is not a completed 160s graph or successful final trajectory.')
with (ROOT/'partial_journal_validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
