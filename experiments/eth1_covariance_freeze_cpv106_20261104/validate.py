"""Saved receiver output/payload audit; never launches an optimizer."""
import collections,csv,math,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
sys.path.insert(0,str(P))
from expiry_audit import ExpiryAudit
assert read(H/'replay_complete.json')['passed'] and read(H/'baseline_parity.json')['passed']
incoming={};eventmeta=[]
with (H/'variant_events.csv').open() as f:
    for row in csv.reader(f):
        eventmeta.append((row[0],row[1]))
        if row[0]=='B':
            key=(row[2],row[3],row[6],row[7],row[5]);assert key not in incoming
            incoming[key]=row
out=H/'frozen_kimera';summary=read(out/'summary.json')
assert summary==read(H/'baseline/summary.json')
actions=collections.Counter();expiry=ExpiryAudit();meanerr=0.;coverr=0.;state_rows={'online_incremental':[],'finalized_incremental':[]}
first=read(H/'first_accepted_covariances.json');first_seen=set();cov_count=0
for directory in sorted((out/'archive/commits').iterdir(),key=lambda p:int(p.name)):
    with (directory/'records.txt').open() as f:
        for row in csv.reader(f):
            expiry.consume(row,int(directory.name))
            if row[:2]==['belief','sealed_reference']:continue
            if row[0]=='belief':
                b=incoming.pop(tuple(row[2:7]));actions[row[1]]+=1
                assert row[7]==b[4] and row[8:11]==b[8:10]+[b[1]]
                meanerr=max(meanerr,max(abs(float(x)-float(y)) for x,y in zip(row[13:19],b[10:16])))
                coverr=max(coverr,max(abs(float(x)-float(y)) for x,y in zip(row[19:],b[16:])))
                if row[2]=='kimera_front' and row[1] in ('insert','replace'):
                    key='|'.join(row[2:6]);expected=first[key]['input_covariance_tokens']
                    assert np.array_equal(np.asarray(row[19:],float),np.asarray(expected,float))
                    cov_count+=1
                    if row[1]=='insert':first_seen.add(key)
            elif row[0] in state_rows:state_rows[row[0]].append(row)
assert not incoming and coverr==0 and meanerr<1e-12
assert first_seen==set(first)
expired=expiry.finish(summary)
assert actions['insert']==summary['inserted'] and actions['replace']==summary['replaced']
assert actions['before_initialization']==25 and actions['expired_revision']==134
assert sum(actions.values())==202995 and summary['unsupported_end']==summary['pending_at_end']==0
for name,kind in [('online_states.csv','online_incremental'),('finalized_states.csv','finalized_incremental')]:
    with (out/name).open() as f:rows=list(csv.DictReader(f))
    with (H/'baseline'/name).open() as f:base=list(csv.DictReader(f))
    assert len(rows)==len(base)==len(state_rows[kind])==3456
    for i,(r,b,c) in enumerate(zip(rows,base,state_rows[kind])):
        assert r['state_index']==b['state_index']==str(i)
        for k in ('timestamp_ns','availability_ns'):assert r[k]==b[k]
        assert list(r.values())==[c[1],c[2]]+c[4:]+[c[3]]
        assert all(math.isfinite(float(v)) for v in r.values())
        assert abs(sum(float(r[k])**2 for k in ('qx','qy','qz','qw'))-1)<1e-7
with (out/'stream_progress.csv').open() as f:
    for i,(r,(kind,arrival)) in enumerate(zip(csv.DictReader(f),eventmeta,strict=True),1):
        assert int(r['event'])==i and r['kind']==kind and r['arrival_ns']==arrival
check(read(H/'input_sha256.json'))
assert sha(H/'variant_events.csv')==read(H/'prepared.json')['variant_events_sha256']
save('validation.json',dict(passed=True,finished=now(),summary=summary,actions=dict(actions),
    frozen_Kimera_inserted_IDs=len(first_seen),frozen_Kimera_accepted_versions=cov_count,
    covariance_payload_error=coverr,mean_log_roundtrip_error=meanerr,
    expiry=expired,identical_publication_and_availability_timestamps=True,
    no_unsupported_EOF=True,no_stationarity_or_calibration_claim=True))
save('expired_details.json',expiry.details)
print(json.dumps(read(H/'validation.json'),indent=2))
