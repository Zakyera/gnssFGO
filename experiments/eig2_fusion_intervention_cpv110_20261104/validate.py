"""Audit each intervention's actual consumed payload, lifecycle and timestamps."""
import collections,csv,math,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
assert read(H/'replay_complete.json')['passed'] and read(H/'baseline_parity.json')['passed']
anchors=read(H/'anchors.json');prep=read(H/'prepared.json');result={}
for mode in MODES[1:]:
    assert not (H/(mode+'_validation.json')).exists()
    incoming={};eventmeta=[]
    with (H/(mode+'_events.csv')).open() as f:
        for row in csv.reader(f):
            eventmeta.append((row[0],row[1]))
            if row[0]=='B':
                key=(row[2],row[3],row[6],row[7],row[5]);assert key not in incoming;incoming[key]=row
    out=H/mode;summary=read(out/'summary.json');assert summary==read(H/'baseline/summary.json')
    actions=collections.Counter();meanerr=0.;coverr=0.;target=0;ids=set();state_rows={'online_incremental':[],'finalized_incremental':[]}
    for directory in sorted((out/'archive/commits').iterdir(),key=lambda p:int(p.name)):
        with (directory/'records.txt').open() as f:
            for row in csv.reader(f):
                if row[:2]==['belief','sealed_reference']:continue
                if row[0]=='belief':
                    b=incoming.pop(tuple(row[2:7]));actions[row[1]]+=1
                    assert row[7]==b[4] and row[8:11]==b[8:10]+[b[1]]
                    meanerr=max(meanerr,max(abs(float(x)-float(y)) for x,y in zip(row[13:19],b[10:16])))
                    coverr=max(coverr,max(abs(float(x)-float(y)) for x,y in zip(row[19:],b[16:])))
                    key='|'.join(row[2:6]);anchor=anchors.get(key)
                    if anchor and int(row[10])-int(row[9])>2_500_000_000:
                        assert row[1]=='replace';target+=1;ids.add(key)
                        if mode in ('hold_mean','hold_both'):assert np.max(abs(np.array(row[13:19],float)-np.array(anchor['input_mean'],float)))<1e-12
                        if mode in ('hold_cov','hold_both'):assert np.array_equal(np.array(row[19:],float),np.array(anchor['input_covariance'],float))
                elif row[0] in state_rows:state_rows[row[0]].append(row)
    assert not incoming and coverr==0 and meanerr<1e-12
    assert len(ids)==len(anchors) and target==prep['changes'][mode]['eligible_rows']
    assert sum(actions.values())==213750 and actions['insert']==summary['inserted'] and actions['replace']==summary['replaced']
    assert summary['excluded']==50 and summary['unsupported_end']==1 and summary['expired']==summary['pending_at_end']==0
    for name,kind in [('online_states.csv','online_incremental'),('finalized_states.csv','finalized_incremental')]:
        with (out/name).open() as f:rows=list(csv.DictReader(f))
        with (H/'baseline'/name).open() as f:base=list(csv.DictReader(f))
        assert len(rows)==len(base)==len(state_rows[kind])==3545
        for i,(r,b,c) in enumerate(zip(rows,base,state_rows[kind])):
            assert r['state_index']==b['state_index']==str(i)
            for k in ('timestamp_ns','availability_ns'):assert r[k]==b[k]
            assert list(r.values())==[c[1],c[2]]+c[4:]+[c[3]]
            assert all(math.isfinite(float(v)) for v in r.values())
            assert abs(sum(float(r[k])**2 for k in ('qx','qy','qz','qw'))-1)<1e-7
    ignored={'processed_wall_ns','service_seconds','rss_kib'}
    with (out/'stream_progress.csv').open() as f,(H/'baseline/stream_progress.csv').open() as bf:
        for i,(r,b,(kind,arrival)) in enumerate(zip(csv.DictReader(f),csv.DictReader(bf),eventmeta,strict=True),1):
            assert int(r['event'])==i and r['kind']==kind and r['arrival_ns']==arrival
            assert {k:v for k,v in r.items() if k not in ignored}=={k:v for k,v in b.items() if k not in ignored}
    assert sha(H/(mode+'_events.csv'))==prep['variant_sha256'][mode]
    result[mode]=dict(passed=True,at=now(),summary=summary,actions=dict(actions),eligible_IDs=len(ids),held_accepted_revisions=target,covariance_payload_error=coverr,mean_log_roundtrip_error=meanerr,identical_logical_progress=True,identical_publication_and_availability=True)
    save(mode+'_validation.json',result[mode]);print(mode+' validated',flush=True)
check(read(H/'input_sha256.json'))
save('validation.json',dict(passed=True,at=now(),variants=result,receiver_replays=4,source_runs=0))
