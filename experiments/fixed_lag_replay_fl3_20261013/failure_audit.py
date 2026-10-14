"""Audit only retained failed-run artifacts; NEVER rerun or publish partial output."""
import collections,csv,json,math,re
import numpy as np
from driver import ROOT,JOURNAL,ORACLE,ORIGIN,protected
from audit_prefix import rows,dicts

def main():
    run=json.loads((ROOT/'full_timed.json').read_text());assert run['returncode']==1
    log=(ROOT/'full_timed.log').read_text()
    match=re.search(r'FL3_STREAM_FAILURE event=(\d+) kind=(.) arrival=(\d+) reason=(.*)',log)
    event,kind,arrival,reason=match.groups();event=int(event);arrival=int(arrival)
    root=ROOT/'full_timed_outputs';groups=collections.defaultdict(list);active={};absorbed={};counts=collections.Counter()
    maxactive=0;lastbundle=0
    for path in sorted((root/'commits').iterdir(),key=lambda p:int(p.name)):
        lastbundle=int(path.name);assert lastbundle<event
        for r in rows(path/'records.txt'):
            groups[r[0]].append(r)
            if r[0]=='belief':
                key=tuple(r[2:6]);action=r[1];counts[action]+=1
                if action=='before_initialization':continue
                assert key not in absorbed
                if action=='insert':assert key not in active
                else:assert action=='replace' and key in active and int(r[6])>active[key][0]
                active[key]=(int(r[6]),int(r[12]));maxactive=max(maxactive,len(active))
                assert len(set(v[1] for v in active.values()))==len(active)
            elif r[0]=='absorbed':
                key=tuple(r[1:5]);assert active.pop(key)==(int(r[5]),int(r[6]));assert key not in absorbed
                absorbed[key]=(int(r[5]),int(r[6]))
    assert not groups['end'] and not groups['drain']
    assert not list(root.glob('.stage_*'))
    assert len([p for p in (root/'ids').rglob('*') if p.is_file()])==len(absorbed)
    online=groups['online'];pubs=groups['publication'];final=groups['finalized']
    assert [int(r[1]) for r in online]==list(range(len(online)))
    assert [int(r[1]) for r in final]==list(range(len(final)))
    assert len(online)==len(pubs) and len(active)+len(absorbed)==counts['insert']
    assert all(float(r[3])<=1e-5 and int(r[4])<=20 for r in pubs)
    with (ROOT/'full_timed_service.csv').open() as f:service=list(csv.DictReader(f))
    last=service[-1];assert int(last['event'])==event-1
    assert len(online)==int(last['publications']) and len(final)==int(last['oldest'])
    # Compare full ordered admitted prefix to the independent full-history oracle.
    actual=[tuple(r[2:7])+tuple(r[10:12])+(r[1],) for r in groups['belief'] if r[1] in ('insert','replace')]
    oracle=[r for r in dicts(ORACLE/'causal_dispatch.csv') if int(r['insertion_ns'])<=int(last['arrival_ns'])]
    fields=('source','session','from_index','to_index','revision','arrival_ns','insertion_ns','outcome')
    assert actual==[tuple(r[k] for k in fields) for r in oracle]
    raw={};inputcounts=collections.Counter();failureinput=None
    with JOURNAL.open() as f:
        for i,line in enumerate(f,1):
            r=next(csv.reader([line]))
            if i==event:failureinput=r;break
            inputcounts[r[0]]+=1
            if r[0]=='B':
                key=(r[2],r[3],r[6],r[7],r[5]);assert key not in raw;raw[key]=r
    maxcov=maxmean=0.
    for r in groups['belief']:
        b=raw.pop(tuple(r[2:7]));assert r[7]==b[4] and r[8:11]==b[8:10]+[b[1]]
        maxcov=max(maxcov,float(np.max(np.abs(np.array(r[19:],float)-np.array(b[16:],float)))))
        maxmean=max(maxmean,float(np.max(np.abs(np.array(r[13:19],float)-np.array(b[10:16],float)))))
    assert maxcov==0 and maxmean<1e-12
    assert len(raw)==int(last['pending'])==1
    pending=list(raw.values())[0];required=(int(pending[9])-ORIGIN+99999999)//100000000
    assert required>int(last['newest'])
    assert inputcounts['B']==sum(counts.values())+len(raw)
    clock=dicts(ORACLE/'support_clock.csv')[:len(online)]
    for r,c in zip(online,clock):assert r[1:4]==[c['state_index'],c['physical_timestamp_ns'],c['available_timestamp_ns']]
    warnings=re.findall(r'VectorValues Update: key ([xvw])(\d+) does not exist!',log)
    # The library can remove a complete clique without the transitional diagnostic;
    # warnings need not occur at EVERY retirement. None may name retained keys.
    retired_keys={(k,str(i)) for i in range(len(final)) for k in ('x','v','w')}
    assert set(warnings)<=retired_keys and len(set(warnings))==len(warnings)
    reference=dicts(ORACLE/'online_states.csv');diff=[]
    for r in online:
        i=int(r[1]);x=np.array(r[4:],float);y=np.array([reference[i][k] for k in ('px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz')],float)
        q=x[3:7]/np.linalg.norm(x[3:7]);p=y[3:7]/np.linalg.norm(y[3:7])
        angle=2*math.atan2(np.linalg.norm(p[3]*q[:3]-q[3]*p[:3]-np.cross(p[:3],q[:3])),abs(q@p))
        diff.append([i,np.linalg.norm(x[:3]-y[:3]),angle,np.linalg.norm(x[7:10]-y[7:10]),np.linalg.norm(x[10:13]-y[10:13])])
    d=np.array(diff);summary=lambda a:dict(max=np.max(a,axis=0).tolist(),rms=np.sqrt(np.mean(a*a,axis=0)).tolist())
    priorprefix=[]
    for p in sorted((ROOT/'prefix_gate_outputs/commits').iterdir(),key=lambda p:int(p.name)):
        priorprefix.extend([r for r in rows(p/'records.txt') if r[0] in ('online','publication')])
    nowprefix=[r for i in range(len(priorprefix)//2) for r in (online[i],pubs[i])]
    assert priorprefix==nowprefix,'full-start versus accepted prefix determinism failed'
    fullhistory_at_failure=[r for r in dicts(ORACLE/'publication_convergence.csv') if r['index']==str(len(online))]
    result=dict(status='FL3_STOPPED_PRE_RETIREMENT_STATIONARITY',failure_event=event,failure_kind=kind,
      failure_arrival_ns=arrival,failed_support_index=len(online),failed_physical_ns=ORIGIN+len(online)*100000000,
      support_seconds_from_origin=len(online)*.1,seconds_from_first_IMU=(arrival-1727775008003475000)*1e-9,
      reason=reason,failed_gradient='not recorded',failed_refinement_steps='not recorded; may stop on budget or cost safeguard',
      last_committed_event=event-1,last_output_bundle=lastbundle,input_counts=dict(inputcounts),outcomes=dict(counts),
      pending_versions=len(raw),pending_version=pending[:10],pending_required_support=required,
      accepted_publications=len(online),actual_retirements=len(final),retained_supports=len(online)-len(final),
      active_ids=len(active),absorbed_ids=len(absorbed),peak_active_ids=maxactive,
      maximum_accepted_gradient=max(float(r[3]) for r in pubs),maximum_accepted_steps=max(int(r[4]) for r in pubs),
      last_publications=pubs[-10:],failing_support_fullhistory_reference=fullhistory_at_failure,
      admitted_order_identical=True,covariance_payload_max_error=maxcov,log_mean_roundtrip_max_error=maxmean,
      missing_retired_key_warnings=len(warnings),retired_keys_without_warning=sorted(retired_keys-set(warnings)),no_failed_event_output=True,no_staging_left=True,
      no_complete_or_drain_marker=True,prefix_outputs_exact=True,
      state_difference_columns=['p_m','orientation_rad','V_mps','W_radps'],
      online_fullhistory_difference=summary(d[:,1:]),before_retirement_difference=summary(d[:71,1:]),
      protected_unchanged=protected()==json.loads((ROOT/'protected.json').read_text()),
      not_run=['modified retry','full deterministic repeat','full accuracy scoring','real covariance comparison','live capture'])
    with (ROOT/'failure_audit.json').open('x') as f:json.dump(result,f,indent=2)
    with (ROOT/'partial_online_difference.csv').open('x') as f:
        w=csv.writer(f);w.writerow(['index']+result['state_difference_columns']);w.writerows(diff)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
