"""Read-only FL3 prefix audit, retaining pass/fail and missing diagnostics."""
import collections,csv,importlib.util,json,math,re,sys
from pathlib import Path
import numpy as np
from driver import ROOT,HERE,ORACLE,JOURNAL,FL3,protected,sha
ORIGIN=1727775008583475000
CUTOFF=ORIGIN+20_000_000_000

def rows(path):
    with path.open() as f:return list(csv.reader(f))
def dicts(path):
    with path.open() as f:return list(csv.DictReader(f))
def main(name='prefix'):
    full=name=='full'
    cutoff=2**63-1 if full else CUTOFF
    journal=JOURNAL if full else FL3/'prefix20.events'
    product='full' if full else 'prefix'
    out=ROOT/(name+'_outputs');records=[];bundles=[]
    for p in sorted((out/'commits').iterdir(),key=lambda p:int(p.name)):
        r=rows(p/'records.txt')
        for row in r:
            if row[0].endswith('_incremental'):row[0]=row[0].removesuffix('_incremental')
        records.extend(r);bundles.append((int(p.name),r))
    groups=collections.defaultdict(list)
    for r in records:groups[r[0]].append(r)
    active={};dead={};counts=collections.Counter();maxactive=0
    for r in records:
        if r[0]=='belief':
            action=r[1];counts[action]+=1;key=tuple(r[2:6]);version=int(r[6]);slot=int(r[12])
            if action in ('before_initialization','unsupported_at_end'):continue
            assert key not in dead
            if action=='insert':assert key not in active
            elif action=='replace':assert key in active and version>active[key][0]
            else:raise AssertionError(action)
            if key in active:assert r[8:10]==active[key][2]
            active[key]=(version,slot,r[8:10]);assert len(set(x[1] for x in active.values()))==len(active)
            maxactive=max(maxactive,len(active))
        elif r[0]=='absorbed':
            key=tuple(r[1:5]);prior=active.pop(key);assert prior[:2]==(int(r[5]),int(r[6]))
            assert key not in dead;dead[key]=prior
    assert len(groups['end'])==1 and len(groups['drain'])==1
    clock=[r for r in dicts(ORACLE/'support_clock.csv') if int(r['available_timestamp_ns'])<=cutoff]
    assert len(groups['online'])==len(clock)==len(groups['publication'])
    for i,(r,c) in enumerate(zip(groups['online'],clock)):
        assert list(map(int,r[1:4]))==[i,int(c['physical_timestamp_ns']),int(c['available_timestamp_ns'])]
    assert [r[1:3] for r in groups['online']]==[r[1:3] for r in groups['finalized']]
    assert all(r[3]=='not_checked' and int(r[4])<=1 for r in groups['publication'])
    assert groups['end'][0][2]=='not_checked'
    assert not list(out.glob('.stage_*'))
    assert all(np.isfinite(np.array(r[4:],float)).all() for k in ('online','finalized','drain') for r in groups[k])
    actual=[tuple(r[2:7])+tuple(r[10:12])+(r[1],) for r in groups['belief'] if r[1] in ('insert','replace')]
    expected=[]
    for r in dicts(ORACLE/'causal_dispatch.csv'):
        if int(r['insertion_ns'])<=cutoff:
            expected.append(tuple(r[k] for k in ('source','session','from_index','to_index','revision','arrival_ns','insertion_ns','outcome')))
    assert actual==expected,'ordered admitted versions differ from full-history oracle'
    raw={};nI=0
    for r in rows(journal):
        if r[0]=='I':nI+=1
        elif r[0]=='B':
            key=(r[2],r[3],r[6],r[7],r[5]);assert key not in raw;raw[key]=r
    maxcov=0;maxmean=0
    for r in groups['belief']:
        b=raw.pop(tuple(r[2:7]));assert r[7]==b[4] and r[8:11]==b[8:10]+[b[1]]
        maxcov=max(maxcov,float(np.max(np.abs(np.array(r[19:],float)-np.array(b[16:],float)))))
        maxmean=max(maxmean,float(np.max(np.abs(np.array(r[13:19],float)-np.array(b[10:16],float)))))
    assert not raw and maxcov==0 and maxmean<1e-12
    reference=dicts(ORACLE/'online_states.csv');d=[]
    for r in groups['online']:
        i=int(r[1]);v=np.array(r[4:],float);ref=np.array([reference[i][k] for k in ('px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz')],float)
        assert int(r[2])==int(reference[i]['timestamp_ns'])
        q,q0=v[3:7],ref[3:7];q/=np.linalg.norm(q);q0/=np.linalg.norm(q0)
        # Quaternion difference with atan2 avoids cancellation near identical rotations.
        w=q[3]*q0[3]+q[:3]@q0[:3]
        xyz=q0[3]*q[:3]-q[3]*q0[:3]-np.cross(q0[:3],q[:3])
        angle=2*math.atan2(np.linalg.norm(xyz),abs(w))
        d.append([i,np.linalg.norm(v[:3]-ref[:3]),angle,np.linalg.norm(v[7:10]-ref[7:10]),np.linalg.norm(v[10:13]-ref[10:13])])
    a=np.array(d);pre=a[a[:,0]<=70,1:];post=a[a[:,0]>70,1:]
    def summary(x):return dict(max=np.max(x,axis=0).tolist(),rms=np.sqrt(np.mean(x*x,axis=0)).tolist())
    warnings=re.findall(r'VectorValues Update: key ([xvw])(\d+) does not exist!',(ROOT/(name+'.log')).read_text())
    numerical_prepass=True # Exact match to the refined full-history policy is not this candidate's contract.
    result=dict(status='PREFIX_STOCK_ACCOUNTING_PASS',
      received=sum(counts.values()),raw_imu=nI,outcomes=dict(counts),active_ids=len(active),absorbed_ids=len(dead),peak_active_ids=maxactive,
      publications=len(clock),retired_supports=int(groups['publication'][-1][5]),retained_supports=len(clock)-int(groups['publication'][-1][5]),
      stationarity='not_checked',max_retirement_updates=max(int(r[4]) for r in groups['publication']),
      covariance_payload_max_error=maxcov,log_mean_roundtrip_max_error=maxmean,
      state_difference_columns=['position_m','orientation_rad','velocity_mps','omega_radps'],
      before_retirement=summary(pre),after_retirement=summary(post),
      runtime_warning_count=len(warnings),warning_keys=[[k,int(i)] for k,i in warnings],
      warning_requires_explanation=bool(warnings),protected_unchanged=protected()==json.loads((ROOT/'protected.json').read_text()),
      full_gate_accepted=False)
    if full:
        assert counts==dict(before_initialization=25,insert=2201,replace=86053)
        assert len(dead)==2101 and len(active)==100 and len(clock)==1481
        result['status']='FULL_STOCK_ACCOUNTING_PASS'
    def state_errors(source_rows,reference_rows):
        errors=[]
        for r,ref in zip(source_rows,reference_rows):
            v=np.array(r[4:],float);u=np.array([ref[k] for k in ('px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz')],float)
            assert int(r[2])==int(ref['timestamp_ns'])
            q=v[3:7]/np.linalg.norm(v[3:7]);q0=u[3:7]/np.linalg.norm(u[3:7])
            angle=2*math.atan2(np.linalg.norm(q0[3]*q[:3]-q[3]*q0[:3]-np.cross(q0[:3],q[:3])),abs(q@q0))
            errors.append([np.linalg.norm(v[:3]-u[:3]),angle,np.linalg.norm(v[7:10]-u[7:10]),np.linalg.norm(v[10:13]-u[10:13])])
        return summary(np.array(errors))
    if full:
        result['fixed_lag_finalized_vs_full_history_final']=state_errors(groups['finalized'],dicts(ORACLE/'states.csv'))
    result['paths_m']={}
    for kind in ('online','finalized'):
        xyz=np.array([r[4:7] for r in groups[kind]],float)
        result['paths_m'][kind]=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
        with (ROOT/(product+'_'+kind+'_incremental.tum')).open('x') as f:
            for r in groups[kind]:f.write(str(int(r[2])//1000000000)+'.'+str(int(r[2])%1000000000).zfill(9)+' '+' '.join(r[4:11])+'\n')
    result['passed']=True
    if not full:
        other=ROOT/'prefix_repeat_outputs'
        def inventory(folder):return {str(p.relative_to(folder)):sha(p) for p in folder.rglob('*') if p.is_file()}
        assert json.loads((ROOT/'prefix_repeat.json').read_text())['returncode']==0
        assert inventory(out)==inventory(other),'repeat differs'
        result['repeat_archive_byte_identical']=True
    with (ROOT/(product+'_validation.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='warning_keys'},indent=2))
    with (ROOT/(product+'_state_differences.csv')).open('x') as f:
        w=csv.writer(f);w.writerow(['index']+result['state_difference_columns']);w.writerows(d)
    return result
if __name__=='__main__':main(sys.argv[1] if len(sys.argv)>1 else 'prefix')

