"""Bounded read of CP108 actual received revisions and accepted terminal markers."""
import csv,importlib.util,json,sys
from pathlib import Path
import numpy as np
from scipy.linalg import eigvalsh
from localize import H,S,P,pin,read,save,sha,PINS,stats
sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
spec=importlib.util.spec_from_file_location('geometry',pin(H.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'))
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
roles=('glim_hesai','kimera_front');stages=('first','age2.5','age4.5','terminal')
windows=[(0,10),(10,30),(30,50),(50,70),(130,150),(165,180),(275,290),(290,310),(310,350)]
def summarize(e,C,age):
    assert np.linalg.eigvalsh(C).min()>0
    tr=np.trace(C[:,3:,3:],axis1=1,axis2=2);rt=np.trace(C[:,:3,:3],axis1=1,axis2=2)
    q=np.einsum('ni,ni->n',e,np.linalg.solve(C,e[...,None])[...,0])
    qc=np.sum(np.linalg.solve(np.linalg.cholesky(C),e[...,None])[...,0]**2,axis=1)
    assert np.max(abs(q-qc)/(1+q))<1e-8
    return dict(n=len(e),translation_residual_m=stats(np.linalg.norm(e[:,3:],axis=1)),rotation_residual_deg=stats(np.linalg.norm(e[:,:3],axis=1)*180/np.pi),translation_trace_m2=stats(tr),rotation_trace_rad2=stats(rt),full6_normalized_reference_discrepancy=stats(q),arrival_age_s=stats(age),mean_signed_tangent_residual=e.mean(axis=0).tolist())
def main():
    assert not (H/'belief_analysis.json').exists();assert g.self_test()
    epoch=read(P/'input_audit.json')['epoch_ns']
    ref=g.Reference(pin(Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag')))
    tables={};account={};telemetry={}
    for mode in ('off','on'):
        base=P/f'full_{mode}_receiver';R=base/'receiver';data={s:{r:{} for r in roles} for s in stages}
        terminal={r:{} for r in roles};counts={r:0 for r in roles};matches={r:0 for r in roles}
        for role in roles:
            for path in sorted((R/'archive/ids'/role).glob('*/*')):
                f=pin(path).read_text().split();assert len(f)==59
                revision,ordinal,start,end,arrival,slot=map(int,f[1:7]);T=np.array(f[7:23],float).reshape(4,4);C=np.array(f[23:],float).reshape(6,6)
                key=(start,end);assert key not in terminal[role]
                terminal[role][key]=dict(mu=g.log(T),C=C,arrival=arrival,end=end,revision=revision,ordinal=ordinal,T=T,id=path.name)
        with pin(base/'events.csv').open() as stream:
            for f in csv.reader(stream):
                if f[0]!='B':continue
                assert len(f)==52;role=f[2];start,end=int(f[8]),int(f[9]);arrival=int(f[1]);revision=int(f[5]);ordinal=int(f[4]);key=(start,end)
                rec=dict(mu=np.array(f[10:16],float),C=np.array(f[16:],float).reshape(6,6),arrival=arrival,end=end,revision=revision,ordinal=ordinal)
                counts[role]+=1
                if key not in data['first'][role]:data['first'][role][key]=rec
                for age in (2.5,4.5):
                    if arrival-end<=round(age*1e9):data['age'+str(age)][role][key]=rec
                term=terminal[role].get(key)
                if term and revision==term['revision']:
                    assert arrival==term['arrival'] and ordinal==term['ordinal'] and np.max(abs(rec['C']-term['C']))==0
                    assert np.max(abs(g.exp(rec['mu'])-term['T']))<1e-11
                    matches[role]+=1
        for role in roles:
            assert matches[role]==len(terminal[role]);data['terminal'][role]=terminal[role]
        tables[mode]=data;account[mode]=dict(versions=counts,terminal_payload_matches=matches,unique_edges={r:len(data['first'][r]) for r in roles})
        tel=[]
        with pin(R/'stream_progress.csv').open() as f,pin(base/'receipts.jsonl').open() as rf:
            receipts=(json.loads(x) for x in rf)
            for row,receipt in zip(csv.DictReader(f),receipts):
                assert int(row['event'])==receipt['event']+1
                service=float(row['service_seconds']);wait=(int(row['processed_wall_ns'])-receipt['wall_ns'])*1e-9-service
                assert wait>=-.005
                tel.append(((int(row['arrival_ns'])-epoch)*1e-9,service,max(0,wait),int(row['pending']),int(row['active']),int(row['publications'])))
        tel=np.array(tel);tb=[]
        for lo,hi in windows:
            a=tel[(tel[:,0]>=lo)&(tel[:,0]<hi)]
            tb.append(dict(start_s=lo,end_s=hi,n=len(a),service_seconds=stats(a[:,1]),queue_wait_seconds=stats(a[:,2]),pending_max=int(a[:,3].max()),active_min=int(a[:,4].min()),active_max=int(a[:,4].max())))
        telemetry[mode]=tb
    result={};arrays={};cohorts={}
    for role in roles:
        common=set(tables['off']['first'][role])&set(tables['on']['first'][role]);eligible={k for k in common if 3<=(k[1]-epoch)*1e-9<=348 and .18<=(k[1]-k[0])*1e-9<=.22}
        cohort=eligible.copy()
        for mode in tables:
            for stage in stages:
                table=tables[mode][stage][role];cohort &=set(table)
                if stage.startswith('age'):
                    age=float(stage[3:]);cohort={k for k in cohort if (table[k]['arrival']-k[1])*1e-9>=age-.35}
        keys=sorted(cohort);assert len(keys)>1000
        t=np.array([(k[1]-epoch)*1e-9 for k in keys]);arrays[role+'_end_s']=t;arrays[role+'_endpoints_ns']=np.array(keys,np.int64)
        target=[g.inv(ref.at(a))@ref.at(b) for a,b in keys]
        cohorts[role]=dict(common_edges=len(common),eligible=len(eligible),cohort=len(keys),omitted=len(eligible)-len(keys),first_end_s=float(t[0]),last_end_s=float(t[-1]))
        allst={};bins=[];ratio={}
        for mode in tables:
            allst[mode]={}
            for stage in stages:
                records=[tables[mode][stage][role][k] for k in keys];C=np.array([x['C'] for x in records]);age=np.array([(x['arrival']-x['end'])*1e-9 for x in records])
                err=np.array([g.log(g.inv(g.exp(x['mu']))@T) for x,T in zip(records,target)])
                allst[mode][stage]=summarize(err,C,age)
                for suffix,x in [('covariance',C),('residual',err),('arrival_age',age)]:arrays[role+'_'+mode+'_'+stage+'_'+suffix]=x
        for stage in stages:
            off=arrays[role+'_off_'+stage+'_covariance'];on=arrays[role+'_on_'+stage+'_covariance']
            ratio[stage]=dict(translation_trace_on_over_off=stats(np.trace(on[:,3:,3:],axis1=1,axis2=2)/np.trace(off[:,3:,3:],axis1=1,axis2=2)),generalized_covariance_eigenvalues=stats(np.array([eigvalsh(b,a) for a,b in zip(off,on)]).ravel()))
        for lo,hi in windows:
            ix=(t>=lo)&(t<hi)
            if not ix.any():continue
            row=dict(start_s=lo,end_s=hi,n=int(ix.sum()))
            for mode in tables:
                row[mode]={stage:summarize(arrays[role+'_'+mode+'_'+stage+'_residual'][ix],arrays[role+'_'+mode+'_'+stage+'_covariance'][ix],arrays[role+'_'+mode+'_'+stage+'_arrival_age'][ix]) for stage in ('first','terminal')}
                first=arrays[role+'_'+mode+'_age2.5_covariance'][ix];last=arrays[role+'_'+mode+'_terminal_covariance'][ix]
                row[mode]['late_trace_ratio']=stats(np.trace(last[:,3:,3:],axis1=1,axis2=2)/np.trace(first[:,3:,3:],axis1=1,axis2=2))
            bins.append(row)
        result[role]=dict(stages=allst,paired_ratios=ratio,windows=bins)
    for path,digest in PINS.items():assert sha(path)==digest,path
    np.savez_compressed(H/'belief_cohorts.npz',**arrays)
    save(H/'belief_analysis.json',dict(passed=True,cohorts=cohorts,accounting=account,sources=result,telemetry=telemetry,reference_max_bracket_ns=ref.max_bracket_ns,input_sha256=PINS,caveat='Right-local CPT7 reference residuals, not NEES/NIS. Paired physical endpoints, identical cohort all stages, age cutoffswithin0.35s. Terminal includes absorbed/accepted final marker; not expired last arriving data.'))
    print(json.dumps(dict(cohorts=cohorts,accounting=account,paired={r:result[r]['paired_ratios'] for r in roles}),indent=2))
if __name__=='__main__':main()
