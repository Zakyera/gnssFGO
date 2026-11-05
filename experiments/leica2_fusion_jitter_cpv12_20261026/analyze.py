"""Read-only CP-V4 data analysis; no optimizer, replay or runtime mutation."""
import csv
import hashlib
import importlib.util
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
E = H.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
V11 = E/'leica2_velodyne_rerun_cpv11_20261026'
HASHES = {}

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''): h.update(b)
    return h.hexdigest()

def pin(p):
    p = Path(p); HASHES[str(p)] = sha(p); return p

def stats(x):
    x=np.asarray(x,float)
    return dict(n=len(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),
                maximum=float(x.max()),rms=float(np.sqrt(np.mean(x*x)))) if len(x) else dict(n=0)

def norm(x): return np.linalg.norm(x,axis=-1)

def states(p):
    with pin(p).open() as f: rows=list(csv.DictReader(f))
    idx=np.array([int(r['state_index']) for r in rows],np.int64)
    t=np.array([int(r['timestamp_ns']) for r in rows],np.int64)
    av=np.array([int(r['availability_ns']) for r in rows],np.int64)
    xyz=np.array([[float(r[k]) for k in ('px','py','pz')] for r in rows])
    q=np.array([[float(r[k]) for k in ('qx','qy','qz','qw')] for r in rows])
    vel=np.array([[float(r[k]) for k in ('vx','vy','vz')] for r in rows])
    omega=np.array([[float(r[k]) for k in ('wx','wy','wz')] for r in rows])
    assert np.all(np.diff(idx)==1) and np.all(np.diff(t)==100000000)
    assert np.max(np.abs(norm(q)-1))<1e-10
    return dict(idx=idx,t=t,av=av,p=xyz,r=Rotation.from_quat(q).as_matrix(),v=vel,w=omega)

def skew(w):
    x,y,z=w; return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def predict(s):
    pp=[]; rr=[]; maxerr=0.
    for i in range(len(s['t'])-1):
        phi=.1*s['w'][i]; rho=.1*s['r'][i].T@s['v'][i]
        th=norm(phi); W=skew(phi)
        A=(1-np.cos(th))/th**2 if th>1e-4 else .5-th**2/24+th**4/720
        B=(th-np.sin(th))/th**3 if th>1e-4 else 1/6-th**2/120+th**4/5040
        delta=(np.eye(3)+A*W+B*W@W)@rho
        r=s['r'][i]@Rotation.from_rotvec(phi).as_matrix()
        p=s['p'][i]+s['r'][i]@delta
        if i%37==0:
            hat=np.zeros((4,4));hat[:3,:3]=W;hat[:3,3]=rho
            T=np.eye(4);T[:3,:3]=s['r'][i];T[:3,3]=s['p'][i]
            independent=T@expm(hat)
            maxerr=max(maxerr,float(np.max(np.abs(p-independent[:3,3]))),
                       float(np.max(np.abs(r-independent[:3,:3]))))
        pp.append(p);rr.append(r)
    assert maxerr<1e-10
    return np.asarray(pp),np.asarray(rr),maxerr

def path_stats(p,gp,edge_mask):
    d=np.diff(p,axis=0); gd=np.diff(gp,axis=0)
    pairs=edge_mask[1:]&edge_mask[:-1]
    rev=(np.sum(d[1:]*d[:-1],axis=1)<0)&(norm(d[1:])>1e-6)&(norm(d[:-1])>1e-6)
    return dict(path_m=float(norm(d)[edge_mask].sum()),reference_path_m=float(norm(gd)[edge_mask].sum()),
                steps=stats(norm(d)[edge_mask]),step_vector_error_m=stats(norm(d-gd)[edge_mask]),
                successive_displacement_reversals=int((rev & pairs).sum()),successive_pairs=int(pairs.sum()),
                apparent_acceleration_mps2=stats(norm(np.diff(d,axis=0))[pairs]/.01))

def progress(p,s):
    event=[]; kind=[]; arrival=[]; service=[]; pubs=[]; newest=[]; oldest=[]
    with pin(p).open() as f:
        for row in csv.DictReader(f):
            event.append(int(row['event']));kind.append(row['kind']);arrival.append(int(row['arrival_ns']))
            service.append(float(row['service_seconds']));pubs.append(int(row['publications']))
            newest.append(int(row['newest']));oldest.append(int(row['oldest']))
    event=np.array(event);arrival=np.array(arrival,np.int64);pubs=np.array(pubs)
    changes=np.flatnonzero(np.diff(pubs,prepend=0)>0)
    assert np.array_equal(event,np.arange(1,len(event)+1))
    assert len(changes)==len(s['t']) and np.array_equal(arrival[changes],s['av'])
    assert np.array_equal(np.array(newest)[changes],s['idx'])
    assert np.all(np.diff(pubs[changes],prepend=0)==1)
    return dict(event=event,kind=np.array(kind),arrival=arrival,service=np.array(service),
                pub_event=event[changes],pub_service=np.array(service)[changes],
                oldest=np.array(oldest)[changes])

def event_analysis(mode,s,pr,epoch,window):
    root=P/f'full_{mode}_receiver'
    n=len(s['t']); source_names=('glim_velodyne','kimera_front')
    arrivals={src:np.zeros(n,int) for src in source_names}
    firsts={src:np.zeros(n,int) for src in source_names}
    latest={src:np.full(n,np.iinfo(np.int64).min,np.int64) for src in source_names}
    latest_seen={src:np.iinfo(np.int64).min for src in source_names}
    selected=np.flatnonzero(window)+1
    lower=int(pr['pub_event'][selected[0]-1]); upper=int(pr['pub_event'][selected[-1]])
    applied={src:np.zeros(n,int) for src in source_names}
    new={src:np.zeros(n,int) for src in source_names}
    noop=Counter(); cov=defaultdict(list); age=defaultdict(list)
    checks=Counter(); records=[]; blocks={}; count=0
    with pin(root/'events.csv').open() as f:
        for eventno,row in enumerate(csv.reader(f),1):
            count=eventno
            assert row[0]==pr['kind'][eventno-1] and int(row[1])==pr['arrival'][eventno-1]
            j=int(np.searchsorted(pr['pub_event'],eventno,side='left'))
            if row[0]=='B' and j<n:
                src=row[2];assert src in arrivals
                arrivals[src][j]+=1;firsts[src][j]+=int(int(row[5])==0)
                latest_seen[src]=max(latest_seen[src],int(row[9]))
            if j<n and eventno==pr['pub_event'][j]:
                for src in source_names: latest[src][j]=latest_seen[src]
            if lower<eventno<=upper:
                record=root/'receiver/archive/commits'/str(eventno)/'records.txt'
                if not record.exists(): continue
                with pin(record).open() as rf:
                    lines=list(csv.reader(rf))
                blocks[eventno]=lines
                for arow in lines:
                    checks[arow[0]]+=1
                    if arow[0]=='belief':
                        action,src=arow[1:3]
                        if action not in ('insert','replace'): noop[action]+=1;continue
                        applied[src][j]+=1;new[src][j]+=int(action=='insert')
                        C=np.array(arow[19:],float).reshape(6,6)
                        assert np.max(np.abs(C-C.T))<1e-12
                        cov[src].append([np.trace(C[:3,:3]),np.trace(C[3:,3:])])
                        age[src].append((int(arow[11])-int(arow[9]))*1e-9)
                        records.append(dict(event=eventno,published_index=int(s['idx'][j]),source=src,
                                            action=action,revision=int(arow[6]),from_id=int(arow[4]),
                                            to_id=int(arow[5]),end_ns=int(arow[9]),applied_ns=int(arow[11])))
                    if arow[0]=='online_incremental':
                        assert int(arow[1])==int(s['idx'][j]) and int(arow[2])==int(s['t'][j])
                        assert np.max(np.abs(np.array(arow[4:7],float)-s['p'][j]))<1e-14
    assert count==len(pr['event']) and checks['online_incremental']==len(selected)
    summary=dict(selected_event_range=[lower+1,upper],commit_kinds=dict(checks),noops=dict(noop),sources={})
    for src in source_names:
        C=np.asarray(cov[src])
        summary['sources'][src]=dict(arrivals=int(arrivals[src][selected].sum()),
            first_arrivals=int(firsts[src][selected].sum()),applied=int(applied[src][selected].sum()),
            inserted=int(new[src][selected].sum()),replaced=int((applied[src]-new[src])[selected].sum()),
            source_end_age_at_apply_s=stats(age[src]),rotation_cov_trace=stats(C[:,0]),translation_cov_trace=stats(C[:,1]))
    return dict(arrivals=arrivals,firsts=firsts,latest=latest,applied=applied,new=new,
                summary=summary,selected_records=records)

def main():
    assert not (H/'results.json').exists() and not (H/'timeseries.npz').exists()
    for p in (Path(__file__),H/'PROTOCOL.txt',P/'PROTOCOL.txt',
              E/'receiver_finalization_cpk19_20261021/Receiver.h',
              E/'receiver_finalization_cpk19_20261021/Live.cpp',
              E/'receiver_endpoint_reuse_e2b_20261013/SourceWnoaModel.h'):
        pin(p)
    spec=importlib.util.spec_from_file_location('v10',E/'first_published_comparison_cpv10_20261026/analyze.py')
    c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);a=c.a
    for module in (c,c.b,a,a.g): pin(module.__file__)
    meta=json.loads(pin(V11/'data_checks.json').read_text())
    cached=np.load(pin(V11/'trajectories.npz'))
    epoch=meta['epoch_ns'];origin=np.array(meta['origin_reference_enu_m'])
    refpath=c.b.local(meta['reference']['path']);assert sha(pin(refpath))==meta['reference']['sha256']
    ref=a.g.Reference(refpath)
    refcurve=(np.asarray(ref.times,np.int64),np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3])
    output=dict(task='CP-V12',scope='saved-data baseline OFF/ON; no replay or settings change',
                selected_window_s=[380,390],selection=meta['selected_inspection_window'],modes={})
    arrays={};figdata={}
    for mode in ('off','on'):
        root=P/f'full_{mode}_receiver';s=states(root/'receiver/online_states.csv');f=states(root/'receiver/finalized_states.csv')
        pin(root/'resolved_config.json');pin(root/'receiver/summary.json')
        # Match by identity, not floating timestamps or assumed row count.
        m=np.searchsorted(f['idx'],s['idx']);assert np.array_equal(f['idx'][m],s['idx'])
        assert np.array_equal(f['t'][m],s['t'])
        f={k:v[m] for k,v in f.items()}
        pp,rr,predcheck=predict(s)
        op,_=a.prism(s['p'],s['r']);fp,_=a.prism(f['p'],f['r']);pred,_=a.prism(pp,rr)
        gp,gr,ok=a.sample(refcurve,s['t'],50_000_000);assert ok.all();gp,_=a.prism(gp,gr)
        record=meta['products'][mode+'_fusion_online'];rot=np.array(record['alignment_rotation']);tr=np.array(record['alignment_translation_m'])
        op=op@rot.T+tr-origin;fp=fp@rot.T+tr-origin;pred=pred@rot.T+tr-origin;gp-=origin
        assert np.max(np.abs(op-cached[mode+'_fusion_online_p']))<1e-10
        assert np.array_equal(s['t']-epoch,cached[mode+'_fusion_online_t_ns'])
        assert np.max(np.abs(norm(np.diff(gp,axis=0))-cached[mode+'_reference_step_m']))<1e-10
        t=(s['t']-epoch)*1e-9;win=(t[1:]>380)&(t[1:]<=390);assert win.sum()==100
        correction=fp-op;dv=np.diff(op,axis=0)-np.diff(fp,axis=0)
        identity=float(np.max(np.abs(dv+np.diff(correction,axis=0))));assert identity<1e-12
        innovation=op[1:]-pred;predstep=pred-op[:-1]
        pr=progress(root/'receiver/stream_progress.csv',s)
        ea=event_analysis(mode,s,pr,epoch,win)
        modeout=dict(rows=len(t),span_s=float(t[-1]-t[0]),start_s=float(t[0]),end_s=float(t[-1]),
                     prediction_matrix_check_max=predcheck,correction_vector_identity_max=identity,
                     events=ea['summary'],whole={},selected={},categories={})
        for label,mask in [('whole',np.ones(len(t)-1,bool)),('selected',win)]:
            vertices=np.flatnonzero(np.r_[mask,False]|np.r_[False,mask])
            oo=path_stats(op,gp,mask);ff=path_stats(fp,gp,mask);gg=path_stats(gp,gp,mask)
            modeout[label]=dict(online=oo,finalized=ff,reference=gg,
                path_excess_removed_percent=100*(oo['path_m']-ff['path_m'])/(oo['path_m']-gg['path_m']),
                same_state_finalization_shift_m=stats(norm(correction)[vertices]),
                finalization_delay_s=stats((f['av'][vertices]-f['t'][vertices])*1e-9),
                saved_prediction_departure_m=stats(norm(innovation)[mask]),
                predicted_step_m=stats(norm(predstep)[mask]),
                change_in_same_state_correction_per_step_m=stats(norm(np.diff(correction,axis=0))[mask]),
                online_RMSE_using_fixed_CPV11_fit_m=float(np.sqrt(np.mean(norm(op[vertices]-gp[vertices])**2))),
                finalized_RMSE_using_same_fit_m=float(np.sqrt(np.mean(norm(fp[vertices]-gp[vertices])**2))))
        G=ea['applied']['glim_velodyne'][1:]>0;K=ea['applied']['kimera_front'][1:]>0
        for name,mask in [('GLIM only',G&~K),('Kimera only',K&~G),('both',G&K),('neither',~G&~K)]:
            mask &= win
            modeout['categories'][name]=dict(cycles=int(mask.sum()),
                departure_m=stats(norm(innovation)[mask]),step_m=stats(norm(np.diff(op,axis=0))[mask]))
        ranked=np.flatnonzero(win)[np.argsort(norm(innovation)[win])[::-1][:10]]
        modeout['largest_selected_departures']=[]
        for i in ranked:
            modeout['largest_selected_departures'].append(dict(time_s=float(t[i+1]),state_index=int(s['idx'][i+1]),
                departure_m=float(norm(innovation[i])),online_step_m=float(norm(op[i+1]-op[i])),
                finalized_step_m=float(norm(fp[i+1]-fp[i])),reference_step_m=float(norm(gp[i+1]-gp[i])),
                glim_applied=int(ea['applied']['glim_velodyne'][i+1]),kimera_applied=int(ea['applied']['kimera_front'][i+1]),
                glim_inserted=int(ea['new']['glim_velodyne'][i+1]),kimera_inserted=int(ea['new']['kimera_front'][i+1])))
        for name,x in dict(time_s=t,online=op,finalized=fp,reference=gp,predicted=pred,
                           prediction_departure=innovation,correction=correction,window_edges=win,
                           online_velocity=s['v'],online_omega=s['w']).items(): arrays[mode+'_'+name]=x
        for src in ('glim_velodyne','kimera_front'):
            for group in ('arrivals','firsts','applied','new','latest'):
                arrays[f'{mode}_{group}_{src}']=ea[group][src]
        output['modes'][mode]=modeout;figdata[mode]=(t,op,fp,gp,innovation,ea)
        (H/f'{mode}_selected_applied.json').write_text(json.dumps(ea['selected_records'],indent=2)+'\n')
        print(mode,json.dumps(modeout,indent=2),flush=True)
    # A fixed-frame closeup: no local refit of the finalized curve.
    fig,axs=plt.subplots(3,2,figsize=(14,10),constrained_layout=True)
    for col,mode in enumerate(('off','on')):
        t,op,fp,gp,innov,ea=figdata[mode];sel=(t>=379.98)&(t<=390)
        for p,color,label in ((op,'#dc3545','ONLINE first publication'),(fp,'#00876c','DELAYED finalized'),(gp,'#377bd1','IE-TC reference')):
            axs[0,col].plot(t[sel],p[sel,2],color=color,lw=1.3,label=label)
            axs[1,col].plot(t[1:][sel[1:]],norm(np.diff(p,axis=0))[sel[1:]],color=color,lw=1.2,label=label)
        axs[0,col].set(title=f'CBS {mode.upper()} | height in fixed aligned frame',ylabel='height (m)')
        axs[1,col].set(title='Distance between successive 100 ms outputs',ylabel='step (m)')
        axs[2,col].plot(t[1:],norm(innov)*100,color='#6f42c1',label='departure from saved-state prediction (cm)')
        axr=axs[2,col].twinx()
        axr.step(t,ea['applied']['glim_velodyne'],where='mid',color='#b47800',alpha=.6,label='GLIM applied factors')
        axr.step(t,ea['applied']['kimera_front'],where='mid',color='#008e9b',alpha=.6,label='Kimera applied factors')
        axs[2,col].set(ylabel='net departure (cm)',xlabel='seconds from capture start',title='Correction timing (association, not isolated causality)')
        axr.set_ylabel('insertions + replacements per cycle');axr.legend(loc='upper right',fontsize=8)
        for ax in axs[:,col]: ax.set_xlim(380,390);ax.grid(alpha=.2)
        axs[0,col].legend(fontsize=8);axs[2,col].legend(loc='upper left',fontsize=8)
    fig.suptitle('Leica-2 Velodyne: selected worst-OFF 10 s interval, not a typical-window claim\nSame saved states; no smoothing or tuning. Finalized is delayed, not one simultaneous graph snapshot.',fontsize=12)
    fig.savefig(H/'jitter_diagnosis.png',dpi=160);plt.close(fig)
    np.savez_compressed(H/'timeseries.npz',**arrays)
    unchanged=all(sha(Path(p))==h for p,h in HASHES.items());assert unchanged
    output['input_hashes_unchanged']=unchanged;output['input_sha256']=HASHES
    output['limits']=['No per-belief pre/post fused states or simultaneous window snapshot were logged.',
        'Saved prediction departure includes all intervening graph updates; not one source factor innovation.',
        'Finalized path connects different retirement times, not one simultaneous graph solution.',
        'No Qc/weight sweep, Hessian/prior conditioning or nonlinear stationarity check.',
        'IE-TC shares CPT7. Source covariance-alignment approximation remains unchanged.']
    (H/'results.json').write_text(json.dumps(output,indent=2)+'\n')

if __name__=='__main__': main()
