"""Read-only analysis of observer trace after original-output parity passes."""
import csv, hashlib, json, sys
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_paired_receiver_trace_cpv22a_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
V12=H.parent/'leica2_sensor_jitter_audit_cpv21_20261027'
V11=H.parent/'leica2_velodyne_rerun_cpv11_20261026'
EPOCH=1732549034915071665
TP=np.array([.308837802,-.038312997,-.234944116])
PINS={}

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(2**20),b''): h.update(b)
    return h.hexdigest()

def pin(p): PINS[str(p)]=sha(p);return p
def norm(a): return np.linalg.norm(a,axis=-1)
def stats(a):
    a=np.asarray(a,float)
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),
                maximum=float(a.max()),rms=float(np.sqrt(np.mean(a*a))),mean=float(a.mean())) if len(a) else dict(n=0)

def predict(row):
    rot=Rotation.from_quat(row[5:9]).as_matrix();phi=.1*row[12:15];rho=.1*rot.T@row[9:12]
    x,y,z=phi;W=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    hat=np.zeros((4,4));hat[:3,:3]=W;hat[:3,3]=rho
    T=np.eye(4);T[:3,:3]=rot;T[:3,3]=row[2:5]
    value=T@expm(hat)
    return value[:3,3]+value[:3,:3]@TP

def mode_analysis(mode):
    assert json.loads(pin(H/f'{mode}_parity.json').read_text())['pass_']
    cache=np.load(pin(V12/'timeseries.npz'))
    view=H.parent/('leica2_hesai_rerun_cpv20_20261027' if mode=='hesai' else 'leica2_velodyne_rerun_cpv11_20261026')
    meta_fit=json.loads(pin(view/'data_checks.json').read_text())
    fit=meta_fit['products']['off_fusion_online']
    Q=np.array(fit['alignment_rotation']);T=np.array(fit['alignment_translation_m'])-np.array(meta_fit['origin_reference_enu_m'])
    with open(pin(R/f'{mode}_trace'/'trace_meta.csv')) as f: meta=list(csv.DictReader(f))
    data=np.loadtxt(pin(R/f'{mode}_trace'/'trace_states.csv'),delimiter=',',skiprows=1)
    assert data.shape[1]==15 and np.isfinite(data).all()
    xyz=(data[:,2:5]+Rotation.from_quat(data[:,5:9]).apply(TP))@Q.T+T
    cuts=np.r_[0,np.flatnonzero(np.diff(data[:,0]))+1,len(data)]
    assert len(cuts)==len(meta)+1
    snapshots={}
    for j,m in enumerate(meta):
        sid=int(m['snapshot']);a,b=cuts[j:j+2]
        assert np.all(data[a:b,0]==sid)
        ids=data[a:b,1].astype(int)
        assert np.array_equal(ids,np.arange(ids[0],ids[-1]+1))
        snapshots[sid]=(ids,data[a:b],xyz[a:b])
    def snap(m): return snapshots[int(m['snapshot'])]
    ops=[];pubs=[]
    for j,m in enumerate(meta):
        stage=m['stage']
        if stage=='publication':
            ids,rows,pts=snap(m);pubs.append((j,m,ids,rows,pts));continue
        if not stage.endswith('_before'): continue
        assert j+1<len(meta)
        aft=meta[j+1]
        assert aft['stage']==stage.replace('_before','_after') and aft['event']==m['event']
        ids,rows,pts=snap(m);ids2,rows2,pts2=snap(aft)
        assert ids[-1]==int(m['newest']) and ids2[-1]==int(aft['newest'])
        group=m['source']+'_'+m['action'] if stage=='source_before' else stage.split('_')[0]
        op=dict(j=j,after=j+1,event=int(m['event']),group=group,kind=stage.split('_')[0],
                source=m['source'],action=m['action'],from_=int(m['from']),to=int(m['to']),revision=int(m['revision']),
                newest=int(m['newest']),arrival_ns=int(m['arrival_ns']),
                delta=pts2[-1]-pts[-1],newest_imu_delta=rows2[-1,2:5]-rows[-1,2:5],
                newest_velocity_delta=(rows2[-1,9:12]-rows[-1,9:12])@Q.T,
                newest_omega_delta=rows2[-1,12:15]-rows[-1,12:15],
                incoming_error_before=float(m['incoming_error']),incoming_error_after=float(aft['incoming_error']),
                residual_before=np.array([float(m[k]) for k in ['erx','ery','erz','etx','ety','etz']]),
                residual_after=np.array([float(aft[k]) for k in ['erx','ery','erz','etx','ety','etz']]))
        common=np.intersect1d(ids,ids2)
        differences=pts2[common-ids2[0]]-pts[common-ids[0]]
        op['window_mean_delta']=differences.mean(axis=0)
        op['window_rms_shift']=float(np.sqrt(np.mean(norm(differences)**2)))
        op['window_shape_rms']=float(np.sqrt(np.mean(norm(differences-differences.mean(axis=0))**2)))
        op['age_shift']={}
        for age in [0,1,10,30,60,70]:
            idx=ids[-1]-age
            if idx>=max(ids[0],ids2[0]): op['age_shift'][str(age/10)]=float(norm(pts2[idx-ids2[0]]-pts[idx-ids[0]]))
        if op['kind']=='support':
            pred=predict(rows[-1])@Q.T+T
            op['prediction']=pred-pts[-1]
            op['add_adjustment']=pts2[-1]-pred
            op['previous_state_adjustment']=pts2[-2]-pts[-1]
        ops.append(op)
    groups=sorted({o['group'] for o in ops})
    result=dict(mode=mode,snapshots=len(meta),rows=len(data),operations=len(ops),stage_counts=dict(Counter(m['stage'] for m in meta)),
                observer_interval_s=[(int(meta[0]['arrival_ns'])-EPOCH)*1e-9,(int(meta[-1]['arrival_ns'])-EPOCH)*1e-9],groups={})
    for group in groups:
        oo=[o for o in ops if o['group']==group]
        result['groups'][group]=dict(count=len(oo),frontier_displacement_m=stats([norm(o['delta']) for o in oo]),
            newest_velocity_change_mps=stats([norm(o['newest_velocity_delta']) for o in oo]),
            newest_omega_change_rps=stats([norm(o['newest_omega_delta']) for o in oo]),
            window_rms_shift_m=stats([o['window_rms_shift'] for o in oo]),
            nonuniform_window_shift_rms_m=stats([o['window_shape_rms'] for o in oo]),
            age_shift_m={age:stats([o['age_shift'][age] for o in oo if age in o['age_shift']]) for age in ['0.0','0.1','1.0','3.0','6.0','7.0']})
        if oo[0]['kind']=='source':
            result['groups'][group].update(incoming_error_before=stats([o['incoming_error_before'] for o in oo]),
                incoming_error_after=stats([o['incoming_error_after'] for o in oo]),
                fraction_incoming_error_reduced=float(np.mean([o['incoming_error_after']<o['incoming_error_before'] for o in oo])),
                incoming_translation_residual_before_m=stats([norm(o['residual_before'][3:]) for o in oo]),
                incoming_translation_residual_after_m=stats([norm(o['residual_after'][3:]) for o in oo]))
    t=cache[mode+'_time_s'];online=cache[mode+'_online'];final=cache[mode+'_finalized'];reference=cache[mode+'_reference']
    cycles=[];maxidentity=0.;maxpub=0.;windows=[]
    components=groups+['prediction','support_add_adjustment']
    for pj,(j,m,ids,rows,pts) in enumerate(pubs):
        idx=int(m['newest']);maxpub=max(maxpub,float(norm(pts[-1]-online[idx])))
        if idx==0: continue
        # Contemporaneous revised 7-second window, NOT separately finalized rows.
        dp=np.diff(pts,axis=0)
        windows.append(dict(time_s=float(t[idx]),state_index=idx,rows=len(ids),
             simultaneous_window_path_m=float(norm(dp).sum()),stitched_online_window_path_m=float(norm(np.diff(online[ids],axis=0)).sum()),
             finalized_window_path_m=float(norm(np.diff(final[ids],axis=0)).sum()),reference_window_path_m=float(norm(np.diff(reference[ids],axis=0)).sum()),
             simultaneous_last_edge_m=float(norm(dp[-1])),online_last_edge_m=float(norm(online[idx]-online[idx-1])),
             reference_last_edge_m=float(norm(reference[idx]-reference[idx-1])),
             simultaneous_reversals=int((np.sum(dp[1:]*dp[:-1],axis=1)<0).sum())))
        if pj==0 or not 0<t[idx]<=60: continue
        previous_j=pubs[pj-1][0];assert int(pubs[pj-1][1]['newest'])==idx-1
        selected=[o for o in ops if previous_j<o['j']<j]
        vectors={g:np.zeros(3) for g in components};counts=Counter()
        for o in selected:
            vectors[o['group']]+=o['delta'];counts[o['group']]+=1
            if o['kind']=='support':
                vectors['prediction']+=o['prediction'];vectors['support_add_adjustment']+=o['add_adjustment']
        assert counts['support']==1 and counts['retire']==int(idx>70)
        actual=online[idx]-online[idx-1]
        check=sum((vectors[g] for g in groups),np.zeros(3))
        maxidentity=max(maxidentity,float(norm(actual-check)))
        assert norm(vectors['support']-vectors['prediction']-vectors['support_add_adjustment'])<1e-10
        cycles.append(dict(time_s=float(t[idx]),state_index=idx,counts=dict(counts),vectors=vectors,actual=actual,
                           reference=reference[idx]-reference[idx-1]))
    assert len(cycles)==595 and maxidentity<1e-9 and maxpub<1e-9
    result['checks']=dict(publication_vs_original_max_m=maxpub,cycle_vector_identity_max_m=maxidentity,cycles=len(cycles))
    result['cycle_component_m']={g:stats([norm(c['vectors'][g]) for c in cycles]) for g in components}
    result['support_add_existing_state_shift_m']=stats([norm(o['previous_state_adjustment']) for o in ops if o['kind']=='support'])
    result['window_comparison']={k:stats([w[k] for w in windows if 0<w['time_s']<=60]) for k in windows[0] if k not in ['time_s','state_index','rows']}
    # Net update to the previously published state before new support creation.
    result['cycle_source_total_m']=stats([norm(sum((c['vectors'][g] for g in groups if g not in ['support','retire']),np.zeros(3))) for c in cycles])
    top=sorted([o for o in ops if o['kind']=='source'],key=lambda o:norm(o['delta']),reverse=True)[:20]
    result['largest_source_updates']=[{k:v for k,v in o.items() if k not in ['j','after']} for o in top]
    with open(pin(R/f'{mode}_trace'/'trace_costs.csv')) as f: costs=list(csv.DictReader(f))
    result['publication_factor_costs']={category:dict(counts=stats([int(c['count']) for c in costs if c['category']==category]),
        half_whitened_squared_error=stats([float(c['error']) for c in costs if c['category']==category])) for category in sorted({c['category'] for c in costs})}
    # Fresh rows + summaries; no original inputs touched.
    def plain(o):
        if isinstance(o,np.ndarray): return o.tolist()
        if isinstance(o,np.generic): return o.item()
        raise TypeError(type(o))
    with open(H/f'{mode}_analysis.json','x') as f: json.dump(result,f,indent=2,default=plain)
    with open(H/f'{mode}_cycles.json','x') as f: json.dump(cycles,f,indent=2,default=plain)
    with open(H/f'{mode}_windows.json','x') as f: json.dump(windows,f,indent=2,default=plain)
    np.savez_compressed(H/f'{mode}_publication_states.npz',
        newest=np.concatenate([np.full(len(ids),int(m['newest']),dtype=np.int64) for j,m,ids,rows,pts in pubs]),
        index=np.concatenate([ids for j,m,ids,rows,pts in pubs]),
        points=np.concatenate([pts for j,m,ids,rows,pts in pubs]),
        raw_states=np.concatenate([rows for j,m,ids,rows,pts in pubs]))
    with open(H/f'{mode}_analysis_pins.json','x') as f: json.dump(PINS,f,indent=2)
    print(json.dumps({'mode':mode,'snapshots':result['snapshots'],'checks':result['checks'],'groups':{k:{m:v for m,v in g.items() if m in ('count','frontier_displacement_m','newest_velocity_change_mps')} for k,g in result['groups'].items()}},default=plain,indent=2),flush=True)

if __name__=='__main__':
    pin(Path(__file__))
    assert len(sys.argv)==2 and sys.argv[1] in ('hesai','velodyne')
    mode_analysis(sys.argv[1])
