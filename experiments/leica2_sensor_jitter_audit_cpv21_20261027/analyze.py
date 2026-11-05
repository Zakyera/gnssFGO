"""CP-V21 saved-data comparison. Never imports a runner or invokes a solver."""
import csv
import hashlib
import importlib.util
import json
import re
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
E = H.parent
RUNS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
CASES = {
    'hesai': (RUNS/'gnssfgo_leica2_full_cpk29_20261023/prepared', E/'leica2_hesai_rerun_cpv20_20261027'),
    'velodyne': (RUNS/'gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared', E/'leica2_velodyne_rerun_cpv11_20261026'),
}
WINDOWS = {'full': (0,429), 'first60': (0,60), 'first10': (0,10),
           '10to30': (10,30), '30to60': (30,60), 'late380to390': (380,390)}
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
j=module('oldj',E/'leica2_fusion_jitter_cpv12_20261026/analyze.py')
c=module('oldc',E/'first_published_comparison_cpv10_20261026/analyze.py')
a=c.a
pin=j.pin
def save(name,x):
    with (H/name).open('x') as f: json.dump(x,f,indent=2,allow_nan=False)
def stat(x):
    x=np.asarray(x,float);return j.stats(x[np.isfinite(x)])
def json_load(p): return json.loads(pin(p).read_text())
def jsonc(p):
    text=pin(p).read_text();text=re.sub(r'/\*.*?\*/','',text,flags=re.S)
    return json.loads(re.sub(r'//[^\n]*','',text))
def differences(x,y,p=''):
    if isinstance(x,dict) and isinstance(y,dict):
        return sum([differences(x.get(k),y.get(k),p+'/'+k) for k in sorted(set(x)|set(y))],[])
    return [] if x==y else [{'key':p,'hesai':x,'velodyne':y}]

def events(root,s,pr,epoch,refcurve):
    n=len(s['t']);kinds=('glim','kimera');front=np.full((n,2),np.nan)
    count=np.zeros((n,2),int);first=np.zeros((n,2),int)
    current=[None,None];new=[[],[]];kimhash=hashlib.sha256();kimarrival=[];imu=[]
    cycle=0
    with pin(root/'events.csv').open() as f:
        for event,row in enumerate(csv.reader(f),1):
            assert row[0]==pr['kind'][event-1] and int(row[1])==pr['arrival'][event-1]
            if row[0]=='I': imu.append(row[2:])
            if row[0]=='B':
                k=0 if row[2].startswith('glim_') else 1
                assert k==0 or row[2]=='kimera_front'
                if cycle<n:
                    count[cycle,k]+=1;first[cycle,k]+=int(int(row[5])==0)
                end=int(row[9]);current[k]=end if current[k] is None else max(current[k],end)
                if k==1:
                    kimhash.update((','.join(row[5:])+'\n').encode());kimarrival.append(int(row[1]))
                if int(row[5])==0:
                    new[k].append([int(row[1]),int(row[8]),end,*map(float,row[10:])])
            if cycle<n and event==pr['pub_event'][cycle]:
                for k in range(2):
                    if current[k] is not None: front[cycle,k]=(int(s['t'][cycle])-current[k])*1e-9
                cycle+=1
    assert event==len(pr['event']) and cycle==n
    first_stats={}
    for k,kind in enumerate(kinds):
        rows=new[k]
        arrival=np.asarray([r[0] for r in rows],np.int64)
        ti=np.asarray([r[1] for r in rows],np.int64);tj=np.asarray([r[2] for r in rows],np.int64)
        mu=np.asarray([r[3:9] for r in rows]);cov=np.asarray([r[9:] for r in rows]).reshape(-1,6,6)
        pi,ri,oi=a.sample(refcurve,ti,50_000_000);pj,rj,oj=a.sample(refcurve,tj,50_000_000)
        ok=oi&oj
        dt=np.einsum('nji,nj->ni',ri,pj-pi);dr=ri.transpose(0,2,1)@rj
        z=np.asarray([a.g.exp(x) for x in mu])
        te=np.linalg.norm(z[:,:3,3]-dt,axis=1)
        re=np.linalg.norm(Rotation.from_matrix(dr.transpose(0,2,1)@z[:,:3,:3]).as_rotvec(),axis=1)*180/np.pi
        end=(tj-epoch)*1e-9
        out={}
        for label,(lo,hi) in WINDOWS.items():
            sel=(end>lo)&(end<=hi)&ok
            out[label]={'rows':int(sel.sum()), 'source_end_age_at_receipt_s':stat((arrival[sel]-tj[sel])*1e-9),
                'horizon_s':stat((tj[sel]-ti[sel])*1e-9),
                'support_phase_s':stat(((tj[sel]-s['t'][0])%100_000_000)*1e-9),
                'relative_translation_error_m':stat(te[sel]),'relative_rotation_error_deg':stat(re[sel]),
                'translation_cov_trace':stat(np.trace(cov[sel,3:,3:],axis1=1,axis2=2)),
                'rotation_cov_trace':stat(np.trace(cov[sel,:3,:3],axis1=1,axis2=2))}
        first_stats[kind]=out
    return front,count,first,first_stats,kimhash.hexdigest(),np.asarray(kimarrival,np.int64),imu

def main():
    assert not (H/'results.json').exists()
    for p in (Path(__file__),H/'PROTOCOL.txt',Path(j.__file__),Path(c.__file__),Path(a.__file__),Path(a.g.__file__)):pin(p)
    result={'task':'CP-V21','windows':WINDOWS,'cases':{}}
    caches={};configs={};effective={};manifests={};profiles={};kimcurves={};eventcomp={}
    data={}
    for name,(P,V) in CASES.items():
        src=P/'full_off_source';root=P/'full_off_receiver'
        meta=json_load(V/'data_checks.json');cache=np.load(pin(V/'trajectories.npz'));epoch=meta['epoch_ns']
        refpath=c.b.local(meta['reference']['path']);pin(refpath)
        assert j.sha(refpath)==meta['reference']['sha256']
        ref=a.g.Reference(refpath)
        refcurve=(np.asarray(ref.times,np.int64),np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3])
        s=j.states(root/'receiver/online_states.csv');f=j.states(root/'receiver/finalized_states.csv')
        assert np.array_equal(s['idx'],f['idx']) and np.array_equal(s['t'],f['t'])
        gp,gr,ok=a.sample(refcurve,s['t'],50_000_000);assert ok.all();gp,_=a.prism(gp,gr)
        op,_=a.prism(s['p'],s['r']);fp,_=a.prism(f['p'],f['r'])
        fit=meta['products']['off_fusion_online'];rot=np.asarray(fit['alignment_rotation']);tr=np.asarray(fit['alignment_translation_m']);origin=np.asarray(meta['origin_reference_enu_m'])
        op=op@rot.T+tr-origin;fp=fp@rot.T+tr-origin;gp-=origin
        assert np.max(np.abs(op-cache['off_fusion_online_p']))<1e-10
        pr=j.progress(root/'receiver/stream_progress.csv',s)
        front,count,first,fs,kh,ka,imu=events(root,s,pr,epoch,refcurve)
        t=(s['t']-epoch)*1e-9
        out={'epoch_ns':epoch,'rows':len(t),'windows':{},'first_beliefs':fs}
        correction=op-fp
        for label,(lo,hi) in WINDOWS.items():
            sel=(t[1:]>lo)&(t[1:]<=hi);vertices=(t>lo)&(t<=hi)
            out['windows'][label]={'online':j.path_stats(op,gp,sel),'finalized':j.path_stats(fp,gp,sel),
                'source_frontier_age_s':{k:stat(front[vertices,i]) for i,k in enumerate(('glim','kimera'))},
                'publication_service_wall_s':stat(pr['pub_service'][vertices]),
                'received_first_versions':{k:int(first[vertices,i].sum()) for i,k in enumerate(('glim','kimera'))},
                'received_all_versions':{k:int(count[vertices,i].sum()) for i,k in enumerate(('glim','kimera'))}}
            hask=count[:,1]>0;hasg=count[:,0]>0;both=vertices&hask&hasg;go=vertices&~hask&hasg
            if both.sum()>1 and go.sum()>1:
                delta=correction[both].mean(0)-correction[go].mean(0)
                out['windows'][label]['phase_mean_correction_separation_m']=float(np.linalg.norm(delta))
                out['windows'][label]['phase_counts']={'glim_only':int(go.sum()),'both':int(both.sum())}
        configs[name]=json_load(root/'resolved_config.json');effective[name]=json_load(src/'effective_mode_check.json');manifests[name]=json_load(root/'manifest.json')
        out['transport']=json_load(root/'transport_status.json')
        profiles[name]={}
        for group in ('glim_config','kimera_profile'):
            profiles[name][group]={str(p.relative_to(src/group)):j.sha(pin(p)) for p in (src/group).rglob('*') if p.is_file()}
        kimcurves[name]=a.load_curve(pin(src/'kimera_odometry.csv'))
        for fn in ('receiver_cpk19.sh','protected_manifest.json'):pin(P/fn)
        for fn in ('live_receiver_protocol.py','grandtour_live_receiver_ros1.py'):pin(root/'transport_code'/fn)
        result['cases'][name]=out;eventcomp[name]=(kh,ka,imu)
        data[name]=(t,op,fp,gp,front,count)
        caches[name]=(s,meta)
        print(name,'computed',flush=True)
    h,v=caches['hesai'][0],caches['velodyne'][0]
    assert np.array_equal(h['idx'],v['idx']) and np.array_equal(h['t'],v['t'])
    result['identical_receiver_state_indices_and_physical_times']=True
    result['receiver_publication_availability_difference_s']=stat(np.abs(h['av']-v['av'])*1e-9)
    result['receiver_publication_availability_different_rows']=int(np.count_nonzero(h['av']!=v['av']))
    result['receiver_config_differences']=differences(configs['hesai'],configs['velodyne'])
    result['same_receiver_binary']=manifests['hesai']['binary_sha256']==manifests['velodyne']['binary_sha256']
    result['profile_different_files']={}
    result['glim_parameter_differences']={}
    for group in profiles['hesai']:
        ph,pv=profiles['hesai'][group],profiles['velodyne'][group]
        changed=[k for k in sorted(set(ph)|set(pv)) if ph.get(k)!=pv.get(k)]
        result['profile_different_files'][group]=changed
        if group=='glim_config':
            for k in changed:result['glim_parameter_differences'][k]=differences(*[jsonc(CASES[n][0]/'full_off_source'/group/k) for n in CASES])
    result['effective_differences']={k:differences(effective['hesai'][k],effective['velodyne'][k]) for k in ('glim','kimera','kimera_candidate_env','glim_revision_covariance','playback_rate','requested_s','cached_provider_sha256')}
    result['kimera_native_same_timestamps']=bool(np.array_equal(kimcurves['hesai'][0],kimcurves['velodyne'][0]))
    result['kimera_native_position_max_difference_m']=float(np.max(np.abs(kimcurves['hesai'][1]-kimcurves['velodyne'][1])))
    result['kimera_native_rotation_max_matrix_difference']=float(np.max(np.abs(kimcurves['hesai'][2]-kimcurves['velodyne'][2])))
    result['kimera_all_belief_payloads_identical']=eventcomp['hesai'][0]==eventcomp['velodyne'][0]
    result['kimera_belief_rows']=len(eventcomp['hesai'][1])
    result['kimera_receipt_clock_difference_s']=stat(np.abs(eventcomp['hesai'][1]-eventcomp['velodyne'][1])*1e-9)
    result['imu_samples_identical']=eventcomp['hesai'][2]==eventcomp['velodyne'][2]
    fig,axs=plt.subplots(3,2,figsize=(13,10),sharex='row',constrained_layout=True)
    for col,(name,(t,op,fp,gp,front,count)) in enumerate(data.items()):
        for p,color,label in [(op,'#d43f57','ONLINE'),(fp,'#17865c','DELAYED finalized'),(gp,'#347ac8','IE-TC')]:
            axs[0,col].plot(t[1:],np.linalg.norm(np.diff(p,axis=0),axis=1)*100,color=color,lw=.8,label=label)
        axs[0,col].set(title=name.upper()+' CBS OFF',ylabel='100ms step length (cm)',xlim=(0,60),ylim=(0,18));axs[0,col].legend()
        for k,color,i in [('GLIM','#17865c',0),('Kimera','#dc921e',1)]:
            axs[1,col].plot(t,front[:,i]*1000,color=color,lw=.8,label=k)
        axs[1,col].set(ylabel='Newest received endpoint age (ms)',xlim=(10,12),ylim=(0,250));axs[1,col].legend()
        correction=np.linalg.norm(op-fp,axis=1)*100
        axs[2,col].plot(t,correction,color='#d43f57',lw=.8)
        axs[2,col].set(ylabel='Later correction magnitude (cm)',xlabel='Seconds from capture start',xlim=(0,60),ylim=(0,18))
        for ax in axs[:,col]:ax.grid(alpha=.2)
    fig.suptitle('Same original receiver and physical timestamps: Hesai versus Velodyne\nSaved OFF data; no smoothing or local fit. Endpoint age is not CPU latency.')
    fig.savefig(H/'hesai_velodyne_off_timing.png',dpi=150);plt.close(fig)
    arrays={}
    for name,vals in data.items():
        for key,x in zip(('time_s','online','finalized','reference','frontier_age_s','received_counts'),vals):arrays[name+'_'+key]=x
    np.savez_compressed(H/'timeseries.npz',**arrays)
    j.HASHES.update(a.HASHES)
    result['input_sha256']=dict(j.HASHES)
    result['inputs_unchanged']=all(j.sha(p)==digest for p,digest in j.HASHES.items());assert result['inputs_unchanged']
    save('results.json',result)
    for name in CASES:
        for w in ('full','first60','first10','10to30','30to60','late380to390'):
            o=result['cases'][name]['windows'][w]
            print(name,w,'paths',o['online']['path_m'],o['finalized']['path_m'],o['online']['reference_path_m'],
                  'vector_RMS_cm',o['online']['step_vector_error_m']['rms']*100,
                  'freshness_ms', {k:round(v['median']*1000,3) for k,v in o['source_frontier_age_s'].items()})
    print({k:v for k,v in result.items() if k not in ('cases','input_sha256','windows','receiver_config_differences','glim_parameter_differences')})

if __name__=='__main__':main()
