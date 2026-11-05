"""CP-V15 saved-output evaluation, with predeclared scientific screen."""
import csv, hashlib, importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core.geometry import umeyama_alignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent;E=H.parent
R=Path('/home/yeranis/results/gnssfgo_leica2_motion_prior_quarter_cpv15_20261026')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
spec=importlib.util.spec_from_file_location('v10',E/'first_published_comparison_cpv10_20261026/analyze.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);a=c.a
HALF=E/'leica2_motion_prior_cpv14_20261026'
HALF_R=Path('/home/yeranis/results/gnssfgo_leica2_motion_prior_cpv14_20261026')
CP10=E/'first_published_comparison_cpv10_20261026/outputs'
EPOCH=1732549034915071665;TP=np.array([.308837802,-.038312997,-.234944116])
N=lambda x:np.linalg.norm(x,axis=-1)
rms=lambda x:float(np.sqrt(np.mean(np.square(x))))
PINS={}

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(2**20),b''): h.update(b)
    return h.hexdigest()
def pin(p):PINS[str(p)]=sha(p);return p
def save(p,obj):
    def default(x):
        if isinstance(x,np.ndarray):return x.tolist()
        if isinstance(x,np.generic):return x.item()
        raise TypeError(type(x))
    with open(p,'x') as f:json.dump(obj,f,indent=2,default=default,allow_nan=False)
def stat(x):
    x=np.asarray(x,float)
    return dict(n=len(x),rms=rms(x),median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(np.max(x))) if len(x) else dict(n=0)

def metrics(ep,er,gp,gr,mask):
    idx=np.flatnonzero(mask);assert len(idx)>20
    ate,_,check=c.ate(ep[idx],gp[idx])
    rr,tr,scale=umeyama_alignment(ep[idx].T,gp[idx].T,with_scale=False);assert scale==1
    aligned=ep@rr.T+tr
    edge=mask[:-1]&mask[1:]
    d=np.diff(aligned,axis=0);dg=np.diff(gp,axis=0)
    result=dict(ATE_RMSE_m=ate,evo_alignment_difference_m=check,
        path_m=float(N(d)[edge].sum()),reference_path_m=float(N(dg)[edge].sum()),
        displacement100ms_error_m=stat(N(d-dg)[edge]),rows=len(idx),supported_s=float(edge.sum()*.1),horizons={})
    for ticks in [1,10]:
        ii=a.intervals(mask,ticks);jj=ii+ticks
        dt,dr,_=a.rpe(ep,er,gp,gr,ii,jj)
        result['horizons'][str(ticks/10)]=dict(pairs=len(ii),translation_RMSE_m=rms(N(dt)),rotation_RMSE_deg=rms(N(dr)))
        for ix in np.linspace(0,len(ii)-1,15,dtype=int):
            i,j=ii[ix],jj[ix]
            def pose(p,r):
                T=np.eye(4);T[:3,:3]=r;T[:3,3]=p;return T
            de=np.linalg.inv(pose(ep[i],er[i]))@pose(ep[j],er[j])
            dgT=np.linalg.inv(pose(gp[i],gr[i]))@pose(gp[j],gr[j])
            err=np.linalg.inv(dgT)@de
            assert abs(N(err[:3,3])-N(dt[ix]))<1e-7
            assert abs(N(Rotation.from_matrix(err[:3,:3]).as_rotvec())*180/np.pi-N(dr[ix]))<1e-7
    dt,dr,_=a.rpe(ep,er,gp,gr,np.arange(len(ep)-1),np.arange(1,len(ep)))
    speed=N(np.diff(gp,axis=0))/.1
    angular=N(Rotation.from_matrix(np.swapaxes(gr[:-1],1,2)@gr[1:]).as_rotvec())*180/np.pi/.1
    result['motion_cohorts']={}
    for name,cohort in [('moving',speed>=.2),('turning',angular>=5),('slow',speed<.2)]:
        use=cohort&edge
        result['motion_cohorts'][name]=dict(edges=int(use.sum()),translation_RMSE_m=rms(N(dt)[use]) if use.any() else None,
             rotation_RMSE_deg=rms(N(dr)[use]) if use.any() else None,
             displacement100ms_error_RMS_m=rms(N(d-dg)[use]) if use.any() else None)
    return result,aligned,N(d-dg),speed,angular

def lag_diagnostic(ep,er,t,reference):
    es=N(np.diff(ep,axis=0))/.1
    ea=N(Rotation.from_matrix(np.swapaxes(er[:-1],1,2)@er[1:]).as_rotvec())/.1
    usable=(t[:-1]>=reference[0][0]+500_000_000)&(t[1:]<=reference[0][-1]-500_000_000)
    curves=[]
    for lag in range(-500_000_000,500_000_001,20_000_000):
        p0,r0,ok0=a.sample(reference,t[:-1]-lag,50_000_000)
        p1,r1,ok1=a.sample(reference,t[1:]-lag,50_000_000)
        p0,r0=a.prism(p0,r0);p1,r1=a.prism(p1,r1)
        assert (ok0[usable]&ok1[usable]).all()
        gs=N(p1-p0)/.1;ga=N(Rotation.from_matrix(np.swapaxes(r0,1,2)@r1).as_rotvec())/.1
        curves.append(dict(lag_s=lag*1e-9,speed_mse=float(np.mean((es[usable]-gs[usable])**2)),
                           angular_speed_mse=float(np.mean((ea[usable]-ga[usable])**2))))
    return dict(positive_means_estimate_follows_older_reference=True,rows=int(usable.sum()),
        speed_best_lag_s=min(curves,key=lambda x:x['speed_mse'])['lag_s'],
        angular_speed_best_lag_s=min(curves,key=lambda x:x['angular_speed_mse'])['lag_s'],scan=curves)

def trace_summary(mode):
    with open(pin(R/f'{mode}_trace'/'trace_meta.csv')) as f: meta=list(csv.DictReader(f))
    # Retain newest point only; full active-window trace remains on disk.
    newest={int(m['snapshot']):int(m['newest']) for m in meta};poses={}
    with open(pin(R/f'{mode}_trace'/'trace_states.csv')) as f:
        reader=csv.reader(f);next(reader)
        for row in reader:
            sid,idx=map(int,row[:2])
            if idx==newest[sid]:
                pose=np.array(row[2:5],float)+Rotation.from_quat(np.array(row[5:9],float)).apply(TP)
                poses[sid]=pose
    shifts={}
    for j,m in enumerate(meta):
        if not m['stage'].endswith('_before'):continue
        y=meta[j+1];assert y['stage']==m['stage'].replace('_before','_after')
        group=m['source']+'_'+m['action'] if m['stage']=='source_before' else m['stage'].split('_')[0]
        shifts.setdefault(group,[]).append(N(poses[int(y['snapshot'])]-poses[int(m['snapshot'])]))
    return {k:stat(v) for k,v in shifts.items()}

def main():
    assert json.loads(pin(H/'replay_complete.json').read_text())['pass_']
    half_close=json.loads(pin(HALF/'closeout.json').read_text());assert half_close['pass_']
    half_results=json.loads(pin(HALF/'results.json').read_text())
    assert sha(HALF/'results.json')==half_close['artifact_sha256'][str(HALF/'results.json')]
    for mode in ['off','on']:
        for product in ['online','finalized']:
            path=HALF_R/mode/(product+'_trajectory.tum')
            assert sha(pin(path))==half_close['candidate_output_sha256'][str(path)]
    old=json.loads(pin(CP10/'velodyne_leica2.json').read_text())
    common=np.load(pin(CP10/'velodyne_leica2.npz'));grid=common['timestamp_ns'];valid=common['common_valid']
    reference_bag=pin(a.GT);ref=a.g.Reference(reference_bag)
    reference=np.asarray(ref.times,np.int64),np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]
    full={};comm={};curves={};arrays={};traces={};checks={}
    for mode in ['off','on']:
        assert json.loads(pin(H/f'{mode}_validation.json').read_text())['pass_']
        for treatment,root in [('baseline',P/f'full_{mode}_receiver'/'receiver'),('qc_half',HALF_R/mode),('qc_quarter',R/mode)]:
            for product,file in [('online','online_trajectory.tum'),('finalized','finalized_trajectory.tum')]:
                key=f'{mode}_{treatment}_{product}';curve=a.load_curve(pin(root/file));curves[key]=curve
                t,p,r=curve;assert len(t)==4282 and np.all(np.diff(t)==100_000_000)
                gp,gr,ok=a.sample(reference,t,50_000_000);assert ok.all()
                ep,er=a.prism(p,r);gp,gr=a.prism(gp,gr)
                full[key],aligned,disp,speed,angular=metrics(ep,er,gp,gr,np.ones(len(t),bool))
                full[key].update(first_s=float((t[0]-EPOCH)*1e-9),last_s=float((t[-1]-EPOCH)*1e-9),
                   start_gap_s=float((t[0]-EPOCH)*1e-9),tail_gap_s=float(428.75-(t[-1]-EPOCH)*1e-9))
                if treatment=='baseline' and product=='online':
                    delta=abs(full[key]['ATE_RMSE_m']-old['full_saved'][mode+'_fusion_online']['ATE_RMSE_m'])
                    assert delta<1e-7;checks[mode+'_original_ATE_difference']=delta
                if treatment in ['baseline','qc_half']:
                    expected=half_results['full'][key]
                    for field in ['ATE_RMSE_m','path_m','reference_path_m']:
                        assert abs(full[key][field]-expected[field])<1e-7,(key,field)
                p1,r1,ok=a.sample(curve,grid,500_000_000);assert ok[valid].all();ep1,er1=a.prism(p1,r1)
                comm[key],_=c.metrics(ep1,er1,common['reference_position'],common['reference_rotation'],valid,grid)
                if product=='online':
                    full[key]['response']=lag_diagnostic(ep,er,t,reference)
                    sel=((t-EPOCH)*1e-9>=379.984)&((t-EPOCH)*1e-9<=390)
                    full[key]['selected_380_390'],_,_,_,_=metrics(ep,er,gp,gr,sel)
                    full[key]['selected_380_390']['ATE_uses_local_window_fit_not_primary']=True
                    use=sel[:-1]&sel[1:]
                    full[key]['selected_380_390']['displacement_error_using_full_fit_m']=stat(disp[use])
                    arrays[key+'_aligned']=aligned;arrays[key+'_step_error']=disp
        t=curves[mode+'_baseline_online'][0];assert np.array_equal(t,curves[mode+'_qc_quarter_online'][0])
        arrays[mode+'_time_s']=(t-EPOCH)*1e-9
        arrays[mode+'_reference']=gp;arrays[mode+'_reference_speed']=speed;arrays[mode+'_reference_angular_speed']=angular
        traces[mode]=trace_summary(mode)
    gates={};warnings={}
    for mode in ['off','on']:
        b=full[mode+'_baseline_online'];n=full[mode+'_qc_quarter_online']
        gates[mode]={
            '100ms_displacement_improves':n['displacement100ms_error_m']['rms']<b['displacement100ms_error_m']['rms'],
            '100ms_translation_RPE_improves':n['horizons']['0.1']['translation_RMSE_m']<b['horizons']['0.1']['translation_RMSE_m'],
            'ATE_no_worse':n['ATE_RMSE_m']<=b['ATE_RMSE_m']+1e-6,
            'RPE1_translation_no_worse':n['horizons']['1.0']['translation_RMSE_m']<=b['horizons']['1.0']['translation_RMSE_m']+1e-6,
            'RPE1_rotation_no_worse':n['horizons']['1.0']['rotation_RMSE_deg']<=b['horizons']['1.0']['rotation_RMSE_deg']+1e-6}
        for cohort in ['moving','turning']:
            for field in ['translation_RMSE_m','rotation_RMSE_deg']:
                gates[mode][cohort+'_'+field+'_no_worse']=n['motion_cohorts'][cohort][field]<=b['motion_cohorts'][cohort][field]+1e-6
        warnings[mode]={k:(n['response'][k]>max(0,b['response'][k])+.020000001) for k in ['speed_best_lag_s','angular_speed_best_lag_s']}
    result=dict(task='CP-V15',qc_multiplier=.25,full=full,common=comm,common_coverage=old['common_coverage'],
        unchanged_native_controls={m+'_'+role:old['full_saved'][m+'_'+role] for m in ['off','on'] for role in ['glim_initial','kimera']},
        unchanged_native_common={m+'_'+role:old['common'][m+'_'+role] for m in ['off','on'] for role in ['glim_initial','kimera']},
        trace_updates=traces,scientific_gates=gates,scientific_pass=all(all(x.values()) for x in gates.values()),
        response_lag_warnings=warnings,baseline_checks=checks,synthetic_checks=a.tests(),not_promoted=True,
        control_qc_half_reproduced=True,screen_comparator='original Qc, same predeclared CP-V14 criteria')
    for module in [c,c.b,a,a.g]:pin(Path(module.__file__))
    for p in [Path(__file__),H/'PROTOCOL.txt']:pin(p)
    PINS.update({str(k):v for k,v in a.HASHES.items()})
    assert all(sha(p)==v for p,v in PINS.items())
    save(H/'results.json',result);save(H/'scoring_pins.json',PINS)
    np.savez_compressed(H/'comparison_arrays.npz',**arrays)
    fig,axes=plt.subplots(3,2,figsize=(14,10),constrained_layout=True)
    for col,mode in enumerate(['off','on']):
        t=arrays[mode+'_time_s'];gp=arrays[mode+'_reference']
        for tr,label,color in [('baseline','Original Qc','#d12e44'),('qc_half','0.5 x Qc','#e49b0f'),('qc_quarter','0.25 x Qc','#2474b4')]:
            key=mode+'_'+tr+'_online';p=arrays[key+'_aligned'];err=arrays[key+'_step_error']
            axes[0,col].plot(t[1:],100*np.sqrt(np.convolve(err**2,np.ones(10)/10,'same')),label=label,color=color,lw=1)
            axes[1,col].plot(t,np.r_[0,np.cumsum(N(np.diff(p,axis=0)))],label=label,color=color)
            win=(t[1:]>380)&(t[1:]<=390)
            axes[2,col].plot(t[1:][win],100*N(np.diff(p,axis=0))[win],label=label,color=color)
        axes[1,col].plot(t,np.r_[0,np.cumsum(N(np.diff(gp,axis=0)))],label='IE-TC',color='black')
        axes[2,col].plot(t[1:][win],100*N(np.diff(gp,axis=0))[win],label='IE-TC',color='black')
        axes[0,col].set(title='CBS '+mode.upper(),ylabel='100ms vector error, 1s rolling RMS (cm)')
        axes[1,col].set(ylabel='Cumulative first-published path (m)')
        axes[2,col].set(ylabel='100ms step length (cm)',xlabel='Physical time after capture start (s)')
        for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Leica-2 Velodyne: original / half / quarter Qc, receiver-only saved-journal replay\nAge-zero output, unchanged source beliefs/covariances; selected window is secondary')
    fig.savefig(H/'comparison.png',dpi=170);plt.close(fig)
    print(json.dumps(dict(scientific_pass=result['scientific_pass'],gates=gates,response_warnings=warnings),indent=2))

if __name__=='__main__':main()
