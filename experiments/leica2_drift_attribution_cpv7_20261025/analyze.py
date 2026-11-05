"""CP-V7: immutable saved-data analysis only; no ROS/optimizer execution."""
import csv
import gzip
import hashlib
import importlib.util
import json
import sys
from decimal import Decimal
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import spearmanr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
EXP = H.parent
RUN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
P4 = RUN/'gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared'
P6 = RUN/'gnssfgo_leica2_rotation_weight_cpv6a_20261024/prepared'
H5 = EXP/'leica2_velodyne_belief_audit_cpv5_20261024'
H6 = EXP/'leica2_rotation_weight_cpv6a_20261024'
GT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/LEICA-2/2024-11-25-16-36-19_cpt7_ie_tc.bag')
START = 1732549034915071665
STEP = 100_000_000
MODES = ('off', 'original_on', 'rotation10_on')
ROLES = ('glim', 'kimera', 'fusion_online')
NAMES = {'off':'CBS OFF', 'original_on':'Original CBS ON', 'rotation10_on':'Rotation variance 10 ON'}
COLORS = dict(zip(MODES, ('#2077b4', '#ed8a22', '#b34077')))
TP = np.array([.308837802, -.038312997, -.234944116])
RP = R.from_quat([-.005724388, -.001387943, -.004742722, .999971405]).as_matrix()
spec = importlib.util.spec_from_file_location('geo', EXP/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
HASHES = {}


def pin(p):
    p = Path(p).resolve()
    if str(p) not in HASHES: HASHES[str(p)] = g.sha(p)
    return p


def serial(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    raise TypeError(type(x).__name__)


def save(p, v):
    with p.open('x') as f: json.dump(v, f, indent=2, default=serial, allow_nan=False)


def stat(a):
    a = np.asarray(a, float)
    assert len(a) and np.isfinite(a).all()
    return dict(n=len(a), mean=float(a.mean()), rms=float(np.sqrt(np.mean(a*a))),
                median=float(np.median(a)), p95=float(np.percentile(a,95)), max=float(a.max()))


def load_curve(path):
    pin(path)
    if path.suffix == '.csv':
        rows = list(csv.DictReader(path.open()))
        assert all(x['field.child_frame_id']=='cpt7_imu' for x in rows)
        ts = np.array([int(x['field.header.stamp']) for x in rows], np.int64)
        p = np.array([[float(x['field.pose.pose.position.'+k]) for k in 'xyz'] for x in rows])
        q = np.array([[float(x['field.pose.pose.orientation.'+k]) for k in 'xyzw'] for x in rows])
    else:
        rows = [x.split() for x in path.read_text().splitlines() if x.strip() and not x.startswith('#')]
        ts = np.array([int(Decimal(x[0])*10**9) for x in rows],np.int64)
        p = np.array([x[1:4] for x in rows],float); q = np.array([x[4:8] for x in rows],float)
    assert np.all(np.diff(ts)>0) and np.isfinite(p).all() and np.isfinite(q).all()
    return ts,p,R.from_quat(q).as_matrix()


def sample(curve, query, gap):
    ts,p,rot = curve
    idx = np.searchsorted(ts,query)
    right = np.clip(idx,0,len(ts)-1); left = np.maximum(right-1,0)
    exact = (idx<len(ts)) & (ts[right]==query)
    ok = exact | ((idx>0)&(idx<len(ts))&((ts[right]-ts[left])<=gap))
    alpha = np.divide(query-ts[left],ts[right]-ts[left],out=np.zeros(len(query),float),where=right!=left)
    pp = p[left]+alpha[:,None]*(p[right]-p[left])
    delta = R.from_matrix(np.swapaxes(rot[left],1,2)@rot[right]).as_rotvec()
    rr = rot[left]@R.from_rotvec(alpha[:,None]*delta).as_matrix()
    pp[exact] = p[right[exact]]; rr[exact] = rot[right[exact]]
    return pp,rr,ok


def prism(p,rot): return p+np.einsum('nij,j->ni',rot,TP), rot@RP


def intervals(valid,n):
    starts = np.arange(len(valid)-n)
    bad = np.r_[0,np.cumsum(~valid)]
    return starts[(bad[starts+n+1]-bad[starts])==0]


def rpe(ep,er,gp,gr,i,j):
    et = np.einsum('nji,nj->ni',er[i],ep[j]-ep[i])
    gt = np.einsum('nji,nj->ni',gr[i],gp[j]-gp[i])
    eR = np.swapaxes(er[i],1,2)@er[j]
    gR = np.swapaxes(gr[i],1,2)@gr[j]
    phi = R.from_matrix(np.swapaxes(gR,1,2)@eR).as_rotvec()
    world = np.einsum('nij,nj->ni',gr[j],phi)*180/np.pi
    return et-gt,phi*180/np.pi,world


def best_fit(ep,gp):
    ec = ep.mean(0); gc = gp.mean(0)
    u,_,vt = np.linalg.svd((ep-ec).T@(gp-gc))
    d = np.eye(3); d[-1,-1] = np.linalg.det(vt.T@u.T)
    rot = vt.T@d@u.T
    return ep@rot.T + (gc-rot@ec) - gp


def fixed_start(ep,er,gp,gr,anchor):
    rot = gr[anchor]@er[anchor].T
    alignedp = (ep-ep[anchor])@rot.T+gp[anchor]
    alignedr = rot@er
    world = R.from_matrix(alignedr@np.swapaxes(gr,1,2)).as_rotvec()*180/np.pi
    angles = (R.from_matrix(alignedr).as_euler('xyz',degrees=True)-R.from_matrix(gr).as_euler('xyz',degrees=True)+180)%360-180
    return alignedp-gp,world,angles


def summary_signed(world, starts):
    z = world[:,2]
    contiguous = np.diff(starts)==10
    corr = float(np.corrcoef(z[:-1][contiguous],z[1:][contiguous])[0,1]) if contiguous.sum()>2 and np.std(z)>0 else None
    norm = np.linalg.norm(world,axis=1)
    return dict(n=len(z),mean_world_xyz_deg=world.mean(0),rms_world_xyz_deg=np.sqrt(np.mean(world**2,0)),
                mean_vector_norm_over_rms=float(np.linalg.norm(world.mean(0))/np.sqrt(np.mean(norm**2))),
                positive_z_fraction=float(np.mean(z>0)),negative_z_fraction=float(np.mean(z<0)),
                consecutive_1s_z_correlation=corr)


def tests():
    assert g.self_test()
    t = np.arange(401,dtype=np.int64)*STEP
    p = np.c_[np.arange(401)*.02,np.sin(np.arange(401)*.01),np.zeros(401)]
    rot = R.from_euler('z',np.arange(401)[:,None]*.001).as_matrix()
    A = R.from_rotvec([.1,-.2,.3]).as_matrix()
    ee = p@A.T+np.array([2.,3.,4.]); er=A@rot
    i=np.arange(201);j=i+200
    de,dr,_=rpe(ee,er,p,rot,i,j)
    assert np.max(np.abs(de))<1e-12 and np.max(np.abs(dr))<1e-12
    pos,phi,ang=fixed_start(ee,er,p,rot,0)
    assert np.max(np.abs(pos))<1e-12 and np.max(np.abs(phi))<1e-12
    drift=R.from_euler('z',np.arange(401)[:,None]*.001).as_matrix()
    _,err,_=rpe(p,drift,p,np.tile(np.eye(3),(401,1,1)),i,j)
    assert np.allclose(err[:,2],.2*180/np.pi,atol=1e-12)
    bad=np.ones(401,bool);bad[200]=False
    assert all(not(a<=200<=a+10) for a in intervals(bad,10))
    pp,rr,ok=sample((t,p,rot),t,STEP)
    assert ok.all() and np.array_equal(pp,p) and np.array_equal(rr,rot)
    return dict(passed=True,identity_and_global_transform=True,known_yaw_rate=True,gap_exclusion=True,exact_samples=True)


def trajectories():
    print('Load saved native/ONLINE trajectories and IE-TC reference',flush=True)
    curves={};source_roots={}
    for mode in MODES:
        root=P6 if mode=='rotation10_on' else P4; case='off' if mode=='off' else 'on'
        src=root/f'full_{case}_source'; rec=root/f'full_{case}_receiver/receiver'
        source_roots[mode]=src
        for role,path in zip(ROLES,(src/'odom_imu.txt',src/'kimera_odometry.csv',rec/'online_trajectory.tum')):
            curves[mode,role]=load_curve(path)
    ref=g.Reference(pin(GT))
    reference=(ref.times,np.array(ref.poses)[:,:3,3],np.array(ref.poses)[:,:3,:3])
    start=max(c[0][0] for c in curves.values());end=min(c[0][-1] for c in curves.values())
    grid=np.arange(((start+STEP-1)//STEP)*STEP,(end//STEP)*STEP+1,STEP,dtype=np.int64)
    gp,gr,valid=sample(reference,grid,50_000_000)
    samples={}
    for key,c in curves.items():
        ep,er,ok=sample(c,grid,500_000_000);valid &= ok;samples[key]=(ep,er)
    pgt,rgt=prism(gp,gr)
    vi=np.flatnonzero(valid);anchor=int(vi[0]);time=(grid-START)*1e-9
    segments=np.split(vi,np.flatnonzero(np.diff(vi)>1)+1)
    cover=dict(grid_queries=len(grid),valid_queries=len(vi),excluded_queries=int((~valid).sum()),
               start_s=float(time[vi[0]]),end_s=float(time[vi[-1]]),
               covered_contiguous_s=sum(float((grid[x[-1]]-grid[x[0]])*1e-9) for x in segments),
               segments=[dict(start_s=float(time[x[0]]),end_s=float(time[x[-1]]),n=len(x)) for x in segments],
               reference_max_abs_pitch_deg=float(np.abs(R.from_matrix(gr[valid]).as_euler('xyz',degrees=True)[:,1]).max()))
    saved=json.loads(pin(P6/'evaluation_full/summary.json').read_text())
    comparisons={};data={};crosscheck={}
    for (mode,role),(ep,er) in samples.items():
        pe,re=prism(ep,er); key=mode+'/'+role
        # Fixed-start positions at prism; signed orientation in original CPT7 axes.
        dp,_,_=fixed_start(pe,re,pgt,rgt,anchor)
        _,phi,angles=fixed_start(ep,er,gp,gr,anchor)
        fitted=best_fit(pe[valid],pgt[valid])
        fullc=curves[mode,role]; fullg=sample(reference,fullc[0],50_000_000)
        assert fullg[2].all()
        fep,_=prism(fullc[1],fullc[2]);fgp,_=prism(fullg[0],fullg[1])
        fullate=stat(np.linalg.norm(best_fit(fep,fgp),axis=1))['rms']
        oldkey={'off':'off','original_on':'baseline_on','rotation10_on':'on'}[mode]+'_'+{'glim':'native_glim','kimera':'native_kimera','fusion_online':'fused_online'}[role]
        delta=abs(fullate-saved['results'][oldkey]['full_saved']['ATE_RMSE_m'])
        assert delta<1e-5,(key,fullate,delta)
        crosscheck[key]=dict(full_saved_ATE_m=fullate,absolute_difference_from_saved_m=delta)
        d=dict(common_ATE_m=stat(np.linalg.norm(fitted,axis=1))['rms'],
               start_aligned_position_m=stat(np.linalg.norm(dp[valid],axis=1)),
               terminal_common_position_m=float(np.linalg.norm(dp[vi[-1]])),
               terminal_common_heading_deg=float(angles[vi[-1],2]),
               terminal_common_world_rotation_xyz_deg=phi[vi[-1]],
               heading_deg=stat(angles[valid,2]),horizons={},bins=[],anchor_sensitivity=[])
        series={}
        for h in (1,5,10,20):
            ii=intervals(valid,h*10);jj=ii+h*10
            assert np.all(grid[jj]-grid[ii]==h*1_000_000_000)
            dt,dr,dworld=rpe(pe,re,pgt,rgt,ii,jj)
            terr=np.linalg.norm(dt,axis=1);rerr=np.linalg.norm(dr,axis=1)
            d['horizons'][str(h)]=dict(n=len(ii),translation_m=stat(terr),rotation_deg=stat(rerr),
                                      rotation_world_signed=summary_signed(dworld,ii) if h==1 else None)
            series[h]=(ii,jj,terr,rerr,dworld)
        ii,jj,te,rr,ww=series[1]; disjoint=(ii-anchor)%10==0
        d['nonoverlap_1s_signed']=summary_signed(ww[disjoint],ii[disjoint])
        for a in range(0,450,30):
            mask=valid&(time>=a)&(time<a+30)
            m=(time[jj]>=a)&(time[jj]<a+30)
            if not mask.any():continue
            b=dict(start_s=a,end_s=a+30,n=int(mask.sum()),
                   fixed_position_rms_m=stat(np.linalg.norm(dp[mask],axis=1))['rms'],
                   heading_mean_deg=float(angles[mask,2].mean()),heading_last_deg=float(angles[np.flatnonzero(mask)[-1],2]))
            if m.any():
                b.update(rpe1_translation_m=stat(te[m])['rms'],rpe1_rotation_deg=stat(rr[m])['rms'],
                         rpe1_world_signed_mean_deg=ww[m].mean(0))
            ii20,_,_,rr20,_=series[20]; m20=(time[ii20]>=a)&(time[ii20]<a+30)
            if m20.any():b['rpe20_rotation_rms_deg']=stat(rr20[m20])['rms']
            d['bins'].append(b)
        for offset in (0,1,5):
            ai=anchor+offset*10; assert valid[ai]
            sp,_,_=fixed_start(pe,re,pgt,rgt,ai)
            _,_,aa=fixed_start(ep,er,gp,gr,ai)
            later=valid&(np.arange(len(grid))>=ai)
            d['anchor_sensitivity'].append(dict(anchor_s=float(time[ai]),position_rms_m=stat(np.linalg.norm(sp[later],axis=1))['rms'],
                                               terminal_position_m=float(np.linalg.norm(sp[vi[-1]])),terminal_heading_deg=float(aa[vi[-1],2])))
        comparisons[key]=d;data[key]=dict(position=dp,rotation_world=phi,angles=angles,series=series,prism_p=pe)
    return ref,grid,valid,cover,comparisons,data,crosscheck,source_roots


def peer_records(src,mode,ref):
    print('Join actual G2K matrices and exact Kimera pre/post snapshots:',mode,flush=True)
    main=pin(src/'estimators.log')
    logs={p.resolve() for p in src.glob('ros_logs/*/kimera_vio_ros-*-stdout.log')}
    assert len(logs)==1;stdout=pin(next(iter(logs)))
    aligned=[];frozen={};edge={}
    for tag,f in g.rows(main,['KIMERA_CBS_MEAN_ALIGNMENT_ROW','FL4M_FROZEN_FACTOR_ROW','KIMERA_CBS_REVISION_EDGE_ROW']):
        if tag=='KIMERA_CBS_MEAN_ALIGNMENT_ROW':
            cur,si,sj,i,j=map(int,f[:5]);sa,sb,a,b,limit=map(int,f[5:10])
            aligned.append(dict(cur=cur,si=si,sj=sj,i=i,j=j,sa=sa,sb=sb,a=a,b=b,limit=limit,
                                mu=g.matrix(f[11]).ravel(),raw_cov=g.matrix(f[12])))
        elif tag=='FL4M_FROZEN_FACTOR_ROW':
            key=tuple(map(int,f[3:5]));assert key not in frozen
            frozen[key]=g.matrix(f[5]).ravel(),g.matrix(f[6])
        else:
            key=int(f[0]),int(f[2]),int(f[3]);assert key not in edge
            edge[key]=np.array(f[22:28],float),np.array(f[28:64],float).reshape(6,6)
    applied={};prelog={};explicit={}
    for tag,f in g.rows(stdout,['CBS_ODOM_FACTOR_MATRIX_ROW','CBS_ODOM_PREINJECTION_RESIDUAL_ROW','CBS_COVARIANCE_EXCHANGE_MATRIX_ROW']):
        if f[0]!='G2K':continue
        if tag=='CBS_ODOM_FACTOR_MATRIX_ROW':
            key=tuple(int(x[1:]) for x in f[1:3]);assert key not in applied
            assert f[3]=='active_window_temporary_odom_applied';applied[key]=g.matrix(f[4])
        elif tag=='CBS_ODOM_PREINJECTION_RESIDUAL_ROW':
            key=tuple(int(x[1:]) for x in f[3:5]);assert key not in prelog
            prelog[key]=np.array(f[7:],float)
        elif f[8:10]==['receiver_preinjection_state','relative_pose_matrix']:
            key=tuple(int(x[1:]) for x in f[5:7]);explicit[key]=g.matrix(f[12])
    assert {(x['i'],x['j']) for x in aligned}==set(applied)==set(frozen)==set(prelog)
    result=[];maxpre=0.;maxc=0.
    for x in aligned:
        key=x['i'],x['j'];cur=x['cur'];mu=x['mu'];C=x['raw_cov'];ac=applied[key]
        assert x['a']<x['b']<=x['limit'] and x['sa']<x['sb']<=x['limit']
        expected=C.copy()
        if mode=='rotation10_on':expected[:3,:3]*=10
        maxc=max(maxc,float(np.abs(ac-expected).max()))
        assert np.allclose(ac,expected,atol=1e-18,rtol=1e-10)
        assert np.allclose(frozen[key][0],mu,atol=1e-12,rtol=1e-10) and np.array_equal(frozen[key][1],C)
        assert np.linalg.eigvalsh(ac).min()>0
        premu,preC=edge[cur-1,*key];postmu,_=edge[cur,*key]
        z=g.exp(mu);pre=g.exp(premu);post=g.exp(postmu)
        residual=g.log(g.inv(z)@pre);v=prelog[key]
        errs=[abs(np.linalg.norm(residual)-v[0]),abs(np.linalg.norm(residual[:3])-v[1]),abs(np.linalg.norm(residual[3:])-v[2])]
        q=float(residual@np.linalg.solve(ac,residual));errs.append(abs(q-v[7])/(1+q))
        if key in explicit:errs.append(float(np.abs(pre-explicit[key]).max()))
        maxpre=max(maxpre,max(errs));assert max(errs)<1e-8,(key,errs)
        ra,rb=ref.at(x['a']),ref.at(x['b']);target=g.inv(ra)@rb
        rec=dict(mode=mode,i=x['i'],j=x['j'],cur=cur,a=x['a'],b=x['b'],
                 time_s=(x['b']-START)*1e-9,update_time_s=(x['limit']-START)*1e-9,
                 interval_s=(x['b']-x['a'])*1e-9,raw_cov=C,applied_cov=ac,pre_cov=preC,
                 incoming_mu=mu,pre_mu=premu,post_mu=postmu)
        for name,T in [('incoming',z),('pre',pre),('post',post)]:
            er=g.log(g.inv(target)@T)[:3]
            world=rb[:3,:3]@er*180/np.pi
            rec[name]=dict(rotation_deg=float(np.linalg.norm(er)*180/np.pi),world_rotation_xyz_deg=world,
                           translation_mm=float(np.linalg.norm(T[:3,3]-target[:3,3])*1000))
        rec['applied_rotation_norm_deg']=float(np.sqrt(np.trace(ac[:3,:3]))*180/np.pi)
        rec['raw_rotation_norm_deg']=float(np.sqrt(np.trace(C[:3,:3]))*180/np.pi)
        rec['pre_rotation_norm_deg']=float(np.sqrt(np.trace(preC[:3,:3]))*180/np.pi)
        result.append(rec)
    return result,dict(n=len(result),verified_pre_count=len(result),explicit_pre_count=len(explicit),
                       max_pre_check=maxpre,max_applied_covariance_error=maxc)


def peer_summary(rows):
    if not rows:return dict(n=0)
    d=dict(n=len(rows))
    for field in ('incoming','pre','post'):
        world=np.array([x[field]['world_rotation_xyz_deg'] for x in rows])
        d[field]=dict(rotation_deg=stat([x[field]['rotation_deg'] for x in rows]),
                       translation_mm=stat([x[field]['translation_mm'] for x in rows]),
                       world_signed_mean_deg=world.mean(0),world_rms_deg=np.sqrt(np.mean(world**2,0)))
    inc=np.array([x['incoming']['rotation_deg'] for x in rows]);pre=np.array([x['pre']['rotation_deg'] for x in rows]);post=np.array([x['post']['rotation_deg'] for x in rows])
    d.update(incoming_rotation_worse_fraction=float(np.mean(inc>pre)),post_rotation_worse_fraction=float(np.mean(post>pre)),
             incoming_cov_error_rank=float(spearmanr([x['applied_rotation_norm_deg'] for x in rows],inc).statistic),
             applied_rotation_norm_deg=stat([x['applied_rotation_norm_deg'] for x in rows]),
             pre_rotation_norm_deg=stat([x['pre_rotation_norm_deg'] for x in rows]))
    return d


def render(out,time,valid,metrics,data,peers):
    plt.rcParams.update({'font.size':10,'savefig.facecolor':'white'})
    fig,axes=plt.subplots(3,2,figsize=(12,10),sharex=True)
    for row,role in enumerate(ROLES):
        for mode in MODES:
            for col,block,unit in ((0,'translation_m','m'),(1,'rotation_deg','deg')):
                hs=(1,5,10,20);v=[metrics[mode+'/'+role]['horizons'][str(h)][block]['rms'] for h in hs]
                axes[row,col].plot(hs,v,'-o',color=COLORS[mode],label=NAMES[mode])
                axes[row,col].set_ylabel(f'{role.replace("_"," ").title()} RMS ({unit})')
        for ax in axes[row]:ax.grid(alpha=.25);ax.set_xticks([1,5,10,20]);ax.set_xlabel('Relative-motion interval (seconds)')
    axes[0,0].set_title('Translation error');axes[0,1].set_title('Rotation error');axes[0,0].legend(fontsize=8)
    fig.suptitle('LEICA-2: does the short-horizon improvement survive longer intervals?')
    fig.text(.05,.015,'Identical timestamps per horizon; no intervals crossing unsupported gaps. Shared-CPT7 IE-TC reference.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,.96));fig.savefig(out/'01_horizons.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
    for mode in MODES:
        k=data[mode+'/kimera'];f=data[mode+'/fusion_online']
        vals=[np.linalg.norm(k['position'],axis=1),k['angles'][:,2],np.linalg.norm(f['position'],axis=1)]
        for ax,v in zip(axes,vals):
            v=v.copy();v[~valid]=np.nan;ax.plot(time,v,color=COLORS[mode],label=NAMES[mode])
    for ax,label in zip(axes,['Kimera position drift (m)','Kimera heading error (deg)','ONLINE fusion position drift (m)']):
        ax.set_ylabel(label);ax.grid(alpha=.25);ax.axhline(0,color='gray',lw=.6)
    axes[0].legend(fontsize=9);axes[-1].set_xlabel('Seconds after capture start')
    fig.suptitle('LEICA-2: accumulated error with one fixed starting-pose alignment')
    fig.text(.05,.015,'Not whole-path best-fit ATE. Broken lines are excluded coverage; no re-alignment at a gap.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,.96));fig.savefig(out/'02_accumulated_drift.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
    for mode in MODES:
        a=data[mode+'/kimera']['angles'].copy();a[~valid]=np.nan
        for c,ax in enumerate(axes):ax.plot(time,a[:,c],color=COLORS[mode],label=NAMES[mode])
    for ax,label in zip(axes,['Roll difference (deg)','Pitch difference (deg)','Heading difference (deg)']):
        ax.set_ylabel(label);ax.grid(alpha=.25)
    axes[0].legend(fontsize=9);axes[-1].set_xlabel('Seconds after capture start')
    fig.suptitle('Kimera: signed orientation error after fixed-start alignment (CPT7 axes)')
    fig.tight_layout(rect=(0,.02,1,.96));fig.savefig(out/'03_orientation.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
    for mode in MODES[1:]:
        bins=peers[mode]['bins'];xx=[b['start_s']+15 for b in bins]
        axes[0].plot(xx,[b['summary']['incoming']['rotation_deg']['rms'] for b in bins],'-o',color=COLORS[mode],label=NAMES[mode]+' incoming error')
        axes[0].plot(xx,[b['summary']['applied_rotation_norm_deg']['rms'] for b in bins],'--',color=COLORS[mode],label=NAMES[mode]+' reported norm')
        axes[1].plot(xx,[b['summary']['incoming']['world_signed_mean_deg'][2] for b in bins],'-o',color=COLORS[mode],label=NAMES[mode])
        axes[2].plot(xx,[100*b['summary']['post_rotation_worse_fraction'] for b in bins],'-o',color=COLORS[mode],label=NAMES[mode])
    axes[0].set_ylabel('Rotation (deg)');axes[0].set_yscale('log');axes[0].legend(fontsize=8,ncol=2)
    axes[1].set_ylabel('Incoming signed world-Z\nrotation error mean (deg)')
    axes[2].set_ylabel('Post-update worse than pre (%)');axes[2].set_xlabel('Receiver-edge end time after capture start (s)')
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle('Actual GLIM → Kimera factors: error, applied uncertainty and update association')
    fig.text(.05,.012,'Fixed 30 s bins. All native/prior/CBS factors act together: pre/post is not an isolated causal test. Norm is sqrt(trace).',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.96));fig.savefig(out/'04_incoming_beliefs.png',dpi=150);plt.close(fig)


def main():
    out=H/'outputs';assert not out.exists(),'Refuse output overwrite'
    pin(Path(__file__));pin(H/'PROTOCOL.txt');pin(Path(g.__file__))
    checked=tests()
    ref,grid,valid,cover,metrics,data,crosscheck,roots=trajectories()
    peer={};records={}
    for mode in MODES[1:]:
        rr,checks=peer_records(roots[mode],mode,ref);records[mode]=rr
        peer[mode]=dict(checks=checks,all=peer_summary(rr),bins=[])
        for t in range(0,450,30):
            sub=[x for x in rr if t<=x['time_s']<t+30]
            if sub:peer[mode]['bins'].append(dict(start_s=t,end_s=t+30,summary=peer_summary(sub)))
    lookups={mode:{(x['a'],x['b']):x for x in rr} for mode,rr in records.items()}
    common=set.intersection(*(set(x) for x in lookups.values()))
    peer['paired']=dict(n=len(common),unmatched={m:len(rr)-len(common) for m,rr in records.items()},
                        summaries={m:peer_summary([lookups[m][k] for k in sorted(common)]) for m in MODES[1:]})
    # Verify unchanged original factor analysis agrees with independent saved CP-V5.
    old=json.loads(pin(H5/'outputs/peer_summary.json').read_text())
    baseline=old['G2K']['verified_pre_comparison']['rotation']
    assert abs(peer['original_on']['all']['incoming']['rotation_deg']['rms']-baseline['peer_error_rms'])<1e-10
    assert abs(peer['original_on']['all']['pre']['rotation_deg']['rms']-baseline['pre_error_rms'])<1e-10
    print('Rehash inputs and save bounded report data/figures',flush=True)
    assert all(g.sha(Path(p))==v for p,v in HASHES.items())
    out.mkdir(exist_ok=False)
    save(out/'metrics.json',dict(coverage=cover,trajectories=metrics,peers=peer,
         reference_max_factor_bracket_ns=ref.max_bracket_ns,score_crosscheck=crosscheck))
    save(out/'verification.json',dict(passed=True,tests=checked,source_input_hashes_unchanged=True,input_sha256=HASHES,
          original_factor_audit_reproduced=True,no_estimator_replay=True))
    arrays=dict(time_s=(grid-START)*1e-9,timestamp_ns=grid,valid=valid)
    for key,v in data.items():
        for name in ('position','rotation_world','angles'):arrays[key.replace('/','_')+'_'+name]=v[name]
        for h,(i,j,te,rr,ww) in v['series'].items():
            arrays[key.replace('/','_')+f'_rpe_{h}s']=np.c_[i,j,te,rr,ww]
    np.savez_compressed(out/'time_series.npz',**arrays)
    with gzip.open(out/'peer_records.jsonl.gz','wt') as f:
        for mode,rr in records.items():
            for x in rr:f.write(json.dumps(x,default=serial,allow_nan=False)+'\n')
    render(out,(grid-START)*1e-9,valid,metrics,data,peer)
    print(json.dumps(dict(coverage=cover,kimera={m:metrics[m+'/kimera'] for m in MODES}),default=serial,indent=2))


if __name__=='__main__':main()
