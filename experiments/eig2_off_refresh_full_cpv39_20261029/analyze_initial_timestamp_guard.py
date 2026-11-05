"""Saved-only CP-V39 verification and direct evo scoring; never launches ROS."""
import copy,csv,hashlib,importlib.util,json,os,re,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation as R
from evo.core import metrics,trajectory,filters
from evo.tools import file_interface
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eig2_off_refresh_full_cpv39_20261029';C=P/'capture'
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
H32=E/'all_evo_tables_cpv32_20261029'
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
m=module('cpv39_geometry',E/'newest_absolute_comparison_cpv27_20261028/analyze.py');a=m.a
u=module('cpv39_evo',E/'eth1_evo_rpe_distance_cpv31_20261029/evaluate.py')
v=module('cpv39_windows',E/'eig2_kimera_jitter_cpv34_20261029/analyze_windows.py')
epoch=1730638816563546034
ENV=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',MPLBACKEND='Agg')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(path,data):
    with Path(path).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
def stats(x):
    x=np.asarray(x,float);assert len(x) and np.isfinite(x).all()
    return dict(n=len(x),mean=float(x.mean()),median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(x.max()))
def poses(path):
    records=[];warnings=[]
    allowed={'WARNING: no messages received and simulated time is active.','Is /clock being published?'}
    with path.open() as f:
        reader=csv.DictReader(f);fields=reader.fieldnames
        for lineno,d in enumerate(reader,2):
            if d.get('%time') in allowed and all(x is None for k,x in d.items() if k!='%time'):
                warnings.append(dict(line=lineno,text=d['%time']));continue
            assert None not in d and all(x is not None for x in d.values()),(path,lineno)
            assert d['field.header.seq'].isdigit(),(path,lineno)
            records.append(d)
    return records,fields,warnings
def curve(ts,p,r):
    T=np.tile(np.eye(4),(len(ts),1,1));T[:,:3,:3]=r;T[:,:3,3]=p
    return trajectory.PoseTrajectory3D(poses_se3=list(T),timestamps=(ts-epoch)*1e-9)
commands=[]
def score(kind,refpath,estpath,out,relation):
    config=H32/'ape_config.json' if kind=='ape' else E/'eth1_evo_rpe_distance_cpv31_20261029/evo_config.json'
    cmd=[str(Path('/home/yeranis/.local/bin')/('evo_'+kind)),'tum',str(refpath),str(estpath),'-r',relation,'-c',str(config),'--save_results',str(out)]
    cmd+=['--align'] if kind=='ape' else ['-d','0.5','-u','m','--pairs_from_reference']
    assert not out.exists()
    with out.with_suffix('.log').open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=ENV,timeout=60)
    receipt=dict(command=cmd,returncode=r.returncode);save(out.with_suffix('.command.json'),receipt);commands.append(receipt)
    assert r.returncode==0,receipt
    return file_interface.load_res_file(out)
def export(path,tr):return u.export(path,tr)[0]

def window_summary(log,times,native,absolute):
    windows,_,_,_,_,counts=v.parse(log)
    events=[];missing=[];tdiff=[];rdiff=[]
    for k in range(1,len(times)):
        if k not in windows or k not in windows[k] or k-1 not in windows[k]:missing.append(k);continue
        pp,rr=windows[k][k-1];p,r=windows[k][k]
        oldp,oldr=windows.get(k-1,{}).get(k-1,(absolute[1][k-1],absolute[2][k-1]))
        assert np.linalg.norm(p-absolute[1][k])<.001
        ni=native[2][k-1].T@(native[1][k]-native[1][k-1]);nr=native[2][k-1].T@native[2][k]
        tdiff.append(np.linalg.norm(ni-rr.T@(p-pp)));rdiff.append(R.from_matrix(nr.T@rr.T@r).magnitude())
        keys=sorted(set(windows[k])&set(windows.get(k-1,{})))
        e=dict(k=k,time_s=float((times[k]-epoch)*1e-9),previous_revision_m=float(np.linalg.norm(pp-oldp)),absolute_step_m=float(np.linalg.norm(p-oldp)),same_update_motion_m=float(np.linalg.norm(p-pp)),revision_rotation_deg=float(np.rad2deg(R.from_matrix(oldr.T@rr).magnitude())))
        if len(keys)>2:
            old=np.array([windows[k-1][i][0] for i in keys]);new=np.array([windows[k][i][0] for i in keys])
            rold=np.array([windows[k-1][i][1] for i in keys]);rnew=np.array([windows[k][i][1] for i in keys])
            rot=R.from_matrix(rnew@np.swapaxes(rold,1,2)).mean().as_matrix();tr=np.mean(new-old@rot.T,axis=0)
            e['shape_max_m']=float(np.linalg.norm(new-(old@rot.T+tr),axis=1).max())
        events.append(e)
    assert max(tdiff)<1e-7 and max(rdiff)<1e-7
    out={key:stats([e[key] for e in events]) for key in ('previous_revision_m','absolute_step_m','same_update_motion_m','revision_rotation_deg')}
    out.update(rows=len(times),paired_steps=len(events),missing_step_ids=missing,MAIN_counts=counts,increment_parity_translation_max=float(max(tdiff)),increment_parity_rotation_max=float(max(rdiff)),revisions_over_half_m=sum(e['previous_revision_m']>.5 for e in events),revisions_over_one_m=sum(e['previous_revision_m']>1 for e in events),shape_max_m=stats([e['shape_max_m'] for e in events if 'shape_max_m' in e]))
    return out,events

def log_summary(log,costpath):
    timing=[];covs=[];first=active=0
    with log.open() as f:
        for line in f:
            if 'KIMERA_OPTIMIZE_TIMING_ROW,' in line:
                timing.append([float(x) for x in m.ANSI.sub('',line.split('KIMERA_OPTIMIZE_TIMING_ROW,',1)[1]).strip().split(',')])
            if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
                row=m.ANSI.sub('',line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1]).strip().split(',')
                assert len(row)==64;covs.append(np.array(row[28:],float).reshape(6,6))
                if row[7]=='first':first+=1
                elif row[7]=='active':active+=1
                else:raise AssertionError(row[7])
    tt=np.array(timing);cc=np.array(covs);cost=np.loadtxt(costpath,delimiter=',')
    assert np.isfinite(tt).all() and np.isfinite(cc).all() and np.isfinite(cost).all()
    assert np.array_equal(tt[:,0],np.arange(len(tt))) and np.all(tt[:,-1]==1)
    assert np.array_equal(cost[:,0],np.arange(len(cost)))
    sym=float(np.max(abs(cc-cc.transpose(0,2,1))));assert sym<1e-10
    eigen=np.linalg.eigvalsh(cc);assert eigen.min()>0
    return dict(optimize_ms=stats(tt[:,1]),smoother_ms=stats(tt[:,5]),rows=len(cost),post_solve_total_cost=stats(cost[:,5]),post_solve_imu_cost=stats(cost[:,7]),post_solve_imu_cost_after_250s=stats(cost[1250:,7]),native_prior_counter_max=int(cost[:,3].max()),covariance=dict(rows=len(cc),first=first,active=active,symmetry_max=sym,eigenvalue_min=float(eigen.min()),rot_trace_rad2=stats(np.trace(cc[:,:3,:3],axis1=1,axis2=2)),translation_trace_m2=stats(np.trace(cc[:,3:,3:],axis1=1,axis2=2)))),cost,tt

def main():
    assert (C/'capture_complete.json').exists() and (C/'playback_complete.json').exists()
    assert not (C/'failed.txt').exists() and not (H/'controller_failed.json').exists()
    cleanup=json.loads((C/'cleanup.json').read_text());assert not cleanup['errors'] and cleanup['kimera_process_gone']
    assert all(x is not None for x in cleanup['returncodes'].values())
    assert 'true' not in json.loads((H/'container_restoration.json').read_text())['states']
    rows,fields,warnings=poses(C/'odometry.log');oldrows,_,_=poses(S/'kimera_odometry.csv')
    assert len(rows)==len(oldrows)
    for x,y in zip(rows,oldrows):
        for k in ('field.header.seq','field.header.stamp','field.header.frame_id','field.child_frame_id'):assert x[k]==y[k],k
        assert np.isfinite([float(z) for k,z in x.items() if k not in ('field.header.frame_id','field.child_frame_id')]).all()
    derived=H/'derived';derived.mkdir()
    with (derived/'kimera_odometry.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fields);writer.writeheader();writer.writerows(rows)
    (derived/'estimators.log').symlink_to(C/'estimators.log')
    ng,_,checks=m.kimera(derived);bg,_,bchecks=m.kimera(S)
    nn=a.load_curve(derived/'kimera_odometry.csv');bn=a.load_curve(S/'kimera_odometry.csv')
    assert np.array_equal(ng[0],bg[0]) and np.array_equal(nn[0],bn[0])
    assert (ng[0][-1]-epoch)*1e-9>354
    curves={'baseline_method1':bg,'candidate_method1':ng,'baseline_method2':bn,'candidate_method2':nn}
    record=next(r for r in m.c.b.ROWS if r[0]=='velodyne_eig2');_,_,reference=m.c.b.load(record)
    refcurve=reference.times,np.array(reference.poses)[:,:3,3],np.array(reference.poses)[:,:3,:3]
    with np.load(E/'newest_absolute_comparison_cpv27_20261028/velodyne_eig2.npz') as z:base={k:z[k] for k in z.files}
    grid=base['timestamp_ns'];valid=base['common_valid'];ids=np.flatnonzero(valid);segments=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    folder=H/'evo';folder.mkdir();refpath=folder/'reference_common.tum'
    ref=export(refpath,curve(grid[ids],base['reference_position'][ids],base['reference_rotation'][ids]))
    outputs={};arrays={};full={};fits={};pair_records=[];ref_segments=[]
    for si,ii in enumerate(segments):
        target=folder/f'segment_{si}';target.mkdir();rp=target/'reference.tum'
        rt=export(rp,curve(grid[ii],base['reference_position'][ii],base['reference_rotation'][ii]));pairs=filters.filter_pairs_by_path(rt.poses_se3,.5,all_pairs=False)
        ref_segments.append((rt,rp,pairs))
        for i,j in pairs:pair_records.append(dict(segment=si,start_timestamp_ns=int(grid[ii[i]]),end_timestamp_ns=int(grid[ii[j]])))
    oldpairs=json.loads((H32/'velodyne_eig2/pair_indices.json').read_text())
    assert pair_records==[{k:x[k] for k in ('segment','start_timestamp_ns','end_timestamp_ns')} for x in oldpairs]
    oldmetrics=json.loads((H32/'velodyne_eig2.json').read_text())['results']
    for name,(ts,p,r) in curves.items():
        pp,rr,good=a.sample((ts,p,r),grid,500_000_000);assert good[valid].all(),name
        ep,er=a.prism(pp,rr);arrays[name+'_position']=ep;arrays[name+'_rotation']=er
        if name.startswith('baseline'):
            method=name.split('_')[1];source=u.SOURCES[method]
            with np.load(source/'velodyne_eig2.npz') as z:
                key='off_kimera_absolute' if method=='method1' else 'off_kimera'
                assert np.array_equal(ep,z[key+'_position']) and np.array_equal(er,z[key+'_rotation'])
        estpath=folder/(name+'_common.tum');est=export(estpath,curve(grid[ids],ep[ids],er[ids]))
        ar=score('ape',refpath,estpath,folder/(name+'_ape.zip'),'trans_part')
        aligned=copy.deepcopy(est);rot,trans,scale=aligned.align(ref,correct_scale=False);assert scale==1
        am=metrics.APE(metrics.PoseRelation.translation_part);am.process_data((ref,aligned));assert np.max(abs(am.error-ar.np_arrays['error_array']))<1e-8
        fits[name]=dict(rotation=rot.tolist(),translation=trans.tolist(),scale=scale)
        arrays[name+'_aligned_position']=ep@rot.T+trans
        pathlength=0;errors={key:[] for key in u.RELATIONS}
        for si,ii in enumerate(segments):
            sp=folder/f'segment_{si}'/(name+'.tum');st=export(sp,curve(grid[ii],ep[ii],er[ii]));pathlength+=st.path_length
            rt,rp,pairs=ref_segments[si]
            if not pairs:continue
            for key,(relation,rel) in u.RELATIONS.items():
                result=score('rpe',rp,sp,sp.with_name(name+'_'+key+'.zip'),relation)
                rm=u.new_metric(rel);rm.process_data((rt,st));assert np.max(abs(rm.error-result.np_arrays['error_array']))<1e-8
                errors[key].append(result.np_arrays['error_array'])
        row=dict(ATE_RMSE_m=ar.stats['rmse'],path_length_m=pathlength,RPE={})
        for key,(_,rel) in u.RELATIONS.items():
            rm=u.new_metric(rel);rm.error=np.concatenate(errors[key]);assert len(rm.error)==len(pair_records)
            result=rm.get_result('IE-TC',name);out=folder/(name+'_'+key+'_pooled.zip');file_interface.save_res_file(out,result)
            row['RPE'][key]=dict(statistics=result.stats,pairs=len(rm.error))
        outputs[name]=row
        if name.startswith('baseline'):
            previous=oldmetrics[name.split('_')[1]+'_off_kimera'];assert abs(row['ATE_RMSE_m']-previous['ATE_RMSE_m'])<1e-8
            assert abs(pathlength-previous['path_length_m'])<1e-8
            for key in u.RELATIONS:assert abs(row['RPE'][key]['statistics']['rmse']-previous['RPE'][key]['statistics']['rmse'])<1e-8
        gp,gr,good=a.sample(refcurve,ts,50_000_000);assert good.all();gp,gr=a.prism(gp,gr);fp,fr=a.prism(p,r)
        rp=folder/(name+'_full_reference.tum');sp=folder/(name+'_full_native.tum')
        rf=export(rp,curve(ts,gp,gr));sf=export(sp,curve(ts,fp,fr));frs=score('ape',rp,sp,folder/(name+'_full_ape.zip'),'trans_part')
        cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        no_gap_length=sum(curve(ts[ii],fp[ii],fr[ii]).path_length for ii in cuts)
        full[name]=dict(ATE_RMSE_m=frs.stats['rmse'],native_polyline_length_m=sf.path_length,no_gap_bridging_length_m=no_gap_length,reference_native_polyline_length_m=rf.path_length,rows=len(ts),first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),gap_chords=len(cuts)-1)
        print(name,json.dumps(row),flush=True)
    window={};events={};logs={};costs={};timing={}
    for name,src,ab,nat in [('baseline',S,bg,bn),('candidate',C,ng,nn)]:
        window[name],events[name]=window_summary(src/'estimators.log',ab[0],nat,ab)
        logs[name],costs[name],timing[name]=log_summary(src/'estimators.log',src/'objective.csv')
        assert logs[name]['rows']==len(rows)
    headers=list(csv.DictReader((C/'topic_headers.csv').open()));topics={}
    for topic in sorted({x['topic'] for x in headers}):
        rr=[x for x in headers if x['topic']==topic];stamps=np.array([int(x['stamp_ns']) for x in rr],np.int64)
        assert np.all(np.diff(stamps)>0)
        topics[topic]=dict(rows=len(rr),first_ns=int(stamps[0]),last_ns=int(stamps[-1]),maximum_gap_s=float(np.diff(stamps).max()*1e-9),receipt_sensor_delay_s=stats([(int(x['receipt_ros_ns'])-int(x['stamp_ns']))*1e-9 for x in rr]))
    for side in ('left','right'):
        raw=[x['stamp_ns'] for x in headers if x['topic']==f'/grandtour/front_{side}/image_raw']
        comp=[x['stamp_ns'] for x in headers if x['topic']==f'/boxi/alphasense/front_{side}/image_raw/compressed'];assert raw==comp
    assert topics['/kimera_vio_ros/odometry']['rows']==len(rows)
    arrays.update(timestamp_ns=grid,common_valid=valid,reference_position=base['reference_position'],reference_rotation=base['reference_rotation'])
    np.savez_compressed(H/'comparison.npz',**arrays)
    save(H/'window_events.json',events);save(H/'evo_commands.json',commands);save(H/'pair_indices.json',pair_records)
    coverage=json.loads((E/'newest_absolute_comparison_cpv27_20261028/velodyne_eig2.json').read_text())['common_coverage']
    result=dict(passed=True,source_runs=1,source_retries=0,full_capture_s=355.02,common_coverage=coverage,reference_common_length_m=sum(x[0].path_length for x in ref_segments),common=outputs,full_saved=full,window=window,logs=logs,topic_headers=topics,absolute_pose_checks=checks,nondata_warnings=warnings,alignment=fits,evo_version='1.36.3',evo_cli_calls=len(commands),reference='IE-TC shared CPT7, not independent GT',prior_audit_limitation='Legacy prior-only counters omit OriginalChartPrior; total includes it.',cleanup=cleanup)
    save(H/'results.json',result)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
    for name,color in [('baseline','#cc6e27'),('candidate','#167ca1')]:
        key=name+'_method1';pos=arrays[key+'_aligned_position'];err=np.linalg.norm(pos-base['reference_position'],axis=1)
        for ii in segments:axs[0].plot((grid[ii]-epoch)*1e-9,err[ii],color=color,label=name if ii[0]==segments[0][0] else None,linewidth=.8)
        axs[1].plot([x['time_s'] for x in events[name]],[x['previous_revision_m'] for x in events[name]],color=color,label=name,linewidth=.8)
        axs[2].semilogy((ng[0]-epoch)*1e-9,np.maximum(costs[name][:,7],1e-12),color=color,label=name,linewidth=.8)
    axs[0].set_ylabel('Method1 aligned error (m)');axs[1].set_ylabel('Previous-state revision (m)');axs[2].set_ylabel('Native IMU factor cost');axs[2].set_xlabel('Sensor time from original start (s)')
    for ax in axs:ax.grid(alpha=.25);ax.legend()
    fig.suptitle('EIG-2 CBS-OFF: unchanged weights, private current-state refresh\nError plot uses the exact EVO SE3 fit; internal costs are not accuracy metrics')
    fig.tight_layout();fig.savefig(H/'comparison.png',dpi=150);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(14,7))
    for ax,method in zip(axs,('method1','method2')):
        for name,color in [('reference','#406db3'),('baseline','#d18b23'),('candidate','#c93750')]:
            pos=base['reference_position'] if name=='reference' else arrays[name+'_'+method+'_aligned_position']
            for si,ii in enumerate(segments):ax.plot(pos[ii,0],pos[ii,1],color=color,linewidth=1,label=name if si==0 else None)
        ax.set_title(method+': identical common-mask samples');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();ax.set_xlabel('Reference-frame x (m)');ax.set_ylabel('Reference-frame y (m)')
    fig.suptitle('EIG-2 Kimera CBS-OFF: baseline versus current-state refresh\nIndependent EVO SE3 fit per curve, no scale correction or smoothing')
    fig.tight_layout();fig.savefig(H/'trajectories.png',dpi=150);plt.close(fig)
    print('CP-V39 saved analysis complete',flush=True)
if __name__=='__main__':main()
