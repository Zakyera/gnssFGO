"""CP-V80 saved-only Method1 direct EVO report; never runs an estimator."""
import copy,csv,hashlib,importlib.metadata,importlib.util,json,os,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import evo
from evo.core import filters,metrics,trajectory
from evo.tools import file_interface
H=Path(__file__).resolve().parent;E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
V=Path('/home/yeranis/results')/H.name;O=H/'evaluation'
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
m=module('cpv80_absolute',E/'newest_absolute_comparison_cpv27_20261028/analyze.py');a=m.a
u=module('cpv80_evo',E/'eth1_evo_rpe_distance_cpv31_20261029/evaluate.py')
d39=module('cpv80_revision',E/'eig2_off_refresh_full_cpv39_20261029/analyze.py')
ENV=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',MPLBACKEND='Agg')
epoch=1730638816563546034;duration=355.02;PINS={};commands=[]
checks=dict(TUM_roundtrip_max=0.,CLI_API_error_max=0.)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
    p=Path(p).resolve();PINS[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def save(p,v):
    with Path(p).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def curve(ts,p,r):
    poses=np.tile(np.eye(4),(len(ts),1,1));poses[:,:3,3]=p;poses[:,:3,:3]=r
    return trajectory.PoseTrajectory3D(poses_se3=list(poses),timestamps=(ts-epoch)*1e-9)
def export(p,tr):
    out,diff=u.export(p,tr);checks['TUM_roundtrip_max']=max(checks['TUM_roundtrip_max'],diff);return out
def cli(kind,rp,ep,out,relation):
    cfg=E/'all_evo_tables_cpv32_20261029/ape_config.json' if kind=='ape' else E/'eth1_evo_rpe_distance_cpv31_20261029/evo_config.json'
    pin(cfg)
    cmd=[str(Path('/home/yeranis/.local/bin')/('evo_'+kind)),'tum',str(rp),str(ep),'-r',relation,'-c',str(cfg),'--save_results',str(out)]
    cmd+=['--align'] if kind=='ape' else ['-d','0.5','-u','m','--pairs_from_reference']
    assert not out.exists()
    with out.with_suffix('.log').open('x') as log:
        done=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=ENV,timeout=60)
    receipt=dict(command=cmd,exit=done.returncode);save(out.with_suffix('.command.json'),receipt);commands.append(receipt)
    assert done.returncode==0,receipt
    return file_interface.load_res_file(out)
def check_result(result,metric):
    assert result.np_arrays['error_array'].shape==metric.error.shape
    diff=float(np.max(abs(result.np_arrays['error_array']-metric.error)))
    checks['CLI_API_error_max']=max(checks['CLI_API_error_max'],diff);assert diff<1e-8
    for k,v in metric.get_all_statistics().items():assert abs(result.stats[k]-v)<1e-8,k
def reference_segments(folder,ts,gp,gr,segments):
    refs=[];pairs_all=[];info=[]
    for si,ii in enumerate(segments):
        d=folder/f'segment_{si}';d.mkdir();rp=d/'reference.tum';rt=export(rp,curve(ts[ii],gp[ii],gr[ii]))
        pairs=filters.filter_pairs_by_path(rt.poses_se3,.5,all_pairs=False) if len(ii)>1 else []
        refs.append((rt,rp,pairs))
        info.append(dict(segment=si,rows=len(ii),pairs=len(pairs),reference_path_m=rt.path_length,
                         first_s=float((ts[ii[0]]-epoch)*1e-9),last_s=float((ts[ii[-1]]-epoch)*1e-9)))
        for i,j in pairs:pairs_all.append(dict(segment=si,start_ns=int(ts[ii[i]]),end_ns=int(ts[ii[j]]),reference_distance_m=float(rt.distances[j]-rt.distances[i])))
    save(folder/'pair_indices.json',pairs_all)
    return refs,pairs_all,info
def score_series(folder,name,ts,ep,er,gp,gr,segments,refs,refpath,ref):
    ids=np.concatenate(segments);sp=folder/(name+'.tum');est=export(sp,curve(ts[ids],ep[ids],er[ids]))
    result=cli('ape',refpath,sp,folder/(name+'_ape.zip'),'trans_part')
    aligned=copy.deepcopy(est);rot,tr,scale=aligned.align(ref,correct_scale=False);assert scale==1
    am=metrics.APE(metrics.PoseRelation.translation_part);am.process_data((ref,aligned));check_result(result,am)
    errors={k:[] for k in u.RELATIONS};path=0.;paircount=0
    for si,ii in enumerate(segments):
        d=folder/f'segment_{si}';sp=d/(name+'.tum');st=export(sp,curve(ts[ii],ep[ii],er[ii]));path+=st.path_length
        rt,rp,pairs=refs[si];paircount+=len(pairs)
        if not pairs:continue
        for key,(text,relation) in u.RELATIONS.items():
            result_r=cli('rpe',rp,sp,d/(name+'_'+key+'.zip'),text)
            rm=u.new_metric(relation);rm.process_data((rt,st));assert rm.delta_ids==[j for i,j in pairs]
            check_result(result_r,rm);errors[key].append(result_r.np_arrays['error_array'])
    rpe={}
    for key,(_,relation) in u.RELATIONS.items():
        rm=u.new_metric(relation);rm.error=np.concatenate(errors[key]);assert len(rm.error)==paircount
        rr=rm.get_result('IE-TC reference',name);dest=folder/(name+'_'+key+'_pooled.zip')
        file_interface.save_res_file(dest,rr);loaded=file_interface.load_res_file(dest)
        assert np.array_equal(loaded.np_arrays['error_array'],rm.error)
        rpe[key]=dict(statistics=rr.stats,pairs=paircount,archive=str(dest))
    row=dict(ATE_RMSE_m=result.stats['rmse'],path_length_m=path,RPE=rpe,rows=len(ids),
             reference_path_length_m=sum(x[0].path_length for x in refs),alignment=dict(rotation=rot.tolist(),translation=tr.tolist(),scale=scale))
    print(name,json.dumps({k:row[k] for k in ('ATE_RMSE_m','path_length_m')}),flush=True)
    return row
def covariance_summary(events):
    matrices={};rows={}
    with pin(events).open() as f:
        for r in csv.reader(f):
            if r[0]!='B':continue
            assert len(r)==52
            matrices.setdefault(r[2],[]).append([float(x) for x in r[16:]])
            rows[r[2]]=rows.get(r[2],0)+1
    result={}
    for source,values in matrices.items():
        c=np.asarray(values).reshape(-1,6,6);assert np.isfinite(c).all()
        sym=float(np.max(abs(c-c.transpose(0,2,1))));ev=np.linalg.eigvalsh(c)
        assert sym<1e-10 and ev.min()>0
        result[source]=dict(rows=rows[source],symmetry_max=sym,eigenvalue_min=float(ev.min()),
            rotation_trace_rad2=d39.stats(np.trace(c[:,:3,:3],axis1=1,axis2=2)),
            translation_trace_m2=d39.stats(np.trace(c[:,3:,3:],axis1=1,axis2=2)))
    return result
def main():
    assert importlib.metadata.version('evo')=='1.36.3'
    assert not O.exists() and not (H/'results.json').exists()
    assert read(H/'capture_complete.json')['passed'] and read(H/'closeout.json')['passed']
    assert not (H/'failed.json').exists() and not (H/'cleanup_failed.json').exists()
    a.HASHES.clear()
    for mod in (m,m.c,m.c.b,a,a.g,u,d39,d39.v):pin(mod.__file__)
    for f in (Path(__file__),H/'PROTOCOL.txt',H/'on_validated.json'):pin(f)
    for f in Path(evo.__file__).parent.rglob('*.py'):pin(f)
    for kind in ('ape','rpe'):pin(Path('/home/yeranis/.local/bin')/('evo_'+kind))
    record=next(r for r in m.c.b.ROWS if r[0]=='hesai_eig2')
    _,ev,refobj=m.c.b.load(record)
    reference=(refobj.times,np.asarray(refobj.poses)[:,:3,3],np.asarray(refobj.poses)[:,:3,:3])
    curves={};kimera_checks={};diagnostic={};native_inc={}
    for mode in ('on',):
        src=P/f'full_{mode}_source';rec=P/f'full_{mode}_receiver/receiver'
        effective=read(src/'effective_mode_check.json');assert effective['passed'] and effective['kimera_candidate_env']['CPV40_REFRESH']=='1'
        assert effective['glim']['glim_cbs_receive_external_beliefs'] is (mode=='on')
        curves[mode+'_glim']=a.load_curve(src/'glim_odometry.csv');m.c.check_csv(src/'glim_odometry.csv')
        curves[mode+'_kimera'],_,kimera_checks[mode]=m.kimera(src)
        native_inc[mode]=a.load_curve(src/'kimera_odometry.csv')
        curves[mode+'_fusion_online']=a.load_curve(rec/'online_trajectory.tum')
        curves[mode+'_fusion_delayed']=a.load_curve(rec/'finalized_trajectory.tum')
        validation=read(V/mode/'validation.json');assert validation['passed']
        diagnostic[mode]=dict(receiver=validation,source=read(src/'candidate_validation.json'),
            local=read(src/'local_provenance.json'),sensor=read(src/'hesai_validation.json'))
    primary=[mode+'_'+role for mode in ('on',) for role in ('glim','kimera','fusion_online')]
    start=max(curves[k][0][0] for k in primary);end=min(curves[k][0][-1] for k in primary)
    grid=np.arange(((start+a.STEP-1)//a.STEP)*a.STEP,(end//a.STEP)*a.STEP+1,a.STEP,dtype=np.int64)
    gp,gr,valid=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr);sampled={};supported={}
    for key,c in curves.items():
        pp,rr,good=a.sample(c,grid,500_000_000);sampled[key]=a.prism(pp,rr);supported[key]=good
        if key in primary:valid &= good
    for key in curves:assert supported[key][valid].all(),key
    ii=np.flatnonzero(valid);segments=np.split(ii,np.flatnonzero(np.diff(ii)>1)+1)
    O.mkdir();common=O/'common';common.mkdir();rp=common/'reference.tum';rt=export(rp,curve(grid[ii],gp[ii],gr[ii]))
    refs,pairs,info=reference_segments(common,grid,gp,gr,segments)
    outputs={};full={};arrays=dict(timestamp_ns=grid,common_valid=valid,reference_position=gp,reference_rotation=gr)
    for key,(ep,er) in sampled.items():
        outputs[key]=score_series(common,key,grid,ep,er,gp,gr,segments,refs,rp,rt)
        arrays[key+'_position']=ep;arrays[key+'_rotation']=er
        ts,pp,rr=curves[key];fp,fr=a.prism(pp,rr)
        fg,fgr,good=a.sample(reference,ts,50_000_000);assert good.all(),key
        fg,fgr=a.prism(fg,fgr);folder=O/('full_'+key);folder.mkdir()
        cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        rpf=folder/'reference.tum';rtf=export(rpf,curve(ts,fg,fgr))
        rfs,prs,inf=reference_segments(folder,ts,fg,fgr,cuts)
        full[key]=score_series(folder,key,ts,fp,fr,fg,fgr,cuts,rfs,rpf,rtf)
        full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),span_s=float((ts[-1]-ts[0])*1e-9),
          native_polyline_including_gap_chords_m=curve(ts,fp,fr).path_length,
          reference_native_polyline_including_gap_chords_m=rtf.path_length,
          gap_chords=len(cuts)-1,coverage_supported_s=sum(float((ts[x[-1]]-ts[x[0]])*1e-9) for x in cuts),segments=inf,
          RPE_note='Native-cadence reference-distance pairs differ by output; common table owns matched comparison.')
    save(O/'metric_results.json',dict(common=outputs,full_saved=full,coverage=m.c.coverage(grid,valid,epoch,duration),segments=info))
    revision_events={}
    for mode in ('on',):
        src=P/f'full_{mode}_source';pin(src/'objective.csv')
        absolute=curves[mode+'_kimera']
        diagnostic[mode]['revisions'],revision_events[mode]=d39.window_summary(src/'estimators.log',absolute[0],native_inc[mode],absolute)
        diagnostic[mode]['source_logs'],_,_=d39.log_summary(src/'estimators.log',src/'objective.csv')
        diagnostic[mode]['transported_covariance']=covariance_summary(P/f'full_{mode}_receiver/events.csv')
    for p,digest in a.HASHES.items():PINS[p]=digest
    assert all(sha(p)==digest for p,digest in PINS.items())
    save(O/'revision_events.json',revision_events);save(O/'diagnostics.json',diagnostic)
    save(O/'commands.json',commands);save(O/'input_sha256.json',PINS)
    np.savez_compressed(O/'comparison.npz',**arrays)
    coverage=m.c.coverage(grid,valid,epoch,duration)
    result=dict(passed=True,task='CP-V80',evo_version=importlib.metadata.version('evo'),method='Method1 first/newest absolute publication',
      capture_s=duration,epoch_ns=epoch,reference='IE-TC shares CPT7, not independent prism ground truth',
      physical_point='Existing common CPT7-to-prism rigid transform applied to all outputs and reference',
      receiver_Qc_multiplier=1.,common_coverage=coverage,common=outputs,full_saved=full,
      reference_common_path_m=sum(x[0].path_length for x in refs),RPE_pairs=len(pairs),segments=info,
      kimera_absolute_reconstruction_checks=kimera_checks,diagnostics=diagnostic,verification=checks,
      new_evo_CLI_calls=len(commands),source_runs_this_reporting_stage=0,receiver_replays=0,
      live_bidirectional=True,kimera_factor_mode='persistent',glim_factor_mode='active_window_temporary',
      kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True,
      no_fresh_matched_off_control=True)
    save(H/'results.json',result)
    lines=['CP-V80 FULL LIVE EIG-2 HESAI — DIRECT EVO METHOD1 RESULTS',
      'ONE full355.02s CBS-ON live capture: fresh GLIM, general-persistent Kimera, concurrent fusion.',
      'Live bidirectional source exchange. GLIM remains active_window_temporary; CP76 persistent Kimera.',
      'User explicitly accepted mixed-prior feedback: Kimera outgoing LOCAL is NOT native-only information.',
      'Absorbed GLIM information can return to GLIM/fusion. No independence or calibrated-likelihood claim.',
      'No fresh matched OFF control. CP49/CP78 are historical evidence, not this run\'s paired controls.',
      'Original EIG2 WNOA Qc x1.0 and all other frozen weights/profile/calibration. No tuning/default change.',
      'Method1: GLIM first-published absolute; Kimera newest absolute backend; fusion ONLINE absolute.',
      'Kimera absolute log rows matched to exact native header stamps; not native incremental ROS path.',
      'IE-TC reference shares CPT7 IMU; not independent prism GT. Same physical prism point.',
      'ATE: EVO SE3 alignment per curve, no scale. RPE: EVO reference-distance0.5m consecutive pairs.',
      'Common100ms grid; maximum interpolation gap0.5s (reference0.05s); no bridging excluded intervals.',
      'Supported %.3fs / %.2fs (%.3f%%), %d queries; %d common RPE pairs.'%(coverage['supported_s'],duration,coverage['supported_capture_percent'],coverage['valid_queries'],len(pairs)),
      'Reference path on common support: %.6fm'%result['reference_common_path_m'],'',
      'Role | ATE m | Estimated path m | Translation RPE m | Rotation RPE deg']
    for role in ('glim','kimera','fusion_online','fusion_delayed'):
        r=outputs['on_'+role]
        lines.append('%s | %.6f | %.4f | %.6f | %.6f'%(role,r['ATE_RMSE_m'],r['path_length_m'],r['RPE']['translation_m']['statistics']['rmse'],r['RPE']['rotation_deg']['statistics']['rmse']))
    lines+=['','fusion_delayed is DELAYED seven-second fixed-lag-finalized, NOT ONLINE or full-history.',
      'Path is estimated polyline on common support, not physical distance actually travelled.',
      '', 'FULL SAVED (native cadence; different coverage/pair samples, not common-table substitutes)',
      'Product | ATE m | no-gap path m | polyline with gap chords m | rows | first/last seconds | gaps']
    for key,r in full.items():lines.append('%s | %.6f | %.4f | %.4f | %d | %.4f / %.4f | %d'%(key,r['ATE_RMSE_m'],r['path_length_m'],r['native_polyline_including_gap_chords_m'],r['rows'],r['first_s'],r['last_s'],r['gap_chords']))
    lines+=['','RETAINED ENGINEERING EVIDENCE (not nonlinear stationarity/calibrated uncertainty)']
    for mode,d in diagnostic.items():
        sm=d['receiver']['summary'];rev=d['revisions'];src=d['source']
        lines+=['%s: %d native Kimera / %d GLIM / %d ONLINE+%d finalized; %d inserts/%d replacements/%d pre-init exclusions/%d terminal omissions/%d pending.'%(mode,src['native_updates'],d['sensor']['published'],sm['publications'],sm['finalized'],sm['inserted'],sm['replaced'],sm['excluded'],sm['unsupported_end'],sm['pending_at_end']),
          '  Kimera previous-state revision max %.6fm; >0.5m=%d; >1m=%d; observed paired steps=%d, missing IDs=%s.'%(rev['previous_revision_m']['maximum'],rev['revisions_over_half_m'],rev['revisions_over_one_m'],rev['paired_steps'],rev['missing_step_ids']),
          '  Native IMU cost median %.6g / max %.6g; not unwhitened residual or ATE.'%(src['imu_factor_cost']['median'],src['imu_factor_cost']['max']),
          '  Receiver covariance payload error=%g; support service p95 %.6fs; receipt-processing p95 %.6fs.'%(d['receiver']['covariance_payload_error'],d['receiver']['support_cycle_seconds']['p95'],d['receiver']['receipt_to_processing_seconds']['p95'])]
        for source,cov in d['transported_covariance'].items():
            lines.append('  %s: %d transported versions, median tr(Crot) %.9g rad2 / tr(Ctrans) %.9g m2; min eigenvalue %.9g.'%(source,cov['rows'],cov['rotation_trace_rad2']['median'],cov['translation_trace_m2']['median'],cov['eigenvalue_min']))
    lines+=['','All raw logs, journals, source/receiver outputs and covariance histories retained.',
      'No stale/duplicate/expired receiver versions. Source mean-only covariance alignment approximation unchanged.',
      'Do not compare old ON/new OFF as a matched algorithm pair. Historical tables are preserved.',
      'Protected files and Hesai667 passed controller closeout; both containers restored; see closeout.json.',
      'EVO CLI/API parity max %.3g; TUM roundtrip max %.3g; %d direct EVO CLI invocations.'%(checks['CLI_API_error_max'],checks['TUM_roundtrip_max'],len(commands)),
      'See results.json, evaluation/commands.json, pair_indices.json, input_sha256.json and all EVO ZIPs.']
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    print('\n'.join(lines[:20]),flush=True)
if __name__=='__main__':main()
