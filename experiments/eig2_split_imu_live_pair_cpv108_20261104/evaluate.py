"""Saved-only fresh-pair Method1 EVO scoring. No estimator or receiver execution."""
import importlib.metadata,importlib.util,json,sys
from pathlib import Path
import numpy as np
from paths import H,E,W,P,V,ORIGINAL
O=H/'evaluation'
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('cp95_saved_helpers',E/'eth1_alpha_bias_rw_off60_cpv88_20261102/evaluate.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b);d=b.d;a=d.a
audit=json.loads((P/'input_audit.json').read_text());epoch=audit['epoch_ns'];duration=audit['duration_s']
d.epoch=epoch;d.d39.epoch=epoch
def physical(c,body):
    ts,p,r=c;T=np.asarray(audit['T_'+body+'_prism'])
    return ts,p+np.einsum('nij,j->ni',r,T[:3,3]),r@T[:3,:3]
def reference_files(folder,ts,gp,gr,cuts):
    folder.mkdir();ids=np.concatenate(cuts);rp=folder/'reference.tum'
    rt=d.export(rp,d.curve(ts[ids],gp[ids],gr[ids]))
    refs,pairs,info=d.reference_segments(folder,ts,gp,gr,cuts)
    return rp,rt,refs,pairs,info
def load_reference():
    from scipy.spatial.transform import Rotation
    sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
    from rosbags.highlevel import AnyReader
    path=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag'
    ts=[];pp=[];qq=[]
    with AnyReader([d.pin(path)]) as reader:
        for conn,_,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/boxi/inertial_explorer/tc/odometry']):
            msg=reader.deserialize(raw,conn.msgtype);v=msg.pose.pose
            ts.append(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec))
            pp.append([v.position.x,v.position.y,v.position.z]);qq.append([v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
    ts=np.array(ts,np.int64);assert np.all(np.diff(ts)>0)
    return physical((ts,np.array(pp),Rotation.from_quat(qq).as_matrix()),'cpt7')

def main():
    assert importlib.metadata.version('evo')=='1.36.3' and duration==355.02
    assert d.read(H/'capture_complete.json')['passed'] and d.read(H/'closeout.json')['passed']
    assert not O.exists() and not (H/'results.json').exists()
    for mod in (b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(mod.__file__)
    for p in (Path(__file__),H/'PROTOCOL.txt',ORIGINAL/'profile_parity.json',P/'input_audit.json',ORIGINAL/'live_adapter_test.json'):d.pin(p)
    reference=load_reference()
    curves={};native={};diagnostic={}
    for mode in ('off','on'):
        S=P/f'full_{mode}_source';R=P/f'full_{mode}_receiver'
        assert d.read(H/(mode+'_validated.json'))['passed']
        native[mode]=b.kimera(S,'alphasense')
        curves[mode+'_glim']=physical(a.load_curve(S/'glim_odometry.csv'),'cpt7')
        curves[mode+'_kimera']=physical(native[mode],'alphasense')
        curves[mode+'_fusion_online']=physical(a.load_curve(R/'receiver/online_trajectory.tum'),'cpt7')
        curves[mode+'_fusion_delayed']=physical(a.load_curve(R/'receiver/finalized_trajectory.tum'),'cpt7')
        diagnostic[mode]=dict(source=d.read(H/f'{mode}_candidate_validation.json'),local=d.read(H/f'{mode}_local_provenance.json'),
            frame=d.read(H/f'frame_{mode}.json'),receiver=d.read(V/mode/'validation.json'),
            transported_covariance=d.covariance_summary(R/'events.csv'))
    primary=[f'{mode}_{role}' for mode in ('off','on') for role in ('glim','kimera','fusion_online')]
    first=max(curves[k][0][0] for k in primary);last=min(curves[k][0][-1] for k in primary)
    grid=np.arange(((int(first)+99_999_999)//100_000_000)*100_000_000,int(last)+1,100_000_000,dtype=np.int64)
    gp,gr,valid=a.sample(reference,grid,50_000_000);samples={};support={}
    for key,c in curves.items():
        p,r,good=a.sample(c,grid,500_000_000);samples[key]=(p,r);support[key]=good
        if key in primary:valid &= good
    ids=np.flatnonzero(valid);assert len(ids)>100
    assert all(support[k][valid].all() for k in curves),'DELAYED must cover primary mask without reducing it'
    cuts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1);O.mkdir()
    folder=O/'common';rp,rt,refs,pairs,info=reference_files(folder,grid,gp,gr,cuts)
    common={};full={};arrays=dict(timestamp_ns=grid,common_valid=valid,reference_position=gp,reference_rotation=gr)
    for key,(p,r) in samples.items():
        common[key]=d.score_series(folder,key,grid,p,r,gp,gr,cuts,refs,rp,rt)
        arrays[key+'_position']=p;arrays[key+'_rotation']=r
        ts,fp,fr=curves[key];fg,fgr,good=a.sample(reference,ts,50_000_000);assert good.all(),key
        segments=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        ff=O/('full_'+key);rpf,rtf,rfs,prs,inf=reference_files(ff,ts,fg,fgr,segments)
        full[key]=d.score_series(ff,key,ts,fp,fr,fg,fgr,segments,rfs,rpf,rtf)
        supported=sum(float((ts[x[-1]]-ts[x[0]])*1e-9) for x in segments)
        full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
            supported_s=supported,coverage_percent=100*supported/duration,gaps=len(segments)-1,segments=inf,
            native_polyline_including_gap_chords_m=d.curve(ts,fp,fr).path_length,
            reference_polyline_including_gap_chords_m=rtf.path_length)
    revision_events={}
    for mode in ('off','on'):
        S=P/f'full_{mode}_source'
        revision,events=d.d39.window_summary(S/'estimators.log',native[mode][0],b.native_csv(S/'kimera_odometry.csv'),native[mode])
        revision_events[mode]=events;timing=[];cov=[]
        for line in d.pin(S/'estimators.log').open():
            for marker,dest in [('KIMERA_OPTIMIZE_TIMING_ROW,',timing),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
                if marker in line:dest.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
        times=np.asarray(timing,float);mat=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
        assert np.isfinite(mat).all() and np.linalg.eigvalsh(mat).min()>0
        diagnostic[mode].update(revisions=revision,kimera_optimize_ms=d.d39.stats(times[:,1]),
            native_kimera_covariance=dict(body='alphasense_imu',rows=len(mat),
            rotation_trace_rad2=d.d39.stats(np.trace(mat[:,:3,:3],axis1=1,axis2=2)),
            translation_trace_m2=d.d39.stats(np.trace(mat[:,3:,3:],axis1=1,axis2=2))))
        for p in (S/'glim_odometry.csv',S/'kimera_odometry.csv',S/'objective.csv'):d.pin(p)
    coverage=d.m.c.coverage(grid,valid,epoch,duration)
    improvement={role:100*(common['off_'+role]['ATE_RMSE_m']-common['on_'+role]['ATE_RMSE_m'])/common['off_'+role]['ATE_RMSE_m']
                 for role in ('glim','kimera','fusion_online','fusion_delayed')}
    result=dict(passed=True,task='CP-V108',evo_version='1.36.3',capture_s=duration,method='Method1 first absolute publication',
        common=common,full_saved=full,common_coverage=coverage,common_RPE_pairs=len(pairs),common_segments=info,
        ATE_improvement_percent=improvement,diagnostics=diagnostic,verification=d.checks,
        new_evo_CLI_calls=len(d.commands),fresh_matched_off_on=True,receiver_Qc_multiplier=1.,
        live_bidirectional_ON=True,both_persistent_ON=True,source_runs_this_reporting_stage=0,receiver_replays=0,
        kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True,
        physical_point='AP20 prism; Alpha/CPT7 rigid body transforms from author calibration',
        reference='IE-TC shares CPT7, not independent prism ground truth',
        omissions=dict(raw_unmatched_camera_headers=audit['raw_unmatched_camera_headers'],
            source_max_interpolation_gap_s=.5,reference_max_gap_s=.05,delayed='seven-second fixed-lag-finalized, not ONLINE or full-history',
            covariance='relative beliefs, not global state uncertainty; calibration/feedback independence not proven'))
    lines=['CP-V108 FRESH FULL EIG-2 SPLIT-IMU LIVE OFF/ON + FUSION',
        'GLIM Hesai+CPT7/HG4930; Kimera stereo+Alphasense IMU. Both persistent in ON; both reception OFF in OFF.',
        'Receiver7s source_wnoa Qc x1.0; actual live arrival/revision journals, no reconstructed replay.',
        'Method1 EVO1.36.3 SE3/no-scale ATE; reference-distance delta0.5m RPE, same physical prism.',
        f"Six-curve COMMON100ms: {coverage['supported_s']:.3f}s/{duration}s, {len(ids)}rows, {len(pairs)}RPEpairs.",
        'Estimator | ATE OFF -> ON m | improvement | tRPE OFF -> ON m | rRPE OFF -> ON deg | estimated path OFF -> ON m | reference path m']
    for role in ('glim','kimera','fusion_online','fusion_delayed'):
        x,y=common['off_'+role],common['on_'+role]
        lines.append('%s | %.6f -> %.6f | %+.2f%% | %.6f -> %.6f | %.6f -> %.6f | %.4f -> %.4f | %.4f'%(role,x['ATE_RMSE_m'],y['ATE_RMSE_m'],improvement[role],
            x['RPE']['translation_m']['statistics']['rmse'],y['RPE']['translation_m']['statistics']['rmse'],
            x['RPE']['rotation_deg']['statistics']['rmse'],y['RPE']['rotation_deg']['statistics']['rmse'],x['path_length_m'],y['path_length_m'],x['reference_path_length_m']))
    lines+=['','FULL-SAVED native cadence: different support and reference-distance pair samples; do not replace common-mask comparison.',
            'Product | ATE m | tRPE m | rRPE deg | segmented estimated/reference path m | rows | supported seconds | first/last seconds | gaps']
    for key,x in full.items():
        lines.append('%s | %.6f | %.6f | %.6f | %.4f / %.4f | %d | %.4f | %.4f / %.4f | %d'%(key,x['ATE_RMSE_m'],x['RPE']['translation_m']['statistics']['rmse'],x['RPE']['rotation_deg']['statistics']['rmse'],x['path_length_m'],x['reference_path_length_m'],x['rows'],x['supported_s'],x['first_s'],x['last_s'],x['gaps']))
    lines+=['','DELAYED is seven-second fixed-lag-finalized, NOT ONLINE. Paths are estimated polylines, not physical travel.',
        'IE-TC shares CPT7. Separate IMUs do not remove persistent feedback correlation. Native-only outgoing information remains FALSE.',
        'Alpha bias RW is an experimental candidate, not certified calibration. Numerical checks are not nonlinear stationarity/health certification.',
        'All3550 raw stereo pairs matched; actual native keyframe gaps retained; no interpolation across >0.5s.',
        'Full revision/timing/native+transport covariance/payload/EOF summaries and omissions: results.json.',
        'Both arms are fresh EIG-2 captures with the same corrected private Kimera binary.']
    d.save(O/'revision_events.json',revision_events);d.save(O/'commands.json',d.commands)
    d.PINS.update(a.HASHES);assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    d.save(O/'input_sha256.json',d.PINS);d.save(H/'results.json',result)
    np.savez_compressed(O/'comparison.npz',**arrays)
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    print('\n'.join(lines),flush=True)
if __name__=='__main__':main()
