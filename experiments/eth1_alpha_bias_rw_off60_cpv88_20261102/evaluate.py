"""One saved-only EVO evaluation of the two-field ablation; retain controls."""
import csv,importlib.metadata,importlib.util,json,re,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';S=P/'off_source';O=H/'evaluation'
OLD=E/'eth1_split_imu_off60_cpv85_20261101_attempt1';Q=W/'runs'/OLD.name/'prepared'
CONTROL=E/'eth1_cpt7_control_off60_cpv86_20261102';QC=W/'runs'/CONTROL.name/'prepared'
spec=importlib.util.spec_from_file_location('metrics86',E/'eig2_symmetric_persistent_full_cpv84_20261101_attempt1/evaluate.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d);a=d.a
audit=json.loads((P/'input_audit.json').read_text());epoch=audit['epoch_ns'];d.epoch=epoch;d.d39.epoch=epoch
def read(p):return d.read(p)
def native_csv(path):
    with d.pin(path).open() as f:rows=list(csv.DictReader(f))
    assert {x['field.child_frame_id'] for x in rows}=={'alphasense_imu'}
    t=np.asarray([int(x['field.header.stamp']) for x in rows],np.int64)
    p=np.asarray([[float(x['field.pose.pose.position.'+c]) for c in 'xyz'] for x in rows])
    q=np.asarray([[float(x['field.pose.pose.orientation.'+c]) for c in 'xyzw'] for x in rows])
    assert np.all(np.diff(t)>0)
    return t,p,Rotation.from_quat(q).as_matrix()
def physical(c,body):
    ts,p,r=c;T=np.asarray(audit['T_'+body+'_prism'])
    return ts,p+np.einsum('nij,j->ni',r,T[:3,3]),r@T[:3,:3]
def kimera(source,body):
    selected=[]
    for line in d.pin(source/'estimators.log').open():
        if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:
            f=d.m.ANSI.sub('',line.split('KIMERA_BACKEND_POSE_PATH_ROW,',1)[1]).strip().split(',')
            assert len(f)==24;selected.append(f)
    with d.pin(source/'kimera_odometry.csv').open() as f:pub=list(csv.DictReader(f))
    assert len(selected)==len(pub)>100
    assert [int(x[1]) for x in selected]==[int(x['field.header.seq']) for x in pub]==list(range(len(pub)))
    assert {x['field.child_frame_id'] for x in pub}=={body+'_imu'}
    t=np.asarray([int(x['field.header.stamp']) for x in pub],np.int64);v=np.asarray(selected,float)
    assert np.all(np.diff(t)>0) and np.isfinite(v).all() and np.all(v[:,23]==0)
    native=np.asarray([[float(x['field.pose.pose.position.'+c]) for c in 'xyz']+[float(x['field.pose.pose.orientation.'+c]) for c in 'wxyz'] for x in pub])
    logged=v[:,9:16];native[:,3:]*=np.where(np.sum(native[:,3:]*logged[:,3:],axis=1)<0,-1.,1.)[:,None]
    assert np.all(abs(native-logged)<=d.m.rounding_bound(logged)+2e-12)
    return (t,v[:,2:5],Rotation.from_quat(v[:,[6,7,8,5]]).as_matrix())
def main():
    assert importlib.metadata.version('evo')=='1.36.3'
    assert read(H/'capture_complete.json')['passed'] and read(H/'closeout.json')['passed']
    assert not O.exists() and not (H/'results.json').exists()
    for mod in (d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(Path(mod.__file__))
    for p in (Path(__file__),H/'PROTOCOL.txt',H/'profile_parity.json',P/'input_audit.json',S/'effective_mode_check.json',S/'source_coverage.json'):d.pin(p)
    old=read(OLD/'report_completion_1/results.json');control=read(CONTROL/'results.json')
    record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=d.m.c.b.load(record)
    reference=physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
    kg=kimera(S,'alphasense');alpha=kimera(Q/'off_source','alphasense')
    cpt=kimera(QC/'off_source','cpt7')
    curves={'GLIM_CPT7_ablation':physical(a.load_curve(S/'glim_odometry.csv'),'cpt7'),'Kimera_Alphasense_bias_rw':physical(kg,'alphasense')}
    oldref=d.file_interface.read_tum_trajectory_file(d.pin(OLD/'saved_report_1/evaluation/common/reference.tum'))
    grid=np.rint(oldref.timestamps*1e9).astype(np.int64)+epoch
    gp,gr,valid=a.sample(reference,grid,50_000_000);samples={}
    for name,c in curves.items():
        p,r,good=a.sample(c,grid,500_000_000);valid &= good;samples[name]=(p,r)
    ap,ar,agood=a.sample(physical(alpha,'alphasense'),grid,500_000_000);assert agood.all()
    cp,cr,cgood=a.sample(physical(cpt,'cpt7'),grid,500_000_000);assert cgood.all()
    ids=np.flatnonzero(valid);assert len(ids)>100
    splits=(np.diff(ids)>1)|(np.diff(grid[ids])>100_000_001)
    segments=np.split(ids,np.flatnonzero(splits)+1)
    O.mkdir();folder=O/'common';folder.mkdir();rp=folder/'reference.tum';rt=d.export(rp,d.curve(grid[ids],gp[ids],gr[ids]))
    refs,pairs,info=d.reference_segments(folder,grid,gp,gr,segments)
    common={};full={}
    for name,(p,r) in samples.items():
        common[name]=d.score_series(folder,name,grid,p,r,gp,gr,segments,refs,rp,rt)
        ts,p,r=curves[name];fp,fr,good=a.sample(reference,ts,50_000_000);assert good.all()
        ff=O/('full_'+name);ff.mkdir();cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        frp=ff/'reference.tum';frt=d.export(frp,d.curve(ts,fp,fr));rfs,prs,inf=d.reference_segments(ff,ts,fp,fr,cuts)
        full[name]=d.score_series(ff,name,ts,p,r,fp,fr,cuts,rfs,frp,frt)
        full[name].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
            supported_s=sum(float((ts[ii[-1]]-ts[ii[0]])*1e-9) for ii in cuts),gaps=len(cuts)-1,segments=inf)
    reused=bool(valid.all())
    if reused:
        oldpairs=read(OLD/'saved_report_1/evaluation/common/pair_indices.json');assert len(oldpairs)==len(pairs)
        for x,y in zip(oldpairs,pairs):
            for key in ('segment','start_ns','end_ns'):assert x[key]==y[key]
            assert abs(x['reference_distance_m']-y['reference_distance_m'])<1e-8
        assert np.max(abs(rt.positions_xyz-oldref.positions_xyz))<1e-8
        common['Kimera_Alphasense_saved']=old['common']['Kimera_Alphasense']
        common['GLIM_CPT7_saved']=old['common']['GLIM_CPT7']
        cref=d.file_interface.read_tum_trajectory_file(d.pin(CONTROL/'evaluation/common/reference.tum'))
        assert np.array_equal(cref.timestamps,oldref.timestamps)
        assert np.max(abs(cref.positions_xyz-oldref.positions_xyz))<1e-8
        assert np.max(abs(cref.orientations_quat_wxyz-oldref.orientations_quat_wxyz))<1e-8
        assert read(CONTROL/'evaluation/common/pair_indices.json')==oldpairs
        common['Kimera_CPT7_saved']=control['common']['Kimera_CPT7_control']
    else:
        common['Kimera_Alphasense_saved']=d.score_series(folder,'Kimera_Alphasense_saved',grid,ap,ar,gp,gr,segments,refs,rp,rt)
        common['Kimera_CPT7_saved']=d.score_series(folder,'Kimera_CPT7_saved',grid,cp,cr,gp,gr,segments,refs,rp,rt)
    revision,events=d.d39.window_summary(S/'estimators.log',kg[0],native_csv(S/'kimera_odometry.csv'),kg)
    d.save(H/'revision_events.json',events)
    timing=[];cov=[]
    for line in (S/'estimators.log').open():
        for marker,dest in [('KIMERA_OPTIMIZE_TIMING_ROW,',timing),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
            if marker in line:dest.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
    times=np.asarray(timing,float);mat=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
    assert np.isfinite(mat).all() and np.linalg.eigvalsh(mat).min()>0 and np.max(abs(mat-mat.transpose(0,2,1)))<1e-10
    diag=dict(revisions=revision,optimize_ms=d.d39.stats(times[:,1]),covariance=dict(rows=len(mat),
        rotation_trace_rad2=d.d39.stats(np.trace(mat[:,:3,:3],axis1=1,axis2=2)),translation_trace_m2=d.d39.stats(np.trace(mat[:,3:,3:],axis1=1,axis2=2)),calibration_claim=False))
    supported=sum(float((grid[ii[-1]]-grid[ii[0]])*1e-9) for ii in segments)
    result=dict(task='CP-V88',passed=True,scope='ETH1 Alpha CBS-OFF60 bias-random-walk-only ablation',
        common=common,full_saved=full,common_supported_s=supported,common_coverage_percent=100*supported/60,
        common_rows=len(ids),common_segments=info,common_RPE_pairs=len(pairs),CP85_identical_mask_and_reused_scores=reused,
        diagnostics=diag,CP85_diagnostics=old['diagnostics'],CP86_diagnostics=control['diagnostics'],checks=d.checks,no_CBS_ON=True,no_fusion=True,
        limitations=['Only two bias random walks changed; white noise, Alpha hardware/frames/timing retained; not calibrated noise selection','IE-TC shares CPT7, not independent prism GT',
        'Native and common cadence distinct; no bridging gaps >0.5s','No bias/IMU residual trace beyond previously enabled diagnostics',
        'Covariance consistency and nonlinear stationarity not certified'])
    lines=['CP-V88 ETH1 Alpha two-field bias-RW ablation versus saved Alpha and CPT7, CBS OFF60',
        'Method1 first/newest absolute at common physical prism; EVO1.36.3 SE3 no-scale ATE; reference-distance0.5m RPE.',
        f'COMMON100ms support {supported:.3f}s /60; {len(ids)} rows, {len(pairs)} RPE pairs. CP85 scores reused={reused}.',
        'Estimator | ATE m | tRPE m | rRPE deg | estimated path m | reference path m']
    for key,row in common.items():lines.append(f"{key} | {row['ATE_RMSE_m']:.6f} | {row['RPE']['translation_m']['statistics']['rmse']:.6f} | {row['RPE']['rotation_deg']['statistics']['rmse']:.6f} | {row['path_length_m']:.6f} | {row['reference_path_length_m']:.6f}")
    lines+=['Native-cadence new results and revision/timing/covariance summaries: results.json.',
        'IE-TC shares CPT7, not independent prism GT. Estimated polyline is not physical travel.',
        'One new Alpha capture; saved CP85/86 controls. No CBS ON/fusion/full sequence; no default promotion.']
    for p in [S/'glim_odometry.csv',S/'kimera_odometry.csv',S/'estimators.log']:d.pin(p)
    d.PINS.update(a.HASHES);assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS);d.save(H/'results.json',result)
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    d.save(H/'report_verification.json',dict(passed=True,checks=d.checks,CLI_calls=len(d.commands),input_hashes=len(d.PINS),
        CP85_identical_mask_and_reused_scores=reused,outputs={str(p):d.sha(p) for p in [H/'results.json',H/'RESULTS_REPORT.txt',O/'commands.json']}))
    print('\n'.join(lines))
if __name__=='__main__':main()
