"""Saved-only, same physical prism, EVO Method1; no estimator launch."""
import csv,importlib.util,json,re,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared';S=P/'off_source'
spec=importlib.util.spec_from_file_location('metrics84',E/'eig2_symmetric_persistent_full_cpv84_20261101_attempt1/evaluate.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
a=d.a;O=H/'evaluation';audit=json.loads((P/'input_audit.json').read_text());epoch=audit['epoch_ns']
d.epoch=epoch;d.d39.epoch=epoch
def save(p,v):d.save(p,v)
def physical(curve,body):
    ts,p,r=curve;T=np.asarray(audit['T_'+body+'_prism'])
    return ts,p+np.einsum('nij,j->ni',r,T[:3,3]),r@T[:3,:3]
def kimera():
    selected=[]
    for line in d.pin(S/'estimators.log').open():
        if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:
            fields=d.m.ANSI.sub('',line.split('KIMERA_BACKEND_POSE_PATH_ROW,',1)[1]).strip().split(',')
            assert len(fields)==24;selected.append(fields)
    with d.pin(S/'kimera_odometry.csv').open() as f:pubs=list(csv.DictReader(f))
    assert len(selected)==len(pubs)>100
    assert [int(x[1]) for x in selected]==[int(x['field.header.seq']) for x in pubs]==list(range(len(pubs)))
    assert {x['field.child_frame_id'] for x in pubs}=={'alphasense_imu'}
    stamps=np.asarray([int(x['field.header.stamp']) for x in pubs],np.int64)
    assert np.all(np.diff(stamps)>0)
    data=np.asarray(selected,float);assert np.isfinite(data).all() and np.all(data[:,23]==0)
    native=np.asarray([[float(p['field.pose.pose.position.'+c]) for c in 'xyz']+
                       [float(p['field.pose.pose.orientation.'+c]) for c in 'wxyz'] for p in pubs])
    logged=data[:,9:16];sign=np.where(np.sum(native[:,3:]*logged[:,3:],axis=1)<0,-1.,1.)
    native[:,3:]*=sign[:,None]
    assert np.all(abs(native-logged)<=d.m.rounding_bound(logged)+2e-12)
    q=data[:,[6,7,8,5]];assert np.max(abs(np.linalg.norm(q,axis=1)-1))<2e-6
    return (stamps,data[:,2:5],R.from_quat(q).as_matrix()),dict(rows=len(pubs),contiguous_ids=True,
        exact_native_timestamps=True,incremental_pose_join_verified=True,body='alphasense_imu',
        score_output='newest absolute backend, not incremental ROS position')
def main():
    assert json.loads((H/'capture_complete.json').read_text())['passed']
    assert json.loads((H/'closeout.json').read_text())['passed']
    assert not O.exists() and not (H/'results.json').exists()
    for p in [Path(__file__),H/'PROTOCOL.txt',P/'input_audit.json',S/'effective_mode_check.json',S/'source_coverage.json',Path(d.__file__)]:d.pin(p)
    record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1')
    _,_,ref=d.m.c.b.load(record)
    reference=physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
    kg,kchecks=kimera();glim=a.load_curve(S/'glim_odometry.csv');d.m.c.check_csv(S/'glim_odometry.csv')
    curves={'GLIM_CPT7':physical(glim,'cpt7'),'Kimera_Alphasense':physical(kg,'alphasense')}
    start=max(c[0][0] for c in curves.values());end=min(c[0][-1] for c in curves.values())
    grid=np.arange(((start+10**8-1)//10**8)*10**8,(end//10**8)*10**8+1,10**8,dtype=np.int64)
    gp,gr,valid=a.sample(reference,grid,50_000_000);samples={}
    for key,c in curves.items():
        p,r,good=a.sample(c,grid,500_000_000);valid &= good;samples[key]=(p,r)
    ids=np.flatnonzero(valid);assert len(ids)>100
    segments=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    O.mkdir();folder=O/'common';folder.mkdir();rp=folder/'reference.tum'
    rt=d.export(rp,d.curve(grid[ids],gp[ids],gr[ids]));refs,pairs,info=d.reference_segments(folder,grid,gp,gr,segments)
    common={};full={}
    for key,(pp,rr) in samples.items():
        common[key]=d.score_series(folder,key,grid,pp,rr,gp,gr,segments,refs,rp,rt)
        ts,pp,rr=curves[key];fp,fr,good=a.sample(reference,ts,50_000_000);assert good.all()
        ff=O/('full_'+key);ff.mkdir();cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        rpf=ff/'reference.tum';rtf=d.export(rpf,d.curve(ts,fp,fr));rfs,prs,inf=d.reference_segments(ff,ts,fp,fr,cuts)
        full[key]=d.score_series(ff,key,ts,pp,rr,fp,fr,cuts,rfs,rpf,rtf)
        full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
                         covered_s=sum(float((ts[i[-1]]-ts[i[0]])*1e-9) for i in cuts),gaps=len(cuts)-1,segments=inf)
    revisions,events=d.d39.window_summary(S/'estimators.log',kg[0],a.load_curve(S/'kimera_odometry.csv'),kg)
    save(O/'revision_events.json',events)
    timing=[];cov=[]
    for line in (S/'estimators.log').open():
        for marker,dest in [('KIMERA_OPTIMIZE_TIMING_ROW,',timing),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
            if marker in line:dest.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
    timings=np.asarray(timing,float);mat=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
    assert np.isfinite(timings).all() and np.isfinite(mat).all()
    assert np.max(abs(mat-mat.transpose(0,2,1)))<1e-10 and np.linalg.eigvalsh(mat).min()>0
    diag=dict(revisions=revisions,optimize_ms=d.d39.stats(timings[:,1]),
              kimera_native_alphasense_covariance=dict(rows=len(mat),rotation_trace=d.d39.stats(np.trace(mat[:,:3,:3],axis1=1,axis2=2)),
              translation_trace=d.d39.stats(np.trace(mat[:,3:,3:],axis1=1,axis2=2)),calibration_claim=False))
    coverage=d.m.c.coverage(grid,valid,epoch,60.)
    result=dict(passed=True,task='CP-V85',mode='CBS OFF, source-only split-IMU',duration_s=60,
                method='Method1 first/newest absolute',evo_version='1.36.3',common=common,full_saved=full,
                common_coverage=coverage,reference_common_path_m=sum(x[0].path_length for x in refs),RPE_pairs=len(pairs),
                kimera_reconstruction=kchecks,diagnostics=diag,EVO_checks=d.checks,
                uncertainty_consistency_established=False,no_fusion=True,no_CBS_ON=True)
    assert d.checks['CLI_API_error_max']<1e-8
    d.PINS.update(a.HASHES)
    assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    save(O/'commands.json',d.commands);save(O/'input_sha256.json',d.PINS);save(H/'results.json',result)
    lines=['CP-V85 ETH1 separate-IMU CBS-OFF60, Method1 direct EVO',
           'GLIM Hesai+CPT7; Kimera front stereo+Alphasense, author noise/extrinsics, current CP40 fix.',
           'No fusion or CBS ON. Not a full-sequence result or shared-vs-split control comparison.',
           'Reference: IE-TC shares CPT7; not independent prism GT. Both outputs transformed to prism.',
           'ATE SE3/no scale. RPE reference-distance delta0.5m. Estimated polyline is not physical travel.',
           'COMMON100ms mask: '+json.dumps(coverage),
           'Estimator | ATE m | tRPE m | rRPE deg | estimated path m | reference path m']
    for key,row in common.items():lines.append(f"{key} | {row['ATE_RMSE_m']:.6f} | {row['RPE']['translation_m']['statistics']['rmse']:.6f} | {row['RPE']['rotation_deg']['statistics']['rmse']:.6f} | {row['path_length_m']:.6f} | {row['reference_path_length_m']:.6f}")
    lines+=['FULL-SAVED native-cadence results and diagnostics: see results.json.',
            'Native Alphasense covariance retained; no uncertainty calibration/independence claim.',
            'Separate IMUs remove shared raw samples, not bidirectional feedback (not exercised here).']
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    save(H/'report_verification.json',dict(passed=True,checks=d.checks,CLI_calls=len(d.commands),input_hashes=len(d.PINS),
         outputs={str(p):d.sha(p) for p in (H/'results.json',H/'RESULTS_REPORT.txt',O/'commands.json')}))
    print('\n'.join(lines))
if __name__=='__main__':main()
