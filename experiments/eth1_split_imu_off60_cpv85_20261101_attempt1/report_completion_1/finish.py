"""Verify existing EVO archives and finish report; never rerun CLI or sources."""
import copy,csv,hashlib,importlib.metadata,importlib.util,json,re,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core import filters,metrics
from evo.tools import file_interface
sys.dont_write_bytecode=True
Q=Path(__file__).resolve().parent;H=Q.parent;F=H/'saved_report_1';O=F/'evaluation'
spec=importlib.util.spec_from_file_location('saved85',F/'evaluate_saved.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=m.d;S=m.S;P=m.P;epoch=m.epoch
assert importlib.metadata.version('evo')=='1.36.3'
assert json.loads((F/'source_coverage.json').read_text())['passed']
assert json.loads((F/'evaluate_receipt.json').read_text())['exit']==1
assert 'field.child_frame_id' in (F/'evaluate.log').read_text()
assert not (Q/'results.json').exists()
pins={};maxdiff=0.;pose_error=0.
def pin(p):
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def read(p):return json.loads(pin(p).read_text())
def tr(p):return file_interface.read_tum_trajectory_file(pin(p))
def archive(p):return file_interface.load_res_file(pin(p))
def verify(result,metric):
    global maxdiff
    assert result.np_arrays['error_array'].shape==metric.error.shape
    delta=float(np.max(abs(result.np_arrays['error_array']-metric.error)))
    maxdiff=max(maxdiff,delta);assert delta<1e-8
    for k,v in metric.get_all_statistics().items():assert abs(result.stats[k]-v)<1e-8
def score(folder,name):
    ref=tr(folder/'reference.tum');est=tr(folder/(name+'.tum'))
    ar=archive(folder/(name+'_ape.zip'));aligned=copy.deepcopy(est)
    aligned.align(ref,correct_scale=False)
    am=metrics.APE(metrics.PoseRelation.translation_part);am.process_data((ref,aligned));verify(ar,am)
    path=refpath=support=0.;errors={k:[] for k in d.u.RELATIONS};pairs=0
    for seg in sorted(folder.glob('segment_*')):
        rr=tr(seg/'reference.tum');ee=tr(seg/(name+'.tum'))
        assert np.array_equal(rr.timestamps,ee.timestamps)
        path+=ee.path_length;refpath+=rr.path_length;support+=ee.timestamps[-1]-ee.timestamps[0]
        pp=filters.filter_pairs_by_path(rr.poses_se3,.5,all_pairs=False) if len(rr.timestamps)>1 else []
        pairs+=len(pp)
        if not pp:continue
        for key,(_,relation) in d.u.RELATIONS.items():
            result=archive(seg/(name+'_'+key+'.zip'));rm=d.u.new_metric(relation)
            rm.process_data((rr,ee));verify(result,rm);errors[key].append(result.np_arrays['error_array'])
    assert pairs==len(read(folder/'pair_indices.json'))
    rpe={}
    for key,(_,relation) in d.u.RELATIONS.items():
        rm=d.u.new_metric(relation);rm.error=np.concatenate(errors[key]);assert len(rm.error)==pairs
        pooled=archive(folder/(name+'_'+key+'_pooled.zip'));verify(pooled,rm)
        rpe[key]=dict(statistics=pooled.stats,pairs=pairs)
    return dict(ATE_RMSE_m=ar.stats['rmse'],path_length_m=path,reference_path_length_m=refpath,
        RPE=rpe,rows=len(est.timestamps),supported_s=float(support),coverage_percent=float(100*support/60),
        first_s=float(est.timestamps[0]),last_s=float(est.timestamps[-1]),segments=len(list(folder.glob('segment_*'))))
absolute,kchecks=m.kimera();glim=m.a.load_curve(S/'glim_odometry.csv')
with (S/'kimera_odometry.csv').open() as f:rows=list(csv.DictReader(f))
assert {r['field.child_frame_id'] for r in rows}=={'alphasense_imu'}
native=(np.asarray([int(r['field.header.stamp']) for r in rows],np.int64),
        np.asarray([[float(r['field.pose.pose.position.'+c]) for c in 'xyz'] for r in rows]),
        Rotation.from_quat([[float(r['field.pose.pose.orientation.'+c]) for c in 'xyzw'] for r in rows]).as_matrix())
curves={'GLIM_CPT7':m.physical(glim,'cpt7'),'Kimera_Alphasense':m.physical(absolute,'alphasense')}
record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=d.m.c.b.load(record)
reference=m.physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
def check_export(saved,ts,p,r):
    global pose_error
    assert np.max(abs(saved.timestamps-(ts-epoch)*1e-9))<1e-8
    delta=max(float(np.max(abs(saved.positions_xyz-p))),float(np.max(abs(np.asarray(saved.poses_se3)[:,:3,:3]-r))))
    pose_error=max(pose_error,delta);assert delta<1e-8
common={};full={}
for name,(ts,p,r) in curves.items():
    check_export(tr(O/('full_'+name)/(name+'.tum')),ts,p,r)
    cc=tr(O/'common'/(name+'.tum'));t=np.rint(cc.timestamps*1e9).astype(np.int64)+epoch
    pp,rr,good=m.a.sample(curves[name],t,500_000_000);assert good.all();check_export(cc,t,pp,rr)
    gp,gr,good=m.a.sample(reference,t,50_000_000);assert good.all();check_export(tr(O/'common/reference.tum'),t,gp,gr)
    gp,gr,good=m.a.sample(reference,ts,50_000_000);assert good.all();check_export(tr(O/('full_'+name)/'reference.tum'),ts,gp,gr)
    common[name]=score(O/'common',name);full[name]=score(O/('full_'+name),name)
revisions,events=d.d39.window_summary(S/'estimators.log',absolute[0],native,absolute)
save(Q/'revision_events.json',events)
text=(S/'estimators.log').read_text();times=[];cov=[]
for line in text.splitlines():
    if 'KIMERA_OPTIMIZE_TIMING_ROW,' in line:times.append(d.m.ANSI.sub('',line.split('KIMERA_OPTIMIZE_TIMING_ROW,',1)[1]).strip().split(','))
    if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:cov.append(d.m.ANSI.sub('',line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1]).strip().split(',')[28:])
times=np.asarray(times,float);mat=np.asarray(cov,float).reshape(-1,6,6)
assert np.isfinite(mat).all() and np.linalg.eigvalsh(mat).min()>0 and np.max(abs(mat-mat.transpose(0,2,1)))<1e-10
diag=dict(revisions=revisions,optimize_ms=d.d39.stats(times[:,1]),kimera_native_alphasense_covariance=dict(
    rows=len(mat),rotation_trace_rad2=d.d39.stats(np.trace(mat[:,:3,:3],axis1=1,axis2=2)),
    translation_trace_m2=d.d39.stats(np.trace(mat[:,3:,3:],axis1=1,axis2=2)),calibration_claim=False))
commands=[read(p) for p in O.rglob('*.command.json')];assert len(commands)==12 and all(c['exit']==0 for c in commands)
for mod in (m,d,d.m,d.m.c,d.m.c.b,d.a,d.a.g,d.u,d.d39,d.d39.v):pin(Path(mod.__file__))
for p in [Path(__file__),F/'PROTOCOL.txt',F/'validate_saved.py',F/'source_coverage.json',P/'input_audit.json',S/'kimera_odometry.csv',S/'glim_odometry.csv',S/'estimators.log']:pin(p)
pins.update(m.a.HASHES)
results=dict(task='CP-V85',report_verified=True,pilot_numerically_healthy=False,scope='60s ETH1 source-only CBS-OFF split-IMU',
    original_frontend_guard_passed=False,endpoint_audit_passed=True,saved_only_completion=True,source_retries=0,
    common=common,full_saved=full,diagnostics=diag,kimera_reconstruction=kchecks,
    reference='IE-TC shares CPT7; not independent prism ground truth',no_fusion=True,no_CBS_ON=True,
    limitations=['No matched current shared-CPT7 control','Sparse startup keyframe interval excluded from common mask',
    'Uncertainty calibration and independence unproven','Estimated path is not physical travel','No per-frame frontend branch trace'],
    note='Original evaluator completed all12EVO CLI scores, then its diagnostic parser incorrectly demanded CPT7 frame. Archives reused, never rescored.')
save(Q/'results.json',results)
lines=['CP-V85 ETH1 SPLIT IMUs: OFF60 PILOT COMPLETE; KIMERA UNHEALTHY',
    'GLIM Hesai+CPT7; Kimera stereo+Alphasense; author extrinsics/Allan parameters, current binaries.',
    'Method1 first/newest absolute; EVO1.36.3 ATE SE3/no scale, reference-distance0.5m RPE.',
    'Same physical prism point. IE-TC shares CPT7 and is not independent prism GT.',
    'COMMON100ms: estimator | ATE m | tRPE m | rRPE deg | estimated path m | reference path m | support s']
for name,r in common.items():lines.append(f"{name} | {r['ATE_RMSE_m']:.6f} | {r['RPE']['translation_m']['statistics']['rmse']:.6f} | {r['RPE']['rotation_deg']['statistics']['rmse']:.6f} | {r['path_length_m']:.6f} | {r['reference_path_length_m']:.6f} | {r['supported_s']:.6f}")
lines+=['Full-saved native cadence, revisions/timing/covariance: results.json. Estimated path is not physical travel.',
    '601received stereo pairs =3before initialization +597processed +1EOF beyond last IMU by1.4386ms.',
    'Original frontend-count failure preserved; exact saved endpoint audit passed, no source retry.',
    'All12EVO CLI archives were produced once. Post-score diagnostic frame assumption repaired only in this supplemental report.',
    'No ON, fusion, full-sequence or control-comparison claim. Do not proceed to CBS-ON with this unhealthy Kimera baseline.']
with (Q/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
save(Q/'report_verification.json',dict(passed=True,EVO_CLI_calls=12,additional_CLI_calls=0,
    CLI_API_error_max=maxdiff,source_export_error_max=pose_error,input_files=len(pins)))
save(Q/'input_sha256.json',pins)
print('\n'.join(lines))
