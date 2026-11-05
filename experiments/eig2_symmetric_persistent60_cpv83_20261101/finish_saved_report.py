"""Finish CP83 from retained EVO archives; zero repeated CLI scoring/captures."""
import copy,datetime,difflib,hashlib,importlib.util,inspect,json,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;A=H/'saved_validation_repair_1';R=H/'report_completion_1'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def write(p,t):
    with p.open('x') as f:f.write(t)
def call(args):return subprocess.check_output(args,text=True)
assert read(A/'failed.json')=={'error':"AssertionError(('evaluate', 1))",'source_retry_authorized':False}
assert 'post_solve_imu_cost_after_250s=stats(cost[1250:,7])' in (A/'evaluate.log').read_text()
assert read(A/'on_validated.json')['passed'] and read(A/'closeout.json')['passed']
R.mkdir(exist_ok=False)
spec=importlib.util.spec_from_file_location('retained_cp83',A/'evaluate.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
np=e.np;fi=e.file_interface
P=e.P;V=e.V;S=P/'full_on_source'
assert read(V/'on/validation.json')['passed']
pins={str(p):sha(p) for p in A.rglob('*') if p.is_file()}
pins[str(H/'failed.json')]=sha(H/'failed.json')
frozen=read(P/'protected_manifest.json')
for p,d in frozen.items():assert sha(p)==d,p
commands=[read(p) for p in sorted((A/'evaluation').rglob('*.command.json'))]
assert len(commands)==24 and all(c['exit']==0 for c in commands)
parity=0.
for c in commands:
    cmd=c['command'];assert '--correct_scale' not in cmd
    ref=fi.read_tum_trajectory_file(cmd[2]);est=fi.read_tum_trajectory_file(cmd[3])
    saved=fi.load_res_file(cmd[cmd.index('--save_results')+1])
    if cmd[0].endswith('evo_ape'):
        assert '--align' in cmd and cmd[cmd.index('-r')+1]=='trans_part'
        aligned=copy.deepcopy(est);_,_,scale=aligned.align(ref,correct_scale=False);assert scale==1
        metric=e.metrics.APE(e.metrics.PoseRelation.translation_part);metric.process_data((ref,aligned))
    else:
        assert cmd[cmd.index('-d')+1]=='0.5' and cmd[cmd.index('-u')+1]=='m' and '--pairs_from_reference' in cmd
        relation=next(rel for text,rel in e.u.RELATIONS.values() if text==cmd[cmd.index('-r')+1])
        metric=e.u.new_metric(relation);metric.process_data((ref,est))
        pairs=e.filters.filter_pairs_by_path(ref.poses_se3,.5,all_pairs=False)
        assert metric.delta_ids==[j for i,j in pairs]
    assert saved.np_arrays['error_array'].shape==metric.error.shape
    delta=float(np.max(abs(saved.np_arrays['error_array']-metric.error)));parity=max(parity,delta);assert delta<1e-8
    for key,value in metric.get_all_statistics().items():assert abs(saved.stats[key]-value)<1e-8,key
metrics=read(A/'evaluation/metric_results.json')
outputs=metrics['common'];full=metrics['full_saved'];coverage=metrics['coverage']
for group in (outputs,full):
    assert set(group)=={'on_glim','on_kimera','on_fusion_online','on_fusion_delayed'}
    for key,row in group.items():
        assert row['rows']>10 and row['reference_path_length_m']>25
        for name in ('translation_m','rotation_deg'):
            archive=fi.load_res_file(row['RPE'][name]['archive'])
            assert archive.stats==row['RPE'][name]['statistics']
            assert len(archive.np_arrays['error_array'])==row['RPE'][name]['pairs']>40
        folder=A/'evaluation'/('common' if group is outputs else 'full_'+key)
        ape=fi.load_res_file(folder/(key+'_ape.zip'))
        assert row['ATE_RMSE_m']==ape.stats['rmse']
        tum=fi.read_tum_trajectory_file(folder/(key+'.tum'));assert len(tum.timestamps)==row['rows']
        refpath=fi.read_tum_trajectory_file(folder/'reference.tum')
        assert np.array_equal(refpath.timestamps,tum.timestamps)
curves={}
curves['on_glim']=e.a.load_curve(S/'glim_odometry.csv')
curves['on_kimera'],_,kimera_checks=e.m.kimera(S)
native=e.a.load_curve(S/'kimera_odometry.csv')
for role,file in [('online','online_trajectory.tum'),('delayed','finalized_trajectory.tum')]:
    curves['on_fusion_'+role]=e.a.load_curve(P/'full_on_receiver/receiver'/file)
roundtrip=0.
for key,(ts,pp,rr) in curves.items():
    fp,fr=e.a.prism(pp,rr);direct=e.curve(ts,fp,fr)
    kept=fi.read_tum_trajectory_file(A/'evaluation'/('full_'+key)/(key+'.tum'))
    assert np.max(abs(kept.timestamps-direct.timestamps))<1e-8
    diff=max(float(np.max(abs(np.asarray(kept.poses_se3)-np.asarray(direct.poses_se3)))),0.)
    roundtrip=max(roundtrip,diff);assert diff<1e-8
revisions,events=e.d39.window_summary(S/'estimators.log',curves['on_kimera'][0],native,curves['on_kimera'])
# Only the inapplicable >250s diagnostic is changed. No metric/state/weight changes.
original=inspect.getsource(e.d39.log_summary)
needle='post_solve_imu_cost_after_250s=stats(cost[1250:,7])'
assert original.count(needle)==1
short=original.replace(needle,"post_solve_imu_cost_after_250s=None,after_250s_note='Not applicable: this recording is 60s'")
write(R/'short_log_summary.py',short)
write(R/'diagnostic.diff',''.join(difflib.unified_diff(original.splitlines(True),short.splitlines(True),fromfile=str(Path(e.d39.__file__)),tofile=str(R/'short_log_summary.py'))))
namespace=dict(e.d39.__dict__);exec(compile(short,str(R/'short_log_summary.py'),'exec'),namespace)
source_logs,_,_=namespace['log_summary'](S/'estimators.log',S/'objective.csv')
diagnostic=dict(receiver=read(V/'on/validation.json'),source=read(A/'candidate_validation.json'),
    local=read(S/'local_provenance.json'),sensor=read(S/'hesai_validation.json'),
    revisions=revisions,source_logs=source_logs,transported_covariance=e.covariance_summary(P/'full_on_receiver/events.csv'))
pins.update(e.PINS);pins.update(e.a.HASHES)
for p in [Path(__file__),H/'PROTOCOL.txt',S/'objective.csv',A/'on_validated.json',A/'closeout.json',V/'on/validation.json']:
    pins[str(p)]=sha(p)
for p,d in pins.items():assert sha(p)==d,p
result=dict(passed=True,task='CP-V83',method='Method1 first/newest absolute publication',evo_version='1.36.3',capture_s=60.,
    common=outputs,full_saved=full,common_coverage=coverage,reference_common_path_m=outputs['on_glim']['reference_path_length_m'],
    RPE_pairs=outputs['on_glim']['RPE']['translation_m']['pairs'],segments=metrics['segments'],diagnostics={'on':diagnostic},
    verification=dict(CLI_API_error_max=parity,TUM_roundtrip_max=roundtrip),kimera_absolute_reconstruction_checks=kimera_checks,
    retained_evo_CLI_calls=24,new_evo_CLI_calls=0,source_runs_this_reporting_stage=0,receiver_replays=0,
    original_capture_count=1,live_bidirectional=True,glim_factor_mode='inject_persistent',kimera_factor_mode='persistent',
    receiver_Qc_multiplier=1.,kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True,
    no_fresh_matched_off_control=True,reference='IE-TC shares CPT7, not independent prism GT',
    path_note='Estimated polyline, not physical travel; native and common cadences differ',
    original_failures_preserved=[str(H/'failed.json'),str(A/'failed.json')],
    repairs=['Read GLIM log for the GLIM injection assertion','Mark >250s diagnostic inapplicable to60s'],
    scoring_repeated=False)
save(R/'results.json',result);save(R/'commands.json',commands);save(R/'input_sha256.json',pins);save(R/'revision_events.json',events)
lines=['CP-V83 — BOTH-PERSISTENT LIVE EIG-2 HESAI, requested60s PASS',
    'One original live capture, no source/receiver rerun; all capture/sensor/payload/lifecycle/receiver gates pass.',
    'Private CP82 GLIM inject_persistent + CP76 Kimera persistent + concurrent CP-K19 WNOA Qc x1.0.',
    'Method1 first/newest absolute outputs; EVO1.36.3 SE3/no-scale ATE, reference-distance delta0.5m RPE.',
    'No fresh OFF control; this is not a full355s sequence or a matched OFF/ON improvement claim.',
    'IE-TC shares CPT7, not independent prism GT. Path is estimated polyline, not physical travel.',
    '', 'COMMON100ms SUPPORT: %.3fs/60s (%.3f%%),%dqueries; reference %.6fm'%(coverage['supported_s'],coverage['supported_capture_percent'],coverage['valid_queries'],result['reference_common_path_m']),
    'Estimator | ATE RMSE m | Translation RPE m | Rotation RPE deg | Estimated path m']
for key,row in outputs.items():lines.append('%s | %.6f | %.6f | %.6f | %.6f'%(key,row['ATE_RMSE_m'],row['RPE']['translation_m']['statistics']['rmse'],row['RPE']['rotation_deg']['statistics']['rmse'],row['path_length_m']))
lines+=['','FULL SAVED60s NATIVE CADENCE (not interchangeable with common-mask table)',
    'Estimator | ATE m | tRPE m | rRPE deg | estimated/reference path m | rows | span/coverage s']
for key,row in full.items():lines.append('%s | %.6f | %.6f | %.6f | %.6f / %.6f | %d | %.6f / %.6f'%(key,row['ATE_RMSE_m'],row['RPE']['translation_m']['statistics']['rmse'],row['RPE']['rotation_deg']['statistics']['rmse'],row['path_length_m'],row['reference_path_length_m'],row['rows'],row['span_s'],row['coverage_supported_s']))
sm=diagnostic['receiver']['summary'];cov=diagnostic['transported_covariance'];rev=revisions
lines+=['','DELAYED is seven-second fixed-lag-finalized, not ONLINE/full-history. No trajectory smoothing.',
    'Source injections GLIM->Kimera292 / Kimera->GLIM293; Kimera298native updates,0temporary retirements.',
    'All %d payload versions exact; %dinserts/%dreplacements/%dpre-init exclusions/%dEOFomissions/%dpending.'%(sm['received'],sm['inserted'],sm['replaced'],sm['excluded'],sm['unsupported_end'],sm['pending_at_end']),
    'Kimera previous-state revision p95 %.6fm,max %.6fm; %dpaired steps,missingIDs=%s.'%(rev['previous_revision_m']['p95'],rev['previous_revision_m']['maximum'],rev['paired_steps'],rev['missing_step_ids']),
    'Kimera optimize mean %.3fms,p95 %.3fms; playback0.20x, NOT a real-time claim.'%(source_logs['optimize_ms']['mean'],source_logs['optimize_ms']['p95'])]
for source,c in cov.items():lines.append('%s: %dversions,median tr(Crot)=%.9g rad2/tr(Ctrans)=%.9g m2,min eigenvalue %.9g.'%(source,c['rows'],c['rotation_trace_rad2']['median'],c['translation_trace_m2']['median'],c['eigenvalue_min']))
lines+=['Kimera mixed-prior outgoing information remains NOT native-only; useraccepted feedback/correlation risk.',
    'Covariance formulas/weights/mean-only alignment approximation unchanged; calibration not certified.',
    'Live logs do not give a full per-peer absorption genealogy; CP82 synthetic checks do, in their tested scope.',
    'Incremental checks are not nonlinear stationarity. >250s diagnostic inapplicable, not fabricated.',
    '', 'REPORT-ONLY REPAIRS, ALL FAILED EVIDENCE PRESERVED:',
    'Original candidate gate read only launcher/Kimera logs; supplemental gate also reads exact GLIM stdout.',
    'Original report requested >250s statistics from60s; marked inapplicable. All24EVO CLI archives retained,',
    'independently checked via EVO API, no CLI re-scoring. No capture or receiver replay.',
    'Protected1989/Hesai667 verified; containers stopped; see post_closeout.json for final independent audit.']
write(R/'RESULTS_REPORT.txt','\n'.join(lines)+'\n')
states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for p,d in frozen.items():assert sha(p)==d,p
hesai=call(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
verification=dict(passed=True,evo_archives_verified=24,EVO_API_parity_max=parity,TUM_roundtrip_max=roundtrip,
    reused_CLI_results=True,new_CLI_invocations=0,protected_files=len(frozen),containers=states,hesai=hesai)
save(R/'report_verification.json',verification)
save(R/'complete.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),saved_only_completion=True,
    output_sha256={str(p):sha(p) for p in R.rglob('*') if p.is_file()},source_retries=0,receiver_replays=0,EVO_CLI_repeats=0))
print('\n'.join(lines),flush=True)
