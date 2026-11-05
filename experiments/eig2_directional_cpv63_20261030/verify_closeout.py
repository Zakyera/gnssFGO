"""CP63 closeout: integrity/cleanup pass is separate from failed scientific gates."""
import datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
A=H/'attempt_vector_api'
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
def save(name,value):
    with (H/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
a=read(H/'analysis.json');e=read(H/'gate_explanation.json')
assert a['audit_completed'] and not a['numerical_checks_passed'] and a['optimizer_calls']==0
failed=[k for k,v in a['checks'].items() if not v['passed']]
assert set(failed)=={'native_model_independent','nonvisual_directional_FD_1e-05','nonvisual_directional_FD_1e-06'}
assert all(p['factor_count']==356 for p in a['phases'].values())
assert [a['phases'][p]['starting_accepted'] for p in ['ordinary','refresh']]==[285,266]
assert a['checks']['native_residual']['count']==9256
assert a['checks']['manual_visual_cost']['count']==7163
# The supplemental exact physical costs/derivatives explain failed native
# approximations; they do NOT replace the native models or original verdict.
for phase,p in e['phases'].items():
    fs={z['slot']:z for z in a['phases'][phase]['directional_checks']}
    for slot,r in p['nonvisual_exact_chain'].items():
        assert r['independent_cost_max_relative']<1e-8
        for x,y in zip(r['exact_chain'],fs[int(slot)]['checks']):
            assert x['eps']==y['eps']
            assert abs(x['exact_chain_slope']-y['actual'])/(1+abs(y['actual']))<1e-6
assert len(e['phases']['ordinary']['native_model_discrepancies'])==1
assert not e['phases']['refresh']['native_model_discrepancies']
pins=read(A/'protected_manifest.json')
for collection in [a['input_sha256'],e['input_sha256']]:
    for p,h in collection.items():
        if p in pins:assert pins[p]==h,p
        pins[p]=h
for i,(p,h) in enumerate(pins.items()):
    assert sha(p)==h,p
    if i%2000==0:print('verified',i,'/',len(pins),flush=True)
assert read(H/'release_compile_receipt.json')['returncode']==1
assert read(A/'extraction_complete.json')['optimizer_calls']==0
for mode in ['release','san']:
    for stage in ['compile','link','selftest']:
        assert read(A/(mode+'_'+stage+'_receipt.json'))['returncode']==0
assert read(A/'audit_receipt.json')['returncode']==0
for phase in ['ordinary','refresh']:
    q=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_hesai_insertion_cpv51_20261030/prepared/full_on_source/checkpoints/update_6')/('after_'+phase)/'checkpoint.bin'
    assert sha(q)==sha(P/'attempt_vector_api/results'/(phase+'_before.bin'))==sha(P/'attempt_vector_api/results'/(phase+'_after.bin'))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
processes=subprocess.check_output(['ps','-eo','pid=,comm=,args='],text=True)
owned=[]
for line in processes.splitlines():
    z=line.split(None,2)
    if len(z)==3 and int(z[0])!=os.getpid() and (z[1].startswith('python') or z[1] in ['release','san','cc1plus','c++']) and H.name in z[2]:owned.append(line)
assert not owned,owned
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
v=dict(integrity_cleanup_passed=True,scientific_checks_all_passed=False,failed_gate_categories=failed,
       protected_and_analysis_inputs=len(pins),hesai=hesai,containers=states,owned_jobs_exited=True,
       optimizer_calls=0,source_replays=0,production_changes=0,time=datetime.datetime.now().astimezone().isoformat())
save('verification.json',v)
retained={str(p):sha(p) for root in [H,P] for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts
          and p.name not in ['complete.json','retained_manifest.json']}
save('retained_manifest.json',retained)
complete=dict(audit_completed=True,completed=True,status='complete_with_failed_scientific_gates_explained_not_relaxed',
              numerical_checks_passed=False,exact_native_Jacobian_approximations_identified=True,
              derivative_approximation_response_effect_tested=False,full_zigzag_root_cause_proved=False,
              optimizer_calls=0,production_fix=False,next_experiment_started=False,
              build_failure_preserved=True,fresh_harness_repair='attempt_vector_api',verification=v,
              report_sha256=sha(H/'REPORT.txt'),manifest_sha256=sha(H/'retained_manifest.json'),retained_files=len(retained))
save('complete.json',complete);print(json.dumps(complete,indent=2))

