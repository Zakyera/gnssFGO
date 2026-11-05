"""Exclusive CP64 scientific/integrity/cleanup closeout."""
import datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,d):
    with (H/n).open('x') as f:json.dump(d,f,indent=2,allow_nan=False)
a=read(H/'analysis.json');m=read(H/'metrics_check.json')
assert a['completed'] and a['numerical_checks_passed'] and all(v['passed'] for v in a['checks'].values())
assert m['passed'] and m['count']==930
assert a['checkpoint_linear_solves']==6 and a['derivative_records']==10
for arm in ['B','C']:
    assert a['metrics'][arm+'_refresh']['under_one_percent_on_both_primary_reductions']
    for phase in ['ordinary','refresh']:
        assert a['systems'][arm+'_'+phase]['used_valid_slots']==a['systems']['A_'+phase]['used_valid_slots']
        assert a['costs'][arm+'_'+phase]['valid_slots']==a['costs']['A_'+phase]['valid_slots']
pins=read(H/'protected_manifest.json')
for collection in [a['input_sha256'],m['input_sha256']]:
    for p,h in collection.items():
        if p in pins:assert pins[p]==h,p
        pins[p]=h
for i,(p,h) in enumerate(pins.items()):
    assert sha(p)==h,p
    if i%2000==0:print('verified',i,'/',len(pins),flush=True)
assert read(H/'extraction_complete.json')['checkpoint_linear_solves']==6
assert read(P/'results/result.json')['fixture_linear_solves']==0
for mode in ['release','san']:
    for stage in ['compile','link','selftest']:assert read(H/(mode+'_'+stage+'_receipt.json'))['returncode']==0
assert read(H/'audit_receipt.json')['returncode']==0
for phase in ['ordinary','refresh']:
    original=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_hesai_insertion_cpv51_20261030/prepared/full_on_source/checkpoints/update_6')/('after_'+phase)/'checkpoint.bin'
    assert sha(original)==sha(P/'results'/(phase+'_before.bin'))==sha(P/'results'/(phase+'_after.bin'))
for arm in ['B','C']:assert sha(P/'results'/(arm+'_function_identity.bin'))==sha(P/'results/ordinary_before.bin')
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
owned=[]
for row in subprocess.check_output(['ps','-eo','pid=,comm=,args='],text=True).splitlines():
    f=row.split(None,2)
    if len(f)==3 and int(f[0])!=os.getpid() and (f[1].startswith('python') or f[1] in ['release','san','cc1plus','c++']) and H.name in f[2]:owned.append(row)
assert not owned,owned
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
verification=dict(passed=True,scientific_checks_passed=True,protected_and_analysis_inputs=len(pins),hesai=hesai,
                  containers=states,owned_jobs_exited=True,checkpoint_linear_solves=6,fixture_linear_solves=0,
                  source_replays=0,production_changes=0,time=datetime.datetime.now().astimezone().isoformat())
save('verification.json',verification)
retained={str(p):sha(p) for root in [H,P] for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts
          and p.name not in ['complete.json','retained_manifest.json']}
save('retained_manifest.json',retained)
complete=dict(completed=True,status='complete_no_material_first_event_response_correction',numerical_checks_passed=True,
              exact_derivatives_verified=True,combined_candidate_tested=False,production_fix=False,full_zigzag_root_cause_proved=False,
              next_experiment_started=False,failures=0,retries=0,verification=verification,
              rotation_revision_reduction_pct={arm:a['metrics'][arm+'_refresh']['reduction_percent']['rotation_deg'] for arm in ['B','C']},
              report_sha256=sha(H/'REPORT.txt'),manifest_sha256=sha(H/'retained_manifest.json'),retained_files=len(retained))
save('complete.json',complete);print(json.dumps(complete,indent=2))

