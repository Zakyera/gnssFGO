"""Exclusive CP62 closeout, historical hashes and actual host cleanup."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,d):
    with (H/n).open('x') as f:json.dump(d,f,indent=2)
analysis=json.loads((H/'analysis_v2.json').read_text())
assert analysis['audit_completed'] and analysis['numerical_checks_passed']
assert len(analysis['records'])==1014 and sum(r['accepted'] for r in analysis['records'])==818
assert all(r['passed'] for r in analysis['records'])
assert len(analysis['systems'])==2
pins=json.loads((H/'protected_manifest.json').read_text())
for p,h in analysis['input_sha256'].items():
    if p in pins:assert pins[p]==h,p
    pins[p]=h
for i,(p,h) in enumerate(pins.items()):
    assert sha(p)==h,p
    if i%2000==0:print('verified',i,'/',len(pins),flush=True)
assert json.loads((H/'extraction_complete.json').read_text())['completed']
for mode in ['release','san']:
    assert json.loads((H/(mode+'_selftest_receipt.json')).read_text())['returncode']==0
assert json.loads((H/'audit_receipt.json').read_text())['returncode']==0
for phase in ['ordinary','refresh']:
    assert sha(P/'results'/(phase+'_before.bin'))==sha(P/'results'/(phase+'_after.bin'))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
processes=subprocess.check_output(['ps','-eo','pid=,comm=,args='],text=True)
owned=[]
for line in processes.splitlines():
    f=line.split(None,2)
    if len(f)==3 and int(f[0])!=os.getpid() and (f[1].startswith('python') or f[1] in ['release','san','cc1plus','c++']) and H.name in f[2]:owned.append(line)
assert not owned,owned
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
verification=dict(passed=True,protected_and_analysis_inputs=len(pins),containers=states,owned_jobs_exited=True,
                  hesai=hesai,source_replays=0,production_changes=0,saved_baseline_linear_solves=2,
                  nonlinear_forward_steps=0,time=datetime.datetime.now().astimezone().isoformat())
save('verification.json',verification)
retained={str(p):sha(p) for root in [H,P] for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts
          and p.name not in ['complete.json','retained_manifest.json']}
save('retained_manifest.json',retained)
complete=dict(completed=True,suspect1_parked_by_user=True,suspect2_first_event_visual_math_passed=True,
              full_zigzag_root_cause_proved=False,production_fix=False,next_suspect_started=False,
              accepted_factor_state_checks=818,rejected_zero_checks=196,
              reporting_repair_only=True,authoritative_analysis='analysis_v2.json',verification=verification,
              report_sha256=sha(H/'REPORT.txt'),manifest_sha256=sha(H/'retained_manifest.json'),retained_files=len(retained))
save('complete.json',complete);print(json.dumps(complete,indent=2))

