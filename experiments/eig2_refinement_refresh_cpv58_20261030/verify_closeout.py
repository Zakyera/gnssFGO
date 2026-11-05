"""Finalize CP58 without changing retained evidence or historical runs."""
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


def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2)


pins=json.loads((H/'protected_manifest.json').read_text())
analysis=json.loads((H/'analysis.json').read_text())
for p,h in analysis['input_sha256'].items():
    if p in pins:assert pins[p]==h,p
    pins[p]=h
for i,(p,h) in enumerate(pins.items()):
    assert sha(p)==h,p
    if i%2000==0:print('verified',i,'/',len(pins),flush=True)
assert json.loads((H/'test_passed.json').read_text())['passed']
for mode in ['release','san']:assert json.loads((H/(mode+'_selftest_receipt.json')).read_text())['returncode']==0
assert sha(P/'results/B_restored_before.bin')==sha(P/'results/refresh_before.bin')
for phase in ['ordinary','refresh']:assert sha(P/'results'/(phase+'_before.bin'))==sha(P/'results'/(phase+'_after.bin'))
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
processes=subprocess.check_output(['ps','-eo','pid=,comm=,args='],text=True)
owned=[]
for line in processes.splitlines():
    f=line.split(None,2)
    if len(f)==3 and int(f[0])!=os.getpid() and (f[1].startswith('python') or f[1] in ['release','san']) and H.name in f[2]:owned.append(line)
assert not owned,owned
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
verification=dict(passed=True,protected_and_analysis_inputs=len(pins),containers=containers,
                  owned_jobs_exited=True,hesai=hesai,source_replays=0,production_changes=0,
                  real_saved_graph_GN_steps=2,time=datetime.datetime.now().astimezone().isoformat())
save('verification.json',verification)
retained={str(p):sha(p) for root in [H,P] for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts
          and p.name not in ['complete.json','retained_manifest.json']}
save('retained_manifest.json',retained)
complete=dict(completed=True,suspect1_closed=False,full_zigzag_root_cause_proved=False,
              production_fix=False,next_suspect_started=False,verification=verification,
              original_equation_error=analysis['parity']['equation_relative'],
              original_state_error=analysis['parity']['state_tangent'],restored_factors=7,
              untouched_factor_gaussians=349,retained_files=len(retained),
              report_sha256=sha(H/'REPORT.txt'),manifest_sha256=sha(H/'retained_manifest.json'))
save('complete.json',complete);print(json.dumps(complete,indent=2))
