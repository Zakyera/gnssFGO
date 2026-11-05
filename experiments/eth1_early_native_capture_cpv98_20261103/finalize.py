"""Read-only verification plus new terminal receipt; no experiment stages."""
import datetime,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def exited(r):
    p=Path('/proc')/str(r.get('pid',r.get('supervisor_pid')))/'stat'
    return not p.exists() or p.read_text().split()[21]!=r['proc_start']
assert not (H/'complete.json').exists() and read(H/'verification.json')['passed']
for p,d in read(H/'analysis_inputs.json').items():assert sha(p)==d,p
for p,d in read(H/'verification.json')['output_sha256'].items():assert sha(p)==d,p
receipts=[H/'start.json',H/'launch.json',*H.glob('*_process.json')]
for d in [K/'analysis',K/'analysis_v2']:receipts.extend([d/'start.json',*d.glob('*_process.json')])
for p in receipts:assert exited(read(p)),p
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
now=datetime.datetime.now().astimezone().isoformat()
(H/'status.json').write_text(json.dumps(dict(stage='complete',updated=now,source_runs=1,offline_analysis_passed=True),indent=2))
outputs={str(p):sha(p) for p in H.iterdir() if p.is_file() and p.name not in ('controller.log','controller.lock')}
with (H/'complete.json').open('x') as f:json.dump(dict(passed=True,finished=now,containers=states,hesai=hesai,
    admission_pins=773,verified_analysis_inputs=len(read(H/'analysis_inputs.json')),exact_owned_process_receipts=len(receipts),
    all_owned_jobs_exited=True,source_captures=1,estimator_builds=0,optimizer_runs=0,old_results_unchanged=True,
    output_sha256=outputs,scientific_scope='State-dependent visual gating demonstrated; historical GLIM ATE cause and covariance calibration remain open.'),f,indent=2)
print(json.dumps(dict(passed=True,finished=now,hesai=hesai,containers=states,owned_receipts=len(receipts))))
