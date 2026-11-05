"""Final read-only integrity/ownership verification and exclusive receipt."""
import datetime,hashlib,json,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name;P=K/'prepared';V=Path('/home/yeranis/results')/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,want in pins.items():assert sha(p)==want,p
 return len(pins)
for n in ('build_passed.json','capture_complete.json','closeout.json','inspection_passed.json','factors_v2_passed.json','factor_audit.json','off_snapshots.json','on_snapshots.json'):
 assert json.loads((H/n).read_text())['passed'],n
for mode in ('off','on'):
 assert json.loads((H/(mode+'_validated.json')).read_text())['passed']
 assert json.loads((V/mode/'validation.json').read_text())['passed']
 assert (P/f'full_{mode}_control/complete.json').exists() and not (P/f'full_{mode}_control/failed.txt').exists()
 assert (K/'inspection'/mode/'complete.txt').exists()
for mode in ('release','san'):
 assert json.loads((K/(mode+'_initial/passed.json')).read_text())['passed']
 assert 'PASS 12 OFF and 12 ON updates, 60 phase snapshots' in (K/(mode+'_initial/observer.log')).read_text()
assert json.loads((K/'release_initial/disabled_parity.json').read_text())['exact']
analysis=json.loads((H/'analysis.json').read_text())
counts=dict(protected=check(json.loads((P/'protected_manifest.json').read_text())),
 snapshots=check(analysis['checkpoint_sha256']),analysis_inputs=check(analysis['analysis_input_sha256']),
 inspection_inputs=check(json.loads((H/'inspection_inputs.json').read_text())),
 factor_audit_inputs=check(json.loads((H/'factor_audit.json').read_text())['input_sha256']))
counts['CPV49_outputs']=check(json.loads((H.parent/'eig2_hesai_refresh_full_cpv49_20261030/complete.json').read_text())['output_sha256'])
jobs={}
for path in [H/'launch.json',H/'start.json',*H.glob('*_process.json')]:
 d=json.loads(path.read_text());pid=d['pid'];proc=Path('/proc',str(pid))
 assert not proc.exists(),('owned host PID still exists',pid,path)
 jobs[path.name]=pid
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
v=dict(passed=True,time=datetime.datetime.now().astimezone().isoformat(),counts=counts,
 containers=containers,owned_jobs_exited=jobs,hesai_originals=hesai,free_bytes=shutil.disk_usage(W).free,
 observer_nonmutation_both_modes=True,source_pairs=1,source_retries=0,estimator_tuning=False,
 covariance_analysis=False,nonlinear_stationarity_certified=False)
with (H/'verification.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v,indent=2))
