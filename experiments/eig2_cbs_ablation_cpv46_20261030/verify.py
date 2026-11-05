"""Final read-only preservation checks, one new immutable closeout receipt."""
import csv,datetime,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
Q=W/'runs/eig2_on_checkpoint_cpv44_20261029'
def load(p):return json.loads(p.read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,h in pins.items():assert sha(p)==h,p
assert not (H/'complete.json').exists() and not (H/'failure.json').exists()
assert load(H/'ablation_passed.json')['passed']
pins=load(H/'protected_manifest.json');check(pins)
a=load(H/'analysis.json');check(a['input_sha256'])
assert a['one_step_solves']==15 and a['source_replays']==0 and a['iterative_optimizations']==0
assert [r['update'] for r in a['updates']]==list(range(21,26))
peers=0
for r in a['updates']:
 k=r['update'];d=P/'results'/f'update_{k}'
 assert sha(d/'before.bin')==sha(d/'after.bin')==sha(Q/'checkpoints'/f'update_{k}'/'after_ordinary/checkpoint.bin')
 assert r['parity']['cached_fresh_relative']==0
 assert r['parity']['captured_ordinary_tangent_difference']<1e-6 and r['parity']['exact_peer_fd_max']<1e-7
 assert r['initial_old_movement']['maximum_position_change_m']==0
 for mode in 'ABC':
  z=r['arms'][mode];assert z['equations']['active_peer_factors']==(0 if mode=='A' else k-4)
  assert z['equations']['linear_backward_relative']<1e-14
  assert z['equations']['native_factors']==r['arms']['B']['equations']['native_factors']
 assert r['arms']['A']['from_previous']['common_world_tilt_deg']>r['arms']['B']['from_previous']['common_world_tilt_deg']
 assert r['arms']['A']['cost']['native']>r['arms']['B']['cost']['native']
 assert r['C_vs_B']['common_world_rotation_deg']<.051
 assert r['C_vs_B']['maximum_position_change_m']<.003
 peers+=int(r['arms']['B']['equations']['active_peer_factors'])
assert peers==95
for mode in ('release','san'):assert (P/(mode+'_selftest.log')).read_text().startswith('PASS ')
assert (P/'results/all_B_passed.txt').exists() and (P/'results/complete.txt').exists()
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
owned=[]
for p in [H/'start.json',*H.glob('*_process.json')]:
 r=load(p);pid=r['pid'];proc=Path('/proc')/str(pid)/'cmdline'
 present=proc.exists() and H.name.encode() in proc.read_bytes()
 assert not present,(p,pid);owned.append(pid)
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
result=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),protected_files=len(pins),
 snapshot_nonmutation=5,baseline_reproduction_passed=True,exact_peer_fd_checks=peers*36,
 source_replays=0,independent_one_step_solves=15,iterative_optimizations=0,
 covariance_investigated=False,accuracy_scored=False,containers=states,owned_jobs_exited=owned,hesai=hesai,
 output_sha256={str(p):sha(p) for root in (H,P) for p in root.rglob('*') if p.is_file()})
with (H/'complete.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k!='output_sha256'},indent=2))
