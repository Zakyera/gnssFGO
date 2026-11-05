"""Final non-mutating evidence check; writes one new closeout receipt only."""
import csv,datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
Q=W/'runs/eig2_on_checkpoint_cpv44_20261029'
def load(p):return json.loads(p.read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,h in pins.items():assert sha(p)==h,p
assert not (H/'complete.json').exists()
assert load(H/'audit_passed.json')['passed']
pins=load(H/'protected_manifest.json');check(pins)
a=load(H/'analysis.json');check(a['input_sha256'])
assert a['native_convergence_admitted'] is False and a['iterative_optimizations']==0
assert [r['update'] for r in a['updates']]==list(range(21,26))
old=load(H.parent/Q.name/'analysis.json')
total=failed=peer_instances=chain_checks=0
for r,oldr in zip(a['updates'],old['updates']):
 k=r['update'];d=P/'results'/f'update_{k}'
 assert sha(d/'before.bin')==sha(d/'after.bin')==sha(Q/'checkpoints'/f'update_{k}'/'after_refresh/checkpoint.bin')
 assert r['captured_cost']['total']==oldr['costs']['refresh']['total']
 assert len(r['probes'])==13
 assert r['native_equations']['all_native_gate']==0
 assert r['native_equations']['activity_fail_checks']==0
 assert r['peer_gradient']['chain_failed_checks']==0
 assert r['native_equations']['peer_chain_jacobian_max']<1e-9
 assert r['native_equations']['linear_backward_relative']<1e-14
 assert r['derivative_families']['imu']['failed_checks']==0
 assert r['derivative_families']['bias_between']['failed_checks']==0
 assert r['probes'][0]['from_before_ordinary']['common_world_tilt_deg']>r['captured_net_movement']['common_world_tilt_deg']
 assert any(p['lower_reported_cost'] and p['status_changes']==0 for p in r['probes'])
 for z in r['derivative_families'].values():total+=z['checks'];failed+=z['failed_checks']
 peer_instances+=r['derivative_families']['peer_pose']['factors'];chain_checks+=r['peer_gradient']['checks']
assert (total,failed,peer_instances,chain_checks)==(26580,8694,95,3420)
assert sum(r['probes'][0]['lower_reported_cost'] for r in a['updates'])==4
for mode in ('release','san'):assert (P/(mode+'_selftest.log')).read_text().startswith('PASS ')
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
owned=[]
for p in [H/'start.json',*H.glob('*_process.json')]:
 r=load(p);pid=r['pid'];proc=Path('/proc')/str(pid)/'cmdline'
 present=proc.exists() and H.name.encode() in proc.read_bytes()
 assert not present,(p,pid);owned.append(pid)
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
code=[W/'devel/include/gtsam/config.h',W/'devel/include/gtsam/slam/BetweenFactor.h',W/'devel/include/gtsam/nonlinear/ExtendedPriorFactor.h',W/'src/cbs/src/bpsam/bpsam.cpp']
result=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),protected_files=len(pins),
 snapshot_nonmutation=5,native_derivative_admission_passed=False,derivative_gate_checks=total,failed_gate_checks=failed,
 exact_peer_chain_checks=chain_checks,source_replays=0,iterative_optimizations=0,fixed_one_direction_probes=65,
 covariance_investigated=False,accuracy_scored=False,containers=states,owned_jobs_exited=owned,hesai=hesai,
 output_sha256={str(p):sha(p) for root in (H,P) for p in root.rglob('*') if p.is_file()},
 code_evidence_sha256={str(p):sha(p) for p in code})
with (H/'complete.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ('output_sha256','code_evidence_sha256')},indent=2))
