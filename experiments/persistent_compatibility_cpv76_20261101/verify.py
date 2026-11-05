"""Read-only closeout verification; new receipt only, no source execution."""
from pathlib import Path
from datetime import datetime
import hashlib,json,subprocess,sys
H=Path(__file__).resolve().parent;R=H.parent.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
attempt=sys.argv[1]
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def js(p):return json.loads(Path(p).read_text())
assert js(H/('build_'+attempt)/'passed.json')['passed']
pins=js(H/'protected_before.json')
for p,v in pins.items():assert sha(p)==v,p
old=js(H.parent/'retention_readiness_cpv73_20261031/complete.json')['output_sha256']
for p,v in old.items():assert sha(p)==v,p
for mode in ('release','san','backend'):
 d=P/(mode+'_'+attempt);assert js(d/'passed.json')['passed']
 for p,v in js(d/'sources.json').items():assert sha(p.replace('/workspace/cbs_gtsam4.3',str(W),1))==v,p
assert js(P/('release_'+attempt)/'disabled_parity.json')['exact']
status=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert status=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
git={name:subprocess.check_output(['git','-C',str(W/'src'/name),'status','--short'],text=True) for name in ('cbs','Kimera-VIO')}
assert all(not v for v in git.values()),git
checkpoint=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
libraries={p.name:sha(p) for p in (P/'private_build').glob('*.so')}
processes={}
for p in (H/('build_'+attempt)).glob('*_process.json'):
 pid=js(p)['pid'];alive=Path('/proc')/str(pid)
 processes[str(pid)]=alive.exists();assert not alive.exists(),pid
receipt=dict(passed=True,completed=datetime.now().astimezone().isoformat(),attempt=attempt,
 predecessor_inputs=len(pins),CP73_outputs_unchanged=len(old),
 libraries=libraries,containers=status,source_worktrees=git,owned_processes_alive=processes,
 checkpoint_verification=checkpoint,source_replay_launched=False,default_promoted=False,
 vendor_libraries_sanitized=False,live_bidirectional_native_only_LOCAL_certified=False)
with (H/('verification_'+attempt+'.json')).open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps(receipt,indent=2))
