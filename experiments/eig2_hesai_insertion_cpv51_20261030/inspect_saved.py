"""Saved-only native phase inspection after matched capture/controller exit."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
I='/workspace/cbs_gtsam4.3/runs/'+H.name;K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
D=P/'inspection';assert not (D/'compile.log').exists()
assert json.loads((H/'capture_complete.json').read_text())['passed']
def call(args):return subprocess.check_output(args,text=True)
def save(name,v):
 with (H/name).open('x') as f:json.dump(v,f,indent=2)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for name in ('start.json','launch.json'):
 pid=json.loads((H/name).read_text())['pid'];assert not Path('/proc',str(pid)).exists(),('capture job still exists',pid)
inputs={str(p):sha(p) for mode in ('off','on') for p in (P/f'prepared/full_{mode}_source/checkpoints').rglob('*') if p.is_file()}
assert sum(p.endswith('checkpoint.bin') for p in inputs)==120
inputs[str(D/'Inspect.cpp')]=sha(D/'Inspect.cpp')
save('inspection_inputs.json',inputs);save('inspection_start.json',dict(time=time.time()))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((P.parent/'kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for kind,args in commands.items():
  args=[a.replace(K,I).replace(I+'/test.cpp',I+'/inspection/Inspect.cpp').replace(I+'/test.o',I+'/inspection/Inspect.o') for a in args]
  args=[I+'/inspection/Inspect' if a==I+'/test' else a for a in args]
  with (D/(kind+'_command.json')).open('x') as f:json.dump(args,f,indent=2)
  with (D/(kind+'.log')).open('x') as f:
   job=subprocess.Popen(['docker','exec','cbsms_ws_gpu',*args],stdout=f,stderr=subprocess.STDOUT)
   save('inspection_'+kind+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
  assert rc==0,(kind,rc)
 for mode in ('off','on'):
  args=['docker','exec','-e','LD_LIBRARY_PATH='+I+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/inspection/Inspect',I+'/prepared/full_'+mode+'_source/checkpoints',I+'/inspection/'+mode]
  with (D/(mode+'.log')).open('x') as f:
   job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save('inspection_'+mode+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
  save('inspection_'+mode+'_receipt.json',dict(exit=rc));assert rc==0,mode
 for p,want in inputs.items():assert sha(p)==want,p
 save('inspection_passed.json',dict(passed=True,time=time.time(),snapshots=120))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
 assert after==before
 save('inspection_cleanup.json',dict(jobs=top,containers=after))
