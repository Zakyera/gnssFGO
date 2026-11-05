"""Supplemental saved-only actual-factor mean/prior identity audit."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
I='/workspace/cbs_gtsam4.3/runs/'+H.name;K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
D=P/'factor_inspection';assert not (D/'compile.log').exists()
assert json.loads((H/'inspection_passed.json').read_text())['passed']
def call(args):return subprocess.check_output(args,text=True)
def save(name,v):
 with (H/name).open('x') as f:json.dump(v,f,indent=2)
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
save('factors_start.json',dict(time=time.time(),source_sha256=hashlib.sha256((D/'Factors.cpp').read_bytes()).hexdigest()))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((P.parent/'kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for kind,args in commands.items():
  args=[a.replace(K,I).replace(I+'/test.cpp',I+'/factor_inspection/Factors.cpp').replace(I+'/test.o',I+'/factor_inspection/Factors.o') for a in args]
  args=[I+'/factor_inspection/Factors' if a==I+'/test' else a for a in args]
  with (D/(kind+'_command.json')).open('x') as f:json.dump(args,f,indent=2)
  with (D/(kind+'.log')).open('x') as f:
   job=subprocess.Popen(['docker','exec','cbsms_ws_gpu',*args],stdout=f,stderr=subprocess.STDOUT);save('factors_'+kind+'_process.json',dict(pid=job.pid));rc=job.wait()
  assert rc==0,(kind,rc)
 for mode in ('off','on'):
  args=['docker','exec','-e','LD_LIBRARY_PATH='+I+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/factor_inspection/Factors',I+'/prepared/full_'+mode+'_source/checkpoints',I+'/factor_inspection/'+mode]
  with (D/(mode+'.log')).open('x') as f:
   job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save('factors_'+mode+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
  assert rc==0,mode
 save('factors_passed.json',dict(passed=True,time=time.time()))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
 assert after==before;save('factors_cleanup.json',dict(jobs=top,containers=after))
