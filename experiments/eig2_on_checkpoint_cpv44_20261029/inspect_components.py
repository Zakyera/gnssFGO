"""Once-only offline native checkpoint equation_components, stopped-container envelope."""
import json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
I='/workspace/cbs_gtsam4.3/runs/'+H.name
K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
D=P/'equation_components';assert not (D/'compile.log').exists()
assert json.loads((H/'complete.json').read_text())['passed']
assert len(list((P/'checkpoints').glob('update_*/*/checkpoint.bin')))==15
def call(args):return subprocess.check_output(args,text=True)
def save(name,v):
 with (H/name).open('x') as f:json.dump(v,f,indent=2)
assert call(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu']).strip()=='exited'
save('equation_components_start.json',dict(time=time.time()))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((P.parent/'kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for kind,args in commands.items():
  args=[a.replace(K,I).replace(I+'/test.cpp',I+'/equation_components/Components.cpp').replace(I+'/test.o',I+'/equation_components/Components.o') for a in args]
  args=[I+'/equation_components/Components' if a==I+'/test' else a for a in args]
  with (D/(kind+'_command.json')).open('x') as f:json.dump(args,f,indent=2)
  with (D/(kind+'.log')).open('x') as f:
   job=subprocess.Popen(['docker','exec','cbsms_ws_gpu',*args],stdout=f,stderr=subprocess.STDOUT)
   save('equation_components_'+kind+'_process.json',dict(pid=job.pid));rc=job.wait()
  assert rc==0,(kind,rc)
 with (D/'analysis.log').open('x') as f:
  args=['docker','exec','-e','LD_LIBRARY_PATH='+I+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/equation_components/Components',I+'/checkpoints',I+'/equation_components/results']
  job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save('equation_components_analysis_process.json',dict(pid=job.pid,args=args));rc=job.wait()
 assert rc==0,('analysis',rc)
 save('equation_components_passed.json',dict(passed=True,time=time.time()))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args']);assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 save('equation_components_cleanup.json',dict(jobs=top,container=call(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu']).strip()))
