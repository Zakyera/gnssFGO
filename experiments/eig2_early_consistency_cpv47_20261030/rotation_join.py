"""Additional read-only rotation identity, fresh outputs; no solver/replay."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
I='/workspace/cbs_gtsam4.3/runs/'+H.name;J='/workspace/cbs_gtsam4.3/runs/eig2_on_checkpoint_cpv44_20261029'
K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def run(n,args):
 with (P/(n+'.log')).open('x') as f:
  job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(n+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
 assert rc==0,(n,rc)
assert subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu'],text=True).strip()=='exited'
save('rotation_join_start.json',dict(time=time.time()))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 for kind in ('compile','link'):
  args=json.loads((H/'attempt_api1'/('release_'+kind+'_command.json')).read_text())
  args=[a.replace(I+'/attempt_api1/Inspect.cpp',I+'/RotationJoin.cpp').replace(I+'/attempt_api1/release.o',I+'/RotationJoin.o').replace(I+'/attempt_api1/release',I+'/RotationJoin') for a in args]
  save('rotation_join_'+kind+'_command.json',args);run('rotation_join_'+kind,['docker','exec','cbsms_ws_gpu',*args])
 run('rotation_join',['docker','exec','-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/RotationJoin',J+'/checkpoints'])
finally:
 top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True);assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True);save('rotation_join_cleanup.json',dict(top=top,time=time.time()))
