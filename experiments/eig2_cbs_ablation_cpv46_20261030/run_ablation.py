"""Once-only offline equation experiment; no source/campaign controller called."""
import datetime,hashlib,json,os,subprocess,time,traceback
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
Q=W/'runs/eig2_on_checkpoint_cpv44_20261029'
H45=H.parent/'eig2_on_convergence_cpv45_20261029'
I='/workspace/cbs_gtsam4.3/runs/'+H.name
J='/workspace/cbs_gtsam4.3/runs/'+Q.name
K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
def save(name,data):
 with (H/name).open('x') as f:json.dump(data,f,indent=2)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,h in pins.items():assert sha(p)==h,p
def call(args):return subprocess.check_output(args,text=True)
def run(label,args):
 with (P/(label+'.log')).open('x') as f:
  proc=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
  save(label+'_process.json',dict(pid=proc.pid,args=args,time=time.time()))
  rc=proc.wait()
 assert rc==0,(label,rc)
save('start.json',dict(pid=os.getpid(),time=datetime.datetime.now().astimezone().isoformat()))
pins=json.loads((H45/'protected_manifest.json').read_text())
v=json.loads((H45/'complete.json').read_text());pins.update(v['output_sha256']);pins.update(v['code_evidence_sha256'])
pins[str(H45/'complete.json')]=sha(H45/'complete.json')
for root in (H,P):
 for p in root.rglob('*'):
  if p.is_file() and p.name!='start.json':pins[str(p)]=sha(p)
save('protected_manifest.json',pins);check(pins)
states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited'],states
free=os.statvfs(W).f_bavail*os.statvfs(W).f_frsize;assert free>8*1024**3,free
save('preflight.json',dict(protected=len(pins),containers=states,free_bytes=free))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((W/'runs/kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for mode in ('release','san'):
  for kind,original in commands.items():
   args=[]
   for a in original:
    if a in ('-I'+K,'-I'+K+'/include'):a=a.replace(K,J)
    if a==K+'/test.cpp':a=I+'/Ablation.cpp'
    elif a==K+'/test.o':a=I+'/'+mode+'.o'
    elif a==K+'/test':a=I+'/'+mode
    args.append(a)
   if mode=='san':
    args=[a for a in args if a!='-O3']
    flags=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
    flags+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie']
    args[1:1]=flags
   save(mode+'_'+kind+'_command.json',args)
   run(mode+'_'+kind,['docker','exec','cbsms_ws_gpu',*args])
  env=['-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib']
  if mode=='san':env+=['-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1']
  run(mode+'_selftest',['docker','exec',*env,'cbsms_ws_gpu',I+'/'+mode,'selftest'])
 run('ablation',['docker','exec','-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/release',J+'/checkpoints',I+'/results'])
 for k in range(21,26):
  d=P/'results'/('update_'+str(k));assert sha(d/'before.bin')==sha(d/'after.bin')==sha(Q/'checkpoints'/('update_'+str(k))/'after_ordinary/checkpoint.bin')
 check(pins)
 save('ablation_passed.json',dict(passed=True,protected=len(pins),nonmutation_snapshots=5,finished=datetime.datetime.now().astimezone().isoformat()))
except Exception:
 save('failure.json',dict(traceback=traceback.format_exc(),time=time.time()));raise
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']).splitlines()
 save('cleanup.json',dict(jobs=top,containers=states,time=time.time()))
