"""One saved-data native inspector; no source scripts or optimizer calls."""
import datetime,hashlib,json,os,subprocess,time,traceback
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.parent.name/H.name
Q=W/'runs/eig2_on_checkpoint_cpv44_20261029';H46=H.parent.parent/'eig2_cbs_ablation_cpv46_20261030'
I='/workspace/cbs_gtsam4.3/runs/'+H.parent.name+'/'+H.name;J='/workspace/cbs_gtsam4.3/runs/'+Q.name
K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,h in pins.items():assert sha(p)==h,p
def call(a):return subprocess.check_output(a,text=True)
def run(n,a):
 with (P/(n+'.log')).open('x') as f:
  job=subprocess.Popen(a,stdout=f,stderr=subprocess.STDOUT);save(n+'_process.json',dict(pid=job.pid,args=a,time=time.time()));rc=job.wait()
 assert rc==0,(n,rc)
save('start.json',dict(pid=os.getpid(),time=datetime.datetime.now().astimezone().isoformat()))
pins=json.loads((H.parent/'protected_manifest.json').read_text());pins.update(json.loads((H46/'complete.json').read_text())['output_sha256']);pins[str(H46/'complete.json')]=sha(H46/'complete.json')
for root in (H.parent,P.parent):
 for p in root.rglob('*'):
  if p.is_file() and p != H/'start.json':pins[str(p)]=sha(p)
save('protected_manifest.json',pins);check(pins)
states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited'],states
save('preflight.json',dict(protected=len(pins),containers=states))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((W/'runs/kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for mode in ('release','san'):
  for kind,original in commands.items():
   args=[]
   for a in original:
    if a in ('-I'+K,'-I'+K+'/include'):a=a.replace(K,J)
    if a==K+'/test.cpp':a=I+'/Inspect.cpp'
    elif a==K+'/test.o':a=I+'/'+mode+'.o'
    elif a==K+'/test':a=I+'/'+mode
    args.append(a)
   if mode=='san':
    args=[a for a in args if a!='-O3'];flags=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
    flags+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie'];args[1:1]=flags
   save(mode+'_'+kind+'_command.json',args);run(mode+'_'+kind,['docker','exec','cbsms_ws_gpu',*args])
  env=['-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib']
  if mode=='san':env+=['-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1']
  run(mode+'_selftest',['docker','exec',*env,'cbsms_ws_gpu',I+'/'+mode,'selftest'])
 run('inspection',['docker','exec','-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/release',J+'/checkpoints',I+'/results'])
 for k in range(21,26):
  d=P/'results'/f'update_{k}'
  for stage in ('before_ordinary','after_ordinary','after_refresh'):
   assert sha(d/(stage+'_before.bin'))==sha(d/(stage+'_after.bin'))==sha(Q/'checkpoints'/f'update_{k}'/stage/'checkpoint.bin')
  assert sha(d/'initialized_after.bin')==sha(d/'after_ordinary_before.bin')
 check(pins);save('inspection_passed.json',dict(passed=True,protected=len(pins),nonmutation_snapshots=15,time=time.time()))
except Exception:
 save('failure.json',dict(traceback=traceback.format_exc(),time=time.time()));raise
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args']);assert all(x.split()[1] in ('tail','zsh') for x in top.splitlines()[1:] if x.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 save('cleanup.json',dict(jobs=top,containers=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']).splitlines(),time=time.time()))
