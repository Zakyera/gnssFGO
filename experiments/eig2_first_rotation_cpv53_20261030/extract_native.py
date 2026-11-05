"""Private native PIM extraction, never source replay or graph optimization."""
import datetime,hashlib,json,os,subprocess,time,traceback
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
H52=H.parent/'eig2_first_factor_cpv52_20261030'
Q=W/'runs/eig2_hesai_insertion_cpv51_20261030'
I='/workspace/cbs_gtsam4.3/runs/'+H.name;J='/workspace/cbs_gtsam4.3/runs/'+Q.name
K='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,d):
 with (H/n).open('x') as f:json.dump(d,f,indent=2)
def check(pins):
 for p,h in pins.items():assert sha(p)==h,p
def run(label,args):
 with (P/(label+'.log')).open('x') as f:
  job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(label+'_process.json',dict(pid=job.pid,args=args,time=time.time()));rc=job.wait()
 assert rc==0,(label,rc)
save('start.json',dict(pid=os.getpid(),time=datetime.datetime.now().astimezone().isoformat()))
pins=json.loads((H52/'attempt_compile1/protected_manifest.json').read_text())
pins.update(json.loads((H52/'complete.json').read_text())['output_sha256']);pins[str(H52/'complete.json')]=sha(H52/'complete.json')
for root in (H,P):
 for p in root.rglob('*'):
  if p.is_file() and p!=H/'start.json':pins[str(p)]=sha(p)
save('protected_manifest.json',pins);check(pins)
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
assert os.statvfs(P).f_bavail*os.statvfs(P).f_frsize>4*1024**3
save('preflight.json',dict(containers=states,protected=len(pins)))
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 commands=json.loads((W/'runs/kimera_on_incremental_refresh_cpv40_20261029/actual_test_commands.json').read_text())
 for mode in ('release','san'):
  for kind,original in commands.items():
   args=[]
   for item in original:
    if item in ('-I'+K,'-I'+K+'/include'):item=item.replace(K,J)
    item={K+'/test.cpp':I+'/Extract.cpp',K+'/test.o':I+'/'+mode+'.o',K+'/test':I+'/'+mode}.get(item,item)
    args.append(item)
   if mode=='san':
    args=[a for a in args if a!='-O3'];args[1:1]=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']+(['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie'])
   save(mode+'_'+kind+'_command.json',args);run(mode+'_'+kind,['docker','exec','cbsms_ws_gpu',*args])
  env=['-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib']
  if mode=='san':env+=['-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1']
  run(mode+'_selftest',['docker','exec',*env,'cbsms_ws_gpu',I+'/'+mode,'selftest'])
 run('extract',['docker','exec','-e','LD_LIBRARY_PATH='+K+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu',I+'/release',J+'/prepared/full_on_source/checkpoints/update_6/after_ordinary/checkpoint.bin',I+'/extracted'])
 original=Q/'prepared/full_on_source/checkpoints/update_6/after_ordinary/checkpoint.bin'
 assert sha(original)==sha(P/'extracted/before.bin')==sha(P/'extracted/after.bin');check(pins)
 save('extraction_passed.json',dict(passed=True,optimizer_calls=0,protected=len(pins)))
except Exception:
 save('failure.json',dict(traceback=traceback.format_exc(),time=time.time()));raise
finally:
 top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True)
 assert all(r.split()[1] in ('tail','zsh') for r in top.splitlines()[1:] if r.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 after=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
 assert after==states;save('cleanup.json',dict(jobs=top,containers=after,time=time.time()))
