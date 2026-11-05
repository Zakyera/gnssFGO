"""Exclusive private observer build and release/sanitizer fixtures; no sources."""
import hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
I='/workspace/cbs_gtsam4.3/runs/'+H.name;S='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
def save(n,x):
 with (H/n).open('x') as f:json.dump(x,f,indent=2)
def call(a):return subprocess.check_output(a,text=True)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]);assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=json.loads((H.parent/'eig2_first_rotation_cpv53_20261030/protected_manifest.json').read_text())
for p,w in pins.items():assert sha(p)==w,p
save('protected_before.json',pins);save('build_start.json',dict(pid=os.getpid(),time=time.time(),containers=before))
recipes=json.loads((W/'runs/local_revision_covariance_cpi1_20261019/release_commands.json').read_text())
def run(label,cmd,cwd='/workspace/cbs_gtsam4.3/build/glim_ros'):
 args=['docker','exec','-w',cwd,'-e','LD_LIBRARY_PATH=/workspace/cbs_gtsam4.3/devel/lib:/workspace/cbs_gtsam4.3/devel:/opt/ros/noetic/lib:/usr/local/cuda/lib64','-e','ASAN_OPTIONS=detect_leaks=0','-e','UBSAN_OPTIONS=halt_on_error=1','cbsms_ws_gpu',*cmd]
 with (H/(label+'.log')).open('x') as f:
  j=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(label+'_process.json',dict(pid=j.pid,command=args));rc=j.wait()
 save(label+'_receipt.json',dict(exit=rc));assert rc==0,(label,rc)
failure=None
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 for r in recipes[:2]:run(r['name'],[s.replace(S,I) for s in r['command']],r['cwd'])
 base=[s.replace(S,I) for s in recipes[0]['command']]
 base=base[:base.index('-c')]
 base=[s for s in base if s not in ('-fPIC','-DNDEBUG','-O3','-O2')]
 for kind in ('release','san'):
  flags=['-O2'] if kind=='release' else ['-O0','-g1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
  run(kind+'_compile',base+flags+[I+'/Test.cpp','-L/workspace/cbs_gtsam4.3/devel/lib','-lgtsam','-lboost_serialization','-o',I+'/test_'+kind])
  run(kind+'_test',[I+'/test_'+kind])
except BaseException as e:failure=repr(e);save('build_failed.json',dict(error=failure))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(s.split()[1] in ('tail','zsh') for s in top.splitlines()[1:] if s.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 final=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]);assert final==before
 save('build_cleanup.json',dict(containers=final,jobs=top))
if failure:raise SystemExit(1)
for p,w in pins.items():assert sha(p)==w,p
save('build_passed.json',dict(passed=True,bridge_sha256=sha(K/'libglim_ros.so'),protected_files=len(pins)))
