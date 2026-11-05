"""Fixture-only repair: remove Boost serialization singleton UB from test setup.
Use exact round-trip-decimal fingerprints of all PIM numerical state/Jacobians
and covariance, preserving zero-tolerance comparisons and sanitizer enforcement.
No serialization is used in the live observer. Original failures kept.
"""
import hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']
before=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names],text=True)
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
failure=None
subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
try:
 for kind in ('release','san'):
  for action in ('compile','test'):
   cmd=json.loads((H/('release_'+action+'_process.json')).read_text())['command']
   cmd=[s.replace('/Test.cpp','/Test_v4.cpp').replace('/test_release','/test_v4_'+kind) for s in cmd]
   if kind=='san' and action=='compile':
    cmd=[s for s in cmd if s!='-O2'];i=cmd.index('/usr/bin/c++')+1
    cmd[i:i]=['-O0','-g1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
   label='v4_'+kind+'_'+action
   with (H/(label+'.log')).open('x') as f:
    j=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT);save(label+'_process.json',dict(pid=j.pid,command=cmd));rc=j.wait()
   save(label+'_receipt.json',dict(exit=rc));assert rc==0,(label,rc)
except BaseException as e:failure=repr(e);save('v4_failed.json',dict(error=failure))
finally:
 top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True)
 assert all(s.split()[1] in ('tail','zsh') for s in top.splitlines()[1:] if s.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 final=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names],text=True);assert final==before
 save('v4_cleanup.json',dict(containers=final,jobs=top))
if failure:raise SystemExit(1)
pins=json.loads((H/'protected_before.json').read_text())
for p,w in pins.items():assert sha(p)==w,p
save('build_passed.json',dict(passed=True,bridge_sha256=sha(K/'libglim_ros.so'),protected_files=len(pins),fixture_repair='exact numerical PIM identity; strict-zero-tolerance API misuse corrected; serialization singleton excluded from fixture; observer unchanged'))
