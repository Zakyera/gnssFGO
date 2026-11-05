"""Exclusive build envelope with originally-stopped Noetic restoration."""
import hashlib,json,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name
D=H/'build_attempt_phase2';D.mkdir(exist_ok=False)
def save(name,data):
 with (D/name).open('x') as f:json.dump(data,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
save('build_start.json',dict(time=time.time(),containers=before))
assert json.loads((H/'build_failed.json').read_text())['error']=="AssertionError(('release', 1))"
old=W/'runs/eig2_hesai_insertion_cpv51_20261030/private_build/libcbs.so'
assert (P/'private_build/libcbs.so').read_bytes()==old.read_bytes()
save('repair.json',dict(reason='Previous fixture checked wrapper presence AFTER its complete absorption. Move the presence assertion to saved pre-marginalization phases (at least14); all state/cache/function nonmutation gates unchanged. Failed sources/logs preserved. No source capture ran.',dependency=str(old)))
pins=json.loads((W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/protected_manifest.json').read_text())
for p,want in pins.items():
 with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==want,p
save('protected_before.json',pins)
failure=None
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 for mode in ('release','san','backend'):
  args=['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/build.py '+mode+' phase2']
  with (D/('build_'+mode+'.log')).open('x') as f:
   job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(mode+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
  save(mode+'_receipt.json',dict(exit=rc));assert rc==0,(mode,rc)
except BaseException as exc:
 failure=repr(exc);save('build_failed.json',dict(error=failure))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(line.split()[1] in ('tail','zsh') for line in top.splitlines()[1:] if line.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 save('build_cleanup.json',dict(containers=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']),jobs=top))
if failure:raise SystemExit(1)
for p,want in pins.items():
 with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==want,p
with (P/'private_build/libkimera_vio.so').open('rb') as f:sha=hashlib.file_digest(f,'sha256').hexdigest()
save('build_passed.json',dict(passed=True,backend_sha256=sha))
with (H/'build_passed.json').open('x') as f:json.dump(dict(passed=True,backend_sha256=sha,attempt=str(D)),f,indent=2)
