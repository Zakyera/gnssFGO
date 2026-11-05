"""Exclusive CP69 build gate, exact owned processes, stopped-container restoration."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
attempt=sys.argv[1];D=H/('build_'+attempt);D.mkdir(exist_ok=False)
def save(name,v):
 with (D/name).open('x') as f:json.dump(v,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
assert sha(P/'private_build/libcbs.so')==sha(W/'runs/eig2_retirement_capture_cpv66_20261031/private_build/libcbs.so')
pins=json.loads((W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/protected_manifest.json').read_text())
old=H.parent/'retention_compatibility_cpv69_20261031'
pins.update(json.loads((old/'complete.json').read_text())['output_sha256'])
pins[str(old/'complete.json')]=sha(old/'complete.json')
for root in (W/'runs/eig2_retirement_capture_cpv66_20261031',W/'runs/kimera_on_incremental_refresh_cpv40_20261029'):
 for p in [*root.glob('*.h'),*root.glob('*.inc'),*root.glob('*.cpp'),*root.glob('include/**/*.h'),*root.glob('private_build/*.so')]:pins[str(p)]=sha(p)
for p,v in pins.items():assert sha(p)==v,p
save('protected_before.json',pins);save('start.json',dict(time=time.time(),containers=before))
failure=None
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 for mode in ('release','san','backend'):
  args=['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/build.py '+mode+' '+attempt]
  with (D/(mode+'.log')).open('x') as f:
   job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(mode+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
  save(mode+'_receipt.json',dict(exit=rc));assert rc==0,(mode,rc)
except BaseException as exc:
 failure=repr(exc);save('failed.json',dict(error=failure))
finally:
 top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
 assert all(line.split()[1] in ('tail','zsh') for line in top.splitlines()[1:] if line.strip()),top
 subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
 save('cleanup.json',dict(containers=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']),jobs=top))
 for p,v in pins.items():assert sha(p)==v,p
 save('protected_after.json',dict(unchanged=True,count=len(pins)))
if failure:raise SystemExit(1)
save('passed.json',dict(passed=True,backend_sha256=sha(P/'private_build/libkimera_vio.so')))
