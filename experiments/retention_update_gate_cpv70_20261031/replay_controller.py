"""One exclusive bounded pair after all compatibility gates; no retries."""
import ast,datetime,hashlib,json,os,shutil,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;R=K/'replay'
D=H/'replay_attempt';D.mkdir(exist_ok=False)
def save(name,v):
 with (D/name).open('x') as f:json.dump(v,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert json.loads((H/'build_initial/passed.json').read_text())['passed']
for mode in ('release','san','backend'):assert json.loads((K/(mode+'_initial/passed.json')).read_text())['passed']
for p,v in json.loads((K/'backend_initial/sources.json').read_text()).items():
 local=p.replace('/workspace/cbs_gtsam4.3',str(W));assert sha(local)==v,local
assert shutil.disk_usage(W).free>8*1024**3
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=json.loads((H/'build_initial/protected_before.json').read_text())
audit=json.loads((W/'runs/eig2_retirement_capture_cpv66_20261031/prepared/input_audit.json').read_text())
for v in audit['bag_files'].values():pins[v['path']]=v['sha256']
for p in [*R.rglob('*'),*K.glob('*.h'),*K.glob('*.inc'),*K.glob('*.cpp'),*K.glob('*.py'),*K.glob('include/**/*.h'),*K.glob('private_build/*.so')]:
 if p.is_file():pins[str(p)]=sha(p)
for p in R.glob('*.py'):ast.parse(p.read_text())
for p in R.glob('*.launch'):ET.parse(p)
source=W/'runs/eig2_retirement_capture_cpv66_20261031/prepared/full_on_source/kimera_profile'
for p in source.rglob('*'):
 if p.is_file():assert sha(p)==sha(R/'kimera_profile'/p.relative_to(source))
for p,v in pins.items():assert sha(p)==v,p
save('protected_before.json',pins);save('start.json',dict(pid=os.getpid(),time=time.time(),containers=before))
failure=None
def stage(label,script,*extra):
 args=['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/replay/'+script+' '+' '.join(extra)]
 with (D/(label+'.log')).open('x') as f:
  p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);save(label+'_process.json',dict(pid=p.pid,args=args));rc=p.wait()
 save(label+'_receipt.json',dict(exit=rc));assert rc==0,(label,rc)
 print(label,'PASS',flush=True)
try:
 subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
 old=W/'runs/retention_compatibility_cpv69_20261031/replay_pathcheck'
 assert sha(R/'aligned_inputs.bag')==sha(old/'aligned_inputs.bag')
 assert sha(R/'frozen_inputs.json')==sha(old/'frozen_inputs.json')
 save('freeze_reused.json',dict(byte_identical=True,source=str(old),no_new_freeze_stage=True))
 for mode in ('remove','retain'):
  stage('capture_'+mode,'run_case.py',mode)
  stage('validate_'+mode,'validate.py',mode)
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
save('passed.json',dict(passed=True,completed=datetime.datetime.now().astimezone().isoformat(),scoring_pending=True))
