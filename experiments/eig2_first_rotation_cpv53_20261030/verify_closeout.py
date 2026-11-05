"""Read-only verification of originals; exclusive CP-V53 closeout receipts."""
import datetime,hashlib,json,os,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
R=H.parent.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,v in pins.items():assert sha(p)==v,p
def save(n,d):
 with (H/n).open('x') as f:json.dump(d,f,indent=2)
protected=json.loads((H/'protected_manifest.json').read_text());check(protected)
inputs={}
for name in ('analysis.json','support_audit.json'):
 data=json.loads((H/name).read_text());inputs.update(data['input_sha256'])
check(inputs)
original=W/'runs/eig2_hesai_insertion_cpv51_20261030/prepared/full_on_source/checkpoints/update_6/after_ordinary/checkpoint.bin'
assert sha(original)==sha(K/'extracted/before.bin')==sha(K/'extracted/after.bin')
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
owned=[]
for p in [H/'start.json',*H.glob('*_process.json')]:
 d=json.loads(p.read_text());pid=d['pid'];q=Path('/proc')/str(pid)/'cmdline'
 cmd=q.read_bytes().replace(b'\0',b' ').decode(errors='replace') if q.exists() else ''
 # PID reuse is not evidence that the old job still runs; record it explicitly.
 assert not (H.name in cmd), (pid,cmd)
 owned.append(dict(receipt=str(p),pid=pid,present=bool(cmd),current_cmdline=cmd))
allps=subprocess.check_output(['ps','-eo','pid,args'],text=True)
active=[s for s in allps.splitlines() if H.name in s and any(t in s for t in ('extract_native.py','analyze_v','/release ','/san ','Extract.cpp'))]
assert not active,active
verifier=R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
result=subprocess.run(['python3',str(verifier),'--originals'],capture_output=True,text=True)
assert result.returncode==0,(result.stdout,result.stderr)
assert 'PASS: 4 cases, 667 saved files; originals=True' in result.stdout
v=dict(passed=True,time=datetime.datetime.now().astimezone().isoformat(),protected_count=len(protected),
       analysis_support_input_count=len(inputs),snapshot_byte_identity=True,containers=states,
       owned_jobs=owned,active_owned_jobs=active,hesai_checkpoint=result.stdout,hesai_verifier_sha256=sha(verifier),
       source_replays=0,graph_optimizer_calls=0,covariance_analysis=False)
save('verification.json',v)
outputs={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file()}
save('complete.json',dict(passed=True,time=v['time'],result='First-edge native disagreement confirmed; audited transport/time bridge passes; root cause remains open.',
                         verification=v,output_sha256=outputs))
print(json.dumps({k:v[k] for k in ('passed','time','protected_count','analysis_support_input_count','containers','hesai_checkpoint')},indent=2))
