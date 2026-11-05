"""Post-exit integrity and report closeout; no source/scoring invocation."""
import datetime,hashlib,json,subprocess
from pathlib import Path
Q=Path(__file__).resolve().parent;H=Q.parent;E=H.parent;F=H/'saved_report_1'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not (Q/'complete.json').exists() and read(Q/'report_verification.json')['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
receipts=[read(p) for p in H.glob('*_process.json')]+[read(H/'start.json'),read(F/'evaluate_process.json')]
exited=[]
for r in receipts:
    proc=Path('/proc')/str(r['pid']);stat=proc/'stat'
    assert not stat.exists() or stat.read_text().split()[21]!=r['proc_start'],r
    exited.append(dict(pid=r['pid'],proc_start=r['proc_start'],same_identity_present=False))
for root in (H,E/'eth1_split_imu_off60_cpv85_20261101'):
    launch=read(root/'launch.json');pid=launch['supervisor_pid']
    assert not Path(f'/proc/{pid}').exists()
    exited.append(dict(pid=pid,role='supervisor',command=launch['command'],present=False))
pins=read(P/'protected_manifest.json')
for p,d in pins.items():assert sha(Path(p))==d,p
inputs=read(Q/'input_sha256.json')
for p,d in inputs.items():assert sha(Path(p))==d,p
aud=read(P/'input_audit.json');assert sha(Path(aud['tf']['path']))==aud['tf']['sha256']
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(Q/'post_closeout.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
    containers=states,protected_files=len(pins),report_inputs=len(inputs),owned_exits=exited,hesai=hesai,
    source_captures=1,preplayback_failed_launches=1,additional_source_retries=0,additional_EVO_CLI_calls=0))
outputs={}
for root in (Q,F,P/'off_source'):
    for p in root.rglob('*'):
        if p.is_file():outputs[str(p)]=sha(p)
for name in ('failed.json','closeout.json','launch.json','start.json','capture_receipt.json','controller.log'):outputs[str(H/name)]=sha(H/name)
save(Q/'complete.json',dict(report_complete=True,pilot_health_passed=False,
    original_strict_guard_passed=False,endpoint_audit_passed=True,
    finished=datetime.datetime.now().astimezone().isoformat(),output_sha256=outputs))
print('Closeout PASS:',len(pins),'protected;',len(inputs),'report inputs;',len(outputs),'outputs;',hesai.strip())
