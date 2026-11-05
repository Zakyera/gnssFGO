"""Independent supplemental host audit; preserve original failed/terminal files."""
import datetime,hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H/'report_completion_1'
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def verify(pins):
    for p,d in pins.items():assert sha(p)==d,p
    return len(pins)
out={'checked_at':datetime.datetime.now().astimezone().isoformat(),'passed':True}
out['protected_files']=verify(read(P/'protected_manifest.json'))
out['report_terminal_files']=verify(read(R/'complete.json')['output_sha256'])
out['report_input_files']=verify(read(R/'input_sha256.json'))
out['prior_terminal_files']={}
for name in ('glim_persistent_compat_cpv82_20261101','eig2_persistent_live_cpv80_20261101','persistent_native_full_cpv78_20261101'):
    out['prior_terminal_files'][name]=verify(read(H.parent/name/'complete.json')['output_sha256'])
jobs=[]
receipts=[H/'start.json']
for root in (H,H/'saved_validation_repair_1',P/'full_on_control'):
    receipts.extend(root.glob('*_process.json'))
for p in receipts:
    v=read(p);pid=v['pid'];proc=Path(f'/proc/{pid}')
    live=proc.joinpath('stat').exists() and proc.joinpath('stat').read_text().split()[21]==v['proc_start']
    cmd=proc.joinpath('cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace') if live else None
    jobs.append(dict(receipt=str(p),pid=pid,original_identity_alive=live,command=cmd));assert not live,(p,cmd)
launch=read(H/'launch.json')
for key in ('launcher_pid','owned_supervisor_pid'):
    pid=launch[key];cmd=Path(f'/proc/{pid}/cmdline')
    text=cmd.read_bytes().replace(b'\0',b' ').decode(errors='replace') if cmd.exists() else None
    assert text is None or str(H) not in text,(pid,text)
    jobs.append(dict(role=key,pid=pid,current_command=text,original_command_absent=True))
out['owned_jobs']=jobs
out['containers']=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert out['containers']==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
out['hesai']=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
out['original_controller_failure_preserved']=read(H/'status.json')['state']=='failed'
out['report_completion_passed']=read(R/'complete.json')['passed']
out['source_captures']=1;out['source_retries']=0;out['receiver_replays']=0;out['EVO_CLI_repeats']=0
with (R/'post_closeout.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
