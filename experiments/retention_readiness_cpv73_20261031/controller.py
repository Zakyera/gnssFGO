"""Finite full pair: reused frozen inputs, remove/retain gates, EVO, closeout. No retries."""
import datetime, hashlib, json, os, subprocess, sys, time, traceback
from pathlib import Path
H=Path(__file__).resolve().parent; W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name; R=K/'replay'; D=H/'replay_attempt'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
STOPPED='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
def now():return datetime.datetime.now().astimezone().isoformat()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
def status(phase,**extra):
    (H/'status.json').write_text(json.dumps(dict(time=now(),phase=phase,controller_pid=os.getpid(),**extra),indent=2))
def stage(label,args):
    status(label);print(now(),label,flush=True)
    with (D/(label+'.log')).open('x') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
        save(D/(label+'_process.json'),dict(pid=p.pid,args=args,started=now()));rc=p.wait()
    save(D/(label+'_receipt.json'),dict(exit=rc,finished=now()))
    assert rc==0,(label,rc)
def ros(script,*args):
    return ['docker','exec','cbsms_ws_gpu','bash','-lc',
        'source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; '+
        'PYTHONDONTWRITEBYTECODE=1 python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/replay/'+script+' '+' '.join(args)]
D.mkdir(exist_ok=False)
assert json.loads((H/'prepared.json').read_text())['passed']
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before==STOPPED,before
pins=json.loads((H/'protected_before.json').read_text())
save(D/'start.json',dict(pid=os.getpid(),started=now(),containers=before))
failure=None;cleanup_error=None;started=False;scored=False
try:
    status('verifying_preservation')
    for p,v in pins.items():assert sha(p)==v,p
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True);started=True
    frozen=json.loads((R/'frozen_inputs.json').read_text())
    assert frozen['count']==1743 and frozen['original_pose_count']==1762
    for name in ('frozen_inputs.json','aligned_inputs.bag','recorded_updates.txt'):pins[str(R/name)]=sha(R/name)
    save(D/'protected_before.json',pins)
    for mode in ('remove','retain'):
        for p,v in pins.items():assert sha(p)==v,p
        stage('capture_'+mode,ros('run_case.py',mode))
        stage('validate_'+mode,ros('validate.py',mode))
    save(D/'passed.json',dict(passed=True,time=now(),scoring_pending=True))
except BaseException as exc:
    failure=repr(exc);save(D/'failed.json',dict(error=failure,traceback=traceback.format_exc(),time=now()))
finally:
    if started:
        try:
            top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
            save(D/'cleanup_jobs.json',dict(top=top,time=now()))
            assert all(line.split()[1] in ('tail','zsh') for line in top.splitlines()[1:] if line.strip()),top
            subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
        except BaseException as exc:
            cleanup_error=repr(exc);save(D/'cleanup_failed.json',dict(error=cleanup_error,time=now()))
    states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    save(D/'cleanup.json',dict(containers=states,restored=states==before,error=cleanup_error,time=now()))
if not failure and not cleanup_error:
    try:
        for script in ('evaluate.py','verify_report.py','summarize.py'):
            stage(script.removesuffix('.py'),[PY,'-B',str(H/script)])
        scored=True
    except BaseException as exc:
        failure=repr(exc);save(D/'report_failed.json',dict(error=failure,traceback=traceback.format_exc(),time=now()))
status('final_preservation_checks',capture_error=failure,cleanup_error=cleanup_error)
integrity_error=None
try:
    for p,v in pins.items():assert sha(p)==v,p
    stage('hesai_checkpoint',['python3','-B',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
    assert 'PASS: 4 cases, 667 saved files; originals=True' in (D/'hesai_checkpoint.log').read_text()
except BaseException as exc:
    integrity_error=repr(exc)
passed=scored and not any((failure,cleanup_error,integrity_error))
receipt=dict(passed=passed,scored=scored,error=failure,cleanup_error=cleanup_error,
    integrity_error=integrity_error,containers_restored=states==before,protected_files=len(pins),
    completed=now(),controller_pid=os.getpid(),controller_exiting=True,
    live_bidirectional_provenance_gate_passed=False,default_promotion=False)
save(H/'verification.json',receipt)
status('complete' if passed else 'failed',**{k:v for k,v in receipt.items() if k!='controller_pid'})
with (H/'REPORT.txt').open('a') as f:
    f.write('\nFINAL CONTROLLER STATUS '+now()+'\n'+json.dumps(receipt,indent=2)+'\n')
    if scored:f.write('\n'+(H/'RESULTS_REPORT.txt').read_text())
save(H/'complete.json',dict(**receipt,output_sha256={str(p):sha(p) for root in (H,K) for p in root.rglob('*')
    if p.is_file() and p.name not in ('controller.log','launch.json','launch_exit.json')}))
sys.exit(0 if passed else 1)
