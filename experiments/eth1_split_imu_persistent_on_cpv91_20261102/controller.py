"""Exactly one bounded source capture, restore containers, then saved-only scoring."""
import datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';PC=str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(p.read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
def status(stage):
    data=dict(stage=stage,updated=now(),pid=os.getpid());(H/'status.json').write_text(json.dumps(data,indent=2))
    print(json.dumps(data),flush=True)
def hashes():
    pins=read(P/'protected_manifest.json')
    for p,d in pins.items():assert sha(p)==d,p
    return len(pins)
def step(name,args,timeout=120):
    status(name)
    with (H/(name+'.log')).open('x') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=p.pid,command=args,proc_start=Path(f'/proc/{p.pid}/stat').read_text().split()[21],started=now()))
        deadline=time.monotonic()+timeout
        while p.poll() is None:
            if time.monotonic()>deadline or shutil.disk_usage(P).free<5*1024**3:
                p.terminate()
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:p.kill();p.wait()
                raise RuntimeError(name+' timeout or storage guard')
            time.sleep(1)
    save(name+'_receipt.json',dict(exit=p.returncode,finished=now()))
    assert p.returncode==0,(name,p.returncode)
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']
def states():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names])
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']
assert not (P/'on_source').exists()
before=states();assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
save('start.json',dict(pid=os.getpid(),proc_start=Path('/proc/self/stat').read_text().split()[21],started=now(),containers=before))
failure=None;started=False
try:
    status('preflight');hashes();assert shutil.disk_usage(P).free>10*1024**3
    inhibit=call(['systemd-inhibit','--list','--no-pager','--no-legend']);assert 'ETH1-CPV91' in inhibit
    save('inhibitors.json',dict(value=inhibit))
    started=True;step('start_container',['docker','start','cbsms_ws_gpu'])
    step('initial_container_jobs',['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    top=(H/'initial_container_jobs.log').read_text()
    assert all(line.split()[1] in {'tail','bash','zsh'} for line in top.splitlines()[1:] if line.strip()),top
    command=f'export CBSMS_RUN_NAME=on_source CBSMS_CBS_ENABLE=true; source {PC}/source.env; exec timeout --signal=INT --kill-after=20s 2400s bash {PC}/source_runner_on.sh'
    step('capture',['docker','exec','cbsms_ws_gpu','bash','-lc',command],timeout=2430)
    assert (P/'on_source/complete.marker').exists()
    assert read(P/'on_source/source_coverage.json')['passed'] and read(P/'on_source/effective_mode_check.json')['passed']
except BaseException as exc:
    failure=repr(exc);save('failed.json',dict(error=failure,at=now()));status('failed')
finally:
    try:
        if started:
            top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
            save('jobs_before_restore.json',dict(top=top,success=failure is None))
            if failure is None:assert all(line.split()[1] in {'tail','bash','zsh'} for line in top.splitlines()[1:] if line.strip()),top
            # This previously-stopped container was started solely for this owned capture.
            step('restore_container',['docker','stop','--time','20','cbsms_ws_gpu'],timeout=60)
        assert states()==before
        count=hashes()
        hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
        save('closeout.json',dict(passed=True,containers=states(),protected_files=count,hesai=hesai,finished=now()))
    except BaseException as exc:
        save('cleanup_failed.json',dict(error=repr(exc),at=now()));status('cleanup_failed');raise
if failure:raise SystemExit(1)
save('capture_complete.json',dict(passed=True,duration_s=346.32,CBS_ON=True,fusion=False,finished=now()))
try:
    step('frame_audit',[PY,'-B',str(H/'frame_audit.py'),'on'],timeout=180)
    assert read(H/'frame_validation.json')['passed']
    step('evaluate',[PY,'-B',str(H/'evaluate.py')],timeout=300)
    assert read(H/'report_verification.json')['passed']
    count=hashes()
    for p in H.glob('*_process.json'):
        rec=read(p);stat=Path(f"/proc/{rec['pid']}/stat")
        assert not stat.exists() or stat.read_text().split()[21]!=rec['proc_start'],rec
    status('complete')
    save('complete.json',dict(passed=True,finished=now(),protected_files=count,
         output_sha256={str(p):sha(p) for p in H.rglob('*') if p.is_file() and p.name not in ('controller.log','controller.lock','REPORT.txt')},
         containers=states(),single_capture=True,no_fusion=True,both_persistent=True,live_bidirectional=True))
except BaseException as exc:
    save('report_failed.json',dict(error=repr(exc),at=now()));status('report_failed');raise
