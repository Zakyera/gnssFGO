"""Exactly one CP98 short source capture; restore originally-stopped containers."""
import csv,datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';PC=str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(p.read_text())
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
def identity(pid):return Path(f'/proc/{pid}/stat').read_text().split()[21]
def status(stage):
    data=dict(stage=stage,updated=now(),pid=os.getpid());(H/'status.json').write_text(json.dumps(data,indent=2));print(json.dumps(data),flush=True)
def hashes():
    pins=read(P/'protected_manifest.json')
    for p,d in pins.items():assert sha(p)==d,p
    return len(pins)
def step(name,args,timeout=120):
    status(name)
    with (H/(name+'.log')).open('x') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=p.pid,command=args,proc_start=identity(p.pid),started=now()))
        deadline=time.monotonic()+timeout
        while p.poll() is None:
            if time.monotonic()>deadline or shutil.disk_usage(P).free<10*1024**3:
                p.terminate()
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:p.kill();p.wait()
                raise RuntimeError(name+' timeout or storage guard')
            time.sleep(1)
    save(name+'_receipt.json',dict(exit=p.returncode,finished=now()));assert p.returncode==0,(name,p.returncode)
def states():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and read(H/'prepared.json')['passed'] and not (P/'on_source').exists()
before=states();assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
save('start.json',dict(pid=os.getpid(),proc_start=identity(os.getpid()),started=now(),containers=before))
failure=None;started=False
try:
    status('preflight');hashes();assert shutil.disk_usage(P).free>20*1024**3
    inhibit=call(['systemd-inhibit','--list','--no-pager','--no-legend']);assert 'ETH1-CPV98' in inhibit
    save('inhibitors.json',dict(value=inhibit))
    started=True;step('start_container',['docker','start','cbsms_ws_gpu'])
    deadline=time.monotonic()+30
    while True:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        if all(r.split()[1] in {'tail','bash','zsh'} for r in top.splitlines()[1:] if r.strip()):break
        assert time.monotonic()<deadline,top
        time.sleep(1)
    save('initial_container_jobs.json',dict(top=top))
    command=f'export CBSMS_RUN_NAME=on_source CBSMS_CBS_ENABLE=true; source {PC}/source.env; exec timeout --signal=INT --kill-after=20s 600s bash {PC}/source_runner_on.sh'
    step('capture',['docker','exec','cbsms_ws_gpu','bash','-lc',command],timeout=630)
    S=P/'on_source'
    assert (S/'complete.marker').exists() and read(S/'source_coverage.json')['passed'] and read(S/'effective_mode_check.json')['passed']
    with (S/'objective.csv').open() as f:updates=[int(float(r[0])) for r in csv.reader(f)]
    assert updates==list(range(len(updates))) and len(updates)>30
    expected={(k,s) for k in updates if k>=2 for s in ('before_ordinary','after_ordinary','after_refresh','after_marginalization')}
    paths=list((S/'checkpoints').glob('update_*/*/checkpoint.bin'))
    assert {(int(p.parent.parent.name.split('_')[1]),p.parent.name) for p in paths}==expected
    times=[];nmax=0
    for p in paths:
        assert p.stat().st_size>1000
        done=(p.parent/'complete.txt').read_text();assert 'values readback PASS' in done
        times.append(float(done.split('seconds,')[1]))
        rt=dict(csv.reader((p.parent/'roundtrip.txt').open()));assert int(rt['factors'])>0
        assert float(rt['max_relative_linear_difference'])<1e-12
        nmax=max(nmax,int(rt['factors']))
    for k in updates:
        if k>=2:
            ev=list(csv.reader((S/'checkpoints'/f'update_{k}'/'event.txt').open()))
            assert any(r[0]=='added' for r in ev) and not any(r[0]=='removed' for r in ev)
    save('snapshot_validation.json',dict(passed=True,updates=len(updates),snapshots=len(paths),bytes=sum(p.stat().st_size for p in paths),max_live_factors=nmax,observer_seconds_sum=sum(times),observer_seconds_max=max(times),clones_only_roundtrip=True))
except BaseException as exc:
    failure=repr(exc);save('failed.json',dict(error=failure,at=now()));status('failed')
finally:
    try:
        if started:
            top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
            save('jobs_before_restore.json',dict(top=top,success=failure is None))
            # Only this controller started this previously-stopped container.
            step('restore_container',['docker','stop','--time','20','cbsms_ws_gpu'],timeout=60)
        assert states()==before
        count=hashes()
        hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
        save('closeout.json',dict(passed=True,containers=states(),protected_files=count,hesai=hesai,finished=now()))
    except BaseException as exc:
        save('cleanup_failed.json',dict(error=repr(exc),at=now()));status('cleanup_failed');raise
if failure:raise SystemExit(1)
for p in H.glob('*_process.json'):
    r=read(p);s=Path(f"/proc/{r['pid']}/stat");assert not s.exists() or identity(r['pid'])!=r['proc_start']
status('capture_complete')
save('capture_complete.json',dict(passed=True,duration_s=12.0,source_runs=1,settings_unchanged=True,offline_analysis_pending=True,finished=now(),output_sha256={str(p):sha(p) for p in (P/'on_source').rglob('*') if p.is_file() and not p.is_symlink()}))
