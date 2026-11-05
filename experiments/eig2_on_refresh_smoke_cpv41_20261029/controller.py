"""ONE bounded ON capture/gates with exact ownership and container restoration."""
import fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python';names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and json.loads((H/'prepared.json').read_text())['passed']
assert not (P/'full_on_control').exists() and shutil.disk_usage(P).free>8*1024**3
def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
def inspect():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]).splitlines()
before=inspect();assert all(s.endswith(' exited') for s in before)
inhibitors=call(['systemd-inhibit','--list','--no-pager','--no-legend']);assert 'EIG2-CPV41' in inhibitors
session=call(['gnome-session-inhibit','--list']);assert 'EIG2-CPV41:' in session
save('start.json',dict(pid=os.getpid(),wall_ns=time.time_ns(),before=before,inhibitors=inhibitors,session=session))
started=False;failure=None;stage='start_containers'
def step(label,args):
    global stage
    stage=label
    with (H/(label+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(label+'_process.json',dict(pid=job.pid,args=args,start_ns=time.time_ns()))
        code=job.wait()
    save(label+'_receipt.json',dict(exit=code,end_ns=time.time_ns()))
    print(label,code,flush=True);assert code==0,(label,code)
try:
    started=True;step('start_containers',['docker','start',*names])
    step('capture_on',['python3',str(P/'run_case_on.py'),'full','on'])
    for name,args in [('sensor',['validate_velodyne.py','on']),('local',['validate_local.py','on']),
                      ('candidate',['validate_candidate.py','on']),('receiver',['validate_case.py','check','on'])]:
        step('validate_'+name,[PY,str(P/args[0]),*args[1:]])
except BaseException as exc:
    failure=dict(stage=stage,error=repr(exc));save('failed.json',failure)
finally:
    cleanup=None
    try:
        if started:
            deadline=time.monotonic()+60;allowed={'tail','zsh','ros2','rmw_zenohd'}
            while True:
                top={c:call(['docker','top',c,'-eo','pid,comm,args']) for c in names}
                extra=[line for text in top.values() for line in text.splitlines()[1:] if line.strip() and line.split()[1] not in allowed]
                if not extra:break
                assert time.monotonic()<deadline,('jobs remain; do not stop containers',extra)
                time.sleep(2)
            save('owned_jobs_exited.json',top);step('restore_containers',['docker','stop',*names])
        final=inspect();assert final==before
        pins=json.loads((P/'protected_manifest.json').read_text())
        for p,want in pins.items():
            with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==want,p
        hesai=call(['python3','/home/yeranis/repos/gnssFGO/experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py','--originals'])
        save('closeout.json',dict(passed=True,containers=final,protected_files=len(pins),hesai=hesai,free_bytes=shutil.disk_usage(P).free))
    except BaseException as exc:cleanup=repr(exc);save('cleanup_failed.json',dict(error=cleanup))
    if not failure and not cleanup:save('complete.json',dict(passed=True,duration_s=30,mode='on',accuracy_scored=False,wall_ns=time.time_ns()))
    print(json.dumps(dict(failure=failure,cleanup_error=cleanup)),flush=True)
    if failure or cleanup:raise SystemExit(1)
