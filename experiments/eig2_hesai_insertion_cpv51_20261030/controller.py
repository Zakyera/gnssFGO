"""Finite exclusive matched capture/gates; no retries or competing stages."""
import datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
V=Path('/home/yeranis/results')/H.name
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'];stage='admission'
def now():return datetime.datetime.now().astimezone().isoformat()
def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
def inspect():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]).splitlines()
def status(state,**kwargs):
    data=dict(state=state,stage=stage,updated=now(),pid=os.getpid(),**kwargs)
    p=H/'status.pending';p.write_text(json.dumps(data,indent=2)+'\n');p.replace(H/'status.json')
    print(json.dumps(data),flush=True)
def hashes():
    pins=json.loads((P/'protected_manifest.json').read_text())
    for p,want in pins.items():
        with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==want,p
    return len(pins)
def step(label,args):
    global stage
    stage=label;status('running')
    with (H/(label+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(label+'_process.json',dict(pid=job.pid,args=args,start_ns=time.time_ns()))
        code=job.wait()
    save(label+'_receipt.json',dict(exit=code,end_ns=time.time_ns()))
    assert code==0,(label,code)
def gate_paths(mode):
    return [P/f'full_{mode}_source/{name}.json' for name in ('hesai_validation','local_provenance','candidate_validation')]+[V/mode/'validation.json']
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and json.loads((H/'prepared.json').read_text())['passed']
assert not V.exists() and not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
forecast=json.loads((H/'storage_preflight.json').read_text())
assert shutil.disk_usage(P).free>forecast['required_free_bytes']
before=inspect();assert all(s.endswith(' exited') for s in before)
inhibitors=call(['systemd-inhibit','--list','--no-pager','--no-legend'])
matches=[s for s in inhibitors.splitlines() if 'EIG2-CPV51' in s]
assert len(matches)==1 and all(t in matches[0] for t in ('block','sleep','idle','handle-lid-switch'))
session=call(['gnome-session-inhibit','--list'])
matches=[s for s in session.splitlines() if s.startswith('EIG2-CPV51:')]
assert len(matches)==1 and all(t in matches[0] for t in ('suspend','idle'))
count=hashes()
save('start.json',dict(pid=os.getpid(),started=now(),before=before,inhibitors=inhibitors,session=session,protected_files=count))
started=False;failure=None
try:
    started=True;step('start_containers',['docker','start',*names])
    for mode in ('off','on'):
        if mode=='on':
            assert all(json.loads(p.read_text())['passed'] for p in gate_paths('off'))
            assert json.loads((H/'off_snapshots.json').read_text())['passed']
            step('preflight_on',['python3',str(P/'preflight_on.py')])
        runner='run_case.py' if mode=='off' else 'run_case_on.py'
        step('capture_'+mode,['python3',str(P/runner),'full',mode])
        for name,args in [('sensor',['validate_hesai.py',mode]),('local',['validate_local.py',mode]),
                          ('candidate',['validate_candidate.py',mode]),('receiver',['validate_case.py','check',mode])]:
            step('validate_'+mode+'_'+name,[PY,str(P/args[0]),*args[1:]])
        assert all(json.loads(p.read_text())['passed'] for p in gate_paths(mode))
        step('validate_'+mode+'_snapshots',[PY,str(H/'validate_snapshots.py'),mode])
        save(mode+'_validated.json',dict(passed=True,finished=now(),gates=list(map(str,gate_paths(mode)))))
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
        count=hashes()
        hesai=call(['python3','/home/yeranis/repos/gnssFGO/experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py','--originals'])
        save('closeout.json',dict(passed=True,containers=final,protected_files=count,hesai=hesai,free_bytes=shutil.disk_usage(P).free))
    except BaseException as exc:cleanup=repr(exc);save('cleanup_failed.json',dict(error=cleanup))
    ok=not failure and not cleanup
    if ok:save('capture_complete.json',dict(passed=True,duration_s=10.0,modes=['off','on'],accuracy_scored=False,finished=now()))
    status('captured_validated_reporting_pending' if ok else 'failed',failure=failure,cleanup_error=cleanup)
    with (H/'REPORT.txt').open('a') as f:
        f.write('\nController finished '+now()+': '+('both captures/gates passed; native checkpoint analysis pending' if ok else 'STOPPED/FAILED')+'\n')
        f.write(json.dumps(dict(failure=failure,cleanup_error=cleanup))+'\nNo automatic retry.\n')
    if not ok:raise SystemExit(1)

# Native checkpoint analysis is saved-only and starts after this controller exits.
