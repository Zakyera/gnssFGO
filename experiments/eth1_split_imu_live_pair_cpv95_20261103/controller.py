"""One exclusive finite fresh OFF/ON pair. No tuning, restart or automatic retry."""
import datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';V=Path('/home/yeranis/results')/H.name
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'];stage='admission'
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(p.read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def call(args):return subprocess.check_output(args,text=True)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def states():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]).splitlines()
def status(state,**kwargs):
    data=dict(state=state,stage=stage,updated=now(),pid=os.getpid(),**kwargs)
    temp=H/'status.pending';temp.write_text(json.dumps(data,indent=2)+'\n');temp.replace(H/'status.json')
    print(json.dumps(data),flush=True)
def hashes():
    pins=read(P/'protected_manifest.json')
    for p,d in pins.items():assert sha(p)==d,p
    return len(pins)
def step(name,args,timeout=300):
    global stage
    stage=name;status('running')
    with (H/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,args=args,proc_start=Path(f'/proc/{job.pid}/stat').read_text().split()[21],started=now()))
        deadline=time.monotonic()+timeout
        while job.poll() is None:
            if time.monotonic()>deadline or shutil.disk_usage(P).free<5*2**30:
                # The exact owned child handles SIGTERM with its capture cleanup.
                job.terminate()
                try:job.wait(timeout=150)
                except subprocess.TimeoutExpired:pass
                raise RuntimeError(name+' outer timeout or storage guard; original child exit='+str(job.poll()))
            time.sleep(1)
    save(name+'_receipt.json',dict(exit=job.returncode,finished=now()));assert job.returncode==0,(name,job.returncode)
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']
assert not V.exists() and not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
before=states();assert before==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
save('start.json',dict(pid=os.getpid(),proc_start=Path('/proc/self/stat').read_text().split()[21],started=now(),before=before,fresh_pair=True))
started=False;failure=None;cleanup_error=None
try:
    status('preflight');hashes()
    assert shutil.disk_usage(P).free>read(H/'storage_preflight.json')['required_free_bytes']
    inhibit=call(['systemd-inhibit','--list','--no-pager','--no-legend']);matches=[x for x in inhibit.splitlines() if 'ETH1-CPV95' in x]
    assert len(matches)==1 and 'sleep' in matches[0] and 'block' in matches[0]
    save('inhibitors.json',dict(value=inhibit))
    started=True;step('start_containers',['docker','start',*names])
    top={c:call(['docker','top',c,'-eo','pid,comm,args']) for c in names}
    allowed={'tail','bash','zsh','ros2','rmw_zenohd'}
    assert all(line.split()[1] in allowed for text in top.values() for line in text.splitlines()[1:] if line.strip()),top
    save('initial_container_jobs.json',top)
    inside=str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
    code="import sys,json;sys.path.insert(0,"+repr(inside+'/transport_tools')+");import live_receiver_protocol as p;[p.validate_config(json.load(open("+repr(inside)+"+'/full_'+m+'.json'))) for m in ('off','on')];print('private parser dependencies and both configs passed')"
    step('container_adapter_import',['docker','exec','cbsms_ws_gpu','python3','-B','-c',code])
    for mode in ('off','on'):
        if mode=='on':assert read(H/'off_validated.json')['passed']
        step('capture_'+mode,['python3','-B',str(P/'run_case_on.py'),'full',mode],timeout=3300)
        gates=[]
        for label,script,args in [('source','validate_source.py',[mode]),('payload_route','validate_local.py',[mode]),('receiver','validate_case.py',['check',mode])]:
            step('validate_'+mode+'_'+label,[PY,'-B',str(P/script),*args],timeout=900)
        step('frame_'+mode,[PY,'-B',str(H/f'frame_{mode}.py'),mode],timeout=300)
        gates=[P/f'full_{mode}_source/candidate_validation.json',P/f'full_{mode}_source/local_provenance.json',V/mode/'validation.json',H/f'frame_{mode}.json']
        assert all(read(p)['passed'] for p in gates)
        save(mode+'_validated.json',dict(passed=True,finished=now(),gates=list(map(str,gates))))
except BaseException as exc:
    failure=dict(stage=stage,error=repr(exc));save('failed.json',failure)
finally:
    try:
        if started:
            deadline=time.monotonic()+90;allowed={'tail','bash','zsh','ros2','rmw_zenohd'}
            while True:
                top={c:call(['docker','top',c,'-eo','pid,comm,args']) for c in names}
                extra=[line for text in top.values() for line in text.splitlines()[1:] if line.strip() and line.split()[1] not in allowed]
                if not extra:break
                assert time.monotonic()<deadline,('Owned jobs remain; containers not stopped',extra)
                time.sleep(2)
            save('owned_jobs_exited.json',top);step('restore_containers',['docker','stop',*names])
        assert states()==before
        count=hashes();hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
        save('closeout.json',dict(passed=True,containers=states(),protected_files=count,hesai=hesai,finished=now(),free_bytes=shutil.disk_usage(P).free))
    except BaseException as exc:
        cleanup_error=repr(exc);save('cleanup_failed.json',dict(error=cleanup_error))
    if failure or cleanup_error:
        status('failed',failure=failure,cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as f:f.write('\nTERMINAL FAILURE '+now()+'\n'+json.dumps(dict(failure=failure,cleanup_error=cleanup_error))+'\nNo retry; prefixes are not full-pair results.\n')
        raise SystemExit(1)
save('capture_complete.json',dict(passed=True,duration_s=346.32,modes=['off','on'],fresh_live_all_estimators=True,finished=now()))
try:
    step('evaluate',[PY,'-B',str(H/'evaluate.py')],timeout=1800)
    step('verify_report',[PY,'-B',str(H/'verify_report.py')],timeout=300)
    assert read(H/'report_verification.json')['passed'];count=hashes()
    jobs={}
    for p in H.glob('*_process.json'):
        z=read(p);f=Path(f"/proc/{z['pid']}/stat");jobs[str(z['pid'])]=f.exists() and f.read_text().split()[21]==z['proc_start']
    assert not any(jobs.values()),jobs
    with (H/'REPORT.txt').open('a') as f:f.write('\nCOMPLETE '+now()+'\nFresh full346.32s OFF/ON with concurrent fusion; see RESULTS_REPORT.txt/results.json. Both containers restored; all protected/Hesai checks pass. Persistent correlation/native-only FALSE remains.\n')
    status('complete')
    save('complete.json',dict(passed=True,finished=now(),fresh_matched_off_on=True,source_captures=2,receiver_captures=2,
        protected_files=count,containers=states(),owned_stage_jobs_alive=jobs,controller_pid=os.getpid(),controller_exiting=True,
        output_sha256={str(p):sha(p) for p in H.rglob('*') if p.is_file() and p.name not in ('controller.log','controller.lock') and '__pycache__' not in p.parts}))
except BaseException as exc:
    save('report_failed.json',dict(stage=stage,error=repr(exc)));status('report_failed',error=repr(exc));raise
