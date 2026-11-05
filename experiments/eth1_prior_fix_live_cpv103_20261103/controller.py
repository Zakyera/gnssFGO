"""One finite corrected-backend ON qualification; no retry or evaluation stage."""
import datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
from paths import H,E,W,P,V,PY,FIX_H
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'];stage='admission'
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(p.read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def call(a):return subprocess.check_output(a,text=True)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def identity(pid):
    p=Path(f'/proc/{pid}/stat');return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
def states():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]).splitlines()
def status(state,**kw):
    data=dict(state=state,stage=stage,updated=now(),pid=os.getpid(),**kw)
    tmp=H/'status.pending';tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(H/'status.json')
    print(json.dumps(data),flush=True)
def hashes():
    pins=read(P/'protected_manifest.json')
    for p,v in pins.items():assert sha(p)==v,p
    return len(pins)
def step(name,args,timeout=300):
    global stage
    stage=name;status('running')
    with (H/(name+'.log')).open('x') as log:
        job=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,proc_start=identity(job.pid),args=args,started=now()))
        deadline=time.monotonic()+timeout
        while job.poll() is None:
            if time.monotonic()>deadline or shutil.disk_usage(P).free<5*2**30:
                job.terminate()
                try:job.wait(timeout=150)
                except subprocess.TimeoutExpired:pass
                raise RuntimeError(name+' timeout/storage guard; child='+str(job.poll()))
            time.sleep(1)
    save(name+'_receipt.json',dict(exit=job.returncode,finished=now()));assert job.returncode==0,(name,job.returncode)
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']
assert read(FIX_H/'complete.json')['passed'] and read(H/'validator_test.json')['passed']
assert not V.exists() and not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
before=states();assert before==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
save('start.json',dict(pid=os.getpid(),proc_start=identity(os.getpid()),started=now(),before=before,ON_only=True))
started=False;failure=None;cleanup_error=None
try:
    status('preflight');hashes()
    assert shutil.disk_usage(P).free>read(H/'storage_preflight.json')['required_free_bytes']
    inhibit=call(['systemd-inhibit','--list','--no-pager','--no-legend'])
    matches=[s for s in inhibit.splitlines() if 'ETH1-CPV103' in s]
    assert len(matches)==1 and 'sleep' in matches[0] and 'block' in matches[0]
    save('inhibitors.json',dict(value=inhibit))
    started=True;step('start_containers',['docker','start',*names])
    allowed={'tail','bash','zsh','ros2','rmw_zenohd'};deadline=time.monotonic()+30;observed=[];quiet=0
    while quiet<2:
        top={c:call(['docker','top',c,'-eo','pid,comm,args']) for c in names}
        extra=[line for text in top.values() for line in text.splitlines()[1:] if line.strip() and line.split()[1] not in allowed]
        observed.append(dict(at=now(),top=top,extra=extra));quiet=0 if extra else quiet+1
        assert time.monotonic()<deadline,('container idle guard',top)
        time.sleep(1)
    save('startup_quiescence.json',dict(passed=True,snapshots=observed));save('initial_container_jobs.json',top)
    inside=str(P).replace(str(W),'/workspace/cbs_gtsam4.3')
    step('container_adapter_import',['docker','exec','cbsms_ws_gpu','python3','-B',inside+'/transport_tools/qualify_runtime.py'])
    step('capture_on',['python3','-B',str(P/'run_case_on.py'),'full','on'],3300)
    for label,script,args in [('source','validate_source.py',['on']),('payload_route','validate_local.py',['on']),('receiver','validate_case.py',['check','on'])]:
        step('validate_on_'+label,[PY,'-B',str(H/script),*args],900)
    step('frame_on',[PY,'-B',str(H/'frame_on.py'),'on'],300)
    gates=[H/'on_candidate_validation.json',H/'on_local_provenance.json',V/'on/validation.json',H/'frame_on.json']
    assert all(read(p)['passed'] for p in gates)
    save('on_validated.json',dict(passed=True,finished=now(),gates=list(map(str,gates))))
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
                assert time.monotonic()<deadline,('owned jobs remain; no container stop',extra)
                time.sleep(2)
            save('owned_jobs_exited.json',top);step('restore_containers',['docker','stop',*names])
        assert states()==before
        count=hashes();hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
        assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
        save('closeout.json',dict(passed=True,containers=states(),protected_files=count,hesai=hesai,finished=now(),free_bytes=shutil.disk_usage(P).free))
    except BaseException as exc:
        cleanup_error=repr(exc);save('cleanup_failed.json',dict(error=cleanup_error))
    if failure or cleanup_error:
        status('failed',failure=failure,cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as f:f.write('\nTERMINAL FAILURE '+now()+'\n'+json.dumps(dict(failure=failure,cleanup_error=cleanup_error))+'\nOriginal prefix preserved; no retry.\n')
        raise SystemExit(1)
stage='qualification_summary'
try:
    coverage=read(P/'full_on_source/source_coverage.json')
    receiver=read(V/'on/validation.json')
    archives=list((P/'full_on_source/prior_failures').iterdir());assert not archives
    result=dict(passed=True,full_input_window_s=346.32,source_coverage=coverage,receiver_validation=receiver,
        frame_validation=str(H/'frame_on.json'),prior_failure_archives=0,
        correction=str(FIX_H/'complete.json'),same_scientific_settings=True,changed_backend=True,
        no_failure_in_this_realization=True,historic_cp99_cause_proven=False,
        no_new_OFF=True,no_evo_scoring=True,no_health_calibration_or_randomness_certificate=True)
    save('qualification.json',result)
    jobs=[]
    for p in H.glob('*_process.json'):
        r=read(p);assert identity(r['pid'])!=str(r['proc_start']),p
        jobs.append(dict(receipt=str(p),pid=r['pid'],proc_start=r['proc_start'],exited=True))
    with (H/'REPORT.txt').open('a') as f:
        f.write('\nCOMPLETE '+now()+'\nFull346.32s ON input/capture and all4gates passed. No prior-preservation failure archives. See qualification.json; no new OFF or EVO score. Historical CP99 exact cause remains unproven. Both containers restored and protected/Hesai checks passed.\n')
    status('complete')
    save('complete.json',dict(passed=True,finished=now(),source_captures=1,receiver_captures=1,mode='on',
        protected_files=count,containers=states(),owned_stage_jobs=jobs,controller_pid=os.getpid(),controller_exiting=True,
        output_sha256={str(p):sha(p) for p in H.rglob('*') if p.is_file() and p.name not in ('controller.log','controller.lock') and '__pycache__' not in p.parts}))
except BaseException as exc:
    save('report_failed.json',dict(stage=stage,error=repr(exc)));status('report_failed',error=repr(exc));raise
