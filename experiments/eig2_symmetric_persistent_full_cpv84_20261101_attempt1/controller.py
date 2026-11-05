"""One full355.02s live ON, gated and saved-only reported; never retries a failed stage."""
import datetime,fcntl,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
V=Path('/home/yeranis/results')/H.name
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
names=['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'];stage='admission'
def now():return datetime.datetime.now().astimezone().isoformat()
def read(p):return json.loads(p.read_text())
def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
def call(args):return subprocess.check_output(args,text=True)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def inspect():return call(['docker','inspect','-f','{{.Name}} {{.State.Status}}',*names]).splitlines()
def status(state,**kwargs):
    data=dict(state=state,stage=stage,updated=now(),pid=os.getpid(),**kwargs)
    p=H/'status.pending';p.write_text(json.dumps(data,indent=2)+'\n');p.replace(H/'status.json')
    print(json.dumps(data),flush=True)
def hashes():
    pins=read(P/'protected_manifest.json')
    for p,want in pins.items():assert sha(p)==want,p
    return len(pins)
def step(label,args):
    global stage
    stage=label;status('running')
    with (H/(label+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        proc=Path(f'/proc/{job.pid}')
        save(label+'_process.json',dict(pid=job.pid,args=args,start_ns=time.time_ns(),
            proc_start=proc.joinpath('stat').read_text().split()[21]))
        code=job.wait()
    save(label+'_receipt.json',dict(exit=code,end_ns=time.time_ns()))
    assert code==0,(label,code)
def gate_paths():
    return [P/'full_on_source'/f'{name}.json' for name in
            ('hesai_validation','local_provenance','candidate_validation')]+[V/'on/validation.json']
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (H/'start.json').exists() and read(H/'prepared.json')['passed']
assert not V.exists() and not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
before=inspect();assert all(s.endswith(' exited') for s in before)
save('start.json',dict(pid=os.getpid(),proc_start=Path('/proc/self/stat').read_text().split()[21],
    started=now(),before=before,single_live_on=True,
    kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True))
started=False;failure=None;cleanup_error=None
try:
    assert shutil.disk_usage(P).free>read(H/'storage_preflight.json')['required_free_bytes']
    inhibitors=call(['systemd-inhibit','--list','--no-pager','--no-legend'])
    matches=[s for s in inhibitors.splitlines() if 'EIG2-CPV84-A1' in s]
    assert len(matches)==1 and all(t in matches[0] for t in ('block','sleep'))
    save('inhibitors.json',dict(system=inhibitors))
    hashes()
    step('preflight_on',['python3','-B',str(P/'preflight_on.py')])
    started=True;step('start_containers',['docker','start',*names])
    step('capture_on',['python3','-B',str(P/'run_case_on.py'),'full','on'])
    for name,args in [('sensor',['validate_hesai.py','on']),('payload_route',['validate_local.py','on']),
                      ('candidate',['validate_candidate.py','on']),('receiver',['validate_case.py','check','on'])]:
        step('validate_on_'+name,[PY,'-B',str(P/args[0]),*args[1:]])
    assert all(read(p)['passed'] for p in gate_paths())
    save('on_validated.json',dict(passed=True,finished=now(),gates=list(map(str,gate_paths())),
        kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True))
except BaseException as exc:
    failure=dict(stage=stage,error=repr(exc));save('failed.json',failure)
finally:
    try:
        if started:
            deadline=time.monotonic()+90;allowed={'tail','zsh','ros2','rmw_zenohd'}
            while True:
                top={c:call(['docker','top',c,'-eo','pid,comm,args']) for c in names}
                extra=[line for text in top.values() for line in text.splitlines()[1:]
                       if line.strip() and line.split()[1] not in allowed]
                if not extra:break
                assert time.monotonic()<deadline,('jobs remain; do not stop containers',extra)
                time.sleep(2)
            save('owned_jobs_exited.json',top)
            step('restore_containers',['docker','stop',*names])
        final=inspect();assert final==before
        count=hashes()
        hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
        save('closeout.json',dict(passed=True,containers=final,protected_files=count,hesai=hesai,
            free_bytes=shutil.disk_usage(P).free))
    except BaseException as exc:
        cleanup_error=repr(exc);save('cleanup_failed.json',dict(error=cleanup_error))
    if failure or cleanup_error:
        status('failed',failure=failure,cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as f:
            f.write('\nTERMINAL FAILURE '+now()+'\n'+json.dumps(dict(failure=failure,cleanup_error=cleanup_error))+
                    '\nNo retry; prefixes are NOT full results.\n')
        raise SystemExit(1)
    save('capture_complete.json',dict(passed=True,duration_s=355.02,modes=['on'],finished=now(),
        fresh_live_all_estimators=True,accuracy_scored=False))
try:
    step('evaluate',[PY,'-B',str(H/'evaluate.py')])
    step('verify_report',[PY,'-B',str(H/'verify_report.py')])
    assert read(H/'report_verification.json')['passed']
    count=hashes()
    jobs={}
    for p in H.glob('*_process.json'):
        r=read(p);proc=Path(f"/proc/{r['pid']}/stat")
        jobs[str(r['pid'])]=proc.exists() and proc.read_text().split()[21]==r['proc_start']
    assert not any(jobs.values()),jobs
    with (H/'REPORT.txt').open('a') as f:
        f.write('\nCOMPLETE '+now()+'\nfull355.02s live ON, all capture/route/lifecycle/receiver/EVO gates pass.\n'
                'See RESULTS_REPORT.txt, results.json and report_verification.json.\n'
                'Kimera native-only information provenance remains FALSE; user accepted mixed-prior feedback.\n'
                'Both containers restored; protected/Hesai667 pass; no retries or default promotion.\n')
    status('complete')
    output={}
    for p in H.rglob('*'):
        if p.is_file() and p.name not in ('controller.log','controller.lock') and '__pycache__' not in p.parts:
            output[str(p)]=sha(p)
    save('complete.json',dict(passed=True,finished=now(),completed_requested_live_window=True,
        protected_files=count,output_sha256=output,owned_stage_jobs_alive=jobs,
        controller_pid=os.getpid(),controller_exiting=True,containers=inspect(),
        kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True,
        fresh_matched_off=False))
except BaseException as exc:
    save('report_failed.json',dict(error=repr(exc),stage=stage))
    status('report_failed',error=repr(exc))
    with (H/'REPORT.txt').open('a') as f:f.write('\nREPORT FAILED: '+repr(exc)+'; no capture retry.\n')
    raise
