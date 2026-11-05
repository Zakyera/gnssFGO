"""One full OFF capture, strict original gates, reporting replay and closeout."""
from pathlib import Path
import datetime, fcntl, hashlib, json, os, shutil, subprocess, time
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_incremental_cpv29_20261028/prepared')
V=Path('/home/yeranis/results/gnssfgo_eth1_incremental_cpv29_20261028')
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
CONTAINERS=('cbsms_ws_gpu','ubuntu-jammy-humble-nvidia')
stage='preflight'
def now():return datetime.datetime.now().astimezone().isoformat()
def save(name,value):
    with (H/name).open('x') as f:json.dump(value,f,indent=2);f.write('\n')
def status(state,**kw):
    data=dict(state=state,stage=stage,updated=now(),pid=os.getpid(),**kw)
    tmp=H/'status.pending';tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(H/'status.json')
    print(json.dumps(data),flush=True)
def step(name,cmd):
    global stage
    stage=name;status('running')
    with (H/(name+'.log')).open('x') as f:
        proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=proc.pid,command=cmd,started=now()))
        code=proc.wait()
    save(name+'_finished.json',dict(returncode=code,finished=now()))
    assert code==0,(name,code)
def hashes():
    pins={}
    for p in (P/'protected_manifest.json',H/'control_manifest.json'):pins.update(json.loads(p.read_text()))
    for name,want in pins.items():
        with Path(name).open('rb') as f:got=hashlib.file_digest(f,'sha256').hexdigest()
        assert got==want,name
    return len(pins)
def main():
    lock=(H/'controller.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not (H/'start.json').exists() and not (H/'status.json').exists(),'one attempt only'
    assert not V.exists() and not any(P.glob('full_*_control')) and not any(P.glob('full_*_source'))
    assert json.loads((P/'preflight/validation.json').read_text())['passed']
    assert shutil.disk_usage(H).free>8*2**30
    count=hashes()
    inhibitors=subprocess.check_output(['systemd-inhibit','--list','--no-pager','--no-legend'],text=True)
    matches=[line for line in inhibitors.splitlines() if 'ETH1-CPV29' in line]
    assert len(matches)==1 and all(t in matches[0] for t in ('block','sleep','idle','handle-lid-switch'))
    initial=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',*CONTAINERS],text=True).split()
    assert initial==['exited','exited'],initial
    save('start.json',dict(pid=os.getpid(),started=now(),original_containers=initial,
                          inhibitor=matches[0],protected_files=count))
    started=False;failure=None;cleanup_error=None
    try:
        started=True;step('start_containers',['docker','start',*CONTAINERS])
        step('capture_off',['python3',str(P/'run_case.py'),'full','off'])
        assert (P/'full_off_control/complete.json').exists() and not (P/'full_off_control/failed.txt').exists()
        for name,args in [('local',['validate_local.py','off']),('candidate',['validate_candidate.py','off']),
                          ('receiver',['validate_case.py','check','off'])]:
            step('validate_'+name,[PY,str(P/args[0]),*args[1:]])
        gates=[P/'full_off_source/local_provenance.json',P/'full_off_source/candidate_validation.json',V/'off/validation.json']
        assert all(json.loads(p.read_text())['passed'] for p in gates)
        save('off_validated.json',dict(passed=True,time=now(),gates=list(map(str,gates))))
        step('fusion_reporting_replay',[PY,str(H/'replay_fusion.py')])
        step('score',[PY,str(H/'analyze.py')])
        assert json.loads((H/'results.json').read_text())['passed']
    except BaseException as exc:
        failure=dict(stage=stage,error=repr(exc),at=now());save('failed.json',failure)
    finally:
        try:
            if started:
                deadline=time.monotonic()+60
                while True:
                    top={c:subprocess.check_output(['docker','top',c,'-eo','pid,comm,args'],text=True) for c in CONTAINERS}
                    extra=[(c,line) for c,value in top.items() for line in value.splitlines()[1:]
                           if line.strip() and line.split()[1] not in {'tail','zsh','ros2','rmw_zenohd'}]
                    if not extra:break
                    assert time.monotonic()<deadline,('jobs remain; containers left running',extra)
                    time.sleep(2)
                save('container_idle_before_stop.json',top)
                with (H/'container_cleanup.log').open('x') as f:
                    subprocess.run(['docker','stop',*CONTAINERS],stdout=f,stderr=subprocess.STDOUT,check=True)
            final=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',*CONTAINERS],text=True).split()
            assert final==['exited','exited'],final
            count=hashes()
            checkpoint=H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
            step('verify_hesai_checkpoint',['python3',str(checkpoint),'--originals'])
            save('closeout.json',dict(passed=True,time=now(),containers=final,protected_files=count,
                                     old_hesai_checkpoint_passed=True,free_bytes=shutil.disk_usage(H).free))
        except BaseException as exc:
            cleanup_error=repr(exc);save('cleanup_failed.json',dict(error=cleanup_error,time=now()))
        ok=failure is None and cleanup_error is None
        status('complete' if ok else 'failed',failure=failure,cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as f:
            f.write('\nFinished '+now()+': '+('COMPLETED' if ok else 'STOPPED/FAILED')+'\n')
            f.write(json.dumps(dict(failure=failure,cleanup_error=cleanup_error))+'\n')
            if ok:f.write('Full OFF capture, original gates, live-output validation and reporting parity passed. See RESULTS_REPORT.txt/results.json.\n')
            f.write('No automatic retry, no ON capture, no default promotion. Sleep inhibitor releases on exit.\n')
    if failure or cleanup_error:raise SystemExit(1)
if __name__=='__main__':main()
