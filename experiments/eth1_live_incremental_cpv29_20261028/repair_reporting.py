"""Finish never-started reporting stages in their original owning environment."""
import datetime,fcntl,hashlib,importlib.util,json,os,subprocess,time,traceback
from pathlib import Path
H=Path(__file__).resolve().parent
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
C='ubuntu-jammy-humble-nvidia'
def now():return datetime.datetime.now().astimezone().isoformat()
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2);f.write('\n')
def status(stage,**kw):
    d=dict(stage=stage,time=now(),pid=os.getpid(),**kw)
    tmp=H/'reporting_repair_status.pending';tmp.write_text(json.dumps(d,indent=2)+'\n');tmp.replace(H/'reporting_repair_status.json')
    print(json.dumps(d),flush=True)
def run(stage,cmd):
    status(stage)
    with (H/('repair_'+stage+'.log')).open('xb') as log:
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,
                              env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1'))
        save('repair_'+stage+'_launch.json',dict(pid=proc.pid,command=cmd,time=now()))
        code=proc.wait()
    save('repair_'+stage+'_finish.json',dict(returncode=code,time=now()))
    assert code==0,(stage,code)
def main():
    lock=(H/'reporting_repair.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not (H/'reporting_repair_start.json').exists()
    assert json.loads((H/'off_validated.json').read_text())['passed']
    for n in ('fusion_replay_pins.json','fusion_replay_launch.json','fusion_observer_replay','fusion_publications.csv','results.json'):assert not (H/n).exists(),n
    spec=importlib.util.spec_from_file_location('cpv29_run',H/'run.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    initial=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',C,'cbsms_ws_gpu'],text=True).split();assert initial==['exited','exited']
    save('reporting_repair_start.json',dict(time=now(),pid=os.getpid(),initial_containers=initial,protected_files=m.hashes()))
    failure=None;cleanup_error=None
    try:
        run('start_container',['docker','start',C])
        run('fusion_replay',['docker','exec','-u','1000:1000',C,'python3','-B',str(H/'replay_fusion.py')])
        run('score',[PY,'-B',str(H/'analyze.py')])
        assert json.loads((H/'results.json').read_text())['passed']
    except BaseException:
        failure=traceback.format_exc();save('reporting_repair_failure.json',dict(time=now(),traceback=failure))
    finally:
        try:
            state=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',C],text=True).strip()
            if state=='running':
                top=subprocess.check_output(['docker','top',C,'-eo','pid,comm,args'],text=True)
                extra=[l for l in top.splitlines()[1:] if l.strip() and l.split()[1] not in {'tail','zsh','ros2','rmw_zenohd'}]
                assert not extra,('non-idle jobs remain; do not stop',extra)
                save('repair_container_idle.json',dict(top=top));run('stop_container',['docker','stop',C])
            states=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}',C,'cbsms_ws_gpu'],text=True).split();assert states==initial
            count=m.hashes()
            run('checkpoint',['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
            save('reporting_repair_closeout.json',dict(passed=failure is None,time=now(),containers=states,protected_files=count,source_reruns=0,owned_jobs_exited=True,old_checkpoint_passed=True))
        except BaseException:
            cleanup_error=traceback.format_exc();save('reporting_repair_cleanup_failure.json',dict(traceback=cleanup_error))
        ok=failure is None and cleanup_error is None;status('complete' if ok else 'failed',failure=failure,cleanup_error=cleanup_error)
        with (H/'REPORT.txt').open('a') as f:
            f.write('\nReporting repair '+now()+': '+('COMPLETE' if ok else 'FAILED')+'\n')
            f.write('No source rerun or setting change. Original reporting failure retained; see reporting_repair_status.json and closeout.\n')
    if not ok:raise SystemExit(1)
if __name__=='__main__':main()
