"""One finite finalizer: wait for existing replay controller, score and close."""
import hashlib,json,os,subprocess,time,traceback
from pathlib import Path
H=Path(__file__).resolve().parent
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
CONTAINER='ubuntu-jammy-humble-nvidia'
def save(p,d):
    with p.open('x') as f:json.dump(d,f,indent=2)
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def run(stage,cmd):
    print(stage,flush=True);t=time.time()
    with (H/('finalize_'+stage+'.log')).open('xb') as f:
        p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1'))
    save(H/('finalize_'+stage+'.json'),dict(command=cmd,returncode=p.returncode,seconds=time.time()-t))
    assert p.returncode==0,(stage,'see retained log')
def owned_processes():
    output=subprocess.check_output(['docker','top',CONTAINER,'-eo','pid,comm,args'],text=True)
    return [line for line in output.splitlines() if ('live_publications' in line or str(H/'replay.py') in line)]
def main():
    save(H/'finalizer_start.json',dict(pid=os.getpid(),time=time.time()))
    print('waiting_for_existing_replay_controller',flush=True)
    while not (H/'replay_complete.json').exists():
        assert not (H/'replay_failed.txt').exists(),'replay controller failed; do not rerun'
        time.sleep(20)
    # The controller writes completion before removing its verified RAM copy.
    for _ in range(12):
        if not owned_processes():break
        time.sleep(5)
    assert not owned_processes(),'owned replay still active'
    run('restore',['python3',str(H/'stage_products.py'),'restore'])
    run('score',[PY,str(H/'analyze.py'),'score','all'])
    run('verify',[PY,str(H/'verify.py')])
    run('report',[PY,str(H/'report.py')])
    run('checkpoint',['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
    assert not owned_processes()
    run('stop',['docker','stop',CONTAINER])
    states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',CONTAINER,'cbsms_ws_gpu'],text=True)
    assert states.count('exited')==2
    assert not any(p.is_symlink() for p in H.iterdir()),'temporary symlink remains'
    output_hashes={str(p):sha(p) for p in H.rglob('*') if p.is_file() and not p.name.endswith('.log')}
    save(H/'closeout.json',dict(pass_=True,time=time.time(),method=2,containers=states,
         owned_jobs_exited=True,temporary_products_restored=True,
         hesai_checkpoint=(H/'finalize_checkpoint.log').read_text().strip(),output_sha256=output_hashes,
         receiver_replays=12,reused_receiver_cases=2,source_reruns=0,builds=0,model_changes=False))
    print('complete',flush=True)
if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'finalize_failed.txt').open('x') as f:f.write(traceback.format_exc())
        # Never stop an active replay or another user's job on a reporting error.
        raise
