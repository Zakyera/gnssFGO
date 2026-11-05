"""Finish after documented storage continuation; never launches a replay."""
import os,subprocess,time,traceback
import finalize as m
H=m.H
def owned():
    text=subprocess.check_output(['docker','top',m.CONTAINER,'-eo','pid,comm,args'],text=True)
    return [x for x in text.splitlines() if 'live_publications' in x or any(str(H/y) in x for y in ('replay.py','resume_storage.py'))]
def main():
    m.save(H/'finalizer_storage_start.json',dict(pid=os.getpid(),time=time.time()))
    print('waiting_for_remaining_nine',flush=True)
    while not (H/'replay_complete.json').exists():
        assert not (H/'storage_resume_failed.txt').exists(),'continuation failed; no automatic retry'
        time.sleep(20)
    for _ in range(12):
        if not owned():break
        time.sleep(5)
    assert not owned()
    m.run('restore',['python3',str(H/'stage_products.py'),'restore'])
    m.run('score',[m.PY,str(H/'analyze.py'),'score','all'])
    m.run('verify',[m.PY,str(H/'verify.py')])
    m.run('report',[m.PY,str(H/'report.py')])
    m.run('checkpoint',['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
    assert not owned();m.run('stop',['docker','stop',m.CONTAINER])
    states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',m.CONTAINER,'cbsms_ws_gpu'],text=True)
    assert states.count('exited')==2
    assert not any(p.is_symlink() for p in H.iterdir())
    output_hashes={str(p):m.sha(p) for p in H.rglob('*') if p.is_file() and not p.name.endswith('.log')}
    m.save(H/'closeout.json',dict(pass_=True,time=time.time(),method=2,containers=states,owned_jobs_exited=True,
         temporary_products_restored=True,hesai_checkpoint=(H/'finalize_checkpoint.log').read_text().strip(),
         output_sha256=output_hashes,receiver_replays=12,reused_receiver_cases=2,source_reruns=0,builds=0,
         model_changes=False,storage_only_continuation=True,duplicate_replays=0))
    print('complete',flush=True)
if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'finalize_storage_failed.txt').open('x') as f:f.write(traceback.format_exc())
        raise
