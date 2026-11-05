"""Supplemental bounded coordinate test. No modification/retry of first test."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K = W / 'runs' / H.name
D = H / 'nearby_stage1'
Q = W / 'runs/eth1_early_native_capture_cpv98_20261103/prepared/on_source/checkpoints'
C = W / 'runs/persistent_compatibility_cpv76_20261101'

def read(p): return json.loads(p.read_text())
def inside(p): return str(p).replace(str(W), '/workspace/cbs_gtsam4.3')
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def save(name, value):
    with (D / name).open('x') as f: json.dump(value, f, indent=2)
def call(args): return subprocess.check_output(args, text=True)
def verify(pins):
    for p, v in pins.items(): assert sha(p) == v, p
    return len(pins)
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
def run(name,args,timeout=300):
    with (D/(name+'.log')).open('x') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=p.pid,proc_start=identity(p.pid),args=args))
        try: rc=p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.terminate();p.wait();raise
    save(name+'_receipt.json',dict(exit=rc))
    assert rc==0,(name,rc)

assert read(H/'diagnostic_passed.json')['passed']
assert read(H/'cleanup.json')['owned_jobs_exited']
initial=read(H/'start.json')
assert identity(initial['pid'])!=initial['proc_start']
D.mkdir()
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=read(H/'inputs.json')
for p in [*K.rglob('*'),H/'run_nearby.py',*H.glob('*.json'),*H.glob('*.log')]:
    if p.is_file():pins[str(p)]=sha(p)
verify(pins);save('inputs.json',pins)
save('start.json',dict(pid=os.getpid(),proc_start=identity(os.getpid()),time=time.time(),containers=before))
failure=None
try:
    run('start_container',['docker','start','cbsms_ws_gpu'],60)
    deadline=time.monotonic()+30
    while True:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        if all(r.split()[1] in {'tail','bash','zsh'} for r in top.splitlines()[1:] if r.strip()):break
        assert time.monotonic()<deadline,top
        time.sleep(1)
    save('initial_jobs.json',dict(top=top))
    for kind in ('compile','link'):
        args=read(H/(kind+'_command.json'))
        args=[a.replace(inside(K/'probe.cpp'),inside(K/'nearby.cpp'))
              .replace(inside(K/'probe.o'),inside(K/'nearby.o'))
              .replace(inside(K/'dependencies.d'),inside(K/'nearby_dependencies.d'))
              .replace(inside(K/'probe'),inside(K/'nearby')) for a in args]
        save(kind+'_command.json',args)
        run(kind,['docker','exec','cbsms_ws_gpu',*args])
    env=['docker','exec','-e','LD_LIBRARY_PATH='+inside(C/'private_build')+':'+inside(W/'devel/lib'),'cbsms_ws_gpu']
    run('probe',env+[inside(K/'nearby'),inside(Q),inside(K/'nearby_results')])
    save('passed.json',dict(passed=True,inputs_verified=verify(pins),source_runs=0,optimizer_runs=0))
except BaseException as e:
    failure=repr(e);save('failed.json',dict(error=failure))
finally:
    save('final_jobs.json',dict(top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])))
    run('restore_container',['docker','stop','--time','20','cbsms_ws_gpu'],60)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    assert before==after
    assert all(identity(r['pid'])!=r['proc_start'] for r in map(read,D.glob('*_process.json')))
    save('cleanup.json',dict(containers=after,owned_jobs_exited=True,inputs_verified=verify(pins)))
if failure:raise SystemExit(failure)
print((D/'probe.log').read_text())
