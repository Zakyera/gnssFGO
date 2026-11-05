"""Build/test private header candidate only; never replace an estimator library."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import difflib

ROOT=Path(__file__).resolve().parent
H=ROOT/"lifecycle_trace1"
H.mkdir()
E=ROOT.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/ROOT.name
BASE=E/'prior_conversion_offline_cpv100_20261103'
F=W/'runs/persistent_compatibility_cpv76_20261101/release_fixture1'
S=F/'source_snapshot'
C=F.parent
Q=W/'runs/eth1_early_native_capture_cpv98_20261103/prepared/on_source/checkpoints'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(name,value):
    with (H/name).open('x') as f:json.dump(value,f,indent=2)
def inside(p):return str(p).replace(str(W),'/workspace/cbs_gtsam4.3')
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
def verify(pins):
    for p,v in pins.items():assert sha(p)==v,p
    return len(pins)
def call(args):return subprocess.check_output(args,text=True)
def run(name,args,timeout=300):
    with (H/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,proc_start=identity(job.pid),args=args))
        try:rc=job.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            job.terminate();job.wait();raise
    save(name+'_receipt.json',dict(exit=rc))
    assert rc==0,(name,rc)

assert not (H/'start.json').exists(),'one-shot candidate stage already started'
assert read(BASE/'complete.json')['passed']
for d in (BASE,BASE/'nearby_stage1'):
    r=read(d/'start.json');assert identity(r['pid'])!=r['proc_start']
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
assert read(ROOT/'cleanup.json')['owned_jobs_exited']
original=read(ROOT/'start.json');assert identity(original['pid'])!=original['proc_start']
pins=read(ROOT/'inputs.json')
for p in [*ROOT.glob('*.json'),*ROOT.glob('*.log'),*ROOT.glob('*.patch'),K/'test',K/'test.o',K/'dependencies.d',*sorted((K/'results').rglob('*'))]:
    if p.is_file():pins[str(p)]=sha(p)
pins.update(read(BASE/'complete.json')['output_sha256'])
for p in [ROOT/'PROTOCOL.txt',ROOT/'run_trace.py',K/'lifecycle_trace.cpp',K/'test_v2.cpp',*sorted((K/'candidate').glob('*.h'))]:pins[str(p)]=sha(p)
verify(pins);save('inputs.json',pins)
for name in ('OriginalChartPrior.h','NativePriorRefresh.h'):
    with (H/(name+'.patch')).open('x') as f:
        f.writelines(difflib.unified_diff((S/name).read_text().splitlines(True),(K/'candidate'/name).read_text().splitlines(True),fromfile=str(S/name),tofile=str(K/'candidate'/name)))
save('start.json',dict(pid=os.getpid(),proc_start=identity(os.getpid()),time=time.time(),containers=before,input_files=len(pins)))
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
        args=read(BASE/(kind+'_command.json'))
        old=W/'runs'/BASE.name
        replacement={inside(old/'probe.cpp'):inside(K/'lifecycle_trace.cpp'),inside(old/'probe.o'):inside(K/'lifecycle_trace.o'),
                     inside(old/'probe'):inside(K/'lifecycle_trace'),inside(old/'dependencies.d'):inside(K/'lifecycle_trace.d')}
        args=[replacement.get(a,a) for a in args]
        if kind=='compile':args=[('-O0' if a=='-O3' else a) for a in args]+['-g']
        else:args+=['-rdynamic']
        save(kind+'_command.json',args)
        run(kind,['docker','exec','cbsms_ws_gpu',*args])
    env=['docker','exec','-e','LD_LIBRARY_PATH='+inside(C/'private_build')+':'+inside(W/'devel/lib'),'cbsms_ws_gpu']
    run('tests',env+[inside(K/'lifecycle_trace')])
    save('debug_complete.json',dict(debugger_completed=True,inputs_verified=verify(pins),source_runs=0,estimator_builds=0))
except BaseException as e:
    failure=repr(e);save('failed.json',dict(error=failure))
finally:
    save('final_jobs.json',dict(top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])))
    run('restore_container',['docker','stop','--time','20','cbsms_ws_gpu'],60)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']);assert before==after
    assert all(identity(r['pid'])!=r['proc_start'] for r in map(read,H.glob('*_process.json')))
    save('cleanup.json',dict(containers=after,owned_jobs_exited=True,inputs_verified=verify(pins)))
if failure:raise SystemExit(failure)
print((H/'tests.log').read_text())



