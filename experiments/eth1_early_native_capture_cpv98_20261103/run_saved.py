"""Compile an offline reader only, fixture it, and analyze saved snapshots."""
import hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
C=W/'runs/persistent_compatibility_cpv76_20261101';D=K/'analysis';D.mkdir()
I=str(K).replace(str(W),'/workspace/cbs_gtsam4.3');IC=str(C).replace(str(W),'/workspace/cbs_gtsam4.3')
def read(p):return json.loads(p.read_text())
def save(n,v):
    with (D/n).open('x') as f:json.dump(v,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(a):return subprocess.check_output(a,text=True)
def exited(r):
    p=Path('/proc')/str(r.get('pid',r.get('supervisor_pid')))/'stat'
    return not p.exists() or p.read_text().split()[21]!=r['proc_start']
assert read(H/'capture_complete.json')['passed'] and read(H/'historical_states.json')['passed']
assert exited(read(H/'start.json')) and exited(read(H/'launch.json'))
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=read(K/'prepared/protected_manifest.json');pins.update(read(H/'capture_complete.json')['output_sha256'])
for p in [K/'VisualStudy.cpp',H/'run_saved.py',H/'prepare_history.py',H/'historical_states.json',*sorted((K/'historical_poses').glob('*.txt'))]:pins[str(p)]=sha(p)
for p,h in pins.items():assert sha(p)==h,p
save('inputs.json',pins);save('start.json',dict(pid=os.getpid(),proc_start=Path('/proc/self/stat').read_text().split()[21],time=time.time(),containers=before))
def run(name,args,timeout=240):
    with (D/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,proc_start=Path(f'/proc/{job.pid}/stat').read_text().split()[21],args=args))
        try:rc=job.wait(timeout=timeout)
        except subprocess.TimeoutExpired:job.terminate();job.wait();raise
    save(name+'_receipt.json',dict(exit=rc));assert rc==0,(name,rc)
failure=None
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
    deadline=time.monotonic()+30
    while True:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        if all(r.split()[1] in {'tail','bash','zsh'} for r in top.splitlines()[1:] if r.strip()):break
        assert time.monotonic()<deadline,top
        time.sleep(1)
    for kind in ('compile','link'):
        args=read(C/'release_fixture1'/f'compatibility_{kind}_command.json')
        args=[a.replace(IC+'/persistent_test.cpp',I+'/VisualStudy.cpp').replace(IC+'/release_fixture1/compatibility.o',I+'/analysis/study.o').replace(IC+'/release_fixture1/compatibility',I+'/analysis/study') for a in args]
        run(kind,['docker','exec','cbsms_ws_gpu',*args])
    env=['docker','exec','-e','LD_LIBRARY_PATH='+IC+'/private_build:/workspace/cbs_gtsam4.3/devel/lib','cbsms_ws_gpu']
    run('fixtures',env+[I+'/analysis/study','selftest'])
    run('native',env+[I+'/analysis/study',I+'/prepared/on_source/checkpoints',I+'/historical_poses',I+'/analysis/results'])
    for p,h in pins.items():assert sha(p)==h,p
    save('passed.json',dict(passed=True,source_runs=0,optimizer_runs=0,real_data_updates=23,measurement_unchanged_contrasts=True))
except BaseException as exc:failure=repr(exc);save('failed.json',dict(error=failure))
finally:
    top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    subprocess.run(['docker','stop','--time','20','cbsms_ws_gpu'],check=True)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']);assert before==after
    save('cleanup.json',dict(containers=after,final_jobs=top,owned_stage_jobs_exited=all(exited(read(p)) for p in D.glob('*_process.json'))))
if failure:raise SystemExit(1)
