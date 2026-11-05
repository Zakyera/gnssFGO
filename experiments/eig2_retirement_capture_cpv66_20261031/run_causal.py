"""Saved-event tests only, after both captures and controller exit; no retry."""
import hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name;D=K/'causal';C=W/'runs/kimera_on_incremental_refresh_cpv40_20261029'
I='/workspace/cbs_gtsam4.3/runs/'+H.name
assert json.loads((H/'capture_complete.json').read_text())['passed']
assert (H/'capture_analysis.json').exists()
for name in ('start.json','launch.json'):
    assert not Path('/proc',str(json.loads((H/name).read_text())['pid'])).exists()
def save(name,obj):
    with (D/name).open('x') as f:json.dump(obj,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=json.loads((K/'prepared/protected_manifest.json').read_text())
pins.update(json.loads((H/'capture_analysis.json').read_text())['input_sha256'])
for p in (D/'Retirement.cpp',H/'causal_targets.txt'):pins[str(p)]=sha(p)
for p,h in pins.items():assert sha(p)==h,p
save('inputs.json',pins);save('start.json',dict(pid=os.getpid(),time=time.time(),containers=before))
# Copy the small generated target manifest into the container-visible evidence
# directory, preserving the original; never reselect by counterfactual benefit.
with (D/'targets.txt').open('xb') as f:f.write((H/'causal_targets.txt').read_bytes())
def run(name,args):
    with (D/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,args=args));rc=job.wait()
    save(name+'_receipt.json',dict(exit=rc));assert rc==0,(name,rc)
failure=None
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
    template=json.loads((C/'actual_test_commands.json').read_text())
    for mode in ('release','san'):
        for kind,args0 in template.items():
            args=[a.replace('/workspace/cbs_gtsam4.3/runs/'+C.name,I) for a in args0]
            args=[a.replace(I+'/test.cpp',I+'/causal/Retirement.cpp').replace(I+'/test.o',I+'/causal/'+mode+'.o') for a in args]
            args=[I+'/causal/'+mode if a==I+'/test' else a for a in args]
            if mode=='san':
                args=[a for a in args if a!='-O3']+['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
                args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie']
            run(mode+'_'+kind,['docker','exec','cbsms_ws_gpu',*args])
        run(mode+'_fixtures',['docker','exec','-e','LD_LIBRARY_PATH='+I+'/private_build:/workspace/cbs_gtsam4.3/devel/lib',
             '-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1',
             'cbsms_ws_gpu',I+'/causal/'+mode,'selftest'])
    run('native',['docker','exec','-e','LD_LIBRARY_PATH='+I+'/private_build:/workspace/cbs_gtsam4.3/devel/lib',
                 'cbsms_ws_gpu',I+'/causal/release',I+'/prepared/full_on_source/checkpoints',I+'/causal/targets.txt',I+'/causal/results'])
    for p,h in pins.items():assert sha(p)==h,p
    save('passed.json',dict(passed=True,real_native_steps=88,events=11,fixture_steps_separate=True,vendor_sanitized=False))
except BaseException as exc:
    failure=repr(exc);save('failed.json',dict(error=failure))
finally:
    top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    assert all(r.split()[1] in ('tail','zsh') for r in top.splitlines()[1:] if r.strip()),top
    subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    assert before==after
    save('cleanup.json',dict(containers=after,owned_jobs_exited=top))
if failure:raise SystemExit(1)
