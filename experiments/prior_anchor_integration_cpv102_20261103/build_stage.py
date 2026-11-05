"""Finite private-build/test stage. Never touches installed targets or launches sources."""
import difflib,hashlib,json,os,subprocess,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
C=W/'runs/persistent_compatibility_cpv76_20261101';F=C/'release_fixture1'
stage=sys.argv[1];assert stage in ('baseline','candidate','tests')
D=H/stage;D.mkdir()
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,v):
    with (D/n).open('x') as f:json.dump(v,f,indent=2)
def inside(p):return str(p).replace(str(W),'/workspace/cbs_gtsam4.3')
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
def verify(pins):
    for p,v in pins.items():assert sha(p)==v,p
    return len(pins)
def call(args):return subprocess.check_output(args,text=True)
def run(name,args,timeout=900):
    with (D/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,proc_start=identity(job.pid),args=args))
        try:rc=job.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            job.terminate();job.wait();raise
    save(name+'_receipt.json',dict(exit=rc));assert rc==0,(name,rc)
def docker(args,lib):
    return ['docker','exec','-e','LD_LIBRARY_PATH='+inside(lib)+':'+inside(W/'devel/lib'),
            'cbsms_ws_gpu',*args]
for predecessor in ({'baseline':[], 'candidate':['baseline'], 'tests':['baseline','candidate']}[stage]):
    d=H/predecessor;assert read(d/'passed.json')['passed']
    r=read(d/'start.json');assert identity(r['pid'])!=r['proc_start']
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
pins=read(H/'inputs.json')
for root in (H,K):
    for p in root.rglob('*'):
        if p.is_file() and not p.is_relative_to(D):pins[str(p)]=sha(p)
verify(pins);save('inputs.json',pins)
save('start.json',dict(pid=os.getpid(),proc_start=identity(os.getpid()),time=time.time(),stage=stage,containers=before))
failure=None
try:
    run('container_start',['docker','start','cbsms_ws_gpu'],60)
    deadline=time.monotonic()+30
    while True:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        if all(r.split()[1] in {'tail','bash','zsh'} for r in top.splitlines()[1:] if r.strip()):break
        assert time.monotonic()<deadline,top;time.sleep(1)
    save('idle.json',dict(top=top))
    if stage in ('baseline','candidate'):
        source=K/stage;build=K/('baseline_build' if stage=='baseline' else 'private_build')
        old=inside(C);new=inside(source)
        commands=read(C/'backend_fixture1/commands.json')
        compile_args=[a.replace(old,new) for a in commands['compile']]
        compile_args=[inside(build/'VioBackend.o') if a==inside(source/'VioBackend.cpp.o') else a for a in compile_args]
        compile_args+=['-MMD','-MF',inside(build/'VioBackend.d')]
        if stage=='baseline':compile_args+=['-ffile-prefix-map='+new+'='+old]
        save('backend_compile_command.json',compile_args)
        run('backend_compile',docker(compile_args,build))
        if stage=='candidate':
            archive_args=[a.replace('VioBackend.cpp','PriorFailureArchive.cpp').replace('VioBackend.o','PriorFailureArchive.o').replace('VioBackend.d','PriorFailureArchive.d') for a in compile_args]
            save('archive_compile_command.json',archive_args);run('archive_compile',docker(archive_args,build))
        args=commands['link'];link=[]
        cwd=Path(read(H/'prepared.json')['build_cwd'])
        for a in args:
            if a==inside(C/'VioBackend.cpp.o'):a=inside(build/'VioBackend.o')
            elif a==inside(C/'private_build/libkimera_vio.so'):a=inside(build/'libkimera_vio.so')
            elif a==inside(W/'devel/lib/libcbs.so'):a=inside(build/'libcbs.so')
            elif a.endswith(('.o','.a')) and not a.startswith('/'):a=inside(cwd/a)
            link.append(a)
        if stage=='candidate':link.insert(link.index(inside(build/'VioBackend.o'))+1,inside(build/'PriorFailureArchive.o'))
        save('backend_link_command.json',link);run('backend_link',docker(link,build))
        run('dependencies',docker(['ldd','-r',inside(build/'libkimera_vio.so')],build))
        log=(D/'dependencies.log').read_text();assert 'not found' not in log and 'undefined symbol' not in log
        if stage=='baseline':
            expected=read(H/'prepared.json')['original_kimera_sha256'];actual=sha(build/'libkimera_vio.so')
            save('baseline_identity.json',dict(expected=expected,actual=actual,identical=actual==expected))
            assert actual==expected,'Reconstructed baseline DSO differs; inspect before candidate build'
        save('libraries.json',{p.name:sha(p) for p in build.glob('*.so')})
    else:
        build=K/'private_build';source=K/'candidate'
        for label,src in [('compatibility',source/'persistent_test.cpp'),('archive_integration',K/'archive_integration.cpp')]:
            for kind in ('compile','link'):
                args=read(F/('compatibility_'+kind+'_command.json'))
                mapped=[]
                for a in args:
                    if a==inside(C/'persistent_test.cpp'):a=inside(src)
                    elif a==inside(F/'compatibility.o'):a=inside(K/(label+'.o'))
                    elif a==inside(F/'compatibility'):a=inside(K/label)
                    elif a.startswith('-I'+inside(C)):a=a.replace(inside(C),inside(source),1)
                    elif a==inside(C/'private_build/libcbs.so'):a=inside(build/'libcbs.so')
                    mapped.append(a)
                if kind=='compile':mapped+=['-MMD','-MF',inside(K/(label+'.d'))]
                else:mapped += [inside(build/'libkimera_vio.so'),'-ldl']
                save(label+'_'+kind+'_command.json',mapped);run(label+'_'+kind,docker(mapped,build))
            run(label,docker([inside(K/label),inside(K/(label+'_results'))],build),300)
        run('baseline_compatibility',docker([inside(F/'compatibility'),inside(K/'baseline_compatibility_results')],K/'baseline_build'),300)
    save('passed.json',dict(passed=True,stage=stage,inputs_verified=verify(pins),source_runs=0,deployed=False))
except BaseException as e:
    failure=repr(e);save('failed.json',dict(error=failure))
finally:
    save('final_jobs.json',dict(top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])))
    run('container_restore',['docker','stop','--time','20','cbsms_ws_gpu'],60)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']);assert after==before
    assert all(identity(r['pid'])!=r['proc_start'] for r in map(read,D.glob('*_process.json')))
    save('cleanup.json',dict(containers=after,owned_jobs_exited=True,inputs_verified=verify(pins)))
if failure:raise SystemExit(failure)
print('PASS',stage,'; containers restored; inputs preserved.')
