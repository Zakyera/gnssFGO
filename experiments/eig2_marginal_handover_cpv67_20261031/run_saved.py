"""One isolated saved-data attempt; no source launch and no automatic retry."""
import datetime,hashlib,json,os,subprocess,sys,time
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name
OLD=W/'runs/eig2_retirement_capture_cpv66_20261031'
OLDH=H.parent/OLD.name
ATTEMPT=sys.argv[1] if len(sys.argv)==2 else 'attempt1'
assert ATTEMPT.isalnum(), 'explicit attempt identifier only'
D=K/ATTEMPT
D.mkdir(exist_ok=False)
I='/workspace/cbs_gtsam4.3/runs/'+H.name
O='/workspace/cbs_gtsam4.3/runs/'+OLD.name
def stamp():return datetime.datetime.now().astimezone().isoformat()
def save(name,obj):
    with (D/name).open('x') as f:json.dump(obj,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
def run(name,args):
    with (D/(name+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(name+'_process.json',dict(pid=job.pid,args=args,started=stamp()))
        rc=job.wait()
    save(name+'_receipt.json',dict(exit=rc,finished=stamp()))
    assert rc==0,(name,rc)
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
assert json.loads((OLDH/'complete.json').read_text())['passed']
pins=json.loads((OLD/'causal/inputs.json').read_text())
pins.update(json.loads((OLDH/'snapshot_sha256.json').read_text()))
for p in [K/'Handover.cpp',H/'PROTOCOL.txt',Path(__file__),OLD/'VioBackend.cpp',W/'src/cbs/src/bpsam/bpsam.cpp',W/'src/gtsam/gtsam/nonlinear/ISAM2.cpp']:
    pins[str(p)]=sha(p)
for p,h in pins.items():assert sha(p)==h,p
save('inputs.json',pins)
save('start.json',dict(pid=os.getpid(),started=stamp(),containers=before))
failure=None;started=False
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True);started=True
    for mode in ('release','san'):
        for kind in ('compile','link'):
            base=json.loads((OLD/f'causal/release_{kind}_process.json').read_text())['args']
            args=[a.replace(O+'/causal/Retirement.cpp',I+'/Handover.cpp').replace(O+'/causal/release.o',I+'/'+ATTEMPT+'/'+mode+'.o') for a in base]
            args=[I+'/'+ATTEMPT+'/'+mode if a==O+'/causal/release' else a for a in args]
            if mode=='san':
                args=[a for a in args if a!='-O3']+['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
                args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie']
            run(mode+'_'+kind,args)
        run(mode+'_fixtures',['docker','exec','-e','LD_LIBRARY_PATH='+O+'/private_build:/workspace/cbs_gtsam4.3/devel/lib',
            '-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1',
            'cbsms_ws_gpu',I+'/'+ATTEMPT+'/'+mode,'selftest',I+'/'+ATTEMPT+'/'+mode+'_fixtures'])
    run('native',['docker','exec','-e','LD_LIBRARY_PATH='+O+'/private_build:/workspace/cbs_gtsam4.3/devel/lib',
        'cbsms_ws_gpu',I+'/'+ATTEMPT+'/release','real',O+'/prepared/full_on_source/checkpoints/update_250',I+'/'+ATTEMPT+'/results'])
    for p,h in pins.items():assert sha(p)==h,p
    save('passed.json',dict(passed=True,finished=stamp(),sources_launched=0,production_changed=False,vendor_libraries_sanitized=False))
except BaseException as exc:
    failure=repr(exc);save('failed.json',dict(error=failure,finished=stamp()))
finally:
    if started:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        assert all(r.split()[1] in ('tail','zsh') for r in top.splitlines()[1:] if r.strip()),top
        subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
    else:top='not started'
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    assert before==after
    save('cleanup.json',dict(containers=after,remaining_processes_before_restore=top,finished=stamp()))
if failure:raise SystemExit(failure)
print(json.dumps(dict(passed=True,attempt=str(D))))
