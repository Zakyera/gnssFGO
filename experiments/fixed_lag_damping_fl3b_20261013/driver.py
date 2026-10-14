"""Bounded FL3b diagnostic, create-once products, no source or receiver launch."""
import fcntl,hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_damping_fl3b_20261013')
OLD=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl3a_20261013')
WIRE=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def protect():
    saved=json.loads((OLD/'protected.json').read_text())
    for p,h in saved.items():assert sha(Path(p))==h,('protected change',p)
    for folder in (HERE.parent/'fixed_lag_failure_fl3a_20261013',OLD/'diagnostics'/'snapshot'):
        for p in folder.iterdir():
            if p.is_file():saved[str(p)]=sha(p)
    for p in (OLD/'build_snapshot',OLD/'validation.json',RT/'lib/libgtsam.so.4'):
        saved[str(p)]=sha(p)
    for p in (RT/'include/gtsam/nonlinear').glob('*LevenbergMarquardt*'):
        if p.is_file():saved[str(p)]=sha(p)
    return saved
def main():
    action=sys.argv[1]
    if action=='prepare':
        ROOT.mkdir(exist_ok=False)
        with (ROOT/'protected.json').open('x') as f:json.dump(protect(),f,indent=2)
        print('protected',len(protect()));return
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0',ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        if action.startswith('build_'):
            cmd=json.loads((WIRE/'build_wire3.json').read_text())['command']
            source='Fixture.cpp' if 'fixture' in action else 'test_gate.cpp'
            cmd=[str(HERE/source) if x.endswith('/Replay.cpp') else x for x in cmd];cmd[-1]=str(ROOT/action)
            if 'sanitize' in action:
                cmd.remove('-O3');cmd.remove('-DNDEBUG')
                cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
        elif action.startswith(('test_release','test_sanitize')):
            binary=sys.argv[2] if len(sys.argv)>2 else ('build_release3' if action.startswith('test_release') else 'build_sanitize')
            assert json.loads((ROOT/(binary+'.json')).read_text())['returncode']==0
            cmd=[str(ROOT/binary)]
        elif action in ('fixture','repeat'):
            for required in ('test_release2','test_sanitize','build_fixture'):
                assert json.loads((ROOT/(required+'.json')).read_text())['returncode']==0
            if action=='repeat':assert json.loads((ROOT/'fixture_outputs/summary.json').read_text())['passed']
            cmd=[str(ROOT/'build_fixture'),str(OLD/'diagnostics/snapshot'),str(ROOT/(action+'_outputs'))]
        else:raise ValueError(action)
        os.sched_setaffinity(0,{2});start=time.monotonic()
        with (ROOT/(action+'.log')).open('x') as log:
            rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600).returncode
        result=dict(command=cmd,returncode=rc,seconds=time.monotonic()-start,protected_unchanged=protect()==saved,
          max_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
        if rc==0:result['binary_sha256']=sha(Path(cmd[-1] if action.startswith('build_') else cmd[0]))
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(action,json.dumps({k:v for k,v in result.items() if k not in ('command','code')}),flush=True)
        raise SystemExit(rc)
if __name__=='__main__':main()
