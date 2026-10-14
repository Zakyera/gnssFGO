"""Isolated stock-policy build/tests/replay. No source estimator invocation."""
import fcntl,hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_slots_fl3i_20261014')
BASE=HERE.parent/'fixed_lag_native_fl3h_20261014'
BASEROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_native_fl3h_20261014')
OLD=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
G=Path('/home/yeranis/results/gnssfgo_fixed_lag_cause_fl3g_20261014')
FL3=Path('/home/yeranis/results/gnssfgo_fixed_lag_replay_fl3_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
ORACLE=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def protected():
    saved=json.loads((BASEROOT/'protected.json').read_text())
    for p,h in saved.items():assert sha(p)==h,('protected changed',p)
    for p in [JOURNAL,FL3/'prefix20.events',*ORACLE.iterdir(),*BASE.iterdir(),*BASEROOT.iterdir(),
              *BASEROOT.joinpath('full_diagnostics').iterdir(),
              Path('/home/yeranis/workspaces/gnssfgo_ct_runtime/rwth_gtsam_src/gtsam/nonlinear/ISAM2.cpp'),
              REPO/'online_fgo/include/solver/IncrementalFixedLagSmoother.h',REPO/'online_fgo/src/solver/IncrementalFixedLagSmoother.cpp',
              *HERE.parent.joinpath('fixed_lag_lifecycle_fl2b_20261013').glob('*')]:
        if p.is_file():saved[str(p)]=sha(p)
    return saved
def source_hashes():return {p.name:sha(p) for p in HERE.iterdir() if p.suffix in ('.cpp','.h')}
def main():
    action=sys.argv[1]
    if action=='prepare':
        saved=protected();ROOT.mkdir(exist_ok=False)
        with (ROOT/'protected.json').open('x') as f:json.dump(saved,f,indent=2)
        print('protected',len(saved));return
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        if action.startswith('build_'):
            source='Replay.cpp' if action=='build_replay' else 'test_gate.cpp'
            cmd=json.loads((OLD/'build_wire3.json').read_text())['command']
            cmd=[x for x in cmd if not x.endswith('.cpp')]
            cmd[6:6]=['-I'+str(HERE),str(HERE/source),str(HERE/'RecordedFixedLagSmoother.cpp'),str(HERE/'SlotISAM2.cpp')]
            if source=='test_gate.cpp':cmd.insert(6,str(BASE/'RecordedFixedLagSmoother.cpp'))
            cmd[-1]=str(ROOT/action)
            if action=='build_sanitize':
                cmd=[x for x in cmd if x not in ('-O3','-DNDEBUG')]
                cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
        elif action in ('test','test_repeat','sanitest'):
            binary='build_sanitize' if action=='sanitest' else 'build_test2'
            build=json.loads((ROOT/(binary+'.json')).read_text());assert build['returncode']==0 and build['sources']==source_hashes()
            cmd=[str(ROOT/binary)]
        elif action in ('prefix','prefix_repeat','full'):
            assert json.loads((ROOT/'synthetic_validation.json').read_text())['passed']
            build=json.loads((ROOT/'build_replay.json').read_text());assert build['returncode']==0 and build['sources']==source_hashes()
            if action=='full':assert json.loads((ROOT/'prefix_validation.json').read_text())['passed']
            cmd=[str(ROOT/'build_replay'),str(JOURNAL if action=='full' else FL3/'prefix20.events'),str(ROOT/(action+'_outputs')),str(ROOT/(action+'_service.csv')),str(ROOT/(action+'_diagnostics'))]
        else:raise ValueError(action)
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0',
                   FL3I_TEST_ROOT=str(ROOT/(action+'_outputs')),ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        os.sched_setaffinity(0,{2});start=time.monotonic();error=None
        with (ROOT/(action+'.log')).open('x') as log:
            try:rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
            except Exception as e:rc=-1;error=repr(e)
        result=dict(returncode=rc,error=error,seconds=time.monotonic()-start,command=cmd,protected_unchanged=saved==protected(),sources=source_hashes(),rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss)
        if rc==0:result['binary_sha256']=sha(cmd[-1] if action.startswith('build_') else cmd[0])
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(action,rc,result['seconds'],flush=True);raise SystemExit(rc)
if __name__=='__main__':main()
