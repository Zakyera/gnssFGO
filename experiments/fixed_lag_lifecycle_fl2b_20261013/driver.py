"""Reuse preserved compile/link flags. Synthetic receiver tests only, no dataset."""
import importlib.util,json,os,sys,subprocess,time,fcntl
from pathlib import Path
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'fixed_lag_boundary_fl2a_20261013'
spec=importlib.util.spec_from_file_location('fl2adriver',OLD/'driver.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
base=old.base
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
def protected():
    d=old.protected()
    for p in [*OLD.glob('*'),
      Path('/home/yeranis/results/gnssfgo_fixed_lag_boundary_fl2a_20261013/build_release3'),
      Path('/home/yeranis/results/gnssfgo_fixed_lag_boundary_fl2a_20261013/validation.json'),
      base.REPO/'online_fgo/include/offline_process/CausalSupportClock.h']:
        if p.is_file():d[str(p)]=base.digest(p)
    return d
def main():
    action=sys.argv[1];ROOT.mkdir(exist_ok=True)
    with (ROOT/'lock').open('a') as lock:
      fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
      if action=='preserve':
        with (ROOT/'protected.json').open('x') as f:json.dump(protected(),f,indent=2)
        print('protected',len(protected()));return
      saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
      with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
      if action.startswith('build_'):
        cmd=json.loads((base.OLD/'candidate_build.json').read_text())['commands'][0]
        cmd=[a for a in cmd if not a.endswith('.cpp')]
        source='Replay.cpp' if 'wire' in action else 'test_gate.cpp'
        cmd[6:6]=['-I'+str(HERE),'-I'+str(base.E2),str(HERE/source)];cmd[-1]=str(ROOT/action)
        if 'sanitize' in action:
          cmd.remove('-O3');cmd.remove('-DNDEBUG')
          cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
      else:
        cmd=[str(ROOT/sys.argv[2])]
        if action.startswith('wire_'):cmd.extend([sys.argv[3],str(ROOT/(action+'_outputs'))])
      env=dict(os.environ,LD_LIBRARY_PATH=str(base.RT/'lib'),OMP_NUM_THREADS='1',
        ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1',
        E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0',FL2B_TEST_ROOT=str(ROOT/(action+'_outputs')))
      env.pop('LD_PRELOAD',None);os.sched_setaffinity(0,{2});start=time.monotonic()
      with (ROOT/(action+'.log')).open('x') as log:
        rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
      result=dict(command=cmd,returncode=rc,seconds=time.monotonic()-start,protected_unchanged=saved==protected(),
        code={str(p):base.digest(p) for p in HERE.glob('*') if p.is_file()})
      with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
      print(action,json.dumps({k:v for k,v in result.items() if k not in ('command','code')}),flush=True);raise SystemExit(rc)
if __name__=='__main__':main()
