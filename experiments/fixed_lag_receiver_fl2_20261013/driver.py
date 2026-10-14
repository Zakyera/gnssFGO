"""Create-once synthetic FL2 build/test commands; cannot launch a journal replay."""
import hashlib,json,os,subprocess,sys,time,fcntl
from pathlib import Path
HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_receiver_fl2_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
E2=REPO/'experiments/receiver_endpoint_reuse_e2b_20261013'
OLD=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def protected():
    files=list(E2.glob('*'))
    files+=list((REPO/'online_fgo/include/solver').glob('*'))
    files+=list((REPO/'online_fgo/src/solver').glob('*'))
    files+=[REPO/'online_fgo/include/factor/motion/GPWNOAPrior.h',
      REPO/'online_fgo/include/model/gp_interpolator/GPWNOAInterpolator.h',
      RT/'lib/libgtsam.so.4',RT/'lib/libonline_fgo_solver.so',
      Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/build/replay')]
    return {str(p):digest(p) for p in files if p.is_file()}
def main():
    action=sys.argv[1]; ROOT.mkdir(exist_ok=True)
    with (ROOT/'lock').open('a') as lock:
      fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
      if action=='preserve':
        with (ROOT/'protected.json').open('x') as f:json.dump(protected(),f,indent=2)
        print('protected',len(protected()));return
      saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
      with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
      if action.startswith('build_'):
        cmd=json.loads((OLD/'candidate_build.json').read_text())['commands'][0]
        cmd=[a for a in cmd if not a.endswith('.cpp')]
        cmd[6:6]=['-I'+str(HERE),'-I'+str(E2),str(HERE/'test_gate.cpp')]
        cmd[-1]=str(ROOT/action)
        if 'sanitize' in action:
          cmd.remove('-O3');cmd.remove('-DNDEBUG')
          cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1',
           '-fno-omit-frame-pointer','-fsanitize=address,undefined']
      else:
        # Explicit binary passed to allow preserved development build revisions.
        cmd=[str(ROOT/sys.argv[2])]
      env=dict(os.environ,LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',
        ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1',
        E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
      env.pop('LD_PRELOAD',None);os.sched_setaffinity(0,{2})
      start=time.monotonic()
      with (ROOT/(action+'.log')).open('x') as log:
        rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1200).returncode
      result=dict(command=cmd,returncode=rc,seconds=time.monotonic()-start,
          protected_unchanged=saved==protected(),code={str(p):digest(p) for p in HERE.glob('*') if p.is_file()})
      with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
      print(action,json.dumps({k:v for k,v in result.items() if k not in ('command','code')}),flush=True)
      raise SystemExit(rc)
if __name__=='__main__':main()
