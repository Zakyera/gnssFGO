"""Retained focused tests or explicitly requested fixture benchmark, no sensors."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_local_update_batching_20261012'
E1=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
OLD=WS/'runs/e1b_local_profile_20261012/release_2'
def main():
    p=argparse.ArgumentParser();p.add_argument('build');p.add_argument('--test-attempt',default='2')
    p.add_argument('--tag',default='checks1');p.add_argument('--benchmark',action='store_true')
    p.add_argument('--cpu-only',action='store_true');p.add_argument('--dispatch-only',action='store_true');a=p.parse_args()
    build=ROOT/a.build;out=build/a.tag;out.mkdir(exist_ok=False)
    env=os.environ.copy();env.pop('CBS_E1B_PROFILE_PATH',None)
    env.pop('CBS_E1B_LOCAL_UPDATE_GPU_BATCHING',None)
    env['LD_LIBRARY_PATH']=f'{build}:{OLD}:{E1}:{WS}/devel/lib:'+env.get('LD_LIBRARY_PATH','')
    env['OMP_NUM_THREADS']='1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    jobs=[('test_dispatch','UpdateBatchScope.*:LocalLinearizerCpu.*' if a.cpu_only else '*'),
          ('test_solver','SolverProfile.*' if a.cpu_only else 'SolverProfile.*:UpdateBatchSolver.*')]
    if a.dispatch_only:jobs=jobs[:1]
    if a.benchmark:
        assert (build/'checks3/passed.json').exists(),'correctness must pass first'
        jobs=[('test_solver','UpdateBatchSolver.DISABLED_AlternatingBenchmark')]
    records=[]
    for name,pattern in jobs:
        exe=build/(name+'_'+a.test_attempt)
        resolved=subprocess.check_output(['ldd',str(exe)],env=env,text=True)
        (out/(name+'_ldd.txt')).write_text(resolved)
        assert str(build/'libgtsam.so.4') in resolved, resolved
        cmd=['taskset','-c','2',str(exe),'--gtest_filter='+pattern,'--gtest_output=xml:'+str(out/(name+'.xml'))]
        if a.benchmark:cmd.append('--gtest_also_run_disabled_tests')
        begin=time.monotonic()
        with (out/(name+'.log')).open('x') as log:
            result=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
        records.append(dict(command=cmd,seconds=time.monotonic()-begin,returncode=result.returncode))
        (out/'commands.json').write_text(json.dumps(records,indent=2)+'\n')
        print(name,result.returncode,round(records[-1]['seconds'],2),flush=True)
        if result.returncode:
            print((out/(name+'.log')).read_text()[-6000:]);raise SystemExit(result.returncode)
    if not a.benchmark and not a.dispatch_only:
        # Match the historical validator's splitlines() convention (no newline).
        lines=(out/'test_solver.log').read_bytes().splitlines()
        fingerprints=[x for x in lines if x.startswith(b'E1B_MATH,')]
        assert len(fingerprints)==1
        digest=hashlib.sha256(fingerprints[0]).hexdigest()
        assert digest=='f156d2b49e28f9d56f1ac6305c0a1ee1c7a0ef64d658c36ff6107d023b4e3d12',digest
    (out/'passed.json').write_text(json.dumps(dict(passed=True,library_path=env['LD_LIBRARY_PATH'],cpu_only=a.cpu_only),indent=2)+'\n')
if __name__=='__main__':main()
