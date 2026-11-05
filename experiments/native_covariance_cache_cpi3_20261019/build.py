"""Private provider build/tests only, using the frozen ABI recipes."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
BASE=WS/'runs/shared_linearization_s4_provider_20261012/release_2'
RECIPES=json.loads((BASE/'commands.json').read_text())

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--sanitize',action='store_true')
    p.add_argument('--attempt',default='')
    p.add_argument('--benchmark',type=Path)
    a=p.parse_args()
    tag=('san' if a.sanitize else 'release')+a.attempt
    out=ROOT/tag
    assert not out.exists(),'refuse overwrite'
    out.mkdir()
    records=[]
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(map(str,[out,BASE,
        WS/'runs/e1b4a_local_update_batching_20261012/release_2',
        WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',WS/'devel',
        '/opt/ros/noetic/lib','/usr/local/cuda/lib64']))
    env['OMP_NUM_THREADS']='1'
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    for k in ['GLIM_SHARED_NATIVE_PROVIDER','GLIM_SHARED_NATIVE_SHADOW','GLIM_REVISION_COVARIANCE_POLICY']:
        env.pop(k,None)
    def run(name,cmd,cwd=ROOT):
        log=out/(name+'.log');assert not log.exists()
        start=time.monotonic()
        with log.open('x') as f:
            code=subprocess.call(cmd,cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT)
        records.append(dict(name=name,command=cmd,cwd=str(cwd),returncode=code,seconds=time.monotonic()-start))
        (out/'commands.json').write_text(json.dumps(records,indent=2)+'\n')
        print(name,code,flush=True)
        if code:raise SystemExit('FAILED '+str(log))
    def mapped(cmd,exe=False):
        result=[]
        for v in cmd:
            v=v.replace(str(BASE/'source'),str(ROOT/'source'))
            # Existing frozen wrapper sources stay unchanged, objects are private.
            if v.startswith(str(BASE)) and v.endswith('.o'):v=str(out/Path(v).name)
            if v==str(BASE/'libgtsam_points.so.1.2.1'):v=str(out/'libgtsam_points.so.1.2.1')
            if v=='-L'+str(BASE):v='-L'+str(out)
            result.append(v)
        if a.sanitize:
            result=[v for v in result if v not in ['-O2','-O3']]
            result+=['-O0','-g1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
            if exe:result+=['-fno-pie','-no-pie']
        return result
    for i in range(5):
        run('provider_'+str(i),mapped(RECIPES[i]['command']),RECIPES[i]['cwd'])
    for name in ['libgtsam_points.so','libgtsam_points.so.1']:
        (out/name).symlink_to('libgtsam_points.so.1.2.1')
    def compile_file(src,obj,exe=False):
        cmd=RECIPES[0]['command'].copy()
        cmd=[str(src) if x==str(BASE/'source/NativeGaussianHistory.cpp') else
             str(obj) if x==str(BASE/'NativeGaussianHistory.o') else x for x in cmd]
        run(obj.stem,mapped(cmd,exe))
    compile_file(ROOT/'source/OracleHistory.cpp',out/'oracle.o')
    compile_file(ROOT/'source/test_revision.cpp',out/'revision.o',True)
    compile_file(ROOT/'test_cache.cpp',out/'cache_tests.o',True)
    cmd=RECIPES[9]['command'].copy()
    cmd=[str(out/'revision.o') if x==str(BASE/'test_lifecycle.o') else
         str(out/'cache_tests.o') if x==str(BASE/'test_native_gaussian.o') else
         str(out/'tests') if x==str(BASE/'test_shadow') else x for x in cmd]
    cmd.insert(1,str(out/'oracle.o'))
    run('tests_link',mapped(cmd,True))
    run('tests',[str(out/'tests')])
    run('repeat',[str(out/'tests')])
    if not a.sanitize:
        compile_file(ROOT/'benchmark.cpp',out/'benchmark.o')
        cmd=[str(out/'benchmark.o') if x==str(BASE/'test_lifecycle.o') else
             str(out/'oracle.o') if x==str(BASE/'test_native_gaussian.o') else
             str(out/'benchmark') if x==str(BASE/'test_shadow') else x for x in RECIPES[9]['command']]
        cmd=[v for v in cmd if v not in ['-lgtest_main','-lgtest']]
        run('benchmark_link',mapped(cmd))
        if a.benchmark:run('benchmark',[str(out/'benchmark'),str(a.benchmark)])
    (out/'passed.json').write_text(json.dumps(dict(passed=True,
        scope='provider/lifecycle/revision/covariance tests; vendor libraries not fully sanitized'))+'\n')

if __name__=='__main__':main()
