"""Private S4c objects; existing reader/tests/runtime reused unchanged."""
from pathlib import Path
import argparse,hashlib,json,subprocess,time
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
WS=Path('/workspace/cbs_gtsam4.3')
S4A=WS/'runs/shared_linearization_s4_timing_20261012'
S4B=WS/'runs/shared_linearization_s4_eigenvalues_20261012'

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['release','asan']);a=p.parse_args()
    out=ROOT/a.mode;out.mkdir(exist_ok=False)
    opts=['-std=c++17','-O2','-DNDEBUG','-pthread']
    if a.mode=='asan':opts+=['-O1','-g1','-fno-omit-frame-pointer','-fsanitize=address,undefined','-fno-sanitize-recover=all','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    inc=['-I'+str(HERE),'-I'+str(WS/'src/cbs/include'),'-isystem',str(WS/'devel/include/gtsam/3rdparty/Eigen'),'-isystem',str(WS/'devel/include')]
    lib=WS/'runs/e1b4a_local_update_batching_20261012/release_2'
    link=['-L'+str(lib),'-L'+str(WS/'devel/lib'),'-Wl,-rpath,'+str(lib)+':'+str(WS/'devel/lib'),'-lgtsam','-ltbb']
    if a.mode=='asan':link+=['-no-pie']
    commands=[];start=time.monotonic()
    with (out/'build.log').open('x') as log:
        def run(cmd):
            t=time.monotonic();r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
            commands.append(dict(command=cmd,seconds=time.monotonic()-t,returncode=r.returncode))
            (out/'commands.json').write_text(json.dumps(commands,indent=2));r.check_returncode()
        for name in ['candidate','rank_access','certificate_tests']:
            run(['c++',*opts,*inc,'-c',str(HERE/(name+'.cpp')),'-o',str(out/(name+'.o'))])
        run(['c++',*opts,str(out/'candidate.o'),str(S4A/a.mode/'replay.o'),'-o',str(out/'candidate'),*link])
        run(['c++',*opts,str(out/'rank_access.o'),str(out/'certificate_tests.o'),str(S4B/a.mode/'rank_tests.o'),str(S4A/a.mode/'tests.o'),'-o',str(out/'tests'),*link,'-lgtest_main','-lgtest'])
    result=dict(passed=True,seconds=time.monotonic()-start,binaries={n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['candidate','tests']})
    (out/'complete.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
