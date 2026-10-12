"""Private CPU build only; never rebuild or install production libraries."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import time

WS = Path('/workspace/cbs_gtsam4.3')
ROOT = WS / 'runs/shared_linearization_s2_20261012'
LIB = WS / 'runs/e1b4a_local_update_batching_20261012/release_2'

def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['release', 'ubsan', 'asan'])
    p.add_argument('attempt')
    p.add_argument('--main-cache', action='store_true')
    args = p.parse_args()
    out = ROOT / (args.mode + '_' + args.attempt)
    out.mkdir(exist_ok=False)
    shutil.copytree(ROOT / 'authoring', out / 'source')
    includes = ['-I'+str(out/'source'), '-I'+str(WS/'src/cbs/include'),
                '-isystem', str(WS/'devel/include/gtsam/3rdparty/Eigen'),
                '-isystem', str(WS/'devel/include')]
    if args.main_cache:
        includes += ['-I'+str(WS/'src/gtsam_points/include')]
    options = ['-std=c++17', '-O2', '-DNDEBUG', '-pthread']
    if args.mode != 'release':
        options += ['-O1', '-g1', '-fno-omit-frame-pointer',
                    '-fsanitize='+('undefined' if args.mode=='ubsan' else 'address,undefined'),
                    '-fno-sanitize-recover=all']
    if args.mode == 'asan':
        options += ['-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    commands = []
    start = time.monotonic()
    with (out/'build.log').open('x') as log:
        units=['NativeGaussianHistory','test_native_gaussian']
        if args.main_cache: units+=['test_main_cache']
        for name in units:
            cmd = ['c++']+includes+options+['-c',str(out/'source'/(name+'.cpp')),'-o',str(out/(name+'.o'))]
            commands.append(cmd)
            result = subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
            if result.returncode: break
        if not result.returncode:
            cmd = ['c++']+options+[str(out/(n+'.o')) for n in units]
            cmd += ['-o',str(out/'test_native_gaussian'),'-L'+str(LIB),'-L'+str(WS/'devel/lib'),
                    '-Wl,-rpath,'+str(LIB)+':'+str(WS/'devel/lib'),'-lgtsam','-lgtest_main','-lgtest','-ltbb']
            if args.main_cache:
                old=WS/'runs/e1b_local_profile_20261012/release_2'
                cmd += ['-L'+str(old),'-Wl,-rpath,'+str(old),'-lgtsam_points',
                        '-lgtsam_points_cuda','-le1b_profile','-L/usr/local/cuda/lib64','-lcudart','-fopenmp']
            commands.append(cmd)
            result = subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
    (out/'commands.json').write_text(json.dumps(commands,indent=2))
    summary = {'returncode':result.returncode,'seconds':time.monotonic()-start}
    if not result.returncode:
        summary['sha256']=hashlib.sha256((out/'test_native_gaussian').read_bytes()).hexdigest()
    (out/'build_result.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary),flush=True)
    if result.returncode: print((out/'build.log').read_text()[-8000:])
    raise SystemExit(result.returncode)
if __name__ == '__main__':main()
