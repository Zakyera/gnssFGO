from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time

def main():
    build=Path(sys.argv[1]);out=build/sys.argv[2];out.mkdir(exist_ok=False)
    ws=Path('/workspace/cbs_gtsam4.3')
    env=os.environ.copy()
    env.pop('GLIM_SHARED_NATIVE_SHADOW',None)
    env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
       ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',UBSAN_OPTIONS='print_stacktrace=1:halt_on_error=1')
    env['LD_LIBRARY_PATH']=':'.join(map(str,[build,
       ws/'runs/e1b4a_local_update_batching_20261012/release_2',
       ws/'runs/e1b_local_profile_20261012/release_2',ws/'devel/lib',Path('/usr/local/cuda/lib64')]))
    exe=build/'test_shadow';ldd=subprocess.check_output(['ldd',str(exe)],text=True,env=env)
    (out/'ldd.txt').write_text(ldd);mapped={}
    expected={'libgtsam.so.4':'369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37',
              'libgtsam_points.so.1':hashlib.sha256((build/'libgtsam_points.so.1.2.1').read_bytes()).hexdigest()}
    for name,digest in expected.items():
        path=next(line.split('=>')[1].strip().split()[0] for line in ldd.splitlines() if name+' =>' in line)
        actual=hashlib.sha256(Path(path).read_bytes()).hexdigest();assert actual==digest,(name,path,actual)
        mapped[name]={'path':path,'sha256':actual}
    command=[str(exe),'--gtest_output=xml:'+str(out/'tests.xml')]
    start=time.monotonic()
    with (out/'tests.log').open('x') as log: result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
    (out/'result.json').write_text(json.dumps(dict(returncode=result.returncode,seconds=time.monotonic()-start,
        command=command,LD_LIBRARY_PATH=env['LD_LIBRARY_PATH'],libraries=mapped,
        executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()),indent=2))
    print((out/'tests.log').read_text());raise SystemExit(result.returncode)
if __name__=='__main__':main()
