from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import time

def main():
    build=Path(sys.argv[1])
    out=build/sys.argv[2]
    out.mkdir(exist_ok=False)
    env=os.environ.copy()
    env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
               ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',
               UBSAN_OPTIONS='print_stacktrace=1:halt_on_error=1')
    ws=Path('/workspace/cbs_gtsam4.3')
    private=ws/'runs/e1b4a_local_update_batching_20261012/release_2'
    old=ws/'runs/e1b_local_profile_20261012/release_2'
    env['LD_LIBRARY_PATH']=':'.join(map(str,[private,old,ws/'devel/lib',Path('/usr/local/cuda/lib64')]))
    exe=build/'test_native_gaussian'
    ldd=subprocess.check_output(['ldd',str(exe)],text=True,env=env)
    (out/'ldd.txt').write_text(ldd)
    actual=next(line.split('=>',1)[1].strip().split()[0] for line in ldd.splitlines() if 'libgtsam.so.4 =>' in line)
    digest=hashlib.sha256(Path(actual).read_bytes()).hexdigest()
    if digest!='369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37':
        raise SystemExit('wrong GTSAM library: '+actual+' '+digest)
    command=[str(exe),'--gtest_output=xml:'+str(out/'tests.xml')]
    start=time.monotonic()
    with (out/'tests.log').open('x') as stream:
        result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=env,timeout=90)
    summary={'returncode':result.returncode,'seconds':time.monotonic()-start,
             'command':command,'gtsam_sha256':digest,'LD_LIBRARY_PATH':env['LD_LIBRARY_PATH'],
             'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest()}
    (out/'result.json').write_text(json.dumps(summary,indent=2))
    print((out/'tests.log').read_text())
    print(json.dumps(summary))
    raise SystemExit(result.returncode)
if __name__=='__main__':main()
