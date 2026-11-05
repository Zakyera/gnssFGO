"""Run inside existing Noetic container. Compiles ONLY CP-K6 test.cpp."""
import json
import os
from pathlib import Path
import subprocess
import sys

R = Path(__file__).resolve().parent
W = Path('/workspace/cbs_gtsam4.3')
mode = sys.argv[1]
assert mode in ('release', 'san')
out = R / (mode + '_' + sys.argv[2])
out.mkdir(exist_ok=False)
flags = ['-std=gnu++17', '-DNDEBUG', '-DBOOST_ALL_NO_LIB', '-DBOOST_SERIALIZATION_DYN_LINK',
         '-DTBB_USE_THREADING_TOOLS', '-I'+str(R), '-I'+str(W/'src/cbs/include'),
         '-isystem',str(W/'devel/include'),
         '-isystem',str(W/'devel/include/gtsam/3rdparty/Eigen'),
         '-isystem','/usr/include/eigen3']
extra = []
if mode == 'san':
    flags += ['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all',
              '-fno-omit-frame-pointer','-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    extra = ['-fsanitize=address,undefined','-fno-sanitize-recover=all','-no-pie']
else:
    flags += ['-O3']
compile_cmd = ['/usr/bin/c++'] + flags + ['-c',str(R/'test.cpp'),'-o',str(out/'test.o')]
link_cmd = ['/usr/bin/c++',str(out/'test.o'),'-o',str(out/'test')]+extra+[
    '-Wl,-rpath,'+str(W/'devel/lib'),str(W/'devel/lib/libgtsam.so.4.3a1'),
    '/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.71.0',
    '/usr/lib/x86_64-linux-gnu/libtbb.so.2', '-lpthread']
commands = {'compile':compile_cmd,'link':link_cmd}
(out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
env = dict(os.environ)
env['LD_LIBRARY_PATH'] = str(W/'devel/lib')
if mode == 'san':
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
for name,cmd in commands.items():
    with (out/(name+'.log')).open('x') as f:
        result = subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
    print(name,result.returncode,flush=True)
    if result.returncode:
        sys.exit(result.returncode)
with (out/'dependencies.txt').open('x') as f:
    subprocess.run(['ldd',str(out/'test')],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
for i in range(2 if mode=='release' else 1):
    with (out/('run%d.log'%i)).open('x') as f:
        result = subprocess.run([str(out/'test')],env=env,stdout=f,stderr=subprocess.STDOUT)
    print('run',i,result.returncode,flush=True)
    if result.returncode:
        sys.exit(result.returncode)
if mode=='release':
    assert (out/'run0.log').read_bytes()==(out/'run1.log').read_bytes(), 'Repeat output mismatch'
(out/'passed.json').write_text(json.dumps({'mode':mode,'passed':True,'repeat_byte_identical':mode=='release',
    'vendor_libraries_instrumented':False},indent=2)+'\n')
