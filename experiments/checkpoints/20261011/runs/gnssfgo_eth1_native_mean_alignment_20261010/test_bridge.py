"""Focused native bridge validation using existing Noetic/GTSAM runtime."""
from pathlib import Path
import hashlib
import json
import os
import subprocess

root = Path(__file__).resolve().parent
ws = root.parents[1]
src = ws/'src/Kimera-VIO'
out = root/'unit_v2'
out.mkdir(exist_ok=False)
env = os.environ.copy()
env['LD_LIBRARY_PATH'] = str(ws/'build/kimera_vio')+':'+str(ws/'devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
common = ['g++','-std=c++17','-g','-I'+str(src/'include'),
          '-I'+str(ws/'devel/include'),'-I'+str(ws/'devel/include/gtsam/3rdparty/Eigen'),'-I/usr/include/opencv4',
          str(src/'tests/testCbsNativeImuMeanBridge.cpp'),
          '-L'+str(ws/'build/kimera_vio'),'-L'+str(ws/'devel/lib'),
          '-lkimera_vio','-lgtsam','-lglog','-lgtest','-lgtest_main','-lpthread']
results = []
for label, flags in [('release',['-O2']),('asan_ubsan',['-O1','-fno-omit-frame-pointer','-fsanitize=address,undefined'])]:
    binary = out/('test_bridge_'+label)
    assert not binary.exists()
    command = common + flags + ['-o',str(binary)]
    with (out/(label+'_build.log')).open('x') as log:
        subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    env['ASAN_OPTIONS'] = 'detect_leaks=0'
    env['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
    with (out/(label+'_tests.log')).open('x') as log:
        subprocess.run([str(binary)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    results.append(dict(label=label,command=command,sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),passed=True))
with (out/'unit_validation.json').open('x') as f:
    json.dump(dict(results=results,scope='Focused helper instrumented; linked production libraries are not sanitizer-instrumented.'),f,indent=2)
print('Release and focused ASan/UBSan passed')
