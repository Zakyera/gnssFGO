from pathlib import Path
import json
import os
import subprocess

root=Path(__file__).resolve().parent
ws=root.parents[1]
out=root/'unit_glim_availability';out.mkdir(exist_ok=False)
env=os.environ.copy()
env['LD_LIBRARY_PATH']=str(ws/'devel')+':'+str(ws/'devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
base=['g++','-std=c++17','-g','-I'+str(ws/'src/glim_ros1/include'),
      '-I'+str(ws/'src/glim/include'),'-I'+str(ws/'devel/include'),
      '-I'+str(ws/'devel/include/gtsam/3rdparty/Eigen'),
      str(ws/'src/glim_ros1/test/cbs_native_imu_mean_bridge_test.cpp'),
      '-L'+str(ws/'devel'),'-L'+str(ws/'devel/lib'),
      '-lglim','-lgtsam','-lgtest','-lgtest_main','-lpthread']
result=[]
for label,flags in [('release',['-O2']),('asan_ubsan',['-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
    binary=out/label;command=base+flags+['-o',str(binary)]
    with (out/(label+'_build.log')).open('x') as f:
        subprocess.run(command,env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    env['ASAN_OPTIONS']='detect_leaks=0';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    with (out/(label+'_tests.log')).open('x') as f:
        subprocess.run([str(binary)],env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    result.append(dict(label=label,passed=True,command=command))
with (out/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print('Native GLIM bridge tests passed; linked production libraries not sanitizer-instrumented')
