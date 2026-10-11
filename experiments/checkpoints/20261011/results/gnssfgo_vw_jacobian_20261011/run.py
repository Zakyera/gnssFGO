"""Private candidate only; one frozen-journal replay, never ROS/source runs."""
import json, os, subprocess, sys, time
from pathlib import Path
root=Path('/audit'); action=sys.argv[1]
assert action in ('unit','sanitizer','build','replay')
record=root/f'command_{action}.json'
assert not record.exists(), 'Refuse duplicate action'
includes=['-I/audit/build','-I/prior/build','-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
includes+=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
flags=['c++','-std=c++20','-O3','-DNDEBUG','-fopenmp','-fPIC',*includes]
link=['-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib']
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',LD_LIBRARY_PATH='/runtime/lib')
if action in ('unit','sanitizer'):
    if action=='sanitizer':
        flags=['c++','-std=c++20','-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined',*includes]
    binary=str(root/'build'/f'tests_{action}')
    commands=[flags+['/audit/build/test_twist.cpp','/audit/build/test_cache.cpp','/prior/build/test_refinement.cpp',
      '/source/online_fgo/test/test_transactional_isam2_rebuild.cpp','/source/online_fgo/test/test_stationary_qr_consensus.cpp',
      '/source/online_fgo/test/test_bounded_isam2_refinement.cpp',*link,'-lgtest_main','-lgtest','-lpthread','-o',binary],
      [binary]]
    env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
elif action=='build':
    assert json.loads((root/'command_unit.json').read_text())['passed']
    commands=[flags+['/audit/build/DiagnosticReplay.cpp',*link,'-o','/audit/build/replay']]
else:
    for gate in ('unit','sanitizer','build'):
        assert json.loads((root/f'command_{gate}.json').read_text())['passed'],gate
    assert not (root/'candidate').exists(), 'Refuse replay overwrite'
    env.update(WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1')
    commands=[['/audit/build/replay','--receiver-model','source_wnoa','--mode','sources','--timeline','record',
      '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
      '--receiver-startup-samples','59','--event-stream','/captured/events.csv','--out','/audit/candidate']]
results=[]
with (root/'logs'/f'{action}.log').open('x') as log:
    for command in commands:
        start=time.monotonic()
        try:
            p=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
            code=p.returncode
        except subprocess.TimeoutExpired:code=124
        results.append(dict(command=command,returncode=code,seconds=time.monotonic()-start))
        if code:break
payload=dict(action=action,passed=all(x['returncode']==0 for x in results),commands=results,
    environment={k:v for k,v in env.items() if k.startswith(('WNOJ_','CACHE_','OMP_','LD_','ASAN_','UBSAN_'))})
with record.open('x') as f:json.dump(payload,f,indent=2)
print(json.dumps([{k:v for k,v in x.items() if k!='command'} for x in results]),flush=True)
if code:print((root/'logs'/f'{action}.log').read_text()[-6000:])
raise SystemExit(code)
