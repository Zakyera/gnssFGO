"""Scoped isolated build/replay harness; never starts sources or ROS playback."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path('/audit'); action=sys.argv[1]
label=sys.argv[2] if len(sys.argv)>2 else action
assert label.replace('_','').isalnum()
assert action in ('unit','sanitizer','build','baseline','cached')
record=ROOT/f'command_{label}.json'
assert not record.exists(), f'Refusing duplicate action: {action}'
assert not (ROOT/'logs'/f'{label}.log').exists(), 'Refusing existing log'
includes=['-I/audit/build','-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
includes+=['-I'+str(p) for p in Path('/opt/ros/humble/include').iterdir() if p.is_dir()]
flags=['c++','-std=c++20','-O3','-DNDEBUG','-fopenmp','-fPIC',*includes]
link=['-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib']
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',LD_LIBRARY_PATH='/runtime/lib')
commands=[]
if action in ('unit','sanitizer'):
    if action=='sanitizer':
        flags=['c++','-std=c++20','-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined',*includes]
    binary=str(ROOT/'build'/('tests_'+action))
    commands.append(flags+['/audit/build/test_cache.cpp','/audit/build/test_refinement.cpp',
        '/source/online_fgo/test/test_transactional_isam2_rebuild.cpp',
        '/source/online_fgo/test/test_stationary_qr_consensus.cpp',
        '/source/online_fgo/test/test_bounded_isam2_refinement.cpp',*link,'-lgtest_main','-lgtest','-lpthread','-o',binary])
    commands.append([binary]);env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
elif action=='build':
    commands.append(flags+['/audit/build/DiagnosticReplay.cpp',*link,'-o','/audit/build/replay'])
else:
    assert not (ROOT/action).exists()
    executable='/frozen/build/wnoa/replay' if action=='baseline' else '/audit/build/replay'
    env.update(WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1')
    if action=='cached':env['CACHE_FAILURE_PROBE_DIR']='/audit/cached/failure_probe'
    commands.append([executable,'--receiver-model','source_wnoa','--mode','sources','--timeline','record',
        '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
        '--receiver-startup-samples','59','--event-stream','/captured/events.csv','--out',str(ROOT/action)])
results=[]
with (ROOT/'logs'/f'{label}.log').open('x') as log:
    for command in commands:
        start=time.monotonic();p=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        results.append(dict(command=command,returncode=p.returncode,seconds=time.monotonic()-start))
        if p.returncode:break
with record.open('x') as f:json.dump(dict(action=action,commands=results,environment={k:v for k,v in env.items() if k.startswith(('WNOJ_','CACHE_','OMP_','LD_','ASAN_','UBSAN_'))}),f,indent=2)
print(json.dumps([dict(executable=r['command'][0],returncode=r['returncode'],seconds=r['seconds']) for r in results]),flush=True)
if action in ('baseline','cached'):
    rows=list(csv.DictReader((ROOT/action/'publication_convergence.csv').open()))
    assert p.returncode!=0 and rows[-1]['index']=='1272' and rows[-1]['status']=='budget_exhausted', 'Unexpected replay result; stop for inspection'
    print('Expected strict-publication stop preserved at support1272. This is not a completed trajectory.')
else:
    if p.returncode:print((ROOT/'logs'/f'{label}.log').read_text()[-5000:])
    raise SystemExit(p.returncode)
