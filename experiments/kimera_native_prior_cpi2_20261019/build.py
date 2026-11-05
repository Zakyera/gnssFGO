"""Compile isolated synthetic gate with the frozen FL4q recipe."""
from pathlib import Path
import json,os,subprocess,time,sys
ROOT=Path(__file__).resolve().parent
BASE=ROOT.parent/'fixed_lag_current_chart_fl4q_20261016'
recipe=json.loads((BASE/'actual_test_commands.json').read_text())
san='--sanitize' in sys.argv
tag='san' if san else 'release'
env=os.environ.copy();env['OMP_NUM_THREADS']='1'
env['LD_LIBRARY_PATH']='/workspace/cbs_gtsam4.3/devel/lib:/opt/ros/noetic/lib'
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
records=[]
for name in ['compile','link','test','repeat']:
    if name in recipe:
        cmd=[x.replace(str(BASE/'test.cpp'),str(ROOT/'test.cpp')).replace(str(BASE/'test.o'),str(ROOT/(tag+'.o'))).replace(str(BASE/'test'),str(ROOT/tag)) for x in recipe[name]]
        if name=='compile':cmd.insert(1,'-I'+str(ROOT/'include'))
        if san:
            cmd=[x for x in cmd if x!='-O3']
            cmd+=['-O0','-g1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
        if name=='link':cmd+=['-ldl']
    else:cmd=[str(ROOT/tag)]
    start=time.monotonic()
    with (ROOT/(tag+'_'+name+'.log')).open('x') as f:
        r=subprocess.run(cmd,env=env,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
    records.append(dict(name=name,command=cmd,returncode=r.returncode,seconds=time.monotonic()-start))
    (ROOT/(tag+'_commands.json')).write_text(json.dumps(records,indent=2)+'\n')
    print(name,r.returncode,flush=True)
    if r.returncode:raise SystemExit(r.returncode)
