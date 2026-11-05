import json,os,subprocess,sys
from pathlib import Path
R=Path(__file__).resolve().parent
old=Path('/workspace/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
commands=json.loads((R/'original_commands.json').read_text())
mode=sys.argv[1] if len(sys.argv)>1 else 'release'
assert mode.replace('_','').isalnum()
san=mode.startswith('san')
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(old/'private_build')+':'+env.get('LD_LIBRARY_PATH','')
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
flags=[] if not san else ['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
mapping={str(old/'test.cpp'):str(R/'test.cpp'),str(old/'test.o'):str(R/(mode+'.o')),str(old/'test'):str(R/mode)}
actual={}
for name in ('compile','link'):
    args=[mapping.get(a,a) for a in commands[name] if not san or a!='-O3']
    if name=='compile':args.insert(1,'-I'+str(R))
    args+=flags
    if san:args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if name=='compile' else ['-no-pie']
    actual[name]=args
    with (R/(mode+'_'+name+'.log')).open('x') as log:p=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(name,p.returncode,flush=True)
    if p.returncode:raise SystemExit(p.returncode)
with (R/(mode+'_commands.json')).open('x') as f:json.dump(actual,f,indent=2)
cmd=[str(R/mode),str(old/'off_capture/factor_derivatives.txt.snapshot'),str(R/(mode+'_factors.csv'))]
if mode.endswith('fixtures'):cmd=[str(R/mode)]
with (R/(mode+'_run.log')).open('x') as log:p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
print('run',p.returncode,flush=True)
with (R/(mode+'_status.json')).open('x') as f:json.dump({'passed':p.returncode==0,'returncode':p.returncode,'source_replay':False,'linked_libraries_instrumented':False},f,indent=2)
raise SystemExit(p.returncode)
