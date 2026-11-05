"""Exclusive focused gates, then optional private VioBackend/DSO build only."""
import json
import os
from pathlib import Path
import subprocess
import sys
R=Path(__file__).resolve().parent
mode=sys.argv[1]
assert mode in ('release','san','backend')
out=R/(mode+'_'+sys.argv[2]);out.mkdir(exist_ok=False)
env=dict(os.environ)
env['LD_LIBRARY_PATH']=str(R/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib:'+env.get('LD_LIBRARY_PATH','')
env.update(FL4Q_ENABLE='0',FL4M_RETIREMENT='remove',CPK9_PRIOR_REFRESH='1')
if mode=='backend':
    assert list(R.glob('release_*/passed.json')) and list(R.glob('san_*/passed.json')), 'focused gates required'
    commands=json.loads((R/'build_commands.json').read_text());cwd=commands.pop('cwd')
else:
    commands=json.loads((R/'actual_test_commands.json').read_text());cwd=None
    for k,args in commands.items():
        args=[a.replace(str(R/'test.o'),str(out/'test.o')) for a in args]
        args=[str(out/'test') if a==str(R/'test') else a for a in args]
        if mode=='san':
            args=[a for a in args if a!='-O3']
            args+=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
            args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if k=='compile' else ['-no-pie']
        commands[k]=args
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
(out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
def run(name,args):
    with (out/(name+'.log')).open('x') as f:
        rc=subprocess.run(args,env=env,cwd=cwd,stdout=f,stderr=subprocess.STDOUT).returncode
    print(name,rc,flush=True)
    if rc:raise SystemExit(rc)
for name,args in commands.items():run(name,args)
if mode!='backend':
    run('dependencies',['ldd',str(out/'test')])
    for i in range(2 if mode=='release' else 1):run('run'+str(i),[str(out/'test')])
    if mode=='release':assert (out/'run0.log').read_bytes()==(out/'run1.log').read_bytes()
(out/'passed.json').write_text(json.dumps({'passed':True,'mode':mode,'vendor_instrumented':False},indent=2)+'\n')
