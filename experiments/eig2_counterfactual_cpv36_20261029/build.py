"""Build only the private observer; frozen original artifacts remain untouched."""
import json,os,subprocess,sys
from pathlib import Path
P=Path('/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
mode=sys.argv[1];assert mode in ('release','san','backend')
D=P/('build_'+mode+('_'+sys.argv[2] if len(sys.argv)>2 else ''));D.mkdir(exist_ok=False)
env=dict(os.environ,LD_LIBRARY_PATH=str(P/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib:'+os.environ.get('LD_LIBRARY_PATH',''),CPK9_PRIOR_REFRESH='1',FL4Q_ENABLE='0',FL4M_RETIREMENT='remove')
if mode=='backend':
    for gate in ('release','san'):assert (P/('build_'+gate+'_native2')/'passed.json').exists()
    commands=json.loads((P/'build_commands.json').read_text());cwd=commands.pop('cwd')
else:
    commands=json.loads((P/'actual_test_commands.json').read_text());cwd=None
    for k,args in commands.items():
        args=[a.replace(str(P/'test.o'),str(D/'test.o')) for a in args]
        args=[str(D/'test') if a==str(P/'test') else a for a in args]
        if mode=='san':
            args=[a for a in args if a!='-O3']
            args+=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
            args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if k=='compile' else ['-no-pie']
        commands[k]=args
    env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
def run(name,args):
    with (D/(name+'.log')).open('x') as f:r=subprocess.run(args,cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT)
    print(name,r.returncode,flush=True);assert r.returncode==0,name
(D/'commands.json').write_text(json.dumps(commands,indent=2))
for name,args in commands.items():run(name,args)
if mode!='backend':run('test',[str(D/'test'),str(D/'snapshots')])
else:run('dependencies',['ldd','-r',str(P/'private_build/libkimera_vio.so')])
(D/'passed.json').write_text(json.dumps(dict(passed=True,vendor_sanitized=False)))
