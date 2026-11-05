import json,os,subprocess,sys
from pathlib import Path
P=Path('/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029');D=P/('build_offline_'+sys.argv[1]);D.mkdir(exist_ok=False)
commands=json.loads((P/'actual_test_commands.json').read_text())
for name,args in commands.items():
    args=[a.replace(str(P/'observer_test.cpp'),str(P/'Counterfactual.cpp')).replace(str(P/'test.o'),str(D/'counter.o')) for a in args]
    args=[str(D/'counter') if a==str(P/'test') else a for a in args]
    if len(sys.argv)>2 and sys.argv[2]=='san':
        args=[a for a in args if a!='-O3']
        args+=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
        args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if name=='compile' else ['-no-pie']
    commands[name]=args
(D/'commands.json').write_text(json.dumps(commands,indent=2))
for name,args in commands.items():
    with (D/(name+'.log')).open('x') as f:r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT)
    print(name,r.returncode,flush=True);assert r.returncode==0
env=dict(os.environ,LD_LIBRARY_PATH=str(P/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib:'+os.environ.get('LD_LIBRARY_PATH',''))
env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
with (D/'test.log').open('x') as f:r=subprocess.run([str(D/'counter'),str(P/'build_release_native2/snapshots/update_11/checkpoint.bin'),str(D/'fixture')],env=env,stdout=f,stderr=subprocess.STDOUT)
assert r.returncode==0
(D/'passed.json').write_text(json.dumps(dict(passed=True)))
