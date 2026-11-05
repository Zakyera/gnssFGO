import json, os, subprocess
from pathlib import Path
R=Path('/workspace/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
H=Path('/home/yeranis/repos/gnssFGO/experiments/kimera_failed_factor_cpk1_20261019')
# The host stages the source beside this runner; no protected file is edited.
commands=json.loads((R/'actual_test_commands.json').read_text())
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(R/'private_build')+':'+env.get('LD_LIBRARY_PATH','')
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
for mode in ('release','sanitize'):
    name='factor_probe_'+mode
    def command(args):
        mapping={str(R/'test.cpp'):str(R/'factor_probe_test.cpp'),str(R/'test.o'):str(R/(name+'.o')),str(R/'test'):str(R/name)}
        result=[mapping.get(a,a) for a in args if mode=='release' or a!='-O3']
        if mode=='sanitize':
            result+=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
            result+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if '-c' in args else ['-no-pie']
        return result
    for step,args in [('compile',command(commands['compile'])),('link',command(commands['link'])),('run',[str(R/name),str(R/name)])]:
        with (R/(name+'_'+step+'.log')).open('x') as log:
            p=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT)
        print(name,step,p.returncode,flush=True)
        if p.returncode:raise SystemExit(p.returncode)
with (R/'localization_tests_passed.json').open('x') as f:json.dump({'passed':True,'known_good_and_bad_jacobian':True,'input_nonmutation':True,'release':True,'asan_ubsan':True,'linked_libraries_instrumented':False,'leaks_checked':False},f)
