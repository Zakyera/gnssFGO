"""Exclusive private build/test; invoked inside build container only."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

P=Path(__file__).resolve().parent
mode,attempt=sys.argv[1:]
assert mode in ('release','san','backend')
out=P/(mode+'_'+attempt);out.mkdir(exist_ok=False)
env=dict(os.environ,LD_LIBRARY_PATH=str(P/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib',
         FL4Q_ENABLE='0',FL4M_RETIREMENT='remove',CPK9_PRIOR_REFRESH='1',CPV38_OFF_REFRESH='0',
         ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
def write(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2)
write(out/'start.json',{'pid':os.getpid(),'mode':mode,'start':time.time()})
def run(name,args,cwd=None):
    start=time.time()
    with (out/(name+'.log')).open('x') as f:
        process=subprocess.Popen(args,env=env,cwd=cwd,stdout=f,stderr=subprocess.STDOUT)
        write(out/(name+'.process.json'),{'pid':process.pid,'args':args,'start':start})
        rc=process.wait()
    write(out/(name+'.receipt.json'),{'exit':rc,'seconds':time.time()-start})
    print(name,rc,flush=True)
    if rc:raise SystemExit(rc)
if mode=='backend':
    assert list(P.glob('release_*/passed.json')) and list(P.glob('san_*/passed.json'))
    commands=json.loads((P/'build_commands.json').read_text());cwd=commands.pop('cwd')
    write(out/'commands.json',commands)
    for name,args in commands.items():run(name,args,cwd)
    run('dependencies',['ldd','-r',str(P/'private_build/libkimera_vio.so')])
    deps=(out/'dependencies.log').read_text()
    assert 'not found' not in deps and 'undefined symbol' not in deps
else:
    commands=json.loads((P/'actual_test_commands.json').read_text())
    def compile_test(label,source,original=False):
        result={}
        for kind,args in commands.items():
            args=[a.replace(str(P/'test.o'),str(out/(label+'.o'))) for a in args]
            args=[str(out/label) if a==str(P/'test') else a for a in args]
            args=[str(source) if a==str(P/'test.cpp') else a for a in args]
            if original:
                args=[a.replace('-I'+str(P),'-I/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019') for a in args]
            if mode=='san':
                args=[a for a in args if a!='-O3']+['-O1','-g1','-fsanitize=address,undefined',
                    '-fno-sanitize-recover=all','-fno-omit-frame-pointer']
                args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie']
            result[kind]=args
        write(out/(label+'_commands.json'),result)
        for kind,args in result.items():run(label+'_'+kind,args)
    if mode=='release' and attempt=='initial':
        compile_test('baseline',P/'baseline.cpp',True)
        compile_test('disabled',P/'baseline.cpp')
        run('baseline',[str(out/'baseline')]);run('disabled',[str(out/'disabled')])
        assert (out/'baseline.log').read_bytes()==(out/'disabled.log').read_bytes(), 'disabled parity'
    elif mode=='release':
        previous=P/'release_initial'
        assert (previous/'baseline.log').read_bytes()==(previous/'disabled.log').read_bytes()
        for label in ('baseline','disabled'):
            assert json.loads((previous/(label+'.receipt.json')).read_text())['exit']==0
        write(out/'reused_disabled_parity.json',{'source':str(previous),'unchanged_candidate_headers':True})
    compile_test('test',P/'test.cpp')
    run('dependencies',['ldd',str(out/'test')])
    deps=(out/'dependencies.log').read_text()
    assert str(P/'private_build/libcbs.so') in deps and 'not found' not in deps
    with (out/'CTestTestfile.cmake').open('x') as f:
        f.write('add_test(cpv38_off_refresh "'+str(out/'test')+'")\n')
    for i in range(2 if mode=='release' else 1):
        run('ctest'+str(i),['ctest','-V','--output-on-failure'],out)
    if mode=='release':
        def records(i):
            return [s for s in (out/('ctest'+str(i)+'.log')).read_text().splitlines()
                    if s.startswith('1: ') and ('=' in s or 'PASS CP-V38' in s)]
        assert records(0) and records(0)==records(1), 'nondeterministic repeat'
write(out/'passed.json',{'passed':True,'mode':mode,'vendor_instrumented':False,'finished':time.time()})
