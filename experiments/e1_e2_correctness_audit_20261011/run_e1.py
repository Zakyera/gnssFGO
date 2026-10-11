"""Run only the saved E1 fixture/libraries in the existing Noetic container."""
import json,os,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'glim_sync_compatible_batching_e1_20261011'
action=sys.argv[1];assert action in ['release','ubsan','cpu_sanitize','micro']
record=ROOT/(action+'.json');assert not record.exists()
env=os.environ.copy();env['OMP_NUM_THREADS']='1'
base=OLD/('ubsan' if action=='ubsan' else 'private')
env['LD_LIBRARY_PATH']=str(base)+':/workspace/cbs_gtsam4.3/devel/lib:'+env.get('LD_LIBRARY_PATH','')
env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
commandfile={'release':'test_attempt3','ubsan':'ubsan_attempt1','cpu_sanitize':'sanitize_attempt1','micro':'test_attempt3'}[action]
command=json.loads((base/(commandfile+'_commands.json')).read_text())[0]
binary=ROOT/(action+'_test')
command=[str(ROOT/'test_e1_audit.cpp') if arg.endswith('/cbs_local_graph_linearizer_test.cpp') else arg for arg in command]
command[command.index('-o')+1]=str(binary)
commands=[command,['taskset','-c','2',str(binary),'--gtest_output=xml:'+str(ROOT/(action+'.xml'))]]
if action=='cpu_sanitize':commands[1].append('--gtest_filter=LocalLinearizerCpu.*')
if action=='micro':commands[1]+= ['--gtest_also_run_disabled_tests','--gtest_filter=LocalLinearizerGpu.DISABLED_Timing']
start=time.monotonic();code=0
with (ROOT/(action+'.log')).open('x') as log:
    for c in commands:
        try:code=subprocess.run(c,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600).returncode
        except subprocess.TimeoutExpired:code=124
        if code:break
with record.open('x') as f:json.dump(dict(passed=code==0,returncode=code,commands=commands,
    seconds=time.monotonic()-start,environment={k:env[k] for k in ['OMP_NUM_THREADS','LD_LIBRARY_PATH','ASAN_OPTIONS','UBSAN_OPTIONS']}),f,indent=2)
print(action,code,flush=True)
if code:print((ROOT/(action+'.log')).read_text()[-5000:])
raise SystemExit(code)
