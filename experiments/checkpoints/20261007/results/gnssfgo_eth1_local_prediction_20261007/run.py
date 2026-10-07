from pathlib import Path
import json,os,subprocess,sys,time,resource
ROOT=Path('/audit');action=sys.argv[1];model=sys.argv[2]if len(sys.argv)>2 else 'wnoa'
assert model in ('wnoj','wnoa')
def run(label,command,env=None):
 base=label;attempt=1
 while (ROOT/f'command_{label}.json').exists():
  attempt+=1;label=base+'_attempt'+str(attempt)
 start=time.monotonic()
 with (ROOT/'logs'/f'{label}.log').open('x')as f:
  p=subprocess.run(command,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=3600)
 record=dict(name=label,command=command,returncode=p.returncode,seconds=time.monotonic()-start,maximum_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,environment={k:v for k,v in (env or {}).items()if k.startswith(('WNOJ_','AUDIT_','COST_','ASAN_','UBSAN_'))})
 (ROOT/f'command_{label}.json').write_text(json.dumps(record,indent=2)+'\n')
 print(json.dumps(record),flush=True)
 if p.returncode:print((ROOT/'logs'/f'{label}.log').read_text()[-3000:],flush=True)
 raise SystemExit(p.returncode)
build=ROOT/'build'/model
if action in ('build616','diagnostic616'):build=ROOT/'build/diagnostic616'
includes=[f'-I{build}','-I/runtime/include','-I/source/online_fgo','-I/source/online_fgo/include','-I/usr/include/eigen3']
includes+=['-I'+str(p)for p in Path('/opt/ros/humble/include').iterdir()if p.is_dir()]
flags=['c++','-std=c++20','-O3','-DNDEBUG','-fopenmp','-fPIC',*includes]
link=['-L/runtime/lib','-lonline_fgo_solver','-lonline_fgo_util','/runtime/lib/libgtsam.so.4','-ltbb','-Wl,-rpath,/runtime/lib']
if action=='gradient1174_build':run('gradient1174_build',flags+[str(build/'SavedGradient1174.cpp'),*link,'-o',str(build/'gradient1174')])
elif action=='gradient1174':run('gradient1174',[str(build/'gradient1174')],os.environ.copy())
elif action=='saved1174_build':run('saved1174_build',flags+[str(build/'SavedGraph1174.cpp'),*link,'-o',str(build/'saved1174')])
elif action=='saved1174':run('saved1174',[str(build/'saved1174')],os.environ.copy())
elif action=='saved_build':run('saved_build',flags+[str(build/'SavedGraph.cpp'),*link,'-o',str(build/'saved_graph')])
elif action=='saved':run('saved',[str(build/'saved_graph')],os.environ.copy())
elif action in ('build','build2'):run('diagnostic_'+action+'_'+model,flags+[str(build/'DiagnosticReplay.cpp'),*link,'-o',str(build/'diagnostic')])
elif action=='diagnostic':
 env=os.environ.copy();env.update(WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1',COST_AUDIT_DIR='/audit/failed_step')
 run('diagnostic_'+model,[str(build/'diagnostic'),'--receiver-model','source_'+model,'--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59','--event-stream','/long/inputs/off_events.csv','--out','/audit/diagnostic_off_'+model],env)
elif action=='build616':run('build616',flags+[str(build/'DiagnosticReplay.cpp'),*link,'-o',str(build/'diagnostic')])
elif action=='diagnostic616':
 env=os.environ.copy();env.update(WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1',COST_AUDIT_DIR='/audit/failed_step_616')
 run('diagnostic616',[str(build/'diagnostic'),'--receiver-model','source_wnoa','--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59','--event-stream','/long/inputs/off_events.csv','--out','/audit/diagnostic616'],env)
elif action in ('stable_build','stable_build2'):run(action+'_'+model,flags+[str(build/'DiagnosticReplay.cpp'),*link,'-o',str(build/'replay')])
elif action in ('short','short_repeat','primary','repeat'):
 mode=sys.argv[3];assert mode in ('off','on')
 label=mode+'_'+model+('_short_repeat'if action=='short_repeat'else '_short'if action=='short'else '_repeat'if action=='repeat'else '')
 env=os.environ.copy();env.update(WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1')
 # Repeat uses the identical instrumentation and audit checkpoints as primary.
 env.update(AUDIT_RECORD_ALL='1',WNOJ_AUDIT_INDICES='74,194,344,394,594,794,964,994,1192')
 inp='/inputs40/inputs/'if action in ('short','short_repeat')else '/long/inputs/'
 run(label,[str(build/'replay'),'--receiver-model','source_'+model,'--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59','--event-stream',inp+mode+'_events.csv','--out',str(ROOT/label)],env)
elif action in ('unit','unit2','sanitizer','sanitizer2'):
 if action.startswith('sanitizer'):flags=['c++','-std=c++20','-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined',*includes]
 binary=str(ROOT/'build'/('test_stable_'+action))
 sources=['/source/online_fgo/test/test_stationary_qr_consensus.cpp','/source/online_fgo/test/test_transactional_isam2_rebuild.cpp','/source/online_fgo/test/test_stable_gaussian_cost.cpp','/source/online_fgo/test/test_bounded_isam2_refinement.cpp']
 start=time.monotonic()
 buildcmd=flags+sources+[*link,'-lgtest_main','-lgtest','-lpthread','-o',binary]
 with (ROOT/'logs'/f'{action}_build.log').open('x')as f:p=subprocess.run(buildcmd,stdout=f,stderr=subprocess.STDOUT,timeout=600)
 (ROOT/f'command_{action}_build.json').write_text(json.dumps(dict(command=buildcmd,returncode=p.returncode,seconds=time.monotonic()-start),indent=2)+'\n')
 if p.returncode:print((ROOT/'logs'/f'{action}_build.log').read_text()[-4000:]);raise SystemExit(p.returncode)
 env=os.environ.copy();env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
 run(action,[binary],env)
else:raise SystemExit('unknown action')
