"""Private CP73 readiness + inherited compatibility; no baseline writes."""
import hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
P=Path(__file__).resolve().parent
K=P.parent/'kimera_on_incremental_refresh_cpv40_20261029'
mode,attempt=sys.argv[1:];assert mode in ('release','san','backend')
D=P/(mode+'_'+attempt);D.mkdir(exist_ok=False)
env=dict(os.environ,LD_LIBRARY_PATH=str(P/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib',
 ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
def save(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2)
def sha(p):
 with p.open('rb') as f:return hashlib.sha256(f.read()).hexdigest()
sources=[*P.glob('*.h'),*P.glob('*.inc'),*P.glob('*.cpp'),*P.glob('include/**/*.h')]
pins={str(p):sha(p) for p in sources};save(D/'sources.json',pins)
for p in sources:
 q=D/'source_snapshot'/p.relative_to(P);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
save(D/'start.json',dict(pid=os.getpid(),time=time.time(),mode=mode))
def run(name,args,cwd=None):
 start=time.time()
 with (D/(name+'.log')).open('x') as f:
  job=subprocess.Popen(args,cwd=cwd,env=env,stdout=f,stderr=subprocess.STDOUT)
  save(D/(name+'_process.json'),dict(pid=job.pid,args=args));rc=job.wait()
 save(D/(name+'_receipt.json'),dict(exit=rc,seconds=time.time()-start))
 print(name,rc,flush=True);assert rc==0,name
def compile(label,source,old=False):
 commands=json.loads((K/'actual_test_commands.json').read_text())
 for kind,args in commands.items():
  args=[a.replace(str(K),str(P)) for a in args]
  args=[str(source) if a==str(P/'test.cpp') else a for a in args]
  args=[a.replace(str(P/'test.o'),str(D/(label+'.o'))) for a in args]
  args=[str(D/label) if a==str(P/'test') else a for a in args]
  if old:args=[a.replace('-I'+str(P),'-I'+str(P.parent/'eig2_retirement_capture_cpv66_20261031')) for a in args]
  if mode=='san':
   args=[a for a in args if a!='-O3']+['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
   args+=['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1'] if kind=='compile' else ['-no-pie']
  save(D/(label+'_'+kind+'_command.json'),args);run(label+'_'+kind,args)
if mode=='backend':
 for gate in ('release','san'):
  receipts=list(P.glob(gate+'_*/passed.json'));assert receipts
  assert json.loads((receipts[-1].parent/'sources.json').read_text())==pins
 commands=json.loads((K/'build_commands.json').read_text().replace(str(K),str(P)))
 cwd=commands.pop('cwd');save(D/'commands.json',commands)
 for name,args in commands.items():run(name,args,cwd)
 run('dependencies',['ldd','-r',str(P/'private_build/libkimera_vio.so')])
 deps=(D/'dependencies.log').read_text();assert 'not found' not in deps and 'undefined symbol' not in deps
else:
 compile('compatibility',P/'persistent_test.cpp')
 run('compatibility',[str(D/'compatibility'),str(D/'snapshots')])
 if mode=='release':
  compile('old',P/'disabled_parity.cpp',True);compile('new',P/'disabled_parity.cpp')
  for name in ('old','new'):run(name,[str(D/name)])
  assert (D/'old.log').read_bytes()==(D/'new.log').read_bytes(),'disabled observer changed output'
  save(D/'disabled_parity.json',dict(exact=True,states=True,cache=True,tree=True))
assert pins=={str(p):sha(p) for p in sources}
save(D/'passed.json',dict(passed=True,mode=mode,finished=time.time(),vendor_instrumented=False))
