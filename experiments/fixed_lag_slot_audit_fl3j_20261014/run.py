"""One prefix reconstruction and same-input diagnostic. No source/live run."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'fixed_lag_slots_fl3i_20261014'
sys.path.insert(0,str(BASE))
import driver
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_slot_audit_fl3j_20261014')
def protected():
    old=json.loads((driver.ROOT/'protected.json').read_text())
    assert old==driver.protected()
    for path in [*BASE.iterdir(),*driver.ROOT.rglob('*')]:
        if path.is_file() and '__pycache__' not in path.parts:old[str(path)]=driver.sha(path)
    return old
action=sys.argv[1]
if action=='prepare':
    saved=protected();ROOT.mkdir(exist_ok=False)
    with (ROOT/'protected.json').open('x') as f:json.dump(saved,f,indent=2)
    print('protected',len(saved));sys.exit(0)
saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
if action=='verify':
    result=json.loads((ROOT/'run.json').read_text());assert result['returncode']==0
    count=0
    for p in (ROOT/'prefix').rglob('*'):
        if p.is_file():
            q=driver.ROOT/'full_outputs'/p.relative_to(ROOT/'prefix')
            assert q.is_file() and driver.sha(p)==driver.sha(q),str(p)
            count+=1
    assert json.loads((ROOT/'comparison.json').read_text())['passed']
    result=dict(passed=True,prefix_files_byte_identical=count,protected_files_unchanged=len(saved),
                binary_sha256=driver.sha(ROOT/'probe'))
    with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(result);sys.exit(0)
with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
if action=='build':
    cmd=json.loads((driver.ROOT/'build_replay.json').read_text())['command']
    cmd=[x for x in cmd if not x.endswith('.cpp')]
    cmd[6:6]=['-I'+str(HERE),str(HERE/'Probe.cpp'),str(HERE/'ObservedSmoother.cpp'),str(BASE/'SlotISAM2.cpp')]
    cmd[-1]=str(ROOT/'probe')
elif action=='run':
    assert json.loads((ROOT/'build.json').read_text())['returncode']==0
    cmd=[str(ROOT/'probe'),str(driver.JOURNAL),str(ROOT/'prefix')]
else:raise ValueError(action)
env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
env.update(LD_LIBRARY_PATH=str(driver.RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0',FL3J_ROOT=str(ROOT),PYTHONDONTWRITEBYTECODE='1')
os.sched_setaffinity(0,{2});start=time.monotonic()
with (ROOT/(action+'.log')).open('x') as f:
    rc=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=240).returncode
result=dict(returncode=rc,seconds=time.monotonic()-start,command=cmd,protected_unchanged=saved==protected(),
            sources={str(p):driver.sha(p) for p in HERE.iterdir() if p.is_file()})
with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
print(action,rc,result['seconds']);sys.exit(rc)
