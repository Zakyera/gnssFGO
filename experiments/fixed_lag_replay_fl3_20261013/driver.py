"""Create-once FL3 gate with unchanged FL2b binary. No estimator source runs."""
import collections,fcntl,hashlib,importlib.util,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_replay_fl3_20261013')
PREV=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
ORACLE=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
ORIGIN=1727775008583475000
CUTOFF=ORIGIN+20_000_000_000
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def protected():
    old=json.loads((PREV/'protected.json').read_text())
    for s,h in old.items():assert digest(Path(s))==h,('changed frozen file',s)
    validation=json.loads((PREV/'validation.json').read_text())
    for n,h in validation['source_hashes'].items():
        p=HERE.parent/'fixed_lag_lifecycle_fl2b_20261013'/n
        assert digest(p)==h;old[str(p)]=h
    for n,h in validation['binary_hashes'].items():
        p=PREV/n;assert digest(p)==h;old[str(p)]=h
    for p in [JOURNAL,PREV/'validation.json',*ORACLE.iterdir()]:
        if p.is_file():old[str(p)]=digest(p)
    return old
def prepare():
    ROOT.mkdir(exist_ok=False)
    saved=protected()
    with (ROOT/'protected.json').open('x') as f:json.dump(saved,f,indent=2)
    counts=collections.Counter();last=None
    with JOURNAL.open('rb') as src,(ROOT/'prefix20.events').open('xb') as dst:
        for line in src:
            fields=line.split(b',',2);arrival=int(fields[1])
            if fields[0]==b'E' or arrival>CUTOFF:break
            if last is not None:assert arrival>=last
            last=arrival;counts[fields[0].decode()]+=1;dst.write(line)
        dst.write(f'E,{CUTOFF}\n'.encode())
    result=dict(origin_ns=ORIGIN,cutoff_ns=CUTOFF,last_original_arrival_ns=last,
      counts=dict(counts),prefix_sha256=digest(ROOT/'prefix20.events'),
      journal_sha256=digest(JOURNAL),protected_files=len(saved))
    with (ROOT/'prepare.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
def run(action):
    assert action in ('prefix_gate','build_observer','prefix_observe','build_timed','prefix_timed','full_timed','full_repeat'),'unknown action'
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protected()
        assert not (ROOT/(action+'_outputs')).exists()
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        cmd=[str(PREV/'build_wire3'),str(ROOT/'prefix20.events'),str(ROOT/(action+'_outputs'))]
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
        if action=='build_observer':
            cmd=['c++','-std=c++20','-O2','-fPIC','-shared','-I'+str(RT/'include'),'-I/usr/include/eigen3',
                 str(HERE/'ObserveVectorUpdate.cpp'),'-ldl','-o',str(ROOT/'observe.so')]
        if action=='prefix_observe':
            assert json.loads((ROOT/'prefix_audit.json').read_text())['status']=='PREFIX_NUMERICAL_AND_ACCOUNTING_PASS'
            assert json.loads((ROOT/'build_observer.json').read_text())['returncode']==0
            env['LD_PRELOAD']=str(ROOT/'observe.so')
        if action=='build_timed':
            assert json.loads((ROOT/'prefix_acceptance.json').read_text())['status']=='PREFIX_ACCEPTED_FOR_FULL_REPLAY'
            cmd=json.loads((PREV/'build_wire3.json').read_text())['command']
            cmd=[str(HERE/'ReplayTimed.cpp') if x.endswith('/Replay.cpp') else x for x in cmd]
            cmd[-1]=str(ROOT/'replay_timed')
        if action in ('prefix_timed','full_timed','full_repeat'):
            assert json.loads((ROOT/'prefix_acceptance.json').read_text())['status']=='PREFIX_ACCEPTED_FOR_FULL_REPLAY'
            if action!='prefix_timed':assert json.loads((ROOT/'timed_prefix_acceptance.json').read_text())['passed']
            if action=='full_repeat':assert json.loads((ROOT/'full_timed.json').read_text())['returncode']==0
            cmd=[str(ROOT/'replay_timed'),str(ROOT/'prefix20.events' if action=='prefix_timed' else JOURNAL),
                 str(ROOT/(action+'_outputs')),str(ROOT/(action+'_service.csv'))]
        os.sched_setaffinity(0,{2});start=time.monotonic();error=None
        with (ROOT/(action+'.log')).open('x') as log:
            try:rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
            except Exception as exc:rc=-1;error=repr(exc)
        seconds=time.monotonic()-start;usage=resource.getrusage(resource.RUSAGE_CHILDREN)
        result=dict(command=cmd,returncode=rc,error=error,seconds=seconds,max_rss_kib=usage.ru_maxrss,
          user_seconds=usage.ru_utime,system_seconds=usage.ru_stime,protected_unchanged=saved==protected(),
          binary_sha256=digest(PREV/'build_wire3'),prefix_sha256=digest(ROOT/'prefix20.events'),
          code={p.name:digest(p) for p in HERE.iterdir() if p.is_file()})
        if not action.startswith('build_'):result['executed_binary_sha256']=digest(Path(cmd[0]))
        elif rc==0:result['built_binary_sha256']=digest(Path(cmd[-1]))
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps(result,indent=2));raise SystemExit(rc)
if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    else:run(sys.argv[1])
