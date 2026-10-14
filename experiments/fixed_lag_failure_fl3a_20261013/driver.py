"""Create-once FL3a diagnostic. Never launches source estimators or continues FL3."""
import fcntl,hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl3a_20261013')
PREV=Path('/home/yeranis/results/gnssfgo_fixed_lag_replay_fl3_20261013')
WIRE=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def protect():
    saved=json.loads((PREV/'protected.json').read_text())
    for p,h in saved.items():assert sha(Path(p))==h,('protected change',p)
    for folder in (HERE.parent/'fixed_lag_replay_fl3_20261013',):
        for p in folder.iterdir():
            if p.is_file():saved[str(p)]=sha(p)
    for n in ('replay_timed','full_timed.json','failure_audit.json','timed_prefix_acceptance.json'):
        saved[str(PREV/n)]=sha(PREV/n)
    return saved
def main():
    action=sys.argv[1]
    if action=='prepare':
        ROOT.mkdir(exist_ok=False)
        with (ROOT/'protected.json').open('x') as f:json.dump(protect(),f,indent=2)
        return
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
        if action.startswith('build'):
            cmd=json.loads((WIRE/'build_wire3.json').read_text())['command']
            source='SnapshotAudit.cpp' if action=='build_snapshot' else 'Diagnose.cpp'
            cmd=[str(HERE/source) if x.endswith('/Replay.cpp') else x for x in cmd]
            cmd[-1]=str(ROOT/action)
        elif action=='snapshot_audit':
            assert json.loads((ROOT/'validation.json').read_text())['observer_valid']
            assert json.loads((ROOT/'build_snapshot.json').read_text())['returncode']==0
            cmd=[str(ROOT/'build_snapshot'),str(ROOT/'diagnostics'/'snapshot'),str(ROOT/'linear_audit')]
        elif action=='observe':
            build=sys.argv[2];assert json.loads((ROOT/(build+'.json')).read_text())['returncode']==0
            cmd=[str(ROOT/build),str(JOURNAL),str(ROOT/'archive'),str(ROOT/'diagnostics')]
        else:raise ValueError(action)
        os.sched_setaffinity(0,{2});start=time.monotonic()
        with (ROOT/(action+'.log')).open('x') as log:
            rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
        result=dict(command=cmd,returncode=rc,seconds=time.monotonic()-start,
          protected_unchanged=protect()==saved,max_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
          code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
        if action in ('observe','snapshot_audit'):result['binary_sha256']=sha(Path(cmd[0]))
        elif rc==0:result['binary_sha256']=sha(Path(cmd[-1]))
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(json.dumps(result,indent=2));raise SystemExit(rc)
if __name__=='__main__':main()
