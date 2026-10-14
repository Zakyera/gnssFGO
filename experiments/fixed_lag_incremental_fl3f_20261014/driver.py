"""One bounded-incremental candidate and exact reference reconstruction; no retries."""
import fcntl,hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_incremental_fl3f_20261014')
OLD=Path('/home/yeranis/results/gnssfgo_fixed_lag_complete_update_fl3e_20261013')
REFERENCE=OLD
WIRE=Path('/home/yeranis/results/gnssfgo_fixed_lag_lifecycle_fl2b_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def check_patch():
    original=(HERE.parent/'fixed_lag_complete_update_fl3e_20261013/Receiver.h').read_text()
    replacements=[
      ('publicationBudget=s.newest==1224?200:20; // FL3e target only.',
       'publicationBudget=s.newest==1224?4:20; // FL3f target only.'),
      ('"pre-retirement stationarity"','"pre-retirement numerical validity"'),
      ('"post-retirement stationarity"','"post-retirement numerical validity"'),
      ('rowState(out,s,value,s.newest,s.lastArrival,"online");',
       'rowState(out,s,value,s.newest,s.lastArrival,s.newest==1224?"incremental_candidate":"online");'),
      ('rows<<"publication,"','rows<<(s.newest==1224?"publication_candidate,":"publication,")')]
    for a,b in replacements:
        assert original.count(a)==1,(a,original.count(a))
        original=original.replace(a,b)
    assert original==(HERE/'Receiver.h').read_text(),'private receiver differs beyond declared policy patch'
def protect():
    check_patch()
    saved=json.loads((OLD/'protected.json').read_text())
    for p,h in saved.items():assert sha(Path(p))==h,('protected change',p)
    for folder in (HERE.parent/'fixed_lag_complete_update_fl3e_20261013',OLD/'diagnostics'):
        for p in folder.rglob('*'):
            if p.is_file():saved[str(p)]=sha(p)
    assert sha(JOURNAL)=='da7f4f8e339f469ff05c5c20590941a062385d86a8979c92c1e42cd8c86d3f68'
    return saved
def main():
    action=sys.argv[1]
    if action=='prepare':
        saved=protect();ROOT.mkdir(exist_ok=False)
        with (ROOT/'protected.json').open('x') as f:json.dump(saved,f,indent=2)
        print('protected',len(saved));return
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
        if action=='build':
            cmd=json.loads((WIRE/'build_wire3.json').read_text())['command']
            cmd=[str(HERE/'Diagnose.cpp') if x.endswith('/Replay.cpp') else x for x in cmd]
            cmd[-1]=str(ROOT/'diagnose')
        elif action=='observe':
            built=json.loads((ROOT/'build.json').read_text());assert built['returncode']==0
            assert sha(ROOT/'diagnose')==built['binary_sha256']
            assert built['code']=={p.name:sha(p) for p in HERE.iterdir() if p.is_file()}
            cmd=[str(ROOT/'diagnose'),str(JOURNAL),str(ROOT/'archive'),str(ROOT/'diagnostics')]
        else:raise ValueError(action)
        os.sched_setaffinity(0,{2});start=time.monotonic()
        with (ROOT/(action+'.log')).open('x') as log:
            rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600).returncode
        result=dict(command=cmd,returncode=rc,seconds=time.monotonic()-start,
          protected_unchanged=protect()==saved,max_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
          code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
        if rc in (0,10):result['binary_sha256']=sha(ROOT/'diagnose')
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(action,json.dumps({k:v for k,v in result.items() if k not in ('command','code')}),flush=True)
        raise SystemExit(rc)
if __name__=='__main__':main()
