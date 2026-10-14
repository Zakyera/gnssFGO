"""One saved-objective numerical diagnosis; no source or journal replay."""
import fcntl,hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_cause_fl3g_20261014')
PREV=Path('/home/yeranis/results/gnssfgo_fixed_lag_incremental_fl3f_20261014')
A=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl3a_20261013')
D=Path('/home/yeranis/results/gnssfgo_fixed_lag_isam_convergence_fl3d_20261013')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def protect():
    saved=json.loads((PREV/'protected.json').read_text())
    for path,h in saved.items():assert sha(path)==h,('changed protected',path)
    for folder in (A/'diagnostics/snapshot',D/'diagnostics'):
        for p in folder.iterdir():
            if p.is_file():saved[str(p)]=sha(p)
    for p in [HERE.parent/'fixed_lag_failure_fl3a_20261013/SnapshotAudit.cpp',HERE.parent/'fixed_lag_incremental_fl3f_20261014/PROTOCOL.txt']:
        saved[str(p)]=sha(p)
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
        if action=='build':
            cmd=json.loads((A/'build_snapshot.json').read_text())['command']
            cmd=[str(HERE/'Cause.cpp') if x.endswith('/SnapshotAudit.cpp') else x for x in cmd];cmd[-1]=str(ROOT/'cause')
        elif action=='diagnose':
            built=json.loads((ROOT/'build.json').read_text());assert built['returncode']==0 and built['binary_sha256']==sha(ROOT/'cause')
            assert built['code']=={p.name:sha(p) for p in HERE.iterdir() if p.is_file()}
            cmd=[str(ROOT/'cause'),str(A/'diagnostics/snapshot'),str(D/'diagnostics'),str(ROOT/'diagnostics')]
        else:raise ValueError(action)
        env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
        os.sched_setaffinity(0,{2});start=time.monotonic()
        with (ROOT/(action+'.log')).open('x') as log:rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
        result=dict(returncode=rc,seconds=time.monotonic()-start,command=cmd,protected_unchanged=protect()==saved,code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
        if rc==0:result['binary_sha256']=sha(ROOT/'cause')
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(action,rc,result['seconds']);raise SystemExit(rc)
if __name__=='__main__':main()
