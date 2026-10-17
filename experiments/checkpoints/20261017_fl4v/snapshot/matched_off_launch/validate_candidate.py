"""Additional source solver gate; keep the existing receiver validator intact."""
import csv, json, sys
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent
mode=sys.argv[1];assert mode in ['off','on']
S=R/f'live120_{mode}_source'
assert (R/f'live120_{mode}_control/complete.json').exists()
assert not (R/f'live120_{mode}_control/failed.txt').exists()
a=np.loadtxt(S/'acceptance.csv',delimiter=',',ndmin=2)
obj=np.loadtxt(S/'objective.csv',delimiter=',',ndmin=2)
with (S/'kimera_odometry.csv').open() as f:poses=list(csv.DictReader(f))
assert len(a)==len(obj)==len(poses) and len(a)>=570
assert np.array_equal(a[:,0],np.arange(len(a)))
assert np.isfinite(a).all() and np.isfinite(obj).all()
assert np.all(a[:,3]<=a[:,1]+1e-10*(1+np.abs(a[:,1])))
fb=a[a[:,4]==1]
assert not len(fb) or (np.all(fb[:,3]<fb[:,1]) and np.max(fb[:,7:9])<=1e-8 and np.max(fb[:,5])<=10)
log=(S/'estimators.log').read_text()
for marker in ['FL4Q','active_window_rollback','active_window_retry_local_update_','Smoother recovery','Persistent BPSAM recovery']:
    assert marker not in log,marker
effective=json.loads((S/'effective_mode_check.json').read_text())
assert effective['passed'] and effective['kimera']['cbs_experimental_native_imu_mean_alignment']=='true'
meta=(S/'run_metadata.txt').read_text()
def count(name):
    return int(next(l.split('=',1)[1] for l in meta.splitlines() if l.startswith(name+'=')))
counts={k:count(k) for k in ['g2k_injected','k2g_injected']}
assert all(v==0 if mode=='off' else v>0 for v in counts.values())
report=dict(passed=True,mode=mode,native_updates=len(a),all_steps_nonincreasing=True,
    fallback_count=len(fb),fallback_rows=fb.tolist(),iteration_cap_hits=int(np.sum(fb[:,5]==10)),
    max_cache_fresh_error=float(np.max(a[:,7])),max_tree_error=float(np.max(a[:,8])),
    planned_retirements=int(obj[:,2].sum()),source_injections=counts,stationarity_certified=False)
with (S/'candidate_validation.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
