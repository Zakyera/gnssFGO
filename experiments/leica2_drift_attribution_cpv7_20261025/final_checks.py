"""Read-only integrity checks; writes only a new CP-V7 closeout receipt."""
import json
from pathlib import Path
import numpy as np
import analyze as a

h=Path(__file__).resolve().parent
assert not (h/'closeout.json').exists()
o=h/'outputs'
v=json.loads((o/'verification.json').read_text())
s=json.loads((o/'directional_checks.json').read_text())
m=json.loads((o/'metrics.json').read_text())
assert v['passed'] and s['passed']
assert all(a.g.sha(Path(p))==sha for p,sha in v['input_sha256'].items())
assert all(a.g.sha(Path(p))==sha for p,sha in s['input_sha256'].items())
z=np.load(o/'time_series.npz');mask=z['valid'];ts=z['timestamp_ns']
assert mask.sum()==m['coverage']['valid_queries']==3850
assert sum(x['n'] for x in m['coverage']['segments'])==3850
assert np.all(np.diff(ts)==100_000_000)
for mode in a.MODES:
    for role in a.ROLES:
        key=mode+'_'+role
        for hor in (1,5,10,20):
            x=z[key+f'_rpe_{hor}s'];i=x[:,0].astype(int);j=x[:,1].astype(int)
            assert np.isfinite(x).all() and np.all(ts[j]-ts[i]==hor*1_000_000_000)
            assert np.array_equal(i,a.intervals(mask,hor*10))
            # Independent norm conservation for saved angular components.
            assert np.allclose(np.linalg.norm(x[:,4:7],axis=1),x[:,3],atol=1e-10)
            d=m['trajectories'][mode+'/'+role]['horizons'][str(hor)]
            assert abs(np.sqrt(np.mean(x[:,2]**2))-d['translation_m']['rms'])<1e-12
            assert abs(np.sqrt(np.mean(x[:,3]**2))-d['rotation_deg']['rms'])<1e-12
assert m['peers']['original_on']['checks']['verified_pre_count']==1905
assert m['peers']['rotation10_on']['checks']['verified_pre_count']==1908
assert m['peers']['paired']['n']==1384
assert max(x['absolute_difference_from_saved_m'] for x in m['score_crosscheck'].values())<1e-8
report=(h/'REPORT.txt').read_text()
assert 'No source run' in report and 'NOT end of the428.75s capture' in report
files=[p for p in h.rglob('*') if p.is_file() and p.name!='closeout.json']
artifacts={str(p):a.g.sha(p) for p in sorted(files)}
result=dict(passed=True,input_hashes_verified=len(v['input_sha256']),supplement_input_hashes_verified=len(s['input_sha256']),
            independent_saved_array_checks=True,artifacts_sha256=artifacts,
            output_size_bytes=sum(p.stat().st_size for p in files),
            no_capture_or_solver_executed=True,no_container_operation=True,no_covariance_or_setting_changed=True,
            no_new_experiment_authorized=True,baseline_preserved=True,candidate_not_promoted=True)
a.save(h/'closeout.json',result)
print(json.dumps({k:v for k,v in result.items() if k!='artifacts_sha256'},indent=2))
