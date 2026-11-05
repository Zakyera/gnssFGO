"""Check saved-baseline reader identities, not a new capture or candidate score."""
import importlib.util,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('cpv39',H/'analyze.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
g,_,checks=m.m.kimera(m.S);native=m.a.load_curve(m.S/'kimera_odometry.csv')
with np.load(m.E/'newest_absolute_comparison_cpv27_20261028/velodyne_eig2.npz') as z:
    p,r,good=m.a.sample(g,z['timestamp_ns'],500000000);p,r=m.a.prism(p,r)
    assert np.array_equal(p,z['off_kimera_absolute_position']) and np.array_equal(r,z['off_kimera_absolute_rotation']) and good[z['common_valid']].all()
summary,events=m.window_summary(m.S/'estimators.log',g[0],native,g)
old=json.loads((m.E/'eig2_kimera_jitter_cpv34_20261029/velodyne_eig2_off_summary.json').read_text())
for key in ('previous_revision_m','absolute_step_m','same_update_motion_m','revision_rotation_deg'):
    for stat in ('median','p95','maximum','mean'):assert summary[key][stat]==old[key][stat],(key,stat)
for key in ('revisions_over_half_m','revisions_over_one_m','paired_steps','missing_step_ids'):assert summary[key]==old[key],key
logs,cost,timing=m.log_summary(m.S/'estimators.log',m.S/'objective.csv');assert len(cost)==len(timing)==1761
m.save(H/'reporting_selftest.json',dict(passed=True,rows=1761,exact_original_common_grid_and_prism=True,exact_CPV34_revision_stats=True,baseline_covariance=logs['covariance'],baseline_extraction_checks=checks))
print('PASS saved-baseline reporting selftest; no candidate scoring/source execution')
