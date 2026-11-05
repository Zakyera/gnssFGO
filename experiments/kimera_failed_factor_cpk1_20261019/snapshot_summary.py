import csv,json,math
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
original={int(r[1]):r for r in csv.reader((R/'off_capture/factor_derivatives.txt').open()) if r[0]=='factor' and r[2]=='smart_stereo'}
rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader((R/'analysis/restored_smart.csv').open())]
assert len(rows)==len(original)==687
cost_error=max(abs(r['cost']-float(original[int(r['slot'])][4])) for r in rows)
slope_error=max(abs(r['schur_slope']-float(original[int(r['slot'])][5])) for r in rows)
fd_error=max(abs(r['cached_fd']-float(original[int(r['slot'])][10])) for r in rows)
assert cost_error==0 and slope_error<1e-14 and fd_error==0,(cost_error,slope_error,fd_error)
fields=['cost','schur_slope','cached_fd','held_point_slope','schur_correction','fresh_cost','fresh_schur_slope','fresh_fd','fresh_held_point_slope','fresh_schur_correction']
sums={k:sum(r[k] for r in rows) for k in fields}
out=dict(passed=True,restored_count=len(rows),valid_count=sum(r['status']==0 for r in rows),
    max_restored_cost_error=cost_error,max_restored_schur_slope_error=slope_error,max_restored_numerical_slope_error=fd_error,
    sums=sums,max_fixed_point_analytic_vs_numerical_error=max(abs(r['held_point_slope']-r['cached_fd']) for r in rows),
    max_schur_decomposition_error=max(abs(r['schur_slope']-r['held_point_slope']-r['schur_correction']) for r in rows),
    aggregate_schur_decomposition_error=sums['schur_slope']-sums['held_point_slope']-sums['schur_correction'],
    fresh_retriangulation_slope_gap=sums['fresh_fd']-sums['fresh_schur_slope'],
    cached_slope_gap=sums['cached_fd']-sums['schur_slope'],
    largest=[r for r in rows if int(r['slot']) in (109497,109470,109467)])
assert out['max_fixed_point_analytic_vs_numerical_error']<1e-8
assert out['max_schur_decomposition_error']<1e-10
with (H/'snapshot_validation.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
