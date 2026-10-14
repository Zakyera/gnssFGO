"""Summarize the saved numerical diagnostics; never modifies their inputs."""
import csv,json,math
from collections import defaultdict
from driver import ROOT,HERE,protect,sha

diag=ROOT/'diagnostics'
original=list(csv.DictReader((diag/'curvature.csv').open()))
groups={r['slot']:r['group'] for r in original}
counts={g:len({r['slot'] for r in original if r['group']==g}) for g in set(groups.values())}
rows=list(csv.DictReader((diag/'gradient_curvature.csv').open()))
aggregate=defaultdict(lambda:defaultdict(float))
for r in rows:
    assert all(math.isfinite(float(r[k])) for k in ('gn_curvature','true_curvature','residual_curvature'))
    for group in (groups[r['slot']],'ALL'):
        for field in ('gn_curvature','true_curvature','residual_curvature'):
            aggregate[(r['h'],group)][field]+=float(r[field])
result=[]
for (h,group),v in aggregate.items():
    result.append(dict(h=float(h),group=group,**v,residual_to_gn=v['residual_curvature']/v['gn_curvature']))
saved=json.loads((ROOT/'protected.json').read_text());assert saved==protect()
checks=dict(line.split() for line in (diag/'checks.txt').read_text().splitlines())
assert abs(float(checks['reference_gradient'])-7.977808819599585e-6)<1e-12
summary=dict(protected_unchanged=len(saved),saved_objective_verified=True,
             checks=checks,factor_counts=counts,gradient_curvature=result,
             original_residual_second_difference_max_first_derivative_error=max(float(r['fd_jd_error']) for r in original),
             limitations=['directional, not a full Hessian or eigenmode attribution',
                          'source-factor second derivatives depend on finite-difference step; no precise causal shares',
                          'no native-source solve or optimizer candidate executed'],
             late_steps=list(csv.DictReader((diag/'late_steps.csv').open())),
             marginal_offsets=list(csv.DictReader((diag/'marginal_offsets.csv').open())),
             code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
with (ROOT/'analysis.json').open('x') as f:json.dump(summary,f,indent=2)
print('PROTECTED_UNCHANGED',len(saved))
for r in result:print(r)
