import csv,json,statistics,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OLD=Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/cached/failure_probe')
def rows(p):return list(csv.DictReader(p.open()))
b=rows(ROOT/'jacobians.csv');g=rows(ROOT/'gradients.csv');c=rows(ROOT/'gradient_contributions.csv');v=rows(ROOT/'reconstruction.csv')
summary={}
for kind in ('wnoa','source'):
 for char in ('x','v','w'):
  for h in (1e-3,1e-4,1e-5):
   a=[x for x in b if x['kind']==kind and x['key'][0]==char and float(x['step'])==h]
   summary[f'{kind}_{char}_{h}']=dict(n=len(a),
    native_max=max(float(x['native_vs_numeric']) for x in a),analytic_max=max(float(x['analytic_vs_numeric']) for x in a),
    native_median=statistics.median(float(x['native_vs_numeric']) for x in a),analytic_median=statistics.median(float(x['analytic_vs_numeric']) for x in a),
    analytic_closer_count=sum(float(x['analytic_vs_numeric'])<float(x['native_vs_numeric']) for x in a),
    max_relative_analytic=max(float(x['analytic_vs_numeric'])/max(1,float(x['reference_norm'])) for x in a))
kind_change={k:[] for k in ('wnoa','source')}
per_target={}
old={int(x['rank']):x for x in rows(OLD/'gradient_step_sensitivity.csv')}
for rank in range(20):
 a=[x for x in g if int(x['rank'])==rank]
 per_target[rank]=dict(saved_native=float(old[rank]['native_normalized']),results=a)
 for kind in kind_change:
  totals={m:sum(float(x['gradient']) for x in c if int(x['rank'])==rank and x['kind']==kind and x['method']==m) for m in ('native','analytic')}
  kind_change[kind].append(abs(totals['analytic']-totals['native']))
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for z in iter(lambda:f.read(1024*1024),b''):h.update(z)
 return h.hexdigest()
selection=json.loads((ROOT/'selection.json').read_text())
result=dict(reconstruction_max_residual=max(float(x['residual_diff']) for x in v),
 reconstruction_max_covariance=max(float(x['covariance_diff']) for x in v),
 reconstruction_max_analytic_residual=max(float(x['analytic_residual_diff']) for x in v),
 blocks=summary,targets=per_target,
 native_to_analytic_gradient_change={k:dict(sum=sum(a),maximum=max(a)) for k,a in kind_change.items()},
 inputs_unchanged=all(sha(p)==h for p,h in selection['input_hashes'].items()))
result['source_fraction_of_absolute_gradient_change']=sum(kind_change['source'])/(sum(kind_change['source'])+sum(kind_change['wnoa']))
result['maximum_selected_native_gradient_roundtrip_difference']=max(abs(float(next(x for x in g if int(x['rank'])==rank and x['method']=='native')['normalized'])-float(old[rank]['native_normalized'])) for rank in range(20))
result['maximum_selected_analytic_gradient']=max(float(x['normalized']) for x in g if x['method']=='analytic')
with (ROOT/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ('blocks','targets')},indent=2))
for k,x in summary.items():
 if k.endswith('0.001'):print(k,x)
assert result['inputs_unchanged']
