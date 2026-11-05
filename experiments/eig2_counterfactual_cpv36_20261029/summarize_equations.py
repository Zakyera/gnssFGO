"""Join read-only equation and derivative evidence; no solver invocation."""
import csv,json
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
d=json.loads((H/'diagnostic_results_v2.json').read_text());out=[]
for r in d['checkpoints']:
    k=r['update'];p=P/'inspection'/('update_'+str(k))
    eq={a:float(b) for a,b in (l.split(',',1) for l in (p/'equations.txt').read_text().splitlines())}
    types={int(x['packed_index']):x['type'] for x in csv.DictReader((p/'native_factor_map.csv').open())}
    failures={}
    for name,s in r['starts'].items():
        failed=s['derivatives']['native']['failed_factor_ids']
        assert all('SmartStereoProjectionPoseFactor' in types[x] for x in failed)
        failures[name]=dict(failed_factors=len(failed),all_failed_are_smart_stereo=True)
    out.append(dict(update=k,equations=eq,native_derivative_failures=failures))
with (H/'equation_audit.json').open('x') as f:json.dump(out,f,indent=2)
for r in out:print(json.dumps(r))
