"""Independent arithmetic and immutability checks for the retained-log audit."""
import hashlib,json,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.stats import spearmanr

ROOT=Path('/home/yeranis/results/gnssfgo_eth1_covariance_contraction_events_20261008')
HERE=Path(__file__).resolve().parent
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()

a,b=ROOT/'final',ROOT/'repeat'
files=['summary.json','edges.json','steps.json','updates.json','inputs_before.json','inputs_after.json',
       'validation.json','presentation/detail.json','presentation/representative_edges.json','presentation/window_age_and_onset.png']
repeats={f:sha(a/f) for f in files}
assert all(sha(b/f)==h for f,h in repeats.items())
protected=json.loads((a/'inputs_before.json').read_text())
assert json.loads((a/'inputs_after.json').read_text())==protected
assert all(sha(p)==h for p,h in protected.items())
S=json.loads((a/'summary.json').read_text());E=json.loads((a/'edges.json').read_text());T=json.loads((a/'steps.json').read_text())
ratio_error=0.;rank_error=0.;count=0
for mode in E:
    by_edge={(e['i'],e['j']):{v['u']:v for v in e['evolution']} for e in E[mode]}
    for s in T[mode]:
        e=by_edge[(s['i'],s['j'])];x=e[s['u']];y=e[s['from_u']]
        for j,part in enumerate(('rotation','rho')):
            # Independently recover root-trace ratio from retained source numeric
            # diagnostic, not from the journal sqrt trace used by analyze.py.
            r=np.sqrt(x['numeric'][part]/y['numeric'][part])
            ratio_error=max(ratio_error,abs(r-s['sigma_ratio'][j]));count+=1
    for j,part in enumerate(('rotation','rho')):
        ranks=[]
        for u in sorted(set(s['u'] for s in T[mode])):
            ss=[s for s in T[mode] if s['u']==u and s['late']]
            if len(ss)>=10:ranks.append(float(spearmanr([s['age_updates'] for s in ss],[s['sigma_ratio'][j] for s in ss]).statistic))
        rank_error=max(rank_error,abs(np.median(ranks)-S['modes'][mode]['within_update_age'][part]['correlation']['median']))
assert ratio_error<1e-10 and rank_error<1e-12
repo=Path('/home/yeranis/repos/gnssFGO')
assert subprocess.check_output(['git','-C',str(repo),'diff','HEAD','--binary'])==b''
heads={}
for name,path in [('gnssFGO',repo),('cbs',Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs')),('Kimera-VIO',Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO'))]:
    heads[name]=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
result=dict(passed=True,protected_files_unchanged=len(protected),deterministic_files=len(files),
    repeated_sha256=repeats,independent_source_ratio_checks=count,max_ratio_difference=ratio_error,
    max_scipy_rank_difference=rank_error,heads=heads,
    audit_files={str(p):sha(p) for p in sorted(HERE.glob('*')) if p.is_file()},
    no_estimator_runs=True,no_production_changes=True,no_new_commits=True)
with (ROOT/'validation_final.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ('audit_files','repeated_sha256')},indent=2))
