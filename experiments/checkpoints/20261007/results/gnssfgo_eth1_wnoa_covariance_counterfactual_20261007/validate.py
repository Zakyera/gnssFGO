"""Post-run conservation/identity checks and descriptive results; no estimator invocation."""
from pathlib import Path
import csv,json,hashlib,subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import ROOT,BASE,REPO,sha
manifest=json.loads((ROOT/'manifest.json').read_text())
assert json.loads((ROOT/'complete.json').read_text())['passed']
assert json.loads((ROOT/'substitution_validation.json').read_text())['passed']
for p,h in manifest['protected'].items():assert sha(p)==h,p
head=subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip()
assert head==manifest['head']
assert hashlib.sha256(subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])).hexdigest()==manifest['tracked_diff_sha256']
baseline=np.loadtxt(BASE/'on_wnoa/snapshot_final/incremental_states.csv',delimiter=',',skiprows=1)
checks={};families={}
for label in ['unchanged_control','on_means_off_cov']:
    p=ROOT/label;meta=json.loads((p/'optimizer.json').read_text());assert meta['normalized_gradient']<=1e-5
    states=np.loadtxt(p/'states.csv',delimiter=',',skiprows=1)
    assert np.array_equal(states[:,:2],baseline[:,:2]) and np.isfinite(states).all()
    assert np.max(abs(np.linalg.norm(states[:,5:9],axis=1)-1))<1e-12
    pos=np.linalg.norm(states[:,2:5]-baseline[:,2:5],axis=1)
    angles=(Rotation.from_quat(baseline[:,5:9]).inv()*Rotation.from_quat(states[:,5:9])).magnitude()
    checks[label]={'position_rms_m':float(np.sqrt(np.mean(pos**2))),'position_max_m':float(pos.max()),'rotation_rms_deg':float(np.sqrt(np.mean(angles**2))*180/np.pi)}
    if label=='unchanged_control':assert max(checks[label].values())<1e-5
    families[label]={}
    for stage in ['initial','final']:
        rows=list(csv.DictReader((p/(stage+'_factors.csv')).open()));assert len(rows)==2965
        fs={}
        for k in {r['kind'] for r in rows}:
            rr=[r for r in rows if r['kind']==k];cost=sum(float(r['half_squared_whitened_error']) for r in rr)
            fs[k]={'count':len(rr),'cost':cost,'mean_squared_whitened_residual':2*cost/len(rr)}
        assert {k:v['count'] for k,v in fs.items()}=={'glim_hesai':1189,'kimera_front':581,'wnoa':1192,'prior':3}
        families[label][stage]=fs
orig=list(csv.DictReader((ROOT/'unchanged_control/initial_factors.csv').open()))
swap=list(csv.DictReader((ROOT/'on_means_off_cov/initial_factors.csv').open()))
assert all((a['slot'],a['kind'],a['dimension'],a['keys'])==(b['slot'],b['kind'],b['dimension'],b['keys']) for a,b in zip(orig,swap))
assert all(a==b for a,b in zip(orig,swap) if a['kind'] in ['wnoa','prior'])
result={'passed':True,'protected_files_unchanged':len(manifest['protected']),'unchanged_head':head,'tracked_diff_unchanged':True,
        'state_checks':checks,'factor_summaries':families,'diagnostic_binary_sha256':sha(ROOT/'counterfactual'),
        'artifact_hashes':{str(p.relative_to(ROOT)):sha(p) for p in ROOT.rglob('*') if p.is_file() and p.suffix in ['.cpp','.py','.json','.tum','.csv']}}
if (ROOT/'metrics.json').exists():
    m=json.loads((ROOT/'metrics.json').read_text());a=m['off_fused_final'];b=m['on_fused_final'];c=m['on_means_off_cov']
    result['score_effect']={k:{'off':a[k],'on':b[k],'swap':c[k],'on_relative_change':c[k]/b[k]-1,
          'fraction_off_on_gap_closed':(b[k]-c[k])/(b[k]-a[k])} for k in ['ate_m','translation_rpe_m','rotation_rpe_deg']}
with (ROOT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['factor_summaries','artifact_hashes']},indent=2))
