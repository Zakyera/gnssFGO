"""Independent numerical and immutable-input checks of analysis outputs."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.stats import chi2
sys.dont_write_bytecode=True
import analyze as n
a=n.a;H=n.H
assert not (H/'verification.json').exists()
a.geo.self_test()
rng=np.random.default_rng(419)
M=rng.normal(size=(6,6));D=np.diag([.01,.02,.03,.1,.2,.3]);C=D@(M@M.T+np.eye(6))@D
draws=rng.normal(size=(50000,6))@np.linalg.cholesky(C).T
dense=np.einsum('ni,ni->n',draws,np.linalg.solve(C,draws.T).T)
white=np.linalg.solve(np.linalg.cholesky(C),draws.T).T
assert np.max(np.abs(dense-np.sum(white**2,axis=1)))<1e-10
coverage=float(np.mean(dense<chi2.ppf(.95,6)));assert .945<coverage<.955
J=rng.normal(size=(6,6))+3*np.eye(6);e=draws@J.T
assert np.max(np.abs(np.einsum('ni,ni->n',e,np.linalg.solve(J@C@J.T,e.T).T)-dense))<1e-9
checks={}
for seq in n.CASES:
 p=H/(seq+'_results.json');x=json.loads(p.read_text())
 assert all(a.geo.sha(Path(path))==h for path,h in x['input_hashes'].items())
 assert x['reference']['max_used_bracket_ns']<=5000001
 seen=set();max_error=0.
 for r in x['records']:
  key=(r['source'],r['a'],r['b'],r['stage']);assert key not in seen;seen.add(key)
  assert r['off_metadata']['update']==r['on_metadata']['update']
  for mode in ('off','on'):
   cov=np.asarray(r[mode+'_cov']);err=np.r_[r[mode]['phi'],r[mode]['rho']]
   assert np.max(np.abs(cov-cov.T))==0 and np.linalg.eigvalsh(cov).min()>0
   z=np.linalg.solve(np.linalg.cholesky(cov),err)
   max_error=max(max_error,float(abs(z@z-r[mode]['joint_normalized_squared'])/(1+z@z)))
   # Independent transform residual construction from each stored endpoint pair.
   if mode=='on':
    Zoff=a.geo.exp(r['off_mu']);Zon=a.geo.exp(r['on_mu'])
    reference_off=Zoff@a.geo.exp(np.r_[r['off']['phi'],r['off']['rho']])
    reference_on=Zon@a.geo.exp(err)
    assert np.max(np.abs(reference_on-reference_off))<1e-10
 assert max_error<1e-8
 for source,c in x['age4s_cohorts'].items():
  assert c['eligible']+c['excluded']==c['total']
  groups={stage:{(r['a'],r['b']) for r in x['records'] if r['stage']==stage and r['source']==source} for stage in n.STAGES}
  assert groups['first']==groups['last_common_update']
  assert groups['age4s_first']==groups['age4s_later']
  assert len(groups['age4s_later'])==c['eligible']
 checks[seq]=dict(passed=True,unique_paired_records=len(seen),max_cholesky_relative_error=max_error,
  source_covariance_policy='native_local',inputs_unchanged=True,result_sha256=a.geo.sha(p))
a.write(H/'verification.json',dict(passed=True,synthetic_coverage=coverage,dense_coordinate_invariance=True,
 cases=checks,limits='Analysis numerical validation only; no covariance calibration or independence certificate'))
print(json.dumps(checks,indent=2))
