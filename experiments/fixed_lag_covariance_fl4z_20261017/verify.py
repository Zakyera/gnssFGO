"""Independent numerical checks and retained-result cross-checks for FL4z."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.stats import chi2

sys.dont_write_bytecode = True
import analyze as a

out = a.HERE/'verification.json'
assert not out.exists()
a.geo.self_test()
rng = np.random.default_rng(419)
M = rng.normal(size=(6,6))
D = np.diag([.01,.02,.03,.1,.2,.3])
C = D@(M@M.T+np.eye(6))@D
draws = rng.normal(size=(50000,6))@np.linalg.cholesky(C).T
dense = np.einsum('ni,ni->n',draws,np.linalg.solve(C,draws.T).T)
white = np.linalg.solve(np.linalg.cholesky(C),draws.T).T
assert np.max(np.abs(dense-np.sum(white**2,axis=1)))<1e-10
coverage = float(np.mean(dense<chi2.ppf(.95,6)))
assert .945<coverage<.955
# Invertible coordinate changes must preserve a dense quadratic discrepancy.
J = rng.normal(size=(6,6))+3*np.eye(6)
transformed = draws@J.T
q = np.einsum('ni,ni->n',transformed,np.linalg.solve(J@C@J.T,transformed.T).T)
assert np.max(np.abs(q-dense))<1e-9

checks = {}
for seq in a.CASES:
    p = a.HERE/f'{seq}_results.json'
    x = json.loads(p.read_text())
    assert all(a.geo.sha(Path(path))==h for path,h in x['input_hashes'].items())
    assert x['reference']['max_used_bracket_ns']<=5000001
    assert all(v['matches_exported_version']==v['selected'] for v in x['injection_checks'].values())
    assert len(x['records'])==2*x['common_edges']
    identity=set()
    max_chol=0.
    for r in x['records']:
        key=(r['source'],r['a'],r['b'],r['stage'])
        assert key not in identity
        identity.add(key)
        assert r['off_metadata']['update']==r['on_metadata']['update']
        for mode in ['off','on']:
            m=r[mode+'_metadata']
            assert m['a']==r['a'] and m['b']==r['b']
            assert m['arrival']<=x['config']['upper']
            c=np.asarray(r[mode+'_cov'])
            e=np.r_[r[mode]['phi'],r[mode]['rho']]
            z=np.linalg.solve(np.linalg.cholesky(c),e)
            delta=abs(z@z-r[mode]['joint_normalized_squared'])/(1+z@z)
            max_chol=max(max_chol,float(delta))
            assert np.max(np.abs(c-c.T))==0 and np.linalg.eigvalsh(c).min()>0
        if r['stage']=='first':
            assert r['off_metadata']['revision']==r['on_metadata']['revision']==0
    assert max_chol<1e-8
    if seq in ['eig2','spx3']:
        previous=json.loads((a.HERE.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017'/f'{seq}_results.json').read_text())
        old=previous['overall']['errors']['aligned_glim'] if 'aligned_glim' in previous['overall']['errors'] else None
        # Use per-edge records, independent of summary label naming.
        old_records={(r['receiver_from_ns'],r['receiver_to_ns']):r for r in previous['records']}
        max_error=0.
        for r in x['injections']['G2K']['records']:
            before=old_records[r['receiver_from_ns'],r['receiver_to_ns']]
            labels=list(before['errors'])
            label=next(k for k in labels if 'aligned' in k and 'oracle' not in k)
            max_error=max(max_error,abs(before['errors'][label]['translation_m']*1000-r['aligned']['cartesian_error_mm']),
                          abs(before['errors'][label]['rotation_deg']-r['aligned']['rotation_error_deg']))
        assert max_error<1e-10
    else:
        max_error=None
    checks[seq]=dict(unique_paired_records=len(identity),max_relative_cholesky_error=max_chol,
                     fl4y_max_error_difference=max_error,result_sha256=a.geo.sha(p),inputs_unchanged=True)

a.write(out,dict(synthetic_full_dense_chi2_95_coverage=coverage,
                 dense_whitening_and_coordinate_invariance=True,seq=checks,
                 caveat='Synthetic check validates analysis only, not empirical covariance calibration.'))
print(json.dumps(checks,indent=2))
