"""Numerical verification only, not a new covariance implementation."""
import json
import sys
from pathlib import Path
import numpy as np

sys.dont_write_bytecode = True
import analyze as n
a, g, H = n.a, n.g, n.H
assert not (H/'verification.json').exists()
g.self_test()


def adj(T):
    R, t = T[:3,:3], T[:3,3]
    return np.block([[R,np.zeros((3,3))],[g.hat(t)@R,R]])


rng = np.random.default_rng(722)
maximum = 0.
for _ in range(30):
    L, Z, R = (g.exp(rng.normal(size=6)*.2) for _ in range(3))
    base = L@Z@R
    expected = [adj(g.inv(Z@R)), adj(g.inv(R)), np.eye(6)]
    numeric = []
    for block in range(3):
        J = np.zeros((6,6))
        for k in range(6):
            delta = np.zeros(6); delta[k] = 1e-6
            values = []
            for sign in (1,-1):
                factors = [L.copy(), Z.copy(), R.copy()]
                factors[block] = factors[block]@g.exp(sign*delta)
                values.append(g.log(g.inv(base)@factors[0]@factors[1]@factors[2]))
            J[:,k] = (values[0]-values[1])/(2e-6)
        numeric.append(J)
        maximum = max(maximum, float(np.max(np.abs(J-expected[block]))))
    assert maximum < 2e-9
    # Full correlated joint covariance; propagation retains its cross terms.
    M = rng.normal(size=(18,18)); C = M@M.T+np.eye(18)
    Ja, Jn = np.hstack(expected), np.hstack(numeric)
    exact, finite = Ja@C@Ja.T, Jn@C@Jn.T
    assert np.linalg.norm(exact-finite)/np.linalg.norm(exact)<1e-8
    assert np.linalg.eigvalsh(exact).min()>0
    # With fixed L,R, only J_Z acts. In particular R=identity preserves Sigma_Z.
    assert np.array_equal(adj(np.eye(4)),np.eye(6))

x = json.loads((H/'results.json').read_text())
assert all(g.sha(Path(p))==h for p,h in x['input_hashes'].items())
assert x['reference']['max_bracket_ns']<=5000000
checks={}
for direction, d in x['directions'].items():
    keys=set(); worst=0.; max_cov=0.
    for r in d['records']:
        key=r['receiver_i'],r['receiver_j']; assert key not in keys;keys.add(key)
        C=np.asarray(r['cov']);assert np.max(np.abs(C-C.T))==0
        assert np.linalg.eigvalsh(C).min()>0
        assert r['source_version']['revision']==0
        for stage in ('raw','aligned','unaligned_on_receiver','raw_exact_source_timestamps'):
            e=np.r_[r[stage]['phi'],r[stage]['rho']]
            white=np.linalg.solve(np.linalg.cholesky(C),e)
            q=white@white
            worst=max(worst,float(abs(q-r[stage]['joint_normalized_squared'])/(1+q)))
        # Aligned and unaligned timing-control residuals reconstruct SAME target.
        target1=g.exp(r['aligned_mu'])@g.exp(np.r_[r['aligned']['phi'],r['aligned']['rho']])
        target2=g.exp(r['raw_mu'])@g.exp(np.r_[r['unaligned_on_receiver']['phi'],r['unaligned_on_receiver']['rho']])
        assert np.max(np.abs(target1-target2))<1e-10
    assert worst<1e-8
    checks[direction]=dict(unique_installed_factors=len(keys),max_cholesky_relative_error=worst,
        all_source_versions_first=True,independent_frozen_mean_log=(direction=='G2K'),
        sampled_preinjection_checks=d['checks']['sampled_preinjection_checks'])
result=dict(passed=True,synthetic_right_local_composition_jacobian_max_error=maximum,
    synthetic_full_joint_covariance_cross_terms_checked=True,real_factors=checks,
    input_hashes_unchanged=True,result_sha256=g.sha(H/'results.json'),
    verify_script_sha256=g.sha(Path(__file__)),
    limits='Synthetic derivative check is not a reconstruction of unlogged real bridges or their uncertainty.')
a.write(H/'verification.json',result)
print(json.dumps(result,indent=2))
