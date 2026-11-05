"""Read-only input/code integrity and bounded CP1 analysis checks."""
import ast
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
ARCHIVE = REPO/'experiments/checkpoints/20261017_fl4v'


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()


manifest = json.loads((ARCHIVE/'manifest.json').read_text())
print('manifest sections',list(manifest))
checked = {}
for section in manifest.values():
    if not isinstance(section,list):
        continue
    for entry in section:
        if not isinstance(entry,dict):
            continue
        archived = entry.get('archived','')
        if archived.startswith('snapshot/glim_s4/') and archived.endswith(('.h','.cpp','.json')):
            for p in [ARCHIVE/archived,Path(entry['original'])]:
                assert sha(p)==entry['sha256'],str(p)
                checked[str(p)]=entry['sha256']
        if entry.get('path','').endswith('/release_2/libglim_ros.so'):
            p=Path(entry['resolved'])
            assert sha(p)==entry['sha256']
            checked[str(p)]=entry['sha256']

assert any(p.endswith('/overlay/cbs_bridge.cpp') for p in checked)
assert any(p.endswith('/source/Provider.h') for p in checked)
assert any(p.endswith('/libglim_ros.so') for p in checked)
results = {}
for seq in ['eig2','spx3']:
    p=HERE/f'{seq}_evidence.json'
    x=json.loads(p.read_text())
    assert x['inputs_unchanged']
    for inp,h in x['input_hashes'].items():
        assert sha(inp)==h,inp
        checked[inp]=h
    results[seq]=dict(evidence_sha256=sha(p),matched_age40=x['matched_age40_cohort']['n'])
    for mode in ['off','on']:
        m=x['modes'][mode]
        assert m['log_counts']['first']==m['log_counts']['provider']==590
        assert m['log_counts']['matrix_dumps']==0
        assert m['exact_source_transport_matches']==m['log_counts']['first']+m['log_counts']['revised']
        assert m['schur_direct_relative_trace_max']<1e-12
        assert m['decomposition_relative_sum_error']<2e-5

# Minimal independent mathematical check: for a fixed linear Gaussian model,
# changing only residual/RHS changes the estimate, not its curvature covariance.
# This illustrates scope; it does NOT reproduce an actual LiDAR error mechanism.
rng=np.random.default_rng(20261019)
A=rng.normal(size=(48,12));H=A.T@A
b1=rng.normal(size=48);b2=b1+10*A[:,0]
x1=np.linalg.solve(H,A.T@b1);x2=np.linalg.solve(H,A.T@b2)
P=np.linalg.inv(H);J=rng.normal(size=(6,12))
C=J@P@J.T
decomp=J[:,:6]@P[:6,:6]@J[:,:6].T+J[:,6:]@P[6:,6:]@J[:,6:].T
decomp+=J[:,:6]@P[:6,6:]@J[:,6:].T+J[:,6:]@P[6:,:6]@J[:,:6].T
assert np.max(np.abs(C-decomp))<1e-12
assert np.linalg.norm(x2-x1)>9
for p in HERE.glob('*.py'):
    ast.parse(p.read_text())
out=dict(checked_hashes=checked,inputs_unchanged=True,sequences=results,
    synthetic_decomposition_max=float(np.max(np.abs(C-decomp))),
    fixed_H_mean_shift_norm=float(np.linalg.norm(x2-x1)),
    fixed_H_covariance_unchanged_by_construction=True,
    estimator_or_cpp_tests_run=False)
with (HERE/'verification.json').open('x') as f:
    json.dump(out,f,indent=2,allow_nan=False)
print('verified',len(checked),'input/code hashes; no production execution')
