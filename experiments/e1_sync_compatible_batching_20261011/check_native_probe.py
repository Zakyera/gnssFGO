#!/usr/bin/env python3
"""Compare old-API native probes under installed/private libraries, fixed gates."""
import json
from pathlib import Path
import numpy as np

ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/glim_sync_compatible_batching_e1_20261011/private')
NAMES = ['native_installed.log', 'native_candidate.log',
         'native_installed_repeat.log', 'native_candidate_repeat.log']

def parse(name):
    lines = (ROOT/name).read_text().splitlines()
    mats = {}
    for n, label in [(31, 'augmented_hessian'), (30, 'joint_covariance')]:
        rows = [list(map(float, line.split())) for line in lines if len(line.split()) == n]
        mats[label] = np.array(rows).reshape(-1, n, n)
    assert len(mats['augmented_hessian']) == 24
    assert len(mats['joint_covariance']) == 12
    mats['errors'] = np.array([float(line.split()[1]) for line in lines
                              if line.startswith(('CACHED_ERROR ', 'HOOK_ERROR '))])
    assert len(mats['errors']) == 24
    return mats

all_data = {name: parse(name) for name in NAMES}
reference = all_data[NAMES[0]]
result = {}
for name in NAMES[1:]:
    result[name] = {}
    for label, a in reference.items():
        b = all_data[name][label]
        delta = np.abs(a-b)
        assert np.isfinite(a).all() and np.isfinite(b).all()
        if label == 'errors':
            limits = 1e-8+1e-6*np.abs(a)
            per = delta
        else:
            limits = (1e-10 if label == 'joint_covariance' else 1e-8) + 1e-6*np.max(np.abs(a),axis=(1,2))
            per = np.max(delta,axis=(1,2))
        assert np.all(per <= limits), (name,label,per,limits)
        result[name][label] = {'max_abs': float(delta.max()),
                              'max_fraction_of_predeclared_bound': float(np.max(per/limits))}
with (ROOT/'native_probe_comparison.json').open('x') as f:
    json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
