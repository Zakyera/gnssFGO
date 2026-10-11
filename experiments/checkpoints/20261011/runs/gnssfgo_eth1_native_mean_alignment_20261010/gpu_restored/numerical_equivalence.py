"""Frozen, unit-aware validation tolerance; never used by the estimator."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

LIMITS = dict(position_m=1e-7, orientation_rad=1e-8,
              velocity_m_s=1e-7, omega_rad_s=1e-7)
EXACT_FILES = ['support_clock.csv', 'source_model_covariance.csv',
               'model_covariance.csv', 'causal_dispatch.csv',
               'source_accounting.csv', 'unsupported_beliefs.csv',
               'measurement_events.csv']

def rows(path):
    with Path(path).open() as f: return list(csv.DictReader(f))

def check_bounds(values):
    for key, limit in LIMITS.items():
        assert np.isfinite(values[key]) and values[key] <= limit, (key, values[key], limit)

def compare(a, b):
    a, b = Path(a), Path(b)
    exact = {}
    for name in EXACT_FILES:
        x, y = (a/name).read_bytes(), (b/name).read_bytes()
        assert x == y, ('non-numerical mismatch', name)
        exact[name] = hashlib.sha256(x).hexdigest()
    states = {}
    for name in ['states.csv', 'online_states.csv']:
        x, y = rows(a/name), rows(b/name)
        assert len(x) == len(y) and len(x) > 0
        for c in ['state_index', 'timestamp_ns']:
            assert [r[c] for r in x] == [r[c] for r in y], (name, c)
        def values(r, cols):
            v = np.array([[float(i[c]) for c in cols] for i in r])
            assert np.isfinite(v).all()
            return v
        d = {}
        for key, cols in [('position_m', ['px','py','pz']),
                          ('velocity_m_s', ['vx','vy','vz']),
                          ('omega_rad_s', ['wx','wy','wz'])]:
            d[key] = float(np.linalg.norm(values(x, cols)-values(y, cols), axis=1).max())
        qx, qy = values(x, ['qx','qy','qz','qw']), values(y, ['qx','qy','qz','qw'])
        assert np.max(np.abs(np.linalg.norm(qx, axis=1)-1)) < 1e-10
        assert np.max(np.abs(np.linalg.norm(qy, axis=1)-1)) < 1e-10
        d['orientation_rad'] = float((Rotation.from_quat(qx).inv()*Rotation.from_quat(qy)).magnitude().max())
        check_bounds(d)
        states[name] = dict(rows=len(x), max_difference=d,
                           byte_identical=(a/name).read_bytes() == (b/name).read_bytes())
    for name in ['trajectory.tum', 'online_trajectory.tum']:
        x, y = np.loadtxt(a/name), np.loadtxt(b/name)
        assert x.shape == y.shape and np.array_equal(x[:,0], y[:,0])
        assert np.isfinite(x).all() and np.isfinite(y).all()
        dp = float(np.linalg.norm(x[:,1:4]-y[:,1:4], axis=1).max())
        dq = float((Rotation.from_quat(x[:,4:8]).inv()*Rotation.from_quat(y[:,4:8])).magnitude().max())
        assert dp <= LIMITS['position_m'] and dq <= LIMITS['orientation_rad']
        states[name] = dict(rows=len(x), max_position_m=dp, max_orientation_rad=dq,
                           byte_identical=(a/name).read_bytes() == (b/name).read_bytes())
    # Slot identities/order remain exact; wall time/correction floats are not IDs.
    x, y = rows(a/'revision_events.csv'), rows(b/'revision_events.csv')
    assert len(x) == len(y)
    for col in ['ordinal','from_index','to_index','revision','record_timestamp_ns',
                'outcome','old_factor_index','new_factor_index','active_factors']:
        assert [r[col] for r in x] == [r[col] for r in y], ('revision ledger', col)
    sa, sb = json.loads((a/'summary.json').read_text()), json.loads((b/'summary.json').read_text())
    ignored = {'wall_seconds','revision_update_seconds','path_length_m'}
    assert {k:v for k,v in sa.items() if k not in ignored} == {k:v for k,v in sb.items() if k not in ignored}
    stationarity = {}
    for key, directory in [('reference', a), ('repeat', b)]:
        c = rows(directory/'publication_convergence.csv')
        assert len(c) == sa['support_states']
        assert all(r['status']=='stationary' and np.isfinite(float(r['gradient_after']))
                   and float(r['gradient_after']) <= 1e-5 for r in c)
        stationarity[key] = max(float(r['gradient_after']) for r in c)
    return dict(passed=True, equivalence='NUMERICAL_NOT_NECESSARILY_BYTE_IDENTICAL',
                limits=LIMITS, exact_sha256=exact, states=states,
                revision_slot_ledger_exact=True, max_publication_gradient=stationarity)
