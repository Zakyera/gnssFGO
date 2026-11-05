"""Standalone native-log frame adapter candidate; not installed in transport.

Call ONLY on native diagnostic rows. Recorded ROS wire beliefs are already CPT7.
Preserve IDs, physical endpoints, source versions and all availability metadata.
No clock or arrival time is invented here.
"""
import importlib.util
from pathlib import Path
import numpy as np

_geometry = Path('/home/yeranis/repos/gnssFGO/experiments/fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
_spec = importlib.util.spec_from_file_location('cp94_geometry', _geometry)
geo = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(geo)


def adjoint(T):
    A = np.zeros((6, 6))
    A[:3,:3] = A[3:,3:] = T[:3,:3]
    A[3:,:3] = geo.hat(T[:3,3]) @ T[:3,:3]
    return A


def native_to_cpt7(kind, belief, T_alpha_cpt7):
    if kind not in ('glim', 'kimera'):
        raise ValueError('Explicit native logger kind required')
    out = dict(belief)
    if kind == 'glim':
        return out
    X = np.asarray(T_alpha_cpt7, float)
    Y = geo.inv(X)
    J = adjoint(Y)
    M = geo.exp(np.array(belief['mean'], float, copy=True))
    C = np.array(belief['covariance'], float).reshape(6,6)
    out['mean'] = geo.log(Y @ M @ X).tolist()
    out['covariance'] = (J @ C @ J.T).reshape(-1).tolist()
    return out
