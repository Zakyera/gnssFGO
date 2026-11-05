"""Standalone native-log frame adapter candidate; not installed in transport.

Call ONLY on native diagnostic rows. Recorded ROS wire beliefs are already CPT7.
Preserve IDs, physical endpoints, source versions and all availability metadata.
No clock or arrival time is invented here.
"""
import math


def adjoint(T):
    R=[[float(T[i][j]) for j in range(3)] for i in range(3)]
    x,y,z=[float(T[i][3]) for i in range(3)]
    skew=[[0.,-z,y],[z,0.,-x],[-y,x,0.]]
    A=[[0.]*6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            A[i][j]=A[i+3][j+3]=R[i][j]
            A[i+3][j]=math.fsum(skew[i][k]*R[k][j] for k in range(3))
    return A


def native_to_cpt7(kind, belief, T_alpha_cpt7):
    if kind not in ('glim', 'kimera'):
        raise ValueError('Explicit native logger kind required')
    out = dict(belief)
    if kind == 'glim':
        return out
    X = [list(map(float,row)) for row in T_alpha_cpt7]
    if len(X)!=4 or any(len(row)!=4 for row in X) or not all(math.isfinite(v) for row in X for v in row):
        raise ValueError('Invalid explicit rigid-body calibration')
    Y=[[float(i==j) for j in range(4)] for i in range(4)]
    for i in range(3):
        for j in range(3):Y[i][j]=X[j][i]
        Y[i][3]=-math.fsum(X[j][i]*X[j][3] for j in range(3))
    J=adjoint(Y)
    mu=list(map(float,belief['mean']));flat=list(map(float,belief['covariance']))
    if len(mu)!=6 or len(flat)!=36 or math.sqrt(math.fsum(v*v for v in mu[:3]))>math.pi+1e-8:
        raise ValueError('Expected canonical rotation-translation Logmap coordinates')
    C=[flat[i*6:(i+1)*6] for i in range(6)]
    # Log(X^-1 Exp(mu) X)=Ad(X^-1)mu for canonical source Logmap coordinates.
    # Fixed scalar sums avoid host/container BLAS rounding differences.
    out['mean']=[math.fsum(J[i][k]*mu[k] for k in range(6)) for i in range(6)]
    JC=[[math.fsum(J[i][k]*C[k][j] for k in range(6)) for j in range(6)] for i in range(6)]
    out['covariance']=[math.fsum(JC[i][k]*J[j][k] for k in range(6)) for i in range(6) for j in range(6)]
    return out
