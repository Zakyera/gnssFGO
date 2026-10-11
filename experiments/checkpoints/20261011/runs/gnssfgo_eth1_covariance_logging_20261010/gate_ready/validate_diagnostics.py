#!/usr/bin/env python3
"""Read-only diagnostic algebra gate; no estimation or weight changes.

Checks actual GTSAM factor Hessians/gradients against full-covariance whitening,
and checks the Kimera manually evaluated residual against its actual Values.
"""
import json
import re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent

def matrix(token):
    return np.array([[float(v) for v in row.split(':')]
                     for row in token.split(';')])

def relative_error(a, b):
    return float(np.linalg.norm(a-b) / max(1., np.linalg.norm(a), np.linalg.norm(b)))

def check_factor(cov, residual, hf, ht, hessian, gradient, cost):
    assert cov.shape == (6,6) and residual.size == 6
    assert all(np.isfinite(a).all() for a in [cov,residual,hf,ht,hessian,gradient])
    assert np.max(np.abs(cov-cov.T)) < 1e-12
    minimum = float(np.linalg.eigvalsh(cov).min())
    assert minimum > 0
    J = np.hstack([hf, ht])
    H = J.T @ np.linalg.solve(cov, J)
    g = J.T @ np.linalg.solve(cov, residual.reshape(6,1))
    expected_cost = .5 * float(residual.ravel() @ np.linalg.solve(cov, residual.ravel()))
    errors = dict(hessian=relative_error(H,hessian), gradient=relative_error(g,gradient),
                  cost=relative_error(np.array(cost),np.array(expected_cost)))
    assert max(errors.values()) < 1e-8, errors
    return dict(min_covariance_eigenvalue=minimum, algebra_relative_errors=errors,
                rotation_trace_rad2=float(np.trace(cov[:3,:3])),
                rho_trace_m2=float(np.trace(cov[3:,3:])), cost=float(cost))

def rows(text, marker):
    for line in text.splitlines():
        if marker in line:
            yield line.split(marker,1)[1].split(',')

def main():
    source = ROOT/'gate_on_source'
    assert (ROOT/'gate_on_control/complete.json').exists()
    logs=[p for p in (source/'ros_logs').glob('*/*stdout.log') if p.parent.name!='latest']
    glog,=[p for p in logs if 'glim_rosnode' in p.name]
    clean=lambda s:re.sub(r'\x1b\[[0-9;]*m','',s)
    gtext=clean(glog.read_text())
    ktext=clean((source/'estimators.log').read_text())
    gmat={}
    for v in rows(gtext,'CBS_RECEIVER_INCOMING_MATRIX_ROW,'):
        key=tuple(v[:4]); label=v[4]
        m=matrix(v[7]); assert m.shape==(int(v[5]),int(v[6]))
        assert label not in gmat.setdefault(key,{})
        gmat[key][label]=m
    assert 1 <= len(gmat) <= 40, 'No GLIM incoming factor audit'
    glim=[]
    for key,m in gmat.items():
        out=check_factor(m['covariance'],m['residual'],m['H_from'],m['H_to'],
                         m['hessian'],m['gradient'],m['cost'].item())
        jw=m['J_whitened']; rw=m['whitened_residual']
        errors=dict(H=relative_error(jw.T@jw,m['hessian']),
                    g=relative_error(jw.T@rw,m['gradient']),
                    rw=relative_error(rw,m['whitened_system_residual']))
        assert max(errors.values()) < 1e-8, errors
        glim.append(dict(identity=key,whitening_errors=errors,**out))
    local=list(rows(gtext,'CBS_RECEIVER_LOCAL_COMPETITION_ROW,'))
    statuses=list(rows(gtext,'CBS_RECEIVER_LOCAL_COMPETITION_STATUS_ROW,'))
    assert not statuses, statuses
    assert len(local)==len(gmat), 'Missing GLIM local-endpoint covariance'
    for v in local:
        assert v[-1]=='ok'
        covariance=matrix(v[16])
        assert np.isfinite(covariance).all() and np.linalg.eigvalsh(covariance).min()>0
    kmat={}
    for v in rows(ktext,'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW,'):
        key=tuple(v[:8]); label=(v[8],v[9])
        m=matrix(v[12]); assert m.shape==(int(v[10]),int(v[11]))
        assert label not in kmat.setdefault(key,{})
        kmat[key][label]=m
    summaries=list(rows(ktext,'CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW,'))
    assert 1<=len(summaries)<=40, 'No Kimera incoming factor audit'
    kimera=[]
    for v in summaries:
        key=tuple(v[:8]);m=kmat[key]
        assert v[10]=='0', 'Robust noise unexpectedly active'
        residual=matrix(v[25]); unwhitened=matrix(v[26]); whitened=matrix(v[27])
        consistency=relative_error(residual,unwhitened)
        assert consistency<1e-12, 'Kimera manual/Values residual mismatch'
        cov=m[('receiver_inserted_factor','covariance')]
        assert np.array_equal(cov,m[('receiver_after_source_scale','covariance')])
        assert np.array_equal(cov,m[('receiver_after_frame_conversion','covariance')])
        lin=lambda n:m[('receiver_factor_linearization',n)]
        out=check_factor(cov,unwhitened,lin('H_from'),lin('H_to'),
                         lin('external_factor_hessian'),lin('external_factor_gradient_at_zero'),float(v[29]))
        assert abs(.5*float(whitened.ravel()@whitened.ravel())-float(v[29]))<1e-9
        kimera.append(dict(identity=key,pose_source=v[9],same_Values_residual_error=consistency,**out))
    kstatus=list(rows(ktext,'CBS_COVARIANCE_EXCHANGE_STATUS_ROW,'))
    assert not kstatus, kstatus
    krelative=list(rows(ktext,'CBS_COVARIANCE_EXCHANGE_RECEIVER_RELATIVE_ROW,'))
    assert len(krelative)>=2*len(summaries), 'Missing Kimera full/local joint endpoint diagnostics'
    result=dict(passed=True,glim_samples=glim,kimera_samples=kimera,
                glim_local_covariance_rows=len(local),kimera_relative_rows=len(krelative),
                interpretation='Full-covariance factor algebra and sampled local diagnostics only. '
                'No calibrated dominance/accuracy claim. Raw mixed-unit trace ratios are NOT weight scores.')
    with (ROOT/'diagnostics_validation.json').open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
