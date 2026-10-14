"""Read-only numerical summaries of the one frozen failed objective."""
import csv,json
from pathlib import Path
import numpy as np
from driver import ROOT,sha,protect
def matrix(p):return np.loadtxt(p,skiprows=1,ndmin=2)
def values(p):
    out={}
    for line in p.read_text().splitlines():
        s=line.split();out[s[0]+s[1]]=np.array([float(x) for x in s[2:]])
    return out
def motion(a,b):
    keys=[k for k in a if k.startswith('x')]
    pa=np.array([a[k].reshape(3,4)[:,3] for k in keys])
    pb=np.array([b[k].reshape(3,4)[:,3] for k in keys])
    ca,cb=pa.mean(axis=0),pb.mean(axis=0)
    u,s,vt=np.linalg.svd((pa-ca).T@(pb-cb));D=np.eye(3);D[2,2]=np.linalg.det(vt.T@u.T)
    R=vt.T@D@u.T;t=cb-R@ca
    raw=np.linalg.norm(pa-pb,axis=1);res=np.linalg.norm(pa@R.T+t-pb,axis=1)
    return dict(raw_position_rms_m=float(np.sqrt(np.mean(raw**2))),raw_position_max_m=float(raw.max()),
      position_best_rigid_fit_residual_rms_m=float(np.sqrt(np.mean(res**2))),
      position_best_rigid_fit_residual_max_m=float(res.max()),
      common_translation_m=float(np.linalg.norm(t)),common_rotation_rad=float(np.arccos(np.clip((np.trace(R)-1)/2,-1,1))))
def main():
    root=ROOT/'linear_audit';snap=ROOT/'diagnostics'/'snapshot'
    A=matrix(root/'A');sv=np.linalg.svd(A,compute_uv=False)
    norm=np.linalg.norm(A,axis=0);assert (norm>0).all()
    scaled=np.linalg.svd(A/norm,compute_uv=False)
    prior=matrix(snap/'0.A');prior_sv=np.linalg.svd(prior,compute_uv=False)
    trials={r['label']:r for r in csv.DictReader((ROOT/'diagnostics'/'trials.csv').open())}
    checks=list(csv.DictReader((root/'linear_checks.csv').open()))
    for r in checks:
        original=trials['initial' if r['seed']=='0' else 'trial_'+r['seed']]
        assert float(r['cost'])==float(original['cost'])
        assert float(r['gradient'])==float(original['gradient'])
    result=dict(rows=A.shape[0],columns=A.shape[1],singular_max=float(sv[0]),singular_min=float(sv[-1]),
      full_column_rank=bool(sv[-1]>sv[0]*max(A.shape)*np.finfo(float).eps),
      condition_jacobian=float(sv[0]/sv[-1]),condition_normal_equations=float((sv[0]/sv[-1])**2),
      column_normalized_condition=float(scaled[0]/scaled[-1]),
      marginal_A_max=float(abs(prior).max()),marginal_condition=float(prior_sv[0]/prior_sv[-1]),
      reconstructed_objective_exact_cost_gradient_seeds=[int(r['seed']) for r in checks],
      fresh_qr_vs_recorded_step_local_max=float(max(float(r['max_local_difference_to_recorded_next']) for r in checks)),
      initial_to_last=motion(values(snap/'initial.values'),values(snap/'last_accepted.values')),
      initial_to_independent_lm=motion(values(snap/'initial.values'),values(ROOT/'diagnostics'/'lm_from_initial.values')),
      inputs={str(p):sha(p) for p in (root/'A',root/'b',root/'delta',snap/'0.A')},
      protected_unchanged=json.loads((ROOT/'protected.json').read_text())==protect())
    with (ROOT/'linear_analysis.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
