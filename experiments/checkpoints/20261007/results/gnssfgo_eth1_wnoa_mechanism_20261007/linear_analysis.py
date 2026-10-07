"""Read-only fixed-linearization curvature and precision sensitivities.
Adapted from the validated September29 exporter analysis to 12D X/V/W state.
No resulting step is applied to a nonlinear graph.
"""
from pathlib import Path
import csv,json
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import splu
from scipy.spatial.transform import Rotation
from prepare import ROOT
from beliefs import stat

groups={'rotation':slice(0,3),'translation':slice(3,6),'velocity':slice(6,9),'omega':slice(9,12)}
all_results={}
for req in csv.DictReader((ROOT/'requests.csv').open()):
    mode,label=req['mode'],req['snapshot'];prefix=ROOT/'interaction'/f'{mode}_{label}'
    factors=list(csv.DictReader(Path(str(prefix)+'_factors.csv').open()))
    r=np.fromfile(str(prefix)+'_r.bin',dtype='<f8');trip=np.fromfile(str(prefix)+'_jac.bin',dtype=[('row','<i8'),('col','<i8'),('v','<f8')])
    states=np.loadtxt(ROOT/(mode+'_wnoa')/('snapshot_'+label)/'incremental_states.csv',delimiter=',',skiprows=1,ndmin=2)
    n=len(states);A=sparse.coo_matrix((trip['v'],(trip['row'],trip['col'])),shape=(len(r),12*n)).tocsr()
    assert np.isfinite(A.data).all() and np.isfinite(r).all()
    family=np.empty(len(r),dtype=object)
    for f in factors:
        sl=slice(int(f['row']),int(f['row'])+int(f['dimension']));family[sl]=f['family']
        assert np.isclose(.5*r[sl]@r[sl],float(f['cost']),rtol=1e-10,atol=1e-10)
    H=(A.T@A).tocsc();diag=H.diagonal();assert (diag>0).all()
    scale=1/np.sqrt(diag);D=sparse.diags(scale);Hn=(D@H@D).tocsc();lu=splu(Hn)
    families=sorted(set(family));grad={k:np.asarray(A[family==k].T@r[family==k]).ravel() for k in families};total=sum(grad.values())
    step=scale*lu.solve(-scale*total);normalr=r+A@step
    positions=states[:,2:5];rot=Rotation.from_quat(states[:,5:9]).as_matrix();gauge=np.zeros((n,3,6));gauge[:,:,:3]=np.eye(3)
    for i,(x,y,z) in enumerate(positions):gauge[i,:,3:]=[[0,z,-y],[-z,0,x],[y,-x,0]]
    gauge=gauge.reshape(-1,6)
    def shapes(d):
        world=np.einsum('nij,nj->ni',rot,d.reshape(n,12)[:,3:6]);fit=np.linalg.lstsq(gauge,world.ravel(),rcond=None)[0]
        return stat(np.linalg.norm(world-(gauge@fit).reshape(n,3),axis=1))
    result={'states':n,'cost':float(.5*r@r),'fresh_normalized_gradient':float(np.max(abs(total)*scale)),
        'unapplied_GN_step_shape_m':shapes(step),'families':{},'gradient_cosines':{}}
    derivatives=[]
    for k in families:
        mask=family==k;Ak=A[mask];rk=r[mask];rhs=-np.asarray(Ak.T@normalr[mask]).ravel();sens=scale*lu.solve(scale*rhs);derivatives.append(sens)
        d=np.asarray(Ak.power(2).sum(axis=0)).ravel();family_factors=[f for f in factors if f['family']==k]
        residual=float(np.linalg.norm(Hn@(sens/scale)-scale*rhs)/max(np.linalg.norm(scale*rhs),1e-30))
        assert residual<1e-7,('normal solve',mode,label,k,residual)
        result['families'][k]={'count':len(family_factors),'cost':float(.5*rk@rk),'mean_squared_whitened_residual':float(rk@rk/len(family_factors)),
            'curvature_diagonal_sum':{name:float(d.reshape(n,12)[:,sl].sum()) for name,sl in groups.items()},
            'local_precision_sensitivity':{name:stat(np.linalg.norm(sens.reshape(n,12)[:,sl],axis=1)) for name,sl in groups.items()},
            'shape_sensitivity_m':shapes(sens),'linear_solve_relative_residual':residual}
        # Check derivatives on the final graphs with algebraic central differences.
        if label=='final':
            hs=(Ak.T@Ak).tocsc();eps=1e-4;dx=[]
            for sign in [-1,1]:
                a=np.exp(sign*eps)-1;dx.append(scale*splu((D@(H+a*hs)@D).tocsc()).solve(-scale*(total+a*grad[k])))
            fd=(dx[1]-dx[0])/(2*eps);err=float(np.linalg.norm(fd-sens)/max(np.linalg.norm(sens),1e-30))
            result['families'][k]['finite_difference_relative_error']=err
            assert err<1e-3,('sensitivity derivative',k,err)
    for i,k in enumerate(families):
        for l in families[i+1:]:
            result['gradient_cosines'][k+'__'+l]={}
            for name,sl in groups.items():
                a=grad[k].reshape(n,12)[:,sl].ravel();b=grad[l].reshape(n,12)[:,sl].ravel();den=np.linalg.norm(a)*np.linalg.norm(b)
                result['gradient_cosines'][k+'__'+l][name]=float(a@b/den) if den else None
    result['all_family_derivatives_sum_max_abs']=float(np.max(abs(sum(derivatives))))
    all_results[mode+'_'+label]=result
    (ROOT/'linear_summary.json').write_text(json.dumps(all_results,indent=2))
    print(mode,label,'cost',result['cost'],'gradient',result['fresh_normalized_gradient'],flush=True)
