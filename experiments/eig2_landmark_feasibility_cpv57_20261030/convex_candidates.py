"""Further candidate search in homogeneous, fixed-camera cone inequalities.

Unlike a rational minimax fit, these fixed-threshold inequalities are concave
constraints in the landmark position. Still accept ONLY an independently checked
point; solver failure is unresolved, not evidence of infeasibility.
"""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize

H=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('cp57',H/'analyze.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)


def solve(g,seed):
    threshold=3.-1e-5; radius=10.-1e-5
    def c(q):
        n=np.einsum('nij,j->ni',g.N,q)+g.no
        z=g.P[:,2]@q+g.offset[:,2]
        return np.r_[threshold*z-np.linalg.norm(n,axis=1),
                     z-1e-8,radius**2-np.sum((q-g.t)**2,axis=1)]
    def j(q):
        n=np.einsum('nij,j->ni',g.N,q)+g.no
        u=n/np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-20)
        return np.vstack((threshold*g.P[:,2]-np.einsum('ni,nij->nj',u,g.N),
                          g.P[:,2],-2*(q-g.t)))
    r=minimize(lambda q:.5*np.dot(q,q),seed,jac=lambda q:q,
               method='SLSQP',bounds=[(-10,10)]*3,
               constraints=[dict(type='ineq',fun=c,jac=j)],
               options={'ftol':1e-12,'maxiter':1000})
    return dict(**g.metrics(r.x),q=r.x.tolist(),success=bool(r.success),
                message=str(r.message),min_constraint=float(min(c(r.x))),
                inner_pixel_limit=threshold,inner_distance_limit=radius)


def main():
    pins={str(p):m.sha(p) for p in [m.GEOMETRY,H/'attempt_inner1/tracks.jsonl',H/'boundary_analysis.json',Path(__file__)]}
    rows=[json.loads(l) for l in (H/'attempt_inner1/tracks.jsonl').open()]
    gs={(r['phase'],r['slot'],r['transform']):r for line in m.GEOMETRY.open() if (r:=json.loads(line))}
    output=[]
    for r in rows:
        if r['feasibility']['outcome']!='unresolved':continue
        g=m.Geometry(gs[(r['phase'],r['slot'],r['transform'])])
        ray=g.R[0]@np.linalg.solve(g.K[0],np.r_[g.z[0],1.]);ray/=np.linalg.norm(ray)
        candidates=[solve(g,ray*d) for d in [1.,5.,9.]]
        valid=[c for c in candidates if c['passes_geometric_gates']]
        best=min(valid or candidates,key=lambda c:c['max_px'])
        output.append(dict(slot=r['slot'],phase=r['phase'],transform=r['transform'],
                           outcome='feasible' if valid else 'unresolved',best=best,
                           candidates=candidates))
    for p,h in pins.items():assert m.sha(p)==h
    m.save('convex_candidates.json',dict(completed=True,inputs=pins,results=output))
    for r in output:
        if r['transform']=='identity':print(r['phase'],r['slot'],r['outcome'],r['best']['max_px'],r['best']['max_distance_m'])


if __name__=='__main__':main()
