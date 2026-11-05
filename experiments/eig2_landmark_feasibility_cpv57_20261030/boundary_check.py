"""Supplement CP57: close unresolved geometric feasibility, not a graph fix.

Minimax is a diagnostic existence search, not a proposed replacement objective.
Distance/depth constraints are retained. Candidate search uses a strictly inner
distance bound to avoid accepting a roundoff violation of the native10m guard.
"""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize

H = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('cp57', H/'analyze.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def optimize(g, q, minimax):
    radius = 10.-1e-6
    distance = np.linalg.norm(q-g.t, axis=1).max()
    if distance>radius:
        q = q * (radius*.9/max(np.linalg.norm(q), radius))
    if minimax:
        seed = np.r_[q, max(3., g.metrics(q)['max_px'])+1]
        def fun(x): return x[3]
        def jac(x): return np.array([0., 0., 0., 1.])
    else:
        seed = q
        def fun(x):
            e, _, _ = g.project(x)
            return .5*np.sum((e*g.weight/3)**2)
        def jac(x):
            e, j, _ = g.project(x)
            return np.einsum('ni,nij->j', e*g.weight**2/9, j)
    def constraints(x):
        e, _, depth = g.project(x[:3])
        dist = radius**2-np.sum((x[:3]-g.t)**2, axis=1)
        rest = np.r_[depth-1e-8, dist]
        return np.r_[x[3]**2-np.sum(e**2,axis=1),rest] if minimax else rest
    def constraint_jac(x):
        e, j, depth = g.project(x[:3])
        rest = np.vstack((g.P[:,2], -2*(x[:3]-g.t)))
        if not minimax: return rest
        error = np.column_stack((-2*np.einsum('ni,nij->nj',e,j), np.full(len(e),2*x[3])))
        return np.vstack((error,np.column_stack((rest,np.zeros(len(rest))))))
    result = minimize(fun, seed, jac=jac, method='SLSQP',
                      bounds=[(-10,10)]*3+([(0,None)] if minimax else []),
                      constraints=[dict(type='ineq',fun=constraints,jac=constraint_jac)],
                      options={'maxiter':1000,'ftol':1e-12})
    metrics = g.metrics(result.x[:3])
    return dict(**metrics, q=result.x[:3].tolist(), success=bool(result.success),
                message=str(result.message), iterations=int(result.nit),
                min_constraint=float(np.min(constraints(result.x))),
                objective='maximum per-camera reprojection' if minimax else 'native smart L2',
                candidate_search_radius_m=radius)


def main():
    before = {str(p):m.sha(p) for p in [m.GEOMETRY,H/'attempt_inner1/tracks.jsonl',Path(__file__)]}
    rows = [json.loads(l) for l in (H/'attempt_inner1/tracks.jsonl').open()]
    geometry = {(r['phase'],r['slot'],r['transform']):r for line in m.GEOMETRY.open() if (r:=json.loads(line))}
    # Check constrained optimizer on a known valid saved starting geometry first.
    fixture = next(r for r in rows if r['slot']==669 and r['phase']=='ordinary_theta' and r['transform']=='identity')
    g = m.Geometry(geometry[(fixture['phase'],fixture['slot'],'identity')])
    f = optimize(g,np.array(fixture['smart_L2']['best']['q']),True)
    assert f['passes_geometric_gates'] and f['max_px']<=fixture['smart_L2']['best']['max_px']+1e-6
    results=[]
    for r in rows:
        if r['transform']!='identity' or r['phase']=='ordinary_theta': continue
        if r['warm_status']==0:continue
        g = m.Geometry(geometry[(r['phase'],r['slot'],'identity')])
        seed = np.array(r['smart_L2']['best']['q'])
        if r['feasibility'].get('candidate'):
            seed = np.array(r['feasibility']['candidate']['point_world'])-g.origin
        dlt = np.array(r['native']['point_world'])-g.origin
        candidates=[optimize(g, s, True) for s in [seed,dlt]]
        good = [c for c in candidates if c['min_depth_m']>0 and c['max_distance_m']<=10]
        best = min(good or candidates,key=lambda c:c['max_px'])
        fit = optimize(g,np.array(best['q']),False)
        if best['passes_geometric_gates']:
            assert r['feasibility']['outcome']!='infeasible'
            outcome='feasible'
        else: outcome=r['feasibility']['outcome']
        result=dict(slot=r['slot'],phase=r['phase'],native_status=r['native_status'],
                    original_outcome=r['feasibility']['outcome'],outcome=outcome,
                    minimax_candidates=candidates,best=best,bounded_L2=fit,
                    rank=r['rank'],in_original_27=r['in_original_27'])
        results.append(result)
        print(r['phase'],r['slot'],r['feasibility']['outcome'],'->',outcome,
              'max_px',round(best['max_px'],7),'distance',round(best['max_distance_m'],7),flush=True)
    for p,h in before.items():assert m.sha(p)==h
    m.save('boundary_analysis.json',dict(completed=True,inputs=before,fixture=f,results=results,
             graph_optimizer_calls=0,qualification='Verified feasible candidates or prior dual certificates; minimax is not native L2 nor a production change.'))


if __name__=='__main__':main()
