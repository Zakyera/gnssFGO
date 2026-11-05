"""CP57: fixed-camera landmark fits and independently checked feasibility bounds.

Never loads or modifies the graph optimizer. All input artifacts are read-only.
"""
import collections
import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import scipy
from scipy.optimize import least_squares, linprog, minimize

H = Path(__file__).resolve().parent
OLD = H.parent / 'eig2_visual_support_cpv56_20261030'
K = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs') / OLD.name
GEOMETRY = K / 'gauge_decomposition/results/gauge.jsonl'
ACTIVITY = K / 'measure_gate_failure/results/factors.jsonl'


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def save(name, data):
    with (H / name).open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False)


class Geometry:
    def __init__(self, row):
        self.row = row
        self.R, self.t, self.K, self.z, self.weight = [], [], [], [], []
        for c in row['cameras']:
            pose = np.asarray(c['pose']).reshape(4, 4)
            stereo = c['z'][1] is not None
            for side, u in enumerate(c['z'][:2]):
                if u is None:
                    continue
                R = pose[:3, :3]
                self.R.append(R)
                self.t.append(pose[:3, 3] + R @ [side*c['baseline'], 0, 0])
                self.K.append(np.asarray(c['K']).reshape(3, 3))
                self.z.append([u, c['z'][2]])
                # Stereo smart-factor cost has ONE vertical residual, not two.
                self.weight.append([1., 1/np.sqrt(2) if stereo else 1.])
        self.R, self.t, self.K, self.z, self.weight = map(
            np.asarray, (self.R, self.t, self.K, self.z, self.weight))
        self.origin = self.t[0].copy()
        self.t = self.t - self.origin
        self.Rt = self.R.transpose(0, 2, 1)
        self.P = np.einsum('nij,njk->nik', self.K, self.Rt)
        self.offset = -np.einsum('nij,nj->ni', self.P, self.t)
        self.N = self.P[:, :2] - self.z[:, :, None] * self.P[:, 2:3]
        self.no = self.offset[:, :2] - self.z * self.offset[:, 2:3]

    def project(self, q):
        pix = np.einsum('nij,j->ni', self.P, q) + self.offset
        err = pix[:, :2]/pix[:, 2:3] - self.z
        jac = (self.P[:, :2]*pix[:, 2, None, None]
               - pix[:, :2, None]*self.P[:, 2:3]) / pix[:, 2, None, None]**2
        return err, jac, pix[:, 2]

    def metrics(self, q):
        e, j, depth = self.project(q)
        norms = np.linalg.norm(e, axis=1)
        distance = np.linalg.norm(q-self.t, axis=1)
        return dict(point_world=(q+self.origin).tolist(),
                    max_px=float(max(norms)), min_depth_m=float(min(depth)),
                    max_distance_m=float(max(distance)),
                    smart_cost=float(np.sum((e*self.weight/3)**2)/2),
                    mono_cost=float(np.sum((e/3)**2)/2),
                    passes_geometric_gates=bool(max(norms)<=3 and min(depth)>0
                                               and max(distance)<=10))

    def dlt(self):
        # Preserve NATIVE WORLD ORIGIN for native DLT/rank parity; q only for fits.
        A = self.N.reshape(-1, 3)
        b = self.no.reshape(-1) - A @ self.origin
        _, s, vt = np.linalg.svd(np.column_stack((A, b)), full_matrices=True)
        rank = int(np.sum(s > self.row['rank_tolerance']))
        point = vt[-1, :3]/vt[-1, 3]
        m = self.metrics(point-self.origin)
        status = (1 if rank<3 else 2 if m['min_depth_m']<=0 else
                  4 if m['max_distance_m']>10 else 3 if m['max_px']>3 else 0)
        return point-self.origin, rank, status, m, s.tolist()

    def linear_constraints(self, threshold, angles=32):
        # ||N_i q + n_i|| <= threshold * depth_i, depth_i >= 0.
        # Each unit direction gives a NECESSARY linear halfspace, not sufficient.
        theta = np.arange(angles)*2*np.pi/angles
        direction = np.column_stack((np.cos(theta), np.sin(theta)))
        A = np.einsum('dj,njk->ndk', direction, self.N)
        A -= threshold*self.P[:, None, 2]
        b = threshold*self.offset[:, None, 2] - self.no@direction.T
        A = list(A.reshape(-1, 3)); b = list(b.ravel())
        A.extend(-self.P[:, 2]); b.extend(self.offset[:, 2])
        # Native distance <=10 from first camera implies these necessary bounds.
        A.extend(np.eye(3)); b.extend([10.]*3)
        A.extend(-np.eye(3)); b.extend([10.]*3)
        return np.array(A), np.array(b)

    def certificate(self, A, b):
        n = len(b)
        result = linprog(b, A_eq=np.vstack((A.T, np.ones(n))),
                         b_eq=[0., 0., 0., 1.], bounds=(0, None), method='highs')
        if not result.success:
            return None
        y = np.maximum(result.x, 0.)
        # Independently verify contradiction including finite equality residual:
        # q is in [-10,10]^3, so y*A*q >= -10*||y*A||_1.
        residual = y@A
        bound = float(y@b)
        margin = -bound - 10*float(np.sum(np.abs(residual)))
        if margin <= 1e-8:
            return None
        ids = np.flatnonzero(y>0)
        return dict(numerical_infeasibility_certificate=True,
                    margin=margin, weighted_rhs=bound,
                    equality_residual=residual.tolist(),
                    nonnegative_weights=y[ids].tolist(),
                    constraint_indices=ids.tolist(),
                    constraint_rows=A[ids].tolist(), constraint_rhs=b[ids].tolist(),
                    qualification='Verified floating-point contradiction, not interval/exact arithmetic.')

    def feasible(self, threshold=3.):
        A, b = self.linear_constraints(threshold)
        for iteration in range(60):
            lp = linprog(np.zeros(3), A_ub=A, b_ub=b,
                         bounds=[(None, None)]*3, method='highs',
                         options={'primal_feasibility_tolerance':1e-9,
                                  'dual_feasibility_tolerance':1e-9})
            if not lp.success:
                cert = self.certificate(A, b) if lp.status==2 else None
                return dict(outcome='infeasible' if cert else 'unresolved',
                            iterations=iteration+1, certificate=cert,
                            solver_status=int(lp.status))
            q = lp.x
            e, _, z = self.project(q)
            norm = np.linalg.norm(e, axis=1)
            if np.all(z>1e-9) and np.max(norm)<=threshold+1e-9:
                m = self.metrics(q)
                if m['passes_geometric_gates']:
                    return dict(outcome='feasible', iterations=iteration+1,
                                candidate=m, q=q.tolist())
            # Add necessary supporting planes along violations. Distance-ball
            # tangents are also necessary and retain the original 10m limit.
            added = False
            for i in range(len(z)):
                num = self.N[i]@q + self.no[i]
                n = np.linalg.norm(num)
                if n > threshold*z[i]+1e-11 and n>0:
                    u = num/n
                    A = np.vstack((A, u@self.N[i]-threshold*self.P[i, 2]))
                    b = np.append(b, threshold*self.offset[i, 2]-u@self.no[i])
                    added = True
                d = q-self.t[i]; nd = np.linalg.norm(d)
                if nd>10+1e-10:
                    A = np.vstack((A, d/nd))
                    b = np.append(b, 10+np.dot(d/nd, self.t[i]))
                    added = True
            if not added:
                return dict(outcome='unresolved', iterations=iteration+1,
                            candidate=self.metrics(q))
        return dict(outcome='unresolved', iterations=60)

    def fit(self, starts, mono=False):
        w = np.ones_like(self.weight) if mono else self.weight
        candidates = []
        for seed in starts:
            result = least_squares(lambda q:(self.project(q)[0]*w/3).ravel(), seed,
                                   jac=lambda q:(self.project(q)[1]*w[:, :, None]/3).reshape(-1, 3),
                                   ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=300)
            m = self.metrics(result.x)
            candidates.append(dict(**m, q=result.x.tolist(), success=bool(result.success),
                                   nfev=int(result.nfev), optimality=float(result.optimality)))
        positive = [c for c in candidates if c['min_depth_m']>0]
        best = min(positive or candidates, key=lambda c:c['mono_cost' if mono else 'smart_cost'])
        return dict(best=best, starts=candidates)


def selftest():
    K = np.array([[400., 0, 320], [0, 400, 240], [0, 0, 1]])
    pt = np.array([.4, .2, 4.])
    cameras = []
    for x in [0., .15, .3]:
        pose = np.eye(4); pose[0, 3]=x
        q = pt-pose[:3, 3]
        cameras.append(dict(pose=pose.ravel().tolist(), K=K.ravel().tolist(),
                            baseline=.1, z=[400*q[0]/q[2]+320,
                                            400*(q[0]-.1)/q[2]+320,400*q[1]/q[2]+240]))
    row = dict(cameras=cameras, rank_tolerance=1.)
    g = Geometry(row); seed, rank, status, _, _ = g.dlt()
    assert status==0 and np.linalg.norm(seed-pt)<1e-10
    assert g.feasible()['outcome']=='feasible'
    q = pt+[.1, -.04, .3]
    numeric = np.column_stack([(g.project(q+np.eye(3)[i]*1e-6)[0].ravel()
                               -g.project(q-np.eye(3)[i]*1e-6)[0].ravel())/2e-6 for i in range(3)])
    jac_error = float(np.max(np.abs(numeric-g.project(q)[1].reshape(-1, 3))))
    assert jac_error<1e-6
    fit = g.fit([q, seed])['best']; assert fit['max_px']<1e-8
    # Contradictory observations from identical cameras cannot fit within 3px.
    a = dict(cameras[0]); a['z']=[320., None, 240.]
    b = dict(a); b['z']=[340., None, 240.]
    bad = Geometry(dict(cameras=[a, b], rank_tolerance=1.))
    # Zero camera depth would satisfy homogeneous pixel inequalities; add the
    # physically nonzero stereo baseline so no all-zero-depth escape is possible.
    a['z']=[320., 310., 240.]; b['z']=[340., 330., 240.]
    bad = Geometry(dict(cameras=[a, b], rank_tolerance=1.))
    # Here zero depth remains a relaxed feasible point; do NOT claim impossible
    # from solver failure. Use a third translated camera to eliminate this case.
    c = dict(cameras[1]); c['z']=[320., 310., 250.]
    bad = Geometry(dict(cameras=[a, b, c], rank_tolerance=1.))
    negative = bad.feasible(); assert negative['outcome']=='infeasible', negative
    # Identical left-only path with measurable parallax, missing-right support.
    mono = json.loads(json.dumps(row))
    for c in mono['cameras']: c['z'][1]=None
    gm = Geometry(mono); assert gm.fit([pt+[.1, 0, .1]])['best']['max_px']<1e-7
    # Translation invariance of projection/objective at corresponding points.
    shifted = json.loads(json.dumps(row)); shift=np.array([3., -2., 1.])
    for c in shifted['cameras']:
        p = np.array(c['pose']).reshape(4, 4); p[:3, 3]+=shift; c['pose']=p.ravel().tolist()
    gt = Geometry(shifted)
    assert np.max(np.abs(g.project(q)[0]-gt.project(q)[0]))<1e-10
    assert abs(g.fit([q])['best']['smart_cost']-gt.fit([q])['best']['smart_cost'])<1e-12
    return dict(passed=True, point_jacobian_max_error=jac_error,
                exact_stereo=True, missing_right=True, contradictory_tracks=negative,
                unchanged_sigma=3, translation_invariance=True)


def main():
    started = time.time()
    manifest = json.loads((OLD/'retained_manifest.json').read_text())
    selected_inputs = [GEOMETRY, ACTIVITY, OLD/'analysis.json', OLD/'independent_dlt.json',
                       OLD/'REPORT.txt', H/'PROTOCOL.txt', Path(__file__)]
    pins = {str(p):sha(p) for p in selected_inputs}
    for p in selected_inputs:
        if str(p) in manifest: assert pins[str(p)]==manifest[str(p)], str(p)
    save('inputs.json', pins)
    save('start.json', dict(pid=os.getpid(), time=datetime.datetime.now().astimezone().isoformat(),
                            numpy=np.__version__, scipy=scipy.__version__, executable=sys.executable))
    fixtures = selftest(); save('selftest.json', fixtures)
    print('PASS fixed-camera projection, fit, missing-right, infeasibility-certificate and translation fixtures', flush=True)
    geometry = {(r['phase'], r['slot'], r['transform']):r for line in GEOMETRY.open() if (r:=json.loads(line))}
    activity = {(r['label'], r['slot']):r for line in ACTIVITY.open() if (r:=json.loads(line))}
    initial = {s:r for (label, s), r in activity.items() if label=='on_ordinary_theta'}
    affected = sorted(s for s, r in initial.items() if r['warm_status']==0 and any(
        activity[('on_'+phase, s)]['warm_status']!=0 for phase in ['ordinary_estimate','refresh_estimate']))
    expected27 = {r['slot'] for r in json.loads((OLD/'analysis.json').read_text())['initially_valid_final_outliers']}
    assert len(expected27)==27 and expected27.issubset(affected)
    selections = sorted(set(affected)|{571})
    records = []
    with (H/'tracks.jsonl').open('x') as out:
        for si, slot in enumerate(selections):
            for phase in ['ordinary_theta', 'ordinary_estimate', 'refresh_estimate']:
                for transform in ['identity', 'translation', 'rotation']:
                    row = geometry[(phase, slot, transform)]
                    g = Geometry(row)
                    seed, rank, status, native, singular = g.dlt()
                    assert status==row['status'], (slot, phase, transform, status, row['status'])
                    if status in [0,3,4]:
                        assert np.linalg.norm(np.array(native['point_world'])-row['native_point'])<1e-6
                    # Equivalent seed set in each representation, not native DLT
                    # alone; fixed-pose L2 objective can have problematic starts.
                    inh = np.linalg.lstsq(g.N.reshape(-1,3), -g.no.ravel(), rcond=None)[0]
                    ray = g.R[0]@np.linalg.solve(g.K[0], np.r_[g.z[0],1.]); ray/=np.linalg.norm(ray)
                    starts = [seed, inh, ray*3, ray*7]
                    fit = g.fit(starts)
                    mono_fit = g.fit([np.array(fit['best']['q']), seed], mono=True)
                    feasibility = g.feasible()
                    # LP can find a boundary point slightly >3px due to floating
                    # point; a tighter INNER probe supplies a safe candidate only.
                    if feasibility['outcome']=='unresolved':
                        inner = g.feasible(3.-1e-6)
                        if inner['outcome']=='feasible':
                            feasibility=dict(**inner, inner_probe_px=3.-1e-6)
                    if fit['best']['passes_geometric_gates']:
                        assert feasibility['outcome']!='infeasible', (slot, phase, transform)
                        if feasibility['outcome']!='feasible':
                            feasibility=dict(outcome='feasible', candidate=fit['best'], via='L2 candidate')
                    function_key = hashlib.sha256(json.dumps(row['cameras'],sort_keys=True).encode()).hexdigest()
                    record = dict(slot=slot, phase=phase, transform=transform,
                                  initial_warm_status=initial[slot]['warm_status'],
                                  warm_status=activity[('on_'+phase,slot)]['warm_status'] if transform=='identity' else None,
                                  in_original_27=slot in expected27, rank=rank, singular_values=singular,
                                  native_status=status, native=native, smart_L2=fit,
                                  mono_L2=mono_fit, feasibility=feasibility,
                                  geometry_hash=function_key,
                                  stereo=all(c['z'][1] is not None for c in row['cameras']),
                                  keys=initial[slot]['keys'])
                    records.append(record); out.write(json.dumps(record, allow_nan=False)+'\n'); out.flush()
            print(f'track-factor {si+1}/{len(selections)} slot={slot} complete', flush=True)
    summaries = {}
    for phase in ['ordinary_theta','ordinary_estimate','refresh_estimate']:
        rr = [r for r in records if r['phase']==phase and r['transform']=='identity'
              and r['initial_warm_status']==0 and r['warm_status']!=0]
        summaries[phase]=dict(invalidated_instances=len(rr), distinct_geometries=len({r['geometry_hash'] for r in rr}),
            native_status_counts=dict(collections.Counter(r['native_status'] for r in rr)),
            reprojection_rejected=sum(r['native_status']==3 for r in rr),
            L2_geometric_pass=sum(r['smart_L2']['best']['passes_geometric_gates'] for r in rr),
            L2_native_rank_and_geometric_pass=sum(r['rank']>=3 and r['smart_L2']['best']['passes_geometric_gates'] for r in rr),
            feasibility_counts=dict(collections.Counter(r['feasibility']['outcome'] for r in rr)))
    for p, h in pins.items(): assert sha(p)==h, p
    result = dict(completed=True, affected_factor_instances=affected, tested_slots=selections,
                  records=len(records), summaries=summaries, inputs_unchanged=True,
                  graph_optimizer_calls=0, source_replays=0, production_changes=0,
                  landmark_only_L2_calls=sum(len(r['smart_L2']['starts'])+len(r['mono_L2']['starts']) for r in records),
                  elapsed_seconds=time.time()-started)
    save('analysis.json', result); print(json.dumps(result, indent=2), flush=True)


if __name__=='__main__':
    try:
        main()
    except Exception as exc:
        if not (H/'failure.json').exists():
            save('failure.json', dict(type=type(exc).__name__, error=str(exc)))
        raise
