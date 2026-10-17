"""Score frozen FL4f2 native/as-played/finalized paths and received covariances.

Run after the standalone ON case validates, inside the existing ROS Noetic container.
IE-TC is a shared-CPT7 GNSS/INS reference, not independent prism ground truth.
"""
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'evaluation_full'
TOOLS = Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
sys.path.insert(0, str(TOOLS))
import score_grandtour_prism as prism
import score_grandtour_relative_pose as rel

GT_BAG = Path('/workspace/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag')
GT_TOPIC = '/boxi/inertial_explorer/tc/odometry'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()

def stat(values):
    a = np.asarray(values, float)
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))

def prism_pose(pose):
    p, R = pose
    fixed_R = rel.quaternion_to_matrix(rel.CPT7_TO_PRISM_Q)
    return p + R @ rel.CPT7_TO_PRISM_T, R @ fixed_R

def path_length(positions):
    return float(np.linalg.norm(np.diff(np.asarray(positions), axis=0), axis=1).sum())

def score(curve, reference, times):
    est = [prism_pose(rel.interpolate_pose(*curve, t, .5)) for t in times]
    gt = [prism_pose(rel.interpolate_pose(*reference, t, .05)) for t in times]
    E = np.asarray([p for p, _ in est]); G = np.asarray([p for p, _ in gt])
    A, b = prism.rigid_kabsch(E, G)
    errors = np.linalg.norm(E @ A.T + b - G, axis=1)
    terr, rerr = [], []
    for i in range(len(times)-10):
        if abs(times[i+10]-times[i]-1.0) > 1e-4:
            continue
        et, er = rel.relative_pose(est[i], est[i+10])
        gt_t, gt_r = rel.relative_pose(gt[i], gt[i+10])
        terr.append(np.linalg.norm(et-gt_t))
        dR = gt_r.T @ er
        rerr.append(math.degrees(math.acos(float(np.clip((np.trace(dR)-1)/2, -1, 1)))))
    assert len(terr) >= 10
    return dict(rows=len(times), start_s=float(times[0]), end_s=float(times[-1]),
                ATE_RMSE_m=float(np.sqrt(np.mean(errors**2))),
                translation_RPE_1s_RMSE_m=float(np.sqrt(np.mean(np.square(terr)))),
                rotation_RPE_1s_RMSE_deg=float(np.sqrt(np.mean(np.square(rerr)))),
                path_length_prism_m=path_length(E), reference_path_length_prism_m=path_length(G),
                error_m=stat(errors), rpe_pairs=len(terr))

def score_saved(curve, reference):
    times=curve[0]
    est=[prism_pose((p,rel.quaternion_to_matrix(q))) for p,q in zip(curve[1],curve[2])]
    gt_raw=[rel.interpolate_pose(*reference,t,.05) for t in times]
    assert all(p is not None for p in gt_raw), 'Some saved output rows lack IE-TC brackets'
    gt=[prism_pose(p) for p in gt_raw]
    E=np.asarray([p for p,_ in est]); G=np.asarray([p for p,_ in gt])
    A,b=prism.rigid_kabsch(E,G)
    errors=np.linalg.norm(E@A.T+b-G,axis=1)
    grid=np.arange(math.ceil(times[0]*10)/10,math.floor((times[-1]-1)*10)/10+1e-8,.1)
    terr=[];rerr=[]
    for t in grid:
        e0=rel.interpolate_pose(*curve,t,.5);e1=rel.interpolate_pose(*curve,t+1,.5)
        g0=rel.interpolate_pose(*reference,t,.05);g1=rel.interpolate_pose(*reference,t+1,.05)
        if any(x is None for x in (e0,e1,g0,g1)):
            continue
        et,er=rel.relative_pose(prism_pose(e0),prism_pose(e1))
        gt_t,gt_r=rel.relative_pose(prism_pose(g0),prism_pose(g1))
        terr.append(np.linalg.norm(et-gt_t))
        dR=gt_r.T@er
        rerr.append(math.degrees(math.acos(float(np.clip((np.trace(dR)-1)/2,-1,1)))))
    assert len(terr)>=10
    return dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),
        span_s=float(times[-1]-times[0]),
        ATE_RMSE_m=float(np.sqrt(np.mean(errors**2))),
        translation_RPE_1s_RMSE_m=float(np.sqrt(np.mean(np.square(terr)))),
        rotation_RPE_1s_RMSE_deg=float(np.sqrt(np.mean(np.square(rerr)))),
        path_length_prism_m=path_length(E),reference_path_length_prism_m=path_length(G),
        rpe_pairs=len(terr),error_m=stat(errors))

def covariances(mode):
    path = ROOT / f'live120_{mode}_receiver/events.csv'
    groups = collections.defaultdict(list)
    versions = collections.defaultdict(list)
    with path.open(newline='') as f:
        for row in csv.reader(f):
            if row[0] != 'B':
                continue
            assert len(row) == 52
            source = row[2]
            C = np.asarray([float(x) for x in row[16:]], float).reshape(6, 6)
            assert np.isfinite(C).all()
            sym = float(np.max(np.abs(C-C.T)))
            eig = np.linalg.eigvalsh((C+C.T)/2)
            groups[source].append((float(np.trace(C[:3,:3])),
                                   float(np.trace(C[3:,3:])), float(eig.min()),
                                   float(eig.max()), sym))
            versions[(source,row[3],row[6],row[7])].append(C)
    result = {}
    for source, values in groups.items():
        a = np.asarray(values)
        first_last = [(float(np.trace(v[0][:3,:3])), float(np.trace(v[-1][:3,:3])),
                       float(np.trace(v[0][3:,3:])), float(np.trace(v[-1][3:,3:])))
                      for key,v in versions.items() if key[0] == source and len(v)>1]
        result[source] = dict(versions=len(a), stable_ids=sum(k[0]==source for k in versions),
            revised_ids=len(first_last), rotation_trace_rad2=stat(a[:,0]),
            translation_trace_m2=stat(a[:,1]), min_eigenvalue=float(a[:,2].min()),
            max_eigenvalue=float(a[:,3].max()), max_symmetry_error=float(a[:,4].max()),
            first_last_rotation_trace_ratio=stat([x[1]/x[0] for x in first_last if x[0]>0]) if first_last else None,
            first_last_translation_trace_ratio=stat([x[3]/x[2] for x in first_last if x[2]>0]) if first_last else None)
    return result

def main():
    assert not OUT.exists(), 'Refusing to overwrite an evaluation'
    for mode in ('on',):
        assert (ROOT/f'live120_{mode}_control/complete.json').is_file()
        assert (ROOT/f'live120_{mode}_source/complete.marker').is_file()
        assert (ROOT/f'live120_{mode}_receiver/complete.marker').is_file()
    paths = {}
    for mode in ('on',):
        src = ROOT / f'live120_{mode}_source'
        rec = ROOT / f'live120_{mode}_receiver/receiver'
        paths[f'{mode}_native_glim'] = src/'odom_imu.txt'
        paths[f'{mode}_native_kimera'] = src/'kimera_odometry.csv'
        paths[f'{mode}_fused_online'] = rec/'online_trajectory.tum'
        paths[f'{mode}_fused_finalized'] = rec/'finalized_trajectory.tum'
    curves={}
    for name, path in paths.items():
        loader = rel.load_odometry_csv if name.endswith('native_kimera') else rel.load_tum
        curves[name] = rel.sort_unique(*loader(path))
        assert len(curves[name][0]) > 100
    t,p,q,frames,children = rel.load_reference_bag(GT_BAG,GT_TOPIC)
    reference = rel.sort_unique(t,p,q)
    assert children == ['cpt7_imu']
    start=max(c[0][0] for c in [*curves.values(),reference])
    end=min(c[0][-1] for c in [*curves.values(),reference])
    grid=np.arange(math.ceil(start*10)/10, math.floor(end*10)/10+1e-8,.1)
    times=np.asarray([stamp for stamp in grid if
        rel.interpolate_pose(*reference,stamp,.05) is not None and
        all(rel.interpolate_pose(*c,stamp,.5) is not None for c in curves.values())])
    assert len(times)>1100 and max(np.diff(times))<=.100001
    results={}
    for name,curve in curves.items():
        results[name]=dict(path=str(paths[name]),sha256=sha(paths[name]),
            full_saved_rows=len(curve[0]),full_saved_span_s=float(curve[0][-1]-curve[0][0]),
            full_saved_path_length_CPT7_m=path_length(curve[1]),
            full_saved=score_saved(curve,reference),
            common=score(curve,reference,times))
    report=dict(reference=dict(path=str(GT_BAG),sha256=sha(GT_BAG),topic=GT_TOPIC,
            frame_ids=frames,child_frame_ids=children,
            caveat='IE-TC GNSS/INS reference shares CPT7 IMU; not independent prism GT'),
        common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),
            timestamps_sha256=hashlib.sha256(times.astype('<f8').tobytes()).hexdigest(),
            interpolation_gap_estimate_s=.5,interpolation_gap_reference_s=.05),
        convention='Per-path rigid SE(3) no-scale alignment for ATE at prism lever arm; 1s RPE in prism frame, no global alignment',
        results=results,covariance={mode:covariances(mode) for mode in ('on',)})
    OUT.mkdir(exist_ok=False)
    with (OUT/'summary.json').open('x') as f:
        json.dump(report,f,indent=2,allow_nan=False)
    print(json.dumps({k:{'full_saved':v['full_saved'],'common':v['common']} for k,v in results.items()},indent=2))

if __name__=='__main__':
    main()
