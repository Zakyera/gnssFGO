"""Evaluate frozen native/fused outputs against full-window IE-TC, no estimator run.

Run inside the existing Noetic container. Refuse missing reference coverage,
input changes, and overwriting results. Reuse the retained evaluator's physical
prism lever arm and SE(3), unit-scale Kabsch alignment for both trajectories.
"""
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

WORK = Path('/workspace/cbs_gtsam4.3')
TOOLS = WORK / 'src/cbsms/tools'
sys.path.insert(0, str(TOOLS))
import score_grandtour_prism as prism
import score_grandtour_relative_pose as relative

OFF = WORK / 'runs/gnssfgo_eth1_active_window40_off_20261008'
ON = WORK / 'runs/gnssfgo_eth1_active_window40_prepared_20261008'
OUT = OFF / 'full_reference_evaluation_20261008'
REFERENCE = WORK / 'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag'
TOPIC = '/boxi/inertial_explorer/tc/odometry'
FIRST_IMU = 1727775008.003475


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def pose_at(curve, stamp, gap):
    return relative.interpolate_pose(*curve, stamp, gap)


def positions_at_prism(poses):
    return np.asarray([x + R @ prism.BOX_TO_PRISM_T for x, R in poses])


def path_length(x):
    return float(np.linalg.norm(np.diff(x, axis=0), axis=1).sum())


def score(name, kind, times, poses, reference):
    gt = [pose_at(reference, t, .05) for t in times]
    missing = [float(t) for t, p in zip(times, gt) if p is None]
    assert not missing, (name, kind, 'Missing IE-TC coverage', missing)
    x = positions_at_prism(poses)
    truth = positions_at_prism(gt)
    R, tr = prism.rigid_kabsch(x, truth)
    aligned = (R @ x.T).T + tr
    error = np.linalg.norm(aligned - truth, axis=1)
    assert np.linalg.det(R) > .999999999
    # Reference is sampled at precisely the estimator timestamps, no extrapolation.
    # Retain CPT7-frame TUM pairs for an independent evo APE cross-check too.
    from tf.transformations import quaternion_from_matrix
    def save_tum(path, values):
        rows = []
        for t, (pos, rot) in zip(times, values):
            T = np.eye(4)
            T[:3, :3] = rot
            rows.append([t, *(pos + rot @ prism.BOX_TO_PRISM_T),
                         *quaternion_from_matrix(T)])
        with path.open('x') as stream:
            np.savetxt(stream, rows, fmt='%.12f')
    save_tum(OUT / (name + '_' + kind + '_estimate_prism.tum'), poses)
    save_tum(OUT / (name + '_' + kind + '_reference_prism.tum'), gt)
    with (OUT / (name + '_' + kind + '_errors.csv')).open('x') as stream:
        w = csv.writer(stream)
        w.writerow(['timestamp', 'seconds_from_first_imu', 'aligned_x', 'aligned_y',
                    'aligned_z', 'reference_x', 'reference_y', 'reference_z', 'error_m'])
        w.writerows([[t, t-FIRST_IMU, *a, *b, e]
                     for t, a, b, e in zip(times, aligned, truth, error)])
    return dict(rows=len(times), reference_matched_rows=len(gt), reference_coverage_percent=100,
                start_timestamp=float(times[0]), end_timestamp=float(times[-1]),
                start_offset_s=float(times[0]-FIRST_IMU), end_offset_s=float(times[-1]-FIRST_IMU),
                timestamp_span_s=float(times[-1]-times[0]),
                max_pose_gap_s=float(np.diff(times).max()),
                ATE_RMSE_m=float(np.sqrt(np.mean(error**2))),
                ATE_median_m=float(np.median(error)), ATE_max_m=float(error.max()),
                full_CPT7_polyline_path_m=path_length(np.asarray([p[0] for p in poses])),
                evaluation_prism_polyline_path_m=path_length(x),
                matched_reference_prism_polyline_path_m=path_length(truth),
                alignment_rotation=R.tolist(), alignment_translation=tr.tolist(), scale=1.0,
                timestamps_sha256=hashlib.sha256(np.asarray(times, dtype='<f8').tobytes()).hexdigest())


def main():
    assert not (OUT / 'summary.json').exists(), 'Refusing to overwrite evaluation'
    assert not list(OUT.glob('*.tum')), 'Partial evaluation exists: inspect, do not retry'
    protected = json.loads((OFF / 'comparison/input_hashes.json').read_text())
    for file, expected in protected.items():
        assert sha(Path(file)) == expected, ('Changed frozen input', file)
    curves = {}
    sources = {}
    for mode, root in [('off', OFF), ('on', ON)]:
        src = root / ('pilot_' + mode + '_source')
        rec = root / ('pilot_' + mode + '_receiver/receiver')
        paths = {'native_glim': src / 'odom_imu.txt',
                 'native_kimera': src / 'kimera_odometry.csv',
                 'fused_online': rec / 'online_trajectory.tum',
                 'fused_final': rec / 'trajectory.tum'}
        for role, path in paths.items():
            name = mode + '_' + role
            curve = (relative.load_odometry_csv(path) if role == 'native_kimera'
                     else relative.load_tum(path))
            assert np.all(np.diff(curve[0]) > 0) and all(np.isfinite(a).all() for a in curve)
            curves[name] = curve
            sources[name] = dict(path=str(path), sha256=sha(path), rows=len(curve[0]))
    t, x, q, frames, children = relative.load_reference_bag(REFERENCE, TOPIC)
    assert children == ['cpt7_imu'], children
    reference = relative.sort_unique(t, x, q)
    first40 = reference[0][(reference[0] >= FIRST_IMU) & (reference[0] < FIRST_IMU+40)]
    assert np.max(np.diff(first40)) <= .05
    assert reference[0][0] <= min(c[0][0] for c in curves.values())
    assert reference[0][-1] >= max(c[0][-1] for c in curves.values())
    # Numerical check of unchanged unit-scale alignment on a known rigid motion.
    synthetic = np.array([[0,0,0], [1,0,0], [0,2,0], [0,0,3], [1,2,3]], float)
    rotation = np.array([[0,-1,0], [1,0,0], [0,0,1]], float)
    target = synthetic @ rotation.T + [3,4,5]
    rr, tt = prism.rigid_kabsch(synthetic, target)
    assert np.max(np.abs(synthetic @ rr.T + tt-target)) < 1e-12
    assert pose_at(reference, reference[0][0]-1, .05) is None
    gap_curve = (np.array([0.,1.]), np.zeros((2,3)), np.array([[0,0,0,1.]]*2))
    assert pose_at(gap_curve, .5, .05) is None
    common_times = np.loadtxt(OFF / 'comparison/common_grid/off_native_glim.tum')[:,0]
    results = {}
    for name, curve in curves.items():
        full_poses = [(p, relative.quaternion_to_matrix(qq)) for p, qq in zip(curve[1],curve[2])]
        common_poses = [pose_at(curve, stamp, .5) for stamp in common_times]
        assert all(p is not None for p in common_poses)
        results[name] = dict(
            full_saved_output=score(name, 'full', curve[0], full_poses, reference),
            identical_time_comparison=score(name, 'common', common_times, common_poses, reference))
        assert results[name]['full_saved_output']['rows'] == sources[name]['rows']
    assert len({r['identical_time_comparison']['timestamps_sha256'] for r in results.values()}) == 1
    for file, expected in protected.items():
        assert sha(Path(file)) == expected, ('Frozen input modified', file)
    report = dict(
        purpose='Full saved-output ATE, not the former 11.2 s prism-only ATE',
        reference=dict(path=str(REFERENCE), sha256=sha(REFERENCE), topic=TOPIC,
                       frame_ids=frames, child_frame_ids=children, first40_samples=len(first40),
                       first40_max_gap_s=float(np.diff(first40).max()),
                       identity='Dataset CPT7 Inertial Explorer tightly-coupled GNSS/INS reference',
                       independence='Shares CPT7 measurements with the native estimators; NOT independent prism GT'),
        convention=dict(alignment='One SE(3) Kabsch per complete trajectory, no scale',
                        evaluation_point='Physical prism: same retained CPT7-to-prism lever arm on both sides',
                        reference_interpolation='Position linear, quaternion SLERP; gap <=0.05 s; no extrapolation',
                        whole_output='All original saved poses included. No interpolation across startup gaps.',
                        common_comparison='361 identical timestamps, 3.571568 to39.571568 s; no startup gap filling',
                        new_estimator_runs=0, changed_estimator_files=0),
        sources=sources, results=results,
        checks=dict(frozen_input_hashes_unchanged=len(protected), all_original_rows_scored=True,
                    every_pose_has_reference=True, identical_common_masks=True,
                    synthetic_alignment_passed=True, missing_reference_gap_rejected=True))
    write_json(OUT / 'summary.json', report)
    print(json.dumps({n:dict(full_ATE_m=v['full_saved_output']['ATE_RMSE_m'],
                             common_ATE_m=v['identical_time_comparison']['ATE_RMSE_m'],
                             full_rows=v['full_saved_output']['rows'],
                             full_path_m=v['full_saved_output']['full_CPT7_polyline_path_m'])
                      for n,v in results.items()}, indent=2))


if __name__ == '__main__':
    main()
