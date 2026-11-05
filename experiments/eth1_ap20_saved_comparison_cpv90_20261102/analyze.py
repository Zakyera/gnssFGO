"""CP90: exclusive saved-data AP20 comparison; no ROS, sources or old writes."""
import copy
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.highlevel import AnyReader
from evo.core import metrics, trajectory
from evo.tools import file_interface

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = E / 'eth1_alpha_bias_rw_full_off_cpv89_20261102'
P = W / 'runs' / OLD.name / 'prepared'
DATA = W / 'src/datasets/GrandTour'
AP = DATA / '2024-10-01-11-29-55_ap20_prism_position.bag'
IE = DATA / '2024-10-01-11-29-55_cpt7_ie_tc.bag'
CHECKPOINT = E / 'checkpoints/20261023_hesai_four_sequences'
OUT = H / 'evaluation'
EVO_APE = '/home/yeranis/.local/bin/evo_ape'
SOURCES = {n: OLD / 'evaluation' / ('full_' + n) / (n + '.tum')
           for n in ('GLIM_CPT7', 'Kimera_Alphasense')}


def read(p):
    return json.loads(Path(p).read_text())


def save(p, value):
    with Path(p).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def integrity():
    groups = {
        'CP89_protected': read(P / 'protected_manifest.json'),
        'CP89_terminal': read(OLD / 'complete.json')['output_sha256'],
        'CP89_report_inputs': read(OLD / 'evaluation/input_sha256.json'),
    }
    checkpoint = read(CHECKPOINT / 'manifest.json')
    groups['Hesai_copies'] = {str(CHECKPOINT / k): v['sha256'] for k, v in checkpoint['files'].items()}
    groups['Hesai_originals'] = {v['original']: v['sha256'] for v in checkpoint['files'].values()}
    all_hashes = {}
    for group, values in groups.items():
        for path, expected in values.items():
            if path not in all_hashes:
                all_hashes[path] = sha(path)
            assert all_hashes[path] == expected, (group, path)
    assert [len(groups[x]) for x in groups] == [3716, 148, 28, 667, 667]
    assert read(OLD / 'post_closeout.json')['passed']
    return {'passed': True, 'counts': {k: len(v) for k, v in groups.items()},
            'unique_files_checked': len(all_hashes), 'time_utc': datetime.now(timezone.utc).isoformat()}


def load_bag(path, topic, prism=None):
    times, points, frames, children, deltas = [], [], set(), set(), []
    with AnyReader([path]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        assert connections, (path, topic)
        for connection, record_time, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            stamp = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
            frames.add(msg.header.frame_id)
            if prism is None:
                point = [msg.point.x, msg.point.y, msg.point.z]
            else:
                children.add(msg.child_frame_id)
                pose = msg.pose.pose
                q = pose.orientation
                quat = np.array([q.x, q.y, q.z, q.w])
                assert abs(np.linalg.norm(quat) - 1) < 1e-6
                point = np.array([pose.position.x, pose.position.y, pose.position.z])
                point += Rotation.from_quat(quat).apply(prism)
            times.append(stamp)
            points.append(point)
            deltas.append(int(record_time) - stamp)
    ts, xyz = np.asarray(times, dtype=np.int64), np.asarray(points, dtype=float)
    assert np.isfinite(xyz).all() and (ts > 0).all() and (np.diff(ts) > 0).all()
    return ts, xyz, {'rows': len(ts), 'frames': sorted(frames), 'child_frames': sorted(children),
                     'first_ns': int(ts[0]), 'last_ns': int(ts[-1]),
                     'record_header_max_delta_ns': max(abs(x) for x in deltas),
                     'median_period_s': float(np.median(np.diff(ts)) * 1e-9),
                     'max_gap_s': float(np.max(np.diff(ts)) * 1e-9)}


def sample(ts, xyz, query, max_gap):
    insertion = np.searchsorted(ts, query)
    hi = np.minimum(insertion, len(ts) - 1)
    lo = np.maximum(insertion - 1, 0)
    exact = (insertion < len(ts)) & (ts[hi] == query)
    valid = ((insertion > 0) & (insertion < len(ts)) & ((ts[hi] - ts[lo]) <= max_gap)) | exact
    lo[exact] = hi[exact]
    gap = ts[hi] - ts[lo]
    weight = np.zeros(len(query))
    np.divide(query - ts[lo], gap, out=weight, where=gap != 0)
    positions = xyz[lo] * (1 - weight[:, None]) + xyz[hi] * weight[:, None]
    return positions, valid, lo, hi, weight


def blocks(ids, ts, max_gap):
    return np.split(ids, np.flatnonzero((np.diff(ids) != 1) | (np.diff(ts[ids]) > max_gap)) + 1)


def curve(ts, xyz, epoch):
    # Identity quaternions are placeholders; only translation metrics are allowed.
    return trajectory.PoseTrajectory3D(
        positions_xyz=xyz, orientations_quat_wxyz=np.tile([1., 0., 0., 0.], (len(ts), 1)),
        timestamps=(ts - epoch).astype(float) * 1e-9)


def path_length(ts, xyz, segments, epoch):
    return sum(float(curve(ts[ii], xyz[ii], epoch).path_length) for ii in segments if len(ii) > 1)


def export(name, ts, xyz, epoch):
    path = OUT / (name + '.tum')
    data = np.column_stack(((ts - epoch).astype(float) * 1e-9, xyz,
                            np.zeros((len(ts), 3)), np.ones(len(ts))))
    with path.open('x') as f:
        np.savetxt(f, data, fmt=['%.9f'] + ['%.15g'] * 7)
    loaded = file_interface.read_tum_trajectory_file(path)
    assert np.array_equal(np.rint(loaded.timestamps * 1e9).astype(np.int64) + epoch, ts)
    assert np.max(abs(loaded.positions_xyz - xyz)) < 1e-10
    return path


def score(estimate, reference, paths, commands):
    name = estimate + '_vs_' + reference
    archive = OUT / (name + '.zip')
    cmd = [EVO_APE, 'tum', str(paths[reference]), str(paths[estimate]),
           '-r', 'trans_part', '-c', str(H / 'ape_config.json'), '--save_results', str(archive)]
    assert not archive.exists()
    commands.append(cmd)
    save(OUT / (name + '.command.json'), cmd)
    with (OUT / (name + '.log')).open('x') as log:
        subprocess.run(cmd, check=True, stdout=log, stderr=subprocess.STDOUT)
    saved = file_interface.load_res_file(archive)
    ref = file_interface.read_tum_trajectory_file(paths[reference])
    est = file_interface.read_tum_trajectory_file(paths[estimate])
    assert np.array_equal(ref.timestamps, est.timestamps)
    aligned = copy.deepcopy(est)
    rotation, translation, scale = aligned.align(ref, correct_scale=False)
    assert scale == 1
    metric = metrics.APE(metrics.PoseRelation.translation_part)
    metric.process_data((ref, aligned))
    assert np.allclose(metric.error, saved.np_arrays['error_array'], rtol=1e-9, atol=1e-10)
    statistics = metric.get_all_statistics()
    for key, value in statistics.items():
        assert np.isclose(value, saved.stats[key], rtol=1e-9, atol=1e-10)
    return {'rows': len(metric.error), 'statistics_m': statistics,
            'p95_m': float(np.percentile(metric.error, 95)),
            'alignment': {'rotation': rotation.tolist(), 'translation': translation.tolist(), 'scale': scale},
            'CLI_API_parity': True, 'archive': str(archive)}


def main():
    assert importlib.metadata.version('evo') == '1.36.3'
    OUT.mkdir()  # Exclusive: a second invocation cannot overwrite an earlier analysis.
    audit = read(P / 'input_audit.json')
    epoch, end, duration = audit['epoch_ns'], audit['end_ns'], audit['duration_s']
    assert duration == 346.32 and end - epoch == 346_320_000_000
    input_paths = [Path(__file__), H / 'PROTOCOL.txt', H / 'ape_config.json', AP, IE,
                   P / 'input_audit.json', P / 'protected_manifest.json', OLD / 'complete.json',
                   OLD / 'post_closeout.json', OLD / 'evaluation/input_sha256.json',
                   CHECKPOINT / 'manifest.json', *SOURCES.values()]
    pins = {str(p): sha(p) for p in input_paths}
    save(OUT / 'input_sha256.json', pins)
    print('Checking historical integrity before analysis.', flush=True)
    save(H / 'integrity_before.json', integrity())
    raw_t, raw_p, raw_info = load_bag(AP, '/boxi/ap20/prism_position')
    assert raw_info['rows'] == 4547 and raw_info['frames'] == ['leica_total_station']
    assert raw_info['record_header_max_delta_ns'] == 0
    in_window = (raw_t >= epoch) & (raw_t <= end)
    query = raw_t[in_window]
    raw_indices = np.flatnonzero(in_window)
    assert len(query) == len(raw_t)  # This bag's entire AP20 track is in the requested window.
    native, samples, source_info = {}, {}, {}
    good = np.ones(len(query), dtype=bool)
    for name, path in SOURCES.items():
        tum = file_interface.read_tum_trajectory_file(path)
        ts = np.rint(tum.timestamps * 1e9).astype(np.int64) + epoch
        xyz = tum.positions_xyz
        assert np.isfinite(xyz).all() and (np.diff(ts) > 0).all()
        assert len(ts) == (3454 if name == 'GLIM_CPT7' else 1493)
        native[name] = (ts, xyz)
        samples[name] = sample(ts, xyz, query, 500_000_000)
        good &= samples[name][1]
        segments = blocks(np.arange(len(ts)), ts, 500_000_000)
        support = sum(int(ts[ii[-1]] - ts[ii[0]]) for ii in segments) * 1e-9
        source_info[name] = {'rows': len(ts), 'first_s': float((ts[0] - epoch) * 1e-9),
                             'last_s': float((ts[-1] - epoch) * 1e-9), 'supported_s': support,
                             'coverage_percent': 100 * support / duration, 'gaps_over_0_5s': len(segments) - 1,
                             'segmented_native_path_m': path_length(ts, xyz, segments, epoch),
                             'AP20_rows_supported_individually': int(samples[name][1].sum())}
    ids = np.flatnonzero(good)
    assert len(ids) > 100
    ie_t, ie_p, ie_info = load_bag(IE, '/boxi/inertial_explorer/tc/odometry',
                                 np.asarray(audit['T_cpt7_prism'])[:3, 3])
    assert ie_info['frames'] == ['enu_origin'] and ie_info['child_frames'] == ['cpt7_imu']
    samples['IE_TC'] = sample(ie_t, ie_p, query, 50_000_000)
    assert samples['IE_TC'][1][ids].all(), 'IE support must not shrink the primary AP20 mask'
    save(OUT / 'raw_AP20.json', {'timestamp_ns': raw_t.tolist(), 'position_m': raw_p.tolist(), 'audit': raw_info})
    accepted = []
    rejected = []
    for index in range(len(query)):
        entry = {'raw_AP20_index': int(raw_indices[index]), 'timestamp_ns': int(query[index]),
                 'relative_s': float((query[index] - epoch) * 1e-9)}
        if good[index]:
            for name, values in samples.items():
                points, valid, lo, hi, weight = values
                times = ie_t if name == 'IE_TC' else native[name][0]
                entry[name] = {'left_index': int(lo[index]), 'right_index': int(hi[index]),
                               'left_ns': int(times[lo[index]]), 'right_ns': int(times[hi[index]]),
                               'right_weight': float(weight[index])}
            accepted.append(entry)
        else:
            entry['unsupported_estimators'] = [name for name in SOURCES if not samples[name][1][index]]
            rejected.append(entry)
    save(OUT / 'accepted_mask.json', accepted)
    save(OUT / 'rejected_mask.json', rejected)
    raw_segments = blocks(np.arange(len(query)), query, 200_000_000)
    segments = blocks(ids, query, 200_000_000)
    supported = sum(int(query[ii[-1]] - query[ii[0]]) for ii in segments) * 1e-9
    raw_supported = sum(int(query[ii[-1]] - query[ii[0]]) for ii in raw_segments) * 1e-9
    segment_info = [{'first_raw_index': int(raw_indices[ii[0]]), 'last_raw_index': int(raw_indices[ii[-1]]),
                     'rows': len(ii), 'first_s': float((query[ii[0]] - epoch) * 1e-9),
                     'last_s': float((query[ii[-1]] - epoch) * 1e-9)} for ii in segments]
    positions = {'AP20': raw_p[in_window], **{n: v[0] for n, v in samples.items()}}
    paths = {n: export(n, query[ids], xyz[ids], epoch) for n, xyz in positions.items()}
    commands = []
    comparisons = {}
    for est, ref in [('GLIM_CPT7', 'AP20'), ('Kimera_Alphasense', 'AP20'), ('IE_TC', 'AP20'),
                     ('GLIM_CPT7', 'IE_TC'), ('Kimera_Alphasense', 'IE_TC')]:
        comparisons[est + '_vs_' + ref] = score(est, ref, paths, commands)
        print(est, 'vs', ref, comparisons[est + '_vs_' + ref]['statistics_m']['rmse'], flush=True)
    save(OUT / 'commands.json', commands)
    print('Checking historical integrity after analysis.', flush=True)
    after = integrity()
    assert all(sha(path) == expected for path, expected in pins.items())
    save(H / 'integrity_after.json', after)
    result = {'task': 'CP-V90', 'passed': True, 'evo_version': '1.36.3',
              'scope': 'Saved CP89 ETH1 CBS-OFF Method1 physical-prism positions, no source rerun',
              'epoch_ns': epoch, 'requested_duration_s': duration, 'AP20_raw': raw_info, 'IE_raw': ie_info,
              'full_saved_native': source_info, 'common': {'rows': len(ids), 'rejected_AP20_rows': len(rejected),
              'first_s': segment_info[0]['first_s'], 'last_s': segment_info[-1]['last_s'],
              'supported_s': supported, 'coverage_percent': 100 * supported / duration,
              'raw_reference_supported_s': raw_supported, 'raw_reference_coverage_percent': 100 * raw_supported / duration,
              'segments': segment_info, 'path_lengths_m': {n: path_length(query, xyz, segments, epoch) for n, xyz in positions.items()}},
              'comparisons': comparisons, 'limitations': [
                  'AP20 position-only and partially visible; not an official ETH1 benchmark score',
                  'No RPE or orientation score; identity quaternions only serialize position curves',
                  'Estimator positions interpolated only over native gaps <=0.5s; AP20 never interpolated',
                  'No reference outage chords; support splits at raw gaps >0.20s or rejected raw rows',
                  'Global rigid no-scale alignment removes one constant coordinate-frame discrepancy',
                  'Same-mask IE-TC scores use different support/alignment from original CP89 full-window scores',
                  'IE-TC shares CPT7 with GLIM; AP20 is an external physical position check, not covariance calibration',
                  'Estimated polyline length is not physical travel; sparse measurements underresolve motion',
                  'Experimental Alpha bias noise candidate; no full CPT7 Kimera matched control or CBS exchange']}
    save(H / 'results.json', result)
    save(H / 'verification.json', {'passed': True, 'CLI_calls': len(commands), 'CLI_API_parity': True,
                                  'all_five_comparisons_same_raw_AP20_mask': True, 'IE_does_not_reduce_mask': True,
                                  'AP20_interpolation': False, 'calibration_applied_to_native_TUM_again': False,
                                  'historical_hashes': after, 'input_hashes_unchanged': len(pins),
                                  'TUM_roundtrip_timestamps_exact': True, 'TUM_position_roundtrip_error_lt': 1e-10,
                                  'native_TUM_counts': {n: v['rows'] for n, v in source_info.items()}})
    print(json.dumps({'common': result['common'], 'full_saved_native': source_info}, indent=2), flush=True)


if __name__ == '__main__':
    for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        assert os.environ.get(variable) == '1'
    main()
