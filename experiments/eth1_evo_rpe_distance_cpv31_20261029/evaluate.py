"""Saved ETH1 RPE; evo owns pair selection, pose errors and statistics."""
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
import numpy as np
import evo
from evo.core import filters, metrics, trajectory, units
from evo.tools import file_interface

H = Path(__file__).resolve().parent
ROOT = H.parent
CLI = Path('/home/yeranis/.local/bin/evo_rpe')
SOURCES = {
    'method1': ROOT/'newest_absolute_comparison_cpv27_20261028',
    'method2': ROOT/'all_incremental_comparison_cpv28_20261028',
}
ROLES = {
    'method1': {'GLIM': 'glim_initial', 'Kimera': 'kimera_absolute', 'Fusion': 'fusion_online'},
    'method2': {'GLIM': 'glim', 'Kimera': 'kimera', 'Fusion': 'fusion'},
}
RELATIONS = {
    'translation_m': ('trans_part', metrics.PoseRelation.translation_part),
    'rotation_deg': ('angle_deg', metrics.PoseRelation.rotation_angle_deg),
}


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def new_metric(relation):
    return metrics.RPE(relation, delta=.5, delta_unit=units.Unit.meters,
                       rel_delta_tol=.1, all_pairs=False, pairs_from_reference=True)


def curve(data, stem, ids, origin):
    poses = np.tile(np.eye(4), (len(ids), 1, 1))
    poses[:, :3, :3] = data[stem+'_rotation'][ids]
    poses[:, :3, 3] = data[stem+'_position'][ids]
    return trajectory.PoseTrajectory3D(
        poses_se3=list(poses), timestamps=(data['timestamp_ns'][ids]-origin)*1e-9)


def export(path, traj):
    assert not path.exists(), path
    file_interface.write_tum_trajectory_file(path, traj)
    loaded = file_interface.read_tum_trajectory_file(path)
    assert np.array_equal(loaded.timestamps, traj.timestamps)
    difference = float(np.max(np.abs(np.asarray(loaded.poses_se3)-np.asarray(traj.poses_se3))))
    assert difference < 1e-10, (path, difference)
    return loaded, difference


def main():
    assert importlib.metadata.version('evo') == '1.36.3'
    assert not (H/'results.json').exists()
    output = H/'outputs'
    output.mkdir(exist_ok=False)
    pins = {str(p): sha(p) for p in (Path(__file__), H/'PROTOCOL.txt', CLI.resolve())}
    package_root = Path(evo.__file__).parent
    for p in package_root.rglob('*.py'):
        pins[str(p)] = sha(p)
    data, metadata = {}, {}
    for method, source in SOURCES.items():
        receipt = source/'closeout.json'
        pins[str(receipt)] = sha(receipt)
        manifest = json.loads(receipt.read_text())['output_sha256']
        for suffix in ('.json', '.npz'):
            p = source/('hesai_eth1'+suffix)
            pins[str(p)] = sha(p)
            assert pins[str(p)] == manifest[str(p)], p
        metadata[method] = json.loads((source/'hesai_eth1.json').read_text())
        with np.load(source/'hesai_eth1.npz', allow_pickle=False) as saved:
            data[method] = {k: saved[k] for k in saved.files}
    for key in ('timestamp_ns', 'common_valid', 'reference_position', 'reference_rotation'):
        assert np.array_equal(data['method1'][key], data['method2'][key]), key
    base = data['method1']
    origin = int(base['timestamp_ns'][0])
    valid = np.flatnonzero(base['common_valid'])
    segments = np.split(valid, np.flatnonzero(np.diff(valid)>1)+1)
    config = dict(delta=.5, delta_unit='m', all_pairs=False, pairs_from_reference=True,
                  delta_tol=.1, align=False, align_origin=False, correct_scale=False,
                  t_max_diff=1e-9, t_offset=0., project_to_plane=None,
                  downsample=None, motion_filter=None)
    save(H/'evo_config.json', config)
    pins[str(H/'evo_config.json')] = sha(H/'evo_config.json')
    save(H/'input_sha256.json', pins)
    segment_info, pairs_all, groups, command_receipts = [], [], {}, []
    max_roundtrip, max_parity, max_original_parity = 0., 0., 0.
    for index, ids in enumerate(segments):
        folder = output/f'segment_{index}'
        folder.mkdir()
        ref_original = curve(base, 'reference', ids, origin)
        ref_file = folder/'reference.tum'
        ref, check = export(ref_file, ref_original)
        max_roundtrip = max(max_roundtrip, check)
        pairs = filters.filter_pairs_by_path(ref.poses_se3, .5, all_pairs=False)
        pairs_original = filters.filter_pairs_by_path(ref_original.poses_se3, .5, all_pairs=False)
        assert pairs == pairs_original
        info = dict(segment=index, grid_first=int(ids[0]), grid_last=int(ids[-1]),
                    rows=len(ids), path_length_m=ref.path_length, pairs=len(pairs),
                    first_timestamp_ns=int(base['timestamp_ns'][ids[0]]),
                    last_timestamp_ns=int(base['timestamp_ns'][ids[-1]]))
        if not pairs:
            info['omission'] = 'No consecutive delta0.5m pair from evo; no delta relaxation or gap joining.'
        else:
            distances = ref.distances
            ds = [float(distances[j]-distances[i]) for i,j in pairs]
            info['actual_pair_distance_min_m'] = min(ds)
            info['actual_pair_distance_max_m'] = max(ds)
            for (i,j), distance in zip(pairs, ds):
                pairs_all.append(dict(segment=index, start_grid_index=int(ids[i]), end_grid_index=int(ids[j]),
                    start_timestamp_ns=int(base['timestamp_ns'][ids[i]]),
                    end_timestamp_ns=int(base['timestamp_ns'][ids[j]]), reference_distance_m=distance))
        segment_info.append(info)
        for method in SOURCES:
            for mode in ('off', 'on'):
                for label, role in ROLES[method].items():
                    key = f'{method}_{mode}_{label.lower()}'
                    original = curve(data[method], f'{mode}_{role}', ids, origin)
                    est_file = folder/(key+'.tum')
                    est, check = export(est_file, original)
                    max_roundtrip = max(max_roundtrip, check)
                    if not pairs:
                        continue
                    for metric_name, (cli_relation, relation) in RELATIONS.items():
                        archive = folder/(key+'_'+metric_name+'.zip')
                        logfile = folder/(key+'_'+metric_name+'.log')
                        command = [str(CLI), 'tum', str(ref_file), str(est_file), '-r', cli_relation,
                                   '-d', '0.5', '-u', 'm', '--pairs_from_reference',
                                   '-c', str(H/'evo_config.json'), '--save_results', str(archive)]
                        assert not archive.exists()
                        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                                   PYTHONDONTWRITEBYTECODE='1', MPLBACKEND='Agg')
                        with logfile.open('x') as stream:
                            proc = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                                  env=env, timeout=60, check=False)
                        receipt = dict(command=command, returncode=proc.returncode,
                                       logfile=str(logfile), archive=str(archive))
                        save(folder/(key+'_'+metric_name+'_command.json'), receipt)
                        command_receipts.append(receipt)
                        assert proc.returncode == 0, receipt
                        result = file_interface.load_res_file(archive)
                        metric = new_metric(relation)
                        metric.process_data((ref, est))
                        assert metric.delta_ids == [j for i,j in pairs]
                        errors = result.np_arrays['error_array']
                        assert errors.shape == metric.error.shape == (len(pairs),)
                        assert np.isfinite(errors).all()
                        delta = float(np.max(np.abs(errors-metric.error)))
                        max_parity = max(max_parity, delta)
                        assert delta < 1e-10, (key, metric_name, delta)
                        original_metric = new_metric(relation)
                        original_metric.process_data((ref_original, original))
                        delta_original = float(np.max(np.abs(errors-original_metric.error)))
                        max_original_parity = max(max_original_parity, delta_original)
                        assert delta_original < 1e-7, (key, metric_name, delta_original)
                        for stat, value in metric.get_all_statistics().items():
                            assert abs(float(value)-result.stats[stat]) < 1e-10
                        group = groups.setdefault(key, dict(method=method, mode=mode, estimator=label, metrics={}))
                        group['metrics'].setdefault(metric_name, []).append(errors)
                        print(f'PASS {key} {metric_name} segment{index}: {len(pairs)} pairs; RMSE={result.stats["rmse"]:.9f}', flush=True)
    save(H/'pair_indices.json', pairs_all)
    results = {}
    for key, group in groups.items():
        stats = {}
        for metric_name, errors in group['metrics'].items():
            metric = new_metric(RELATIONS[metric_name][1])
            # Pool ONLY evo-computed errors; evo itself computes the aggregate statistics.
            metric.error = np.concatenate(errors)
            assert len(metric.error) == len(pairs_all)
            result = metric.get_result('ETH1_IE_TC_common_reference', key)
            result.add_info(dict(aggregation='Pooled evo CLI error arrays, no gap-crossing pairs',
                                 pair_indices_file=str(H/'pair_indices.json'),
                                 method=group['method'], mode=group['mode']))
            out = output/(key+'_'+metric_name+'_aggregate.zip')
            file_interface.save_res_file(out, result)
            check = file_interface.load_res_file(out)
            assert np.array_equal(check.np_arrays['error_array'], metric.error)
            assert check.stats == result.stats
            stats[metric_name] = dict(statistics=result.stats, pairs=len(metric.error), archive=str(out))
        results[key] = dict(method=group['method'], mode=group['mode'], estimator=group['estimator'], **stats)
    assert len(results) == 12
    assert all(sha(p) == digest for p,digest in pins.items())
    result = dict(task='CP-V31', sensor='Hesai', sequence='ETH1', evo_version=importlib.metadata.version('evo'),
        python=sys.executable, evo_package=str(package_root), settings=config, timestamp_origin_ns=origin,
        original_receiver_Qc_multiplier=1., common_coverage=metadata['method1']['common_coverage'],
        segments=segment_info, total_pairs=len(pairs_all), results=results,
        reference_path_on_common_segments_m=sum(x['path_length_m'] for x in segment_info),
        verification=dict(passed=True, pinned_files=len(pins), historical_manifest_hashes_pass=True,
            identical_reference_masks_and_pairs=True, CLI_calls=len(command_receipts),
            CLI_API_max_error_difference=max_parity, CLI_original_array_max_error_difference=max_original_parity,
            TUM_roundtrip_pose_max_component_difference=max_roundtrip, aggregate_archives_read_back=True,
            inputs_unchanged=True, source_runs=0, receiver_replays=0, builds=0, containers_started=0),
        caveats=['Method1 Kimera is recovered absolute backend poses, not native incremental ROS.',
            'Method2 is causal same-update incremental chaining, not finalized trajectory.',
            'IE-TC shares CPT7; not independent prism ground truth.',
            'Frozen common-mask coverage, not full capture. Original Qc1.0, not0.25.',
            'Consecutive distance threshold on reference; actual intervals can exceed0.5m.',
            'Saved grid/frame preprocessing remains; all RPE errors, pairing and statistics use evo.'])
    save(H/'commands.json', command_receipts)
    save(H/'results.json', result)
    save(H/'verification.json', result['verification'])
    print(json.dumps(dict(completed=True, total_pairs=len(pairs_all), checks=result['verification'])), flush=True)


if __name__ == '__main__':
    main()
