"""Evo-only APE/RPE, frozen saved grids, both output policies, seven pairs."""
import copy
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
import numpy as np
import evo
from evo.core import filters, metrics
from evo.tools import file_interface

H = Path(__file__).resolve().parent
H31 = H.parent/'eth1_evo_rpe_distance_cpv31_20261029'
spec = importlib.util.spec_from_file_location('cpv31_helpers', H31/'evaluate.py')
u = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u)
CASES = ('hesai_eth1', 'hesai_eig2', 'hesai_spx3', 'hesai_leica2',
         'velodyne_spx3', 'velodyne_eig2', 'velodyne_leica2')
ENV = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
           PYTHONDONTWRITEBYTECODE='1', MPLBACKEND='Agg')
PINNED = {}


def pin(path, expected=None):
    path = Path(path)
    digest = u.sha(path)
    if expected is not None:
        assert digest == expected, path
    PINNED[str(path)] = digest


def run_cli(kind, ref_path, est_path, archive, relation, commands):
    cli = Path('/home/yeranis/.local/bin/evo_'+kind)
    config = H/'ape_config.json' if kind == 'ape' else H31/'evo_config.json'
    cmd = [str(cli), 'tum', str(ref_path), str(est_path), '-r', relation,
           '-c', str(config), '--save_results', str(archive)]
    if kind == 'ape':
        cmd += ['--align']
    else:
        cmd += ['-d', '0.5', '-u', 'm', '--pairs_from_reference']
    assert not archive.exists()
    log = archive.with_suffix('.log')
    with log.open('x') as stream:
        p = subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT,
                           env=ENV, timeout=60, check=False)
    receipt = dict(command=cmd, returncode=p.returncode, logfile=str(log), archive=str(archive))
    u.save(archive.with_suffix('.command.json'), receipt)
    commands.append(receipt)
    assert p.returncode == 0, receipt
    return file_interface.load_res_file(archive)


def export(path, traj, checks):
    loaded, difference = u.export(path, traj)
    checks['TUM_roundtrip_pose_max_component'] = max(checks['TUM_roundtrip_pose_max_component'], difference)
    return loaded


def compare_metric(result, metric, checks):
    error = result.np_arrays['error_array']
    assert error.shape == metric.error.shape and np.isfinite(error).all()
    diff = float(np.max(np.abs(error-metric.error)))
    checks['CLI_API_error_max_difference'] = max(checks['CLI_API_error_max_difference'], diff)
    assert diff < 1e-8, diff
    for name,value in metric.get_all_statistics().items():
        assert abs(float(value)-result.stats[name]) < 1e-8, name


def evaluate(case):
    folder = H/case
    folder.mkdir(exist_ok=False)
    arrays, old = {}, {}
    checks = dict(TUM_roundtrip_pose_max_component=0., CLI_API_error_max_difference=0.,
                  old_ATE_max_difference_m=0., old_path_max_difference_m=0.,
                  historical_hashes_verified=True, masks_and_reference_identical=True)
    for method, source in u.SOURCES.items():
        manifest = json.loads((source/'closeout.json').read_text())['output_sha256']
        for suffix in ('.npz', '.json'):
            path = source/(case+suffix)
            pin(path, manifest[str(path)])
        old[method] = json.loads((source/(case+'.json')).read_text())
        with np.load(source/(case+'.npz'), allow_pickle=False) as z:
            arrays[method] = {k:z[k] for k in z.files}
    for key in ('timestamp_ns', 'common_valid', 'reference_position', 'reference_rotation'):
        assert np.array_equal(arrays['method1'][key], arrays['method2'][key]), (case,key)
    base = arrays['method1']
    origin = int(base['timestamp_ns'][0])
    ids = np.flatnonzero(base['common_valid'])
    segments = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
    epoch = old['method2']['epoch_ns']
    ref_path = folder/'reference_common.tum'
    ref = export(ref_path, u.curve(base, 'reference', ids, origin), checks)
    refs, pair_records, segment_info = [], [], []
    for si, indices in enumerate(segments):
        target = folder/f'segment_{si}'
        target.mkdir()
        path = target/'reference.tum'
        rt = export(path, u.curve(base, 'reference', indices, origin), checks)
        pairs = filters.filter_pairs_by_path(rt.poses_se3, .5, all_pairs=False)
        info = dict(segment=si, rows=len(indices), pairs=len(pairs), reference_path_m=rt.path_length,
                    first_s=(int(base['timestamp_ns'][indices[0]])-epoch)*1e-9,
                    last_s=(int(base['timestamp_ns'][indices[-1]])-epoch)*1e-9)
        if pairs:
            distances = rt.distances
            pair_distances = []
            for i,j in pairs:
                distance = float(distances[j]-distances[i])
                pair_distances.append(distance)
                pair_records.append(dict(segment=si, start_grid_index=int(indices[i]), end_grid_index=int(indices[j]),
                    start_timestamp_ns=int(base['timestamp_ns'][indices[i]]),
                    end_timestamp_ns=int(base['timestamp_ns'][indices[j]]), reference_distance_m=distance))
            info.update(actual_min_m=min(pair_distances), actual_max_m=max(pair_distances))
        else:
            info['omission'] = 'No evo consecutive delta0.5m pair; no bridging or relaxation.'
        segment_info.append(info)
        refs.append((rt, path, pairs))
    if case == 'hesai_eth1':
        assert pair_records == json.loads((H31/'pair_indices.json').read_text())
    u.save(folder/'pair_indices.json', pair_records)
    commands, results = [], {}
    for method in u.SOURCES:
        for mode in ('off', 'on'):
            for label, role in u.ROLES[method].items():
                key = f'{method}_{mode}_{label.lower()}'
                stem = f'{mode}_{role}'
                est_path = folder/(key+'_common.tum')
                est = export(est_path, u.curve(arrays[method], stem, ids, origin), checks)
                archive = folder/(key+'_ape.zip')
                ape = run_cli('ape', ref_path, est_path, archive, 'trans_part', commands)
                aligned = copy.deepcopy(est)
                aligned.align(ref, correct_scale=False)
                metric = metrics.APE(metrics.PoseRelation.translation_part)
                metric.process_data((ref, aligned))
                compare_metric(ape, metric, checks)
                previous = old[method]['common'][stem]
                difference = abs(ape.stats['rmse']-previous['ATE_RMSE_m'])
                checks['old_ATE_max_difference_m'] = max(checks['old_ATE_max_difference_m'], difference)
                assert difference < 1e-8, (case,key,difference)
                errors = {name:[] for name in u.RELATIONS}
                path_length = 0.
                for si,indices in enumerate(segments):
                    target = folder/f'segment_{si}'
                    spath = target/(key+'.tum')
                    se = export(spath, u.curve(arrays[method],stem,indices,origin), checks)
                    path_length += se.path_length
                    sr,srpath,pairs = refs[si]
                    if not pairs or case == 'hesai_eth1':
                        continue
                    for name,(relation_string,relation) in u.RELATIONS.items():
                        out = target/(key+'_'+name+'.zip')
                        rpe = run_cli('rpe', srpath, spath, out, relation_string, commands)
                        rm = u.new_metric(relation)
                        rm.process_data((sr,se))
                        assert rm.delta_ids == [j for i,j in pairs]
                        compare_metric(rpe,rm,checks)
                        errors[name].append(rpe.np_arrays['error_array'])
                difference = abs(path_length-previous['path_length_m'])
                checks['old_path_max_difference_m'] = max(checks['old_path_max_difference_m'],difference)
                assert difference < 1e-8, (case,key,difference)
                row = dict(method=method, mode=mode, estimator=label, ATE_RMSE_m=ape.stats['rmse'],
                           ATE_evo_archive=str(archive), path_length_m=path_length, RPE={})
                for name,(_,relation) in u.RELATIONS.items():
                    if case == 'hesai_eth1':
                        out = H31/'outputs'/(key+'_'+name+'_aggregate.zip')
                        result = file_interface.load_res_file(out)
                        pin(out)
                        reused = True
                    else:
                        rm = u.new_metric(relation)
                        rm.error = np.concatenate(errors[name])
                        result = rm.get_result(case+'_reference',key)
                        result.add_info(dict(aggregation='Pooled evo CLI errors across valid segments',
                                             pair_indices_file=str(folder/'pair_indices.json')))
                        out = folder/(key+'_'+name+'_aggregate.zip')
                        file_interface.save_res_file(out,result)
                        check = file_interface.load_res_file(out)
                        assert check.stats == result.stats
                        assert np.array_equal(check.np_arrays['error_array'],result.np_arrays['error_array'])
                        reused = False
                    assert len(result.np_arrays['error_array']) == len(pair_records)
                    row['RPE'][name] = dict(statistics=result.stats, pairs=len(pair_records),
                                            evo_archive=str(out), reused_CPV31=reused)
                results[key] = row
                print(f'{case} {key}: ATE={row["ATE_RMSE_m"]:.6f}, RPEt={row["RPE"]["translation_m"]["statistics"]["rmse"]:.6f}',flush=True)
    assert all(u.sha(p)==digest for p,digest in PINNED.items())
    result = dict(case=case, sensor=old['method1']['sensor'], sequence=old['method1']['sequence'],
                  original_receiver_Qc_multiplier=1., epoch_ns=epoch, timestamp_origin_ns=origin,
                  common_coverage=old['method1']['common_coverage'], segments=segment_info,
                  reference_path_length_m=sum(x[0].path_length for x in refs),
                  RPE_pairs=len(pair_records), results=results, verification=checks,
                  original_terminal_omissions=old['method1']['original_terminal_omissions'],
                  new_CLI_calls=len(commands))
    u.save(folder/'commands.json',commands)
    u.save(H/(case+'.json'),result)
    print(f'CASE COMPLETE {case}: {len(pair_records)} common RPE pairs, {len(commands)} new evo CLI calls',flush=True)
    return result


def main():
    assert importlib.metadata.version('evo') == '1.36.3'
    for p in (Path(__file__), H/'PROTOCOL.txt',H/'ape_config.json',H31/'evaluate.py',H31/'evo_config.json'):
        pin(p)
    for path in Path(evo.__file__).parent.rglob('*.py'):
        pin(path)
    for name in ('evo_ape','evo_rpe'):
        pin(Path('/home/yeranis/.local/bin',name).resolve())
    for source in u.SOURCES.values():
        pin(source/'closeout.json')
    receipt = H31/'closeout.json'
    pin(receipt)
    for path,digest in json.loads(receipt.read_text())['output_sha256'].items():
        pin(path,digest)
    cases = {case:evaluate(case) for case in CASES}
    assert all(u.sha(p)==digest for p,digest in PINNED.items())
    u.save(H/'input_sha256.json',PINNED)
    u.save(H/'results.json',dict(task='CP-V32',evo_version=importlib.metadata.version('evo'),
        python=sys.executable,cases=cases,passed=True,input_files_verified=len(PINNED),
        new_CLI_calls=sum(d['new_CLI_calls'] for d in cases.values()),
        reused_ETH1_RPE_metrics=24,source_runs=0,receiver_replays=0,builds=0,containers_started=0))
    print('ALL SEVEN CASES COMPLETE',flush=True)


if __name__ == '__main__':
    main()
