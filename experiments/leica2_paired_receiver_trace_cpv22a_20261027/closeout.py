"""Read-only CP-V22A integrity/identity checks; writes one new closeout receipt."""
import csv
import hashlib
import itertools
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/results/gnssfgo_leica2_paired_receiver_trace_cpv22a_20261027')
CASES = {
    'hesai': Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_full_cpk29_20261023/prepared'),
    'velodyne': Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared'),
}


def read(name):
    return json.loads((H / name).read_text())


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    assert not (H / 'closeout.json').exists(), 'do not overwrite closeout'
    pins = {}
    manifests = {'input_pins.json': read('input_pins.json')}
    for mode in CASES:
        manifests[f'{mode}_analysis_pins.json'] = read(f'{mode}_analysis_pins.json')
    for name in ('numerics.json', 'comparison.json', 'failed_prefix_check.json', 'incremental_publication_check.json'):
        manifests[name] = read(name)['input_sha256']
    for name, manifest in manifests.items():
        for path, expected in manifest.items():
            assert path not in pins or pins[path] == expected, (name, path)
            pins[path] = expected
    for path, expected in pins.items():
        assert sha(path) == expected, ('pin mismatch', path)
    print('Pinned original and analysis inputs unchanged:', len(pins), flush=True)

    complete = read('replay_complete.json')
    assert complete['pass_'] and complete['input_pins'] == len(read('input_pins.json'))
    assert complete['dependency_pins'] == len(read('dependency_pins.json')) == 1665
    # Container-only compiler headers were verified by the completed controller.
    # Do not start a container again merely to rehash them on the host.
    assert (H / 'Receiver.h').read_bytes() == (H.parent / 'leica2_receiver_trace_cpv13_20261026/Receiver.h').read_bytes()
    assert (H / 'Live.cpp').read_bytes() == (H.parent / 'receiver_finalization_cpk19_20261021/Live.cpp').read_bytes()
    binary = read('binary.json')
    assert sha(binary['path']) == binary['sha256']
    assert read('build.json')['returncode'] == 0
    numbers = read('numerics.json')
    comparison = read('comparison.json')
    incremental = read('incremental_publication_check.json')
    expected_lengths = {'hesai': 34.743180, 'velodyne': 41.248984}
    checked = {}
    for mode, prepared in CASES.items():
        assert read(f'{mode}_finish.json')['returncode'] == 0
        parity = read(f'{mode}_parity.json')
        assert parity['pass_']
        original = prepared / 'full_off_receiver/receiver'
        replay = R / mode
        for name, expected in parity['byte_identical_outputs'].items():
            assert sha(original / name) == sha(replay / name) == expected
        assert json.loads((original / 'summary.json').read_text()) == json.loads((replay / 'summary.json').read_text()) == parity['summary']
        assert parity['summary']['publications'] == parity['summary']['finalized'] == 4282
        with (original / 'stream_progress.csv').open() as fa, (replay / 'stream_progress.csv').open() as fb:
            rows = 0
            for a, b in itertools.zip_longest(csv.DictReader(fa), csv.DictReader(fb)):
                assert a is not None and b is not None
                assert a.keys() == b.keys()
                assert all(a[k] == b[k] for k in a if k not in ('processed_wall_ns', 'service_seconds', 'rss_kib')), (mode, rows)
                rows += 1
        assert rows == parity['all_event_metadata_equal']
        with (R / f'{mode}_trace/trace_meta.csv').open() as stream:
            events = {row['event'] for row in csv.DictReader(stream)}
        for event in events:
            relative = Path('archive/commits') / event / 'records.txt'
            assert sha(original / relative) == sha(replay / relative)
        assert len(events) == parity['trace_commits_equal']

        analysis = read(f'{mode}_analysis.json')
        assert analysis['checks']['cycles'] == 595
        assert analysis['checks']['publication_vs_original_max_m'] < 1e-9
        assert analysis['checks']['cycle_vector_identity_max_m'] < 1e-9
        cycles = read(f'{mode}_cycles.json')
        assert len(cycles) == 595
        max_error = 0.0
        for cycle in cycles:
            v = {k: np.asarray(x) for k, x in cycle['vectors'].items()}
            total = sum((x for k, x in v.items() if k not in ('prediction', 'support_add_adjustment')), np.zeros(3))
            err = float(np.linalg.norm(total - cycle['actual']))
            max_error = max(max_error, err)
            assert err < 1e-9
            assert np.linalg.norm(v['support'] - v['prediction'] - v['support_add_adjustment']) < 1e-9
        path = float(sum(np.linalg.norm(row['actual']) for row in cycles))
        reference = float(sum(np.linalg.norm(row['reference']) for row in cycles))
        assert abs(path - expected_lengths[mode]) < 1e-6
        assert abs(reference - 32.607887) < 1e-6
        numeric = numbers['cases'][mode]
        assert numeric['all_jacobian_checks_pass'] and numeric['all_endpoint_checks_pass']
        assert sum(v['block_step_comparisons'] for v in numeric['jacobians'].values()) == 21012
        assert all(v['failed_blocks'] == 0 for v in numeric['jacobians'].values())
        assert len(numeric['linearizations']) == 5
        assert all(v['column_normalized_rank'] == v['cols'] and np.isfinite(v['column_normalized_condition']) for v in numeric['linearizations'])
        assert all(v['rows'] == 586 for v in comparison['cases'][mode]['age_products'].values())
        inc = incremental['cases'][mode]
        assert inc['kimera_incremental_flag_enabled'] and inc['rows'] == 596
        assert inc['control_published_to_published_chain_max_difference_m'] < 1e-9
        assert inc['same_snapshot_edge_length_preservation_max_m'] < 1e-9
        assert inc['published_step_decomposition_max_m'] < 1e-9
        assert abs(inc['absolute_online']['path_m'] - path) < 1e-9
        checked[mode] = {'byte_parity': True, 'event_metadata_rows': rows, 'trace_commit_payloads': len(events),
                         'first60_path_m': path, 'first60_reference_path_m': reference,
                         'vector_identity_max_m': max_error, 'numerics_pass': True}
        print(mode, checked[mode], flush=True)

    failed = read('failed_prefix_check.json')
    assert failed['observer_only_delta_verified'] and failed['velodyne_not_started']
    assert failed['original_model_failure'] is False
    assert all(x['exact_original_prefix'] for x in failed['outputs'].values())
    old = (H.parent / 'leica2_paired_receiver_trace_cpv22_20261027/Diagnostics.h').read_text()
    assert (H / 'Diagnostics.h').read_text() == old.replace(
        'auto& files=AuditFiles(root);std::vector<Matrix> J;',
        'Require(bool(factor),"null numerical audit factor");\n  auto& files=AuditFiles(root);std::vector<Matrix> J(factor->keys().size());')
    containers = read('container_closeout.json')
    assert containers['owned_jobs_exited']
    assert all(not state['Running'] and state['Pid'] == 0 for state in containers['containers'].values())
    checkpoint = H.parent / 'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
    result = subprocess.run(['python3', str(checkpoint), '--originals'], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '4 cases, 667 saved files; originals=True' in result.stdout
    print(result.stdout.strip(), flush=True)
    artifacts = {str(path): sha(path) for path in sorted(H.iterdir()) if path.is_file()}
    receipt = {
        'pass_': True, 'utc': datetime.now(timezone.utc).isoformat(),
        'original_input_pins_rechecked': len(manifests['input_pins.json']),
        'combined_unique_pins_rechecked': len(pins),
        'compiler_dependencies_verified_after_replays_by_controller': complete['dependency_pins'],
        'cases': checked, 'failed_observer_attempt_preserved': True,
        'container_closeout_verified': True,
        'checkpoint_stdout': result.stdout.strip(),
        'figures_visually_inspected': ['paired_update_trace.png', 'paired_state_age.png', 'incremental_publication_check.png'],
        'free_GiB': shutil.disk_usage(R).free / 2**30,
        'artifact_sha256': artifacts,
        'scope': 'Saved original-Qc CBS-OFF diagnosis only; no estimator fix, nonlinear stationarity claim, or next experiment.',
    }
    with (H / 'closeout.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
        stream.write('\n')
    print('PASS closeout', flush=True)


if __name__ == '__main__':
    main()
