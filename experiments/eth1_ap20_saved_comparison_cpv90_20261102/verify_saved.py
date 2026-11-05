"""Supplemental read-only input/association audit; does not rerun EVO scoring."""
from collections import Counter
from pathlib import Path
import importlib.metadata
import subprocess
import sys

sys.dont_write_bytecode = True
import analyze as a


def main():
    assert not (a.H / 'supplemental_verification.json').exists()
    assert not (a.H / 'complete.json').exists()
    r = a.read(a.H / 'results.json')
    audit = a.read(a.P / 'input_audit.json')
    epoch = audit['epoch_ns']
    accepted = a.read(a.OUT / 'accepted_mask.json')
    rejected = a.read(a.OUT / 'rejected_mask.json')
    ap_t, ap_p, _ = a.load_bag(a.AP, '/boxi/ap20/prism_position')
    ap = a.file_interface.read_tum_trajectory_file(a.OUT / 'AP20.tum')
    ids = a.np.asarray([x['raw_AP20_index'] for x in accepted])
    assert len(accepted) == r['common']['rows'] == 3873
    assert a.np.array_equal(a.np.rint(ap.timestamps * 1e9).astype(a.np.int64) + epoch, ap_t[ids])
    assert a.np.max(abs(ap.positions_xyz - ap_p[ids])) < 1e-10
    assert sorted(ids.tolist() + [x['raw_AP20_index'] for x in rejected]) == list(range(len(ap_t)))
    sources = {}
    for name, path in a.SOURCES.items():
        tr = a.file_interface.read_tum_trajectory_file(path)
        sources[name] = (a.np.rint(tr.timestamps * 1e9).astype(a.np.int64) + epoch, tr.positions_xyz)
    ie_t, ie_p, _ = a.load_bag(a.IE, '/boxi/inertial_explorer/tc/odometry',
                              a.np.asarray(audit['T_cpt7_prism'])[:3, 3])
    sources['IE_TC'] = (ie_t, ie_p)
    bracket_errors = {}
    for name, (ts, xyz) in sources.items():
        tr = a.file_interface.read_tum_trajectory_file(a.OUT / (name + '.tum'))
        assert a.np.array_equal(tr.timestamps, ap.timestamps)
        reconstructed = []
        for row in accepted:
            bracket = row[name]
            lo, hi = bracket['left_index'], bracket['right_index']
            assert ts[lo] == bracket['left_ns'] and ts[hi] == bracket['right_ns']
            q = row['timestamp_ns']
            assert ts[lo] <= q <= ts[hi]
            assert ts[hi] - ts[lo] <= (50_000_000 if name == 'IE_TC' else 500_000_000)
            w = (q - int(ts[lo])) / (int(ts[hi]) - int(ts[lo])) if hi != lo else 0.
            assert abs(w - bracket['right_weight']) < 1e-15 and 0 <= w <= 1
            reconstructed.append(xyz[lo] + w * (xyz[hi] - xyz[lo]))
        error = float(a.np.max(abs(a.np.asarray(reconstructed) - tr.positions_xyz)))
        assert error < 1e-10
        bracket_errors[name] = error
    reference_parity = {}
    for name in a.SOURCES:
        old = a.file_interface.read_tum_trajectory_file(a.OLD / 'evaluation' / ('full_' + name) / 'reference.tum')
        q = a.np.rint(old.timestamps * 1e9).astype(a.np.int64) + epoch
        p, valid, *_ = a.sample(ie_t, ie_p, q, 50_000_000)
        assert valid.all()
        error = float(a.np.max(abs(p - old.positions_xyz)))
        assert error < 1e-10
        reference_parity[name] = error
    pins = a.read(a.OUT / 'input_sha256.json')
    assert all(a.sha(path) == value for path, value in pins.items())
    assert a.read(a.H / 'verification.json')['passed']
    import evo
    evo_files = list(Path(evo.__file__).parent.rglob('*.py')) + [Path(a.EVO_APE)]
    containers = subprocess.check_output(['docker', 'inspect', '--format', '{{.Name}} {{.State.Status}}',
                                         'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True)
    assert set(containers.splitlines()) == {'/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'}
    a.save(a.H / 'supplemental_verification.json', {
        'passed': True, 'raw_AP20_and_scoring_reference_match': True,
        'every_raw_row_accepted_or_explicitly_rejected_once': True,
        'interpolation_reconstruction_error_m': bracket_errors,
        'raw_IE_prism_conversion_matches_CP89_saved_references_max_error_m': reference_parity,
        'rejections_by_source': {'+'.join(k): v for k, v in Counter(tuple(x['unsupported_estimators']) for x in rejected).items()},
        'rejections_by_period': dict(Counter('startup_before_4s' if x['relative_s'] < 4 else 'tail_after_302s' for x in rejected)),
        'source_container_status': containers, 'EVO_version': importlib.metadata.version('evo'),
        'EVO_runtime_sha256': {str(p): a.sha(p) for p in evo_files},
        'no_new_scoring_or_estimator_run': True})
    outputs = {str(p): a.sha(p) for p in sorted(a.H.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
    a.save(a.H / 'complete.json', {'passed': True, 'task': 'CP-V90',
                                  'scope': 'saved-data AP20 position-only comparison of CP89',
                                  'output_sha256': outputs})
    print('PASS: association, raw-reference parity, input hashes, stopped containers;', len(outputs), 'outputs pinned.')


if __name__ == '__main__':
    main()
