#!/usr/bin/env python3
"""One-shot archival copy of FL4v; never starts or edits an experiment."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R = W / 'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared'
T = W / 'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
Q = W / 'runs/fixed_lag_current_chart_fl4q_20261016'
G = W / 'runs/shared_linearization_s4_provider_20261012/release_2'
RESULTS = Path('/home/yeranis/results')
DEST = HERE / 'snapshot'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    if DEST.exists() or (HERE / 'manifest.json').exists():
        raise SystemExit('Refusing to overwrite an existing checkpoint')
    planned = {}

    def add(source, target):
        source = Path(source)
        assert source.is_file(), source
        assert source.stat().st_size < 4_000_000, source
        target = Path(target)
        assert target not in planned, target
        planned[target] = source

    def tree(root, target, suffixes=None):
        for source in sorted(root.rglob('*')):
            if source.is_file() and '__pycache__' not in source.parts:
                if suffixes is None or source.suffix in suffixes:
                    add(source, Path(target) / source.relative_to(root))

    for source in sorted(R.iterdir()):
        if source.is_file() and source.suffix in {'.py', '.sh', '.launch', '.json', '.env', '.txt'}:
            add(source, Path('launch') / source.name)
    tree(R / 'transport_tools', 'launch/transport_tools', {'.py'})
    for profile in ('glim_config', 'kimera_profile'):
        tree(R / 'live120_on_source' / profile, Path('profiles') / profile)
    for name in ('effective_mode_check.json', 'source_coverage.json',
                 'run_metadata.txt', 'candidate_validation.json',
                 'resolved_glim_params.yaml', 'resolved_kimera_params.yaml',
                 'acceptance.csv', 'objective.csv'):
        add(R / 'live120_on_source' / name, Path('evidence/source') / name)
    for name in ('resolved_config.json', 'transport_complete.json', 'transport_status.json'):
        add(R / 'live120_on_receiver' / name, Path('evidence/receiver') / name)
    add(R / 'live120_on_receiver/receiver/summary.json', 'evidence/receiver/summary.json')
    add(R / 'live120_on_control/complete.json', 'evidence/complete.json')
    add(R / 'evaluation_full/summary.json', 'evidence/evaluation_full.json')
    for name in ('validation.json', 'repeat_validation.json'):
        add(RESULTS / 'gnssfgo_fixed_lag_live_fl4v_20261017/on' / name,
            Path('evidence') / name)
    add(R.parent / 'common_prefix_off_on.py', 'evaluation/common_prefix_off_on.py')
    # Historical OFF controller, not a claim of a completed 120 s OFF capture.
    for source in sorted(T.iterdir()):
        if source.is_file() and source.suffix in {'.py', '.sh', '.launch', '.json', '.env', '.txt'}:
            add(source, Path('matched_off_launch') / source.name)

    for source in sorted(Q.iterdir()):
        if source.is_file() and source.suffix in {'.h', '.cpp', '.py', '.json', '.diff', '.txt'}:
            add(source, Path('kimera_fl4q') / source.name)
    tree(Q / 'include', 'kimera_fl4q/include', {'.h'})
    for directory in ('source', 'overlay'):
        tree(G / directory, Path('glim_s4') / directory, {'.h', '.cpp', '.py'})
    for name in ('commands.json', 'build_result.json', 'protected.json'):
        add(G / name, Path('glim_s4') / name)
    for directory in ('gnssfgo_e1_e2_correctness_audit_20261011/candidate',
                      'gnssfgo_vw_jacobian_20261011/build',
                      'gnssfgo_publication_cache_20261010/build'):
        root = RESULTS / directory
        for source in sorted(root.iterdir()):
            if source.is_file() and source.suffix in {'.h', '.cpp'}:
                add(source, Path('receiver_dependencies') / directory / source.name)
    for name in ('gnssfgo_fixed_lag_live_fl4a_20261014/build.json',
                 'gnssfgo_fixed_lag_slots_fl3i_20261014/build_replay.json'):
        add(RESULTS / name, Path('receiver_build') / name)

    # Actual loaded libraries remain local. Container-absolute symlinks need
    # translation to their host mount, not omission from provenance.
    def host_resolve(path):
        while path.is_symlink():
            linked = path.readlink()
            path = linked if linked.is_absolute() else path.parent / linked
            text = str(path)
            if text.startswith('/workspace/cbs_gtsam4.3/'):
                path = W / text.removeprefix('/workspace/cbs_gtsam4.3/')
        return path

    binary_paths = [R / 'private_build/libkimera_vio.so',
                    RESULTS / 'gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live']
    binary_paths += sorted(G.glob('*.so*'))
    binary_records = []
    for path in binary_paths:
        resolved = host_resolve(path)
        binary_records.append(dict(path=str(path), resolved=str(resolved),
                                   sha256=digest(resolved), size=resolved.stat().st_size))
    assert binary_records[0]['sha256'] == '6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
    assert binary_records[1]['sha256'] == '53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'

    records = []
    for target, source in sorted(planned.items()):
        destination = DEST / target
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        destination.chmod(source.stat().st_mode & 0o777)
        checksum = digest(source)
        assert digest(destination) == checksum
        records.append(dict(archived=str(destination.relative_to(HERE)),
                            original=str(source), sha256=checksum,
                            size=source.stat().st_size))
    manifest = dict(schema=1, checkpoint='eth1-fl4v-live120-cbs-on-20261017',
                    files=records, local_binaries=binary_records,
                    note='Byte copies only. No experiment, compilation or tuning. '
                         'Original scripts contain historical absolute paths; do not run in place.')
    (HERE / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(files=len(records), bytes=sum(x['size'] for x in records),
                          binaries_hashed=len(binary_records))))


if __name__ == '__main__':
    main()
