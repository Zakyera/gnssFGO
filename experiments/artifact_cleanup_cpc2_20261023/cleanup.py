"""Explicitly scoped, user-authorized removal of superseded trial payloads.

Only the listed case directories are eligible. Keep reports, source/builds,
final trajectories, two small regression fixtures, and every pinned live file.
The deletion manifest is an audit trail, NOT a recoverable content backup.
"""
from pathlib import Path
import datetime
import hashlib
import json
import os
import re
import shutil
import stat
import sys

H = Path(__file__).resolve().parent
BASE = Path('/home/yeranis/results')
CASES = {
    'gnssfgo_eth1_stationary_consensus_20261006': (
        'off_wnoa', 'on_wnoa', 'off_wnoj', 'on_wnoj',
        'off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat',
        'off_wnoa_short', 'on_wnoa_short', 'off_wnoj_short', 'on_wnoj_short'),
    'gnssfgo_eth1_local_prediction_20261007': (
        'off_wnoa', 'on_wnoa', 'off_wnoj', 'on_wnoj',
        'off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat'),
    'gnssfgo_eth1_serial_qr_20261007': (
        'off_wnoa', 'on_wnoa', 'off_wnoj', 'on_wnoj',
        'off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat',
        'off_wnoj_short', 'off_wnoj_short_repeat'),
    'gnssfgo_eth1_bounded_polish_20261007': (
        'off_wnoa', 'on_wnoa', 'off_wnoj', 'on_wnoj',
        'off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat',
        'on_wnoj_short'),
    'gnssfgo_eth1_exact_accel_20261007': (
        'off_wnoj', 'on_wnoj', 'off_wnoj_repeat', 'on_wnoj_repeat'),
}
FIXTURES = {
    BASE/'gnssfgo_eth1_bounded_polish_20261007/on_wnoj/snapshot_1155',
    BASE/'gnssfgo_eth1_local_prediction_20261007/on_wnoj/snapshot_1174',
}
PROTECTED = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared/protected_manifest.json')

def now():
    return datetime.datetime.now().astimezone().isoformat()

def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def save(name, data):
    with (H/name).open('x') as f:
        json.dump(data, f, indent=2)

def metadata(path):
    s = path.lstat()
    assert stat.S_ISREG(s.st_mode) and s.st_nlink == 1, path
    return dict(size=s.st_size, allocated=s.st_blocks*512,
                mtime_ns=s.st_mtime_ns, inode=s.st_ino, device=s.st_dev)

def scan(root):
    assert root.resolve() == root and root.is_dir(), root
    device = root.stat().st_dev
    files = {}
    for parent, dirs, names in os.walk(root, followlinks=False):
        for name in dirs + names:
            path = Path(parent)/name
            s = path.lstat()
            assert s.st_dev == device and not stat.S_ISLNK(s.st_mode), path
            assert stat.S_ISREG(s.st_mode) or stat.S_ISDIR(s.st_mode), path
            if stat.S_ISREG(s.st_mode):
                files[str(path)] = metadata(path)
    return files

def eligible_dir(path):
    return (path.parent.parent.parent == BASE and
            path.parent.parent.name in CASES and
            path.parent.name in CASES[path.parent.parent.name] and
            re.fullmatch(r'snapshot_(?:[0-9]+|final)', path.name) and
            path not in FIXTURES)

def eligible_file(path):
    return (path.parent.parent == BASE/path.parent.parent.name and
            path.parent.parent.name in CASES and
            path.parent.name in CASES[path.parent.parent.name] and
            path.suffix == '.csv')

def plan():
    assert not (H/'plan.json').exists() and not (H/'deletion_started.json').exists()
    protected = json.loads(PROTECTED.read_text())
    targets, singles, deleted, kept, by_root = [], [], {}, {}, {}
    for name, cases in CASES.items():
        root = BASE/name
        selected_dirs, selected_files = set(), set()
        for case_name in cases:
            case = root/case_name
            assert case.is_dir() and case.resolve() == case, case
            for path in case.iterdir():
                assert not path.is_symlink(), path
                if path.is_dir():
                    assert re.fullmatch(r'snapshot_(?:[0-9]+|final)', path.name), path
                    if path not in FIXTURES:
                        assert eligible_dir(path), path
                        selected_dirs.add(path)
                else:
                    assert path.name in ('summary.json', 'trajectory.tum', 'online_trajectory.tum') or path.suffix == '.csv', path
                    if path.suffix == '.csv':
                        assert eligible_file(path), path
                        selected_files.add(path)
        root_files = scan(root)
        for raw, info in root_files.items():
            path = Path(raw)
            rel = path.relative_to(root).parts
            candidate = root.joinpath(*rel[:2])
            if candidate in selected_dirs or path in selected_files:
                assert raw not in protected, ('pinned file', raw)
                deleted[raw] = info
            else:
                kept[raw] = dict(**info, sha256=sha(path))
        targets.extend(str(p) for p in sorted(selected_dirs))
        singles.extend(str(p) for p in sorted(selected_files))
        by_root[str(root)] = dict(
            snapshot_directories=len(selected_dirs), trace_files=len(selected_files),
            removed_allocated_file_bytes=sum(v['allocated'] for p,v in deleted.items() if p.startswith(str(root)+'/')),
            kept_allocated_file_bytes=sum(v['allocated'] for p,v in kept.items() if p.startswith(str(root)+'/')))
    assert all(p.is_dir() for p in FIXTURES)
    assert 20000 < len(targets) < 45000, len(targets)
    save('deleted_file_manifest.json', deleted)
    save('preserved_file_manifest.json', kept)
    result = dict(created=now(), target_directories=targets, target_files=singles,
                  regression_fixtures=sorted(map(str,FIXTURES)), by_root=by_root,
                  deleted_files=len(deleted), kept_files=len(kept),
                  protected_manifest_sha256=sha(PROTECTED), protected_count=len(protected),
                  deleted_manifest_sha256=sha(H/'deleted_file_manifest.json'),
                  kept_manifest_sha256=sha(H/'preserved_file_manifest.json'))
    save('plan.json', result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('target_directories','target_files')}, indent=2), flush=True)

def execute():
    assert not (H/'deletion_started.json').exists(), 'No automatic retry after deletion starts'
    p = json.loads((H/'plan.json').read_text())
    assert sha(PROTECTED) == p['protected_manifest_sha256']
    assert sha(H/'deleted_file_manifest.json') == p['deleted_manifest_sha256']
    assert sha(H/'preserved_file_manifest.json') == p['kept_manifest_sha256']
    deleted = json.loads((H/'deleted_file_manifest.json').read_text())
    kept = json.loads((H/'preserved_file_manifest.json').read_text())
    expected = {**deleted, **kept}
    for name in CASES:
        root = BASE/name
        actual = scan(root)
        prior = {path:{k:v for k,v in info.items() if k != 'sha256'}
                 for path,info in expected.items() if path.startswith(str(root)+'/')}
        assert actual == prior, ('tree changed since inventory', root)
    protected = json.loads(PROTECTED.read_text())
    assert not set(deleted).intersection(protected)
    for path, info in kept.items():
        assert sha(path) == info['sha256'], ('retained file changed', path)
    for raw in p['target_directories']:
        target = Path(raw)
        assert eligible_dir(target) and target.resolve() == target, target
    for raw in p['target_files']:
        target = Path(raw)
        assert eligible_file(target) and target.resolve() == target, target
    before = shutil.disk_usage(BASE).free
    save('deletion_started.json', dict(at=now(), free_bytes=before))
    with (H/'deleted_targets.jsonl').open('x') as journal:
        for raw in p['target_directories']:
            shutil.rmtree(raw)
            journal.write(json.dumps(dict(path=raw, kind='directory'))+'\n')
        for raw in p['target_files']:
            Path(raw).unlink()
            journal.write(json.dumps(dict(path=raw, kind='file'))+'\n')
    assert all(not Path(raw).exists() for raw in p['target_directories']+p['target_files'])
    for path, info in kept.items():
        assert sha(path) == info['sha256'], ('retained file changed', path)
    for path, digest in protected.items():
        assert sha(path) == digest, ('protected file changed', path)
    after = shutil.disk_usage(BASE).free
    result = dict(passed=True, completed=now(), deleted_directories=len(p['target_directories']),
                  deleted_trace_files=len(p['target_files']), total_deleted_files=len(deleted),
                  retained_hashes_verified=len(kept), protected_hashes_verified=len(protected),
                  free_before_bytes=before, free_after_bytes=after,
                  free_space_increase_bytes=after-before,
                  removed_allocated_file_bytes=sum(v['allocated'] for v in deleted.values()),
                  retained_allocated_file_bytes=sum(v['allocated'] for v in kept.values()),
                  recoverability='Permanent deletion. Manifests record paths/metadata, not file contents. Old detailed replay/reanalysis cannot run from these pruned outputs.')
    save('completion.json', result)
    print(json.dumps(result, indent=2), flush=True)

if __name__ == '__main__':
    assert len(sys.argv) == 2 and sys.argv[1] in ('plan', 'execute')
    (plan if sys.argv[1] == 'plan' else execute)()
