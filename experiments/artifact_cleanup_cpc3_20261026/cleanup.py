"""Finite manifest-driven cleanup; prepare -> archive -> apply. No source runs."""
from pathlib import Path
import argparse
import collections
import datetime
import hashlib
import json
import os
import stat
import subprocess
import tarfile

H = Path(__file__).resolve().parent
B = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOTS = tuple(B / n for n in (
    'stage2a_implementation', 'stage2c_b', 'stage2c_c', 'stage2c_d1',
    'stage2c_d2', 'stage2c_medium', 'stage3a_characterization',
    'stage16_diagnosis', 'stage17_diagnosis', 'stage18_diagnosis'))
RUNS = (
    'gnssfgo_eth1_awake_full_cpk23_20261021', 'gnssfgo_eig2_full_cpk24_20261021',
    'gnssfgo_spx3_stationary_full_cpk28_20261023', 'gnssfgo_leica2_full_cpk29_20261023',
    'gnssfgo_spx3_velodyne_full_cpv4_20261024', 'gnssfgo_eig2_velodyne_full_cpv4_20261024',
    'gnssfgo_leica2_velodyne_full_cpv4_20261024',
    'gnssfgo_leica2_rotation_weight_cpv6a_20261024',
    'gnssfgo_leica2_gravity_projection_cpv9_20261026')
ORIGINAL = ROOTS[4] / 'stairs/source_work/stairs_gamma_indexed.bag'
ORIGINAL_SHA = '932a5490497634d8e0497c0013c973d804dc892d37d1c0cab661e20141be514d'
BUILD_PARTS = {'candidate_build', 'control_build', 'candidate_devel', 'control_devel',
               'candidate_logs', 'control_logs', 'overlay_build', 'overlay_devel', 'overlay_logs'}
CODE_PARTS = {'src', 'control_source', 'candidate_source', 'provenance', '.git'}

def now():
    return datetime.datetime.now().astimezone().isoformat()

def digest(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def save(name, data):
    with (H / name).open('x') as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write('\n')

def free_bytes():
    s = os.statvfs(B)
    return s.f_bavail * s.f_frsize

def validate_roots():
    for root in ROOTS:
        assert root.parent == B and root.resolve() == root and root.is_dir(), root
        assert not root.is_symlink() and not os.path.ismount(root), root

def under(path, root):
    return path == root or root in path.parents

def entries():
    for root in ROOTS:
        for directory, dirs, files in os.walk(root, followlinks=False):
            directory = Path(directory)
            assert directory.stat().st_dev == root.stat().st_dev, directory
            for name in sorted(dirs):
                p = directory / name
                if p.is_symlink():
                    yield root, p
            dirs[:] = sorted(d for d in dirs if not (directory / d).is_symlink())
            for name in sorted(files):
                yield root, directory / name

def disposition(root, p):
    parts = p.relative_to(root).parts
    if p == ORIGINAL:
        return 'keep_original_dataset'
    if any(part in CODE_PARTS or part.startswith('safety') for part in parts):
        return 'keep_source_provenance_or_safety'
    if any(part in BUILD_PARTS or part == '__pycache__' for part in parts):
        return 'delete_isolated_generated_build'
    if p.is_symlink():
        return 'keep_symlink'
    if p.suffix == '.bag':
        derived = any(part.startswith('converted') or part in ('source_work_corrupt_resume_20260812',)
                      or part == 'runs' or part.endswith('_runs') or part == 'rerun' for part in parts)
        if p == ORIGINAL.with_name('stairs_gamma_indexed.orig.bag'):
            return 'delete_verified_duplicate_original'
        return 'delete_derived_or_failed_bag' if derived else 'keep_unclassified_bag'
    if p.suffix in ('.rrd', '.gif'):
        return 'delete_old_visualization_recording'
    if p.suffix == '.log' and p.stat().st_size > 65536:
        return 'delete_bulk_runtime_log'
    if p.suffix == '.jsonl' and any(part.startswith('converted') for part in parts):
        return 'delete_per_message_conversion_trace'
    runtime = any(part in ('runs', 'logs', 'saved_recordings') or part.endswith('_runs') for part in parts)
    analysis = any('analysis' in part or 'comparison' in part or 'evaluation' in part or 'screening' in part
                   for part in parts)
    final_table = any(word in p.stem.lower() for word in ('summary', 'result', 'trajectory', 'odom', 'metric'))
    if runtime and not analysis and not final_table and p.suffix in ('.csv', '.jsonl', '.npz', '.ply', '.pcd', '.cereal', '.cereal_lie'):
        return 'delete_per_update_runtime_payload'
    return 'keep_other_report_code_configuration_or_trajectory'

def identity(p):
    s = p.lstat()
    return dict(device=s.st_dev, inode=s.st_ino, mode=s.st_mode, size=s.st_size,
                mtime_ns=s.st_mtime_ns, allocated_bytes=s.st_blocks * 512,
                link=os.readlink(p) if stat.S_ISLNK(s.st_mode) else None)

def protected_paths():
    paths = set()
    for run in RUNS:
        p = B / 'runs' / run / 'prepared/protected_manifest.json'
        pins = json.loads(p.read_text())
        assert isinstance(pins, dict) and all(str(x).startswith('/') for x in pins), p
        paths.update(Path(x) for x in pins)
        paths.add(p)
    closeout = Path('/home/yeranis/repos/gnssFGO/experiments/cross_sequence_decision_cpv8_20261026/closeout.json')
    paths.update(Path(x) for x in json.loads(closeout.read_text())['input_sha256'])
    paths.add(closeout)
    for run in RUNS[:7]:
        for mode in ('off', 'on'):
            paths.add(B / 'runs' / run / f'prepared/full_{mode}_source/glim_odometry.csv')
    return sorted(paths)

def prepare():
    assert not (H / 'inventory.json').exists()
    validate_roots()
    assert digest(ORIGINAL) == ORIGINAL_SHA
    assert digest(ORIGINAL.with_name('stairs_gamma_indexed.orig.bag')) == ORIGINAL_SHA
    inventory = []
    for root, p in entries():
        row = dict(path=str(p), root=str(root), disposition=disposition(root, p), **identity(p))
        assert stat.S_ISREG(row['mode']) or stat.S_ISLNK(row['mode']), p
        if row['disposition'].startswith('keep') and row['link'] is None:
            row['sha256'] = digest(p)
        if row['disposition'].startswith('delete'):
            assert p.parent.resolve() == p.parent and os.access(p.parent, os.W_OK | os.X_OK), p
        inventory.append(row)
    print('Inventory and retention hashes complete', flush=True)
    protected = {}
    for p in protected_paths():
        assert not any(under(p, r) or under(p.resolve(), r) for r in ROOTS), p
        protected[str(p)] = dict(sha256=digest(p), resolved=str(p.resolve()))
    save('protected_before.json', protected)
    save('inventory.json', inventory)
    totals = collections.defaultdict(lambda: dict(files=0, allocated_bytes=0, logical_bytes=0))
    by_root = collections.defaultdict(lambda: dict(delete_files=0, delete_allocated_bytes=0, keep_files=0, keep_allocated_bytes=0))
    for row in inventory:
        t = totals[row['disposition']]
        t['files'] += 1; t['allocated_bytes'] += row['allocated_bytes']; t['logical_bytes'] += row['size']
        prefix = 'delete' if row['disposition'].startswith('delete') else 'keep'
        r = by_root[row['root']]
        r[prefix + '_files'] += 1; r[prefix + '_allocated_bytes'] += row['allocated_bytes']
    save('plan.json', dict(created=now(), roots=list(map(str, ROOTS)), totals=totals,
        per_root=by_root, protected_files=len(protected), free_bytes=free_bytes(),
        archive_excludes_original_dataset=str(ORIGINAL), inventory_sha256=digest(H/'inventory.json')))
    print(json.dumps(dict(per_root=by_root, totals=totals, protected_files=len(protected)), indent=2), flush=True)

def check_identity(row):
    p = Path(row['path'])
    assert any(under(p, r) and p != r for r in ROOTS), p
    assert p.parent.resolve() == p.parent, p
    assert identity(p) == {k: row[k] for k in identity(p)}, p

def archive():
    plan = json.loads((H/'plan.json').read_text())
    assert digest(H/'inventory.json') == plan['inventory_sha256']
    rows = json.loads((H/'inventory.json').read_text())
    kept = [r for r in rows if r['disposition'].startswith('keep') and Path(r['path']) != ORIGINAL]
    dest = H / 'retained_compact_archive.tar.gz'
    with tarfile.open(dest, 'x:gz', compresslevel=3, dereference=False) as tf:
        for row in kept:
            check_identity(row)
            p = Path(row['path'])
            tf.add(p, arcname=str(p.relative_to(B)), recursive=False)
    expected = {str(Path(row['path']).relative_to(B)): row for row in kept}
    with tarfile.open(dest, 'r:gz') as tf:
        count = 0
        for member in tf:
            row = expected.pop(member.name)
            if row['link'] is not None:
                assert member.issym() and member.linkname == row['link'], member.name
            else:
                assert member.isfile(), member.name
                with tf.extractfile(member) as f:
                    assert hashlib.file_digest(f, 'sha256').hexdigest() == row['sha256'], member.name
            count += 1
        assert not expected
    save('archive_verified.json', dict(passed=True, created=now(), members=count,
        archive=str(dest), bytes=dest.stat().st_size, sha256=digest(dest)))
    print('Retained-content archive verified:', count, 'members', dest.stat().st_size, 'bytes', flush=True)

def apply():
    validate_roots()
    assert not (H/'deletion_started.json').exists(), 'Refuse duplicate deletion'
    plan = json.loads((H/'plan.json').read_text())
    assert digest(H/'inventory.json') == plan['inventory_sha256']
    arc = json.loads((H/'archive_verified.json').read_text())
    assert arc['passed'] and digest(arc['archive']) == arc['sha256']
    rows = json.loads((H/'inventory.json').read_text())
    for row in rows:
        check_identity(row)
    assert digest(ORIGINAL) == ORIGINAL_SHA
    before = free_bytes()
    save('deletion_started.json', dict(time=now(), free_bytes=before, inventory_sha256=plan['inventory_sha256']))
    removed = 0
    with (H/'deleted.jsonl').open('x') as log:
        for row in rows:
            if not row['disposition'].startswith('delete'):
                continue
            check_identity(row)
            Path(row['path']).unlink()
            log.write(json.dumps(dict(path=row['path'], size=row['size'], disposition=row['disposition']))+'\n')
            removed += 1
    empty = 0
    for root in ROOTS:
        for directory, dirs, files in os.walk(root, topdown=False, followlinks=False):
            p = Path(directory)
            if p == root or p.is_symlink():
                continue
            try:
                p.rmdir(); empty += 1
            except OSError:
                pass
    protected = json.loads((H/'protected_before.json').read_text())
    for path, expected in protected.items():
        p = Path(path)
        assert str(p.resolve()) == expected['resolved'] and digest(p) == expected['sha256'], p
    kept = 0
    for row in rows:
        if row['disposition'].startswith('keep'):
            p = Path(row['path'])
            if row['link'] is not None:
                assert p.is_symlink() and os.readlink(p) == row['link'], p
            else:
                assert digest(p) == row['sha256'], p
            kept += 1
        else:
            assert not os.path.lexists(row['path']), row['path']
    result = dict(passed=True, completed=now(), removed_files_or_links=removed,
        removed_empty_directories=empty, retained_entries_verified=kept,
        protected_files_verified=len(protected), original_dataset_sha256=ORIGINAL_SHA,
        free_bytes_before_delete=before, free_bytes_after_delete=free_bytes(),
        freed_bytes_during_delete=free_bytes()-before, archive=arc)
    save('completion.json', result)
    print(json.dumps(result, indent=2), flush=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('prepare', 'archive', 'apply'))
    args = parser.parse_args()
    globals()[args.action]()
