"""User-authorized pruning of explicitly listed old repeat snapshots only."""
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
ROOTS = (
    Path('/home/yeranis/results/gnssfgo_eth1_bounded_polish_20261007'),
    Path('/home/yeranis/results/gnssfgo_eth1_serial_qr_20261007'),
)
CASES = {
    str(ROOTS[0]): ('off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat'),
    str(ROOTS[1]): ('off_wnoa_repeat', 'on_wnoa_repeat', 'off_wnoj_repeat', 'on_wnoj_repeat', 'off_wnoj_short_repeat'),
}
PROTECTED = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared/protected_manifest.json')

def now():
    return datetime.datetime.now().astimezone().isoformat()

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def save(name, data):
    with (H/name).open('x') as f:
        json.dump(data, f, indent=2)

def safe_tree(root):
    assert root.resolve() == root and root.is_dir() and not root.is_symlink(), root
    device = root.stat().st_dev
    result = {}
    for d, dirs, files in os.walk(root, followlinks=False):
        for name in dirs + files:
            p = Path(d)/name
            s = p.lstat()
            assert s.st_dev == device and not stat.S_ISLNK(s.st_mode), p
            assert stat.S_ISDIR(s.st_mode) or stat.S_ISREG(s.st_mode), p
            if p.is_file():
                assert s.st_nlink == 1, ('hardlink: stop for review', p)
                result[str(p)] = dict(size=s.st_size, allocated=s.st_blocks*512, sha256=sha(p))
    return result

def check_protected():
    protected = json.loads(PROTECTED.read_text())
    for p, expected in protected.items():
        assert sha(p) == expected, ('protected file differs', p)
    return protected

def plan():
    assert not (H/'plan.json').exists() and not (H/'deletion_started.json').exists()
    protected = check_protected()
    targets, kept_last, files, keep, by_root = [], [], {}, {}, {}
    for root in ROOTS:
        protected_here = [Path(p) for p in protected if p.startswith(str(root)+'/')]
        selected = set()
        for case_name in CASES[str(root)]:
            case = root/case_name
            assert case.is_dir() and case.resolve() == case, case
            snapshots = sorted((p for p in case.iterdir() if re.fullmatch(r'snapshot_[0-9]+', p.name)),
                               key=lambda p: int(p.name.split('_')[1]))
            assert len(snapshots) > 1
            kept_last.append(str(snapshots[-1]))
            selected.update(snapshots[:-1])
        all_files = safe_tree(root)
        for p, info in all_files.items():
            components = Path(p).relative_to(root).parts
            candidate = root.joinpath(*components[:2])
            (files if candidate in selected else keep)[p] = info
        for target in sorted(selected):
            assert target.is_dir() and target.resolve() == target
            assert not any(p.is_relative_to(target) for p in protected_here), target
            targets.append(str(target))
        by_root[str(root)] = dict(directories=len(selected),
            allocated_file_bytes=sum(v['allocated'] for p,v in files.items() if Path(p).is_relative_to(root)))
    assert len(targets) > 9000 and len(targets) < 10000, len(targets)
    save('deleted_file_manifest.json', files)
    save('preserved_file_manifest.json', keep)
    result = dict(created=now(), target_directories=targets, preserved_last_numeric_snapshots=kept_last,
                  by_root=by_root, files=len(files), preserved_files=len(keep),
                  protected_manifest=str(PROTECTED), protected_manifest_sha256=sha(PROTECTED),
                  protected_files=len(protected), free_bytes=shutil.disk_usage(ROOTS[0]).free,
                  deletion_file_manifest_sha256=sha(H/'deleted_file_manifest.json'),
                  preserved_file_manifest_sha256=sha(H/'preserved_file_manifest.json'))
    save('plan.json', result)
    print(json.dumps({k:v for k,v in result.items() if k != 'target_directories'}, indent=2), flush=True)

def execute():
    assert not (H/'deletion_started.json').exists(), 'Never automatically retry a partial cleanup'
    p = json.loads((H/'plan.json').read_text())
    assert sha(PROTECTED) == p['protected_manifest_sha256']
    assert sha(H/'deleted_file_manifest.json') == p['deletion_file_manifest_sha256']
    assert sha(H/'preserved_file_manifest.json') == p['preserved_file_manifest_sha256']
    deleted = json.loads((H/'deleted_file_manifest.json').read_text())
    preserved = json.loads((H/'preserved_file_manifest.json').read_text())
    check_protected()
    expected_groups = {path: {} for path in p['target_directories']}
    for path, info in deleted.items():
        parent = Path(path).parent
        while str(parent) not in expected_groups:
            assert parent not in ROOTS and parent != parent.parent, path
            parent = parent.parent
        expected_groups[str(parent)][path] = info
    for path in p['target_directories']:
        t = Path(path)
        assert t.parent.parent in ROOTS and t.parent.name in CASES[str(t.parent.parent)]
        assert re.fullmatch(r'snapshot_[0-9]+', t.name) and t.resolve() == t
        actual = safe_tree(t)
        expected = expected_groups[path]
        assert actual == expected, ('target changed', t)
    before = shutil.disk_usage(ROOTS[0]).free
    save('deletion_started.json', dict(at=now(), free_bytes=before, targets=len(p['target_directories'])))
    # Targets are individual, preinventoried snapshot directories, never a broad root.
    with (H/'deleted_directories.jsonl').open('x') as journal:
        for path in p['target_directories']:
            shutil.rmtree(path)
            journal.write(json.dumps(dict(path=path, deleted_at=now()))+'\n')
            journal.flush()
    assert all(not Path(path).exists() for path in p['target_directories'])
    for path, info in preserved.items():
        assert sha(path) == info['sha256'], ('preserved file changed', path)
    assert all(Path(path).is_dir() for path in p['preserved_last_numeric_snapshots'])
    protected = check_protected()
    after = shutil.disk_usage(ROOTS[0]).free
    result = dict(passed=True, completed=now(), deleted_directories=len(p['target_directories']),
                  deleted_files=len(deleted), preserved_files_checked=len(preserved),
                  protected_files_checked=len(protected), free_before_bytes=before, free_after_bytes=after,
                  free_space_increase_bytes=after-before,
                  selected_allocated_file_bytes=sum(v['allocated'] for v in deleted.values()),
                  recoverability='Permanent deletion; hashes/path manifest is not a content backup. Primary runs and final snapshots retained.')
    save('completion.json', result)
    print(json.dumps(result, indent=2), flush=True)

if __name__ == '__main__':
    assert len(sys.argv) == 2 and sys.argv[1] in ('plan', 'execute')
    (plan if sys.argv[1] == 'plan' else execute)()
