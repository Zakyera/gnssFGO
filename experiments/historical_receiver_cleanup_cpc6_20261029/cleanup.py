"""Finite CP-C6 inventory and cleanup; old receiver outputs only, never datasets.

Unknown content is retained. The inventory is not a backup. No experiment runs.
"""
from pathlib import Path
import collections
import datetime
import gzip
import hashlib
import json
import os
import re
import stat
import subprocess
import shutil
import sys

H = Path(__file__).resolve().parent
BASE = Path('/home/yeranis/results')
REPO = Path('/home/yeranis/repos/gnssFGO')
PIN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_on_refresh_smoke_cpv41_20261029/prepared/protected_manifest.json')
# Exact folders visible in the user's blue selection. Off-screen selections
# are not inferred, and current/full campaign and Leica ablation roots excluded.
ROOT_NAMES = '''gnssfgo_stock_ct_isam2_ac_opensky_20260921
gnssfgo_fixed_lag_slots_fl3i_20261014
gnssfgo_fixed_lag_native_fl3h_20261014
gnssfgo_fixed_lag_replay_fl3_20261013
gnssfgo_fixed_lag_failure_fl4e_20261014
gnssfgo_eth1_qr_recovery_20261006
gnssfgo_fixed_lag_failure_fl3a_20261013
gnssfgo_fixed_lag_complete_update_fl3e_20261013
gnssfgo_fixed_lag_isam_convergence_fl3d_20261013
gnssfgo_fixed_lag_incremental_fl3f_20261014
gnssfgo_fixed_lag_live_fl4v_20261017
gnssfgo_eth1_stable_cost_20261006
gnssfgo_eth1_wnoa_wnoj_120s_20261006
gnssfgo_eig2_live60_fl4x_20261017
gnssfgo_spx3_live60_fl4w_20261017
gnssfgo_spx3_local_cov60_cpr5_20261019
gnssfgo_eth1_local_cov60_cpr6_20261019
gnssfgo_fixed_lag_slot_audit_fl3j_20261014
gnssfgo_spx3_60s_graph_inspect_20260928
gnssfgo_spx3_full_system_revisions_20260927
gnssfgo_eth1_roundoff_40s_20261006
gnssfgo_linear_completion_s5e_20261013
gnssfgo_source_wnoj_full_relinearization_20261006
gnssfgo_eth1_motion_weight_sensitivity_20261002
gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002
gnssfgo_receiver_sampling_s5c_20261013
gnssfgo_eth1_huber_off_on30_20261007
gnssfgo_eth1_wnoa_prediction_audit_20261007
gnssfgo_eig2_local_cov60_on_cpr3_20261019
gnssfgo_eig2_local_cov60_off_cpr4_20261019'''.splitlines()
ROOTS = [BASE / name for name in ROOT_NAMES]
PATTERN = re.compile(r'(?:snapshot_([0-9]+))|(?:([0-9]+))')
RETAIN_TREE = re.compile(r'(?:test|saniti|fixture|build|runtime|source|checkpoint|regression)', re.I)
ALLOWED = set('''records.txt summary.csv incremental_states.csv incremental_errors.csv
ledger.csv batch_errors.csv batch_states.csv factor_payload_probe.txt
full_backsub_errors.csv full_backsub_linear_health.csv full_backsub_states.csv
incremental_dense_poses.csv incremental_linear_health.csv iterations.csv
polished_dense_poses.csv polished_errors.csv polished_linear_health.csv
polished_states.csv profile_validation.csv source_residuals.csv source_weights.csv'''.split())

def now():
    return datetime.datetime.now().astimezone().isoformat()

def eligible(path):
    p = Path(path)
    return (p.is_absolute() and p != BASE and p.is_relative_to(BASE)
            and p.relative_to(BASE).parts[0] in ROOT_NAMES
            and PATTERN.fullmatch(p.name)
            and (p.name.startswith('snapshot_') or p.parent.name == 'commits'))

def pinned_paths():
    pins = json.loads(PIN.read_text())
    paths = set(pins)
    # Honor paths copied into all saved checkpoints, not only the latest run.
    cmd = ['rg', '--files', str(REPO / 'experiments/checkpoints'),
           '-g', '*manifest*.json', '-g', '*protect*.json', '-g', '*pins*.json']
    result = subprocess.run(cmd, text=True, capture_output=True, check=True)
    def strings(obj):
        if isinstance(obj, str):
            if obj.startswith('/home/yeranis/'):
                paths.add(obj)
        elif isinstance(obj, dict):
            for k, v in obj.items():
                strings(k); strings(v)
        elif isinstance(obj, list):
            for v in obj:
                strings(v)
    for name in result.stdout.splitlines():
        strings(json.loads(Path(name).read_text()))
    paths |= {str(Path(p).resolve()) for p in list(paths) if len(p) < 4096}
    return pins, paths

def protected_audit(pins):
    records = {}
    for i, (name, expected) in enumerate(pins.items()):
        actual = sha(name)
        assert actual == expected, ('protected content changed', name)
        records[name] = dict(sha256=actual, resolved=str(Path(name).resolve()))
        if i and i % 1500 == 0:
            print('verified pinned files', i, flush=True)
    return records

def leaf(path):
    info = meta(path)
    assert stat.S_ISDIR(info['mode']), path
    children = {}
    for child in Path(path).iterdir():
        m = meta(child)
        # Refuse unrecognized, linked or nested payloads. Leave those intact.
        if child.name not in ALLOWED or not stat.S_ISREG(m['mode']) or m['links'] != 1:
            return None
        children[child.name] = m
    return dict(path=str(path), directory=info, files=children)

def plan():
    assert not (H / 'plan.json').exists() and not (H / 'deletion_started.json').exists()
    d = json.loads((H / 'discovery.json').read_text())
    assert sha(PIN) == d['protected_manifest_sha256']
    pins, paths = pinned_paths()
    ancestors = paths | {str(a) for p in paths if len(p) < 4096 for a in Path(p).parents}
    protected = protected_audit(pins)
    save('protected_before.json', protected)
    targets = set()
    by_root = collections.defaultdict(lambda: dict(directories=0, files=0, allocated_bytes=0))
    skipped = []
    # One gzip record per exact leaf directory, with every child file identity.
    with gzip.open(H / 'deletion_inventory.jsonl.gz', 'xt') as output:
        for group in d['groups']:
            for name in group['selected']:
                p = Path(name)
                assert eligible(p) and p.resolve() == p, p
                if name in ancestors:
                    skipped.append(dict(path=name, reason='checkpoint/current protected path'))
                    continue
                item = leaf(p)
                if item is None:
                    skipped.append(dict(path=name, reason='unknown content retained'))
                    continue
                targets.add(name)
                output.write(json.dumps(item, separators=(',', ':')) + '\n')
                root = p.relative_to(BASE).parts[0]
                c = by_root[root]
                c['directories'] += 1
                c['files'] += len(item['files'])
                c['allocated_bytes'] += 512 * (item['directory']['blocks'] + sum(v['blocks'] for v in item['files'].values()))
            print('inventoried', group['parent'], flush=True)
    retained = {}
    for root in ROOTS:
        for parent, dirs, names in os.walk(root, followlinks=False):
            for n in list(dirs):
                p = Path(parent) / n
                if str(p) in targets:
                    dirs.remove(n)
                elif p.is_symlink():
                    retained[str(p)] = dict(metadata=meta(p), link=os.readlink(p))
                    dirs.remove(n)
            for n in names:
                p = Path(parent) / n
                m = meta(p)
                if stat.S_ISLNK(m['mode']):
                    retained[str(p)] = dict(metadata=m, link=os.readlink(p))
                else:
                    assert stat.S_ISREG(m['mode']), p
                    retained[str(p)] = dict(metadata=m, sha256=sha(p))
        print('hashed retained files', root.name, flush=True)
    save('retained_before.json', retained)
    livox = {}
    dataset = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour')
    for p in sorted(dataset.glob('**/*livox.bag')):
        livox[str(p)] = meta(p)
    assert len(livox) == 9
    save('livox_before.json', livox)
    save('skipped.json', skipped)
    result = dict(created=now(), scope=d['roots'], by_root=by_root,
                  target_directories=len(targets), target_files=sum(v['files'] for v in by_root.values()),
                  allocated_bytes=sum(v['allocated_bytes'] for v in by_root.values()),
                  retained_files=len(retained), protected_files=len(protected),
                  inventory_sha256=sha(H / 'deletion_inventory.jsonl.gz'),
                  retained_sha256=sha(H / 'retained_before.json'),
                  pins_sha256=sha(PIN), protected_sha256=sha(H / 'protected_before.json'),
                  discovery_sha256=sha(H / 'discovery.json'), skipped=len(skipped))
    save('plan.json', result)
    print(json.dumps(result, indent=2), flush=True)

def verify_retained():
    records = json.loads((H / 'retained_before.json').read_text())
    for p, record in records.items():
        assert meta(p) == record['metadata'], ('retained identity changed', p)
        if 'sha256' in record:
            assert sha(p) == record['sha256'], ('retained content changed', p)
        else:
            assert os.readlink(p) == record['link'], p
    for p, m in json.loads((H / 'livox_before.json').read_text()).items():
        assert meta(p) == m, ('Livox changed', p)
    return len(records)

def inventory():
    with gzip.open(H / 'deletion_inventory.jsonl.gz', 'rt') as f:
        for line in f:
            yield json.loads(line)

def execute():
    assert not (H / 'deletion_started.json').exists(), 'No blind retry after deletion begins'
    p = json.loads((H / 'plan.json').read_text())
    for filename, key in [('deletion_inventory.jsonl.gz', 'inventory_sha256'),
                          ('retained_before.json', 'retained_sha256'),
                          ('protected_before.json', 'protected_sha256'),
                          ('discovery.json', 'discovery_sha256')]:
        assert sha(H / filename) == p[key], filename
    assert sha(PIN) == p['pins_sha256']
    # Validate ALL exact targets before removing any. Recheck each immediately
    # before unlinking as well. Never recursively remove an inferred directory.
    for i, item in enumerate(inventory()):
        target = Path(item['path'])
        assert eligible(target) and target.resolve() == target
        assert leaf(target) == item, ('target changed', target)
        if i and i % 100000 == 0:
            print('preflight targets', i, flush=True)
    count = verify_retained()
    before_pins = json.loads((H / 'protected_before.json').read_text())
    assert protected_audit(json.loads(PIN.read_text())) == before_pins
    before = shutil.disk_usage(BASE).free
    save('deletion_started.json', dict(at=now(), free_bytes=before, targets=p['target_directories']))
    files = dirs = 0
    with gzip.open(H / 'deleted.jsonl.gz', 'xt') as log:
        for item in inventory():
            target = Path(item['path'])
            assert leaf(target) == item and target.resolve() == target
            # Only explicitly inventoried regular files, then the empty leaf.
            for name, expected in item['files'].items():
                child = target / name
                assert meta(child) == expected and child.name in ALLOWED
                child.unlink()
                files += 1
            target.rmdir()
            dirs += 1
            log.write(json.dumps(dict(path=str(target), files=len(item['files']))) + '\n')
            if dirs % 50000 == 0:
                log.flush()
                print('removed leaf snapshots', dirs, 'files', files, flush=True)
    assert dirs == p['target_directories'] and files == p['target_files']
    for item in inventory():
        assert not os.path.lexists(item['path']), item['path']
    assert verify_retained() == count
    assert protected_audit(json.loads(PIN.read_text())) == before_pins
    after = shutil.disk_usage(BASE).free
    result = dict(passed=True, completed=now(), deleted_files=files, deleted_leaf_directories=dirs,
                  retained_hashes_verified=count, protected_hashes_verified=len(before_pins),
                  livox_metadata_verified=9, free_before_bytes=before, free_after_bytes=after,
                  free_space_increase_bytes=after-before, recovery='Permanent; no Trash or new content backup')
    save('completion.json', result)
    print(json.dumps(result, indent=2), flush=True)

def save(name, value):
    with (H / name).open('x') as f:
        json.dump(value, f, indent=2)

def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def meta(path):
    s = Path(path).lstat()
    return dict(size=s.st_size, blocks=s.st_blocks, mode=s.st_mode,
                inode=s.st_ino, device=s.st_dev, mtime=s.st_mtime_ns, links=s.st_nlink)

def numbers_referenced():
    # Preserve explicitly cited numeric fixtures even if the reference is
    # relative/constructed. This intentionally over-retains across all roots.
    cmd = ['rg', '--no-heading', '-o', r'snapshot_[0-9]+|commits/[0-9]+',
           '-g', '*.py', '-g', '*.h', '-g', '*.cpp', '-g', '*.sh',
           '-g', '*.txt', '-g', '*.md', str(REPO / 'experiments')]
    p = subprocess.run(cmd, text=True, capture_output=True)
    assert p.returncode in (0, 1), p.stderr
    ids = {int(x) for x in re.findall(r'(?:snapshot_|commits/)([0-9]+)', p.stdout)}
    # Failure/retirement boundaries explicitly discussed in engineering reports.
    ids.update([8128, 8129, 84957, 84958, 84959, 1155, 1174])
    return ids, p.stdout

def discover():
    assert not (H / 'discovery.json').exists()
    pins = json.loads(PIN.read_text())
    protected = set(pins) | {str(Path(p).resolve()) for p in pins}
    protected_ancestors = protected | {str(a) for p in protected for a in Path(p).parents}
    ids, refs = numbers_referenced()
    groups, excluded = [], []
    for root in ROOTS:
        assert root.is_dir() and root.resolve() == root, root
        def visit(parent, depth=0):
            entries = sorted(parent.iterdir())
            numbered = []
            for child in entries:
                if child.is_symlink() or not child.is_dir():
                    continue
                m = PATTERN.fullmatch(child.name)
                if m and (child.name.startswith('snapshot_') or parent.name == 'commits'):
                    numbered.append((int(m.group(1) or m.group(2)), child))
            if numbered:
                numbers = sorted(n for n, _ in numbered)
                # Preserve first/last two and all referenced checkpoint IDs.
                keep = set(numbers[:2] + numbers[-2:]) | ids
                selected, retained = [], []
                for n, child in numbered:
                    is_pinned = str(child) in protected_ancestors
                    if n in keep or is_pinned or len(numbers) < 20:
                        retained.append(str(child))
                    else:
                        selected.append(str(child))
                groups.append(dict(parent=str(parent), total=len(numbers),
                                   selected=selected, retained=retained))
            numbered_paths = {p for _, p in numbered}
            for child in entries:
                if child in numbered_paths or child.is_symlink() or not child.is_dir():
                    continue
                if RETAIN_TREE.search(child.name):
                    excluded.append(str(child))
                    continue
                if depth < 6:
                    visit(child, depth + 1)
        visit(root)
        print('discovered', root.name, flush=True)
    # Store exact reference matches for review, not an inferred deletion list.
    save('reference_ids.json', sorted(ids))
    save('reference_matches.json', refs.splitlines())
    save('discovery.json', dict(roots=list(map(str, ROOTS)), groups=groups,
                               excluded_subtrees=excluded, protected_manifest_sha256=sha(PIN)))
    for root in ROOTS:
        g = [g for g in groups if g['parent'].startswith(str(root) + '/')]
        print(root.name, 'groups', len(g), 'eligible', sum(len(v['selected']) for v in g),
              'retained', sum(len(v['retained']) for v in g), flush=True)
    print('explicitly referenced IDs retained:', len(ids), flush=True)

if __name__ == '__main__':
    assert len(sys.argv) == 2 and sys.argv[1] in ('discover', 'plan', 'execute')
    dict(discover=discover, plan=plan, execute=execute)[sys.argv[1]]()
