"""Read-only comparison of saved CP91/95/99 runtime settings; no estimator execution."""
import ast
import datetime
import hashlib
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
E = Path('/home/yeranis/repos/gnssFGO/experiments')
NAMES = {
    '91': 'eth1_split_imu_persistent_on_cpv91_20261102_attempt1',
    '95': 'eth1_split_imu_live_pair_cpv95_20261103_attempt3',
    '99': 'eth1_split_imu_repeat_cpv99_20261103',
}
P = {k: W / 'runs' / name / 'prepared' for k, name in NAMES.items()}
S = {k: p / ('on_source' if k == '91' else 'full_on_source') for k, p in P.items()}
PINS = {}

def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def pin(path):
    path = Path(path)
    PINS[str(path)] = digest(path)
    return path

def read(path):
    return json.loads(pin(path).read_text())

def normalize(text, run):
    # Only remove recorded run-root and source-directory identities here.
    # Ports, environment values, UUIDs and other differences remain visible.
    if not isinstance(text, str):
        return text
    return text.replace(str(P[run]), '<P>').replace(
        '/workspace/cbs_gtsam4.3/runs/' + NAMES[run] + '/prepared', '<P>'
    ).replace('full_on_source', '<SOURCE>').replace('on_source', '<SOURCE>')

def files(root):
    return {str(p.relative_to(root)): digest(pin(p)) for p in sorted(root.rglob('*')) if p.is_file()}

def diff(a, b):
    return {k: {'previous': a.get(k), 'repeat': b.get(k)} for k in sorted(a.keys() | b.keys()) if a.get(k) != b.get(k)}

effective = {k: read(p / 'effective_mode_check.json') for k, p in S.items()}
environments, roslogs, node_args = {}, {}, {}
for run, source in S.items():
    environments[run], roslogs[run], node_args[run] = {}, [], {}
    for log in sorted((source / 'ros_logs').glob('*/roslaunch*.log')):
        if log.parent.name == 'latest':
            continue
        text = pin(log).read_text()
        for line in text.splitlines():
            role = ('kimera' if 'process[kimera_vio_ros/kimera_vio_ros_node-' in line else
                    'glim' if 'process[glim_rosnode-' in line else None)
            if not role:
                continue
            if ': env[{' in line:
                env = ast.literal_eval(line.split(': env[', 1)[1].rsplit(']', 1)[0])
                assert role not in environments[run] or env == environments[run][role]
                environments[run][role] = env
                roslogs[run].append(str(log))
            if ': args[' in line:
                node_args[run][role] = line.split(': args[', 1)[1].rsplit(']', 1)[0]
    assert set(environments[run]) == {'glim', 'kimera'}

profiles = {k: {role: files(source / role) for role in ('kimera_profile', 'glim_config')} for k, source in S.items()}
private = {k: files(P[k] / ('private_build' if k == '91' else 'on/private_build')) for k in P}
map_records = {}
for run, data in effective.items():
    map_records[run] = {}
    for role in ('kimera', 'glim'):
        records = {}
        for line in data[role + '_maps'].splitlines():
            f = line.split(maxsplit=5)
            if len(f) != 6 or not f[5].startswith('/') or f[5].startswith('/dev/'):
                continue
            path = f[5]
            records[path] = {'device': f[3], 'inode': f[4]}
        map_records[run][role] = records

out = {'scope': 'Saved ON runtime comparison: successful CP91 and CP95 versus failed CP99',
       'checked_at': datetime.datetime.now().astimezone().isoformat(),
       'normalization': 'Only exact run roots/source-directory identities. Every remaining difference is recorded.',
       'pairs': {}, 'private_library_sha256': private,
       'candidate_env': {k: v['kimera_candidate_env'] for k, v in effective.items()},
       'runtime_environment_counts': {k: {role: len(env) for role, env in v.items()} for k, v in environments.items()},
       'roslaunch_evidence': roslogs}
for previous in ('91', '95'):
    pair = {}
    pair['profile_differences'] = {role: diff(profiles[previous][role], profiles['99'][role]) for role in profiles['99']}
    pair['profile_counts'] = {role: len(profiles['99'][role]) for role in profiles['99']}
    pair['private_library_differences'] = diff(private[previous], private['99'])
    omitted = {'kimera_maps', 'glim_maps', 'kimera_pid', 'kimera_argv'}
    pair['effective_settings_differences'] = diff(
        {k: v for k, v in effective[previous].items() if k not in omitted},
        {k: v for k, v in effective['99'].items() if k not in omitted})
    a, b = effective[previous]['kimera_argv'], effective['99']['kimera_argv']
    assert len(a) == len(b)
    pair['kimera_argv_count'] = len(a)
    pair['kimera_argv_differences'] = [dict(index=i, previous=normalize(x, previous), repeat=normalize(y, '99'))
        for i, (x, y) in enumerate(zip(a, b)) if normalize(x, previous) != normalize(y, '99')]
    pair['environment_differences'] = {}
    pair['mapped_files'] = {}
    for role in ('kimera', 'glim'):
        pair['environment_differences'][role] = diff(
            {k: normalize(v, previous) for k, v in environments[previous][role].items()},
            {k: normalize(v, '99') for k, v in environments['99'][role].items()})
        amap, bmap = map_records[previous][role], map_records['99'][role]
        common = set(amap) & set(bmap)
        pair['mapped_files'][role] = dict(
            previous_count=len(amap), repeat_count=len(bmap),
            removed=[normalize(x, previous) for x in sorted(set(amap) - set(bmap))],
            added=[normalize(x, '99') for x in sorted(set(bmap) - set(amap))],
            common_path_inode_differences={p: [amap[p]['inode'], bmap[p]['inode']] for p in sorted(common) if amap[p]['inode'] != bmap[p]['inode']},
            mount_device_changes=sorted({(amap[p]['device'], bmap[p]['device']) for p in common if amap[p]['device'] != bmap[p]['device']}))
    pair['resolved_parameter_text_differences'] = {}
    for filename in ('resolved_kimera_params.yaml', 'resolved_glim_params.yaml'):
        old = normalize(pin(S[previous] / filename).read_text(), previous)
        new = normalize(pin(S['99'] / filename).read_text(), '99')
        import difflib
        pair['resolved_parameter_text_differences'][filename] = list(difflib.unified_diff(old.splitlines(), new.splitlines(), lineterm=''))
    out['pairs'][previous + '_vs_99'] = pair

# Fresh independent comparison of complete CP95/99 static inputs, not a read of the old parity pass flag.
static = {}
for rel in read(E / NAMES['99'] / 'profile_parity.json')['relative_files']:
    a, b = pin(P['95'] / rel), pin(P['99'] / rel)
    same_bytes = a.read_bytes() == b.read_bytes()
    record = {'same_bytes': same_bytes, 'same_mode': (a.stat().st_mode & 0o777) == (b.stat().st_mode & 0o777)}
    if not same_bytes:
        old, new = a.read_text(), b.read_text()
        expected = old.replace(NAMES['95'], NAMES['99'])
        if a.name in ('source.env', 'run_case_on.py', 'full_off.json', 'full_on.json'):
            expected = expected.replace('11601', '11611').replace('11602', '11612')
        if a.name in ('full_off.json', 'full_on.json'):
            expected = expected.replace('CP-V95', 'CP-V99').replace('eth1_cpv95_', 'eth1_cpv99_')
        record['only_explicit_run_identity_changes'] = expected == new
        record['diff'] = list(difflib.unified_diff(old.splitlines(), new.splitlines(), lineterm=''))
    static[rel] = record
out['static_95_vs_99'] = static

# Match runtime workspace libraries to recorded historical hashes where available.
historical_pins = {
    '91': read(P['91'] / 'protected_manifest.json'),
    '95': read(E / 'eth1_split_imu_live_pair_cpv95_20261103_continuation1/protected_manifest.json'),
    '99': read(P['99'] / 'protected_manifest.json'),
}
loaded = {}
for run, roles in map_records.items():
    loaded[run] = {}
    for role, paths in roles.items():
        loaded[run][role] = {}
        for path in sorted(paths):
            prefix = '/workspace/cbs_gtsam4.3/'
            if not path.startswith(prefix):
                continue
            host = W / path[len(prefix):]
            if host.is_file():
                actual = digest(pin(host))
                expected = historical_pins[run].get(str(host)) or historical_pins[run].get(str(host.resolve()))
                if expected:
                    assert actual == expected, (run, host)
                loaded[run][role][path] = dict(current_sha256=actual, historical_pin=expected,
                    historical_bytes_verified=expected is not None)
out['workspace_loaded_files'] = loaded
out['limits'] = ['Mapped path/inode agreement alone does not prove historical bytes for unpinned dependencies.',
                 'OS scheduling, thread interleavings, runtime floating-point mode, GPU state and full historical machine load were not captured.',
                 'No new source, receiver, optimizer, evaluation, installation, configuration change or retry.']
for path, expected in PINS.items():
    assert digest(path) == expected, path
out['input_sha256'] = PINS
with (OUT / 'audit.json').open('x') as stream:
    json.dump(out, stream, indent=2)
print(json.dumps({
    'pairs': {k: {n: v[n] for n in ('profile_differences', 'profile_counts', 'private_library_differences', 'effective_settings_differences', 'kimera_argv_differences', 'resolved_parameter_text_differences')} for k, v in out['pairs'].items()},
    'static_count': len(static), 'static_unexplained': [k for k, v in static.items() if not v['same_mode'] or not (v['same_bytes'] or v.get('only_explicit_run_identity_changes'))],
    'input_hashes': len(PINS), 'output': str(OUT / 'audit.json')}, indent=2))
