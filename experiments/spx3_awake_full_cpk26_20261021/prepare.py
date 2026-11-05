"""Fresh user-authorized CP-K25 restart; identities and scoped sleep checks only."""
from pathlib import Path
import ast, difflib, hashlib, importlib.util, json, re, shutil, socket, subprocess
import xml.etree.ElementTree as ET

H = Path(__file__).resolve().parent
OLD = H.parent/'spx3_full_cpk25_20261021'
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S = W/'runs/gnssfgo_spx3_full_cpk25_20261021/prepared'
P = W/'runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared'
V = Path('/home/yeranis/results/gnssfgo_spx3_awake_full_cpk26_20261021')

def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def rebase(value):
    for old, new in [(S.parent.name, P.parent.name), (OLD.name, H.name),
                     ('spx3_cpk25_full', 'spx3_cpk26_full'),
                     ('SPX3-CPK25', 'SPX3-CPK26'), ('CP-K25', 'CP-K26'),
                     ('port11475', 'port11476')]:
        value = value.replace(old, new)
    return re.sub(r'(?<![A-Za-z0-9_])11475(?![A-Za-z0-9_])', '11476', value)

assert not P.parent.exists() and not V.exists(), 'Fresh roots only'
assert not (H/'status.json').exists()
assert json.loads((OLD/'closeout.json').read_text())['passed']
assert json.loads((OLD/'status.json').read_text())['state'] == 'failed'
assert shutil.disk_usage(W).free > 15*1024**3
with socket.socket() as sock:
    sock.bind(('127.0.0.1', 11476))
manifest = json.loads((S/'protected_manifest.json').read_text())
for name, want in manifest.items():
    assert sha(name) == want, name

# Same read-only bag audit re-hashes all four originals, not a source replay.
shutil.copy2(OLD/'bag_preflight.py', H/'bag_preflight.py')
with (H/'input_preflight.log').open('x') as log:
    subprocess.run(['/home/yeranis/.local/share/pipx/venvs/evo/bin/python',
                    str(H/'bag_preflight.py')], stdout=log, stderr=subprocess.STDOUT, check=True)
preflight = json.loads((H/'bag_preflight.json').read_text())
assert preflight == json.loads((OLD/'bag_preflight.json').read_text())

P.mkdir(parents=True)
for folder in ('private_build', 'input_profiles', 'transport_tools'):
    shutil.copytree(S/folder, P/folder, symlinks=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
files = ['PROTOCOL.txt', 'coverage_checks.py', 'evaluate_fl4f.py', 'experiment.py',
         'expiry_audit.py', 'eof_boundary.py', 'full_off.json', 'full_on.json',
         'active_window.launch', 'active_window_on.launch', 'glim_profile.launch',
         'glim_profile_on.launch', 'grandtour_eth1_kimera_stereo.launch',
         'kimera_vio_ros.launch', 'kimera_vio_ros_newer_college_2021_stereo.launch',
         'map_guard.py', 'preflight_on.py', 'receiver_cpk19.sh', 'run_case.py',
         'run_case_on.py', 'source.env', 'source_runner.sh', 'source_runner_on.sh',
         'validate_candidate.py', 'validate_case.py', 'validate_local.py',
         'verify_effective.py', 'verify_effective_on.py']
diff = []
def generated(src, dest, value):
    old = src.read_text()
    with dest.open('x') as f:
        f.write(value)
    shutil.copymode(src, dest)
    if old != value:
        diff.extend(difflib.unified_diff(old.splitlines(True), value.splitlines(True),
                                       fromfile=str(src), tofile=str(dest)))

protocol = rebase((OLD/'PROTOCOL.txt').read_text()) + '''
CP-K26 restart authorization: user explicitly requests full SPX-3 restart after
CP-K25 OFF passed and ON was interrupted by host suspend17:14:12--17:34:44.
Do not reuse those captures as this pair or overwrite them. Both modes rerun
from the same start. No estimator, covariance, retention or validation change.
GNOME session suspend/idle inhibitor supplements the original system inhibitor,
both scoped to this finite controller. Verify both before starting containers.
No permanent power-setting edits or sleep-service masks. Manual/forced suspend
can still override protection; no claim the old bypass mechanism is established.
Neither initial diagnostics nor the old failed ON prefix is a completed result.
'''
with (H/'PROTOCOL.txt').open('x') as f:
    f.write(protocol)
for name in files:
    generated(S/name, P/name, protocol if name == 'PROTOCOL.txt' else rebase((S/name).read_text()))
for path in (P/'transport_tools').rglob('*.py'):
    old = path.read_text()
    new = rebase(old)
    if old != new:
        path.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                    fromfile=str(S/path.relative_to(P)), tofile=str(path)))
for name in ('run_pair.py', 'summarize.py'):
    new = rebase((OLD/name).read_text())
    if name == 'run_pair.py':
        needle = "    initial = subprocess.check_output(['docker','inspect'"
        assert new.count(needle) == 1
        check = """    session_inhibitors = subprocess.check_output(['gnome-session-inhibit','--list'], text=True)
    session_matches = [line for line in session_inhibitors.splitlines() if line.startswith('SPX3-CPK26:')]
    assert len(session_matches) == 1 and all(t in session_matches[0] for t in ('suspend','idle'))
"""
        new = new.replace(needle, check+needle)
        new = new.replace('sleep_inhibitor=matches[0], protected_files=count)',
                          'sleep_inhibitor=matches[0], session_inhibitor=session_matches[0], protected_files=count)')
    generated(OLD/name, H/name, new)
with (H/'harness.diff').open('x') as f:
    f.write(''.join(diff))

for folder in ('private_build', 'input_profiles'):
    for path in (S/folder).rglob('*'):
        if path.is_file():
            assert sha(path) == sha(P/path.relative_to(S)), path
for name in ('expiry_audit.py', 'eof_boundary.py', 'map_guard.py', 'coverage_checks.py'):
    assert sha(P/name) == sha(S/name), name
for name in ('full_off.json', 'full_on.json'):
    assert json.loads((P/name).read_text()) == json.loads(rebase((S/name).read_text()))
for path in [*P.rglob('*'), *H.glob('*.py')]:
    if path.suffix == '.py': ast.parse(path.read_text())
    elif path.suffix == '.launch': ET.parse(path)
    elif path.suffix in ('.sh', '.env'): subprocess.run(['bash','-n',str(path)],check=True)
spec = importlib.util.spec_from_file_location('boundary', P/'eof_boundary.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
assert module.test_boundaries() == {'valid': 1, 'rejected': 4}
for base in (P, H):
    for path in base.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name != 'REPORT.txt':
            manifest[str(path)] = sha(path)
with (P/'protected_manifest.json').open('x') as f:
    json.dump(manifest, f, indent=2)
preflight.update(template=str(S), protected_files=len(manifest), model_identical=True,
                 source_profiles_byte_identical=True, all_input_bytes_identical=True,
                 coverage_semantics_unchanged=True, eof_contract='finite_recording_eof_cpk24a',
                 sleep_prevention='scoped systemd and GNOME session inhibitors',
                 disk_free_bytes=shutil.disk_usage(W).free)
(P/'preflight').mkdir()
with (P/'preflight/validation.json').open('x') as f:
    json.dump(preflight, f, indent=2)
print(json.dumps(dict(passed=True, prepared=str(P), protected_files=len(manifest),
                     window=preflight['window']), indent=2))
