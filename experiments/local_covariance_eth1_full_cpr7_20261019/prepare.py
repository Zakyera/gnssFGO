"""Mechanically extend the frozen CP-R6 harness; no estimator/model edits."""
from pathlib import Path
import ast, difflib, hashlib, json, shutil, subprocess
import xml.etree.ElementTree as ET

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
F = H.parent / 'local_covariance_eth1_cpr6_20261019'
T = W / 'runs/gnssfgo_eth1_local_cov60_cpr6_20261019/prepared'
R = W / 'runs/gnssfgo_eth1_local_cov_full_cpr7_20261019/prepared'
DURATION = 346.22

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def replace_once(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

assert not R.parent.exists(), 'refuse duplicate preparation'
assert shutil.disk_usage(W).free > 20 * 1024**3
assert json.loads((F / 'final_checks.json').read_text())['passed']
manifest = json.loads((T / 'protected_manifest.json').read_text())
for p, expected in manifest.items():
    assert sha(p) == expected, p
print('CP-R6 protected inputs unchanged', flush=True)

# Retain the same topic/IMU/reference checks. Extend through the last Hesai
# record, rounded UP to milliseconds; the <1ms end allowance is ONLY the
# metadata rounding tolerance, not a relaxation of trajectory coverage.
check = (F / 'inspect_bags.py').read_text()
check = replace_once(check, 'start=epoch;end=start+60', 'start=epoch;end=start+346.22')
check = replace_once(check, "out[k]['record_end']>=end", "out[k]['record_end']+0.001>=end")
check = replace_once(check, 'bag_offset=0,duration=60', 'bag_offset=0,duration=346.22')
check = replace_once(check, 'print(json.dumps(out,indent=2))',
    "assert 0 <= end-out['hesai']['record_end'] < 0.001\n"
    "out['window']['end_rule']='through last Hesai record, rounded up to1ms; exclude camera/IMU-only tails'\n"
    "print(json.dumps(out,indent=2))")
proc = subprocess.run(['docker', 'exec', '-i', 'cbsms_ws_gpu', 'bash', '-lc',
    'source /opt/ros/noetic/setup.bash; python3 -'], input=check, text=True,
    capture_output=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
bags = json.loads(proc.stdout)
assert bags['passed']
assert bags['window']['duration'] == DURATION
R.mkdir(parents=True)
shutil.copytree(T / 'private_build', R / 'private_build', symlinks=True)
shutil.copytree(T / 'input_profiles', R / 'input_profiles')

def common(s):
    return (s.replace(T.parent.name, R.parent.name)
        .replace('eth1_cpr6_live60_', 'eth1_cpr7_full_')
        .replace('live60', 'full').replace('11452', '11453')
        .replace('CP-R6', 'CP-R7').replace('bag0-60s', 'bag0-346.22s'))

files = ['experiment.py', 'coverage_checks.py', 'validate_case.py',
    'validate_candidate.py', 'map_guard.py', 'source.env', 'receiver_fl4e.sh',
    'run_case.py', 'run_case_on.py', 'verify_effective.py', 'verify_effective_on.py',
    'source_runner.sh', 'source_runner_on.sh', 'active_window.launch',
    'active_window_on.launch', 'glim_profile.launch', 'glim_profile_on.launch',
    'grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros_newer_college_2021_stereo.launch',
    'kimera_vio_ros.launch', 'live60_off.json', 'live60_on.json',
    'validate_local.py', 'evaluate_fl4f.py', 'preflight_on.py']
diff = []
for name in files:
    old = (T / name).read_text()
    s = common(old)
    if name in ('run_case.py', 'run_case_on.py'):
        s = replace_once(s, 'duration=60', 'duration=346.22')
    if name == 'source.env':
        s = replace_once(s, 'CBSMS_BAG_DURATION_SEC:-60.0', 'CBSMS_BAG_DURATION_SEC:-346.22')
    if name in ('source_runner.sh', 'source_runner_on.sh'):
        s = replace_once(s, '"${duration_sec}" == 60.0', '"${duration_sec}" == 346.22')
        s = s.replace('60-second', 'full346.22-second')
        s = replace_once(s, 'min_glim_rows=570', 'min_glim_rows=3430')
        s = replace_once(s, 'min_kimera_rows=270', 'min_kimera_rows=1700')
    if name in ('verify_effective.py', 'verify_effective_on.py'):
        s = replace_once(s, "['CBSMS_BAG_DURATION_SEC'] == '60.0'",
                         "['CBSMS_BAG_DURATION_SEC'] == '346.22'")
    if name == 'validate_case.py':
        s = replace_once(s, "['requested_s']==60", "['requested_s']==346.22")
        s = s.replace("['rows']>=570", "['rows']>=3430").replace("['rows']>=270", "['rows']>=1700")
        s = replace_once(s, '1e-9>=58.', '1e-9>=344.22')
        s = s.replace('bounded pilot', 'full ETH1 capture')
    if name == 'validate_candidate.py':
        s = replace_once(s, 'len(a)>=270', 'len(a)>=1700')
    if name == 'validate_local.py':
        s = replace_once(s, 'first>=570', 'first>=3430')
    if name == 'preflight_on.py':
        s = s.replace('same60s OFF', 'same346.22s OFF')
    target = R / name.replace('live60', 'full')
    target.write_text(s)
    shutil.copymode(T / name, target)
    diff.extend(difflib.unified_diff(old.splitlines(True), s.splitlines(True),
        fromfile=str(T / name), tofile=str(target)))

shutil.copytree(T / 'transport_tools', R / 'transport_tools',
               ignore=shutil.ignore_patterns('__pycache__'))
for p in (R / 'transport_tools').rglob('*.py'):
    p.write_text(common(p.read_text()))
shutil.copy2(H / 'PROTOCOL.txt', R / 'PROTOCOL.txt')
for name in ('summarize.py', 'final_checks.py'):
    s = common((F / name).read_text()).replace('matched60s ETH1', 'matched full346.22s ETH1')
    if name == 'summarize.py':
        s = replace_once(s, "on['span_s'])>=58", "on['span_s'])>=344.22")
    (H / name).write_text(s)
(H / 'inspect_bags.py').write_text(check)
(H / 'harness.diff').write_text(''.join(diff))

assert sha(R / 'receiver_fl4e.sh') == sha(T / 'receiver_fl4e.sh')
assert sha(R / 'coverage_checks.py') == sha(T / 'coverage_checks.py')
def config(p):
    obj = json.loads(p.read_text())
    for key in ('sequence', 'ros_master_uri', 'receiver_binary'):
        obj.pop(key)
    for source in obj['sources']:
        source.pop('session')
    return obj
assert config(R / 'full_off.json') == config(R / 'full_on.json') == config(T / 'live60_off.json')
def functions(code):
    return {x.name: ast.dump(x, include_attributes=False) for x in ast.parse(code).body
            if isinstance(x, ast.FunctionDef)}
expected = functions(common((T / 'evaluate_fl4f.py').read_text()))
assert expected == functions((R / 'evaluate_fl4f.py').read_text())
for p in R.rglob('*'):
    if p.suffix == '.py': ast.parse(p.read_text())
    if p.suffix == '.launch': ET.parse(p)
    if p.suffix in ('.sh', '.env'):
        subprocess.run(['bash', '-n', str(p)], check=True)
for base in (T, R):
    for p in base.rglob('*'):
        # Protect the template code/profile/runtime, not mutable controller files.
        if base == T and p.parts[len(T.parts)] not in [
                *files, 'input_profiles', 'private_build', 'transport_tools']:
            continue
        if p.is_file() and '__pycache__' not in p.parts:
            manifest[str(p)] = sha(p)
for p in H.iterdir():
    if p.is_file() and (p.suffix == '.py' or p.name == 'PROTOCOL.txt'):
        manifest[str(p)] = sha(p)
(R / 'protected_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
(R / 'preflight').mkdir()
preflight = dict(passed=True, template=str(T), bags=bags, requested_s=DURATION,
    bag_start_s=0, protected_files=len(manifest), free_bytes=shutil.disk_usage(W).free,
    binaries_identical=True, receiver_model_identical=True,
    scoring_helpers_unchanged=True, coverage_checks_unchanged=True,
    initial_container_state='both stopped before authorized preparation',
    recording='same diagnostic load as CP-R6; no live inspectors',
    end_rule=bags['window']['end_rule'])
(R / 'preflight/validation.json').write_text(json.dumps(preflight, indent=2)+'\n')
print(json.dumps(preflight, indent=2))
