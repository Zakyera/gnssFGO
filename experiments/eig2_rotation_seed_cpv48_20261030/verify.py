"""Final read-only test/integrity checks and fresh completion receipt."""
import datetime
import hashlib
import json
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
R = H.parent.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W / 'runs' / H.name
Q = W / 'runs/eig2_on_checkpoint_cpv44_20261029/checkpoints'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check(pins):
    for path, expected in pins.items():
        assert sha(path) == expected, path


assert not (H / 'complete.json').exists()
pins = json.loads((H / 'protected_manifest.json').read_text())
check(pins)
analysis = json.loads((H / 'analysis.json').read_text())
check(analysis['input_sha256'])
assert json.loads((H / 'test_passed.json').read_text())['passed']
assert not (H / 'failure.json').exists()
for k in range(21, 26):
    root = P / 'results' / f'update_{k}'
    for phase in ('ordinary', 'refresh'):
        assert sha(root / (phase + '_before.bin')) == sha(root / (phase + '_after.bin')) == sha(Q / f'update_{k}' / ('after_' + phase) / 'checkpoint.bin')
for mode in ('release', 'san'):
    assert (P / (mode + '_selftest.log')).read_text().strip() == 'PASS rotation_only=1 corrected_rotation=1 original_unchanged=1 two_steps=1'
assert (P / 'results/all_A_passed.txt').exists()
assert analysis['experiment_native_GN_steps'] == 20
assert analysis['source_replays'] == 0 and not analysis['accuracy_scored']
for event in analysis['updates']:
    assert event['seed']['position_change_m'] == 0
    assert event['seed']['newest_rotation_residual_after_deg'] < 1e-10
    assert event['parity']['ordinary_equation_relative'] < 1e-12
    assert event['parity']['exact_refresh_equation_relative'] < 1e-12
    assert event['parity']['ordinary_captured_tangent'] < 1e-6
    assert event['parity']['chained_refresh_captured_tangent'] < 1e-5
states = subprocess.check_output(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                                 'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
assert states == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'], states
jobs = []
for receipt in [*H.glob('*_process.json'), H / 'start.json']:
    data = json.loads(receipt.read_text())
    pid = int(data['pid'])
    path = Path(f'/proc/{pid}/cmdline')
    cmd = path.read_bytes().replace(b'\0', b' ').decode(errors='replace') if path.exists() else None
    assert cmd is None or 'eig2_rotation_seed_cpv48_20261030' not in cmd, (pid, cmd)
    jobs.append(dict(receipt=str(receipt), pid=pid, current_command=cmd, owned_job_exited=True))
hesai = subprocess.check_output(['python3', str(R / 'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'), '--originals'], text=True).strip()
assert hesai == 'PASS: 4 cases, 667 saved files; originals=True', hesai
outputs = {str(path): sha(path) for root in (H, P) for path in sorted(root.rglob('*')) if path.is_file()}
out = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
           decision='rotation-only seed candidate rejected for captured large tilts; not a general root-cause proof',
           protected_unchanged=len(pins), analyzed_inputs_unchanged=len(analysis['input_sha256']),
           snapshot_nonmutation=10, release_and_sanitizer_fixtures_pass=True,
           source_replays=0, experiment_native_GN_steps=20, synthetic_fixture_GN_steps=4,
           covariance_analysis=False, production_edits=False, new_accuracy_claim=False,
           containers=states, owned_jobs=jobs, hesai=hesai,
           source_sha256=sha(P / 'RotationSeed.cpp'), output_sha256=outputs)
with (H / 'complete.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps({k: v for k, v in out.items() if k not in ('owned_jobs', 'output_sha256')}, indent=2))
