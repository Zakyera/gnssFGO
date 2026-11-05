"""Read-only provenance/closeout checks and exclusive completion receipt."""
import datetime
import hashlib
import json
import os
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
R = H.parent.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs') / H.name
A = H / 'attempt_compile1'
D = P / 'attempt_compile1'

def sha(p):
    with Path(p).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def check(pins):
    for path, expected in pins.items():
        assert sha(path) == expected, path

assert not (H / 'complete.json').exists()
pins = json.loads((A / 'protected_manifest.json').read_text())
check(pins)
analysis = json.loads((H / 'analysis.json').read_text())
check(analysis['input_sha256'])
visual = json.loads((H / 'visual_support.json').read_text())
check(visual['input_sha256'])
assert json.loads((A / 'test_passed.json').read_text())['passed']
assert not (A / 'failure.json').exists()
assert (H / 'failure.json').exists()  # Preserved first compile failure, no solver run.
assert (H / 'compile_repair.txt').exists()
for mode in ('release', 'san'):
    assert (D / (mode + '_selftest.log')).read_text().strip() == 'PASS single_removal=1 native_retained=1 initial_unchanged=1 two_steps=1'
assert analysis['native_GN_steps'] == 4
assert analysis['source_replays'] == 0 and not analysis['accuracy_scored']
assert (D / 'results/A_passed_before_B.txt').exists()
for phase in ('ordinary', 'refresh'):
    original = P.parent / 'eig2_hesai_insertion_cpv51_20261030/prepared/full_on_source/checkpoints/update_6' / ('after_'+phase) / 'checkpoint.bin'
    assert sha(D / 'results' / (phase+'_before.bin')) == sha(D / 'results' / (phase+'_after.bin')) == sha(original)
assert sha(D / 'results/B_target_restored.bin') == sha(D / 'results/ordinary_before.bin')
states = subprocess.check_output(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                                  'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
assert states == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'], states
jobs = []
for receipt in [*H.glob('*_process.json'), *A.glob('*_process.json'), H/'start.json', A/'start.json']:
    data = json.loads(receipt.read_text()); pid = int(data['pid'])
    path = Path(f'/proc/{pid}/cmdline')
    cmd = path.read_bytes().replace(b'\0', b' ').decode(errors='replace') if path.exists() else None
    assert cmd is None or H.name not in cmd, (pid, cmd)
    jobs.append(dict(receipt=str(receipt), pid=pid, current_command=cmd, owned_job_exited=True))
hesai = subprocess.check_output(['python3', str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'), '--originals'], text=True).strip()
assert hesai == 'PASS: 4 cases, 667 saved files; originals=True', hesai
out = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
           protected_unchanged=len(pins), analysis_inputs_unchanged=len(analysis['input_sha256']),
           original_snapshots_unchanged=2, single_factor_ablation_verified=True,
           release_and_sanitizer_fixtures_pass=True, vendor_libraries_instrumented=False,
           source_replays=0, experiment_native_GN_steps=4, synthetic_fixture_GN_steps=8,
           covariance_analysis=False, production_edits=False, new_accuracy_claim=False,
           root_cause_proved=False, containers=states, owned_jobs=jobs, hesai=hesai,
           free_bytes=os.statvfs(P).f_bavail * os.statvfs(P).f_frsize,
           preserved_compile_failure=True, source_sha256=sha(D/'FirstFactor.cpp'))
with (H / 'verification.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
out['output_sha256'] = {str(p): sha(p) for root in (H, P) for p in sorted(root.rglob('*')) if p.is_file()}
with (H / 'complete.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps({k:v for k,v in out.items() if k not in ('output_sha256','owned_jobs')}, indent=2))
