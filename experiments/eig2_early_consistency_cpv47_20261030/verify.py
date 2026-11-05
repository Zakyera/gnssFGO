"""Read-only integrity/closeout checks; creates only a fresh completion receipt."""
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


def sha(p):
    with Path(p).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def check(pins):
    for p, expected in pins.items():
        assert sha(p) == expected, p


assert not (H / 'complete.json').exists(), 'closeout already exists'
protected = json.loads((H / 'attempt_api1/protected_manifest.json').read_text())
check(protected)
analysis = json.loads((H / 'analysis.json').read_text())
check(analysis['input_sha256'])
for k in range(21, 26):
    root = P / 'attempt_api1/results' / f'update_{k}'
    for phase in ('before_ordinary', 'after_ordinary', 'after_refresh'):
        assert sha(root / (phase + '_before.bin')) == sha(root / (phase + '_after.bin')) == sha(Q / f'update_{k}' / phase / 'checkpoint.bin')
    assert sha(root / 'initialized_after.bin') == sha(root / 'after_ordinary_before.bin')
for mode in ('release', 'san'):
    assert (P / 'attempt_api1' / (mode + '_selftest.log')).read_text().strip() == 'PASS bias_corrected_predict=1 stereo_position_residual=1 PIM_nonmutation=1'
assert (H / 'failure.json').is_file()
assert not (H / 'attempt_api1/failure.json').exists()
assert json.loads((H / 'attempt_api1/inspection_passed.json').read_text())['passed']
assert analysis['source_replays'] == analysis['optimizer_calls'] == 0
assert analysis['checks']['unchanged_pims']

states = subprocess.check_output(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                                 'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
assert states == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'], states
jobs = []
for receipt in [*H.rglob('*_process.json'), H / 'start.json', H / 'attempt_api1/start.json']:
    d = json.loads(receipt.read_text())
    pid = int(d['pid'])
    cmd = Path(f'/proc/{pid}/cmdline')
    current = cmd.read_bytes().replace(b'\0', b' ').decode(errors='replace') if cmd.exists() else None
    assert current is None or 'eig2_early_consistency_cpv47_20261030' not in current, (pid, current)
    jobs.append(dict(receipt=str(receipt), pid=pid, current_command=current,
                     owned_job_exited=True))
hesai = subprocess.check_output(['python3', str(R / 'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'), '--originals'], text=True).strip()
assert hesai == 'PASS: 4 cases, 667 saved files; originals=True', hesai
outputs = {str(p): sha(p) for root in (H, P) for p in sorted(root.rglob('*')) if p.is_file()}
out = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
           protected_unchanged=len(protected), analyzed_inputs_unchanged=len(analysis['input_sha256']),
           snapshot_nonmutation=15, preserved_original_compile_failure=True,
           release_and_sanitizer_fixtures_pass=True, hesai=hesai, containers=states,
           owned_jobs=jobs, source_replays=0, optimizer_calls=0,
           covariance_analysis=False, production_edits=False, root_cause_proved=False,
           new_accuracy_claim=False, output_sha256=outputs)
with (H / 'complete.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps({k: v for k, v in out.items() if k not in ('output_sha256', 'owned_jobs')}, indent=2))
