"""One-shot supplemental failure audit. No capture, scoring, or original writes."""
import datetime
import hashlib
import json
import subprocess
from pathlib import Path
from paths import H, E, P, BASE_H

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def verify(pins):
    for path, expected in pins.items():
        assert sha(path) == expected, path
    return len(pins)

def exited(receipt, pid, start, command):
    stat = Path('/proc') / str(pid) / 'stat'
    current = stat.read_text().rsplit(')', 1)[1].split()[19] if stat.exists() else None
    assert current != str(start), (receipt, pid, start)
    return dict(receipt=str(receipt), pid=pid, recorded_start=str(start),
                recorded_command=command, current_start=current, original_identity_alive=False)

target = H / 'failure_post_closeout.json'
assert not target.exists()
assert not (H / 'complete.json').exists()
assert read(H / 'failed.json')['stage'] == 'capture_on'
assert read(H / 'closeout.json')['passed']
assert read(H / 'off_validated.json')['passed']
terminal_paths = [H / name for name in (
    'failed.json', 'status.json', 'closeout.json', 'REPORT.txt', 'controller.log',
    'capture_on.log', 'capture_on_receipt.json', 'off_validated.json', 'owned_jobs_exited.json')]
terminal_paths += [P / 'full_on_source/estimators.log',
                   P / 'full_on_control/failed.txt', P / 'full_on_control/owned_job_cleanup.json',
                   P / 'full_off_control/complete.json']
pins = {str(path): sha(path) for path in terminal_paths}
out = dict(audit_passed=True, experiment_status='FAILED_ON', full_pair_complete=False,
           checked_at=datetime.datetime.now().astimezone().isoformat())
out['protected_files'] = verify(read(P / 'protected_manifest.json'))
out['cp95_terminal_files'] = verify(read(BASE_H / 'complete.json')['output_sha256'])
out['cp98_terminal_files'] = verify(read(E / 'eth1_early_native_capture_cpv98_20261103/complete.json')['output_sha256'])
jobs = []
for receipt in [H / 'start.json', *sorted(H.glob('*_process.json')),
                *sorted((P / 'full_off_control').glob('*_process.json')),
                *sorted((P / 'full_on_control').glob('*_process.json'))]:
    item = read(receipt)
    jobs.append(exited(receipt, item['pid'], item['proc_start'], item.get('command', 'controller.py')))
launch = read(H / 'launch.json')
jobs.append(exited(H / 'launch.json', launch['owned_supervisor_pid'],
                   launch['supervisor_proc_start'], launch['args']))
out['owned_jobs'] = jobs
containers = json.loads(subprocess.check_output([
    'docker', 'inspect', 'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True))
out['containers'] = [{key: item[key] for key in ('Name', 'State')} for item in containers]
assert all(item['State']['Status'] == 'exited' and item['State']['Pid'] == 0 for item in containers)
out['hesai'] = subprocess.check_output([
    'python3', str(E / 'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),
    '--originals'], text=True)
assert '667 saved files' in out['hesai'] and 'PASS' in out['hesai']
source = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/persistent_compatibility_cpv76_20261101/release_fixture1/source_snapshot/NativePriorRefresh.h')
source_pin = read(source.parent.parent / 'sources.json')['/workspace/cbs_gtsam4.3/runs/persistent_compatibility_cpv76_20261101/NativePriorRefresh.h']
assert sha(source) == source_pin
out['reviewed_source'] = {str(source): source_pin}
out['failure_event'] = dict(update=1476, timestamp_ns=1727775307353716285,
    input_epoch_ns=1727775007855396309,
    relative_seconds=(1727775307353716285 - 1727775007855396309) / 1e9,
    error='CPK9 promotion changes cached prior',
    guard_threshold=1e-8, actual_error_magnitude_logged=False,
    failed_comparison_identified=False)
out['terminal_sha256'] = pins
verify(pins)
with target.open('x') as stream:
    json.dump(out, stream, indent=2)
print(json.dumps({key: value for key, value in out.items() if key not in ('owned_jobs', 'terminal_sha256', 'containers')}, indent=2))
print('Verified owned process identities:', len(jobs))
print('Both originally stopped containers restored stopped; terminal files unchanged.')
