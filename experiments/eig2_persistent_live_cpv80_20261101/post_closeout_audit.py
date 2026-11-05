"""Supplemental read-only integrity/process audit; never starts a run or scorer."""
import datetime
import hashlib
import json
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
P = Path(json.loads((H / 'prepared.json').read_text())['prepared'])
H78 = H.parent / 'persistent_native_full_cpv78_20261101'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify(entries):
    errors = []
    for path, expected in entries.items():
        try:
            actual = sha(path)
            if actual != expected:
                errors.append({'path': path, 'expected': expected, 'actual': actual})
        except OSError as exc:
            errors.append({'path': path, 'error': str(exc)})
    return {'passed': not errors, 'checked': len(entries), 'errors': errors}


def process(pid, expected_start=None, expected_command=None):
    proc = Path('/proc') / str(pid)
    try:
        stat = (proc / 'stat').read_text()
        start = stat[stat.rfind(')') + 2:].split()[19]
        command = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
    except FileNotFoundError:
        return {'pid': pid, 'exists': False, 'owned_alive': False,
                'expected_start': expected_start, 'expected_command': expected_command}
    same_start = expected_start is not None and start == str(expected_start)
    # Without a recorded start identity, require absence rather than infer PID reuse.
    return {'pid': pid, 'exists': True, 'owned_alive': same_start or expected_start is None,
            'actual_start': start, 'actual_command': command,
            'expected_start': expected_start, 'expected_command': expected_command}


assert not (H / 'post_closeout.json').exists(), 'Preserve previous supplemental evidence'
complete = read(H / 'complete.json')
out = {'observed': datetime.datetime.now().astimezone().isoformat(),
       'scope': 'Supplement only: hashes, receipt exits, container state; no validation/scoring replay',
       'complete_sha256': sha(H / 'complete.json'),
       'protected': verify(read(P / 'protected_manifest.json')),
       'terminal_outputs': verify(complete['output_sha256']),
       'cp78_complete_sha256': sha(H78 / 'complete.json'),
       'cp78_terminal_outputs': verify(read(H78 / 'complete.json')['output_sha256'])}
receipts = [H / 'start.json', *sorted(H.glob('*_process.json')),
            *sorted((P / 'full_on_control').glob('*_process.json'))]
out['processes'] = []
for path in receipts:
    row = read(path)
    out['processes'].append(dict(receipt=str(path), **process(
        row['pid'], row.get('proc_start'), row.get('args', row.get('command')))))
launch = read(H / 'launch.json')
for role, pid in [('launcher', launch['launcher_pid']), ('supervisor', launch['owned_supervisor_pid']),
                  ('GLIM_startup_observation', 129798), ('Kimera_startup_observation', 129801),
                  ('receiver_startup_observation', 129583), ('bag_startup_observation', 130028)]:
    out['processes'].append(dict(role=role, **process(pid)))
out['containers'] = subprocess.check_output(
    ['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
     'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
out['containers_restored'] = out['containers'] == read(H / 'start.json')['before']
out['hesai_checkpoint'] = subprocess.check_output(
    ['python3', str(H.parent / 'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),
     '--originals'], text=True).strip()
out['stage_receipts'] = {str(path): read(path) for path in sorted(H.glob('*_receipt.json'))}
out['source_cleanup'] = read(P / 'full_on_control/owned_job_cleanup.json')
out['failure_markers'] = [str(H / name) for name in
                          ('failed.json', 'cleanup_failed.json', 'report_failed.json')
                          if (H / name).exists()]
out['passed'] = (complete['passed'] and read(H / 'report_verification.json')['passed']
                 and all(out[key]['passed'] for key in ('protected', 'terminal_outputs', 'cp78_terminal_outputs'))
                 and not any(row['owned_alive'] for row in out['processes'])
                 and out['containers_restored']
                 and out['hesai_checkpoint'] == 'PASS: 4 cases, 667 saved files; originals=True'
                 and all(row['exit'] == 0 for row in out['stage_receipts'].values())
                 and all(code == 0 for code in out['source_cleanup'].values())
                 and not out['failure_markers'])
with (H / 'post_closeout.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps(out, indent=2))
assert out['passed'], 'Supplemental audit failed; preserve terminal artifacts'
