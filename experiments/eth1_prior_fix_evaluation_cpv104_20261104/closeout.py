"""Read-only old-evidence checks; exclusively create new CP104 terminal receipts."""
import datetime
import hashlib
import json
import stat
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
E = H.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_prior_fix_live_cpv103_20261103/prepared')
read = lambda p: json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()
def check(entries):
    for name, digest in entries.items():
        assert sha(name) == digest, name
    return len(entries)
def save(name, value):
    with (H/name).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)

assert not (H/'complete.json').exists() and not (H/'closeout.json').exists()
assert read(H/'report_verification.json')['passed'] and read(H/'coverage_review.json')['passed']
inputs = check(read(H/'evaluation/input_sha256.json'))
coverage_inputs = check(read(H/'coverage_review.json')['input_sha256'])
protected = check(read(P/'protected_manifest.json'))
assert protected == 8120
old = {}
for name in ('eth1_prior_fix_live_cpv103_20261103',
             'eth1_split_imu_live_pair_cpv95_20261103_continuation1',
             'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'):
    rec = read(E/name/'complete.json')
    assert rec['passed']
    old[name] = check(rec['output_sha256'])
checkpoint = E/'checkpoints/20261023_hesai_four_sequences'
result = subprocess.run(['/usr/bin/python3', '-B', str(checkpoint/'verify_checkpoint.py'), '--originals'],
                        text=True, capture_output=True, check=True, timeout=180)
assert '667 saved files; originals=True' in result.stdout
containers = json.loads(subprocess.check_output(['docker', 'inspect',
                         'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True))
states = {c['Name'].lstrip('/'): {'running': c['State']['Running'], 'status': c['State']['Status']}
          for c in containers}
assert len(states) == 2 and all(not c['running'] for c in states.values())
identities = {}
for pid, expected_start in ((104398, 1346319), (104400, 1346319)):
    proc = Path('/proc')/str(pid)
    if not proc.exists():
        identities[str(pid)] = dict(expected_proc_start=expected_start, original_identity_exited=True, current='absent')
        continue
    raw = (proc/'stat').read_text()
    start = int(raw[raw.rfind(')')+2:].split()[19])
    cmd = (proc/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    assert start != expected_start, (pid, start, cmd)
    identities[str(pid)] = dict(expected_proc_start=expected_start, original_identity_exited=True,
                               current_proc_start=start, current_command=cmd)
out = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
           evaluation_inputs=inputs, coverage_review_inputs=coverage_inputs, protected_files=protected,
           old_terminal_output_counts=old, checkpoint_stdout=result.stdout.strip(),
           containers=states, CP103_controller_supervisor=identities,
           CP103_all_owned_exits_receipt=str(E/'eth1_prior_fix_live_cpv103_20261103/post_closeout.json'),
           source_runs=0, receiver_replays=0, no_old_evidence_modified=True)
assert read(out['CP103_all_owned_exits_receipt'])['all_owned_jobs_exited']
save('closeout.json', out)
outputs = {}
for path in sorted(H.rglob('*')):
    mode = path.lstat().st_mode
    assert not stat.S_ISLNK(mode) and (stat.S_ISREG(mode) or stat.S_ISDIR(mode)), path
    if stat.S_ISREG(mode):
        outputs[str(path)] = sha(path)
save('complete.json', dict(passed=True, task='CP-V104',
     finished=datetime.datetime.now().astimezone().isoformat(),
     saved_data_only=True, source_runs=0, receiver_replays=0,
     output_sha256=outputs, prior_evidence_preserved=True,
     limitations='CP103 changed backend; historical controls, not a new matched OFF/ON pair. No causal, randomness or uncertainty-calibration proof.'))
print(json.dumps(dict(passed=True, protected_files=protected, outputs=len(outputs),
                     checkpoint=result.stdout.strip(), containers=states), indent=2))
