"""Read-only evidence checks; exclusively seal this supplemental diagnosis."""
import datetime
import hashlib
import json
import stat
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
E = H.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eth1_prior_fix_live_cpv103_20261103/prepared')

def read(path):
    return json.loads(Path(path).read_text())

def sha(path):
    path = Path(path)
    assert stat.S_ISREG(path.stat().st_mode), path
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def check(entries):
    for path, digest in entries.items():
        assert sha(path) == digest, path
    return len(entries)

def save(name, data):
    with (H/name).open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)

assert not (H/'complete.json').exists() and not (H/'closeout.json').exists()
counts = {}
for name in ('localization.json', 'belief_analysis.json', 'verification.json'):
    data = read(H/name)
    assert data['passed']
    counts[name] = check(data['input_sha256'])
assert read(H/'verification.json')['causal_proof'] is False
protected = check(read(P/'protected_manifest.json'))
assert protected == 8120
old = {}
for name, expected in (
    ('eth1_prior_fix_evaluation_cpv104_20261104', 318),
    ('eth1_prior_fix_live_cpv103_20261103', 62),
    ('eth1_split_imu_live_pair_cpv95_20261103_continuation1', 426),
):
    terminal = read(E/name/'complete.json')
    assert terminal['passed']
    old[name] = check(terminal['output_sha256'])
    assert old[name] == expected
checkpoint = E/'checkpoints/20261023_hesai_four_sequences'
result = subprocess.run(
    ['/usr/bin/python3', '-B', str(checkpoint/'verify_checkpoint.py'), '--originals'],
    text=True, capture_output=True, check=True, timeout=180,
)
assert '667 saved files; originals=True' in result.stdout
save('closeout.json', dict(
    passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
    diagnostic_input_counts=counts, protected_files=protected,
    historical_terminal_output_counts=old, checkpoint_stdout=result.stdout.strip(),
    captures=0, receiver_replays=0, EVO_calls=0, old_evidence_preserved=True,
    causal_proof=False,
))
outputs = {}
for path in sorted(H.rglob('*')):
    mode = path.lstat().st_mode
    assert stat.S_ISREG(mode) or stat.S_ISDIR(mode), path
    if stat.S_ISREG(mode):
        outputs[str(path)] = sha(path)
save('complete.json', dict(
    passed=True, task='CP-V105', finished=datetime.datetime.now().astimezone().isoformat(),
    saved_data_diagnosis_only=True, output_sha256=outputs,
    captures=0, receiver_replays=0, EVO_calls=0, causal_proof=False,
    conclusion='Revised Kimera covariance contracts without matching relative-reference improvement; contribution to fusion degradation requires a controlled intervention.',
    next_test_proposed_not_run='Parity-first saved ON receiver replay, then covariance-revision freeze diagnostic with all means/order/settings preserved.',
))
print(json.dumps(dict(passed=True, inputs=counts, protected=protected,
                     old=old, outputs=len(outputs), checkpoint=result.stdout.strip()), indent=2))
