"""Read back CP-V31 artifacts and pin the completed saved-data report."""
import hashlib
import json
from pathlib import Path
import subprocess
from datetime import datetime

H = Path(__file__).resolve().parent


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


pins = json.loads((H/'input_sha256.json').read_text())
assert all(sha(p) == digest for p,digest in pins.items())
results = json.loads((H/'results.json').read_text())
assert results['verification']['passed'] and results['total_pairs'] == 365
commands = json.loads((H/'commands.json').read_text())
assert len(commands) == 24 and all(x['returncode'] == 0 for x in commands)
assert len(list((H/'outputs').rglob('*.tum'))) == 39
assert len(list((H/'outputs').rglob('*.zip'))) == 48
assert all(Path(x['archive']).is_file() and Path(x['logfile']).is_file() for x in commands)
checkpoint_command = ['python3', '/home/yeranis/repos/gnssFGO/experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py', '--originals']
checkpoint = subprocess.run(checkpoint_command, capture_output=True, text=True, check=False, timeout=60)
assert checkpoint.returncode == 0, checkpoint.stdout+checkpoint.stderr
assert '667 saved files' in checkpoint.stdout
out = dict(passed=True, time=datetime.now().astimezone().isoformat(), input_pins=len(pins),
           cli_runs=24, native_evo_archives=48, tum_files=39, results=12, pairs_per_result=365,
           checkpoint=dict(command=checkpoint_command, returncode=checkpoint.returncode,
                           stdout=checkpoint.stdout, stderr=checkpoint.stderr),
           source_runs=0, receiver_replays=0, builds=0, containers_started=0,
           output_sha256={str(p):sha(p) for p in sorted(H.rglob('*')) if p.is_file()})
with (H/'closeout.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps({k:v for k,v in out.items() if k != 'output_sha256'}))
