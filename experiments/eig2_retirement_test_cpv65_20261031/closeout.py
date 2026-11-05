"""Read-only host/container check; exclusive staged-completion receipt."""
import datetime
import hashlib
import json
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert not (H / 'complete.json').exists()
verification = json.loads((H / 'verification.json').read_text())
assert verification['passed'] and verification['native_optimizer_calls'] == 0
for path, expected in verification['output_sha256'].items():
    assert sha(path) == expected, path
analysis = json.loads((H / 'analysis.json').read_text())
for path, expected in analysis['input_sha256'].items():
    assert sha(path) == expected, path
states = subprocess.check_output(['docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
                                 'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
assert states == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited'], states
receipt = dict(status='TEST_DESIGN_AND_SAVED_SCREEN_COMPLETE', passed=True,
               causal_test_status='NOT_RUN_MISSING_NATIVE_RETIREMENT_CHECKPOINTS',
               native_causal_harness_implemented=False, native_optimizer_calls=0,
               source_capture_launched=False, production_changes=False,
               stopped_containers=states, original_inputs_verified=len(analysis['input_sha256']),
               tests=9, independent_vectors=verification['independently_checked_vectors'],
               hesai_checkpoint=verification['hesai_checkpoint'],
               finished=datetime.datetime.now().astimezone().isoformat(),
               output_sha256={str(p): sha(p) for p in sorted(H.iterdir()) if p.is_file()})
with (H / 'complete.json').open('x') as stream:
    json.dump(receipt, stream, indent=2)
print(json.dumps({k: v for k, v in receipt.items() if k != 'output_sha256'}, indent=2))
