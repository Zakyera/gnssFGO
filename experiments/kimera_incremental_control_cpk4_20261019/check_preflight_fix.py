"""Check the harness-only path correction without starting ROS or playing data."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_incremental_control_cpk4_20261019')
D = R / 'off_capture'

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()

assert not (H / 'preflight_fix_validation.json').exists()
assert not any((D / name).exists() for name in
               ('started.json', 'play.log', 'objective.csv', 'outputs.bag', 'acceptance.csv'))
assert json.loads((D / 'cleanup.json').read_text())['kimera_process_gone']
root = ET.parse(R / 'kimera_vio_ros.launch').getroot()
audit = [e.attrib['value'] for e in root.iter('env') if e.attrib.get('name') == 'FL4M_AUDIT']
assert len(audit) == 1
actual = audit[0].replace('$(env ROS_LOG_DIR)', str(D / 'ros_logs'))
expected = str(D / 'objective.csv')
assert actual != expected  # Reproduces the failed literal-string check.
assert Path(actual).resolve() == Path(expected).resolve()
assert Path(actual).resolve() != (D.parent / 'wrong/objective.csv').resolve()
ast.parse((R / 'run_case.py').read_text())
assert (R / 'run_case.py').read_bytes() == (H / 'run_case.py').read_bytes()
protected = json.loads((R / 'protected_before.json').read_text())
assert all(sha(p) == h for p, h in protected.items())
manifest = json.loads((R / 'pre_capture_receipt.json').read_text())['manifest']
changes = [p for p, h in manifest.items() if sha(p) != h]
assert changes == [str(R / 'run_case.py')]
result = dict(passed=True, literal_check_was_false=True, normalized_identity_matches=True,
              different_destination_still_rejected=True, source_playback_count=0,
              protected_files_unchanged=len(protected), changed_prepared_files=changes,
              launched_controller_sha256=manifest[str(R / 'run_case.py')],
              corrected_controller_sha256=sha(R / 'run_case.py'),
              limitation='No source capture or estimator validation; no automatic retry.')
with (H / 'preflight_fix_validation.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2))
