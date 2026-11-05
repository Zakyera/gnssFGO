"""Fresh CP-K4 capture after explicit user authorization; preserve failed preflight."""
import ast
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = W / 'runs/kimera_incremental_control_cpk4_20261019'
R = W / 'runs/kimera_incremental_control_cpk4b_20261019'
K = W / 'runs/kimera_failed_factor_cpk1_20261019'
def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

assert not R.exists(), 'Refuse duplicate preparation'
assert json.loads((H/'preflight_fix_validation.json').read_text())['passed']
assert sha(H/'run_case.py') == 'd8b671dc2df30e870f6db7c62b6bee913af40bf645337b254e9f3d03bbf571d0'
assert not (OLD/'off_capture/started.json').exists()
assert json.loads((OLD/'off_capture/cleanup.json').read_text())['kimera_process_gone']
protected = json.loads((OLD/'protected_before.json').read_text())
assert all(sha(p) == h for p,h in protected.items())
assert shutil.disk_usage(W).free > 8*1024**3
R.mkdir()
shutil.copytree(OLD/'kimera_profile', R/'kimera_profile')
for name in ('grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros.launch',
             'kimera_vio_ros_newer_college_2021_stereo.launch', 'preflight_tests.json', 'protected_before.json'):
    shutil.copy2(OLD/name, R/name)
controller = (H/'run_case.py').read_text().replace('11455', '11456')
ast.parse(controller)
(R/'run_case.py').write_text(controller)
protocol = (H/'PROTOCOL.txt').read_text().replace('runs/kimera_incremental_control_cpk4_20261019',
                                               'runs/kimera_incremental_control_cpk4b_20261019')
protocol += '\nExplicit reauthorization after the harness-only preflight blocker.\nFresh root CP-K4b, ROS port11456; old failed preflight unchanged.\nSame one111s control and predeclared gates. No automatic source retry.\n'
(R/'PROTOCOL.txt').write_text(protocol)
for name in ('grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros.launch', 'kimera_vio_ros_newer_college_2021_stereo.launch'):
    ET.parse(R/name)
manifest = {str(p):sha(p) for p in R.rglob('*') if p.is_file()}
old_manifest = json.loads((OLD/'pre_capture_receipt.json').read_text())['manifest']
for p,h in old_manifest.items():
    if p.startswith(str(K)+'/'):
        assert sha(p) == h
        manifest[p] = h
receipt = dict(manifest=manifest, protected_count=len(protected), source_capture_limit=1,
    algorithmic_change='FL4Q_ENABLE=0', binary_rebuilt=False, source_edited=False,
    duration_s=111, rate=.1, ros_port=11456, prior_failed_preflight=str(OLD/'off_capture'))
(R/'pre_capture_receipt.json').write_text(json.dumps(receipt, indent=2))
print(json.dumps({k:v for k,v in receipt.items() if k!='manifest'}, indent=2))
