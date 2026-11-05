"""Pure adapter fixture check in actual Noetic; no ROS or estimator execution."""
import json
from pathlib import Path
import live_receiver_protocol as protocol
from frame_adapter import native_to_cpt7
root=Path(__file__).resolve().parent
for mode in ('off','on'):protocol.validate_config(json.loads((root.parent/f'full_{mode}.json').read_text()))
items=json.loads((root/'runtime_frame_fixture.json').read_text())
for item in items:
    out=native_to_cpt7('kimera',item['native'],protocol.FRAME_CALIBRATION['T_alphasense_cpt7'])
    assert out==item['expected'],'host/container conversion differs'
print(json.dumps(dict(passed=True,fixtures=len(items),host_container_payloads_bit_identical=True,requires_scipy=False)))
