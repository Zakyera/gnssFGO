"""Fresh host-import repair, preserving every failed infrastructure attempt."""
import json, shutil
from pathlib import Path
H=Path(__file__).resolve().parent
old='eth1_split_imu_live_pair_cpv95_20261103_attempt2'
F=H.parent/old; W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
Q=W/'runs'/old/'prepared'; P=W/'runs'/H.name/'prepared'
assert json.loads((F/'closeout.json').read_text())['passed']
assert json.loads((F/'failed.json').read_text())['stage']=='capture_off'
assert not any(Q.glob('full_*_source')) and not any(Q.glob('full_*_receiver'))
assert json.loads((Q/'full_off_control/owned_job_cleanup.json').read_text())=={'master':0}
assert not P.exists()
def copy(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    if src.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=dst.read_text().replace(old,H.name)
        if src.name=='source.env':
            text=text.replace('CBSMS_ROS_MASTER_PORT=11599','CBSMS_ROS_MASTER_PORT=11601').replace('CBSMS_ROS_MASTER_PORT=11600','CBSMS_ROS_MASTER_PORT=11602')
        if src.name in ('run_case_on.py','admission.py'):
            text=text.replace('11599','11601').replace('11600','11602')
        if src.name in ('full_off.json','full_on.json'):
            text=text.replace('http://127.0.0.1:11599','http://127.0.0.1:11601').replace('http://127.0.0.1:11600','http://127.0.0.1:11602')
        dst.write_text(text)
for src in Q.rglob('*'):
    rel=src.relative_to(Q)
    if any(part.startswith('full_') and part.endswith(('_control','_source','_receiver')) for part in rel.parts):continue
    if src.is_file() and src.name not in ('protected_manifest.json','preflight.json','runtime_frame_fixture.json') and '__pycache__' not in src.parts:
        copy(src,P/rel)
for name in ('admission.py','controller.py','evaluate.py','verify_report.py','launch.py','post_closeout.py','frame_off.py','frame_on.py','test_live_adapter.py','PROTOCOL.txt'):
    copy(F/name,H/name)
print(P)
