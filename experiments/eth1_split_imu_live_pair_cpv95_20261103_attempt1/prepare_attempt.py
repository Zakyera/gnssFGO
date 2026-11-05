"""Fresh pre-capture infrastructure attempt; preserve original failure untouched."""
import json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;oldname='eth1_split_imu_live_pair_cpv95_20261103'
F=H.parent/oldname;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';Q=W/'runs'/oldname/'prepared'
assert json.loads((F/'closeout.json').read_text())['passed']
assert json.loads((F/'failed.json').read_text())['stage']=='start_containers'
assert not (F/'capture_off_process.json').exists() and not any(Q.glob('full_*_source')) and not any(Q.glob('full_*_receiver'))
assert not P.exists()
def copy(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    if src.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=dst.read_text().replace(oldname,H.name)
        if src.name=='source.env':text=text.replace('CBSMS_ROS_MASTER_PORT=11595','CBSMS_ROS_MASTER_PORT=11597').replace('CBSMS_ROS_MASTER_PORT=11596','CBSMS_ROS_MASTER_PORT=11598')
        if src.name in ('run_case_on.py','admission.py'):text=text.replace('11595','11597').replace('11596','11598')
        if src.name in ('full_off.json','full_on.json'):text=text.replace('http://127.0.0.1:11595','http://127.0.0.1:11597').replace('http://127.0.0.1:11596','http://127.0.0.1:11598')
        dst.write_text(text)
for p in Q.rglob('*'):
    if p.is_file() and p.name not in ('preflight.json','protected_manifest.json') and '__pycache__' not in p.parts:copy(p,P/p.relative_to(Q))
for name in ('admission.py','controller.py','evaluate.py','verify_report.py','launch.py','post_closeout.py','frame_off.py','frame_on.py','test_live_adapter.py','PROTOCOL.txt'):
    copy(F/name,H/name)
print(P)
