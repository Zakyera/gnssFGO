"""Repair pre-admission staged copies only; never touches original or captured data."""
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;old='eth1_split_imu_live_pair_cpv95_20261103'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name/'prepared';Q=W/'runs'/old/'prepared'
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not any(P.glob('full_*_source')) and not any(P.glob('full_*_receiver'))
changes=[]
for src in Q.rglob('*'):
    if not src.is_file() or src.name in ('protected_manifest.json','preflight.json') or '__pycache__' in src.parts:continue
    dst=P/src.relative_to(Q);data=src.read_bytes()
    if src.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=data.decode().replace(old,H.name)
        if src.name=='source.env':text=text.replace('CBSMS_ROS_MASTER_PORT=11595','CBSMS_ROS_MASTER_PORT=11597').replace('CBSMS_ROS_MASTER_PORT=11596','CBSMS_ROS_MASTER_PORT=11598')
        if src.name=='run_case_on.py':text=text.replace('11595+(args.mode', '11597+(args.mode')
        if src.name in ('full_off.json','full_on.json'):text=text.replace('http://127.0.0.1:11595','http://127.0.0.1:11597').replace('http://127.0.0.1:11596','http://127.0.0.1:11598')
        data=text.encode()
    if dst.read_bytes()!=data:
        changes.append(dict(path=str(dst),old_sha256=hashlib.sha256(dst.read_bytes()).hexdigest(),correct_sha256=hashlib.sha256(data).hexdigest()))
        dst.write_bytes(data)
for mode,name in [('off','eth1_alpha_bias_rw_full_off_cpv89_20261102'),('on','eth1_split_imu_persistent_on_cpv91_20261102_attempt1')]:
    for f in ('input_audit.json','expected_headers.json'):
        assert (P/mode/f).read_bytes()==(W/'runs'/name/'prepared'/f).read_bytes()
with (H/'static_copy_repair.json').open('x') as f:json.dump(dict(passed=True,staged_files=changes,originals_untouched=True,source_captures=0,receiver_captures=0),f,indent=2)
print(json.dumps(changes,indent=2))
