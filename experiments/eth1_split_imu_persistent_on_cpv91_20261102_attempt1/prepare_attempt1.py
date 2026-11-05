"""Preserve failed pre-playback attempt; fresh-path diagnostic destination repair."""
import difflib,hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'eth1_split_imu_persistent_on_cpv91_20261102';Q=W/'runs'/OLD.name/'prepared';P=W/'runs'/H.name/'prepared'
def read(p):return json.loads(p.read_text())
assert read(OLD/'closeout.json')['passed'] and (OLD/'failed.json').exists()
assert 'CBS_E1B_PROFILE_PATH' in (OLD/'capture.log').read_text() and 'AssertionError' in (OLD/'capture.log').read_text()
S=Q/'on_source'
for name in ('rosbag.log','cbs_recorder.log','recordings/cbs.bag','glim_odometry.csv','kimera_odometry.csv','effective_mode_check.json'):assert not (S/name).exists(),name
assert (S/'glim_dump/odom_imu.txt').stat().st_size==0
text=(S/'estimators.log').read_text()
assert all(v not in text for v in ('Initialize Pipeline: timestamp','KIMERA_BACKEND_SPINONCE_TIMING_ROW,','KIMERA_CBS_REVISION_EDGE_ROW,'))
assert not P.exists();P.mkdir(parents=True);diff=[]
def copy(src,dst):
    assert not dst.exists();shutil.copy2(src,dst);old=src.read_text()
    new=old.replace(OLD.name,H.name).replace('11591','11592')
    if src.name=='glim_profile_on.launch':
        assert new.count('/off_source/e1b_profile.jsonl')==1
        new=new.replace('/off_source/e1b_profile.jsonl','/on_source/e1b_profile.jsonl')
    dst.write_text(new);diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(src),tofile=str(dst)))
for name in ('source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py','map_guard.py','coverage_checks.py',
             'active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch'):
    copy(Q/name,P/name)
for name in ('controller.py','launch.py','post_closeout.py','coverage_control.py','freeze.py','evaluate.py','frame_audit.py','PROTOCOL.txt','PRELAUNCH_REVIEW.txt'):copy(OLD/name,H/name)
shutil.copytree(Q/'private_build',P/'private_build');shutil.copytree(Q/'input_profiles',P/'input_profiles')
for name in ('input_audit.json','expected_headers.json'):shutil.copy2(Q/name,P/name)
with (H/'startup_repair.diff').open('x') as f:f.writelines(diff)
with (H/'startup_failure_evidence.json').open('x') as f:json.dump(dict(previous=str(OLD),previous_prepared=str(Q),
    closeout=read(OLD/'closeout.json'),cause='copied diagnostic path off_source instead of on_source',
    no_sensor_playback=True,no_recorders=True,no_native_updates=True,no_estimator_setting_change=True),f,indent=2)
print(P)
