"""Fresh-root template copies and mechanical path/label rebasing only. No run."""
import difflib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
H85=E/'eth1_split_imu_off60_cpv85_20261101_attempt1';Q85=W/'runs'/H85.name/'prepared'
H86=E/'eth1_cpt7_control_off60_cpv86_20261102';Q86=W/'runs'/H86.name/'prepared'
P=W/'runs'/H.name/'prepared'
assert not P.exists();P.mkdir(parents=True)
assert json.loads((H85/'report_completion_1/complete.json').read_text())['report_complete']
assert json.loads((H86/'complete.json').read_text())['passed']
diff=[]
def rebase(src,dst,oldname):
    assert not dst.exists();shutil.copy2(src,dst)
    old=src.read_text();new=old.replace(oldname,H.name)
    new=new.replace('11587','11589').replace('11588','11589')
    new=new.replace('CP-V85','CP-V88').replace('CP-V86','CP-V88').replace('CPV86','CPV88').replace('cpv85_frame','cpv88_frame')
    # Semantic changes are applied separately with apply_patch and parity-checked.
    dst.write_text(new)
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(src),tofile=str(dst)))
for name in ['source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py','map_guard.py','coverage_checks.py',
             'active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch']:
    rebase(Q85/name,P/name,H85.name)
for name in ['coverage_control.py','controller.py','launch.py','post_closeout.py','evaluate.py']:
    rebase(H86/name,H/name,H86.name)
shutil.copytree(Q85/'private_build',P/'private_build')
shutil.copytree(Q85/'input_profiles',P/'input_profiles')
shutil.copy2(Q86/'expected_headers.json',P/'expected_headers.json')
with (H/'mechanical_rebase.diff').open('x') as f:f.write(''.join(diff))
print(P)
