"""Fresh-root mechanical harness rebase only; no source execution."""
import difflib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'eth1_alpha_bias_rw_full_off_cpv89_20261102';Q=W/'runs'/OLD.name/'prepared';P=W/'runs'/H.name/'prepared'
assert json.loads((OLD/'post_closeout.json').read_text())['passed']
assert not P.exists();P.mkdir(parents=True);diff=[]
def copy(src,dst):
    assert not dst.exists();shutil.copy2(src,dst)
    old=src.read_text();new=old.replace(OLD.name,H.name).replace('11590','11591').replace('CP-V89','CP-V91').replace('CPV89','CPV91').replace('cpv89_frame','cpv91_frame')
    dst.write_text(new)
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(src),tofile=str(dst)))
for name in ('source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py','map_guard.py','coverage_checks.py',
             'active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch'):
    copy(Q/name,P/name)
for name in ('controller.py','launch.py','post_closeout.py','coverage_control.py'):copy(OLD/name,H/name)
shutil.copytree(Q/'private_build',P/'private_build');shutil.copytree(Q/'input_profiles',P/'input_profiles')
for name in ('input_audit.json','expected_headers.json'):shutil.copy2(Q/name,P/name)
with (H/'mechanical_rebase.diff').open('x') as f:f.writelines(diff)
print(P)
