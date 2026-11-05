"""Mechanical copy/rebase of frozen harness and sensor profile into NEW roots."""
from pathlib import Path
import shutil
import re
H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S = W/'runs/eig2_symmetric_persistent_full_cpv84_20261101_attempt1/prepared'
P = W/'runs'/H.name/'prepared'
assert not P.exists(), 'No overwriting or re-preparation'
P.mkdir(parents=True)
files = ['source_runner_on.sh', 'source.env', 'verify_effective_on.py',
         'verify_hesai_startup.py', 'map_guard.py', 'coverage_checks.py',
         'active_window_on.launch', 'glim_profile_on.launch',
         'grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros.launch',
         'kimera_vio_ros_newer_college_2021_stereo.launch']
old = str(S).replace(str(W), '/workspace/cbs_gtsam4.3')
new = str(P).replace(str(W), '/workspace/cbs_gtsam4.3')
for name in files:
    shutil.copy2(S/name, P/name)
    text = (P/name).read_text().replace(old, new)
    (P/name).write_text(text)
shutil.copytree(S/'private_build', P/'private_build')
base = W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared/input_profiles'
shutil.copytree(base, P/'input_profiles')
# Only change body-frame calibration; all vision/backend/initialization settings stay fixed.
alpha = W/'src/Kimera-VIO/params/GrandTourEth1StereoPosZStereoGuess'
for name in ['LeftCameraParams.yaml', 'RightCameraParams.yaml']:
    oldtext=(P/'input_profiles/kimera_profile'/name).read_text()
    alphatext=(alpha/name).read_text()
    block = r'T_BS:\n.*?\n\n'
    replacement = re.search(block, alphatext, re.S)[0]
    text, count = re.subn(block, lambda _: replacement, oldtext, count=1, flags=re.S)
    assert count == 1
    text=text.replace('shared cpt7_imu body-frame', 'alphasense_imu body-frame')
    (P/'input_profiles/kimera_profile'/name).write_text(text)
print(P)
