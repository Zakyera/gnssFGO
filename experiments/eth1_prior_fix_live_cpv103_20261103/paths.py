from pathlib import Path
H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared'
V=H/'receiver_validation'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
BASE_H=E/'eth1_split_imu_repeat_cpv99_20261103'
BASE_P=W/'runs'/BASE_H.name/'prepared'
FIX_H=E/'prior_anchor_integration_cpv102_20261103'
FIX_K=W/'runs'/FIX_H.name
