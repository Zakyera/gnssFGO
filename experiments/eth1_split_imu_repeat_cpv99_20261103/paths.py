from pathlib import Path
H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared'
V=H/'receiver_validation'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
BASE_H=E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1'
BASE_P=W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared'
# The reused evaluator pins this experiment's own preflight reports.
ORIGINAL=H
