"""Supplemental gates and ON-only continuation; frozen attempt3 inputs stay put."""
from pathlib import Path
H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ORIGINAL = E / 'eth1_split_imu_live_pair_cpv95_20261103_attempt3'
P = W / 'runs' / ORIGINAL.name / 'prepared'
V = H / 'receiver_validation'
PY = '/home/yeranis/.local/share/pipx/venvs/evo/bin/python'

