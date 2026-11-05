from pathlib import Path
H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared'
V=H/'receiver_validation'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
BASE_H=E/'spx3_split_imu_live_pair_cpv111_20261104'
BASE_P=W/'runs'/BASE_H.name/'prepared'
FIX_H=E/'prior_anchor_integration_cpv102_20261103'
FIX_K=W/'runs'/FIX_H.name
QUAL_H=E/'eth1_prior_fix_live_cpv103_20261103'
QUAL_P=W/'runs'/QUAL_H.name/'prepared'
ORIGINAL=H

FAILED_H=E/'leica2_split_imu_live_pair_cpv112_20261105'
FAILED_P=W/"runs"/FAILED_H.name/"prepared"
