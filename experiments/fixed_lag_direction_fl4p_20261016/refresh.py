"""Copy revised diagnostic sources before capture, never after launch."""
from pathlib import Path
import shutil
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_direction_fl4p_20261016')
assert not (R/'remove_capture/started.json').exists()
for n in ['test.cpp','DirectionAudit.h','build.py','sanitize.py','validate.py']:
    shutil.copy2(H/n,R/n)
