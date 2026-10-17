"""Refresh private development files; never after data capture."""
from pathlib import Path
import shutil
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/fixed_lag_current_chart_fl4q_20261016'
assert not (R/'remove_capture').exists()
for n in ['CurrentChartFallback.h','test.cpp','PROTOCOL.txt']:shutil.copy2(H/n,R/n)
s=(W/'runs/fixed_lag_linear_parity_fl4n_20261015/include/cbs/bpsam/bpsam.h').read_text()
needle='  ARIA_POINTER_TYPEDEFS(BPSAM);';assert s.count(needle)==1
s=s.replace(needle,needle+'\n'+(H/'CoreRebuild.inc').read_text())
(R/'include/cbs/bpsam/bpsam.h').write_text(s)
