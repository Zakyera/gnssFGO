"""Refresh only candidate sources during development; preserve all test logs."""
from pathlib import Path
import shutil
HERE=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/fixed_lag_step_acceptance_fl4o_20261016'
assert not (R/'remove_capture').exists(), 'candidate frozen once capture starts'
for name in ['StepAcceptance.h','test.cpp','sanitize.py']:
    shutil.copy2(HERE/name,R/name)
p=R/'include/gtsam_unstable/slam';p.mkdir(parents=True,exist_ok=True)
s=(W/'devel/include/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h').read_text()
needle='  EIGEN_MAKE_ALIGNED_OPERATOR_NEW'
assert s.count(needle)==1
s=s.replace(needle,needle+'''
  // Private diagnostic transaction only. Immutable parameters, measurements,
  // calibrations and object identity remain untouched. These are ALL mutable
  // fields in this type and its SmartStereoProjectionFactor/SmartFactorBase.
  void fl4oRestoreMutableCaches(const SmartStereoProjectionPoseFactor& saved) {
    result_ = saved.result_;
    cameraPosesTriangulation_ = saved.cameraPosesTriangulation_;
    Fs = saved.Fs;
  }
''')
(p/'SmartStereoProjectionPoseFactor.h').write_text(s)
