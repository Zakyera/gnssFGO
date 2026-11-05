"""Mechanical private copy of frozen bridge and finite CP-V51 controller templates."""
import difflib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
S=W/'runs/local_revision_covariance_cpi1_20261019';HS=R/'experiments/eig2_hesai_insertion_cpv51_20261030'
old=(S/'cbs_bridge.cpp').read_text();s='#include "GlimObserver.h"\n'+old
marker='  updatePoseTimestamps(new_stamps);'
assert s.count(marker)==1
s=s.replace(marker,marker+'\n  cpv54::capture(smoother, has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u, "before", &new_values, &new_factors);')
marker='  publishOdometryAndRerun(smoother);'
assert s.count(marker)==1
s=s.replace(marker,marker+'\n  cpv54::capture(smoother, has_last_pose_key_ ? poseIndex(last_pose_key_) : 0u, "after");')
with (K/'cbs_bridge.cpp').open('x') as f:f.write(s)
shutil.copy2(S/'RevisionCovariance.h',K/'RevisionCovariance.h')
with (H/'observer.diff').open('x') as f:f.writelines(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/'cbs_bridge.cpp'),tofile=str(K/'cbs_bridge.cpp')))
for name in ('controller.py','launch.py','validate_snapshots.py'):
 s=(HS/name).read_text().replace('EIG2-CPV51','EIG2-CPV54')
 if name=='controller.py':
  marker="        save(mode+'_validated.json',dict(passed=True,finished=now(),gates=list(map(str,gate_paths(mode)))))"
  assert marker in s
  s=s.replace(marker,"        step('validate_'+mode+'_glim',[PY,str(H/'validate_glim.py'),mode])\n"+marker)
 with (H/name).open('x') as f:f.write(s)
