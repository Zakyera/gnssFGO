"""Create the isolated candidate, never overwrite previous data/builds."""
from pathlib import Path
import json,hashlib,shutil,difflib
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
N=W/'runs/fixed_lag_linear_parity_fl4n_20261015'
O=W/'runs/fixed_lag_step_acceptance_fl4o_20261016'
R=W/'runs/fixed_lag_current_chart_fl4q_20261016'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
protected=json.loads((N/'protected_before.json').read_text())
for p,h in protected.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
R.mkdir();(R/'include/cbs/bpsam').mkdir(parents=True)
for n in ['protected_before.json','frozen_inputs.json','aligned_inputs.bag','RetirementAudit.h','VioBackend.cpp','LinearParityAudit.h']:
    shutil.copy2(N/n,R/n)
for n in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    (R/n).write_text((N/n).read_text().replace(cp(N),cp(R)))
shutil.copytree(N/'kimera_profile',R/'kimera_profile')
shutil.copytree(N/'private_build',R/'private_build',ignore=shutil.ignore_patterns('libkimera_vio.so'))
shutil.copy2(N/'include/cbs/bpsam/fixed_lag_objective_audit.h',R/'include/cbs/bpsam')
shutil.copy2(W/'runs/fixed_lag_same_graph_fl4l_20261015/OriginalChartPrior.h',R)
(R/'include/gtsam_unstable/slam').mkdir(parents=True)
shutil.copy2(O/'include/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h',R/'include/gtsam_unstable/slam')
s=(N/'include/cbs/bpsam/bpsam.h').read_text()
s=once(s,'  ARIA_POINTER_TYPEDEFS(BPSAM);','  ARIA_POINTER_TYPEDEFS(BPSAM);\n'+(H/'CoreRebuild.inc').read_text())
(R/'include/cbs/bpsam/bpsam.h').write_text(s)
original=(O/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').read_text()
s=original.replace('StepAcceptance.h','CurrentChartFallback.h').replace('fl4o::','fl4q::').replace('fl4oUpdateImpl','fl4qUpdateImpl')
s=once(s,'fl4q::accept(current_timestamp,isam_,fl4m_before);','fl4q::accept(current_timestamp,isam_,fl4m_before,constrainedKeys);')
(R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').write_text(s)
(R/'wrapper.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True))))
for n in ['build_commands.json','actual_test_commands.json']:
    c=json.loads((O/n).read_text())
    for action in ['compile','link']:c[action]=[a.replace(cp(O),cp(R)) for a in c[action]]
    (R/n).write_text(json.dumps(c,indent=2))
for n in ['build.py','sanitize.py']:
    (R/n).write_text((H.parent/'fixed_lag_direction_fl4p_20261016'/n).read_text().replace('FL4P','FL4Q'))
s=(O/'run_case.py').read_text().replace('11436','11439').replace('FL4O','FL4Q')
s=s.replace('--duration=94','--duration=30').replace('duration=94','duration=30').replace('+1150','+460')
(R/'run_case.py').write_text(s)
for n in ['CurrentChartFallback.h','test.cpp','PROTOCOL.txt']:shutil.copy2(H/n,R/n)
print(R)
