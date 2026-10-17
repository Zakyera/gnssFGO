"""Reuse FL4m's isolated replay and build; only add a bounded read-only audit."""
from pathlib import Path
import hashlib,json,shutil,difflib
HERE=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=W/'runs/fixed_lag_retirement_fl4m_20261015'
R=W/'runs/fixed_lag_linear_parity_fl4n_20261015'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
R.mkdir();(R/'include/cbs/bpsam').mkdir(parents=True)
for name in ['protected_before.json','frozen_inputs.json','aligned_inputs.bag','RetirementAudit.h','VioBackend.cpp','validate.py']:
    shutil.copy2(OLD/name,R/name)
for name in ['LinearParityAudit.h','test.cpp','PROTOCOL.txt']:
    shutil.copy2(HERE/name,R/name)
for name in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    (R/name).write_text((OLD/name).read_text().replace(cp(OLD),cp(R)))
shutil.copytree(OLD/'kimera_profile',R/'kimera_profile')
shutil.copytree(OLD/'private_build',R/'private_build',ignore=shutil.ignore_patterns('libkimera_vio.so'))
shutil.copy2(OLD/'include/cbs/bpsam/fixed_lag_objective_audit.h',R/'include/cbs/bpsam')
# Preserve the exact installed ABI: an inline read-only method adds no fields.
h=W/'devel/include/cbs/bpsam/bpsam.h'
s=once(h.read_text(),'  ARIA_POINTER_TYPEDEFS(BPSAM);','  ARIA_POINTER_TYPEDEFS(BPSAM);\n  const GaussianFactorGraph& fl4nLinearCache() const { return linearFactors_; }')
(R/'include/cbs/bpsam/bpsam.h').write_text(s)
original=OLD/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'
s=once(original.read_text(),'#include "RetirementAudit.h"','#include "RetirementAudit.h"\n#include "LinearParityAudit.h"')
s=once(s,'    std::optional<gtsam::Values> reference_pre_update_values;', '''    std::vector<gtsam::NonlinearFactor::shared_ptr> fl4n_retired;
    for (const auto& removal : active_window_cbs_removals)
      if (removal.factor_slot_valid)
        fl4n_retired.push_back(isam_.getFactorsUnsafe().at(removal.factor_index));

    std::optional<gtsam::Values> reference_pre_update_values;''')
s=once(s,'    if (shadow_acceptance_audit_out_ &&','    fl4n::capture(current_timestamp, isam_, fl4n_retired);\n\n    if (shadow_acceptance_audit_out_ &&')
(R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').write_text(s)
(R/'wrapper.diff').write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),s.splitlines(True))))
for name in ['build_commands.json','actual_test_commands.json']:
    c=json.loads((OLD/name).read_text())
    for action in ['compile','link']:c[action]=[a.replace(cp(OLD),cp(R)) for a in c[action]]
    (R/name).write_text(json.dumps(c,indent=2))
s=(OLD/'run_case.py').read_text()
s=once(s,"assert mode in ['remove','retain']","assert mode == 'remove'")
s=once(s,"KIMERA_CBS_REVISION_AUDIT='1')","KIMERA_CBS_REVISION_AUDIT='1',FL4N_CAPTURE=str(D/'linear_parity'))")
(R/'run_case.py').write_text(s)
print(R)
