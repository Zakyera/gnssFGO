"""Private source overlay, preserving the installed ABI and all old artifacts."""
from pathlib import Path
import json, shutil, difflib
HERE=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=W/'runs/fixed_lag_linear_parity_fl4n_20261015'
R=W/'runs/fixed_lag_step_acceptance_fl4o_20261016'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
R.mkdir();(R/'include/cbs/bpsam').mkdir(parents=True)
for n in ['protected_before.json','frozen_inputs.json','aligned_inputs.bag','RetirementAudit.h','VioBackend.cpp','validate.py','build.py','LinearParityAudit.h']:
    shutil.copy2(OLD/n,R/n)
for n in ['StepAcceptance.h','test.cpp','PROTOCOL.txt','sanitize.py']:
    shutil.copy2(HERE/n,R/n)
for n in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    (R/n).write_text((OLD/n).read_text().replace(cp(OLD),cp(R)))
shutil.copytree(OLD/'kimera_profile',R/'kimera_profile')
shutil.copytree(OLD/'private_build',R/'private_build',ignore=shutil.ignore_patterns('libkimera_vio.so'))
shutil.copy2(OLD/'include/cbs/bpsam/fixed_lag_objective_audit.h',R/'include/cbs/bpsam')
s=(OLD/'include/cbs/bpsam/bpsam.h').read_text()
s=once(s,'  ARIA_POINTER_TYPEDEFS(BPSAM);', '''  ARIA_POINTER_TYPEDEFS(BPSAM);
  void fl4oCommitAcceptedValues(const Values& accepted) {
    if (!std::holds_alternative<ISAM2GaussNewtonParams>(BaseSAM::params_.optimizationParams) ||
        params_.use_temporary_cbs_linear_factors ||
        params_.cbs_odom_factor_mode != CbsOdomFactorMode::ActiveWindowTemporary)
      throw std::runtime_error("FL4O incompatible optimizer/CBS mode");
    if (accepted.keys() != theta_.keys())
      throw std::runtime_error("FL4O key-set mismatch");
    const auto next = theta_.localCoordinates(accepted);
    for (const auto& kv : next) if (!kv.second.allFinite())
      throw std::runtime_error("FL4O nonfinite accepted delta");
    // Accepted nonlinear step, not a new Gaussian minimum. As with dogleg,
    // retain the undamped tree for covariance/marginalization, and prevent a
    // lazy getDelta() from replacing this accepted step with the rejected GN.
    delta_ = next;
    deltaReplacedMask_.clear();
    tmp_marginals_.reset();
  }''')
(R/'include/cbs/bpsam/bpsam.h').write_text(s)
original=(OLD/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').read_text()
s=once(original,'#include "LinearParityAudit.h"','#include "LinearParityAudit.h"\n#include "StepAcceptance.h"')
needle='''    const bool debug = ISDEBUG("IncrementalFixedLagBpsamSmoother update");'''
s=once(s,needle,'''    if (fl4o::enabled()) {
      return fl4o::transaction(*this, newFactors, [&] {
        return fl4oUpdateImpl(newFactors,newTheta,timestamps,factorsToRemove);
      });
    }
    return fl4oUpdateImpl(newFactors,newTheta,timestamps,factorsToRemove);
  }

  gtsam::FixedLagSmoother::Result fl4oUpdateImpl(
      const gtsam::NonlinearFactorGraph& newFactors,
      const gtsam::Values& newTheta,
      const gtsam::FixedLagSmoother::KeyTimestampMap& timestamps,
      const gtsam::FactorIndices& factorsToRemove) {
    const bool debug = ISDEBUG("IncrementalFixedLagBpsamSmoother update");''')
s=once(s,'const bool fl4m_audit = std::getenv("FL4M_AUDIT") != nullptr;',
       'const bool fl4m_audit = std::getenv("FL4M_AUDIT") != nullptr || fl4o::enabled();')
s=once(s,'    if (fl4m_audit)\n      cbs::fl4mAudit',
       '    if (fl4o::enabled()) fl4o::accept(current_timestamp,isam_,fl4m_before);\n\n    if (fl4m_audit)\n      cbs::fl4mAudit')
s=once(s,'    fl4n::capture(current_timestamp, isam_, fl4n_retired);','')
(R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').write_text(s)
(R/'wrapper.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True))))
for n in ['build_commands.json','actual_test_commands.json']:
    c=json.loads((OLD/n).read_text())
    for a in ['compile','link']:c[a]=[x.replace(cp(OLD),cp(R)) for x in c[a]]
    if n=='actual_test_commands.json':c['link'].append(cp(W/'devel/lib/libgtsam_unstable.so.4.3a1'))
    (R/n).write_text(json.dumps(c,indent=2))
s=(OLD/'run_case.py').read_text()
s=once(s,"FL4N_CAPTURE=str(D/'linear_parity')", "FL4O_ENABLE='1',FL4O_LOG=str(D/'acceptance.csv')")
s=once(s,"assert (R/'test_passed.json').exists()", "assert (R/'test_passed.json').exists()\nassert json.loads((R/'sanitizer_passed.json').read_text())['passed']")
s=once(s,"    maps=Path('/proc/%d/maps'%pid).read_text()", "    maps=Path('/proc/%d/maps'%pid).read_text()\n    node_start=Path('/proc/%d/stat'%pid).read_text().split()[21]")
s=once(s,"    (D/'preflight.json').write_text", "    assert env['FL4O_ENABLE']=='1' and env['FL4O_LOG']==str(D/'acceptance.csv')\n    (D/'preflight.json').write_text")
s=once(s,"        assert launch.poll() is None and rec.poll() is None,'owned process exited'", "        assert launch.poll() is None and rec.poll() is None,'owned process exited'\n        assert Path('/proc/%d/stat'%pid).read_text().split()[21]==node_start, 'Kimera node exited/replaced'")
s=once(s,"    assert launch.poll() is None\n", "    assert launch.poll() is None\n    assert Path('/proc/%d/stat'%pid).read_text().split()[21]==node_start, 'Kimera node exited/replaced'\n    assert 'FL4O' not in (D/'kimera.log').read_text(), 'candidate error logged'\n")
(R/'run_case.py').write_text(s)
exec((HERE/'refresh_private.py').read_text())
# After the development receipt, these are the exact final controllers/gates.
for n in ['run_case.py','validate.py','build.py']:
    if (HERE/n).exists():shutil.copy2(HERE/n,R/n)
print(R)
