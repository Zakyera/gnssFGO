"""Mechanical private overlay generation; production files remain untouched."""
from pathlib import Path
import hashlib, json, shutil, shlex, difflib
HERE = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT = W/'runs/fixed_lag_retirement_fl4m_20261015'
OLD = W/'runs/fixed_lag_same_graph_fl4l_20261015'
FROZEN = W/'runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared'
def cp(p): return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def once(s,a,b):
    assert s.count(a)==1, (a,s.count(a))
    return s.replace(a,b)
ROOT.mkdir()
protected = json.loads((FROZEN/'protected_manifest.json').read_text())
assert all(sha(Path(p))==h for p,h in protected.items())
(ROOT/'protected_before.json').write_text(json.dumps(protected,indent=2))
for name in ['RetirementAudit.h','test.cpp','prepare_inputs.py','run_case.py','validate.py','PROTOCOL.txt']:
    shutil.copy2(HERE/name, ROOT/name)
inc = ROOT/'include/cbs/bpsam'; inc.mkdir(parents=True)
shutil.copy2(W/'src/cbs/include/cbs/bpsam/fixed_lag_objective_audit.h',inc)
original = W/'src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'
s = original.read_text()
# Installed runtime lacks the unused experimental GLIM linearizer overload.
# Keep its frozen API; this overload is unrelated to Kimera or this test.
s=once(s,'''  void setMarginalizationGraph(
      cbs::BPSAM::MarginalizationType type,
      const cbs::BPSAM::MarginalizationLinearizer& linearizer) {
    if (type == cbs::BPSAM::MarginalizationType::FULL) {
      isam_.setMarginalizationGraph(type);
      return;
    }
    isam_.setMarginalizationGraph(type, activeLocalKeys(), linearizer);
  }

''','')
s = once(s, '#pragma once', '#pragma once\n#include "RetirementAudit.h"')
s = once(s, '    updateKeyTimestampMap(timestamps);', '''    const bool fl4m_audit = std::getenv("FL4M_AUDIT") != nullptr;
    gtsam::Values fl4m_before, fl4m_theta;
    if (fl4m_audit) {
      fl4m_before = isam_.calculateEstimate();
      for (const auto& kv : newTheta)
        if (!fl4m_before.exists(kv.key)) fl4m_before.insert(kv.key,kv.value);
      fl4m_theta = isam_.getLinearizationPoint();
    }
    updateKeyTimestampMap(timestamps);''')
s = once(s, '''      factors_to_remove.insert(factors_to_remove.end(),
                               cbs_slots_to_remove.begin(),
                               cbs_slots_to_remove.end());''', '''      if (!cbs::fl4mRetain()) {
        factors_to_remove.insert(factors_to_remove.end(),
                                 cbs_slots_to_remove.begin(),
                                 cbs_slots_to_remove.end());
      } else {
        for (const auto& removal : active_window_cbs_removals) {
          if (!removal.factor_slot_valid ||
              (std::find(marginalizableKeys.begin(), marginalizableKeys.end(), removal.from_pose_key) == marginalizableKeys.end() &&
               std::find(marginalizableKeys.begin(), marginalizableKeys.end(), removal.to_pose_key) == marginalizableKeys.end()))
            throw std::runtime_error("FL4m retained expiry has no marginalizing endpoint");
        }
      }''')
s = once(s, '    if (shadow_acceptance_audit_out_ &&', '''    if (fl4m_audit)
      cbs::fl4mAudit(current_timestamp, isam_.getFactorsUnsafe(),
          fl4m_before, isam_.calculateEstimate(), fl4m_theta,
          isam_.getLinearizationPoint(), active_window_cbs_removals.size());

    if (shadow_acceptance_audit_out_ &&''')
(inc/original.name).write_text(s)
(ROOT/'wrapper.diff').write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),s.splitlines(True))))
# This production TU gains only a post-binding payload identity log.
src=W/'src/Kimera-VIO/src/backend/VioBackend.cpp'
s=src.read_text()
s=once(s,'    odom_belief.relax_factor = belief.relax_factor;', '''    LOG(INFO) << std::setprecision(17) << "FL4M_FROZEN_FACTOR_ROW,"
              << cur_id << "," << belief.sender_from_pose_index << ","
              << belief.sender_to_pose_index << "," << from_frame_id << ","
              << to_frame_id << ","
              << cbs::relative_pose_covariance::matrixToken(gtsam::Pose3::Logmap(odom_belief.measured_from_to))
              << "," << cbs::relative_pose_covariance::matrixToken(odom_belief.covariance);
    odom_belief.relax_factor = belief.relax_factor;''')
(ROOT/'VioBackend.cpp').write_text(s)
(ROOT/'VioBackend.diff').write_text(''.join(difflib.unified_diff(src.read_text().splitlines(True),s.splitlines(True))))
commands=json.loads((OLD/'build_commands.json').read_text())
for action in ['compile','link']:
    commands[action]=[a.replace(cp(OLD),cp(ROOT)) for a in commands[action]]
(ROOT/'build_commands.json').write_text(json.dumps(commands,indent=2))
(ROOT/'private_build').mkdir()
for p in (FROZEN/'private_build').iterdir():
    if p.is_file() and p.name != 'libkimera_vio.so': shutil.copy2(p,ROOT/'private_build'/p.name)
for name in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    s=(OLD/name).read_text().replace(cp(OLD),cp(ROOT))
    if name=='kimera_vio_ros.launch':
        s=once(s,'--cbs_experimental_native_imu_mean_alignment=true','--cbs_experimental_native_imu_mean_alignment=false')
    (ROOT/name).write_text(s)
shutil.copytree(OLD/'on_source_capture/kimera_profile', ROOT/'kimera_profile')
# Reuse the existing integration-test compile flags/link lines, replacing only
# its test TU/output and placing the private header first.
B=W/'runs/fixed_lag_objective_fl4k_20261015/release/CMakeFiles/shadow_objective_integration_test.dir'
flags={line.split(' = ',1)[0]:line.split(' = ',1)[1] for line in (B/'flags.make').read_text().splitlines() if ' = ' in line}
compile_test=['/usr/bin/c++','-I'+cp(ROOT),'-I'+cp(ROOT/'include')]
for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']:
    compile_test += [a for a in shlex.split(flags[k]) if '/src/cbs/tests/../include' not in a]
compile_test+=['-c',cp(ROOT/'test.cpp'),'-o',cp(ROOT/'test.o')]
link=shlex.split((B/'link.txt').read_text())
link[link.index('-o')+1]=cp(ROOT/'test')
i=next(i for i,x in enumerate(link) if x.endswith('shadow_objective_integration_test.cpp.o'))
link[i]=cp(ROOT/'test.o')
(ROOT/'test_commands.json').write_text(json.dumps({'compile':compile_test,'link':link},indent=2))
print(ROOT)
