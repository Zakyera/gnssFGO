"""Generate a private diagnostic overlay; never overwrite prior experiments."""
from pathlib import Path
import json,shutil,difflib,hashlib
HERE=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=W/'runs/fixed_lag_linear_parity_fl4n_20261015'
R=W/'runs/fixed_lag_direction_fl4p_20261016'
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
protected=json.loads((OLD/'protected_before.json').read_text())
for p,h in protected.items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
R.mkdir();(R/'include/cbs/bpsam').mkdir(parents=True)
for name in ['protected_before.json','frozen_inputs.json','aligned_inputs.bag','RetirementAudit.h','VioBackend.cpp','LinearParityAudit.h']:
    shutil.copy2(OLD/name,R/name)
for name in ['DirectionAudit.h','test.cpp','PROTOCOL.txt','build.py','sanitize.py','validate.py']:
    shutil.copy2(HERE/name,R/name)
shutil.copy2(W/'runs/fixed_lag_same_graph_fl4l_20261015/OriginalChartPrior.h',R)
for name in ['grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch']:
    (R/name).write_text((OLD/name).read_text().replace(cp(OLD),cp(R)))
shutil.copytree(OLD/'kimera_profile',R/'kimera_profile')
shutil.copytree(OLD/'private_build',R/'private_build',ignore=shutil.ignore_patterns('libkimera_vio.so'))
for name in ['fixed_lag_objective_audit.h','bpsam.h']:
    shutil.copy2(OLD/'include/cbs/bpsam'/name,R/'include/cbs/bpsam'/name)
original=OLD/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'
s=once(original.read_text(),'#include "LinearParityAudit.h"','#include "DirectionAudit.h"')
s=once(s,'    fl4n::capture(current_timestamp, isam_, fl4n_retired);','    fl4p::capture(current_timestamp, isam_, fl4m_before);')
(R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h').write_text(s)
(R/'wrapper.diff').write_text(''.join(difflib.unified_diff(original.read_text().splitlines(True),s.splitlines(True))))
# Inline helpers add no fields/vtable entries. Reset affects diagnostic clones only.
p=W/'devel/include/gtsam_unstable/slam/SmartStereoProjectionFactor.h'
s=once(p.read_text(),'  /// shorthand for a smart pointer to a factor', '''  void fl4pResetCache() { cameraPosesTriangulation_.clear(); result_=TriangulationResult::Degenerate(); }
  std::string fl4pCacheSignature() const {
    std::ostringstream out; out << std::setprecision(17);
    out << result_.valid() << result_.degenerate() << result_.behindCamera() << result_.outlier() << result_.farPoint();
    if(result_) out << (*result_).transpose();
    for(const auto& p:cameraPosesTriangulation_) out << p.matrix();
    for(const auto& f:Fs) out << f;
    return out.str();
  }
  /// shorthand for a smart pointer to a factor''')
s='#include <sstream>\n#include <iomanip>\n'+s
(R/'include/gtsam_unstable/slam').mkdir(parents=True)
(R/'include/gtsam_unstable/slam/SmartStereoProjectionFactor.h').write_text(s)
for name in ['build_commands.json','actual_test_commands.json']:
    c=json.loads((OLD/name).read_text())
    for action in ['compile','link']:c[action]=[a.replace(cp(OLD),cp(R)) for a in c[action]]
    if name=='actual_test_commands.json':c['link'].append(cp(W/'devel/lib/libgtsam_unstable.so.4.3a1'))
    (R/name).write_text(json.dumps(c,indent=2))
s=(OLD/'run_case.py').read_text().replace('11436','11438')
s=once(s,"FL4N_CAPTURE=str(D/'linear_parity')","FL4P_CAPTURE=str(D/'direction')")
s=s.replace('--duration=94','--duration=23').replace('duration=94','duration=23').replace('+1150','+380')
# Bounded wall timeout and hard23s bag cap. Missing update89 is a failure.
s=once(s,'    assert launch.poll() is None\n    (D/\'complete.json\')',"    assert launch.poll() is None\n    assert (D/'direction/update_89/summary.txt').exists(), 'missing update89 capture'\n    (D/'complete.json')")
(R/'run_case.py').write_text(s)
manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in R.rglob('*') if p.is_file()}
(R/'prepared_manifest.json').write_text(json.dumps(manifest,indent=2))
print(R, 'protected',len(protected))
