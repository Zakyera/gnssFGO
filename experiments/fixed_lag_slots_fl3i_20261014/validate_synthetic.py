import json,re
from driver import ROOT,HERE,REPO,BASE,sha,protected
def files(root):return {str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file()}
for name in ('test','test_repeat','sanitest'):
    result=json.loads((ROOT/(name+'.json')).read_text());assert result['returncode']==0 and result['protected_unchanged']
    log=(ROOT/(name+'.log')).read_text();assert 'FL3I_LIFECYCLE_GATE PASS' in log
    assert not re.search(r'ERROR: AddressSanitizer|runtime error:|FAIL ',log)
assert files(ROOT/'test_outputs')==files(ROOT/'test_repeat_outputs')
assert (ROOT/'test.log').read_bytes()==(ROOT/'test_repeat.log').read_bytes()
sanitizer_identical=files(ROOT/'test_outputs')==files(ROOT/'sanitest_outputs')
def names(s):return s.replace('fl3h','fl3i').replace('FL3H','FL3I').replace('FL3h','FL3i').replace('RecordedFixedLagSmoother','SlotFixedLagSmoother').replace('"SlotFixedLagSmoother.h"','"RecordedFixedLagSmoother.h"')
header=names((BASE/'RecordedFixedLagSmoother.h').read_text())
header=header.replace('#include <gtsam/nonlinear/ISAM2.h>','#include "SlotISAM2.h"').replace('isam_ = gtsam::ISAM2(parameters);','isam_ = fl3i_slots::SlotISAM2(parameters);').replace('gtsam::ISAM2 isam_;','fl3i_slots::SlotISAM2 isam_;')
cpp=names((BASE/'RecordedFixedLagSmoother.cpp').read_text())
assert header.rstrip()==(HERE/'RecordedFixedLagSmoother.h').read_text().rstrip()
assert cpp.rstrip()==(HERE/'RecordedFixedLagSmoother.cpp').read_text().rstrip()
from pathlib import Path
pinned=Path('/home/yeranis/workspaces/gnssfgo_ct_runtime/rwth_gtsam_src/gtsam/nonlinear/ISAM2.cpp').read_text()
start=pinned.index('void ISAM2::marginalizeLeaves(')
end=pinned.index('  // Gather factors to add - the new marginal factors',start)
candidate=(HERE/'SlotISAM2.cpp').read_text()
assert pinned[start:end].replace('void ISAM2::','void SlotISAM2::')==candidate[candidate.index('void SlotISAM2::marginalizeLeaves('):candidate.index('  // Slot lifecycle only:')]
for name in ('Receiver.h','NativeSmoother.h','Replay.cpp','RetirementCapture.h'):
    assert names((BASE/name).read_text()).rstrip()==(HERE/name).read_text().rstrip(),name
assert protected()==json.loads((ROOT/'protected.json').read_text())
log=(ROOT/'test.log').read_text();metrics=dict((k,float(v)) for k,v in re.findall(r'^METRIC (\S+) (\S+)$',log,re.M))
metrics.pop('max_gradient') # unused compatibility counter, NOT a convergence result.
summary=dict(passed=True,checks=int(re.search(r'CHECKS (\d+)',log)[1]),metrics=metrics,
             release_repeat_byte_identical=True,sanitizer_archive_byte_identical=sanitizer_identical,
             authored_solver_copy_verified=True,pinned_elimination_prefix_unchanged=True,
             receiver_model_and_policy_unchanged=True,protected_files=len(protected()),
             stationarity='not required or asserted for native incremental policy')
with (ROOT/'synthetic_validation.json').open('x') as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
