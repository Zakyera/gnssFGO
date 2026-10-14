import json,re
from driver import ROOT,HERE,REPO,sha,protected
def files(root):return {str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file()}
for name in ('test','test_repeat','sanitest'):
    result=json.loads((ROOT/(name+'.json')).read_text());assert result['returncode']==0 and result['protected_unchanged']
    log=(ROOT/(name+'.log')).read_text();assert 'FL3H_LIFECYCLE_GATE PASS' in log
    assert not re.search(r'ERROR: AddressSanitizer|runtime error:|FAIL ',log)
assert files(ROOT/'test_outputs')==files(ROOT/'test_repeat_outputs')
assert (ROOT/'test.log').read_bytes()==(ROOT/'test_repeat.log').read_bytes()
sanitizer_identical=files(ROOT/'test_outputs')==files(ROOT/'sanitest_outputs')
header=(REPO/'online_fgo/include/solver/IncrementalFixedLagSmoother.h').read_text().replace('IncrementalFixedLagSmoother','RecordedFixedLagSmoother')
cpp=(REPO/'online_fgo/src/solver/IncrementalFixedLagSmoother.cpp').read_text().replace('IncrementalFixedLagSmoother','RecordedFixedLagSmoother').replace('recursiveMarkAffectedKeys','fl3hMarkAffectedKeys')
cpp=cpp.replace('#include "solver/RecordedFixedLagSmoother.h"','#include "RecordedFixedLagSmoother.h"\n#include "RetirementCapture.h"')
cpp=cpp.replace('            isam_.marginalizeLeaves(leafKeys);','            if (fl3h_capture::before) fl3h_capture::before(isam_);\n            isam_.marginalizeLeaves(leafKeys);')
assert header.rstrip()==(HERE/'RecordedFixedLagSmoother.h').read_text().rstrip()
assert cpp.rstrip()==(HERE/'RecordedFixedLagSmoother.cpp').read_text().rstrip()
assert protected()==json.loads((ROOT/'protected.json').read_text())
log=(ROOT/'test.log').read_text();metrics=dict((k,float(v)) for k,v in re.findall(r'^METRIC (\S+) (\S+)$',log,re.M))
metrics.pop('max_gradient') # unused compatibility counter, NOT a convergence result.
summary=dict(passed=True,checks=int(re.search(r'CHECKS (\d+)',log)[1]),metrics=metrics,
             release_repeat_byte_identical=True,sanitizer_archive_byte_identical=sanitizer_identical,
             authored_solver_copy_verified=True,protected_files=len(protected()),
             stationarity='not required or asserted for native incremental policy')
with (ROOT/'synthetic_validation.json').open('x') as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
