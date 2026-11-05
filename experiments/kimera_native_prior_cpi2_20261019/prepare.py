"""Create a test-only wrapper snapshot overlay; never edit source/run binaries."""
from pathlib import Path
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/fixed_lag_current_chart_fl4q_20261016'
OUT=WS/'runs/kimera_native_prior_cpi2_20261019'
assert not OUT.exists(), 'refuse overwrite'
OUT.mkdir()
p=Path('include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h')
source=(BASE/p).read_text()
changed=source.replace('    gtsam::Values values_after_optimization;', '''    gtsam::Values values_after_optimization;
    gtsam::Values cpi2_theta;
    gtsam::GaussianFactorGraph cpi2_cached;
    gtsam::FactorIndices cpi2_peer_slots;''')
needle='      audit.values_after_optimization = isam_.calculateEstimate();'
assert source.count(needle)==1
changed=changed.replace(needle,needle+'''
      audit.cpi2_theta = isam_.getLinearizationPoint();
      audit.cpi2_cached = isam_.fl4nLinearCache();
      for(size_t i=0;i<isam_.getFactorsUnsafe().size();++i) {
        const auto& f=isam_.getFactorsUnsafe()[i];
        if(f && isam_.isTrackedCbsFactorSlot(i,f)) audit.cpi2_peer_slots.push_back(i);
      }''')
(OUT/p).parent.mkdir(parents=True)
(OUT/p).write_text(changed)
for name in ['test.cpp','build.py']:
    shutil.copy2(HERE/name,OUT/name)
manifest={str(BASE/p):hashlib.sha256((BASE/p).read_bytes()).hexdigest()}
(OUT/'original_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(OUT)
