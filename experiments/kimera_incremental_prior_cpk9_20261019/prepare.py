"""Exclusive isolated overlay. No source/installed edits or launch generation."""
import hashlib
import json
from pathlib import Path
import shutil

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=W/'runs/kimera_failed_factor_cpk1_20261019'
R=W/'runs/kimera_incremental_prior_cpk9_20261019'
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()
def patch(p,old,new):
    s=p.read_text()
    assert s.count(old)==1,(p,old)
    p.write_text(s.replace(old,new))
def main():
    protected=json.loads((W/'runs/kimera_prior_refresh_cpk6_20261019/protected_before.json').read_text())
    for p in sorted(BASE.rglob('*')):
        if p.is_file(): protected[str(p)]=sha(p)
    bad=[p for p,h in protected.items() if not Path(p).is_file() or sha(p)!=h]
    assert not bad,bad
    R.mkdir(exist_ok=False)
    shutil.copytree(BASE/'include',R/'include')
    (R/'private_build').mkdir()
    shutil.copy2(BASE/'private_build/libcbs.so',R/'private_build/libcbs.so')
    for name in ('CurrentChartFallback.h','OriginalChartPrior.h','RetirementAudit.h','LinearParityAudit.h','VioBackend.cpp'):
        shutil.copy2(BASE/name,R/name)
    for name in ('NativePriorRefresh.h','test.cpp','build.py','PROTOCOL.txt'):
        shutil.copy2(H/name,R/name)
    bp=R/'include/cbs/bpsam/bpsam.h'
    patch(bp,'#pragma once','#pragma once\n#include "NativePriorRefresh.h"')
    patch(bp,'  const GaussianFactorGraph& fl4nLinearCache() const { return linearFactors_; }', '''  void cpk9ValidateMode() const {
    if (!std::holds_alternative<ISAM2GaussNewtonParams>(BaseSAM::params_.optimizationParams) ||
        !BaseSAM::params_.cacheLinearizedFactors || params_.use_temporary_cbs_linear_factors ||
        params_.cbs_odom_factor_mode!=CbsOdomFactorMode::ActiveWindowTemporary)
      throw std::runtime_error("CPK9 incompatible native core mode");
  }
  cpk9::Promotion cpk9PromotePriorCharts() {
    cpk9ValidateMode();
    const auto result=cpk9::promote(nonlinearFactors_,theta_,linearFactors_,fixedVariables_);
    if(result.priors) tmp_marginals_.reset();
    return result;
  }
  const GaussianFactorGraph& fl4nLinearCache() const { return linearFactors_; }''')
    p=R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'
    patch(p,'    if (fl4q::enabled()) {\n      return fl4q::transaction', '''    const bool cpk9_enabled=cpk9::enabled();
    if(cpk9_enabled) isam_.cpk9ValidateMode();
    // OFF has no added transaction/copy cost. ON rollback includes smart caches.
    if (fl4q::enabled() || (cpk9_enabled && isam_.activeWindowTemporaryCbsOdomFactorCount()!=0)) {
      return fl4q::transaction''')
    patch(p,'    std::vector<gtsam::NonlinearFactor::shared_ptr> fl4n_retired;', '''    if (cpk9::enabled() && !active_window_cbs_removals.empty()) {
      for(const auto& removal:active_window_cbs_removals)
        if(!removal.factor_slot_valid ||
           std::find(factors_to_remove.begin(),factors_to_remove.end(),removal.factor_index)==factors_to_remove.end())
          throw std::runtime_error("CPK9 retirement not scheduled for removal");
      const auto promoted=isam_.cpk9PromotePriorCharts();
      spdlog::info("CPK9_PRIOR_REFRESH,{},{},{},{},{}",current_timestamp,
          active_window_cbs_removals.size(),promoted.priors,promoted.released,promoted.cache_error);
    }

    std::vector<gtsam::NonlinearFactor::shared_ptr> fl4n_retired;''')
    old='/workspace/cbs_gtsam4.3/runs/'+BASE.name
    new='/workspace/cbs_gtsam4.3/runs/'+R.name
    for name in ('actual_test_commands.json','build_commands.json'):
        s=(BASE/name).read_text().replace(old,new)
        if name=='actual_test_commands.json':
            s=s.replace('/workspace/cbs_gtsam4.3/devel/lib/libcbs.so',new+'/private_build/libcbs.so')
        (R/name).write_text(s)
    (R/'protected_before.json').write_text(json.dumps(protected,indent=2)+'\n')
    (R/'inputs.json').write_text(json.dumps({str(p):sha(p) for p in sorted(R.rglob('*')) if p.is_file()},indent=2)+'\n')
    print(json.dumps({'root':str(R),'protected':len(protected),'launch_generated':False}))
if __name__=='__main__': main()
