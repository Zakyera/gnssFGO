#!/usr/bin/env python3
"""Mechanical, checked overlays. Never edits a source repo or build artifact."""
import difflib
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OUT=WS/'runs/e1b_local_profile_20261012'

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''): h.update(b)
    return h.hexdigest()

def main():
    pinned=json.loads((HERE.parent/'e1b_local_update_audit_20261012/evidence.json').read_text())['source_git']
    for path, info in pinned.items():
        assert subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()==info['head']
        assert not subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True).strip()
    OUT.mkdir(exist_ok=False)
    overlay=OUT/'overlay'; overlay.mkdir()
    protected={}
    def rewrite(src,dest,edits):
        original=(WS/'src'/src).read_text(); text=original
        protected[str(WS/'src'/src)]=sha(WS/'src'/src)
        for old,new,count in edits:
            if text.count(old)!=count:
                raise ValueError(f'{src}: expected {count} occurrences: {old[:100]!r}, got {text.count(old)}')
            text=text.replace(old,new)
        text='#include "RuntimeProfile.h"\n'+text
        path=overlay/dest; path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(text)
        (overlay/(Path(dest).name+'.diff')).write_text(''.join(difflib.unified_diff(
            original.splitlines(True),text.splitlines(True),fromfile=src,tofile=str(path))))
    rewrite('cbs/src/bpsam/bpsam.cpp','bpsam.cpp',[
        ('  NonlinearFactorGraph newFactorsCopy = newFactors;',
         '  e1b::Scope profile_bpsam(e1b::Stage::Bpsam);\n  e1b::Scope profile_prepare(e1b::Stage::Prepare);\n  NonlinearFactorGraph newFactorsCopy = newFactors;',1),
        ('    active_window_update_backup = std::make_unique<BPSAM>(*this);',
         '    active_window_update_backup = e1b::measure(e1b::Stage::Backup, [&] { return std::make_unique<BPSAM>(*this); });',1),
        ('  // optimize and retry after removing dangling robot keys',
         '  profile_prepare.stop();\n  // optimize and retry after removing dangling robot keys',1),
        ('      result = BaseSAM::update(\n          current_new_factors, current_new_theta, current_update_params);',
         '      result = e1b::measure(e1b::Stage::BaseUpdate, [&] { return BaseSAM::update(\n          current_new_factors, current_new_theta, current_update_params); });',1),
        ('    const Values post_update_values = this->calculateEstimate();',
         '    e1b::Scope profile_post(e1b::Stage::PostUpdate);\n    const Values post_update_values = e1b::measure(e1b::Stage::Estimate, [&] { return this->calculateEstimate(); });',1),
    ])
    rewrite('cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',
            'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',[
        ('    const bool debug = ISDEBUG("IncrementalFixedLagBpsamSmoother update");',
         '    e1b::Scope profile_bookkeeping(e1b::Stage::Prepare);\n    const bool debug = ISDEBUG("IncrementalFixedLagBpsamSmoother update");',1),
        ('    if (shadow_acceptance_force_full_solve_) {',
         '    profile_bookkeeping.stop();\n    if (shadow_acceptance_force_full_solve_) {',1),
        ('    if (!marginalizableKeys.empty()) {\n      if (capture_bad_update_stages_once_',
         '    if (!marginalizableKeys.empty()) {\n      e1b::Scope profile_marginal(e1b::Stage::Marginalize);\n      if (capture_bad_update_stages_once_',1),
        ('    eraseKeyTimestampMap(marginalizableKeys);',
         '    e1b::Scope profile_cleanup(e1b::Stage::Cleanup);\n    eraseKeyTimestampMap(marginalizableKeys);',1),
        ('    return result;\n  }\n\n  gtsam::FixedLagSmoother::Result updateWithShadowAcceptanceAudit',
         '''    profile_cleanup.stop();
    if (e1b::active()) {
      e1b::Scope metadata(e1b::Stage::Metadata);
      e1b::metric(e1b::Metric::ActiveKeys,isam_.getLinearizationPoint().size());
      e1b::metric(e1b::Metric::FactorSlots,isam_.getFactorsUnsafe().size());
      size_t nonnull=0; for(const auto& factor:isam_.getFactorsUnsafe()) nonnull+=bool(factor);
      e1b::metric(e1b::Metric::NonNullFactors,nonnull);
      e1b::metric(e1b::Metric::RetiringKeys,marginalizableKeys.size());
      e1b::metric(e1b::Metric::RelinKeys,isamResult_.variablesRelinearized);
      e1b::metric(e1b::Metric::ReeliminatedKeys,isamResult_.variablesReeliminated);
      e1b::metric(e1b::Metric::Cliques,isamResult_.cliques);
    }
    return result;
  }

  gtsam::FixedLagSmoother::Result updateWithShadowAcceptanceAudit''',1),
    ])
    rewrite('glim_ros1/src/glim_ros/cbs_bridge.cpp','cbs_bridge.cpp',[
        ('    sidecar_->update(new_factors, new_values, new_stamps);',
         '''    e1b::Session profile(new_stamps.empty()?0.0:new_stamps.rbegin()->second,e1b::Context::Local);
    e1b::Scope local_total(e1b::Stage::LocalTotal);
    sidecar_->update(new_factors, new_values, new_stamps);''',1),
        ('  std::vector<cbs::BPSAM::CbsOdometryBelief> outgoing;\n  double set_marginalization_graph_ms',
         '  e1b::Session profile(stamp.toSec(),e1b::Context::Covariance);\n  std::vector<cbs::BPSAM::CbsOdometryBelief> outgoing;\n  double set_marginalization_graph_ms',1),
        ('  if (export_endpoint_audit_enable_ || live_revision_enable_) {\n    const size_t update_index',
         '  if (export_endpoint_audit_enable_ || live_revision_enable_) {\n    e1b::Session profile(stamp.toSec(),e1b::Context::Revision);\n    const size_t update_index',1),
    ])
    rewrite('glim/src/glim/odometry/odometry_estimation_imu.cpp','odometry_estimation_imu.cpp',[
        ('    smoother->update(new_factors, new_values, new_stamp, factors_to_remove);',
         '    e1b::Session profile(new_stamp.empty()?0.0:new_stamp.rbegin()->second,e1b::Context::Main);\n    smoother->update(new_factors, new_values, new_stamp, factors_to_remove);',1),
    ])
    rewrite('gtsam_points/src/gtsam_points/factors/integrated_vgicp_factor_gpu.cpp','integrated_vgicp_factor_gpu.cpp',[
        ('double IntegratedVGICPFactorGPU::error(const gtsam::Values& values) const {',
         'double IntegratedVGICPFactorGPU::error(const gtsam::Values& values) const {\n  e1b::Scope profile(e1b::Stage::GpuError);',1),
        ('  if (evaluation_result) {','  if (evaluation_result) {\n    e1b::metric(e1b::Metric::GpuErrorHits,1);',1),
        ('    std::cerr << "warning: computing error in sync mode seriously affects the processing speed!!" << std::endl;',
         '    e1b::Scope sync(e1b::Stage::GpuSyncError);\n    { e1b::Scope warning(e1b::Stage::GpuWarning);\n    std::cerr << "warning: computing error in sync mode seriously affects the processing speed!!" << std::endl; }',1),
        ('gtsam::GaussianFactor::shared_ptr IntegratedVGICPFactorGPU::linearize(const gtsam::Values& values) const {',
         'gtsam::GaussianFactor::shared_ptr IntegratedVGICPFactorGPU::linearize(const gtsam::Values& values) const {\n  e1b::Scope profile(e1b::Stage::GpuLinearize);',1),
        ('  if (linearization_result) {','  if (linearization_result) {\n    e1b::metric(e1b::Metric::GpuLinearizeHits,1);',1),
        ('    std::cerr << "warning: performing linearization in sync mode seriously affects the processing speed!!" << std::endl;',
         '    e1b::Scope sync(e1b::Stage::GpuSyncLinearize);\n    { e1b::Scope warning(e1b::Stage::GpuWarning);\n    std::cerr << "warning: performing linearization in sync mode seriously affects the processing speed!!" << std::endl; }',1),
    ])
    rewrite('gtsam/gtsam/nonlinear/NonlinearFactorGraph.cpp','NonlinearFactorGraph.cpp',[
        ('  GaussianFactorGraph& result_;','  GaussianFactorGraph& result_;\n  e1b::Token profile_token_=e1b::capture();',1),
        ('  void operator()(const tbb::blocked_range<size_t>& blocked_range) const {',
         '  void operator()(const tbb::blocked_range<size_t>& blocked_range) const {\n    e1b::Attach profile_attach(profile_token_);',1),
        ('  gttic(NonlinearFactorGraph_linearize);','  gttic(NonlinearFactorGraph_linearize);\n  e1b::Scope profile(e1b::Stage::FactorGraphLinearize);',1),
    ])
    edits=[]
    for site,stage in [('relinearizeAffectedFactors','IsamAffected'),('recalculate','IsamRecalculate'),
                       ('recalculateBatch','IsamBatch'),('recalculateIncremental','IsamIncremental'),
                       ('ISAM2_update','IsamUpdate'),('updateDelta','IsamDelta')]:
        edits.append((f'  gttic({site});',f'  gttic({site});\n  e1b::Scope profile(e1b::Stage::{stage});',1))
    edits.append(('  // Convert to ordered set\n  KeySet leafKeys',
                  '  e1b::Scope profile(e1b::Stage::IsamMarginalize);\n  // Convert to ordered set\n  KeySet leafKeys',1))
    rewrite('gtsam/gtsam/nonlinear/ISAM2.cpp','ISAM2.cpp',edits)
    for name in ('RuntimeProfile.h','RuntimeProfile.cpp','test_profile.cpp','test_solver.cpp','test_gpu.cpp','build.py','test.py','bench.cpp','validate_extra.py','check_overhead.py','check_protected.py'):
        shutil.copy2(HERE/name,OUT/name)
    (OUT/'source_manifest.json').write_text(json.dumps(protected,indent=2)+'\n')
    (OUT/'source_git.json').write_text(json.dumps(pinned,indent=2)+'\n')
    print(OUT)

if __name__=='__main__': main()
