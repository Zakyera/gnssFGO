#pragma once

#include <gtsam/nonlinear/ISAM2.h>
#include <memory>
#include <stdexcept>

namespace fgo::offline {

inline void RequireSameFactorSlots(const gtsam::NonlinearFactorGraph& a,
                                   const gtsam::NonlinearFactorGraph& b) {
  if (a.size()!=b.size()) throw std::runtime_error("rebuild changed factor slot count");
  for(size_t i=0;i<a.size();++i)
    if(a[i]!=b[i]) throw std::runtime_error("rebuild changed factor identity or slot");
}

// Full-history numerical recovery only. This does not add factors or choose a
// new nonlinear optimizer. Reconstruct GTSAM's own QR tree at a fresh seed.
// Empty slots MUST survive: external stable-ID revision bookkeeping stores them.
class SlotPreservingISAM2 final : public gtsam::ISAM2 {
 public:
  SlotPreservingISAM2(const gtsam::ISAM2Params& parameters,
                     const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& seed) : gtsam::ISAM2(parameters) {
    // FactorGraph::add_factors(...,true) compresses null entries in a new graph.
    // Disable hole reuse for this one bulk load, then restore the exact policy.
    params_.findUnusedFactorSlots=false;
    const auto result=update(graph,seed);
    params_.findUnusedFactorSlots=parameters.findUnusedFactorSlots;
    RequireSameFactorSlots(graph,getFactorsUnsafe());
    if(result.newFactorsIndices.size()!=graph.size())
      throw std::runtime_error("rebuild returned incomplete factor index map");
    for(size_t i=0;i<graph.size();++i)
      if(result.newFactorsIndices[i]!=i)throw std::runtime_error("rebuild remapped a factor slot");
    if(getLinearizationPoint().keys()!=seed.keys())
      throw std::runtime_error("rebuild changed state keys");
  }
};

inline std::unique_ptr<gtsam::ISAM2> RebuildFullHistoryQR(
    const gtsam::ISAM2& original,const gtsam::Values& seed) {
  const auto& params=original.params();
  if(!original.getFixedVariables().empty())
    throw std::runtime_error("rebuild does not support marginalized fixed variables");
  // Only the frozen full-history Gauss-Newton/QR profile is supported. Resetting
  // a Dogleg trust radius or a >1 periodic relinearization counter is not neutral.
  if(params.factorization!=gtsam::ISAM2Params::QR || params.relinearizeSkip!=1 ||
      !boost::get<gtsam::ISAM2GaussNewtonParams>(&params.optimizationParams))
    throw std::runtime_error("unsupported numerical rebuild profile");
  return std::make_unique<SlotPreservingISAM2>(params,original.getFactorsUnsafe(),seed);
}
}  // namespace fgo::offline
