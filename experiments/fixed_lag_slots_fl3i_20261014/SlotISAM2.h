#pragma once
#include <gtsam/nonlinear/ISAM2.h>

// Private GTSAM subclass: stock elimination, marginal slot allocation corrected.
// No installed library or production ISAM2 ABI changes.
namespace fl3i_slots {
class SlotISAM2 : public gtsam::ISAM2 {
 public:
  using gtsam::ISAM2::ISAM2;
  void marginalizeLeaves(const gtsam::FastList<gtsam::Key>& keys,
      boost::optional<gtsam::FactorIndices&> added = boost::none,
      boost::optional<gtsam::FactorIndices&> removed = boost::none);
};
}
