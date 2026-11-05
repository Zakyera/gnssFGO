#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include "LocalUpdateBatching.h"

namespace cpk16 {
enum class Site { NewFactors, FullGraph, Affected, Marginalize, Count };
struct Stats {
  glim::e1b4a::Stats work;
  std::array<size_t, static_cast<size_t>(Site::Count)> sites{};
};
// No modification to an existing class layout, optimizer or global hook list.
class Scope {
 public:
  Scope(const gtsam_points::ISAM2Ext* owner, bool enable, Stats* stats = nullptr);
  ~Scope();
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
 private:
  bool installed_ = false;
  const gtsam_points::ISAM2Ext* owner_;
  Stats own_stats_;
  Stats* stats_;
  friend bool active(const gtsam_points::ISAM2Ext*);
  friend gtsam::GaussianFactorGraph::shared_ptr tryLinearize(
      const gtsam_points::ISAM2Ext*, const gtsam::NonlinearFactorGraph&,
      const gtsam::Values&, Site);
};
bool requested();
bool active(const gtsam_points::ISAM2Ext*);
// nullptr means no preparation occurred: execute the original route unchanged.
gtsam::GaussianFactorGraph::shared_ptr tryLinearize(
    const gtsam_points::ISAM2Ext*, const gtsam::NonlinearFactorGraph&,
    const gtsam::Values&, Site);
}  // namespace cpk16
