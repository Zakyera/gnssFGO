#include "MainGpuBatch.h"
#include <stdexcept>
#include <variant>

namespace cpk16 {
namespace {
thread_local Scope* current = nullptr;
thread_local bool evaluating = false;
}
bool requested() {
  const char* v = std::getenv("CPK16_MAIN_GPU_BATCHING");
  return v && std::strcmp(v, "1") == 0;
}
Scope::Scope(const gtsam_points::ISAM2Ext* owner, bool enable, Stats* stats)
    : owner_(owner), stats_(stats ? stats : &own_stats_) {
  if (!enable) return;
  if (!owner || current)
    throw std::logic_error("CPK16 requires a nonnested explicit MAIN owner");
  if (!std::holds_alternative<gtsam::ISAM2GaussNewtonParams>(
          owner->params().optimizationParams) ||
      owner->params().evaluateNonlinearError)
    throw std::invalid_argument("CPK16 only admits frozen GN/no-error-hook mode");
  current = this;
  installed_ = true;
}
Scope::~Scope() {
  if (installed_) current = nullptr;
}
bool active(const gtsam_points::ISAM2Ext* owner) {
  return current && current->owner_ == owner && !evaluating;
}
gtsam::GaussianFactorGraph::shared_ptr tryLinearize(
    const gtsam_points::ISAM2Ext* owner, const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values, Site site) {
  if (!active(owner)) return {};
  if (static_cast<size_t>(site) >= static_cast<size_t>(Site::Count))
    throw std::invalid_argument("CPK16 invalid linearization site");
  ++current->stats_->sites[static_cast<size_t>(site)];
  struct Guard {
    Guard() { evaluating = true; }
    ~Guard() { evaluating = false; }
  } guard;
  // Reuse the previously tested exact-type/pure-CPU/alias admission and
  // synchronous-compatible GPU helper, including cache cleanup on exceptions.
  return glim::e1b4a::linearize(graph, values, &current->stats_->work);
}
}  // namespace cpk16
