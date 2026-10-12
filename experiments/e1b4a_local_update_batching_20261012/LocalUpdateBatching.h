#pragma once
#include "ScopedGraphLinearizer.h"
#include <glim_ros/cbs_local_graph_linearizer.hpp>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <array>
#include <cstdlib>
#include <cstring>
#include <unordered_set>

namespace glim { namespace e1b4a {
enum class Reason { CpuOnly, UnknownCpu, UnknownGpu, Duplicate, Pending, Count };
struct Stats {
  size_t requests = 0, batched_graphs = 0, prepared_factors = 0;
  std::array<size_t, static_cast<size_t>(Reason::Count)> fallback{};
};
inline bool enabled() {
  const char* value = std::getenv("CBS_E1B_LOCAL_UPDATE_GPU_BATCHING");
  return value && std::strcmp(value, "1") == 0;
}
inline bool pureCpu(const gtsam::NonlinearFactor& factor) {
  const auto& t = typeid(factor);
  using namespace gtsam;
  return t == typeid(PriorFactor<Pose3>) || t == typeid(BetweenFactor<Pose3>) ||
      t == typeid(PriorFactor<Vector3>) || t == typeid(BetweenFactor<Vector3>) ||
      t == typeid(PriorFactor<imuBias::ConstantBias>) ||
      t == typeid(BetweenFactor<imuBias::ConstantBias>) ||
      t == typeid(ImuFactor) || t == typeid(LinearContainerFactor) ||
      t == typeid(gtsam_points::LinearDampingFactor);
}
// nullptr requests the original implementation. GPU helper fallbacks recurse
// through GraphCall, whose depth guard always suppresses redispatch.
inline gtsam::GaussianFactorGraph::shared_ptr linearize(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, void* ptr) {
  auto* stats = static_cast<Stats*>(ptr);
  if (stats) ++stats->requests;
  auto fallback = [&](Reason reason) -> gtsam::GaussianFactorGraph::shared_ptr {
    if (stats) ++stats->fallback[static_cast<size_t>(reason)];
    return {};
  };
#if defined(GTSAM_POINTS_USE_CUDA) && !defined(GTSAM_USE_TBB)
  size_t count = 0;
  std::unordered_set<const gtsam::NonlinearFactor*> seen;
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (!seen.insert(factor.get()).second) return fallback(Reason::Duplicate);
    auto* gpu = dynamic_cast<gtsam_points::NonlinearFactorGPU*>(factor.get());
    if (!gpu) {
      if (!pureCpu(*factor)) return fallback(Reason::UnknownCpu);
      continue;
    }
    if (typeid(*gpu) != typeid(gtsam_points::IntegratedVGICPFactorGPU))
      return fallback(Reason::UnknownGpu);
    if (static_cast<gtsam_points::IntegratedVGICPFactorGPU*>(gpu)->has_cached_linearization())
      return fallback(Reason::Pending);
    ++count;
  }
  if (!count) return fallback(Reason::CpuOnly);
  // The existing E1 helper checks exact Pose3 keys before preparing any GPU
  // result; drains prepared factors on exceptions; keeps native error caches.
  auto result = linearizeCbsLocalGraphBatched(graph, values);
  if (stats) { ++stats->batched_graphs; stats->prepared_factors += count; }
  return result;
#else
  return fallback(Reason::CpuOnly);
#endif
}
}}  // namespace glim::e1b4a
