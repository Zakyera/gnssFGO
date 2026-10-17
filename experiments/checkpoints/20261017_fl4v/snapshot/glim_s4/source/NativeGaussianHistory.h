#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace shared_native {
using Id = std::uint64_t;
enum class Mode { Disabled, Shadow };
struct NativeRef {
  Id id, version;
  std::size_t slot;
};
struct Snapshot {
  Id generation = 1, epoch = 1;
  gtsam::GaussianFactorGraph cache;
  // Positive admission provenance, NOT a class/type-based exclusion filter.
  // Integration must populate this from native admission before CBS is appended.
  std::vector<NativeRef> native;
  std::set<Id> removed;
  gtsam::Values theta;
  gtsam::KeySet retiring, fixed_after;
};
struct Result {
  bool ok;
  std::string message;
};
struct EndpointCovariance {
  gtsam::Matrix joint_theta, joint_current, transport;
  gtsam::Matrix6 relative;
};

// S2-only, single-threaded transactional observer. No nonlinear factors, MAIN
// references, optimizer, publication interface, or GPU calls are owned here.
class NativeGaussianHistory {
 public:
  explicit NativeGaussianHistory(Mode mode = Mode::Disabled);
  ~NativeGaussianHistory();
  NativeGaussianHistory(const NativeGaussianHistory&) = delete;
  NativeGaussianHistory& operator=(const NativeGaussianHistory&) = delete;
  Result stage(const Snapshot& snapshot);
  Result commit(Id generation, Id epoch);
  void abort();
  Result reset(Id new_generation);
  Id epoch() const;
  std::size_t activeCount() const;
  std::size_t absorbedCount() const;
  std::size_t removedCount() const;
  std::size_t admittedCount() const;
  gtsam::GaussianFactorGraph graph(Id expected_epoch) const;
  gtsam::Values theta(Id expected_epoch) const;
  EndpointCovariance covariance(Id expected_epoch, gtsam::Key from,
      gtsam::Key to, const gtsam::Vector6& delta_from,
      const gtsam::Vector6& delta_to) const;
  // Observer-safe query entry point: a candidate failure must never escape into
  // MAIN's fallback handler. On failure the caller's output is unchanged.
  Result tryCovariance(Id expected_epoch, gtsam::Key from, gtsam::Key to,
      const gtsam::Vector6& delta_from, const gtsam::Vector6& delta_to,
      EndpointCovariance& output) const;
 private:
  struct State;
  Mode mode_;
  std::unique_ptr<State> committed_, pending_;
};
}  // namespace shared_native
