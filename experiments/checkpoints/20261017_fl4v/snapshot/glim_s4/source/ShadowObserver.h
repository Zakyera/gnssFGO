#pragma once
#include "NativeGaussianHistory.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <functional>
#include <memory>

namespace shared_native {
// Private prototype interface; no GTSAM/GLIM class layout or vtable changes.
class ShadowObserver {
 public:
  ShadowObserver();
  ~ShadowObserver();
  void admit(const gtsam::NonlinearFactorGraph&) noexcept;
  void begin(const gtsam::NonlinearFactorGraph&, const gtsam::FactorIndices&) noexcept;
  void beforeRetire(const gtsam_points::ISAM2Ext&, const gtsam::ISAM2Result&,
                    const gtsam::KeyVector&) noexcept;
  void afterRetire(const gtsam_points::ISAM2Ext&) noexcept;
  void commit() noexcept;
  void fail(const char*) noexcept;
  void reset() noexcept;
  void setRecording(const char*) noexcept;
  bool logDue() const;
  void recordQueryInput(gtsam::Key,gtsam::Key,const gtsam::Pose3&,const gtsam::Pose3&);
  void recordQueryOutput(const EndpointCovariance&,double);
  bool valid() const;
  const char* error() const;
  const NativeGaussianHistory& history() const;
  const Snapshot& snapshot() const;
  std::size_t captures() const;
  std::size_t resets() const;
  // Diagnostic-only same-chart oracle hook. Exceptions invalidate shadow only.
  void setAudit(std::function<void(const Snapshot&)>);
  Result query(gtsam::Key, gtsam::Key, const gtsam::Pose3&, const gtsam::Pose3&,
               EndpointCovariance&) noexcept;
 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};

// Registry holds only explicitly attached outer smoother identities. attach is
// allowed before startup, never on a partially marginalized live graph.
std::shared_ptr<ShadowObserver> attach(const void* owner);
std::shared_ptr<ShadowObserver> lookup(const void* owner) noexcept;
void detach(const void* owner) noexcept;
void reset(const void* owner) noexcept;
void admitFromEnvironment(const void* owner, bool main_empty,
                          const gtsam::NonlinearFactorGraph&) noexcept;
void queryForLog(const void* owner, gtsam::Key, gtsam::Key,
                 const gtsam::Pose3&, const gtsam::Pose3&) noexcept;

class UpdateScope {
 public:
  UpdateScope(const void* owner, const gtsam::NonlinearFactorGraph&,
              const gtsam::FactorIndices&) noexcept;
  ~UpdateScope() noexcept;
  void commit() noexcept;
 private:
  ShadowObserver* previous_;
  std::shared_ptr<ShadowObserver> observer_;
  bool completed_ = false;
};
void beforeRetire(const gtsam_points::ISAM2Ext&, const gtsam::ISAM2Result&,
                  const gtsam::KeyVector&) noexcept;
void afterRetire(const gtsam_points::ISAM2Ext&) noexcept;
}  // namespace shared_native
