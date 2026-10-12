#pragma once
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Experimental dispatch only. No fields/vtables in existing GTSAM types change.
// All TLS storage lives in libgtsam, not in each including shared library.
namespace gtsam { namespace e1b4a {
using Callback = GaussianFactorGraph::shared_ptr (*)(
    const NonlinearFactorGraph&, const Values&, void*);
class GTSAM_EXPORT ScopedLinearizer {
 public:
  explicit ScopedLinearizer(Callback callback, void* context = nullptr);
  ~ScopedLinearizer();
  ScopedLinearizer(const ScopedLinearizer&) = delete;
  ScopedLinearizer& operator=(const ScopedLinearizer&) = delete;
 private:
  bool installed_;
};
// Keep this guard alive through the ordinary loop as well as the callback.
// A CPU factor can recursively linearize another graph during fallback.
class GTSAM_EXPORT GraphCall {
 public:
  GraphCall();
  ~GraphCall();
  GraphCall(const GraphCall&) = delete;
  GraphCall& operator=(const GraphCall&) = delete;
  GaussianFactorGraph::shared_ptr dispatch(const NonlinearFactorGraph& graph,
                                          const Values& values) const;
 private:
  bool outermost_;
};
}}  // namespace gtsam::e1b4a
