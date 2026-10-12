#pragma once
// Diagnostic clocks only. No estimator inputs, factors or evaluations are added.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <thread>
#include <time.h>
#include <utility>

namespace s5b {
enum Tag {
  Run, Publication, SolverCopy, IsamUpdate, Estimate, GraphError,
  Gradient, GraphLinearize, GradientVector, HessianDiagonal, StableCost,
  Recovery, Polish, MotionLinearize, SourceLinearize, Signature, HitClone,
  MotionBase, SourceBase, MotionPoseDerivative, MotionTwistDerivative,
  MotionError, SourceError, NumericInterpolation, AnalyticInterpolation,
  PublicationIO, MemoIO, Revision, SupportUpdate, EventRead, ProfileIO, Count
};
inline constexpr const char* names[] = {
  "run", "publication", "solver_copy", "isam_update", "estimate", "graph_error",
  "gradient", "graph_linearize", "gradient_vector", "hessian_diagonal", "stable_cost",
  "recovery", "polish", "motion_linearize", "source_linearize", "memo_signature", "memo_hit_clone",
  "motion_base_linearize", "source_base_linearize", "motion_pose_derivative", "motion_twist_derivative",
  "motion_error", "source_error", "numeric_interpolation", "analytic_interpolation",
  "publication_io", "memo_io", "revision", "support_update", "event_read", "profiler_io"
};
static_assert(sizeof(names) / sizeof(*names) == Count);
enum Context { Other, Publishing, Replacing, Supporting, ContextCount };
inline constexpr const char* contexts[] = {"other", "publication", "revision", "support"};
struct Counter { uint64_t calls{}, inclusive{}, exclusive{}, cpu{}; };
inline std::array<std::array<Counter, Count>, ContextCount> counts{};
inline std::atomic<uint64_t> foreign_scopes{};
inline const std::thread::id owner = std::this_thread::get_id();
inline bool Enabled() {
  static const bool enabled = [] { const auto p = std::getenv("S5B_TIMING");
    return p && std::string(p) == "1"; }();
  return enabled;
}
inline bool Owner() { thread_local const bool same = std::this_thread::get_id() == owner; return same; }
inline uint64_t Wall() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t Cpu() {
  timespec t{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  return uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
inline Context context = Other;
class ContextScope {
  Context previous_;
 public:
  explicit ContextScope(Context c) : previous_(context) { if (Enabled() && Owner()) context = c; }
  ~ContextScope() { if (Enabled() && Owner()) context = previous_; }
};
class Scope;
inline Scope* top = nullptr;
class Scope {
  Tag tag_; Context context_{}; Scope* parent_{};
  uint64_t start_{}, children_{}, cpu_{};
  bool active_{false}, time_cpu_{false};
 public:
  explicit Scope(Tag tag) : tag_(tag) {
    if (!Enabled()) return;
    if (!Owner()) { ++foreign_scopes; return; }
    active_ = true; context_ = context; parent_ = top;
    time_cpu_ = tag == Run || tag == Publication || tag == Revision || tag == SupportUpdate;
    if (time_cpu_) cpu_ = Cpu();
    start_ = Wall(); top = this;
  }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
  ~Scope() {
    if (!active_) return;
    const auto elapsed = Wall() - start_;
    auto& c = counts[context_][tag_];
    ++c.calls; c.inclusive += elapsed; c.exclusive += elapsed - children_;
    if (time_cpu_) c.cpu += Cpu() - cpu_;
    top = parent_;
    if (parent_) parent_->children_ += elapsed;
  }
};
template<class F> decltype(auto) Timed(Tag tag, F&& function) {
  Scope timer(tag);
  return std::forward<F>(function)();
}
inline void Flush(size_t index, int64_t physical, const std::string& phase) {
  if (!Enabled() || !Owner()) return;
  const auto path = std::getenv("S5B_PROFILE_OUTPUT");
  if (!path) return;
  Scope timer(ProfileIO);
  static bool first = true;
  std::ofstream out(path, std::ios::app);
  if (first) { out << "phase,index,physical_ns,context,tag,calls,inclusive_ns,exclusive_ns,cpu_ns,foreign_scopes\n"; first = false; }
  for (size_t k = 0; k < ContextCount; ++k) for (size_t t = 0; t < Count; ++t) {
    const auto& c = counts[k][t];
    if (c.calls) out << phase << ',' << index << ',' << physical << ',' << contexts[k] << ',' << names[t]
      << ',' << c.calls << ',' << c.inclusive << ',' << c.exclusive << ',' << c.cpu << ',' << foreign_scopes << '\n';
  }
}
struct Finalizer { ~Finalizer() { Flush(0, 0, "process_end"); } };
}  // namespace s5b
