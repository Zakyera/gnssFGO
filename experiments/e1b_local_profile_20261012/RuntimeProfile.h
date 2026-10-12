#pragma once
// Private instrumentation ABI; no GTSAM/ROS/CUDA dependency or solver calls.
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <utility>

namespace e1b {
enum class Context { Other, Main, Local, Covariance, Revision };
enum class Stage {
  Root, WorkerRoot, LocalTotal, Bpsam, Prepare, Backup, BaseUpdate,
  PostUpdate, Estimate, Marginalize, Cleanup, Metadata,
  GpuLinearize, GpuError, GpuSyncLinearize, GpuSyncError, GpuWarning,
  FactorGraphLinearize, IsamUpdate, IsamDelta, IsamRecalculate,
  IsamBatch, IsamIncremental, IsamAffected, IsamMarginalize, Count
};
enum class Metric {
  ActiveKeys, FactorSlots, NonNullFactors, RetiringKeys, RelinKeys,
  ReeliminatedKeys, Cliques, GpuLinearizeHits, GpuErrorHits, Count
};
struct Cell { uint64_t calls=0, inclusive_ns=0, exclusive_ns=0, exceptions=0; };
struct Record {
  double timestamp=0;
  Context context=Context::Other;
  std::array<Cell, static_cast<unsigned>(Stage::Count)> owner{}, worker{};
  std::array<uint64_t, static_cast<unsigned>(Metric::Count)> metrics{};
  bool aborted=false;
};
class Scope;
struct Collector;
struct ThreadState { Record* record; Collector* collector; Scope* parent; bool worker; };
// One definition in libe1b_profile.so: shared across all instrumented DSOs.
extern thread_local ThreadState* current;
inline uint64_t tick() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool active() noexcept { return current != nullptr; }
inline void metric(Metric key, uint64_t value) noexcept {
  if (current) current->record->metrics[static_cast<unsigned>(key)] += value;
}
class Scope {
 public:
  explicit Scope(Stage stage) noexcept : state_(current), stage_(stage) {
    if (state_) {
      parent_=state_->parent; state_->parent=this;
      exceptions_=std::uncaught_exceptions(); start_=tick();
    }
  }
  ~Scope() noexcept { stop(); }
  void stop() noexcept {
    if (!state_) return;
    const uint64_t elapsed=tick()-start_;
    auto& cells=state_->worker ? state_->record->worker : state_->record->owner;
    auto& cell=cells[static_cast<unsigned>(stage_)];
    ++cell.calls; cell.inclusive_ns+=elapsed;
    cell.exclusive_ns+=elapsed-children_;
    cell.exceptions+=std::uncaught_exceptions()>exceptions_;
    if (parent_) parent_->children_+=elapsed;
    state_->parent=parent_; state_=nullptr;
  }
  Scope(const Scope&)=delete;
  Scope& operator=(const Scope&)=delete;
 private:
  ThreadState* state_;
  Stage stage_;
  Scope* parent_=nullptr;
  uint64_t start_=0, children_=0;
  int exceptions_=0;
};
template<class F> decltype(auto) measure(Stage stage, F&& function) {
  Scope scope(stage);
  return std::forward<F>(function)();
}
// capture/attach propagates measurement context to GTSAM's existing TBB ranges.
// Worker durations are summed separately, NEVER added to owner wall time.
struct Token { Collector* collector=nullptr; };
inline Token capture() noexcept { return {current ? current->collector : nullptr}; }
class Attach {
 public:
  explicit Attach(Token token);
  ~Attach() noexcept;
  Attach(const Attach&)=delete;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
class Session {
 public:
  // A non-null result enables an in-memory test record. Otherwise the absent
  // CBS_E1B_PROFILE_PATH environment variable leaves profiling disabled.
  Session(double timestamp, Context context, Record* result=nullptr);
  ~Session() noexcept;
  Session(const Session&)=delete;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
const char* stageName(Stage stage) noexcept;
}  // namespace e1b
