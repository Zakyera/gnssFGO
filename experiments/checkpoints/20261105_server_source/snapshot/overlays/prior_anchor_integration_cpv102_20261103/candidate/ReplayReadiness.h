#pragma once
// CP73 private replay coordination. No ROS clock, payload or graph mutation.
#include "RecordedUpdateGate.h"
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <iterator>

namespace cpv73 {
template <class Belief> struct ReadyBatch {
  std::vector<Belief> arrived;
  double wall_ms = 0;
  size_t waits = 0;
};
class ReplayReadiness {
 public:
  using Clock = std::chrono::steady_clock;
  explicit ReplayReadiness(unsigned timeout_ms = 0) : timeout_ms_(timeout_ms) {}
  static unsigned environmentTimeout() {
    const char* s = std::getenv("CPV73_REPLAY_READY_TIMEOUT_MS");
    if (!s || !*s) return 0;
    const std::string value(s);
    if (value.find_first_not_of("0123456789") != std::string::npos ||
        value.size() > 4 || std::stoul(value) != 2000)
      throw std::runtime_error("CPV73_READINESS_FAILURE: expected explicit 2000ms");
    return 2000;
  }
  bool enabled() const { return timeout_ms_ != 0; }
  // Called after real enqueue. No data-provider/backend locks acquired here.
  void acknowledgeEnqueue() {
    if (!enabled()) return;
    { std::lock_guard<std::mutex> lock(mutex_); ++generation_; }
    cv_.notify_all();
  }
  template <class Pop>
  auto collect(const cpv70::RecordedUpdateGate& gate, cpv70::Id update,
               Pop pop, size_t cap) -> ReadyBatch<typename decltype(pop())::value_type> {
    using Belief = typename decltype(pop())::value_type;
    ReadyBatch<Belief> result;
    if (!enabled()) { result.arrived = pop(); return result; }
    if (!gate.enabled())
      throw std::runtime_error("CPV73_READINESS_FAILURE: schedule required");
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::milliseconds(timeout_ms_);
    while (true) {
      // Snapshot BEFORE pop closes notify-between-empty-pop-and-wait race.
      uint64_t observed;
      { std::lock_guard<std::mutex> lock(mutex_); observed = generation_; }
      auto batch = pop();  // No readiness mutex held; enqueue remains independent.
      if (result.arrived.size() > cap || batch.size() > cap-result.arrived.size())
        throw std::runtime_error("CPV73_READINESS_FAILURE: accumulated queue cap");
      result.arrived.insert(result.arrived.end(),
          std::make_move_iterator(batch.begin()), std::make_move_iterator(batch.end()));
      if (gate.ready(update, result.arrived)) {
        result.wall_ms = std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        return result;
      }
      ++result.waits;
      std::unique_lock<std::mutex> lock(mutex_);
      if (!cv_.wait_until(lock, deadline, [&]{ return generation_ != observed; }))
        throw std::runtime_error("CPV73_READINESS_FAILURE: timeout at update " +
                                 std::to_string(update));
      if (Clock::now() >= deadline)
        throw std::runtime_error("CPV73_READINESS_FAILURE: deadline at update " +
                                 std::to_string(update));
    }
  }
 private:
  const unsigned timeout_ms_;
  std::mutex mutex_;
  std::condition_variable cv_;
  uint64_t generation_ = 0;
};
inline ReplayReadiness& readiness() {
  // One backend per diagnostic process; no class-layout/DSO ABI change.
  static ReplayReadiness barrier(ReplayReadiness::environmentTimeout());
  return barrier;
}
}  // namespace cpv73
