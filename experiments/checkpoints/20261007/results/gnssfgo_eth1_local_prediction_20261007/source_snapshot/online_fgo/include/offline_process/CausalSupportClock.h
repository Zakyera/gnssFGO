// Copyright 2026 Yerasimos Zaky
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fgo::offline
{
struct SupportReady
{
  int64_t physical_ns;
  int64_t available_ns;
};

// Transport-independent clock. Feed IMU samples in physical order; receipt
// times may be out of order. Prefix maximum availability waits for missing
// earlier samples. No source belief, source rate, or future suffix is an input.
// The Nth startup sample anchors X0; all N samples initialize the host.
class CausalSupportClock
{
public:
  CausalSupportClock(size_t startup_count, int64_t period_ns)
  : startup_count_(startup_count), period_ns_(period_ns)
  {
    if (startup_count < 2 || period_ns <= 0) {
      throw std::invalid_argument("invalid independent support clock");
    }
  }

  std::vector<SupportReady> ObserveImu(int64_t physical_ns, int64_t available_ns)
  {
    if ((count_ && physical_ns <= previous_ns_) || available_ns < physical_ns) {
      throw std::invalid_argument("non-monotonic IMU or availability before measurement");
    }
    previous_ns_ = physical_ns;
    available_ns_ = count_ ? std::max(available_ns_, available_ns) : available_ns;
    ++count_;
    std::vector<SupportReady> ready;
    if (count_ < startup_count_) return ready;
    if (count_ == startup_count_) next_ns_ = physical_ns;
    while (next_ns_ <= physical_ns) {
      ready.push_back({next_ns_, available_ns_});
      if (next_ns_ > std::numeric_limits<int64_t>::max() - period_ns_) {
        throw std::overflow_error("support clock overflow");
      }
      next_ns_ += period_ns_;
    }
    return ready;
  }

private:
  size_t startup_count_, count_{};
  int64_t period_ns_, previous_ns_{}, available_ns_{}, next_ns_{};
};
}  // namespace fgo::offline
