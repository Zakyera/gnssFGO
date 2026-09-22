// Copyright 2026 Yerasimos Zaky
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace fgo::offline
{

  struct AllRightSegment
  {
    size_t sample_index {};
    int64_t overlap_start_ns {};
    int64_t overlap_end_ns {};

    [[nodiscard]] double durationSeconds() const
    {
      return static_cast < double > (overlap_end_ns - overlap_start_ns) * 1e-9;
    }
  };

// Return the exact overlaps for the interval-end convention: sample k, stamped
// t_k, owns [t_(k-1), t_k]. The caller uses sample_index to retrieve the held
// rate. This helper deliberately knows nothing about an estimator or GTSAM.
  inline std::vector < AllRightSegment > MakeAllRightSegments(
    const std::vector < int64_t > &timestamps_ns,
    int64_t start_ns,
    int64_t end_ns)
  {
    if (timestamps_ns.size() < 2 || end_ns <= start_ns ||
      start_ns < timestamps_ns.front() || end_ns > timestamps_ns.back())
    {
      throw std::invalid_argument("invalid ALL_RIGHT integration interval");
    }
    if (!std::is_sorted(timestamps_ns.begin(), timestamps_ns.end()) ||
      std::adjacent_find(timestamps_ns.begin(), timestamps_ns.end()) != timestamps_ns.end())
    {
      throw std::invalid_argument("IMU timestamps must be strictly increasing");
    }

    const auto first_after_start = std::upper_bound(
      timestamps_ns.begin() + 1, timestamps_ns.end(), start_ns);
    size_t sample_index = static_cast < size_t > (
      std::distance(timestamps_ns.begin(), first_after_start));
    std::vector < AllRightSegment > segments;
    while (sample_index < timestamps_ns.size() &&
      timestamps_ns[sample_index - 1] < end_ns)
    {
      const int64_t overlap_start = std::max(start_ns, timestamps_ns[sample_index - 1]);
      const int64_t overlap_end = std::min(end_ns, timestamps_ns[sample_index]);
      if (overlap_end > overlap_start) {
        segments.push_back({sample_index, overlap_start, overlap_end});
      }
      if (timestamps_ns[sample_index] >= end_ns) {
        break;
      }
      ++sample_index;
    }

    int64_t integrated_ns = 0;
    for (const AllRightSegment & segment : segments) {
      integrated_ns += segment.overlap_end_ns - segment.overlap_start_ns;
    }
    if (integrated_ns != end_ns - start_ns) {
      throw std::runtime_error("ALL_RIGHT overlap integration did not conserve duration");
    }
    return segments;
  }

}  // namespace fgo::offline
