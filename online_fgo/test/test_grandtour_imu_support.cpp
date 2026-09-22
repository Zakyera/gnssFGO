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

#include <cstdint>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "offline_process/GrandTourImuSupport.h"

namespace
{

using fgo::offline::AllRightSegment;
using fgo::offline::MakeAllRightSegments;

int64_t IntegratedNanoseconds(const std::vector<AllRightSegment> & segments)
{
  return std::accumulate(
    segments.begin(), segments.end(), int64_t{0},
    [](int64_t total, const AllRightSegment & segment) {
      return total + segment.overlap_end_ns - segment.overlap_start_ns;
    });
}

TEST(GrandTourImuSupport, ExactFullIntervalsUseFollowingTimestampedSamples)
{
  const std::vector<int64_t> timestamps{0, 10, 20, 35};
  const auto segments = MakeAllRightSegments(timestamps, 10, 35);
  ASSERT_EQ(segments.size(), 2U);
  EXPECT_EQ(segments[0].sample_index, 2U);
  EXPECT_EQ(segments[0].overlap_start_ns, 10);
  EXPECT_EQ(segments[0].overlap_end_ns, 20);
  EXPECT_EQ(segments[1].sample_index, 3U);
  EXPECT_EQ(segments[1].overlap_start_ns, 20);
  EXPECT_EQ(segments[1].overlap_end_ns, 35);
}

TEST(GrandTourImuSupport, FractionalLeadingAndTrailingIntervalsAreExact)
{
  const std::vector<int64_t> timestamps{0, 10, 20, 35};
  const auto segments = MakeAllRightSegments(timestamps, 5, 27);
  ASSERT_EQ(segments.size(), 3U);
  EXPECT_EQ(segments[0].sample_index, 1U);
  EXPECT_EQ(segments[0].overlap_start_ns, 5);
  EXPECT_EQ(segments[0].overlap_end_ns, 10);
  EXPECT_EQ(segments[1].sample_index, 2U);
  EXPECT_EQ(segments[1].overlap_start_ns, 10);
  EXPECT_EQ(segments[1].overlap_end_ns, 20);
  EXPECT_EQ(segments[2].sample_index, 3U);
  EXPECT_EQ(segments[2].overlap_start_ns, 20);
  EXPECT_EQ(segments[2].overlap_end_ns, 27);
  EXPECT_EQ(IntegratedNanoseconds(segments), 22);
}

TEST(GrandTourImuSupport, BoundarySplitConservesTheOriginalIncrement)
{
  const std::vector<int64_t> timestamps{0, 10, 20};
  const auto first = MakeAllRightSegments(timestamps, 2, 6);
  const auto second = MakeAllRightSegments(timestamps, 6, 10);
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(first.front().sample_index, 1U);
  EXPECT_EQ(second.front().sample_index, 1U);
  EXPECT_EQ(IntegratedNanoseconds(first) + IntegratedNanoseconds(second), 8);
}

TEST(GrandTourImuSupport, RejectsMissingCoverageAndNonMonotonicTimestamps)
{
  EXPECT_THROW(
    MakeAllRightSegments(std::vector<int64_t>{0, 10, 20}, -1, 10),
    std::invalid_argument);
  EXPECT_THROW(
    MakeAllRightSegments(std::vector<int64_t>{0, 10, 10}, 0, 10),
    std::invalid_argument);
}

}  // namespace
