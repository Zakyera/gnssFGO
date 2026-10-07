// Copyright 2026 Yerasimos Zaky
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "offline_process/CausalSupportClock.h"

using fgo::offline::CausalSupportClock;

TEST(CausalSupportClock, StartupAndFractionalRightBoundary)
{
  CausalSupportClock clock(2, 15);
  EXPECT_TRUE(clock.ObserveImu(0, 0).empty());
  auto ready = clock.ObserveImu(10, 12);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].physical_ns, 10);
  EXPECT_EQ(ready[0].available_ns, 12);
  EXPECT_TRUE(clock.ObserveImu(20, 21).empty());
  ready = clock.ObserveImu(30, 37);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].physical_ns, 25);
  EXPECT_EQ(ready[0].available_ns, 37);
}

TEST(CausalSupportClock, MissingEarlierImuBlocksLaterSupports)
{
  CausalSupportClock clock(2, 10);
  clock.ObserveImu(0, 0);
  clock.ObserveImu(10, 80);
  const auto ready = clock.ObserveImu(20, 25);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].available_ns, 80);
}

TEST(CausalSupportClock, FutureSuffixCannotAlterPrefixAndSourcesAreNotInputs)
{
  CausalSupportClock a(2, 10), b(2, 10);
  for (int64_t t : {0, 7, 18, 31}) {
    const auto x = a.ObserveImu(t, t + 3);
    const auto y = b.ObserveImu(t, t + 3);
    ASSERT_EQ(x.size(), y.size());
    for (size_t i = 0; i < x.size(); ++i) {
      EXPECT_EQ(x[i].physical_ns, y[i].physical_ns);
      EXPECT_EQ(x[i].available_ns, y[i].available_ns);
    }
  }
  EXPECT_NE(a.ObserveImu(40, 40).size(), b.ObserveImu(100, 100).size());
}

TEST(CausalSupportClock, RejectsInvalidInput)
{
  EXPECT_THROW(CausalSupportClock(1, 10), std::invalid_argument);
  CausalSupportClock clock(2, 10);
  clock.ObserveImu(10, 10);
  EXPECT_THROW(clock.ObserveImu(10, 11), std::invalid_argument);
  EXPECT_THROW(clock.ObserveImu(20, 19), std::invalid_argument);
  EXPECT_EQ(clock.ObserveImu(20, 20).size(), 1U);
}
