// Copyright 2026 Yerasimos Zaky
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>

#include "offline_process/SourceModelDiscrepancy.h"

namespace
{

TEST(SourceModelDiscrepancy, ZeroDefaultIsExactAndOtherSourcesAreIsolated)
{
  gtsam::Matrix6 reported = gtsam::Matrix6::Identity();
  reported(0, 4) = reported(4, 0) = 0.125;
  const fgo::offline::SourceModelNoiseMap model{{"kimera", {0.0, 0.0}}};
  EXPECT_EQ(fgo::offline::EffectiveSourceCovariance(reported, "kimera", model), reported);
  EXPECT_EQ(fgo::offline::EffectiveSourceCovariance(reported, "glim", model), reported);
}

TEST(SourceModelDiscrepancy, AddsOnlyMarginalDiagonalVariancesAndPreservesDenseCrossTerms)
{
  gtsam::Matrix6 reported = 2.0 * gtsam::Matrix6::Identity();
  reported(0, 4) = reported(4, 0) = 0.125;
  reported(3, 5) = reported(5, 3) = -0.25;
  const fgo::offline::SourceModelNoiseMap model{{"kimera", {0.1, 0.2}}};
  const auto effective = fgo::offline::EffectiveSourceCovariance(reported, "kimera", model);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(effective(axis, axis), 2.01, 1e-14);
    EXPECT_NEAR(effective(axis + 3, axis + 3), 2.04, 1e-14);
  }
  EXPECT_EQ(effective(0, 4), reported(0, 4));
  EXPECT_EQ(effective(4, 0), reported(4, 0));
  EXPECT_EQ(effective(3, 5), reported(3, 5));
  EXPECT_EQ(effective(5, 3), reported(5, 3));
  EXPECT_EQ(reported(0, 0), 2.0);
}

TEST(SourceModelDiscrepancy, RejectsNegativeOrNonfiniteSigma)
{
  EXPECT_THROW(fgo::offline::ValidateSourceModelNoise({-1.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(fgo::offline::ValidateSourceModelNoise({0.0, INFINITY}), std::invalid_argument);
}

TEST(SourceModelDiscrepancy, RejectsUnusedConfiguredSourceId)
{
  const fgo::offline::SourceModelNoiseMap model{{"glim_hesai", {0.1, 0.2}},
                                                {"kimera_front", {0.1, 0.2}}};
  EXPECT_NO_THROW(fgo::offline::ValidateConfiguredSourceCoverage(
      model, {{"glim_hesai", 10}, {"kimera_front", 5}}));
  EXPECT_THROW(fgo::offline::ValidateConfiguredSourceCoverage(
      model, {{"glim_hesai", 10}}), std::runtime_error);
  EXPECT_THROW(fgo::offline::ValidateConfiguredSourceCoverage(
      model, {{"glim_hesai", 10}, {"kimera_front", 0}}), std::runtime_error);
}

}  // namespace
