// Copyright 2026 Yerasimos Zaky
// Licensed under the Apache License, Version 2.0 (the "License");
#pragma once

#include <gtsam/base/Matrix.h>

#include <cmath>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>

namespace fgo::offline
{

// Pose3 tangent ordering is [rotation, translation]. These are independent
// receiver-model errors, not changes to the covariance reported by a source.
struct SourceModelNoise
{
  double rotation_sigma_rad{};
  double translation_sigma_m{};
};

using SourceModelNoiseMap = std::map<std::string, SourceModelNoise>;

inline void ValidateConfiguredSourceCoverage(
  const SourceModelNoiseMap & configured,
  const std::map<std::string, size_t> & accepted_counts)
{
  for (const auto & [source, noise] : configured) {
    const auto found = accepted_counts.find(source);
    if (found == accepted_counts.end() || found->second == 0) {
      throw std::runtime_error("configured source model noise matched no accepted factors: " + source);
    }
  }
}

inline void ValidateSourceModelNoise(const SourceModelNoise & noise)
{
  if (!std::isfinite(noise.rotation_sigma_rad) || noise.rotation_sigma_rad < 0.0 ||
    !std::isfinite(noise.translation_sigma_m) || noise.translation_sigma_m < 0.0) {
    throw std::invalid_argument("source model standard deviations must be finite and nonnegative");
  }
}

inline gtsam::Matrix6 EffectiveSourceCovariance(
  const gtsam::Matrix6 & reported, const std::string & source,
  const SourceModelNoiseMap & model_noise)
{
  const auto found = model_noise.find(source);
  if (found == model_noise.end()) return reported;
  ValidateSourceModelNoise(found->second);
  gtsam::Matrix6 effective = reported;
  const double rotation_variance = std::pow(found->second.rotation_sigma_rad, 2);
  const double translation_variance = std::pow(found->second.translation_sigma_m, 2);
  for (int axis = 0; axis < 3; ++axis) {
    effective(axis, axis) += rotation_variance;
    effective(axis + 3, axis + 3) += translation_variance;
  }
  return effective;
}

}  // namespace fgo::offline
