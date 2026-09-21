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

#ifndef ONLINE_FGO_GPINTERPOLATEDSAMEINTERVALPOSE3BETWEENFACTOR_H
#define ONLINE_FGO_GPINTERPOLATEDSAMEINTERVALPOSE3BETWEENFACTOR_H

#pragma once

#include <array>
#include <iostream>
#include <memory>
#include <string>

#include <gtsam/base/Lie.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "factor/FactorTypeID.h"
#include "model/gp_interpolator/GPInterpolatorBase.h"

namespace fgo::factor
{

/**
 * Relative Pose3 factor whose two measurement endpoints are interpolated from
 * the same pair of continuous-time support states.
 *
 * Keeping the six support keys unique is important: a 12-key double
 * interpolation factor cannot safely repeat the same six keys when it is
 * linearized into a Gaussian factor.
 */
class GPInterpolatedSameIntervalPose3BetweenFactor
  : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3,
    gtsam::Vector3, gtsam::Pose3,
    gtsam::Vector3, gtsam::Vector3>
{
private:
  using This = GPInterpolatedSameIntervalPose3BetweenFactor;
  using Base = gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3,
      gtsam::Vector3, gtsam::Pose3,
      gtsam::Vector3, gtsam::Vector3>;
  using GPBase = std::shared_ptr<fgo::models::GPInterpolator>;

  GPBase firstInterpolator_;
  GPBase secondInterpolator_;
  gtsam::Pose3 measured_;

public:
  using shared_ptr = boost::shared_ptr<This>;

  GPInterpolatedSameIntervalPose3BetweenFactor() = default;

  GPInterpolatedSameIntervalPose3BetweenFactor(
    gtsam::Key poseI, gtsam::Key velocityI, gtsam::Key omegaI,
    gtsam::Key poseJ, gtsam::Key velocityJ, gtsam::Key omegaJ,
    const gtsam::Pose3 & measured,
    const GPBase & firstInterpolator,
    const GPBase & secondInterpolator,
    const gtsam::SharedNoiseModel & model = nullptr)
  : Base(model, poseI, velocityI, omegaI, poseJ, velocityJ, omegaJ),
    firstInterpolator_(firstInterpolator),
    secondInterpolator_(secondInterpolator),
    measured_(measured)
  {
    factorTypeID_ = FactorTypeID::GPSameIntervalBetweenPose;
    factorName_ = "GPInterpolatedSameIntervalPose3BetweenFactor";
  }

  ~GPInterpolatedSameIntervalPose3BetweenFactor() override = default;

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(
    const std::string & prefix = "",
    const gtsam::KeyFormatter & formatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << prefix << factorName_ << "("
              << formatter(this->key1()) << ", "
              << formatter(this->key2()) << ", "
              << formatter(this->key3()) << ", "
              << formatter(this->key4()) << ", "
              << formatter(this->key5()) << ", "
              << formatter(this->key6()) << ")\n";
    gtsam::traits<gtsam::Pose3>::Print(measured_, "  measured: ");
    if (this->noiseModel_) {
      this->noiseModel_->print("  noise model: ");
    }
  }

  [[nodiscard]] bool equals(
    const gtsam::NonlinearFactor & expected,
    double tolerance = 1e-9) const override
  {
    const auto * other = dynamic_cast<const This *>(&expected);
    return other != nullptr && Base::equals(*other, tolerance) &&
           measured_.equals(other->measured_, tolerance);
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 & poseI, const gtsam::Vector3 & velocityI,
    const gtsam::Vector3 & omegaI, const gtsam::Pose3 & poseJ,
    const gtsam::Vector3 & velocityJ, const gtsam::Vector3 & omegaJ,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none,
    boost::optional<gtsam::Matrix &> H3 = boost::none,
    boost::optional<gtsam::Matrix &> H4 = boost::none,
    boost::optional<gtsam::Matrix &> H5 = boost::none,
    boost::optional<gtsam::Matrix &> H6 = boost::none) const override
  {
    std::array<gtsam::Matrix, 6> firstJacobians;
    std::array<gtsam::Matrix, 6> secondJacobians;
    const bool computeJacobians = H1 || H2 || H3 || H4 || H5 || H6;

    gtsam::Pose3 firstPose;
    gtsam::Pose3 secondPose;
    if (computeJacobians) {
      firstPose = firstInterpolator_->interpolatePose(
        poseI, velocityI, omegaI, poseJ, velocityJ, omegaJ,
        firstJacobians[0], firstJacobians[1], firstJacobians[2],
        firstJacobians[3], firstJacobians[4], firstJacobians[5]);
      secondPose = secondInterpolator_->interpolatePose(
        poseI, velocityI, omegaI, poseJ, velocityJ, omegaJ,
        secondJacobians[0], secondJacobians[1], secondJacobians[2],
        secondJacobians[3], secondJacobians[4], secondJacobians[5]);
    } else {
      firstPose = firstInterpolator_->interpolatePose(
        poseI, velocityI, omegaI, poseJ, velocityJ, omegaJ);
      secondPose = secondInterpolator_->interpolatePose(
        poseI, velocityI, omegaI, poseJ, velocityJ, omegaJ);
    }

    gtsam::Matrix betweenFirst;
    gtsam::Matrix betweenSecond;
    const gtsam::Pose3 predicted = firstPose.between(
      secondPose, betweenFirst, betweenSecond);

    gtsam::Matrix localPredicted;
    const gtsam::Vector residual = gtsam::traits<gtsam::Pose3>::Local(
      measured_, predicted, boost::none, localPredicted);

    if (computeJacobians) {
      std::array<boost::optional<gtsam::Matrix &>, 6> outputs{
        H1, H2, H3, H4, H5, H6};
      for (size_t index = 0; index < outputs.size(); ++index) {
        if (outputs[index]) {
          *outputs[index] = localPredicted *
            (betweenFirst * firstJacobians[index] +
            betweenSecond * secondJacobians[index]);
        }
      }
    }
    return residual;
  }

  [[nodiscard]] const gtsam::Pose3 & measured() const {return measured_;}
};

}  // namespace fgo::factor

namespace gtsam
{
template<>
struct traits<fgo::factor::GPInterpolatedSameIntervalPose3BetweenFactor>
  : public Testable<fgo::factor::GPInterpolatedSameIntervalPose3BetweenFactor> {};
}  // namespace gtsam

#endif  // ONLINE_FGO_GPINTERPOLATEDSAMEINTERVALPOSE3BETWEENFACTOR_H
