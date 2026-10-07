// Opt-in source-driven receiver glue. Reuses gnssFGO's full WNOJ model;
// there are deliberately no IMU samples, biases or gravity in this interface.
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include "factor/motion/GPWNOJPriorFull.h"
#include "model/gp_interpolator/GPWNOJInterpolatorFull.h"

namespace fgo::offline {
using gtsam::symbol_shorthand::A;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

struct SourceWnojState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{gtsam::Vector3::Zero()};  // world
  gtsam::Vector3 omega{gtsam::Vector3::Zero()};     // body
  gtsam::Vector6 acceleration{gtsam::Vector6::Zero()};  // full-WNOJ tangent derivative
};

inline SourceWnojState ReadSourceWnojState(const gtsam::Values& values, size_t i) {
  return {values.at<gtsam::Pose3>(X(i)), values.at<gtsam::Vector3>(V(i)),
          values.at<gtsam::Vector3>(W(i)), values.at<gtsam::Vector6>(A(i))};
}

inline void InsertSourceWnojState(gtsam::Values& values, size_t i, const SourceWnojState& s) {
  values.insert(X(i), s.pose);
  values.insert(V(i), s.velocity);
  values.insert(W(i), s.omega);
  values.insert(A(i), s.acceleration);
}

// Zero-process-noise local transition of GPWNOJPriorFull, used ONLY as an
// optimizer initial guess. Subsequent optimization estimates all four states.
inline SourceWnojState PredictSourceWnoj(const SourceWnojState& s, double dt) {
  if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("invalid WNOJ interval");
  const auto twist = fgo::utils::convertVwWbToVbWb(s.velocity, s.omega, s.pose);
  const gtsam::Vector6 r = dt * twist + 0.5 * dt * dt * s.acceleration;
  const gtsam::Matrix6 jr = fgo::utils::rightJacobianPose3inv(r).inverse();
  const gtsam::Vector6 local_velocity = twist + dt * s.acceleration;
  const gtsam::Vector6 next_twist = jr * local_velocity;
  SourceWnojState next;
  next.pose = s.pose.compose(gtsam::Pose3::Expmap(r));
  next.velocity = next.pose.rotation().rotate(next_twist.tail<3>());
  next.omega = next_twist.head<3>();
  next.acceleration = jr * (s.acceleration +
    0.5 * fgo::utils::curlyhat(local_velocity) * next_twist);
  return next;
}

// Numeric derivatives are the existing library's useAutoDiff=true path (the
// name is historical). Keep the original analytic implementation untouched.
struct FullWnojEndpoint {
  bool exact{true};
  size_t left{}, right{};
  std::shared_ptr<fgo::models::GPWNOJInterpolatorFull> interpolator;

  static FullWnojEndpoint Exact(size_t i) { return {true, i, i, nullptr}; }
  static FullWnojEndpoint Between(size_t i, size_t j, double dt, double tau,
                                 const gtsam::SharedNoiseModel& qc) {
    if (i >= j || !std::isfinite(dt) || !std::isfinite(tau) ||
        dt <= 0 || tau < 0 || tau > dt)
      throw std::invalid_argument("invalid full-WNOJ endpoint support");
    return {false, i, j, std::make_shared<fgo::models::GPWNOJInterpolatorFull>(
      qc, dt, tau, true, true)};
  }

  gtsam::KeyVector keys() const {
    if (exact) return {X(left)};
    return {X(left), V(left), W(left), A(left), X(right), V(right), W(right), A(right)};
  }

  gtsam::Pose3 pose(const gtsam::Values& values, std::vector<gtsam::Matrix>* h = nullptr) const {
    if (exact) {
      if (h) *h = {gtsam::Matrix6::Identity()};
      return values.at<gtsam::Pose3>(X(left));
    }
    const auto i = ReadSourceWnojState(values, left), j = ReadSourceWnojState(values, right);
    if (h) {
      h->resize(8);
      return interpolator->interpolatePose(i.pose, i.velocity, i.omega, i.acceleration,
        j.pose, j.velocity, j.omega, j.acceleration,
        (*h)[0], (*h)[1], (*h)[2], (*h)[3], (*h)[4], (*h)[5], (*h)[6], (*h)[7]);
    }
    return interpolator->interpolatePose(i.pose, i.velocity, i.omega, i.acceleration,
      j.pose, j.velocity, j.omega, j.acceleration);
  }
};

// One relative factor for arbitrary endpoints. Its key list is the UNION of
// endpoint supports: same/adjacent intervals share variables, not copies.
class FullWnojRelativeFactor final : public gtsam::NoiseModelFactor {
 public:
  FullWnojRelativeFactor(const FullWnojEndpoint& from, const FullWnojEndpoint& to,
                        const gtsam::Pose3& measured, const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor(noise, UniqueKeys(from, to)),
        from_(from), to_(to), measured_(measured) {
    if (!noise || noise->dim() != 6) throw std::invalid_argument("relative noise must be 6D");
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<FullWnojRelativeFactor>(*this);
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& values,
      boost::optional<std::vector<gtsam::Matrix>&> h = boost::none) const override {
    std::vector<gtsam::Matrix> hi, hj;
    const auto pi = from_.pose(values, h ? &hi : nullptr);
    const auto pj = to_.pose(values, h ? &hj : nullptr);
    if (!h) return gtsam::traits<gtsam::Pose3>::Local(measured_, pi.between(pj));
    gtsam::Matrix6 bi, bj, local;
    const auto relative = pi.between(pj, bi, bj);
    const auto error = gtsam::traits<gtsam::Pose3>::Local(measured_, relative, boost::none, local);
    h->clear();
    for (auto key : keys_) h->push_back(gtsam::Matrix::Zero(6, values.at(key).dim()));
    auto add = [&](const gtsam::KeyVector& keys, const std::vector<gtsam::Matrix>& hs,
                   const gtsam::Matrix6& endpoint_h) {
      for (size_t k = 0; k < keys.size(); ++k) {
        const size_t slot = std::find(keys_.begin(), keys_.end(), keys[k]) - keys_.begin();
        (*h)[slot] += local * endpoint_h * hs[k];
      }
    };
    add(from_.keys(), hi, bi);
    add(to_.keys(), hj, bj);
    return error;
  }
 private:
  static gtsam::KeyVector UniqueKeys(const FullWnojEndpoint& a, const FullWnojEndpoint& b) {
    auto keys = a.keys();
    for (auto key : b.keys())
      if (std::find(keys.begin(), keys.end(), key) == keys.end()) keys.push_back(key);
    return keys;
  }
  FullWnojEndpoint from_, to_;
  gtsam::Pose3 measured_;
};
}  // namespace fgo::offline
