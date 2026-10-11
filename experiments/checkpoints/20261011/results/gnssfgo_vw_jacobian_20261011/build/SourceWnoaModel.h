// Opt-in source-driven receiver glue. Reuses gnssFGO's full WNOA model;
// there are deliberately no IMU samples, biases or gravity in this interface.
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include "factor/motion/GPWNOAPrior.h"
#include "model/gp_interpolator/GPWNOAInterpolator.h"

namespace fgo::offline {
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

struct SourceWnoaState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{gtsam::Vector3::Zero()};  // world
  gtsam::Vector3 omega{gtsam::Vector3::Zero()};     // body
};

inline SourceWnoaState ReadSourceWnoaState(const gtsam::Values& values, size_t i) {
  return {values.at<gtsam::Pose3>(X(i)), values.at<gtsam::Vector3>(V(i)),
          values.at<gtsam::Vector3>(W(i))};
}

inline void InsertSourceWnoaState(gtsam::Values& values, size_t i, const SourceWnoaState& s) {
  values.insert(X(i), s.pose);
  values.insert(V(i), s.velocity);
  values.insert(W(i), s.omega);
}

// Zero-process-noise local transition of GPWNOAPrior, used ONLY as an
// optimizer initial guess. Subsequent optimization estimates all three states.
inline SourceWnoaState PredictSourceWnoa(const SourceWnoaState& s, double dt) {
  if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("invalid WNOA interval");
  const auto twist = fgo::utils::convertVwWbToVbWb(s.velocity, s.omega, s.pose);
  const gtsam::Vector6 r = dt * twist;
  const gtsam::Matrix6 jr = fgo::utils::rightJacobianPose3inv(r).inverse();
  const gtsam::Vector6 local_velocity = twist;
  const gtsam::Vector6 next_twist = jr * local_velocity;
  SourceWnoaState next;
  next.pose = s.pose.compose(gtsam::Pose3::Expmap(r));
  next.velocity = next.pose.rotation().rotate(next_twist.tail<3>());
  next.omega = next_twist.head<3>();
  return next;
}

// Private derivative-only candidate: keep native residuals and numeric pose
// derivatives. Reuse the library's closed-form velocity/angular-rate blocks.
// The original library and production receiver remain untouched.
struct WnoaEndpoint {
  bool exact{true};
  size_t left{}, right{};
  std::shared_ptr<fgo::models::GPWNOAInterpolator> interpolator;
  std::shared_ptr<fgo::models::GPWNOAInterpolator> twist_interpolator;

  static WnoaEndpoint Exact(size_t i) { return {true, i, i, nullptr}; }
  static WnoaEndpoint Between(size_t i, size_t j, double dt, double tau,
                                 const gtsam::SharedNoiseModel& qc) {
    if (i >= j || !std::isfinite(dt) || !std::isfinite(tau) ||
        dt <= 0 || tau < 0 || tau > dt)
      throw std::invalid_argument("invalid full-WNOA endpoint support");
    return {false, i, j, std::make_shared<fgo::models::GPWNOAInterpolator>(
      qc, dt, tau, true, true),
      std::make_shared<fgo::models::GPWNOAInterpolator>(qc, dt, tau, false, true)};
  }

  gtsam::KeyVector keys() const {
    if (exact) return {X(left)};
    return {X(left), V(left), W(left), X(right), V(right), W(right)};
  }

  gtsam::Pose3 pose(const gtsam::Values& values, std::vector<gtsam::Matrix>* h = nullptr) const {
    if (exact) {
      if (h) *h = {gtsam::Matrix6::Identity()};
      return values.at<gtsam::Pose3>(X(left));
    }
    const auto i = ReadSourceWnoaState(values, left), j = ReadSourceWnoaState(values, right);
    if (h) {
      h->resize(6);
      const auto result = interpolator->interpolatePose(i.pose, i.velocity, i.omega,
        j.pose, j.velocity, j.omega,
        (*h)[0], boost::none, boost::none, (*h)[3], boost::none, boost::none);
      // Do not request analytic pose blocks (they use a separate derivative
      // path). Do not substitute the returned analytic pose into the residual.
      (void)twist_interpolator->interpolatePose(i.pose, i.velocity, i.omega,
        j.pose, j.velocity, j.omega,
        boost::none, (*h)[1], (*h)[2], boost::none, (*h)[4], (*h)[5]);
      return result;
    }
    return interpolator->interpolatePose(i.pose, i.velocity, i.omega,
      j.pose, j.velocity, j.omega);
  }
};

// One relative factor for arbitrary endpoints. Its key list is the UNION of
// endpoint supports: same/adjacent intervals share variables, not copies.
class WnoaRelativeFactor final : public gtsam::NoiseModelFactor {
 public:
  WnoaRelativeFactor(const WnoaEndpoint& from, const WnoaEndpoint& to,
                        const gtsam::Pose3& measured, const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor(noise, UniqueKeys(from, to)),
        from_(from), to_(to), measured_(measured) {
    if (!noise || noise->dim() != 6) throw std::invalid_argument("relative noise must be 6D");
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<WnoaRelativeFactor>(*this);
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
  static gtsam::KeyVector UniqueKeys(const WnoaEndpoint& a, const WnoaEndpoint& b) {
    auto keys = a.keys();
    for (auto key : b.keys())
      if (std::find(keys.begin(), keys.end(), key) == keys.end()) keys.push_back(key);
    return keys;
  }
  WnoaEndpoint from_, to_;
  gtsam::Pose3 measured_;
};
}  // namespace fgo::offline
