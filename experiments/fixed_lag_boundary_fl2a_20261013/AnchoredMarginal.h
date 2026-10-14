// Experimental FL2a: an immutable joint marginal, not a new sensor prior.
#pragma once
#include "../fixed_lag_receiver_fl2_20261013/MarginalAudit.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/GenericValue.h>
#include <stdexcept>

namespace fl2a {
using namespace gtsam;
class AnchoredMarginal final : public NonlinearFactor {
  GaussianFactor::shared_ptr original_;
  Values anchor_;
  Matrix A_; Vector b_;
 public:
  AnchoredMarginal(const GaussianFactor::shared_ptr& gaussian,const Values& anchor)
    :NonlinearFactor(gaussian->keys()),original_(gaussian->clone()),
     anchor_(fl2::Subset(anchor,gaussian->keys())) {
    const auto ab=gaussian->jacobian(); A_=ab.first;b_=ab.second;
    if(!A_.allFinite()||!b_.allFinite())throw std::invalid_argument("nonfinite marginal");
    for(auto key:keys()) {
      const auto& value=anchor_.at(key);
      if(!dynamic_cast<const GenericValue<Pose3>*>(&value) &&
         !dynamic_cast<const GenericValue<Vector3>*>(&value))
        throw std::invalid_argument("unsupported marginal state type");
    }
  }
  const Values& anchor() const{return anchor_;}
  const GaussianFactor& gaussian() const{return *original_;}
  const Matrix& A() const{return A_;} const Vector& b() const{return b_;}
  VectorValues delta(const Values& x) const {
    return anchor_.localCoordinates(fl2::Subset(x,keys()));
  }
  size_t dim() const override{return A_.rows();}
  double error(const Values& x) const override {
    // Evaluate the original Gaussian too, preserving any additive constant.
    return original_->error(delta(x));
  }
  GaussianFactor::shared_ptr linearize(const Values& x) const override {
    std::vector<std::pair<Key,Matrix>> terms;Vector d(A_.cols());size_t column=0;
    for(auto key:keys()) {
      const auto& value=anchor_.at(key);const size_t n=value.dim();Matrix D;
      if(dynamic_cast<const GenericValue<Pose3>*>(&value)) {
        Matrix6 J;d.segment<6>(column)=traits<Pose3>::Local(
          anchor_.at<Pose3>(key),x.at<Pose3>(key),boost::none,J);D=J;
      } else {
        d.segment<3>(column)=x.at<Vector3>(key)-anchor_.at<Vector3>(key);D=Matrix3::Identity();
      }
      terms.emplace_back(key,A_.middleCols(column,n)*D);column+=n;
    }
    return boost::make_shared<JacobianFactor>(terms,b_-A_*d);
  }
  NonlinearFactor::shared_ptr clone() const override{return boost::make_shared<AnchoredMarginal>(*this);}
};
} // namespace fl2a
