// CP-V101 isolated candidate, based on frozen CP76. Not deployed.
// Preserve the immutable original-chart quadratic; never reset its anchor.
#pragma once
#include <cbs/bpsam/fixed_lag_objective_audit.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <memory>

namespace fl4l {
using namespace gtsam;
inline Values subset(const Values& values, const KeyVector& keys) {
  Values result;
  for (Key key : keys) result.insert(key, values.at(key));
  return result;
}
class OriginalChartPrior final : public NonlinearFactor {
  std::shared_ptr<LinearContainerFactor> original_;
  Values anchor_;
  Matrix H_;
  Vector eta_;
  double constant_;
 public:
  const LinearContainerFactor& cpv66Original() const { return *original_; }
  const Values& cpv66Anchor() const { return anchor_; }
  OriginalChartPrior(const LinearContainerFactor& factor, const Values& theta)
      : NonlinearFactor(factor.keys()),
        original_(std::dynamic_pointer_cast<LinearContainerFactor>(factor.clone())),
        anchor_(subset(factor.hasLinearizationPoint() ? *factor.linearizationPoint() : theta,
                       factor.keys())) {
    const Matrix augmented = factor.factor()->augmentedInformation();
    const auto n = augmented.rows() - 1;
    H_ = augmented.topLeftCorner(n,n);
    eta_ = augmented.topRightCorner(n,1);
    constant_ = augmented(n,n);
    if (!augmented.allFinite()) throw std::runtime_error("invalid prior matrix");
  }
  size_t dim() const override { return H_.rows(); }
  double error(const Values& values) const override {
    return cbs::fixedLagFactorErrorInChart(*original_,values,anchor_);
  }
  GaussianFactor::shared_ptr linearize(const Values& values) const override {
    // Keep validation before equals_: GenericValue's cast is unchecked.
    error(values);
    bool at_anchor = true;
    for (Key key : keys()) {
      if (!anchor_.at(key).equals_(values.at(key), 0.0)) {
        at_anchor = false;
        break;
      }
    }
    // At exactly the stored anchor the chart derivative is identity and its
    // displacement zero. Preserve the Gaussian coefficients exactly; finite
    // differences here can falsely trip the unchanged CPK9 preservation guard.
    // This is NOT a near-anchor threshold or a new linearization point.
    if (at_anchor) return original_->factor()->clone();

    const size_t n = H_.rows();
    Matrix J = Matrix::Zero(n,n); Vector d(n);
    std::vector<size_t> offsets{0};
    for (Key key : keys()) {
      const Value& v = values.at(key); const Value& a = anchor_.at(key);
      const size_t offset = offsets.back(), m = v.dim();
      d.segment(offset,m) = a.localCoordinates_(v);
      for (size_t j=0;j<m;++j) {
        Vector h=Vector::Zero(m); h[j]=1e-6;
        std::unique_ptr<Value> plus(v.retract_(h)), minus(v.retract_(-h));
        J.block(offset,offset+j,m,1) =
          (a.localCoordinates_(*plus)-a.localCoordinates_(*minus))/(2e-6);
      }
      offsets.push_back(offset+m);
    }
    if (offsets.back()!=n) throw std::runtime_error("prior dimension mismatch");
    const Matrix transformed = J.transpose()*H_*J;
    const Vector eta = J.transpose()*(eta_-H_*d);
    const double c = d.dot(H_*d)-2*eta_.dot(d)+constant_;
    std::vector<Matrix> blocks; std::vector<Vector> rhs;
    for (size_t i=0;i<keys().size();++i) {
      const size_t mi=offsets[i+1]-offsets[i];
      rhs.push_back(eta.segment(offsets[i],mi));
      for (size_t j=i;j<keys().size();++j)
        blocks.push_back(transformed.block(offsets[i],offsets[j],mi,offsets[j+1]-offsets[j]));
    }
    return std::make_shared<HessianFactor>(keys(),blocks,rhs,c);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<OriginalChartPrior>(*this);
  }
};
inline NonlinearFactorGraph anchored(const NonlinearFactorGraph& input,
                                     const Values& theta) {
  NonlinearFactorGraph result;
  for (const auto& f : input) {
    if (!f) continue;
    if (const auto* p=dynamic_cast<const LinearContainerFactor*>(f.get()))
      result.push_back(std::make_shared<OriginalChartPrior>(*p,theta));
    else result.push_back(f->clone());
  }
  return result;
}
} // namespace fl4l
