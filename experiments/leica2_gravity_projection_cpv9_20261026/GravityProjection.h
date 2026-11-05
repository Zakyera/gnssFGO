#pragma once
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Cholesky>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace cpv9 {
using namespace gtsam;
using Projection = Eigen::Matrix<double,4,6>;
inline bool enabled() {
  const char* value=std::getenv("CPV9_G2K_GRAVITY_PROJECTION");
  if (!value || std::string(value)=="0") return false;
  if (std::string(value)=="1") return true;
  throw std::runtime_error("CPV9 invalid projection selector");
}
inline bool selected(char receiver,char sender) {
  return receiver=='k' && sender=='g' && enabled();
}
inline Vector3 gravity(const NonlinearFactorGraph& old_graph,
                       const NonlinearFactorGraph& new_graph) {
  Vector3 value=Vector3::Zero(); bool found=false;
  for (const auto* graph:{&old_graph,&new_graph}) for (const auto& factor:*graph) {
    Vector3 candidate; bool imu=false;
    if (auto f=dynamic_cast<const ImuFactor*>(factor.get())) {
      candidate=f->preintegratedMeasurements().params()->n_gravity; imu=true;
    } else if (auto f=dynamic_cast<const CombinedImuFactor*>(factor.get())) {
      candidate=f->preintegratedMeasurements().params()->n_gravity; imu=true;
    }
    if (!imu) continue;
    if (!candidate.allFinite() || candidate.norm()<1e-8)
      throw std::runtime_error("CPV9 invalid native gravity");
    if (found && (value-candidate).norm()>1e-10)
      throw std::runtime_error("CPV9 inconsistent native gravity");
    value=candidate; found=true;
  }
  if (!found) throw std::runtime_error("CPV9 no native IMU gravity");
  return value;
}
inline Projection projection(const Pose3& from,const Pose3& measurement,
                             const Vector3& world_gravity) {
  if (!world_gravity.allFinite() || world_gravity.norm()<1e-8)
    throw std::runtime_error("CPV9 invalid gravity");
  // r=Log(Z^-1 Xi^-1 Xj) is the right-local error of Z. Its axes are
  // the nominal endpoint j, whose world orientation is Ri*Rz.
  const Vector3 axis=measurement.rotation().unrotate(
      from.rotation().unrotate(world_gravity.normalized()));
  Projection M=Projection::Zero();
  M.block<1,3>(0,0)=axis.transpose(); M.block<3,3>(1,3)=Matrix3::Identity();
  return M;
}
inline Matrix covariance(const Matrix& raw,const Projection& M) {
  if (raw.rows()!=6 || raw.cols()!=6 || !raw.allFinite() ||
      (raw-raw.transpose()).norm()>1e-10*std::max(1.0,raw.norm()) ||
      Eigen::LLT<Matrix>(raw).info()!=Eigen::Success)
    throw std::runtime_error("CPV9 raw covariance not SPD");
  Matrix C=M*raw*M.transpose();
  if (Eigen::LLT<Matrix>(C).info()!=Eigen::Success)
    throw std::runtime_error("CPV9 projected covariance not SPD");
  return C;
}
// Immutable specialization: preserves the existing BetweenFactor ownership/
// rollback contract (no mutable caches). Error dimension follows noiseModel_.
class GravityProjectedBetweenFactor final : public BetweenFactor<Pose3> {
  Pose3 measurement_;
  Projection projection_;
 public:
  using BetweenFactor<Pose3>::evaluateError;
  GravityProjectedBetweenFactor(Key i,Key j,const Pose3& measured,
      const Matrix& raw,const Projection& M)
      :BetweenFactor<Pose3>(i,j,measured,noiseModel::Gaussian::Covariance(raw)),
       measurement_(measured),projection_(M) {
    noiseModel_=noiseModel::Gaussian::Covariance(covariance(raw,M));
  }
  Vector evaluateError(const Pose3& a,const Pose3& b,
      OptionalMatrixType Ha,OptionalMatrixType Hb) const override {
    Matrix6 A,B,L;
    const Pose3 predicted=a.between(b,A,B);
    const Vector6 r=Pose3::Logmap(measurement_.inverse()*predicted,L);
    if (Ha) *Ha=projection_*L*A;
    if (Hb) *Hb=projection_*L*B;
    return projection_*r;
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<GravityProjectedBetweenFactor>(*this);
  }
  bool equals(const NonlinearFactor& other,double tolerance=1e-9) const override {
    const auto* p=dynamic_cast<const GravityProjectedBetweenFactor*>(&other);
    return p && NoiseModelFactor::equals(other,tolerance) &&
      measurement_.equals(p->measurement_,tolerance) &&
      (projection_-p->projection_).norm()<=tolerance;
  }
  const Projection& M() const {return projection_;}
  const Pose3& measured() const {return measurement_;}
};
}
