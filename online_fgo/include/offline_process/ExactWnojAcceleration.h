#pragma once
#include "factor/motion/GPWNOJPriorFull.h"
#include "model/gp_interpolator/GPWNOJInterpolatorFull.h"

namespace fgo::offline {
// Opt-in numerical implementation, not a different motion/measurement model.
// Keep stock residuals and all non-acceleration Jacobian blocks. Acceleration
// enters the local WNOJ coordinates affinely, so finite differencing it is
// unnecessary and loses precision (especially after world-pose composition).
class ExactAccelerationWnojPrior final : public fgo::factor::GPWNOJPriorFull {
 public:
  using fgo::factor::GPWNOJPriorFull::GPWNOJPriorFull;
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<ExactAccelerationWnojPrior>(*this);
  }
  gtsam::Vector evaluateError(
      const gtsam::Pose3& p,const gtsam::Vector3& v,const gtsam::Vector3& w,const gtsam::Vector6& a,
      const gtsam::Pose3& q,const gtsam::Vector3& u,const gtsam::Vector3& z,const gtsam::Vector6& b,
      boost::optional<gtsam::Matrix&> H1=boost::none,boost::optional<gtsam::Matrix&> H2=boost::none,
      boost::optional<gtsam::Matrix&> H3=boost::none,boost::optional<gtsam::Matrix&> H4=boost::none,
      boost::optional<gtsam::Matrix&> H5=boost::none,boost::optional<gtsam::Matrix&> H6=boost::none,
      boost::optional<gtsam::Matrix&> H7=boost::none,boost::optional<gtsam::Matrix&> H8=boost::none)const override {
    const auto error=fgo::factor::GPWNOJPriorFull::evaluateError(p,v,w,a,q,u,z,b,H1,H2,H3,boost::none,H5,H6,H7,boost::none);
    if(H4){H4->resize(18,6);*H4<<-.5*delta_t_*delta_t_*gtsam::Matrix6::Identity(),-delta_t_*gtsam::Matrix6::Identity(),-gtsam::Matrix6::Identity();}
    if(H8){H8->setZero(18,6);H8->bottomRows(6)=calcJacobian_?fgo::utils::rightJacobianPose3inv(gtsam::Pose3::Logmap(p.inverse().compose(q))):gtsam::Matrix6::Identity();}
    return error;
  }
};

class ExactAccelerationWnojInterpolator final : public fgo::models::GPWNOJInterpolatorFull {
 public:
  using fgo::models::GPWNOJInterpolatorFull::GPWNOJInterpolatorFull;
  gtsam::Pose3 interpolatePose(
      const gtsam::Pose3& p,const gtsam::Vector3& v,const gtsam::Vector3& w,const gtsam::Vector6& a,
      const gtsam::Pose3& q,const gtsam::Vector3& u,const gtsam::Vector3& z,const gtsam::Vector6& b,
      boost::optional<gtsam::Matrix&> H1=boost::none,boost::optional<gtsam::Matrix&> H2=boost::none,
      boost::optional<gtsam::Matrix&> H3=boost::none,boost::optional<gtsam::Matrix&> H4=boost::none,
      boost::optional<gtsam::Matrix&> H5=boost::none,boost::optional<gtsam::Matrix&> H6=boost::none,
      boost::optional<gtsam::Matrix&> H7=boost::none,boost::optional<gtsam::Matrix&> H8=boost::none)const override {
    const auto pose=fgo::models::GPWNOJInterpolatorFull::interpolatePose(p,v,w,a,q,u,z,b,H1,H2,H3,boost::none,H5,H6,H7,boost::none);
    if(H4||H8){
      const gtsam::Vector6 r=gtsam::Pose3::Logmap(p.inverse().compose(q));
      const gtsam::Matrix6 J=calcJacobian_?fgo::utils::rightJacobianPose3inv(r):gtsam::Matrix6::Identity();
      const gtsam::Vector6 v1=fgo::utils::convertVwWbToVbWb(v,w,p),v2=fgo::utils::convertVwWbToVbWb(u,z,q);
      const fgo::utils::Vector_18 r1=(fgo::utils::Vector_18()<<gtsam::Vector6::Zero(),v1,a).finished();
      const fgo::utils::Vector_18 r2=(fgo::utils::Vector_18()<<r,J*v2,-.5*fgo::utils::curlyhat(J*v2)*v2+J*b).finished();
      const gtsam::Vector6 xi=Lambda_.block<6,18>(0,0)*r1+Psi_.block<6,18>(0,0)*r2;
      gtsam::Matrix6 expH,composeH;
      const auto local=gtsam::Pose3::Expmap(xi,expH);
      p.compose(local,boost::none,composeH);
      // Right-local derivative of p*Exp(xi); no additional left-Jacobian factor.
      if(H4)*H4=composeH*expH*Lambda_.block<6,6>(0,12);
      if(H8)*H8=composeH*expH*Psi_.block<6,6>(0,12)*J;
    }
    return pose;
  }
};
} // namespace fgo::offline
