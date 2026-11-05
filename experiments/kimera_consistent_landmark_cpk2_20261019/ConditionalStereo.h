#pragma once
// CP-K2 standalone candidate. Not installed in any estimator.
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace cpk2 {
using Smart = gtsam::SmartStereoProjectionPoseFactor;
struct Fit {
  gtsam::Point3 point;
  Smart::FBlocks F;
  gtsam::Matrix E;
  gtsam::Vector b;
  double cost=0, scaledGradient=0;
  size_t iterations=0;
};

inline Fit equations(const Smart& factor, const Smart::Cameras& cameras,
                     const gtsam::Point3& point) {
  if(!point.allFinite()) throw std::runtime_error("CPK2 nonfinite landmark");
  for(const auto& camera:cameras)
    if(camera.pose().transformTo(point).z()<=0)
      throw std::runtime_error("CPK2 nonpositive depth");
  Fit f; f.point=point;
  // This includes body-to-sensor Jacobians and missing-right-row handling.
  factor.computeJacobians(f.F,f.E,f.b,cameras,point);
  factor.whitenJacobians(f.F,f.E,f.b);
  if(!f.E.allFinite() || !f.b.allFinite())
    throw std::runtime_error("CPK2 nonfinite stereo equations");
  const gtsam::Matrix3 H=f.E.transpose()*f.E;
  const gtsam::Vector3 g=-f.E.transpose()*f.b;
  const gtsam::Vector3 scale=H.diagonal().cwiseSqrt();
  if((scale.array()<=0).any() || !scale.allFinite())
    throw std::runtime_error("CPK2 zero landmark information");
  const gtsam::Matrix3 normalized=scale.cwiseInverse().asDiagonal()*H*scale.cwiseInverse().asDiagonal();
  Eigen::LLT<gtsam::Matrix3> llt(normalized);
  if(llt.info()!=Eigen::Success || llt.matrixL().toDenseMatrix().diagonal().minCoeff()<1e-7)
    throw std::runtime_error("CPK2 singular landmark information");
  f.cost=.5*f.b.squaredNorm();
  f.scaledGradient=g.cwiseQuotient(scale).lpNorm<Eigen::Infinity>();
  return f;
}

inline Fit minimize(const Smart& factor,const gtsam::Values& poses,
                    const gtsam::Point3& initial,std::ostream* trace=nullptr) {
  const auto cameras=factor.cameras(poses);
  Fit f=equations(factor,cameras,initial);
  double lambda=0;
  for(size_t iteration=0;iteration<=30;++iteration){
    f.iterations=iteration;
    if(trace)*trace<<std::setprecision(17)<<"point_iteration,"<<iteration<<','<<f.cost<<','<<f.scaledGradient<<','<<lambda<<','<<f.point.transpose()<<'\n';
    if(f.scaledGradient<=1e-10*(1+std::sqrt(2*f.cost))) return f;
    if(iteration==30) break;
    const gtsam::Matrix3 H=f.E.transpose()*f.E;
    const gtsam::Vector3 rhs=f.E.transpose()*f.b;
    const gtsam::Vector3 scale=H.diagonal().cwiseSqrt();
    bool accepted=false;
    for(size_t trial=0;trial<12;++trial){
      // Solve the same 3D damped least squares directly, without squaring its
      // condition number in a normal-equation solve (weak-depth tracks matter).
      gtsam::Matrix A(f.E.rows()+3,3);gtsam::Vector b=gtsam::Vector::Zero(f.b.size()+3);
      A.topRows(f.E.rows())=f.E*scale.cwiseInverse().asDiagonal();
      A.bottomRows(3)=std::sqrt(lambda)*gtsam::Matrix3::Identity();b.head(f.b.size())=f.b;
      const gtsam::Vector3 step=A.colPivHouseholderQr().solve(b).cwiseQuotient(scale);
      try {
        Fit next=equations(factor,cameras,f.point+step);
        const double roundoff=1e-12*(1+f.cost);
        const double predicted=rhs.dot(step)-.5*step.dot(H*step);
        if(trace)*trace<<"point_trial,"<<trial<<','<<next.cost-f.cost<<','<<next.scaledGradient<<','<<predicted<<','<<step.norm()<<'\n';
        // A roundoff-sized cost tie is permissible only with a smaller gradient.
        // Final convergence still requires the explicit gradient criterion.
        const bool numericallyFlat=std::abs(predicted)<=roundoff;
        if((!numericallyFlat && next.cost<f.cost) ||
           (numericallyFlat && next.cost<=f.cost+roundoff && next.scaledGradient<f.scaledGradient)){
          f=std::move(next);lambda*=.1;accepted=true;break;
        }
      } catch(const std::runtime_error&) { /* reject this trial; original untouched */ }
      lambda=lambda==0?1e-6:lambda*10;
    }
    if(!accepted){
      std::ostringstream detail;detail<<std::setprecision(17)
        <<"CPK2 no stationary decreasing landmark step: iteration="<<iteration
        <<" cost="<<f.cost<<" scaled_gradient="<<f.scaledGradient
        <<" limit="<<1e-10*(1+std::sqrt(2*f.cost))<<" lambda="<<lambda;
      throw std::runtime_error(detail.str());
    }
  }
  std::ostringstream detail;detail<<std::setprecision(17)
    <<"CPK2 landmark not stationary within30 iterations: cost="<<f.cost
    <<" scaled_gradient="<<f.scaledGradient<<" limit="<<1e-10*(1+std::sqrt(2*f.cost))
    <<" lambda="<<lambda;
  throw std::runtime_error(detail.str());
}

inline gtsam::Matrix denseF(const Fit& f){
  gtsam::Matrix F=gtsam::Matrix::Zero(f.b.size(),6*f.F.size());
  for(size_t i=0;i<f.F.size();++i) F.block(3*i,6*i,3,6)=f.F[i];
  return F;
}
inline std::shared_ptr<gtsam::GaussianFactor> schur(const Smart& factor,const Fit& fit){
  return std::make_shared<gtsam::RegularHessianFactor<6>>(
      factor.keys(),Smart::Cameras::SchurComplement(fit.F,fit.E,fit.b));
}
}
