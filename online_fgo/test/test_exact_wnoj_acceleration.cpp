#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include "offline_process/ExactWnojAcceleration.h"
#include "offline_process/ExactAccelerationSourceWnoj.h"
namespace {
using namespace gtsam;
using namespace fgo::offline;
const auto noise=noiseModel::Diagonal::Variances((Vector6()<<2.5,2.5,2.5,10,10,10).finished());
template<class F> Matrix FivePoint(F fn,const Vector6& x,double h){
  Matrix out(fn(x).size(),6);
  for(int k=0;k<6;++k){Vector6 d=Vector6::Zero();d[k]=h;out.col(k)=(8*(fn(x+d)-fn(x-d))-(fn(x+2*d)-fn(x-2*d)))/(12*h);}
  return out;
}
TEST(ExactWnojAcceleration, PriorPreservesResidualAndOtherJacobianBlocks){
  for(bool jac:{false,true}){
    fgo::factor::GPWNOJPriorFull old(1,2,3,4,5,6,7,8,.1,noise,true,jac);
    ExactAccelerationWnojPrior exact(1,2,3,4,5,6,7,8,.1,noise,true,jac);
    Pose3 p(Rot3::RzRyRx(.2,-.1,.3),Point3(80,50,2)),q=p.compose(Pose3::Expmap((Vector6()<<.03,-.02,.01,.4,.1,.05).finished()));
    Vector3 v(1,2,3),u(2,1,3),w(.1,.2,.3),z(.3,.2,.1);
    Vector6 a=Vector6::LinSpaced(6,-1.,2.),b=-a;
    Matrix h[8],k[8];auto r=old.evaluateError(p,v,w,a,q,u,z,b,h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]);
    auto s=exact.evaluateError(p,v,w,a,q,u,z,b,k[0],k[1],k[2],k[3],k[4],k[5],k[6],k[7]);
    EXPECT_EQ((r-s).norm(),0.);EXPECT_EQ((exact.evaluateError(p,v,w,a,q,u,z,b)-r).norm(),0.);
    for(int i:{0,1,2,4,5,6})EXPECT_EQ((h[i]-k[i]).norm(),0.);
    for(double step:{1e-2,1e-3,1e-4}){
      auto da=FivePoint([&](const Vector6& aa){return old.evaluateError(p,v,w,aa,q,u,z,b);},a,step);
      auto db=FivePoint([&](const Vector6& bb){return old.evaluateError(p,v,w,a,q,u,z,bb);},b,step);
      EXPECT_LT((da-k[3]).cwiseAbs().maxCoeff(),1e-10);EXPECT_LT((db-k[7]).cwiseAbs().maxCoeff(),1e-10);
    }
    EXPECT_EQ((exact.noiseModel()->sigmas()-old.noiseModel()->sigmas()).norm(),0.);
    EXPECT_NE(boost::dynamic_pointer_cast<ExactAccelerationWnojPrior>(exact.clone()),nullptr);
  }
}
TEST(ExactWnojAcceleration, InterpolationPreservesPoseAndOtherBlocks){
 for(bool jac:{false,true})for(double tau:{0.,.001,.023,.05,.071,.099,.1}){
    fgo::models::GPWNOJInterpolatorFull old(noise,.1,tau,true,jac);
    ExactAccelerationWnojInterpolator exact(noise,.1,tau,true,jac);
    Pose3 p(Rot3::RzRyRx(.2,-.1,.3),Point3(80,50,2)),q=p.compose(Pose3::Expmap((Vector6()<<.03,-.02,.01,.4,.1,.05).finished()));
    Vector3 v(1,2,3),u(2,1,3),w(.1,.2,.3),z(.3,.2,.1);Vector6 a=Vector6::LinSpaced(6,-1.,2.),b=-a;
    Matrix h[8],k[8];const auto r=old.interpolatePose(p,v,w,a,q,u,z,b,h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]);
    const auto s=exact.interpolatePose(p,v,w,a,q,u,z,b,k[0],k[1],k[2],k[3],k[4],k[5],k[6],k[7]);
    EXPECT_EQ((r.matrix()-s.matrix()).norm(),0.);EXPECT_EQ((r.matrix()-exact.interpolatePose(p,v,w,a,q,u,z,b).matrix()).norm(),0.);
    for(int i:{0,1,2,4,5,6})EXPECT_EQ((h[i]-k[i]).norm(),0.);
    for(double step:{1e-2,1e-3,1e-4}){
      auto da=FivePoint([&](const Vector6& aa){return Pose3::Logmap(r.between(old.interpolatePose(p,v,w,aa,q,u,z,b)));},a,step);
      auto db=FivePoint([&](const Vector6& bb){return Pose3::Logmap(r.between(old.interpolatePose(p,v,w,a,q,u,z,bb)));},b,step);
      EXPECT_LT((da-k[3]).cwiseAbs().maxCoeff(),2e-9);EXPECT_LT((db-k[7]).cwiseAbs().maxCoeff(),2e-9);
    }
 }
}
TEST(ExactWnojAcceleration, ExplicitFactoryPreservesSharedEndpointFactorAndCovariance){
 Values values;SourceWnojState state;state.pose=Pose3(Rot3::RzRyRx(.2,.1,-.3),Point3(80,50,2));
 state.velocity=Vector3(1,2,.3);state.omega=Vector3(.1,.2,.3);state.acceleration=Vector6::LinSpaced(6,-1.,2.);
 InsertSourceWnojState(values,0,state);InsertSourceWnojState(values,1,PredictSourceWnoj(state,.1));
 const auto a=FullWnojEndpoint::Between(0,1,.1,.023,noise),b=FullWnojEndpoint::Between(0,1,.1,.071,noise);
 const auto x=ExactAccelerationWnojEndpointBetween(0,1,.1,.023,noise),y=ExactAccelerationWnojEndpointBetween(0,1,.1,.071,noise);
 const auto measured=Pose3::Expmap((Vector6()<<.01,.02,.03,.1,.2,.3).finished());
 FullWnojRelativeFactor old(a,b,measured,noise),exact(x,y,measured,noise);
 std::vector<Matrix> h,k;const auto r=old.unwhitenedError(values,h),s=exact.unwhitenedError(values,k);
 EXPECT_EQ((r-s).norm(),0.);EXPECT_EQ(old.keys(),exact.keys());EXPECT_EQ(old.noiseModel(),exact.noiseModel());
 ASSERT_EQ(h.size(),8);
 for(size_t i:{0,1,2,4,5,6})EXPECT_EQ((h[i]-k[i]).norm(),0.);
 EXPECT_EQ(old.error(values),exact.error(values));
 EXPECT_THROW(ExactAccelerationWnojEndpointBetween(0,1,.1,.2,noise),std::invalid_argument);
}
}
