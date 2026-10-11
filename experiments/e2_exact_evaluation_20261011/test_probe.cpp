#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include "SourceWnoaModel.h"
#include "ExactLinearizationProbe.h"
using namespace fgo::offline;
using namespace e2_probe;
namespace {
gtsam::Values States() {
  gtsam::Values v;
  SourceWnoaState s;
  s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.2,-.3,.4),{1,2,3});
  s.velocity={.6,.2,-.1};s.omega={.1,.05,.2};
  for(size_t i=0;i<3;++i){InsertSourceWnoaState(v,i,s);s=PredictSourceWnoa(s,.1);}
  return v;
}
void Same(const gtsam::GaussianFactor::shared_ptr& a,const gtsam::GaussianFactor::shared_ptr& b) {
  auto x=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(a);
  auto y=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(b);
  ASSERT_TRUE(x&&y);
  EXPECT_TRUE((x->augmentedJacobian().array()==y->augmentedJacobian().array()).all());
}
TEST(E2Probe, ExactSignatureNotApproximateEquality) {
  auto a=States();const auto keys=gtsam::KeyVector{X(0),V(0),W(0)};
  const auto s=Signature(keys,a,true);
  a.update(V(2),gtsam::Vector3(9,8,7));EXPECT_EQ(s,Signature(keys,a,true));
  auto v=a.at<gtsam::Vector3>(V(0));v[0]=std::nextafter(v[0],INFINITY);
  a.update(V(0),v);EXPECT_NE(s,Signature(keys,a,true));
  a=States();auto p=a.at<gtsam::Pose3>(X(0));auto t=p.translation();
  t[1]=std::nextafter(t[1],INFINITY);a.update(X(0),gtsam::Pose3(p.rotation(),t));
  EXPECT_NE(s,Signature(keys,a,true));EXPECT_NE(s,Signature(keys,States(),false));
}
TEST(E2Probe, RealMotionRepeatsPreserveGaussian) {
  counts={};auto v=States();
  auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  fgo::factor::GPWNOAPrior native(X(0),V(0),W(0),X(1),V(1),W(1),.1,qc,true,true);
  ObservedFactor<fgo::factor::GPWNOAPrior,0> observed(native);
  Same(native.linearize(v),observed.linearize(v));Same(native.linearize(v),observed.linearize(v));
  EXPECT_EQ(counts[0].calls,2);EXPECT_EQ(counts[0].repeats,1);
  auto w=v.at<gtsam::Vector3>(W(0));w[0]=std::nextafter(w[0],INFINITY);v.update(W(0),w);
  Same(native.linearize(v),observed.linearize(v));EXPECT_EQ(counts[0].repeats,1);
}
TEST(E2Probe, DenseRelativeVersionAndCloneHaveNoInheritedSignature) {
  counts={};auto v=States();
  auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  auto a=WnoaEndpoint::Between(0,1,.1,.023,qc),b=WnoaEndpoint::Between(1,2,.1,.071,qc);
  gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.3;l(2,1)=-.2;
  auto noise=gtsam::noiseModel::Gaussian::Covariance(l*l.transpose());
  WnoaRelativeFactor native(a,b,a.pose(v).between(b.pose(v)),noise);
  ObservedFactor<WnoaRelativeFactor,1> observed(native);
  Same(native.linearize(v),observed.linearize(v));
  Same(native.linearize(v),observed.linearize(v));
  EXPECT_EQ(counts[1].repeats,1);
  auto cloned=observed.clone();Same(native.linearize(v),cloned->linearize(v));
  EXPECT_EQ(counts[1].repeats,1);
  ObservedFactor<WnoaRelativeFactor,1> replaced(a,b,gtsam::Pose3(),noise);
  replaced.linearize(v);EXPECT_EQ(counts[1].repeats,1);
}
TEST(E2Probe, MissingValueFailureDoesNotEstablishARepeat) {
  counts={};auto v=States();
  auto qc=gtsam::noiseModel::Isotropic::Sigma(6,1);
  ObservedFactor<fgo::factor::GPWNOAPrior,0> f(X(0),V(0),W(0),X(1),V(1),W(1),.1,qc,true,true);
  auto missing=v;missing.erase(W(0));EXPECT_ANY_THROW(f.linearize(missing));
  f.linearize(v);EXPECT_EQ(counts[0].calls,1);EXPECT_EQ(counts[0].repeats,0);
}
}
