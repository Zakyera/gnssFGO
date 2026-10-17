#include "test_memo.cpp"
#include <gtsam/linear/HessianFactor.h>
#include <chrono>
#include <iostream>
namespace {
TEST(E2Audit, EveryReferencedVectorCoordinateAndPoseEndpointInvalidates) {
  auto native=Native();e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> f(native,true);
  auto v=States();Same(native.linearize(v),f.linearize(v));
  for(auto key:{V(0),W(0),V(1),W(1)})for(int axis=0;axis<3;++axis) {
    auto before=e2_memo::counts[0].misses.load();
    auto x=v.at<gtsam::Vector3>(key);x[axis]=std::nextafter(x[axis],INFINITY);v.update(key,x);
    Same(native.linearize(v),f.linearize(v));EXPECT_EQ(e2_memo::counts[0].misses,before+1);
  }
  for(auto key:{X(0),X(1)})for(int axis=0;axis<6;++axis) {
    auto before=e2_memo::counts[0].misses.load();
    auto d=gtsam::Vector6::Zero().eval();d[axis]=1e-8;
    v.update(key,v.at<gtsam::Pose3>(key).retract(d));
    Same(native.linearize(v),f.linearize(v));EXPECT_EQ(e2_memo::counts[0].misses,before+1);
  }
}
// Deterministic synthetic failure paths are functions of Values, not mutation
// of factor parameters (which the production memo explicitly does not support).
class EdgeCaseFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  EdgeCaseFactor():NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(3,1),V(0)){}
  bool active(const gtsam::Values& v)const override{return v.at<gtsam::Vector3>(V(0))[0]!=-1.;}
  gtsam::Vector evaluateError(const gtsam::Vector3& x,boost::optional<gtsam::Matrix&> H=boost::none)const override {
    if(H)*H=gtsam::Matrix3::Identity();return x;
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& v)const override {
    double mode=v.at<gtsam::Vector3>(V(0))[0];
    if(mode==-2.)return {};
    if(mode==-3.)return boost::make_shared<gtsam::JacobianFactor>(V(0),
      gtsam::Matrix3::Constant(NAN),gtsam::Vector3::Zero());
    if(mode==-4.)throw std::runtime_error("after signature failure");
    if(mode==-5.)return boost::make_shared<gtsam::HessianFactor>(V(0),gtsam::Matrix3::Identity(),gtsam::Vector3::Zero(),0.);
    return NoiseModelFactor1::linearize(v);
  }
};
TEST(E2Audit, InactiveNullNonfiniteUnexpectedTypeAndFailureNeverCached) {
  auto native=EdgeCaseFactor();e2_memo::MemoFactor<EdgeCaseFactor,0> f(native,true);
  gtsam::Values v;v.insert(V(0),gtsam::Vector3(1,0,0));f.linearize(v);
  for(double mode:{-1.,-2.,-3.,-4.,-5.}) {
    v.update(V(0),gtsam::Vector3(mode,0,0));auto hits=e2_memo::counts[0].hits.load();
    for(int repeat=0;repeat<2;++repeat) {
      if(mode==-4.)EXPECT_THROW(f.linearize(v),std::runtime_error);
      else {auto out=f.linearize(v);if(mode==-1.||mode==-2.)EXPECT_FALSE(out);else EXPECT_TRUE(out);}
    }
    EXPECT_EQ(e2_memo::counts[0].hits,hits);
    v.update(V(0),gtsam::Vector3(1,0,0));Same(native.linearize(v),f.linearize(v));
    EXPECT_EQ(e2_memo::counts[0].hits,hits);
  }
}
TEST(E2Audit, RevisionMeanCovarianceAndReplacementHaveNoInheritedCache) {
  auto v=States();auto a=WnoaEndpoint::Between(0,1,.1,.023,Qc());
  auto b=WnoaEndpoint::Between(1,2,.1,.071,Qc());
  auto entries=e2_memo::counts[1].live_entries.load(),bytes=e2_memo::counts[1].live_bytes.load();
  for(int rev=0;rev<8;++rev) {
    gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(5,0)=.05*rev;l(2,1)=-.03*rev;
    auto measured=a.pose(v).between(b.pose(v)).compose(gtsam::Pose3(gtsam::Rot3::Rz(.001*rev),{.01*rev,0,0}));
    WnoaRelativeFactor native(a,b,measured,gtsam::noiseModel::Gaussian::Covariance(l*l.transpose()));
    e2_memo::MemoFactor<WnoaRelativeFactor,1> f(native,true);
    auto hits=e2_memo::counts[1].hits.load();Same(native.linearize(v),f.linearize(v));
    EXPECT_EQ(e2_memo::counts[1].hits,hits);Same(native.linearize(v),f.linearize(v));
    EXPECT_EQ(e2_memo::counts[1].hits,hits+1);
    auto returned=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(f.linearize(v));
    returned->getb().setConstant(999);Same(native.linearize(v),f.linearize(v));
  }
  EXPECT_EQ(e2_memo::counts[1].live_entries,entries);EXPECT_EQ(e2_memo::counts[1].live_bytes,bytes);
}
TEST(E2Audit, DISABLED_Microbench) {
  auto native=Native();auto v=States();
  e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> off(native,false),on(native,true);
  on.linearize(v);double sink=0.;
  for(int repeat=0;repeat<5;++repeat)for(int j=0;j<4;++j) {
    int mode=repeat%2?3-j:j;auto begin=std::chrono::steady_clock::now();
    for(int i=0;i<3000;++i) {
      if(mode==2)v.update(V(0),gtsam::Vector3(.6+1e-6*(i%2),.2,-.1));
      auto out=mode==0?native.linearize(v):mode==1?off.linearize(v):on.linearize(v);
      sink+=out->size();
    }
    std::cout<<"MICRO mode="<<mode<<" repeat="<<repeat<<" calls=3000 seconds="
      <<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<"\n";
  }
  EXPECT_GT(sink,0.);
}
}
