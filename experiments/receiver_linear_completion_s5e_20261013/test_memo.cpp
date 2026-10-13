#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <future>
#include "SourceWnoaModel.h"
#include "ExactLinearizationMemo.h"
#include "FullPublicationRefinement.h"
using namespace fgo::offline;
namespace {
gtsam::Values States() {
  gtsam::Values v;SourceWnoaState s;
  s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.2,-.3,.4),{1,2,3});
  s.velocity={.6,.2,-.1};s.omega={.1,.05,.2};
  for(size_t i=0;i<3;++i){InsertSourceWnoaState(v,i,s);s=PredictSourceWnoa(s,.1);}return v;
}
auto Qc() {return gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());}
auto Native() {return fgo::factor::GPWNOAPrior(X(0),V(0),W(0),X(1),V(1),W(1),.1,Qc(),true,true);}
void Same(const gtsam::GaussianFactor::shared_ptr& a,const gtsam::GaussianFactor::shared_ptr& b) {
  const auto x=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(a),y=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(b);
  ASSERT_TRUE(x&&y);EXPECT_TRUE((x->augmentedJacobian().array()==y->augmentedJacobian().array()).all());
}
TEST(E2Memo, ExactHitSmallChangeMissIrrelevantChangeHit) {
  auto native=Native();e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> f(native,true);
  auto v=States();auto& c=e2_memo::counts[0];auto hits=c.hits.load(),misses=c.misses.load();
  Same(native.linearize(v),f.linearize(v));Same(native.linearize(v),f.linearize(v));
  EXPECT_EQ(c.hits,hits+1);EXPECT_EQ(c.misses,misses+1);
  v.update(V(2),gtsam::Vector3(9,8,7));Same(native.linearize(v),f.linearize(v));EXPECT_EQ(c.hits,hits+2);
  for(auto key:{V(0),W(0)}) {
    auto a=v.at<gtsam::Vector3>(key);a[0]=std::nextafter(a[0],INFINITY);v.update(key,a);
    Same(native.linearize(v),f.linearize(v));
  }
  auto p=v.at<gtsam::Pose3>(X(0));auto t=p.translation();t[0]=std::nextafter(t[0],INFINITY);
  v.update(X(0),gtsam::Pose3(p.rotation(),t));Same(native.linearize(v),f.linearize(v));
  p=v.at<gtsam::Pose3>(X(0));v.update(X(0),p.compose(gtsam::Pose3(gtsam::Rot3::Rz(1e-15),{0,0,0})));
  Same(native.linearize(v),f.linearize(v));EXPECT_EQ(c.misses,misses+5);
}
TEST(E2Memo, CallerMutationAndCloningCannotCorruptStoredGaussian) {
  auto native=Native();e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> f(native,true);auto v=States();
  auto returned=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(f.linearize(v));returned->getb().setZero();
  Same(native.linearize(v),f.linearize(v));auto hits=e2_memo::counts[0].hits.load();
  auto cloned=f.clone();Same(native.linearize(v),cloned->linearize(v));EXPECT_EQ(e2_memo::counts[0].hits,hits);
  auto noise=gtsam::noiseModel::Isotropic::Sigma(12,2.);
  auto changed=f.cloneWithNewNoiseModel(noise),reference=native.cloneWithNewNoiseModel(noise);
  Same(reference->linearize(v),changed->linearize(v));EXPECT_EQ(e2_memo::counts[0].hits,hits);
}
TEST(E2Memo, DefaultOffAndExceptionDiscardEntry) {
  auto native=Native();auto v=States();e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> off(native,false);
  auto calls=e2_memo::counts[0].calls.load();Same(native.linearize(v),off.linearize(v));EXPECT_EQ(e2_memo::counts[0].calls,calls);
  e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> f(native,true);f.linearize(v);
  auto missing=v;missing.erase(W(0));EXPECT_ANY_THROW(f.linearize(missing));
  auto hits=e2_memo::counts[0].hits.load();Same(native.linearize(v),f.linearize(v));EXPECT_EQ(e2_memo::counts[0].hits,hits);
}
TEST(E2Memo, DenseInterpolatedFactorAndFullJointCovariance) {
  auto v=States();auto a=WnoaEndpoint::Between(0,1,.1,.023,Qc()),b=WnoaEndpoint::Between(1,2,.1,.071,Qc());
  gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.3;l(2,1)=-.2;
  WnoaRelativeFactor native(a,b,a.pose(v).between(b.pose(v)),gtsam::noiseModel::Gaussian::Covariance(l*l.transpose()));
  auto factor=boost::make_shared<e2_memo::MemoFactor<WnoaRelativeFactor,1>>(native,true);
  Same(native.linearize(v),factor->linearize(v));Same(native.linearize(v),factor->linearize(v));
  gtsam::NonlinearFactorGraph reference;
  reference.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0),v.at<gtsam::Pose3>(X(0)),gtsam::noiseModel::Isotropic::Sigma(6,.1));
  reference.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0),v.at<gtsam::Vector3>(V(0)),gtsam::noiseModel::Isotropic::Sigma(3,.2));
  reference.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(0),v.at<gtsam::Vector3>(W(0)),gtsam::noiseModel::Isotropic::Sigma(3,.3));
  for(size_t i=1;i<3;++i)reference.emplace_shared<fgo::factor::GPWNOAPrior>(X(i-1),V(i-1),W(i-1),X(i),V(i),W(i),.1,Qc(),true,true);
  reference.push_back(boost::make_shared<WnoaRelativeFactor>(native));reference.push_back(gtsam::NonlinearFactor::shared_ptr());
  auto candidate=reference;candidate[5]=factor;
  for(size_t i=0;i<2;++i)candidate[3+i]=boost::make_shared<e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0>>(*boost::dynamic_pointer_cast<fgo::factor::GPWNOAPrior>(reference[3+i]),true);
  const auto H=reference.linearize(v)->augmentedHessian(),C=candidate.linearize(v)->augmentedHessian();
  EXPECT_TRUE((H.array()==C.array()).all());
  EXPECT_EQ(reference.error(v),candidate.error(v));EXPECT_EQ(NormalizedGradient(reference,v),NormalizedGradient(candidate,v));
  gtsam::Marginals mr(reference,v),mc(candidate,v);
  auto cr=mr.jointMarginalCovariance(v.keys()).fullMatrix(),cc=mc.jointMarginalCovariance(v.keys()).fullMatrix();
  EXPECT_TRUE((cr.array()==cc.array()).all());
}
class NonlinearVector : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  explicit NonlinearVector(bool cycle):NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(3,1),V(0)),cycle_(cycle){}
  gtsam::Vector evaluateError(const gtsam::Vector3& x,boost::optional<gtsam::Matrix&> H=boost::none)const override {
    if(H){*H=gtsam::Matrix3::Identity();(*H)(0,0)=cycle_?3*x[0]*x[0]-2:2*x[0];}
    return (gtsam::Vector3()<<(cycle_?x[0]*x[0]*x[0]-2*x[0]+2:x[0]*x[0]-4),x[1],x[2]).finished();
  }
 private:bool cycle_;
};
TEST(E2Memo, RefinementDecisionsAndTransactionalRejectionPreserved) {
  for(bool cycle:{false,true}) {
    auto make=[&](bool enabled) {
      gtsam::ISAM2Params p;p.relinearizeSkip=1;p.relinearizeThreshold=.01;p.factorization=gtsam::ISAM2Params::QR;
      auto s=std::make_unique<gtsam::ISAM2>(p);gtsam::NonlinearFactorGraph g;
      g.push_back(boost::make_shared<e2_memo::MemoFactor<NonlinearVector,0>>(NonlinearVector(cycle),enabled));
      gtsam::Values v;v.insert(V(0),gtsam::Vector3(cycle?0.:3.,0,0));s->update(g,v);return s;
    };
    auto reference=make(false),candidate=make(true);auto* pointer=candidate.get();auto before=candidate->calculateEstimate();
    auto r=numerical_audit::Refine(reference),c=numerical_audit::Refine(candidate);
    EXPECT_EQ(r.status,c.status);EXPECT_EQ(r.attempted,c.attempted);EXPECT_EQ(r.accepted,c.accepted);
    EXPECT_EQ(r.error_after,c.error_after);EXPECT_EQ(r.gradient_after,c.gradient_after);
    EXPECT_TRUE(reference->calculateEstimate().equals(candidate->calculateEstimate(),0));
    if(cycle){EXPECT_EQ(c.status,"rejected_objective");EXPECT_EQ(pointer,candidate.get());EXPECT_TRUE(before.equals(candidate->calculateEstimate(),0));}
  }
}
TEST(E2Memo, ConcurrentIndependentInputsCannotReturnStaleGaussian) {
  auto native=Native();e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0> f(native,true);
  std::vector<std::future<bool>> jobs;
  for(int i=0;i<4;++i)jobs.push_back(std::async(std::launch::async,[&,i] {
    auto v=States();v.update(V(0),gtsam::Vector3(.1*i,.2,.3));
    auto expected=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(native.linearize(v))->augmentedJacobian();
    for(int j=0;j<20;++j) {
      auto got=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(f.linearize(v))->augmentedJacobian();
      if(!(expected.array()==got.array()).all())return false;
    }return true;
  }));
  for(auto& job:jobs)EXPECT_TRUE(job.get());
}
}
