#include <gtest/gtest.h>
#include <gtsam/slam/BetweenFactor.h>
#include "ExactLinearizationMemo.h"
#include "LinearCompletionObserver.h"

namespace {
struct Inspectable: gtsam::ISAM2 {
  using gtsam::ISAM2::ISAM2;
  const auto& linear()const{return linearFactors_;}
};
struct CountSquare: gtsam::NoiseModelFactor1<double> {
  static inline int evaluations=0;
  CountSquare():gtsam::NoiseModelFactor1<double>(gtsam::noiseModel::Isotropic::Sigma(1,1),0){}
  gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> h=boost::none)const override {
    ++evaluations;if(h)*h=gtsam::Matrix::Constant(1,1,2*x);
    return gtsam::Vector1(x*x-4);
  }
};
TEST(S5e, FullBacksubDoesNotEvaluateNonlinearFactorsAndNeedNotBeStationary) {
  Inspectable input;gtsam::NonlinearFactorGraph graph;graph.emplace_shared<CountSquare>();
  gtsam::Values values;values.insert(0,1.);input.update(graph,values);
  const auto before=input.calculateEstimate();const auto theta=input.getLinearizationPoint();
  const auto delta=input.getDelta();const int calls=CountSquare::evaluations;
  auto shadow=input;const auto full=shadow.calculateBestEstimate();
  EXPECT_EQ(calls,CountSquare::evaluations);
  EXPECT_NEAR(full.at<double>(0),2.5,1e-12);
  EXPECT_EQ(input.calculateEstimate().localCoordinates(before).norm(),0.);
  EXPECT_EQ(input.getLinearizationPoint().localCoordinates(theta).norm(),0.);EXPECT_TRUE(input.getDelta().equals(delta,0));
  EXPECT_GT(s5e::Verify(graph,full).gradient,1.); // correct linear solve is not nonlinear convergence
  EXPECT_TRUE(full.equals(theta.retract(input.linear().optimize()),1e-12));
}
TEST(S5e, WildfireIncompleteDeltaComparedWithIndependentGaussianQR) {
  gtsam::ISAM2Params params;params.factorization=gtsam::ISAM2Params::QR;
  params.optimizationParams=gtsam::ISAM2GaussNewtonParams(100.);params.enableRelinearization=false;
  Inspectable input(params);gtsam::Values values;gtsam::NonlinearFactorGraph graph;
  auto noise=gtsam::noiseModel::Isotropic::Sigma(1,1.);
  graph.emplace_shared<gtsam::PriorFactor<double>>(0,0.,noise);
  for(size_t i=0;i<8;++i){values.insert(i,double(i));if(i)graph.emplace_shared<gtsam::BetweenFactor<double>>(i-1,i,1.,noise);}
  input.update(graph,values);input.calculateEstimate();
  gtsam::NonlinearFactorGraph changed;changed.emplace_shared<gtsam::PriorFactor<double>>(7,7.1,noise);
  input.update(changed,{});const auto before=input.calculateEstimate();const auto delta=input.getDelta();
  const auto theta=input.getLinearizationPoint();const auto expected=theta.retract(input.linear().optimize());
  auto shadow=input;const auto full=shadow.calculateBestEstimate();
  EXPECT_GT(before.localCoordinates(full).norm(),1e-5);
  EXPECT_TRUE(full.equals(expected,1e-12));EXPECT_EQ(shadow.getLinearizationPoint().localCoordinates(theta).norm(),0.);
  EXPECT_EQ(input.calculateEstimate().localCoordinates(before).norm(),0.);EXPECT_TRUE(input.getDelta().equals(delta,0));
  auto repeat=input;EXPECT_EQ(repeat.calculateBestEstimate().localCoordinates(full).norm(),0.);
  for(size_t i=0;i<input.getFactorsUnsafe().size();++i)
    EXPECT_EQ(input.getFactorsUnsafe()[i],shadow.getFactorsUnsafe()[i]);
}
TEST(S5e, UncachedClonesPreserveDenseNoiseResidualsJacobiansAndSlots) {
  using namespace fgo::offline;
  const auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  gtsam::Values values;
  for(size_t i=0;i<3;++i){SourceWnoaState s;
    s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.03*i,.05*i,.1*i),{.2*i,-.1*i,.04*i});
    s.velocity={.3,-.1,.2};s.omega={.1,.2,.03};InsertSourceWnoaState(values,i,s);}
  auto cache=std::make_shared<e2b::Cache>();
  auto a=WnoaEndpoint::Between(0,1,.1,.023,qc).withCache(cache);
  auto b=WnoaEndpoint::Between(1,2,.1,.071,qc).withCache(cache);
  gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.2;l(2,1)=-.1;
  auto noise=gtsam::noiseModel::Gaussian::Covariance(.01*l*l.transpose(),false);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<e2_memo::MemoFactor<WnoaRelativeFactor,1>>(WnoaRelativeFactor(a,b,gtsam::Pose3(),noise),true);
  graph.emplace_shared<e2_memo::MemoFactor<fgo::factor::GPWNOAPrior,0>>(
    fgo::factor::GPWNOAPrior(X(0),V(0),W(0),X(1),V(1),W(1),.1,qc,true),true);
  graph.push_back(gtsam::NonlinearFactor::shared_ptr());
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0),gtsam::Pose3(),noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0),gtsam::Vector3::Zero().eval(),gtsam::noiseModel::Isotropic::Sigma(3,10));
  const auto original=s5e::Verify(graph,values);const auto calls=cache->counts().calls;
  const auto memo0=e2_memo::counts[0].calls.load(),memo1=e2_memo::counts[1].calls.load();
  const auto clone=s5e::Uncached(graph);const auto result=s5e::Verify(clone,values);
  EXPECT_DOUBLE_EQ(original.error,result.error);EXPECT_DOUBLE_EQ(original.gradient,result.gradient);
  EXPECT_TRUE(original.g.equals(result.g,0));EXPECT_TRUE(original.d.equals(result.d,0));
  EXPECT_EQ(cache->counts().calls,calls);EXPECT_EQ(e2_memo::counts[0].calls.load(),memo0);EXPECT_EQ(e2_memo::counts[1].calls.load(),memo1);
  EXPECT_EQ(graph.size(),clone.size());
  for(size_t i=0;i<graph.size();++i){if(!graph[i]){EXPECT_FALSE(clone[i]);continue;}
    const auto* x=dynamic_cast<const gtsam::NoiseModelFactor*>(graph[i].get());
    const auto* y=dynamic_cast<const gtsam::NoiseModelFactor*>(clone[i].get());
    EXPECT_EQ(x->noiseModel(),y->noiseModel());EXPECT_EQ(x->keys(),y->keys());
    auto ja=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(x->linearize(values));
    auto jb=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(y->linearize(values));
    EXPECT_EQ((ja->augmentedJacobian()-jb->augmentedJacobian()).norm(),0.);
  }
  auto bad=values;bad.erase(V(0));EXPECT_ANY_THROW(s5e::Verify(clone,bad));
  EXPECT_DOUBLE_EQ(original.error,s5e::Verify(clone,values).error);
}
TEST(S5e, UnknownFactorsFailClosedAndSelectionIsExplicit) {
  gtsam::NonlinearFactorGraph graph;graph.emplace_shared<CountSquare>();
  EXPECT_THROW(s5e::Uncached(graph),std::runtime_error);
  unsetenv("S5E_INDICES");EXPECT_FALSE(s5e::Selected(10,"support"));
  setenv("S5E_INDICES","10,20",1);EXPECT_TRUE(s5e::Selected(20,"support"));
  EXPECT_FALSE(s5e::Selected(20,"final"));EXPECT_FALSE(s5e::Selected(21,"support"));unsetenv("S5E_INDICES");
}
} // namespace
