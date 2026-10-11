#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "offline_process/BoundedISAM2Refinement.h"
#include "offline_process/StationaryQRConsensus.h"
#include "CachedGaussianEvaluation.h"
#include "SourceWnoaModel.h"

using namespace publication_cache;
using namespace fgo::offline;
static void Same(const StableCostChange& a,const StableCostChange& b) {
  EXPECT_EQ(a.delta,b.delta);EXPECT_EQ(a.arithmetic_bound,b.arithmetic_bound);
  EXPECT_EQ(a.predicted_delta,b.predicted_delta);EXPECT_EQ(a.predicted_bound,b.predicted_bound);
  EXPECT_EQ(a.residual_components,b.residual_components);
}
TEST(Cache, DensePoseSharedKeysNullSlotsAndRepeatedEvaluation) {
  gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.3;l(2,1)=-.2;
  auto noise=gtsam::noiseModel::Gaussian::Covariance(l*l.transpose());
  gtsam::NonlinearFactorGraph g;
  g.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0,gtsam::Pose3(),noise);
  g.push_back(gtsam::NonlinearFactor::shared_ptr());
  g.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(0,1,gtsam::Pose3(),noise);
  gtsam::Values a,b,c;a.insert(0,gtsam::Pose3());a.insert(1,gtsam::Pose3(gtsam::Rot3::RzRyRx(.1,.2,.3),{1,2,3}));
  auto d=a.zeroVectors();d.at(1)<<.03,.01,-.02,.4,-.1,.2;b=a.retract(d);c=b.retract(d);
  Evaluation anchor(g,a),seed(g,b),target(g,c);
  const auto expected=CumulativeCostWithLocalPrediction(GaussianCostChange(g,a,c),GaussianCostChange(g,b,c));
  Same(expected,CumulativeChange(anchor,seed,target));
  Same(expected,CumulativeChange(anchor,seed,target));
  EXPECT_EQ(anchor.linearizations,0);EXPECT_EQ(seed.linearizations,1);
  EXPECT_EQ(anchor.residual_evaluations,2);EXPECT_EQ(target.residual_evaluations,2);
  EXPECT_DOUBLE_EQ(target.gradient(),NormalizedGradient(g,c));
  EXPECT_EQ(target.linearizations,1);
}
TEST(Cache, ActualWnoaPriorAndInterpolatedSourceFactor) {
  const auto qc=gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
  gtsam::Values a;
  SourceWnoaState s;s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.2,-.3,.4),{1.,2.,3.});s.velocity={.6,.2,-.1};s.omega={.1,.05,.2};
  for(size_t i=0;i<3;++i){InsertSourceWnoaState(a,i,s);s=PredictSourceWnoa(s,.1);}
  gtsam::NonlinearFactorGraph g;
  for(size_t i=1;i<3;++i)g.emplace_shared<fgo::factor::GPWNOAPrior>(X(i-1),V(i-1),W(i-1),X(i),V(i),W(i),.1,qc,true,true);
  auto e1=WnoaEndpoint::Between(0,1,.1,.03,qc),e2=WnoaEndpoint::Between(1,2,.1,.07,qc);
  g.emplace_shared<WnoaRelativeFactor>(e1,e2,e1.pose(a).between(e2.pose(a)),gtsam::noiseModel::Isotropic::Sigma(6,.02));
  auto d=a.zeroVectors();d.at(X(1))<<.01,-.02,.005,.03,.01,-.02;auto b=a.retract(d),c=b.retract(d);
  Evaluation ea(g,a),eb(g,b),ec(g,c);
  Same(CumulativeCostWithLocalPrediction(GaussianCostChange(g,a,c),GaussianCostChange(g,b,c)),CumulativeChange(ea,eb,ec));
  EXPECT_DOUBLE_EQ(ec.gradient(),NormalizedGradient(g,c));
}
TEST(Cache, OwnsValuesAndRejectsDifferentFactorVersions) {
  gtsam::NonlinearFactorGraph g;g.emplace_shared<gtsam::PriorFactor<double>>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,.5));
  gtsam::Values a,b;a.insert(0,3.);b.insert(0,2.);Evaluation ea(g,a),eb(g,b);
  a.update(0,100.);EXPECT_EQ(ea.values().at<double>(0),3.);
  EXPECT_EQ(ActualChange(ea,eb).delta,-2.L);
  g[0]=boost::make_shared<gtsam::PriorFactor<double>>(0,5.,gtsam::noiseModel::Isotropic::Sigma(1,.5));
  Evaluation changed(g,b);EXPECT_THROW(CumulativeChange(ea,ea,changed),std::runtime_error);
}
TEST(Cache, RejectsRobustAndNonfiniteResiduals) {
  gtsam::Values a,b;a.insert(0,3.);b.insert(0,2.);
  auto robust=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1),gtsam::noiseModel::Isotropic::Sigma(1,1));
  gtsam::NonlinearFactorGraph g;g.emplace_shared<gtsam::PriorFactor<double>>(0,2.,robust);
  Evaluation ea(g,a),eb(g,b);EXPECT_THROW(CumulativeChange(ea,ea,eb),std::runtime_error);
  g[0]=boost::make_shared<gtsam::PriorFactor<double>>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,1));
  b.update(0,std::numeric_limits<double>::quiet_NaN());Evaluation ga(g,a),bad(g,b);EXPECT_THROW(ActualChange(ga,bad),std::runtime_error);
}
