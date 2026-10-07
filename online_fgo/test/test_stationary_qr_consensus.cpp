#include <gtest/gtest.h>
#include "offline_process/StationaryQRConsensus.h"
#include "offline_process/FullHistoryPublicationRefinement.h"
namespace {
using namespace fgo::offline;
TEST(StationaryQRConsensus, TinyAmbiguousCostRequiresIndependentStationarity) {
 StableCostChange a{1.6e-10,1e-17,-3.2e-10,1e-17,24000},b{2.7e-10,1e-17,-3.2e-10,1e-17,24000};
 auto accepts=[&](double ga,double gb,double d){return StationaryQRConsensus(a,b,310832,1.4e-5,ga,gb,d,1e-5);};
 EXPECT_TRUE(accepts(1e-7,7e-8,1e-13));
 EXPECT_FALSE(accepts(1.1e-5,7e-8,1e-13));
 EXPECT_FALSE(accepts(1e-7,1.1e-5,1e-13));
 EXPECT_FALSE(accepts(1e-7,7e-8,1e-6));
 EXPECT_FALSE(accepts(NAN,7e-8,1e-13));
 a.delta=1e-4;EXPECT_FALSE(accepts(1e-7,7e-8,1e-13));
 a.delta=1e-10;a.predicted_delta=1e-15;EXPECT_FALSE(accepts(1e-7,7e-8,1e-13));
 a.predicted_delta=-1e-4;EXPECT_FALSE(accepts(1e-7,7e-8,1e-13));
}
TEST(StationaryQRConsensus, ComparisonUsesAllStateComponentsAndRejectsInvalidValues) {
 gtsam::Values old,a,b;old.insert(0,1.);a.insert(0,1.+1e-7);b.insert(0,1.+1e-7+1e-13);
 EXPECT_LT(QRDisagreement(old,a,b),2e-13);
 b.update(0,1.1);EXPECT_GT(QRDisagreement(old,a,b),.01);
 b.update(0,std::numeric_limits<double>::quiet_NaN());EXPECT_THROW(QRDisagreement(old,a,b),std::runtime_error);
}
TEST(StationaryQRConsensus, LaterStepsUseLocalPredictionButCumulativeActualCost) {
 StableCostChange cumulative{1e-9,1e-16,2e-10,1e-16,1000};
 StableCostChange local{-2e-10,2e-16,-4e-10,3e-16,1000};
 const auto c=CumulativeCostWithLocalPrediction(cumulative,local);
 EXPECT_EQ(c.delta,cumulative.delta);EXPECT_EQ(c.arithmetic_bound,cumulative.arithmetic_bound);
 EXPECT_EQ(c.predicted_delta,local.predicted_delta);EXPECT_EQ(c.predicted_bound,local.predicted_bound);
 EXPECT_EQ(c.residual_components,cumulative.residual_components);
 EXPECT_TRUE(StationaryQRConsensus(c,c,667884,1.15e-5,9e-6,9e-6,0,1e-5));
 cumulative.delta=1e-3;
 const auto too_far=CumulativeCostWithLocalPrediction(cumulative,local);
 EXPECT_FALSE(StationaryQRConsensus(too_far,too_far,667884,1.15e-5,9e-6,9e-6,0,1e-5));
}
class FlatCycleFactor:public gtsam::NoiseModelFactor1<double> {
 public:
 FlatCycleFactor():NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2,1),0){}
 gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> h=boost::none)const override {
  if(h){h->resize(2,1);*h<<0.,3*x*x-2.;}
  return (gtsam::Vector2()<<1e6,x*x*x-2*x+2).finished();
 }
};
std::unique_ptr<gtsam::ISAM2> CycleSolver(){
 gtsam::ISAM2Params p;p.factorization=gtsam::ISAM2Params::QR;p.relinearizeSkip=1;
 auto s=std::make_unique<gtsam::ISAM2>(p);gtsam::NonlinearFactorGraph g;
 g.push_back(boost::make_shared<FlatCycleFactor>());gtsam::Values v;v.insert(0,0.);s->update(g,v);return s;
}
TEST(StationaryQRConsensus, PrivatePolishCannotPublishNonstationaryCycle){
 auto s=CycleSolver();auto* identity=s.get();auto state=s->calculateEstimate();auto factor=s->getFactorsUnsafe()[0];
 auto r=numerical_audit::Refine(s,4);
 EXPECT_NE(r.status,"stationary");EXPECT_EQ(identity,s.get());EXPECT_EQ(factor,s->getFactorsUnsafe()[0]);
 EXPECT_EQ(state.localCoordinates(s->calculateEstimate()).norm(),0.);EXPECT_LE(r.attempted,4);
 EXPECT_GE(r.steps.size(),4);
}
TEST(StationaryQRConsensus, PrivatePolishBudgetDoesNotOverrideConvergence){
 auto s=CycleSolver();auto* identity=s.get();const auto state=s->calculateEstimate();
 auto r=numerical_audit::Refine(s,1);
 EXPECT_EQ(r.status,"budget_exhausted");EXPECT_EQ(r.attempted,1);EXPECT_EQ(identity,s.get());
 EXPECT_EQ(state.localCoordinates(s->calculateEstimate()).norm(),0.);
}
class FlatCubicFactor:public gtsam::NoiseModelFactor1<double> {
 public:
 FlatCubicFactor():NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2,1),0){}
 gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> h=boost::none)const override {
  if(h){h->resize(2,1);*h<<0.,3*x*x;}
  return (gtsam::Vector2()<<1e11,x*x*x-1).finished();
 }
};
TEST(StationaryQRConsensus, PrivatePolishCommitsOnlyFinalConvergedSolver){
 // The initial solve moves -.75 to .09259. The next GN step overshoots,
 // increasing cost within the relative tolerance. Private iterations converge
 // to the actual root without exposing those intermediate bad estimates.
 gtsam::ISAM2Params p;p.factorization=gtsam::ISAM2Params::QR;p.relinearizeSkip=1;
 auto s=std::make_unique<gtsam::ISAM2>(p);gtsam::NonlinearFactorGraph graph;
 graph.push_back(boost::make_shared<FlatCubicFactor>());
 gtsam::Values seed;seed.insert(0,-.75);s->update(graph,seed);
 auto* original=s.get();auto factor=s->getFactorsUnsafe()[0];
 auto r=numerical_audit::Refine(s,20);
 ASSERT_EQ(r.status,"stationary");EXPECT_NE(original,s.get());
 EXPECT_EQ(factor,s->getFactorsUnsafe()[0]);EXPECT_LE(r.attempted,20);
 EXPECT_LE(r.gradient_after,1e-5);EXPECT_NEAR(s->calculateEstimate().at<double>(0),1.,1e-5);
 EXPECT_EQ(r.accepted,1);EXPECT_GT(r.attempted,2);
 size_t private_candidates=0;
 for(const auto& step:r.steps)if(step.recovery)++private_candidates;
 EXPECT_GT(private_candidates,1);EXPECT_TRUE(r.steps.back().accepted);
}
}
