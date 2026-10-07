#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include "offline_process/StableGaussianCost.h"
#include "offline_process/FullHistoryPublicationRefinement.h"

namespace {
using namespace fgo::offline;
TEST(StableGaussianCost, CompensatedSummationRetainsSmallTerms) {
 StableSum s;s.add(1e20L);s.add(1);s.add(-1e20L);EXPECT_EQ(s.value(),1);
}
TEST(StableGaussianCost, ResidualDifferenceSurvivesLargeCommonCost) {
 gtsam::Vector a(3),b(3);a<<1e10,3.,4.;b<<1e10,2.,4.;
 EXPECT_EQ(a.squaredNorm(),b.squaredNorm());
 StableSum s;AccumulateResidualChange(a,b,s);EXPECT_EQ(s.value(),-2.5L);
 EXPECT_LT(s.arithmeticBound(),1e-12);
}
TEST(StableGaussianCost, ResolvedDescentAndConvergenceRequired) {
 StableCostChange c{-1e-10L,1e-20L,-5e-10L,1e-20L,10};
 EXPECT_TRUE(StableStationaryDescent(c,1.3e-5,1e-7,1e-5));
 EXPECT_FALSE(StableStationaryDescent(c,1.3e-5,1.1e-5,1e-5));
 EXPECT_FALSE(StableStationaryDescent(c,1e-8,1e-7,1e-5));
 c.delta=1e-30L;EXPECT_FALSE(StableStationaryDescent(c,1.,1e-7,1e-5));
 c.delta=-1e-30L;EXPECT_FALSE(StableStationaryDescent(c,1.,1e-7,1e-5));
 c.delta=-1e-10L;c.predicted_delta=1e-30L;EXPECT_FALSE(StableStationaryDescent(c,1.,1e-7,1e-5));
}
TEST(StableGaussianCost, InvalidValuesAndResidualsRejected) {
 StableCostChange c{-1.,1e-20,-1.,1e-20,3};
 EXPECT_FALSE(StableStationaryDescent(c,NAN,0.,1e-5));
 EXPECT_FALSE(StableStationaryDescent(c,1.,0.,0.));
 c.delta=NAN;EXPECT_FALSE(StableStationaryDescent(c,1.,0.,1e-5));
 StableSum s;EXPECT_THROW(s.add(INFINITY),std::runtime_error);
 EXPECT_THROW(AccumulateResidualChange(gtsam::Vector::Zero(2),gtsam::Vector::Zero(3),s),std::runtime_error);
}
TEST(StableGaussianCost, RealGaussianFactorsMatchAnalyticChange) {
 gtsam::NonlinearFactorGraph graph;
 graph.add(gtsam::PriorFactor<double>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,.5)));
 gtsam::Values a,b;a.insert(0,3.);b.insert(0,2.);
 const auto c=GaussianCostChange(graph,a,b);EXPECT_EQ(c.delta,-2.L);EXPECT_EQ(c.predicted_delta,-2.L);
 const auto r=GaussianCostChange(graph,b,a);EXPECT_EQ(r.delta,2.L);EXPECT_FALSE(StableStationaryDescent(r,1.,0.,1e-5));
}
TEST(StableGaussianCost, RobustNoiseIsNotSilentlyTreatedAsGaussian) {
 gtsam::NonlinearFactorGraph graph;
 auto noise=gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1),gtsam::noiseModel::Isotropic::Sigma(1,1));
 graph.add(gtsam::PriorFactor<double>(0,2.,noise));gtsam::Values a,b;a.insert(0,3.);b.insert(0,2.);
 EXPECT_THROW(GaussianCostChange(graph,a,b),std::runtime_error);
}
struct Fault {int calls{0},throw_after{-1};};
class ScalarFactor:public gtsam::NoiseModelFactor1<double> {
 bool cycle_;std::shared_ptr<Fault> fault_;
public:
 ScalarFactor(bool cycle=false,std::shared_ptr<Fault> fault=std::make_shared<Fault>())
 :NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1,1),0),cycle_(cycle),fault_(fault){}
 gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> H=boost::none)const override {
  if(H){if(fault_->throw_after>=0&&++fault_->calls>fault_->throw_after)throw std::runtime_error("fault");*H=gtsam::Matrix::Constant(1,1,cycle_?3*x*x-2:2*x);}
  return gtsam::Vector::Constant(1,cycle_?x*x*x-2*x+2:x*x-4);
 }
};
std::unique_ptr<gtsam::ISAM2> Solver(const gtsam::NonlinearFactor::shared_ptr& f,double x) {
 gtsam::ISAM2Params p;p.relinearizeThreshold=.01;p.relinearizeSkip=1;p.factorization=gtsam::ISAM2Params::QR;
 auto s=std::make_unique<gtsam::ISAM2>(p);gtsam::NonlinearFactorGraph g;g.push_back(f);gtsam::Values v;v.insert(0,x);s->update(g,v);return s;
}
TEST(StableGaussianCost, StationaryNoMutation) {
 auto s=Solver(boost::make_shared<gtsam::PriorFactor<double>>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,1)),3.);
 auto* p=s.get();auto r=numerical_audit::Refine(s);EXPECT_EQ(r.status,"stationary");EXPECT_EQ(r.attempted,0);EXPECT_EQ(p,s.get());
}
TEST(StableGaussianCost, NonlinearConvergenceAndFactorIdentity) {
 auto f=boost::make_shared<ScalarFactor>();auto s=Solver(f,3.);auto r=numerical_audit::Refine(s);
 EXPECT_EQ(r.status,"stationary");EXPECT_LE(r.gradient_after,1e-5);EXPECT_GT(r.accepted,0);EXPECT_EQ(s->getFactorsUnsafe()[0],f);
}
TEST(StableGaussianCost, BudgetIsNotConvergence) {
 auto s=Solver(boost::make_shared<ScalarFactor>(),3.);auto r=numerical_audit::Refine(s,1,1e-12);
 EXPECT_EQ(r.status,"budget_exhausted");EXPECT_GT(r.gradient_after,1e-12);
}
TEST(StableGaussianCost, WorseningStepRollsBack) {
 auto f=boost::make_shared<ScalarFactor>(true);auto s=Solver(f,0.);auto* p=s.get();auto x=s->calculateEstimate<double>(0);
 auto r=numerical_audit::Refine(s);EXPECT_EQ(r.status,"rejected_objective");EXPECT_EQ(r.accepted,0);EXPECT_EQ(p,s.get());
 EXPECT_DOUBLE_EQ(x,s->calculateEstimate<double>(0));EXPECT_EQ(s->getFactorsUnsafe()[0],f);
}
TEST(StableGaussianCost, ExceptionRollsBack) {
 auto fault=std::make_shared<Fault>();auto f=boost::make_shared<ScalarFactor>(false,fault);auto s=Solver(f,3.);
 auto* p=s.get();auto x=s->calculateEstimate<double>(0);fault->throw_after=1;
 auto r=numerical_audit::Refine(s);EXPECT_EQ(r.status,"rejected_exception");EXPECT_EQ(r.accepted,0);EXPECT_EQ(p,s.get());EXPECT_DOUBLE_EQ(x,s->calculateEstimate<double>(0));
}
}
