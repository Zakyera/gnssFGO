#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include "offline_process/PublicationRoundoffAcceptance.h"
#include "FullPublicationRefinement.h"

namespace {
using namespace fgo::offline;
using Decision = PublicationStepDecision;
TEST(PublicationRoundoff, UnchangedNonIncreasingPath) {
  EXPECT_EQ(CheckPublicationStep(100.,99.,2.,3.,1e-5),Decision::NonIncreasing);
  EXPECT_EQ(CheckPublicationStep(100.,100.,2.,2.,1e-5),Decision::NonIncreasing);
  // Non-increase may be an intermediate iteration, not necessarily a publication.
}
TEST(PublicationRoundoff, TwoUlpIncreaseRequiresActualConvergence) {
  const double e=170127.49697813028;
  const double up=std::nextafter(std::nextafter(e,INFINITY),INFINITY);
  EXPECT_EQ(CheckPublicationStep(e,up,1.1e-5,1e-7,1e-5),Decision::RoundoffStationary);
  EXPECT_EQ(CheckPublicationStep(e,up,1.1e-5,1.01e-5,1e-5),Decision::Reject);
  EXPECT_EQ(CheckPublicationStep(e,up,1e-7,2e-7,1e-5),Decision::Reject);
  EXPECT_EQ(CheckPublicationStep(e,up,1e-7,1e-7,1e-5),Decision::Reject);
}
TEST(PublicationRoundoff, SignificantIncreasesRejectedAcrossScales) {
  for(double e:{0.,1e-20,1.,1e5,1e15,1e300}) {
    const double delta=32.*std::numeric_limits<double>::epsilon()*std::max(1.,e);
    EXPECT_EQ(CheckPublicationStep(e,e+delta,1.,0.,1e-5),Decision::Reject);
  }
  EXPECT_EQ(CheckPublicationStep(170127.,170127.000001,1.,0.,1e-5),Decision::Reject);
}
TEST(PublicationRoundoff, NonfiniteNegativeAndInvalidInputsRejected) {
  for(double invalid:std::initializer_list<double>{NAN,INFINITY,-INFINITY,-1.}) {
    EXPECT_EQ(CheckPublicationStep(invalid,1.,1.,0.,1e-5),Decision::Reject);
    EXPECT_EQ(CheckPublicationStep(1.,invalid,1.,0.,1e-5),Decision::Reject);
    EXPECT_EQ(CheckPublicationStep(1.,.9,invalid,0.,1e-5),Decision::Reject);
    EXPECT_EQ(CheckPublicationStep(1.,.9,1.,invalid,1e-5),Decision::Reject);
    EXPECT_EQ(CheckPublicationStep(1.,.9,1.,0.,invalid),Decision::Reject);
  }
  EXPECT_EQ(CheckPublicationStep(1.,.9,1.,0.,0.),Decision::Reject);
}
struct Fault { int calls{0}; int throw_after{-1}; };
class ScalarFactor : public gtsam::NoiseModelFactor1<double> {
  bool cycle_; std::shared_ptr<Fault> fault_;
 public:
  ScalarFactor(bool cycle=false,std::shared_ptr<Fault> fault=std::make_shared<Fault>())
    :NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1,1),0),cycle_(cycle),fault_(fault){}
  gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> H=boost::none)const override {
    if(H) {
      if(fault_->throw_after>=0 && ++fault_->calls>fault_->throw_after)throw std::runtime_error("synthetic candidate failure");
      *H=gtsam::Matrix::Constant(1,1,cycle_?3*x*x-2:2*x);
    }
    return gtsam::Vector::Constant(1,cycle_?x*x*x-2*x+2:x*x-4);
  }
};
std::unique_ptr<gtsam::ISAM2> Solver() {
  gtsam::ISAM2Params p;p.relinearizeThreshold=.01;p.relinearizeSkip=1;p.factorization=gtsam::ISAM2Params::QR;
  return std::make_unique<gtsam::ISAM2>(p);
}
void Start(gtsam::ISAM2& s,const gtsam::NonlinearFactor::shared_ptr& f,double x) {
  gtsam::NonlinearFactorGraph graph;graph.push_back(f);gtsam::Values v;v.insert(0,x);s.update(graph,v);
}
TEST(PublicationRoundoff, LinearStationaryDoesNotMutate) {
  auto s=Solver();Start(*s,boost::make_shared<gtsam::PriorFactor<double>>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,1)),3.);
  const auto* before=s.get();const auto r=numerical_audit::Refine(s);
  EXPECT_EQ(r.status,"stationary");EXPECT_EQ(r.attempted,0);EXPECT_EQ(s.get(),before);
}
TEST(PublicationRoundoff, NonlinearConvergenceAndStableFactorIdentity) {
  auto s=Solver();auto f=boost::make_shared<ScalarFactor>();Start(*s,f,3.);
  const auto r=numerical_audit::Refine(s);EXPECT_EQ(r.status,"stationary");
  EXPECT_LE(r.gradient_after,1e-5);EXPECT_LE(r.attempted,20);EXPECT_GT(r.accepted,0);
  EXPECT_EQ(s->getFactorsUnsafe()[0],f);EXPECT_NEAR(s->calculateEstimate<double>(0),2.,1e-5);
}
TEST(PublicationRoundoff, BudgetNotMislabelledAsConverged) {
  auto s=Solver();Start(*s,boost::make_shared<ScalarFactor>(),3.);
  const auto r=numerical_audit::Refine(s,1,1e-12);
  EXPECT_EQ(r.status,"budget_exhausted");EXPECT_EQ(r.attempted,1);EXPECT_GT(r.gradient_after,1e-12);
}
TEST(PublicationRoundoff, TrulyWorseningStepRollsBack) {
  auto s=Solver();auto f=boost::make_shared<ScalarFactor>(true);Start(*s,f,0.);
  const auto* pointer=s.get();const double x=s->calculateEstimate<double>(0);
  const auto r=numerical_audit::Refine(s);
  EXPECT_EQ(r.status,"rejected_objective");EXPECT_EQ(r.accepted,0);EXPECT_EQ(s.get(),pointer);
  EXPECT_DOUBLE_EQ(s->calculateEstimate<double>(0),x);EXPECT_DOUBLE_EQ(r.error_before,r.error_after);
  EXPECT_EQ(s->getFactorsUnsafe()[0],f);
}
TEST(PublicationRoundoff, CandidateExceptionRollsBack) {
  auto fault=std::make_shared<Fault>();auto s=Solver();auto f=boost::make_shared<ScalarFactor>(false,fault);Start(*s,f,3.);
  const auto* pointer=s.get();const double x=s->calculateEstimate<double>(0);fault->throw_after=1;fault->calls=0;
  const auto r=numerical_audit::Refine(s);
  EXPECT_EQ(r.status,"rejected_exception");EXPECT_EQ(r.accepted,0);EXPECT_EQ(s.get(),pointer);
  EXPECT_DOUBLE_EQ(s->calculateEstimate<double>(0),x);EXPECT_EQ(s->getFactorsUnsafe()[0],f);
}
}  // namespace
