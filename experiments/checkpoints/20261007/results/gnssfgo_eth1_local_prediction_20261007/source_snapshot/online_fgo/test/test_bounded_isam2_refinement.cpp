#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include "offline_process/BoundedISAM2Refinement.h"

namespace {
using namespace fgo::offline;
struct Fault { int calls{0}; int throw_after{-1}; };
class ScalarFactor : public gtsam::NoiseModelFactor1<double> {
  bool cycle_;
  std::shared_ptr<Fault> fault_;
 public:
  ScalarFactor(bool cycle = false, std::shared_ptr<Fault> fault = std::make_shared<Fault>())
    : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1,1), 0), cycle_(cycle), fault_(fault) {}
  gtsam::Vector evaluateError(const double& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      if (fault_->throw_after >= 0 && ++fault_->calls > fault_->throw_after)
        throw std::runtime_error("synthetic linearization failure");
      *H = gtsam::Matrix::Constant(1,1,cycle_ ? 3*x*x-2 : 2*x);
    }
    return gtsam::Vector::Constant(1,cycle_ ? x*x*x-2*x+2 : x*x-4);
  }
};
std::unique_ptr<gtsam::ISAM2> Solver(double threshold = .01) {
  gtsam::ISAM2Params p; p.relinearizeThreshold=threshold; p.relinearizeSkip=1;
  p.factorization=gtsam::ISAM2Params::QR;
  return std::make_unique<gtsam::ISAM2>(p);
}
void Start(gtsam::ISAM2& s, const gtsam::NonlinearFactor::shared_ptr& f, double x) {
  gtsam::NonlinearFactorGraph graph; graph.push_back(f);
  gtsam::Values v; v.insert(0,x); s.update(graph,v);
}
TEST(BoundedISAM2, LinearStationaryDoesNotUpdate) {
  auto s=Solver(); Start(*s,boost::make_shared<gtsam::PriorFactor<double>>(0,2.,gtsam::noiseModel::Isotropic::Sigma(1,1)),3);
  const auto* original=s.get(); const auto r=RefineISAM2(s);
  EXPECT_EQ(r.status,"stationary"); EXPECT_EQ(r.attempted,0); EXPECT_EQ(s.get(),original);
}
TEST(BoundedISAM2, NonlinearDescentBoundedAndSlotPreserved) {
  // Tight synthetic threshold allows stationarity; the production-threshold
  // stall is checked separately rather than mislabelled as convergence.
  auto s=Solver(1e-9); auto f=boost::make_shared<ScalarFactor>(); Start(*s,f,3);
  auto original=std::make_unique<gtsam::ISAM2>(*s);
  const double original_value=original->calculateEstimate<double>(0);
  const auto r=RefineISAM2(s);
  EXPECT_GT(r.accepted,0); EXPECT_LE(r.attempted,5);
  EXPECT_LT(r.error_after,r.error_before);
  EXPECT_EQ(s->getFactorsUnsafe()[0],f);
  EXPECT_DOUBLE_EQ(original->calculateEstimate<double>(0),original_value);
  EXPECT_LT(std::abs(s->calculateEstimate<double>(0)-2),1e-5);
  // The original solver copy and shared factor payload remain unmodified.
  EXPECT_DOUBLE_EQ(original->getFactorsUnsafe().error(original->calculateEstimate()),r.error_before);
}
TEST(BoundedISAM2, BudgetAndStallAreNotConvergence) {
  auto s=Solver(); Start(*s,boost::make_shared<ScalarFactor>(),3);
  auto r=RefineISAM2(s,{1,1e-12});
  EXPECT_EQ(r.status,"budget_exhausted"); EXPECT_EQ(r.attempted,1);
  auto stalled=Solver(100); Start(*stalled,boost::make_shared<ScalarFactor>(),3);
  r=RefineISAM2(stalled);
  EXPECT_EQ(r.status,"stalled_nonstationary"); EXPECT_GT(r.gradient_after,1e-5);
}
TEST(BoundedISAM2, IncreasingNewtonStepRollsBack) {
  auto s=Solver(); auto f=boost::make_shared<ScalarFactor>(true); Start(*s,f,0);
  const double before=s->calculateEstimate<double>(0);
  const auto* pointer=s.get(); auto r=RefineISAM2(s);
  EXPECT_EQ(r.status,"rejected_objective"); EXPECT_EQ(r.accepted,0);
  EXPECT_EQ(s.get(),pointer); EXPECT_DOUBLE_EQ(s->calculateEstimate<double>(0),before);
  EXPECT_DOUBLE_EQ(r.error_before,r.error_after); EXPECT_EQ(s->getFactorsUnsafe()[0],f);
}
TEST(BoundedISAM2, CandidateExceptionRollsBack) {
  auto fault=std::make_shared<Fault>(); auto s=Solver();
  Start(*s,boost::make_shared<ScalarFactor>(false,fault),3);
  const auto before=s->calculateEstimate<double>(0); const auto* pointer=s.get();
  fault->throw_after=1; fault->calls=0;
  auto r=RefineISAM2(s);
  EXPECT_EQ(r.status,"rejected_exception"); EXPECT_EQ(r.accepted,0);
  EXPECT_EQ(s.get(),pointer); EXPECT_DOUBLE_EQ(s->calculateEstimate<double>(0),before);
}
TEST(BoundedISAM2, NonfiniteAndInvalidPolicyRejected) {
  EXPECT_FALSE(NonIncreasingFiniteError(1,std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(NonIncreasingFiniteError(1,std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(NonIncreasingFiniteError(1,1.00000001));
  EXPECT_TRUE(NonIncreasingFiniteError(1,1));
  auto s=Solver(); EXPECT_THROW(RefineISAM2(s,{5,-1}),std::invalid_argument);
}
}  // namespace
