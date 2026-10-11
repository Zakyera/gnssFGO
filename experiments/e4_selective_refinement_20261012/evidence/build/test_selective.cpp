#include "SelectivePublicationRefinement.h"
#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <bit>

namespace {
gtsam::KeySet Keys(std::initializer_list<gtsam::Key> keys) {
  return gtsam::KeySet(keys.begin(),keys.end());
}
bool Exact(const gtsam::Values& a,const gtsam::Values& b) {
  if(a.keys()!=b.keys())return false;
  for(auto key:a.keys())if(std::bit_cast<uint64_t>(a.at<double>(key))!=
                           std::bit_cast<uint64_t>(b.at<double>(key)))return false;
  return true;
}
TEST(Selective, ExactTestHelperRejectsOneBitChange) {
  gtsam::Values a,b;a.insert(0,1.);b=a;EXPECT_TRUE(Exact(a,b));
  b.update(0,std::nextafter(1.,2.));EXPECT_FALSE(Exact(a,b));
}
struct Fault {int calls=0,fail_at=-1;};
class Square : public gtsam::NoiseModelFactor1<double> {
 public:
  std::shared_ptr<Fault> fault;
  Square(gtsam::Key k=0,std::shared_ptr<Fault> f=std::make_shared<Fault>())
    :NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1,1),k),fault(f){}
  gtsam::Vector evaluateError(const double& x,boost::optional<gtsam::Matrix&> H=boost::none)const override {
    if(H) {
      if(++fault->calls==fault->fail_at)throw std::runtime_error("injected single-call fault");
      *H=gtsam::Matrix::Constant(1,1,2*x);
    }
    return gtsam::Vector::Constant(1,x*x-4);
  }
};
std::unique_ptr<gtsam::ISAM2> Make(double x,std::shared_ptr<Fault> f=std::make_shared<Fault>()) {
  gtsam::ISAM2Params p;p.relinearizeThreshold=.01;p.relinearizeSkip=1;
  p.factorization=gtsam::ISAM2Params::QR;p.findUnusedFactorSlots=true;
  auto s=std::make_unique<gtsam::ISAM2>(p);
  gtsam::NonlinearFactorGraph g;g.push_back(boost::make_shared<Square>(0,f));
  gtsam::Values v;v.insert(0,x);
  for(size_t k=1;k<4;++k) {
    g.add(gtsam::PriorFactor<double>(k,0.,gtsam::noiseModel::Isotropic::Sigma(1,1)));
    v.insert(k,0.);
  }
  s->update(g,v);return s;
}
TEST(Selective, CurrentGradientMatchesFrozenNotCached) {
  auto s=Make(3.);const auto v=s->calculateEstimate();
  const auto c=e4::Certify(s->getFactorsUnsafe(),v,1e-5);
  EXPECT_DOUBLE_EQ(c.maximum,fgo::offline::NormalizedGradient(s->getFactorsUnsafe(),v));
  EXPECT_GT(c.maximum,1e-5);EXPECT_EQ(c.violating,Keys({0}));
}
TEST(Selective, NeighboursAreOneHopNotTransitive) {
  gtsam::NonlinearFactorGraph g;
  for(size_t k=0;k<5;++k)g.add(gtsam::BetweenFactor<double>(k,k+1,0.,gtsam::noiseModel::Isotropic::Sigma(1,1)));
  EXPECT_EQ(e4::Neighbours(g,Keys({2})),Keys({1,2,3}));
  EXPECT_TRUE(e4::Neighbours(g,{}).empty());
}
TEST(Selective, ActualSubsetRelinearizationAndStationaryCertificate) {
  auto s=Make(2.01);auto r=e4::Refine(s,20,1e-5,true);
  EXPECT_EQ(e4::diagnostic.branch,"selective_accepted");
  EXPECT_EQ(e4::diagnostic.selected,1);EXPECT_EQ(e4::diagnostic.relinearized,1);
  EXPECT_EQ(r.status,"stationary");EXPECT_EQ(r.attempted,1);
  EXPECT_LE(fgo::offline::NormalizedGradient(s->getFactorsUnsafe(),s->calculateEstimate()),1e-5);
}
TEST(Selective, StationaryNoOpDoesNotCopyOrMutate) {
  auto s=Make(2.);auto* ptr=s.get();auto r=e4::Refine(s,20,1e-5,true);
  EXPECT_EQ(r.status,"stationary");EXPECT_EQ(r.attempted,0);EXPECT_EQ(ptr,s.get());
}
TEST(Selective, DisabledIsReferenceExactly) {
  auto a=Make(3.),b=std::make_unique<gtsam::ISAM2>(*a);
  auto r=numerical_audit::Refine(a);auto c=e4::Refine(b,20,1e-5,false);
  EXPECT_EQ(r.status,c.status);EXPECT_EQ(r.attempted,c.attempted);
  EXPECT_TRUE(Exact(a->calculateEstimate(),b->calculateEstimate()));
}
TEST(Selective, RejectedTrialFallsBackFromOriginal) {
  auto a=Make(3.),b=std::make_unique<gtsam::ISAM2>(*a);
  auto r=numerical_audit::Refine(a,19);auto c=e4::Refine(b,20,1e-5,true);
  EXPECT_EQ(e4::diagnostic.branch,"selective_rejected");EXPECT_EQ(c.status,"stationary");
  EXPECT_TRUE(Exact(a->calculateEstimate(),b->calculateEstimate()));
  EXPECT_EQ(c.attempted,r.attempted+1);EXPECT_FALSE(c.steps[0].accepted);
}
TEST(Selective, OneAttemptBudgetDoesNotPublishUnconvergedTrial) {
  auto s=Make(3.);auto initial=s->calculateEstimate();auto* p=s.get();
  auto r=e4::Refine(s,1,1e-5,true);
  EXPECT_EQ(r.status,"budget_exhausted");EXPECT_EQ(r.attempted,1);
  EXPECT_EQ(p,s.get());EXPECT_TRUE(Exact(initial,s->calculateEstimate()));
}
TEST(Selective, ExceptionTrialRollsBackThenReferenceFallback) {
  auto f=std::make_shared<Fault>();auto s=Make(3.,f);
  auto reference=std::make_unique<gtsam::ISAM2>(*s);
  f->fail_at=f->calls+2;auto r=e4::Refine(s,20,1e-5,true);
  EXPECT_EQ(e4::diagnostic.branch,"selective_exception");EXPECT_EQ(r.status,"stationary");
  auto c=numerical_audit::Refine(reference,19);
  EXPECT_EQ(c.status,"stationary");EXPECT_TRUE(Exact(s->calculateEstimate(),reference->calculateEstimate()));
}
TEST(Selective, InvalidPolicyFailsBeforeMutation) {
  auto s=Make(3.);auto* p=s.get();
  EXPECT_THROW(e4::Refine(s,20,0,true),std::invalid_argument);
  EXPECT_THROW(e4::Refine(s,20,NAN,true),std::invalid_argument);EXPECT_EQ(p,s.get());
}
TEST(Selective, FactorReplacementAndSlotIdentityPreserved) {
  auto s=Make(2.);auto f=boost::make_shared<Square>(1);
  gtsam::NonlinearFactorGraph added;added.push_back(f);
  // Seed x1 away from the singular x=0 square derivative through a first prior.
  auto prior=boost::make_shared<gtsam::PriorFactor<double>>(1,2.01,gtsam::noiseModel::Isotropic::Sigma(1,1));
  gtsam::NonlinearFactorGraph seed;seed.push_back(prior);
  auto u=s->update(seed,{},gtsam::FactorIndices{1});
  s->update(added,{},gtsam::FactorIndices{u.newFactorsIndices[0]});
  const auto before=s->getFactorsUnsafe();auto r=e4::Refine(s,20,1e-5,true);
  EXPECT_EQ(r.status,"stationary");
  EXPECT_NO_THROW(fgo::offline::RequireSameFactorSlots(before,s->getFactorsUnsafe()));
}
TEST(Selective, DensePosteriorAndSolutionMatchReference) {
  auto a=Make(2.01),b=std::make_unique<gtsam::ISAM2>(*a);
  auto r=numerical_audit::Refine(a);auto c=e4::Refine(b,20,1e-5,true);
  ASSERT_EQ(r.status,"stationary");ASSERT_EQ(c.status,"stationary");
  EXPECT_TRUE(a->calculateEstimate().equals(b->calculateEstimate(),1e-8));
  const auto av=a->calculateEstimate(),bv=b->calculateEstimate();
  gtsam::Marginals am(a->getFactorsUnsafe(),av,gtsam::Marginals::QR),
    bm(b->getFactorsUnsafe(),bv,gtsam::Marginals::QR);
  const auto ca=am.jointMarginalCovariance({0,1,2,3}).fullMatrix();
  const auto cb=bm.jointMarginalCovariance({0,1,2,3}).fullMatrix();
  // Independent dense inverse of the Jacobian information at the solution.
  gtsam::Matrix expected=gtsam::Matrix::Identity(4,4);
  expected(0,0)=1/(4*bv.at<double>(0)*bv.at<double>(0));
  EXPECT_LE((ca-cb).cwiseAbs().maxCoeff(),1e-8+1e-6*ca.cwiseAbs().maxCoeff());
  EXPECT_LE((cb-expected).cwiseAbs().maxCoeff(),1e-12);
}
TEST(Selective, DeterministicRepeat) {
  auto a=Make(2.01),b=Make(2.01);
  auto r=e4::Refine(a,20,1e-5,true);auto c=e4::Refine(b,20,1e-5,true);
  EXPECT_EQ(r.status,c.status);EXPECT_EQ(r.gradient_after,c.gradient_after);
  EXPECT_TRUE(Exact(a->calculateEstimate(),b->calculateEstimate()));
}
}
