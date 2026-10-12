#include "NativeGaussianHistory.h"
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace gtsam;
using namespace shared_native;
namespace {
struct Calls { int linearize=0, error=0; };
class CountedBetween : public BetweenFactor<Pose3> {
 public:
  CountedBetween(Key a,Key b,const Pose3& pose,const SharedNoiseModel& noise,
                 std::shared_ptr<Calls> calls)
      : BetweenFactor<Pose3>(a,b,pose,noise), calls_(std::move(calls)) {}
  GaussianFactor::shared_ptr linearize(const Values& values) const override {
    ++calls_->linearize;return BetweenFactor<Pose3>::linearize(values);
  }
  double error(const Values& values) const override {
    ++calls_->error;return BetweenFactor<Pose3>::error(values);
  }
 private:
  std::shared_ptr<Calls> calls_;
};
}
TEST(ActualMainCache, ObserverAddsNoNonlinearCallsAndDoesNotChangeMain){
  ISAM2Params params;params.cacheLinearizedFactors=true;params.enableRelinearization=false;
  params.findUnusedFactorSlots=true;
  gtsam_points::ISAM2Ext main(params);
  Values theta;theta.insert(0,Pose3());theta.insert(1,Pose3(Rot3::Rz(.13),Point3(1.1,.1,.2)));
  theta.insert(2,Pose3(Rot3::Ry(-.1),Point3(2.2,.2,-.1)));
  NonlinearFactorGraph factors;
  auto calls=std::make_shared<Calls>();auto noise=noiseModel::Isotropic::Sigma(6,.1);
  factors.add(PriorFactor<Pose3>(0,Pose3(),noise));
  factors.push_back(std::make_shared<CountedBetween>(0,1,Pose3(Rot3(),Point3(1,0,0)),noise,calls));
  factors.push_back(std::make_shared<CountedBetween>(1,2,Pose3(Rot3(),Point3(1,0,0)),noise,calls));
  // Synthetic incoming CBS has the same factor class but is not native-admitted.
  factors.push_back(std::make_shared<CountedBetween>(0,2,Pose3(Rot3::Rz(.05),Point3(2.1,.1,0)),noise,calls));
  FastMap<Key,int> groups;groups[0]=0;groups[1]=1;groups[2]=2;
  const auto update=main.update(factors,theta,{},groups);
  ASSERT_EQ(update.newFactorsIndices.size(),4u);
  const auto estimate_before=main.calculateEstimate();const auto delta=main.getDelta();
  auto all_before=main.getLinearFactorsUnsafe().augmentedHessian();
  const auto counts_before=*calls;
  Snapshot s;s.theta=main.getLinearizationPoint();s.cache=main.getLinearFactorsUnsafe();
  s.native={{1,1,update.newFactorsIndices[0]},
            {2,1,update.newFactorsIndices[1]},
            {3,1,update.newFactorsIndices[2]}};
  s.retiring={0};s.fixed_after={1,2};
  NativeGaussianHistory observer(Mode::Shadow);
  const auto staged=observer.stage(s);ASSERT_TRUE(staged.ok)<<staged.message;
  EXPECT_EQ(calls->linearize,counts_before.linearize);EXPECT_EQ(calls->error,counts_before.error);
  EXPECT_TRUE(main.getLinearFactorsUnsafe().augmentedHessian().isApprox(all_before,0));
  EXPECT_TRUE(main.calculateEstimate().equals(estimate_before,0));
  // This test adapter represents the missing live pre-retirement hook explicitly.
  main.marginalizeLeaves(FastList<Key>{0});
  EXPECT_TRUE(main.getFixedVariables().count(1));
  const auto counts_after_main=*calls;auto cache_after_main=main.getLinearFactorsUnsafe().augmentedHessian();
  ASSERT_TRUE(observer.commit(1,1).ok);
  EndpointCovariance covariance;
  ASSERT_TRUE(observer.tryCovariance(1,1,2,delta.at(1),delta.at(2),covariance).ok);
  EXPECT_EQ(calls->linearize,counts_after_main.linearize);
  EXPECT_EQ(calls->error,counts_after_main.error);
  EXPECT_TRUE(main.getLinearFactorsUnsafe().augmentedHessian().isApprox(cache_after_main,0));
  EXPECT_EQ(observer.absorbedCount(),2u);EXPECT_EQ(observer.activeCount(),1u);
  std::cout<<"MAIN_CACHE nonlinear_calls_added_by_observer=0 absorbed=2 active=1\n";
}
