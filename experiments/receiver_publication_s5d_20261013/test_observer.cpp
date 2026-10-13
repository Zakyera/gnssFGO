#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include "offline_process/BoundedISAM2Refinement.h"
#include "CachedGaussianEvaluation.h"
TEST(S5d, DisabledDoesNotCapture) {
  unsetenv("S5D_INDICES");s5d::Scope scope(1,0,"test","");
  gtsam::Values x;x.insert(0,2.);
  gtsam::NonlinearFactorGraph g;g.emplace_shared<gtsam::PriorFactor<double>>(0,0.,gtsam::noiseModel::Isotropic::Sigma(1,2));
  EXPECT_DOUBLE_EQ(fgo::offline::NormalizedGradient(g,x),1.);
  EXPECT_TRUE(s5d::frames.empty());
}
TEST(S5d, ExactCopyAndChronology) {
  setenv("S5D_INDICES","1,3",1);s5d::Scope scope(1,0,"test","");
  gtsam::Values x;x.insert(0,2.);
  gtsam::NonlinearFactorGraph g;g.emplace_shared<gtsam::PriorFactor<double>>(0,0.,gtsam::noiseModel::Isotropic::Sigma(1,2));
  const double a=fgo::offline::NormalizedGradient(g,x);
  x.update(0,4.);const publication_cache::Evaluation ev(g,x);const double b=ev.gradient();
  ASSERT_EQ(s5d::frames.size(),2);EXPECT_DOUBLE_EQ(a,1.);EXPECT_DOUBLE_EQ(b,2.);
  EXPECT_DOUBLE_EQ(s5d::frames[0].values.at<double>(0),2.);
  EXPECT_DOUBLE_EQ(s5d::frames[0].gradient.at(0)[0],.5);
  EXPECT_DOUBLE_EQ(s5d::frames[0].diagonal.at(0)[0],.25);
  EXPECT_DOUBLE_EQ(s5d::frames[1].values.at<double>(0),4.);
  EXPECT_DOUBLE_EQ(s5d::frames[1].maximum,2.);unsetenv("S5D_INDICES");
}
TEST(S5d, NoNestedContextAndClearedOnScopeExit) {
  setenv("S5D_INDICES","2",1);
  { s5d::Scope a(2,0,"test",""); EXPECT_THROW((s5d::Scope(2,0,"test","")),std::runtime_error); }
  EXPECT_FALSE(s5d::active);EXPECT_TRUE(s5d::frames.empty());unsetenv("S5D_INDICES");
}
TEST(S5d, NonfiniteStillFailsBeforeCapture) {
  setenv("S5D_INDICES","1",1);s5d::Scope scope(1,0,"test","");
  gtsam::Values x;x.insert(0,std::numeric_limits<double>::quiet_NaN());
  gtsam::NonlinearFactorGraph g;g.emplace_shared<gtsam::PriorFactor<double>>(0,0.,gtsam::noiseModel::Isotropic::Sigma(1,1));
  EXPECT_THROW(fgo::offline::NormalizedGradient(g,x),std::runtime_error);
  EXPECT_TRUE(s5d::frames.empty());unsetenv("S5D_INDICES");
}
