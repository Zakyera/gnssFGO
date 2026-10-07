#include <gtest/gtest.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include "offline_process/TransactionalISAM2Rebuild.h"

namespace {
gtsam::ISAM2Params Params(){gtsam::ISAM2Params p;p.factorization=gtsam::ISAM2Params::QR;p.relinearizeSkip=1;p.findUnusedFactorSlots=true;return p;}
gtsam::NonlinearFactorGraph Graph(){
 gtsam::NonlinearFactorGraph g;auto n=gtsam::noiseModel::Isotropic::Sigma(1,.5);
 g.push_back(boost::make_shared<gtsam::PriorFactor<double>>(0,0.,n));g.push_back(gtsam::NonlinearFactor::shared_ptr());
 g.push_back(boost::make_shared<gtsam::BetweenFactor<double>>(0,1,1.,n));g.push_back(gtsam::NonlinearFactor::shared_ptr());return g;
}
TEST(TransactionalRebuild, PreservesNullSlotsPointersParametersAndCovariance){
 const auto graph=Graph();gtsam::Values seed;seed.insert(0,.1);seed.insert(1,1.2);
 fgo::offline::SlotPreservingISAM2 original(Params(),graph,seed);
 auto rebuilt=fgo::offline::RebuildFullHistoryQR(original,original.calculateBestEstimate());
 EXPECT_NO_THROW(fgo::offline::RequireSameFactorSlots(graph,rebuilt->getFactorsUnsafe()));
 EXPECT_TRUE(rebuilt->params().findUnusedFactorSlots);EXPECT_EQ(rebuilt->params().relinearizeSkip,1);
 EXPECT_LT(original.calculateEstimate().localCoordinates(rebuilt->calculateEstimate()).norm(),1e-12);
 EXPECT_TRUE(original.marginalCovariance(1).isApprox(rebuilt->marginalCovariance(1),1e-12));
}
TEST(TransactionalRebuild, SubsequentInsertRemoveAndStableIdReplacement){
 auto graph=Graph();gtsam::Values seed;seed.insert(0,.1);seed.insert(1,1.2);
 fgo::offline::SlotPreservingISAM2 original(Params(),graph,seed);
 auto rebuilt=fgo::offline::RebuildFullHistoryQR(original,original.calculateBestEstimate());
 size_t slot=2;
 for(int revision=1;revision<=12;++revision){
  gtsam::NonlinearFactorGraph revised;revised.push_back(boost::make_shared<gtsam::BetweenFactor<double>>(0,1,1.+revision*.01,gtsam::noiseModel::Isotropic::Sigma(1,.5)));
  auto a=original.update(revised,gtsam::Values(),{slot});auto b=rebuilt->update(revised,gtsam::Values(),{slot});
  ASSERT_EQ(a.newFactorsIndices,b.newFactorsIndices);slot=b.newFactorsIndices.at(0);
  EXPECT_EQ(rebuilt->getFactorsUnsafe()[slot],revised[0]);
  EXPECT_LT(original.calculateBestEstimate().localCoordinates(rebuilt->calculateBestEstimate()).norm(),1e-10);
  EXPECT_EQ(original.getFactorsUnsafe().nrFactors(),rebuilt->getFactorsUnsafe().nrFactors());
 }
}
TEST(TransactionalRebuild, UnsupportedSolverProfileFailsWithoutMutation){
 auto p=Params();p.relinearizeSkip=2;gtsam::ISAM2 original(p);gtsam::Values v;v.insert(0,0.);
 gtsam::NonlinearFactorGraph graph;graph.push_back(boost::make_shared<gtsam::PriorFactor<double>>(0,0.,gtsam::noiseModel::Isotropic::Sigma(1,1)));
 original.update(graph,v);auto old=original.calculateEstimate();
 EXPECT_THROW(fgo::offline::RebuildFullHistoryQR(original,old),std::runtime_error);
 EXPECT_EQ(original.getFactorsUnsafe()[0],graph[0]);EXPECT_EQ(original.calculateEstimate().localCoordinates(old).norm(),0.);
}
TEST(TransactionalRebuild, MismatchedSlotMapIsRejected){
 auto a=Graph(),b=a;b.remove(2);EXPECT_THROW(fgo::offline::RequireSameFactorSlots(a,b),std::runtime_error);
}
}
