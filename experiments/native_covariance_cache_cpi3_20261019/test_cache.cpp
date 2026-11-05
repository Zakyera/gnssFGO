#include "test_native_gaussian.cpp"  // Inherited tests stay literally unchanged.
#include "OracleHistory.h"

namespace {
uncached_native::Snapshot oracleSnapshot(const Snapshot& s) {
  uncached_native::Snapshot o;
  o.generation=s.generation; o.epoch=s.epoch; o.cache=s.cache;
  o.removed=s.removed; o.theta=s.theta; o.retiring=s.retiring; o.fixed_after=s.fixed_after;
  for (const auto& r:s.native) o.native.push_back({r.id,r.version,r.slot});
  return o;
}
void acceptBoth(NativeGaussianHistory& h, uncached_native::NativeGaussianHistory& o,
                const Snapshot& s) {
  auto a=h.stage(s);
  auto b=o.stage(oracleSnapshot(s));
  ASSERT_TRUE(a.ok)<<a.message; ASSERT_TRUE(b.ok)<<b.message;
  ASSERT_TRUE(h.commit(s.generation,s.epoch).ok);
  ASSERT_TRUE(o.commit(s.generation,s.epoch).ok);
  EXPECT_EQ(h.queryPreparations(),0u);
}
template<class A, class B> void exact(const A& a,const B& b) {
  for (auto pair:{std::make_pair(a.joint_theta,b.joint_theta),
                 std::make_pair(a.transport,b.transport),
                 std::make_pair(a.joint_current,b.joint_current),
                 std::make_pair(Matrix(a.relative),Matrix(b.relative))}) {
    EXPECT_EQ((pair.first-pair.second).cwiseAbs().maxCoeff(),0);
  }
}
}

TEST(EpochQueryCache, AllPairsChartsOrderAndRepeatMatchLiteralOriginal) {
  Fixture f; NativeGaussianHistory h(Mode::Shadow);
  uncached_native::NativeGaussianHistory o(uncached_native::Mode::Shadow);
  acceptBoth(h,o,f.s);
  auto original=h.graph(1).augmentedHessian();
  for (int round=0;round<5;++round) for (int a=0;a<3;++a) for(int b=2;b>=0;--b) {
    if(a==b) continue;
    Vector6 da=.01*randomMatrix(6,1,round+32), db=.03*randomMatrix(6,1,round+52);
    auto actual=h.covariance(1,X(a),X(b),da,db);
    auto expected=o.covariance(1,X(a),X(b),da,db);
    exact(actual,expected);
    exact(h.covariance(1,X(a),X(b),da,db),actual);
    EXPECT_EQ(h.queryPreparations(),1u);
  }
  EXPECT_TRUE(original.isApprox(h.graph(1).augmentedHessian(),0));
  EXPECT_TRUE(f.s.theta.equals(h.theta(1),0));
}

TEST(EpochQueryCache, StageAbortAndFailurePreserveCommittedCache) {
  Fixture f; NativeGaussianHistory h(Mode::Shadow); f.accept(h);
  auto first=h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero());
  f.s.epoch=2;
  f.s.native.back().version=2;
  f.raw[5].J*=3; f.s.cache[4]=f.raw[5].factor();
  ASSERT_TRUE(h.stage(f.s).ok);
  EXPECT_FALSE(h.commit(1,3).ok);
  exact(first,h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()));
  h.abort(); EXPECT_EQ(h.queryPreparations(),1u);
  auto bad=f.s; bad.native[0].slot=999;
  EXPECT_FALSE(h.stage(bad).ok);
  exact(first,h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()));
  ASSERT_TRUE(h.stage(f.s).ok); ASSERT_TRUE(h.commit(1,2).ok);
  EXPECT_EQ(h.queryPreparations(),0u);
  auto next=h.covariance(2,X(1),X(2),Vector6::Zero(),Vector6::Zero());
  EXPECT_GT((next.relative-first.relative).norm(),1e-5);
  EXPECT_EQ(h.queryPreparations(),1u);
  EXPECT_THROW(h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()),std::exception);
}

TEST(EpochQueryCache, RetirementRemovalAndResetCannotReuseOldPreparation) {
  Fixture f; NativeGaussianHistory h(Mode::Shadow);
  uncached_native::NativeGaussianHistory o(uncached_native::Mode::Shadow);
  acceptBoth(h,o,f.s);
  (void)h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero());
  f.s.epoch=2; retire(f,0); acceptBoth(h,o,f.s);
  exact(h.covariance(2,X(1),X(2),Vector6::Zero(),Vector6::Zero()),
        o.covariance(2,X(1),X(2),Vector6::Zero(),Vector6::Zero()));
  EXPECT_THROW(h.covariance(2,X(0),X(1),Vector6::Zero(),Vector6::Zero()),std::exception);
  f.s.native.erase(std::remove_if(f.s.native.begin(),f.s.native.end(),
      [](const NativeRef& r){return r.id==1||r.id==2||r.id==4||r.id==5;}),f.s.native.end());
  f.s.removed={5}; f.s.retiring.clear(); f.s.theta=h.theta(2); f.s.epoch=3;
  acceptBoth(h,o,f.s);
  exact(h.covariance(3,X(1),X(2),Vector6::Constant(.02),Vector6::Constant(-.03)),
        o.covariance(3,X(1),X(2),Vector6::Constant(.02),Vector6::Constant(-.03)));
  ASSERT_TRUE(h.reset(2).ok); ASSERT_TRUE(o.reset(2).ok);
  EXPECT_EQ(h.queryPreparations(),0u);
  EXPECT_THROW(h.covariance(3,X(1),X(2),Vector6::Zero(),Vector6::Zero()),std::exception);
  Fixture fresh; fresh.s.generation=2;
  fresh.raw[5].J*=2; fresh.s.cache[4]=fresh.raw[5].factor();
  acceptBoth(h,o,fresh.s);
  exact(h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()),
        o.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero()));
}

TEST(EpochQueryCache, NewReferenceChartInvalidatesEvenWithoutRetirement) {
  Fixture f; NativeGaussianHistory h(Mode::Shadow);
  uncached_native::NativeGaussianHistory o(uncached_native::Mode::Shadow);
  acceptBoth(h,o,f.s);
  (void)h.covariance(1,X(1),X(2),Vector6::Zero(),Vector6::Zero());
  f.s.epoch=2;
  for(auto& r:f.s.native) ++r.version;
  f.s.theta.update(X(2),f.s.theta.at<Pose3>(X(2)).retract(Vector6::Constant(.05)));
  acceptBoth(h,o,f.s);
  exact(h.covariance(2,X(1),X(2),Vector6::Constant(.02),Vector6::Constant(.04)),
        o.covariance(2,X(1),X(2),Vector6::Constant(.02),Vector6::Constant(.04)));
}

TEST(EpochQueryCache, InvalidQueryDoesNotMutateOutputOrInstallFailedPreparation) {
  Fixture f; NativeGaussianHistory h(Mode::Shadow); f.accept(h);
  EndpointCovariance sentinel;
  sentinel.joint_theta=Matrix::Constant(12,12,17); sentinel.relative=Matrix6::Constant(19);
  auto saved=sentinel;
  EXPECT_FALSE(h.tryCovariance(1,X(1),X(99),Vector6::Zero(),Vector6::Zero(),sentinel).ok);
  EXPECT_TRUE(sentinel.joint_theta.isApprox(saved.joint_theta,0));
  EXPECT_TRUE(sentinel.relative.isApprox(saved.relative,0));
  EXPECT_EQ(h.queryPreparations(),0u);
  // Make an exactly rank-deficient pose-only native graph.
  Snapshot s; s.theta.insert(X(0),Pose3()); s.theta.insert(X(1),Pose3());
  s.cache.emplace_shared<JacobianFactor>(X(0),-Matrix6::Identity(),X(1),Matrix6::Identity(),Vector6::Zero());
  s.native.push_back({1,1,0});
  NativeGaussianHistory singular(Mode::Shadow);
  ASSERT_TRUE(singular.stage(s).ok); ASSERT_TRUE(singular.commit(1,1).ok);
  for(int i=0;i<2;++i) {
    EXPECT_FALSE(singular.tryCovariance(1,X(0),X(1),Vector6::Zero(),Vector6::Zero(),sentinel).ok);
    EXPECT_EQ(singular.queryPreparations(),0u);
    EXPECT_TRUE(sentinel.relative.isApprox(saved.relative,0));
  }
}
