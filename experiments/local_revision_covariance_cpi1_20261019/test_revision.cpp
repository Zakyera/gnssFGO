#include "RevisionCovariance.h"
#include "test_lifecycle.cpp"

class DenseRevisionFactor : public BetweenFactor<Pose3> {
 public:
  DenseRevisionFactor(Key a, Key b, const Pose3& z, std::shared_ptr<Counts> c, int k)
      : BetweenFactor<Pose3>(a,b,z,denseNoise(k)), counts(std::move(c)) {}
  static SharedNoiseModel denseNoise(int k) {
    Matrix6 l = Matrix6::Identity() * (.3 + .01*k);
    l(3,0)=.07; l(4,2)=.04; l(5,1)=.02;
    return noiseModel::Gaussian::Covariance(l*l.transpose());
  }
  GaussianFactor::shared_ptr linearize(const Values& v) const override {
    ++counts->linear; return BetweenFactor<Pose3>::linearize(v);
  }
  double error(const Values& v) const override {
    ++counts->error; return BetweenFactor<Pose3>::error(v);
  }
 private:
  std::shared_ptr<Counts> counts;
};

TEST(RevisionPolicy, DefaultUnchangedAndExplicitConfiguration) {
  EXPECT_EQ(parseRevisionCovariance(nullptr), RevisionCovariancePolicy::Main);
  EXPECT_EQ(parseRevisionCovariance(""), RevisionCovariancePolicy::Main);
  EXPECT_EQ(parseRevisionCovariance("main"), RevisionCovariancePolicy::Main);
  EXPECT_EQ(parseRevisionCovariance("native_local"), RevisionCovariancePolicy::NativeLocal);
  EXPECT_THROW(parseRevisionCovariance("local"), std::runtime_error);
  EXPECT_THROW(validateRevisionCovariance(RevisionCovariancePolicy::NativeLocal, false), std::runtime_error);
  EXPECT_NO_THROW(validateRevisionCovariance(RevisionCovariancePolicy::NativeLocal, true));
  EXPECT_NO_THROW(validateRevisionCovariance(RevisionCovariancePolicy::Main, false));
}

TEST(RevisionPolicy, MainCallbackIsUsedOnlyInHistoricalMode) {
  int calls = 0;
  auto query = [&] { ++calls; cbs::relative_pose_covariance::Result r;
    r.covariance = Matrix6::Identity() * 17; return r; };
  const auto result = revisionCovariance(RevisionCovariancePolicy::Main,
      nullptr, x(0), x(1), pose(0), pose(1), query);
  EXPECT_EQ(calls, 1); EXPECT_EQ(result.covariance.trace(), 102);
  EXPECT_THROW(revisionCovariance(RevisionCovariancePolicy::NativeLocal,
      nullptr, x(0), x(1), pose(0), pose(1), query), std::runtime_error);
  EXPECT_EQ(calls, 1);  // Failed LOCAL never falls back to MAIN.
}

TEST(RevisionPolicy, CurrentLocalRevisionsAcrossUpdatesAndMarginalization) {
  Main main(2.5, params()); auto obs = attach(&main);
  initial(main, obs);
  auto counts = std::make_shared<Counts>();
  int main_queries = 0, checked_pairs = 0;
  Matrix6 previous = Matrix6::Zero();
  for (int k = 1; k <= 8; ++k) {
    NonlinearFactorGraph native;
    native.push_back(std::make_shared<DenseRevisionFactor>(x(k-1),x(k),pose(k-1).between(pose(k)),counts,k));
    obs->admit(native);
    // Same factor type, stronger but unadmitted peer information in MAIN.
    NonlinearFactorGraph all = native;
    all.emplace_shared<BetweenFactor<Pose3>>(x(k-1), x(k),
        pose(k-1).between(pose(k)).retract(Vector6::Constant(.002)),
        noiseModel::Isotropic::Sigma(6, .03));
    Values init; init.insert(x(k), pose(k));
    main.update(all, init, {{x(k), double(k)}});
    ASSERT_TRUE(obs->valid()) << obs->error();
    const auto before = main.calculateEstimate();
    const Counts before_counts = *counts;
    for (int j = std::max(0, k-2); j < k; ++j) {
      const auto a = before.at<Pose3>(x(j)), b = before.at<Pose3>(x(j+1));
      auto forbidden = [&]() -> cbs::relative_pose_covariance::Result {
        ++main_queries; throw std::runtime_error("MAIN queried in LOCAL mode"); };
      auto result = revisionCovariance(RevisionCovariancePolicy::NativeLocal,
          &main, x(j), x(j+1), a, b, forbidden);
      PublishedCovariance ordinary;
      ASSERT_TRUE(publicationCovariance(&main,x(j),x(j+1),a,b,ordinary).ok);
      EXPECT_TRUE(result.covariance.isApprox(ordinary.relative.covariance, 0));
      auto repeat = revisionCovariance(RevisionCovariancePolicy::NativeLocal,
          &main, x(j), x(j+1), a, b, forbidden);
      EXPECT_TRUE(result.covariance.isApprox(repeat.covariance, 0));
      EXPECT_GT(result.covariance.cwiseAbs().sum()-result.covariance.diagonal().cwiseAbs().sum(), 1e-5);
      if (j == k-1) {
        auto mixed = cbs::relative_pose_covariance::compute(
            main.jointMarginalInformation({x(j),x(j+1)}),x(j),x(j+1),a,b,a.between(b),
            cbs::relative_pose_covariance::Mode::SchurRelativeBetween);
        EXPECT_GT((result.covariance-mixed.covariance).norm(), .01);
        if (k > 1) EXPECT_GT((result.covariance-previous).norm(), 1e-8);
        previous = result.covariance;
      }
      ++checked_pairs;
    }
    EXPECT_TRUE(before.equals(main.calculateEstimate(),0));
    EXPECT_EQ(counts->linear,before_counts.linear);
    EXPECT_EQ(counts->error,before_counts.error);
  }
  EXPECT_EQ(main_queries,0); EXPECT_EQ(checked_pairs,15);
  EXPECT_GT(obs->history().absorbedCount(),0u);
  auto before=main.calculateEstimate();
  EXPECT_THROW(revisionCovariance(RevisionCovariancePolicy::NativeLocal,
      &main,x(0),x(1),pose(0),pose(1),[&] {
        ++main_queries; return cbs::relative_pose_covariance::Result{};
      }),std::runtime_error);
  EXPECT_EQ(main_queries,0); EXPECT_TRUE(before.equals(main.calculateEstimate(),0));
}
