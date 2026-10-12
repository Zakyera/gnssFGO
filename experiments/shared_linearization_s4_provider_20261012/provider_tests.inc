
TEST(Provider, SelectorIsExplicitAndDefaultOff) {
  EXPECT_FALSE(parseProvider(nullptr));EXPECT_FALSE(parseProvider(""));
  EXPECT_FALSE(parseProvider("independent_local"));EXPECT_TRUE(parseProvider("shared_native"));
  EXPECT_THROW(parseProvider("unknown"),std::runtime_error);
}
TEST(Provider, SameHelperNoExtraEvaluationAndNoMainMutation) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  auto counts=std::make_shared<Counts>();
  NonlinearFactorGraph f;f.add(std::make_shared<Counted>(x(0),x(1),pose(0).between(pose(1)),counts));
  Values v;v.insert(x(1),pose(1));o->admit(f);main.update(f,v,{{x(1),1}});
  auto prior=main.calculateEstimate();auto before=*counts;
  PublishedCovariance result;
  ASSERT_TRUE(publicationCovariance(&main,x(0),x(1),prior.at<Pose3>(x(0)),prior.at<Pose3>(x(1)),result).ok);
  EXPECT_LT((result.relative.covariance-result.endpoints.relative).norm(),1e-12);
  auto oracle=cbs::relative_pose_covariance::compute(main.jointMarginalInformation({x(0),x(1)}),
    x(0),x(1),prior.at<Pose3>(x(0)),prior.at<Pose3>(x(1)),
    prior.at<Pose3>(x(0)).between(prior.at<Pose3>(x(1))),cbs::relative_pose_covariance::Mode::SchurRelativeBetween);
  EXPECT_LT((result.relative.covariance-oracle.covariance).norm(),1e-10);
  EXPECT_EQ(counts->linear,before.linear);EXPECT_EQ(counts->error,before.error);
  EXPECT_TRUE(main.calculateEstimate().equals(prior,0));
}
TEST(Provider, UnavailableAndBadEndpointLeaveOutputUntouched) {
  PublishedCovariance output;output.relative.covariance.setConstant(17);
  int unknown=0;
  EXPECT_FALSE(publicationCovariance(&unknown,x(0),x(1),pose(0),pose(1),output).ok);
  EXPECT_TRUE((output.relative.covariance.array()==17).all());
  Main main(2,params());auto o=attach(&main);initial(main,o);auto prior=main.calculateEstimate();
  EXPECT_FALSE(publicationCovariance(&main,x(0),x(9),pose(0),pose(9),output).ok);
  EXPECT_FALSE(o->valid());EXPECT_TRUE((output.relative.covariance.array()==17).all());
  EXPECT_TRUE(main.calculateEstimate().equals(prior,0));
}
