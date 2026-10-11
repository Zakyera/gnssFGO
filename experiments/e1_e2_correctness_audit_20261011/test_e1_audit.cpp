// Existing 14 assertions remain unchanged, linked to exact saved E1 libraries.
#include "/workspace/cbs_gtsam4.3/src/glim_ros1/test/cbs_local_graph_linearizer_test.cpp"
namespace {
TEST(LocalLinearizerCpu, EmptyAndNullSlotsPreserveShape) {
  NonlinearFactorGraph g;Values v;
  EXPECT_EQ(glim::linearizeCbsLocalGraphBatched(g,v)->size(),0u);
  g.resize(5);auto out=glim::linearizeCbsLocalGraphBatched(g,v);
  ASSERT_EQ(out->size(),5u);for(auto& f:*out)EXPECT_FALSE(f);
}
#ifdef GTSAM_POINTS_USE_CUDA
TEST_F(LocalLinearizerGpu, WrongTypePreflightHasNoCacheSideEffects) {
  auto good=poses(5);auto g=graph(good,12);auto bad=good;
  bad.erase(X(3));bad.insert(X(3),Vector3(1,2,3));
  EXPECT_ANY_THROW(glim::linearizeCbsLocalGraphBatched(g,bad));
  for(auto& f:g)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))
    EXPECT_FALSE(gpu->has_cached_linearization());
  compare(g,poses(5,.04),true);
}
TEST_F(LocalLinearizerGpu, SharedFactorsAcrossGraphsAndValuesRemainFresh) {
  auto v=poses(5);auto main=graph(v,36);auto local=main;
  for(size_t i=8;i<local.size();i+=3)local[i].reset();
  for(int step=0;step<12;++step) {
    auto a=poses(5,.003*step),b=poses(5,-.004*step);
    main.linearize(a);compare(local,b,true,step%2);
    for(auto& f:main)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))
      EXPECT_FALSE(gpu->has_cached_linearization());
    compare(main,a,true,step%2);
  }
}
TEST_F(LocalLinearizerGpu, LargeBatchConsumesEveryCacheDespiteCpuFailure) {
  auto v=poses(16);auto g=graph(v,256);auto previous=g[0];
  g[0]=std::make_shared<ThrowPrior>();
  EXPECT_THROW(glim::linearizeCbsLocalGraphBatched(g,v),std::runtime_error);
  for(auto& f:g)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))
    EXPECT_FALSE(gpu->has_cached_linearization());
  g[0]=previous;compare(g,poses(16,.035),true);
}
#endif
}
