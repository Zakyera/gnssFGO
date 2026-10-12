#include "LocalUpdateBatching.h"
#include "/workspace/cbs_gtsam4.3/runs/e1_correctness_audit_20261011/test_e1_audit.cpp"
#include <thread>

namespace {
using gtsam::e1b4a::ScopedLinearizer;
using glim::e1b4a::Stats;
using glim::e1b4a::Reason;
auto candidate(const NonlinearFactorGraph& graph, const Values& values, Stats* stats=nullptr) {
  ScopedLinearizer scope(&glim::e1b4a::linearize,stats);
  return graph.linearize(values);
}
auto counting(const NonlinearFactorGraph& g,const Values& v,void* p) {
  ++*static_cast<int*>(p);return g.linearize(v);
}
TEST(UpdateBatchScope, ReentrantFallbackNestedRejectionAndThreadIsolation) {
  const auto v=poses(3);auto g=cpuGraph(v);int calls=0,thread_calls=0;
  {ScopedLinearizer scope(counting,&calls);
   g.linearize(v);EXPECT_EQ(calls,1);
   EXPECT_THROW(ScopedLinearizer nested(counting,&calls),std::logic_error);
   std::thread other([&]{g.linearize(v);ScopedLinearizer local(counting,&thread_calls);g.linearize(v);});
   other.join();EXPECT_EQ(calls,1);EXPECT_EQ(thread_calls,1);
   g.linearize(v);EXPECT_EQ(calls,2);}
  g.linearize(v);EXPECT_EQ(calls,2);
}
TEST(UpdateBatchScope, ThrowAndBadShapeRestoreScope) {
  const auto v=poses(3);auto g=cpuGraph(v);
  auto fail=[](const auto&,const auto&,void*)->GaussianFactorGraph::shared_ptr{throw std::runtime_error("test");};
  {ScopedLinearizer scope(fail);EXPECT_THROW(g.linearize(v),std::runtime_error);}
  auto bad=[](const auto&,const auto&,void*){return std::make_shared<GaussianFactorGraph>();};
  {ScopedLinearizer scope(bad);EXPECT_THROW(g.linearize(v),std::invalid_argument);}
  nearMatrix(g.linearize(v)->augmentedHessian(),candidate(g,v)->augmentedHessian(),0,0);
}
struct RecursivePrior : PriorFactor<Pose3> {
  mutable size_t evaluations=0;
  RecursivePrior():PriorFactor<Pose3>(gtsam::symbol_shorthand::X(0),Pose3(),denseNoise()){}
  GaussianFactor::shared_ptr linearize(const Values& v) const override {
    ++evaluations;NonlinearFactorGraph nested;
    nested.emplace_shared<PriorFactor<Pose3>>(gtsam::symbol_shorthand::X(0),Pose3(),denseNoise());
    return nested.linearize(v)->front();
  }
};
TEST(UpdateBatchScope, UnknownStatefulFactorRecursionRunsOrdinaryExactlyOnce) {
  auto v=poses(1);NonlinearFactorGraph g;auto f=std::make_shared<RecursivePrior>();g.push_back(f);
  Stats s;candidate(g,v,&s);EXPECT_EQ(s.requests,1u);EXPECT_EQ(f->evaluations,1u);
  EXPECT_EQ(s.fallback[static_cast<size_t>(Reason::UnknownCpu)],1u);
}
TEST(UpdateBatchScope, CpuOnlyExactAndDefaultOff) {
  unsetenv("CBS_E1B_LOCAL_UPDATE_GPU_BATCHING");EXPECT_FALSE(glim::e1b4a::enabled());
  setenv("CBS_E1B_LOCAL_UPDATE_GPU_BATCHING","1",1);EXPECT_TRUE(glim::e1b4a::enabled());
  unsetenv("CBS_E1B_LOCAL_UPDATE_GPU_BATCHING");
  auto v=poses(4);auto g=cpuGraph(v);Stats s;
  nearMatrix(g.linearize(v)->augmentedHessian(),candidate(g,v,&s)->augmentedHessian(),0,0);
  EXPECT_EQ(s.prepared_factors,0u);EXPECT_EQ(s.fallback[0],1u);
}
#ifdef GTSAM_POINTS_USE_CUDA
void exactSlots(const GaussianFactorGraph& a,const GaussianFactorGraph& b) {
  ASSERT_EQ(a.size(),b.size());
  for(size_t i=0;i<a.size();++i){ASSERT_EQ(bool(a[i]),bool(b[i]));if(a[i])EXPECT_EQ(a[i]->keys(),b[i]->keys());}
}
TEST_F(LocalLinearizerGpu, ScopedIndependentSharedHistoriesAndFullCovariance) {
  const auto start=poses(5);auto old=graph(start,24),batched=graph(start,24);
  // Within each history MAIN and LOCAL share factors; oracle/candidate never do.
  auto local_old=old,local_new=batched;
  for(size_t i=7;i<old.size();i+=3){local_old[i].reset();local_new[i].reset();}
  Stats stats;
  for(int step=0;step<12;++step){
    const auto a=poses(5,.008*step),b=poses(5,-.005*step);
    old.linearize(a);batched.linearize(a);
    auto x=local_old.linearize(b),y=candidate(local_new,b,&stats);exactSlots(*x,*y);
    nearMatrix(x->augmentedHessian(),y->augmentedHessian(),1e-8,1e-6);
    Marginals m(*x,b),n(*y,b);
    nearMatrix(m.jointMarginalCovariance(b.keys()).fullMatrix(),n.jointMarginalCovariance(b.keys()).fullMatrix(),1e-10,1e-6);
    nearMatrix(relativeCov(m,b),relativeCov(n,b),1e-10,1e-6);
    nearMatrix(old.linearize(a)->augmentedHessian(),batched.linearize(a)->augmentedHessian(),1e-8,1e-6);
    EXPECT_DOUBLE_EQ(old.error(b),batched.error(b));
    for(auto& f:batched)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))EXPECT_FALSE(gpu->has_cached_linearization());
  }
  EXPECT_EQ(stats.batched_graphs,12u);EXPECT_GT(stats.prepared_factors,100u);
}
TEST_F(LocalLinearizerGpu, ScopedFallbackDuplicatePendingAndUnknownCpu) {
  auto v=poses(4);auto a=graph(v,8),b=graph(v,8);Stats s;
  a.push_back(a.back());b.push_back(b.back());
  nearMatrix(a.linearize(v)->augmentedHessian(),candidate(b,v,&s)->augmentedHessian(),0,0);
  EXPECT_EQ(s.fallback[static_cast<size_t>(Reason::Duplicate)],1u);a.resize(a.size()-1);b.resize(b.size()-1);
  for(auto* g:{&a,&b}){gtsam_points::NonlinearFactorSetGPU native;native.clear_counts();native.add(*g);native.linearize(v);}
  nearMatrix(a.linearize(v)->augmentedHessian(),candidate(b,v,&s)->augmentedHessian(),0,0);
  EXPECT_EQ(s.fallback[static_cast<size_t>(Reason::Pending)],1u);
  auto previous=b[0];b[0]=std::make_shared<ThrowPrior>();
  EXPECT_THROW(candidate(b,v,&s),std::runtime_error);
  EXPECT_EQ(s.fallback[static_cast<size_t>(Reason::UnknownCpu)],1u);
  b[0]=previous;nearMatrix(a.linearize(v)->augmentedHessian(),candidate(b,v,&s)->augmentedHessian(),1e-8,1e-6);
}
TEST_F(LocalLinearizerGpu, ScopedMissingValuesAndPostGpuCpuExceptionRecover) {
  auto v=poses(4);auto a=graph(v,8),b=graph(v,8);auto bad=v;bad.erase(X(3));
  EXPECT_ANY_THROW(candidate(b,bad));
  // Supported exact CPU factor, but wrong Value type. GPU batches drain before
  // this CPU exception, unlike preflight failure; next ordinary solve must agree.
  b.emplace_shared<PriorFactor<Vector3>>(X(0),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,1));
  EXPECT_ANY_THROW(candidate(b,v));b.resize(b.size()-1);
  for(auto& f:b)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))EXPECT_FALSE(gpu->has_cached_linearization());
  auto next=poses(4,.03);
  nearMatrix(a.linearize(next)->augmentedHessian(),b.linearize(next)->augmentedHessian(),1e-8,1e-6);
}
TEST_F(LocalLinearizerGpu, ScopedUnaryFactor) {
  auto v=poses(3);auto a=cpuGraph(v),b=cpuGraph(v);
  for(auto* g:{&a,&b}){auto [stream,buffer]=streams->get_stream_buffer();g->push_back(std::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(Pose3(),X(2),map,cloud,stream,buffer));}
  Stats s;auto x=a.linearize(v),y=candidate(b,v,&s);exactSlots(*x,*y);
  nearMatrix(x->augmentedHessian(),y->augmentedHessian(),1e-8,1e-6);EXPECT_EQ(s.prepared_factors,1u);
}
TEST_F(LocalLinearizerGpu, ScopedUnknownGpuFallsBack) {
  auto v=poses(3);auto g=graph(v,4);auto [stream,buffer]=streams->get_stream_buffer();
  g.emplace_shared<DerivedGpu>(X(1),X(2),map,cloud,stream,buffer);
  Stats s;auto x=g.linearize(v),y=candidate(g,v,&s);exactSlots(*x,*y);
  nearMatrix(x->augmentedHessian(),y->augmentedHessian(),0,0);
  EXPECT_EQ(s.fallback[static_cast<size_t>(Reason::UnknownGpu)],1u);
}
#endif
}
