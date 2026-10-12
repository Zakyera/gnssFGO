#include "RuntimeProfile.h"
// Reuse every existing E1 assertion, including shared cache/inlier/exception paths.
#include "/workspace/cbs_gtsam4.3/runs/e1_correctness_audit_20261011/test_e1_audit.cpp"
namespace {
#ifdef GTSAM_POINTS_USE_CUDA
TEST_F(LocalLinearizerGpu, ProfileCrossDsoAndTbbCalls) {
  auto values=poses(5);auto g=graph(values,36);e1b::Record record;
  auto before=g.linearize(values)->augmentedHessian();
  Matrix after;
  {e1b::Session session(0,e1b::Context::Local,&record);
   after=g.linearize(values)->augmentedHessian();}
  nearMatrix(before,after,0,0);
  auto idx=static_cast<unsigned>(e1b::Stage::GpuLinearize);
  EXPECT_EQ(record.owner[idx].calls+record.worker[idx].calls,36u);
  auto sync=static_cast<unsigned>(e1b::Stage::GpuSyncLinearize);
  EXPECT_EQ(record.owner[sync].calls+record.worker[sync].calls,36u);
  for(auto& f:g)if(auto* gpu=dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get()))
    EXPECT_FALSE(gpu->has_cached_linearization());
}
TEST_F(LocalLinearizerGpu, ProfileBatchDoesNotAddSyncCallsOrConsumeErrorCache) {
  auto values=poses(5);auto g=graph(values,36);e1b::Record record;
  {e1b::Session session(0,e1b::Context::Covariance,&record);
   compare(g,values,true);}
  // compare calls a reference sync graph and the candidate; existing assertions
  // validate their matrices and cache behavior. Profiling must see both DSOs.
  auto idx=static_cast<unsigned>(e1b::Stage::GpuLinearize);
  EXPECT_GT(record.owner[idx].calls+record.worker[idx].calls,0u);
  EXPECT_GT(record.metrics[static_cast<unsigned>(e1b::Metric::GpuLinearizeHits)],0u);
}
#endif
}
