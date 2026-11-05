#include "MainGpuBatch.h"
#include "ExactGpuSnapshot.h"
// Retain the original focused assertions unchanged, but run the new scope tests
// in a separate process/filter so their reference has no global hook factory.
#include "/workspace/cbs_gtsam4.3/src/glim_ros1/test/cbs_local_graph_linearizer_test.cpp"
#include <thread>
#include <iomanip>

namespace {
using Main = gtsam_points::ISAM2Ext;
using GPU = gtsam_points::IntegratedVGICPFactorGPU;
using gtsam_points::ExactGpuSnapshot;
using e1b4check::FactorCopies;

auto evaluate(Main& owner, const NonlinearFactorGraph& g, const Values& v,
              cpk16::Site site = cpk16::Site::FullGraph) {
  auto out = cpk16::tryLinearize(&owner, g, v, site);
  return out ? out : g.linearize(v);
}
void compareSlots(const GaussianFactorGraph& a, const GaussianFactorGraph& b) {
  ASSERT_EQ(a.size(), b.size());
  for (size_t i=0; i<a.size(); ++i) {
    ASSERT_EQ(bool(a[i]),bool(b[i]));
    if(a[i]) EXPECT_EQ(a[i]->keys(),b[i]->keys());
  }
}
void compareGpuState(const NonlinearFactorGraph& a, const NonlinearFactorGraph& b) {
  ASSERT_EQ(a.size(),b.size());
  for(size_t i=0;i<a.size();++i) {
    if(auto* x=dynamic_cast<GPU*>(a[i].get())) {
      auto* y=dynamic_cast<GPU*>(b[i].get());ASSERT_NE(y,nullptr);
      EXPECT_EQ(ExactGpuSnapshot::state(*x),ExactGpuSnapshot::state(*y));
      EXPECT_FALSE(y->has_cached_linearization());
    }
  }
}
void metric(const char* name, const Matrix& a, const Matrix& b, double abs, double rel) {
  nearMatrix(a,b,abs,rel);
  std::cout<<std::setprecision(17)<<"CPK16_ERROR "<<name<<" max_abs="
           <<(a-b).cwiseAbs().maxCoeff()<<" bound="<<abs+rel*a.cwiseAbs().maxCoeff()<<"\n";
}
TEST(MainBatchCpu, DisabledExplicitIdentityThreadAndNestedScope) {
  unsetenv("CPK16_MAIN_GPU_BATCHING");EXPECT_FALSE(cpk16::requested());
  Main a,b;auto v=poses(3);auto g=cpuGraph(v);cpk16::Stats stats;
  EXPECT_FALSE(cpk16::tryLinearize(&a,g,v,cpk16::Site::NewFactors));
  { cpk16::Scope scope(&a,false,&stats);EXPECT_FALSE(cpk16::active(&a)); }
  {
    cpk16::Scope scope(&a,true,&stats);
    EXPECT_TRUE(cpk16::active(&a));EXPECT_FALSE(cpk16::active(&b));
    EXPECT_THROW(cpk16::Scope nested(&b,true),std::logic_error);
    std::thread other([&]{EXPECT_FALSE(cpk16::active(&a));});other.join();
    metric("cpu",g.linearize(v)->augmentedHessian(),
           evaluate(a,g,v)->augmentedHessian(),1e-12,1e-12);
  }
  EXPECT_FALSE(cpk16::active(&a));
  EXPECT_EQ(stats.work.requests,1u);EXPECT_EQ(stats.work.prepared_factors,0u);
  EXPECT_EQ(stats.work.fallback[static_cast<size_t>(glim::e1b4a::Reason::CpuOnly)],1u);
}
TEST(MainBatchCpu, UnsupportedOptimizerAndErrorHooksFailBeforeScope) {
  ISAM2Params params;params.setOptimizationParams(ISAM2DoglegParams());Main dogleg(params);
  EXPECT_THROW(cpk16::Scope scope(&dogleg,true),std::invalid_argument);
  params.setOptimizationParams(ISAM2GaussNewtonParams());params.evaluateNonlinearError=true;
  Main error(params);EXPECT_THROW(cpk16::Scope scope(&error,true),std::invalid_argument);
  Main normal;{cpk16::Scope scope(&normal,true);EXPECT_TRUE(cpk16::active(&normal));}
}
class MainBatchGpu : public LocalLinearizerGpu {
 protected:
  void SetUp() override {
    // Same deterministic fixture geometry, intentionally NO register_hook().
    std::vector<Eigen::Vector4d> pts;std::vector<Eigen::Matrix4d> covs;
    for(int x=0;x<12;++x)for(int y=0;y<12;++y)for(int z=0;z<6;++z){
      pts.emplace_back(.31*x+.001*y*y,.32*y,.33*z+.002*x*x,1);
      auto c=Eigen::Matrix4d(Eigen::Matrix4d::Identity()*.02);c(3,3)=0;covs.push_back(c);
    }
    auto cpu=std::make_shared<gtsam_points::PointCloudCPU>();
    cpu->add_points(pts);cpu->add_covs(covs);
    cloud=gtsam_points::PointCloudGPU::clone(*cpu);
    map=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);map->insert(*cloud);
    streams.reset(new gtsam_points::StreamTempBufferRoundRobin(4));
  }
};
TEST_F(MainBatchGpu, IndependentHistoriesEquationsCovarianceInliersAndErrorCache) {
  auto v=poses(5);auto original=graph(v,16);
  original.emplace_shared<GPU>(Pose3(),X(3),map,cloud);
  std::rotate(original.begin(),original.begin()+3,original.end());
  for(auto& f:original)if(auto* p=dynamic_cast<GPU*>(f.get()))
    p->set_inlier_update_thresh(.007,.008);
  FactorCopies ac,bc;auto a=ac.graph(original),b=bc.graph(original);
  Main owner;cpk16::Stats stats;cpk16::Scope scope(&owner,true,&stats);
  for(int step=0;step<28;++step) {
    auto values=poses(5,.035*std::sin(.57*step));
    if(step%7==3) {
      // Explicitly prepare pre-existing native caches in BOTH independent
      // branches. Compatibility must not overwrite their error caches.
      for(auto* g:{&a,&b}) {
        gtsam_points::NonlinearFactorSetGPU native;native.clear_counts();
        native.add(*g);native.linearize(values);g->linearize(values);
      }
    }
    auto x=a.linearize(values);
    auto y=evaluate(owner,b,values,static_cast<cpk16::Site>(step%4));
    compareSlots(*x,*y);
    metric("gaussian",x->augmentedHessian(),y->augmentedHessian(),1e-8,1e-6);
    Marginals m(*x,values),n(*y,values);
    metric("joint_covariance",m.jointMarginalCovariance(values.keys()).fullMatrix(),
           n.jointMarginalCovariance(values.keys()).fullMatrix(),1e-10,1e-6);
    metric("relative_covariance",relativeCov(m,values),relativeCov(n,values),1e-10,1e-6);
    compareGpuState(a,b);
    auto next=poses(5,-.04*std::cos(.31*step));
    const double ea=a.error(next),eb=b.error(next);
    EXPECT_LE(std::abs(ea-eb),1e-8+1e-6*std::abs(ea));
    std::cout<<std::setprecision(17)<<"CPK16_ERROR residual max_abs="<<std::abs(ea-eb)<<"\n";
    compareGpuState(a,b);
  }
  EXPECT_EQ(stats.work.batched_graphs,28u);
  EXPECT_EQ(stats.work.prepared_factors,28u*17u);
  for(auto n:stats.sites)EXPECT_EQ(n,7u);
}
TEST_F(MainBatchGpu, DisabledPathAndPendingDuplicateFallbackPreserveState) {
  const auto v=poses(4);auto base=graph(v,8);FactorCopies ac,bc;
  auto a=ac.graph(base),b=bc.graph(base);Main owner;cpk16::Stats stats;
  { cpk16::Scope off(&owner,false,&stats);
    metric("disabled",a.linearize(v)->augmentedHessian(),
           evaluate(owner,b,v)->augmentedHessian(),0,0);compareGpuState(a,b); }
  cpk16::Scope on(&owner,true,&stats);
  a.push_back(a.back());b.push_back(b.back());
  metric("duplicate",a.linearize(v)->augmentedHessian(),
         evaluate(owner,b,v)->augmentedHessian(),0,0);compareGpuState(a,b);
  a.resize(a.size()-1);b.resize(b.size()-1);
  for(auto* g:{&a,&b}) {
    gtsam_points::NonlinearFactorSetGPU native;native.clear_counts();native.add(*g);native.linearize(v);
  }
  auto next=poses(4,.05);
  metric("pending",a.linearize(next)->augmentedHessian(),
         evaluate(owner,b,next)->augmentedHessian(),0,0);compareGpuState(a,b);
  EXPECT_DOUBLE_EQ(a.error(next),b.error(next));compareGpuState(a,b);
  EXPECT_EQ(stats.work.prepared_factors,0u);
  EXPECT_EQ(stats.work.fallback[static_cast<size_t>(glim::e1b4a::Reason::Duplicate)],1u);
  EXPECT_EQ(stats.work.fallback[static_cast<size_t>(glim::e1b4a::Reason::Pending)],1u);
}
TEST_F(MainBatchGpu, InvalidValuesAndCpuFailureLeaveNoPreparedCache) {
  auto v=poses(4);auto base=graph(v,8);FactorCopies ac,bc;
  auto a=ac.graph(base),b=bc.graph(base);Main owner;cpk16::Stats stats;
  cpk16::Scope scope(&owner,true,&stats);
  auto missing=v;missing.erase(X(2));EXPECT_ANY_THROW(evaluate(owner,b,missing));
  compareGpuState(a,b);EXPECT_TRUE(cpk16::active(&owner));
  b.emplace_shared<PriorFactor<Vector3>>(X(0),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,1));
  EXPECT_ANY_THROW(evaluate(owner,b,v));b.resize(b.size()-1);
  for(auto& f:b)if(auto* p=dynamic_cast<GPU*>(f.get()))EXPECT_FALSE(p->has_cached_linearization());
  auto next=poses(4,.03);
  metric("after_exception",a.linearize(next)->augmentedHessian(),
         evaluate(owner,b,next)->augmentedHessian(),1e-8,1e-6);
  compareGpuState(a,b);EXPECT_TRUE(cpk16::active(&owner));
}
TEST_F(MainBatchGpu, UnknownCpuAndGpuTakeReferenceRouteWithoutPreparation) {
  auto v=poses(3);auto a=graph(v,4),b=graph(v,4);
  auto [s1,t1]=streams->get_stream_buffer();auto [s2,t2]=streams->get_stream_buffer();
  a.emplace_shared<DerivedGpu>(X(0),X(2),map,cloud,s1,t1);
  b.emplace_shared<DerivedGpu>(X(0),X(2),map,cloud,s2,t2);
  Main owner;cpk16::Stats stats;cpk16::Scope scope(&owner,true,&stats);
  metric("unknown_gpu",a.linearize(v)->augmentedHessian(),
         evaluate(owner,b,v)->augmentedHessian(),0,0);
  b[0]=std::make_shared<ThrowPrior>();EXPECT_THROW(evaluate(owner,b,v),std::runtime_error);
  EXPECT_EQ(stats.work.prepared_factors,0u);
  EXPECT_EQ(stats.work.fallback[static_cast<size_t>(glim::e1b4a::Reason::UnknownGpu)],1u);
  EXPECT_EQ(stats.work.fallback[static_cast<size_t>(glim::e1b4a::Reason::UnknownCpu)],1u);
}
}  // namespace
