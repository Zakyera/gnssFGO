#include <cuda_runtime.h>
#include "LocalUpdateCheck.h"
#include "/workspace/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012/authoring/test_dispatch.cpp"
#include <iomanip>

namespace {
using e1b4check::FactorCopies;
using e1b4check::LocalUpdateCheck;
using gtsam_points::ExactGpuSnapshot;
using GPU=gtsam_points::IntegratedVGICPFactorGPU;
const GPU& gpu(const NonlinearFactor::shared_ptr& f){return dynamic_cast<const GPU&>(*f);}
std::vector<unsigned char> state(const NonlinearFactor::shared_ptr& f){return ExactGpuSnapshot::state(gpu(f));}

TEST_F(LocalLinearizerGpu, SnapshotFreshUsedUnaryBinaryExactAndIndependent) {
  for(bool unary:{false,true}) {
    auto f=unary?NonlinearFactor::shared_ptr(std::make_shared<GPU>(Pose3(),X(1),map,cloud)):
                 factor(X(0),X(1));
    auto* p=dynamic_cast<GPU*>(f.get());
    p->set_inlier_update_thresh(.007,.008);p->set_enable_offloading(true);
    auto original=state(f);
    FactorCopies copies;auto clone=copies.copy(f);
    EXPECT_EQ(original,state(clone));EXPECT_NE(f.get(),clone.get());
    EXPECT_NE(p->get_target().get(),gpu(clone).get_target().get());
    EXPECT_NE(p->get_target()->buckets,gpu(clone).get_target()->buckets);
    auto v=poses(2,.005);
    auto h=clone->linearize(v)->augmentedInformation();
    EXPECT_EQ(original,state(f)); // Independent cache and inlier state.
    nearMatrix(f->linearize(v)->augmentedInformation(),h,0,0);
    EXPECT_EQ(state(f),state(clone));
    EXPECT_DOUBLE_EQ(f->error(v),clone->error(v));
    FactorCopies second;auto used=second.copy(f);EXPECT_EQ(state(f),state(used));
    auto next=poses(2,.018);
    nearMatrix(f->linearize(next)->augmentedInformation(),used->linearize(next)->augmentedInformation(),0,0);
    EXPECT_EQ(state(f),state(used));
  }
}

TEST_F(LocalLinearizerGpu, SnapshotPendingNativeCachesCompactionAndConsumption) {
  auto v=poses(4);auto g=graph(v,12);
  gtsam_points::NonlinearFactorSetGPU native;
  native.clear_counts();native.add(g);native.linearize(v);
  std::vector<std::vector<unsigned char>> original;
  for(const auto& f:g)if(dynamic_cast<GPU*>(f.get()))original.push_back(state(f));
  FactorCopies a,b;auto x=a.graph(g),y=b.graph(g);
  auto hx=x.linearize(v),hy=candidate(y,v);
  nearMatrix(hx->augmentedHessian(),hy->augmentedHessian(),0,0);
  size_t n=0;for(const auto& f:g)if(dynamic_cast<GPU*>(f.get()))EXPECT_EQ(original[n++],state(f));
  nearMatrix(g.linearize(v)->augmentedHessian(),hx->augmentedHessian(),0,0);
  auto gx=g.error(v),xx=x.error(v),yx=y.error(v);
  EXPECT_DOUBLE_EQ(gx,xx);EXPECT_DOUBLE_EQ(gx,yx);
  for(size_t i=0;i<g.size();++i)if(dynamic_cast<GPU*>(g[i].get())) {
    EXPECT_EQ(state(g[i]),state(x[i]));EXPECT_EQ(state(x[i]),state(y[i]));
  }
}

TEST_F(LocalLinearizerGpu, SnapshotPreservesDenseGeometryAndSurfaceValidation) {
  auto c=std::dynamic_pointer_cast<gtsam_points::PointCloudGPU>(cloud);
  ASSERT_TRUE(c);
  std::vector<Eigen::Vector3f> normals(c->size(),Eigen::Vector3f(0,0,1));
  c->add_normals(normals);
  auto f=factor(X(0),X(1));auto* p=dynamic_cast<GPU*>(f.get());
  p->set_enable_surface_validation(true);p->set_inlier_update_thresh(.1,.1);
  auto v=poses(2);f->linearize(v);FactorCopies cp;auto q=cp.copy(f);
  EXPECT_EQ(state(f),state(q));
  nearMatrix(f->linearize(v)->augmentedInformation(),q->linearize(v)->augmentedInformation(),0,0);
  // Mutate ORIGINAL target bytes; copied target must remain bit-identical.
  auto before=state(q);auto m=p->get_target();
  Eigen::Vector3f mean;ASSERT_EQ(cudaMemcpy(&mean,m->voxel_means,sizeof(mean),cudaMemcpyDeviceToHost),cudaSuccess);
  auto saved=mean;mean.x()+=1;
  ASSERT_EQ(cudaMemcpy(m->voxel_means,&mean,sizeof(mean),cudaMemcpyHostToDevice),cudaSuccess);
  EXPECT_EQ(before,state(q));EXPECT_NE(state(f),state(q));
  ASSERT_EQ(cudaMemcpy(m->voxel_means,&saved,sizeof(saved),cudaMemcpyHostToDevice),cudaSuccess);
}

TEST_F(LocalLinearizerGpu, SnapshotIdentityNullsUnknownAndMissingValuesFailClosed) {
  auto v=poses(3);auto g=graph(v,4);g.push_back(g.back());g.push_back(NonlinearFactor::shared_ptr());
  FactorCopies cp;auto h=cp.graph(g);
  EXPECT_EQ(h[h.size()-2],h[h.size()-3]);EXPECT_FALSE(h.back());
  EXPECT_NE(h[h.size()-2],g[g.size()-2]);
  auto fingerprint=state(g[g.size()-2]);auto bad=v;bad.erase(X(2));
  EXPECT_ANY_THROW(h.linearize(bad));EXPECT_EQ(fingerprint,state(g[g.size()-2]));
  auto unknown=std::make_shared<RecursivePrior>();
  EXPECT_THROW(cp.copy(unknown),std::invalid_argument);EXPECT_EQ(unknown->evaluations,0u);
  auto [stream,buffer]=streams->get_stream_buffer();
  EXPECT_THROW(cp.copy(std::make_shared<DerivedGpu>(X(0),X(1),map,cloud,stream,buffer)),std::invalid_argument);
}

TEST_F(LocalLinearizerGpu, SnapshotNonresidentGeometryRejectedWithoutTouchingInput) {
  auto f=factor(X(0),X(1));ASSERT_TRUE(map->offload_gpu());
  EXPECT_THROW(ExactGpuSnapshot::copy(gpu(f)),std::invalid_argument);
  EXPECT_FALSE(map->loaded_on_gpu());ASSERT_TRUE(map->reload_gpu());
}

TEST_F(LocalLinearizerGpu, SnapshotGeometrySharedOnlyWithinOneBranch) {
  NonlinearFactorGraph g;g.push_back(factor(X(0),X(1)));g.push_back(factor(X(1),X(2)));
  FactorCopies a,b;auto x=a.graph(g),y=b.graph(g);
  EXPECT_EQ(gpu(x[0]).get_target(),gpu(x[1]).get_target());
  EXPECT_NE(gpu(x[0]).get_target(),gpu(y[0]).get_target());
  EXPECT_NE(gpu(x[0]).get_target(),gpu(g[0]).get_target());
}
TEST_F(LocalLinearizerGpu, SnapshotZeroInliersAndOriginalLifetime) {
  auto v=poses(2);v.update(X(1),Pose3(Rot3(),Point3(1000,0,0)));
  auto f=factor(X(0),X(1));NonlinearFactorGraph g;g.push_back(f);
  gtsam_points::NonlinearFactorSetGPU native;native.clear_counts();native.add(g);native.linearize(v);
  EXPECT_EQ(gpu(f).num_inliers(),0);
  auto c=ExactGpuSnapshot::copy(gpu(f));EXPECT_EQ(state(f),state(c));
  auto h=f->linearize(v)->augmentedInformation();
  nearMatrix(c->linearize(v)->augmentedInformation(),h,0,0);
  auto d=ExactGpuSnapshot::copy(gpu(c));c.reset();
  nearMatrix(d->linearize(v)->augmentedInformation(),h,0,0);
}
TEST(SnapshotCpu, DampingAndLinearContainerRetainTypeAndOwnMatrices) {
  NonlinearFactorGraph g;g.emplace_shared<gtsam_points::LinearDampingFactor>(X(0),6,.1);
  g.emplace_shared<LinearContainerFactor>(HessianFactor(X(0),Matrix6::Identity(),Vector6::Zero(),0));
  FactorCopies cp;auto c=cp.graph(g);
  for(size_t i=0;i<g.size();++i) {
    EXPECT_EQ(typeid(*g[i]),typeid(*c[i]));
    auto* a=dynamic_cast<LinearContainerFactor*>(g[i].get());
    auto* b=dynamic_cast<LinearContainerFactor*>(c[i].get());
    EXPECT_NE(a->factor(),b->factor());
    nearMatrix(a->factor()->augmentedInformation(),b->factor()->augmentedInformation(),0,0);
  }
}
TEST(SnapshotCpu, UnsupportedCheckpointDiagnosticsRejected) {
  e1b4check::Smoother s(.5);s.captureBadUpdateStagesOnce();FactorCopies cp;
  EXPECT_THROW(cbs::LocalSnapshotAccess::copy(s,cp),std::invalid_argument);
}

struct SnapshotSolver : LocalLinearizerGpu {
  void run(bool use_gpu,bool inject_failure) {
    using symbol_shorthand::V;using symbol_shorthand::B;
    cbs::BPSAM::Params p;p.robot_id='g';p.sam_params_.relinearizeSkip=1;
    p.sam_params_.relinearizeThreshold=.01;p.sam_params_.findUnusedFactorSlots=true;
    p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
    e1b4check::Smoother ordinary(.65,p);
    auto velocity_noise=noiseModel::Isotropic::Sigma(3,.1);
    auto bias_noise=noiseModel::Isotropic::Sigma(6,.01);
    auto imu=PreintegrationParams::MakeSharedU(9.81);
    imu->accelerometerCovariance=Matrix3::Identity()*.01;
    imu->gyroscopeCovariance=Matrix3::Identity()*.001;
    imu->integrationCovariance=Matrix3::Identity()*1e-6;
    e1b4check::Differences maximum;size_t batches=0,removed=0,retired=0;
    FactorCopies history_a_factors,history_b_factors;
    std::unique_ptr<e1b4check::Smoother> history_a,history_b;
    NonlinearFactor::shared_ptr replaceable;
    for(size_t i=0;i<24;++i) {
      Values v;Pose3 pose(Rot3::Rz(.0002*i),Point3(.001*i,0,0));
      v.insert(X(i),pose);v.insert(V(i),Vector3(.01,0,0));v.insert(B(i),imuBias::ConstantBias());
      NonlinearFactorGraph g;FactorIndices remove;
      FixedLagSmoother::KeyTimestampMap stamps{{X(i),.1*i},{V(i),.1*i},{B(i),.1*i}};
      if(!i) {
        g.emplace_shared<PriorFactor<Pose3>>(X(i),pose,denseNoise());
        g.emplace_shared<PriorFactor<Vector3>>(V(i),Vector3(.01,0,0),velocity_noise);
        g.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(i),imuBias::ConstantBias(),bias_noise);
      }else {
        PreintegratedImuMeasurements pim(imu,imuBias::ConstantBias());
        for(int j=0;j<10;++j)pim.integrateMeasurement(Vector3(0,0,9.81),Vector3(0,0,.002),.01);
        g.emplace_shared<ImuFactor>(X(i-1),V(i-1),X(i),V(i),B(i-1),pim);
        g.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(i-1),B(i),imuBias::ConstantBias(),bias_noise);
        g.emplace_shared<BetweenFactor<Pose3>>(X(i-1),X(i),Pose3(Rot3::Rz(.0002),Point3(.001,0,0)),denseNoise());
        if(use_gpu)for(size_t j=0;j<4;++j)g.push_back(factor(X(i-1),X(i)));
        if(replaceable && i%3==0) {
          for(size_t slot=0;slot<ordinary.getFactors().size();++slot)
            if(ordinary.getFactors()[slot]==replaceable)remove.push_back(slot);
          if(!remove.empty()) {
            g.emplace_shared<BetweenFactor<Pose3>>(X(i-2),X(i-1),
                Pose3(Rot3::Rz(.0002),Point3(.001,0,0)),denseNoise());
            ++removed;
          }
        }
        replaceable=g.back();
      }
      // Model real MAIN/LOCAL sharing: prepare native GPU caches at an actual
      // preupdate Values boundary, before copying either diagnostic branch.
      std::unique_ptr<gtsam_points::NonlinearFactorSetGPU> native;
      if(use_gpu && i%4==0) {
        native=std::make_unique<gtsam_points::NonlinearFactorSetGPU>();
        Values initialized=ordinary.calculateEstimate();initialized.insert(v);
        native->clear_counts();native->add(g);native->linearize(initialized);
      }
      if(inject_failure && i==12) {
        auto bad=v;bad.erase(X(i));
        auto before=ordinary.getLinearizationPoint();auto time=ordinary.timestamps();
        EXPECT_ANY_THROW(LocalUpdateCheck(ordinary,g,bad,stamps,remove));
        EXPECT_TRUE(before.equals(ordinary.getLinearizationPoint(),0));EXPECT_EQ(time,ordinary.timestamps());
      }
      LocalUpdateCheck check(ordinary,g,v,stamps,remove);
      // Continue two independent solver histories from one mature checkpoint,
      // rather than resnapshotting their optimizer states every update.
      if(i==6) {
        history_a=cbs::LocalSnapshotAccess::copy(ordinary,history_a_factors);
        history_b=cbs::LocalSnapshotAccess::copy(ordinary,history_b_factors);
      }
      if(history_a) {
        history_a->update(history_a_factors.graph(g),v,stamps,remove);
        auto next=history_b_factors.graph(g);
        {ScopedLinearizer scope(glim::e1b4a::linearize);
         history_b->update(next,v,stamps,remove);}
      }
      ordinary.update(g,v,stamps,remove);
      auto fidelity=check.finish(ordinary),paired=check.paired();batches+=check.stats().batched_graphs;
      std::vector<e1b4check::Differences> differences{fidelity,paired};
      if(history_a) {
        differences.push_back(e1b4check::compare(*history_a,*history_b));
        differences.push_back(e1b4check::compare(ordinary,*history_a));
      }
      for(auto d:differences) {
        maximum.position=std::max(maximum.position,d.position);
        maximum.orientation=std::max(maximum.orientation,d.orientation);
        maximum.velocity=std::max(maximum.velocity,d.velocity);maximum.bias=std::max(maximum.bias,d.bias);
        maximum.gaussian=std::max(maximum.gaussian,d.gaussian);maximum.covariance=std::max(maximum.covariance,d.covariance);
        maximum.relative=std::max(maximum.relative,d.relative);
      }
      if(i>7 && !ordinary.getLinearizationPoint().exists(X(0)))++retired;
    }
    EXPECT_GT(removed,0u);EXPECT_GT(retired,0u);if(use_gpu)EXPECT_GT(batches,0u);
    std::cout<<std::setprecision(17)<<"SNAPSHOT_SOLVER,"<<use_gpu<<','<<batches<<','<<removed<<','<<retired<<','
      <<maximum.position<<','<<maximum.orientation<<','<<maximum.velocity<<','<<maximum.bias<<','
      <<maximum.gaussian<<','<<maximum.covariance<<','<<maximum.relative<<'\n';
  }
};
TEST_F(SnapshotSolver, ExactBoundaryFidelityBatchingRetirementReplacement){run(true,true);}
TEST_F(SnapshotSolver, DeterministicRepeatedBoundaryCheck){run(true,false);run(true,false);}
struct CpuOnlyRunner:SnapshotSolver {void TestBody() override{}};
TEST(SnapshotCpu, CheckpointFidelityAndFailureIsolation){CpuOnlyRunner runner;runner.run(false,true);}
TEST(SnapshotChecks, NumericalComparatorRejectsMismatchAndNonfinite) {
  Matrix a=Matrix::Identity(6,6),b=a;b(0,0)=2;
  EXPECT_THROW(e1b4check::matrixDiff(a,b,1e-10,1e-6),std::runtime_error);
  b=a;b(0,0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(e1b4check::matrixDiff(a,b,1e-10,1e-6),std::runtime_error);
}
}
