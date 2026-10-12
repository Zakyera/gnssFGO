#include "test_dispatch.cpp"
#include "/workspace/cbs_gtsam4.3/runs/e1b_local_profile_20261012/test_solver.cpp"

namespace {
using symbol_shorthand::V;
using symbol_shorthand::B;
struct SolverSnapshot {
  Values estimate,theta;
  Matrix covariance,relative;
  std::vector<KeyVector> slots;
  std::vector<std::string> types;
  size_t relinearized=0,reeliminated=0;
};
struct SolverRun {
  std::vector<SolverSnapshot> snapshots;
  std::vector<double> milliseconds;
  std::vector<e1b::Record> records;
  Stats stats;
};
struct UpdateBatchSolver : LocalLinearizerGpu {
  SolverRun execute(bool batch,size_t per_step,bool capture=true,double threshold=0.0,bool retry=false) {
    cbs::BPSAM::Params params;params.robot_id='g';
    params.sam_params_.relinearizeSkip=1;params.sam_params_.relinearizeThreshold=threshold;
    params.sam_params_.findUnusedFactorSlots=true;
    params.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
    cbs::IncrementalFixedLagBpsamSmoother solver(.65,params);
    auto imu=PreintegrationParams::MakeSharedU(9.81);
    imu->accelerometerCovariance=Matrix3::Identity()*.01;
    imu->gyroscopeCovariance=Matrix3::Identity()*.001;
    imu->integrationCovariance=Matrix3::Identity()*1e-6;
    auto bias_noise=noiseModel::Isotropic::Sigma(6,.01);
    auto velocity_noise=noiseModel::Isotropic::Sigma(3,.1);
    SolverRun out;NonlinearFactor::shared_ptr replaceable;
    for(size_t i=0;i<40;++i) {
      Pose3 pose(Rot3::Rz(.0002*i),Point3(.001*i,0,0));
      Values values;values.insert(X(i),pose);values.insert(V(i),Vector3(.01,0,0));
      values.insert(B(i),imuBias::ConstantBias());
      NonlinearFactorGraph g;FixedLagSmoother::KeyTimestampMap timestamps{{X(i),.1*i},{V(i),.1*i},{B(i),.1*i}};
      FactorIndices remove;
      if(!i){
        g.emplace_shared<PriorFactor<Pose3>>(X(0),pose,denseNoise());
        g.emplace_shared<PriorFactor<Vector3>>(V(0),Vector3(.01,0,0),velocity_noise);
        g.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(0),imuBias::ConstantBias(),bias_noise);
      }else{
        PreintegratedImuMeasurements pim(imu,imuBias::ConstantBias());
        for(int j=0;j<10;++j)pim.integrateMeasurement(Vector3(0,0,9.81),Vector3(0,0,.002),.01);
        g.emplace_shared<ImuFactor>(X(i-1),V(i-1),X(i),V(i),B(i-1),pim);
        g.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(i-1),B(i),imuBias::ConstantBias(),bias_noise);
        Pose3 previous(Rot3::Rz(.0002*(i-1)),Point3(.001*(i-1),0,0));
        g.emplace_shared<BetweenFactor<Pose3>>(X(i-1),X(i),previous.between(pose),denseNoise());
        for(size_t j=0;j<per_step;++j)g.push_back(factor(X(i-1),X(i)));
        // Remove one still-active factor and replace it at identical keys.
        if(i%3==0 && replaceable){
          const auto& active=solver.getFactors();
          for(size_t slot=0;slot<active.size();++slot)if(active[slot]==replaceable)remove.push_back(slot);
          if(!remove.empty())g.push_back(factor(X(i-2),X(i-1)));
        }
        replaceable=per_step?g.back():NonlinearFactor::shared_ptr();
      }
      if(retry && i==9){
        const auto checkpoint=solver;
        auto fail=[](const auto&,const auto&,void*)->GaussianFactorGraph::shared_ptr{throw std::runtime_error("controlled update failure");};
        {ScopedLinearizer scope(fail);EXPECT_ANY_THROW(solver.update(g,values,timestamps,remove));}
        // Preserve the existing caller checkpoint/restore contract, including
        // FixedLagSmoother timestamps. This patch adds no solver rollback API.
        solver=checkpoint;
      }
      e1b::Record record;
      const auto begin=std::chrono::steady_clock::now();
      {e1b::Session profile(i*.1,e1b::Context::Local,&record);
       ScopedLinearizer scope(batch?glim::e1b4a::linearize:nullptr,&out.stats);
       solver.update(g,values,timestamps,remove);}
      out.milliseconds.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
      out.records.push_back(record);
      {
        SolverSnapshot snap;snap.estimate=solver.calculateEstimate();snap.theta=solver.getLinearizationPoint();
        // Timing fixtures observe states without extra factor evaluations
        // between updates. Check complete covariance only after their last step.
        if(capture || i==39){
        auto linear=solver.getFactors().linearize(snap.estimate); // outside scoped batching
        Marginals marginal(*linear,snap.estimate);
        snap.covariance=marginal.jointMarginalCovariance(snap.estimate.keys()).fullMatrix();
        if(i){
          const auto a=snap.estimate.at<Pose3>(X(i-1)),b=snap.estimate.at<Pose3>(X(i));
          snap.relative=cbs::relative_pose_covariance::compute(marginal.jointMarginalInformation({X(i-1),X(i)}),X(i-1),X(i),a,b,a.between(b),cbs::relative_pose_covariance::Mode::SchurRelativeBetween).covariance;
        }
        }
        for(const auto& f:solver.getFactors()){
          snap.slots.push_back(f?f->keys():KeyVector());snap.types.push_back(f?typeid(*f).name():"");
        }
        snap.relinearized=solver.getISAM2Result().variablesRelinearized;
        snap.reeliminated=solver.getISAM2Result().variablesReeliminated;
        out.snapshots.push_back(std::move(snap));
      }
    }
    return out;
  }
};
void compareRuns(const SolverRun& a,const SolverRun& b){
  ASSERT_EQ(a.snapshots.size(),b.snapshots.size());
  double position=0,angle=0,velocity=0,bias=0,cov=0;
  for(size_t i=0;i<a.snapshots.size();++i){
    const auto& x=a.snapshots[i];const auto& y=b.snapshots[i];
    ASSERT_EQ(x.estimate.keys(),y.estimate.keys());EXPECT_EQ(x.slots,y.slots);EXPECT_EQ(x.types,y.types);
    EXPECT_EQ(x.relinearized,y.relinearized);EXPECT_EQ(x.reeliminated,y.reeliminated);
    for(const auto* pair : {&x.estimate,&x.theta}){
      const auto& other=pair==&x.estimate?y.estimate:y.theta;
      for(Key key:pair->keys()){
        switch(Symbol(key).chr()){
          case 'x':{const auto& p=pair->at<Pose3>(key);const auto& q=other.at<Pose3>(key);
            position=std::max(position,(p.translation()-q.translation()).cwiseAbs().maxCoeff());
            angle=std::max(angle,Rot3::Logmap(p.rotation().between(q.rotation())).norm());break;}
          case 'v':velocity=std::max(velocity,(pair->at<Vector3>(key)-other.at<Vector3>(key)).cwiseAbs().maxCoeff());break;
          case 'b':bias=std::max(bias,(pair->at<imuBias::ConstantBias>(key).vector()-other.at<imuBias::ConstantBias>(key).vector()).cwiseAbs().maxCoeff());break;
        }
      }
    }
    ASSERT_EQ(x.covariance.rows(),y.covariance.rows());
    if(x.covariance.rows()){
      nearMatrix(x.covariance,y.covariance,1e-10,1e-6);cov=std::max(cov,(x.covariance-y.covariance).cwiseAbs().maxCoeff());
      if(i)nearMatrix(x.relative,y.relative,1e-10,1e-6);
    }
    for(auto stage:{e1b::Stage::IsamBatch,e1b::Stage::IsamIncremental,e1b::Stage::IsamMarginalize})
      EXPECT_EQ(a.records[i].owner[unsigned(stage)].calls,b.records[i].owner[unsigned(stage)].calls);
  }
  EXPECT_LE(position,1e-9);EXPECT_LE(angle,1e-10);EXPECT_LE(velocity,1e-9);EXPECT_LE(bias,1e-9);
  std::cout<<"E1B4_STATE_MAX,"<<position<<','<<angle<<','<<velocity<<','<<bias<<','<<cov<<'\n';
}
TEST_F(UpdateBatchSolver, IndependentRepeatedUpdatesRetirementReplacementFullStateAndCovariance){
  auto a=execute(false,12),b=execute(true,12);compareRuns(a,b);
  EXPECT_GT(b.stats.prepared_factors,500u);EXPECT_GT(b.stats.batched_graphs,40u);
  auto repeated=execute(true,12);compareRuns(b,repeated);
}
TEST_F(UpdateBatchSolver, IncrementalBranchAndCheckpointExceptionRecovery){
  auto a=execute(false,4,true,.1),b=execute(true,4,true,.1,true);compareRuns(a,b);
  uint64_t incremental=0;for(auto& r:b.records)incremental+=r.owner[unsigned(e1b::Stage::IsamIncremental)].calls;
  EXPECT_GT(incremental,0u);
}
TEST_F(UpdateBatchSolver, CpuOnlySolverMatchesOrdinary){compareRuns(execute(false,0),execute(true,0));}
TEST_F(UpdateBatchSolver, HeavyBenchmarkWorkloadEquivalence){compareRuns(execute(false,64,false),execute(true,64,false));}
TEST_F(UpdateBatchSolver, DISABLED_AlternatingBenchmark){
  for(size_t factors:{size_t(0),size_t(64)}){
    execute(false,factors,false);execute(true,factors,false); // warm both paths
    for(int round=0;round<6;++round){
      SolverRun pair[2];
      for(int order=0;order<2;++order){
      bool enabled=(round%2)==order;auto r=execute(enabled,factors,false);
      for(size_t i=0;i<r.milliseconds.size();++i){
        const auto& p=r.records[i];
        std::cout<<"E1B4_TIMING,"<<factors<<','<<round<<','<<enabled<<','<<i<<','<<r.milliseconds[i]
          <<','<<p.owner[unsigned(e1b::Stage::GpuLinearize)].calls
          <<','<<p.owner[unsigned(e1b::Stage::GpuSyncLinearize)].calls
          <<','<<p.metrics[unsigned(e1b::Metric::GpuLinearizeHits)]
          <<','<<r.snapshots[i].estimate.size()<<','<<r.snapshots[i].slots.size()<<'\n';
      }
      std::cout<<"E1B4_BATCH_COUNTS,"<<factors<<','<<round<<','<<enabled<<','<<r.stats.requests<<','<<r.stats.batched_graphs<<','<<r.stats.prepared_factors<<'\n';
      pair[enabled]=std::move(r);
      }
      compareRuns(pair[0],pair[1]);
    }
  }
}
}  // namespace
