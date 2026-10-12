#include "RuntimeProfile.h"
#include <gtest/gtest.h>
#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <atomic>
#include <iomanip>
#include <tbb/global_control.h>

using namespace gtsam;
using symbol_shorthand::X;
namespace {
struct CountedBetween : BetweenFactor<Pose3> {
  using BetweenFactor::BetweenFactor;
  static std::atomic<int> linearizations,errors;
  GaussianFactor::shared_ptr linearize(const Values& v) const override {
    ++linearizations; return BetweenFactor::linearize(v);
  }
  double error(const Values& v) const override {++errors;return BetweenFactor::error(v);}
};
std::atomic<int> CountedBetween::linearizations{0}, CountedBetween::errors{0};
struct Result {std::vector<double> numbers;int linearizations,errors;std::vector<e1b::Record> records;};
Result run(bool profile) {
  CountedBetween::linearizations=0;CountedBetween::errors=0;
  cbs::BPSAM::Params params;params.robot_id='g';
  params.sam_params_.relinearizeSkip=1;params.sam_params_.findUnusedFactorSlots=true;
  params.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  cbs::IncrementalFixedLagBpsamSmoother smoother(.6,params);
  auto noise=noiseModel::Diagonal::Sigmas((Vector6()<<.01,.012,.011,.04,.03,.035).finished());
  Result out;
  Pose3 previous;
  for(size_t i=0;i<35;++i) {
    Pose3 pose(Rot3::RzRyRx(.001*i,.002*i,.01*i),Point3(.1*i,.001*i*i,.005*i));
    NonlinearFactorGraph graph;Values values;
    values.insert(X(i),pose.retract(Vector6::Constant(i?.002:0)));
    if(!i)graph.emplace_shared<PriorFactor<Pose3>>(X(i),pose,noise);
    else graph.emplace_shared<CountedBetween>(X(i-1),X(i),previous.between(pose),noise);
    e1b::Record record;
    {e1b::Session session(i*.1,e1b::Context::Local,profile?&record:nullptr);
     e1b::Scope local(e1b::Stage::LocalTotal);smoother.update(graph,values,{{X(i),i*.1}});}
    out.records.push_back(record);
    const auto estimate=smoother.calculateEstimate();
    for(const auto& keyvalue:estimate) {
      auto matrix=estimate.at<Pose3>(keyvalue.key).matrix();
      out.numbers.insert(out.numbers.end(),matrix.data(),matrix.data()+matrix.size());
    }
    smoother.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
    Matrix covariance=smoother.marginalCovariance(X(i));
    out.numbers.insert(out.numbers.end(),covariance.data(),covariance.data()+covariance.size());
    previous=pose;
  }
  out.linearizations=CountedBetween::linearizations;out.errors=CountedBetween::errors;return out;
}
TEST(SolverProfile, SameStatesCovariancesAndEvaluationCounts) {
  tbb::global_control limit(tbb::global_control::max_allowed_parallelism,4);
  auto off=run(false),on=run(true);
  EXPECT_EQ(off.numbers,on.numbers);EXPECT_EQ(off.linearizations,on.linearizations);EXPECT_EQ(off.errors,on.errors);
  const bool native=std::getenv("E1B_NATIVE_ORACLE")!=nullptr;
  for(const auto& record:on.records) {
    if(!native)EXPECT_GT(record.owner[static_cast<unsigned>(e1b::Stage::Backup)].calls,0u);
    if(!native)EXPECT_GT(record.owner[static_cast<unsigned>(e1b::Stage::IsamUpdate)].calls,0u);
    uint64_t total=0;for(auto& c:record.owner)total+=c.exclusive_ns;
    EXPECT_EQ(total,record.owner[static_cast<unsigned>(e1b::Stage::Root)].inclusive_ns);
  }
  // Retained deterministic math fingerprint; timing/log formatting is separate.
  std::cout<<"E1B_MATH,"<<off.linearizations<<','<<off.errors<<std::setprecision(17);
  for(double x:off.numbers)std::cout<<','<<x;std::cout<<'\n';
}
}  // namespace
