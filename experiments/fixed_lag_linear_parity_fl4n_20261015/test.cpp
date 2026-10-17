#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/slam/PriorFactor.h>
#include <sstream>
#include <iostream>
using namespace gtsam;
using symbol_shorthand::X;
void require(bool b,const char* what){if(!b)throw std::runtime_error(what);}
struct Output{Values values;std::vector<Matrix> covariance;};
Output run(bool observe) {
  setenv("FL4M_RETIREMENT","remove",1);unsetenv("FL4M_AUDIT");unsetenv("FL4N_CAPTURE");
  cbs::BPSAM::Params p;p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.sam_params_.relinearizeSkip=1;p.sam_params_.relinearizeThreshold=0.;
  cbs::IncrementalFixedLagBpsamSmoother s(3.5,p);
  auto noise=noiseModel::Isotropic::Sigma(6,.1);Output result;
  Pose3 step(Rot3::RzRyRx(.005,.01,.02),Point3(.1,.02,-.01));
  Pose3 truth;
  for(size_t k=0;k<10;++k) {
    NonlinearFactorGraph graph;Values init;
    if(k)truth=truth.compose(step);
    init.insert(X(k),truth.retract((Vector6()<<.001,-.002,.003,.01,-.02,.01).finished()));
    if(!k)graph.emplace_shared<PriorFactor<Pose3>>(X(k),Pose3(),noise);
    else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,noise);
    if(k==2) {
      cbs::BPSAM::CbsOdometryBelief b;b.source_agent=static_cast<cbs::AgentId>('g');
      b.from_pose_key=X(0);b.to_pose_key=X(1);b.sender_from_pose_key=Symbol('g',0);b.sender_to_pose_key=Symbol('g',1);
      b.measured_from_to=step;b.covariance=Matrix6::Identity()*.0025;b.relax_factor=1;
      require(s.addOdometryBeliefsDetailed({b}).details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"admission");
    }
    s.update(graph,init,{{X(k),double(k)}});
    if(observe) {
      std::ostringstream os;const auto a=fl4n::inspect(s.bpsam(),s.bpsam().fl4nLinearCache(),os);
      require(a.tree_H_relative<1e-10&&a.tree_g_relative<1e-9,"cache/tree mismatch");
      require(a.fresh_H_relative<1e-10&&a.fresh_g_relative<1e-9,"cache/fresh mismatch");
      require(a.cached_normal_residual<1e-9,"delta mismatch");
      require(!a.cache_orphans&&!a.nonlinear_without_cache,"slot liveness mismatch");
      auto corrupt=s.bpsam().fl4nLinearCache();
      for(auto f:corrupt)if(f){corrupt.push_back(f);break;}
      const auto bad=fl4n::inspect(s.bpsam(),corrupt,os);
      require(bad.tree_H_relative>1e-5&&bad.cache_orphans,"corruption not detected");
    }
    result.values.insert(X(k),s.calculateEstimate().at<Pose3>(X(k)));
    result.covariance.push_back(s.bpsam().marginalCovariance(X(k)));
    if(k>=4)require(!s.calculateEstimate().exists(X(k-4)),"expired key retained");
  }
  return result;
}
int main(){try {
  auto a=run(false),b=run(true);
  require(a.values.equals(b.values,0),"observer mutated nominal output");
  for(size_t i=0;i<a.covariance.size();++i)require((a.covariance[i]-b.covariance[i]).norm()==0,"observer mutated covariance");
  std::cout<<"PASS cache/tree/fresh same-chart parity; removal/marginalization; deliberate corruption; exact observer nonmutation\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
