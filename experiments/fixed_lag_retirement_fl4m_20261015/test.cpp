#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <iostream>
using namespace gtsam;
using symbol_shorthand::X;
void require(bool p,const char* why) {if(!p)throw std::runtime_error(why);}
std::vector<Matrix> run(bool retain,bool persistent,bool external) {
  setenv("FL4M_RETIREMENT",retain?"retain":"remove",1);
  cbs::BPSAM::Params p;
  p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=persistent?cbs::BPSAM::CbsOdomFactorMode::Persistent:cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.sam_params_.relinearizeSkip=1;
  cbs::IncrementalFixedLagBpsamSmoother s(2.5,p);
  auto noise=noiseModel::Isotropic::Sigma(6,.1);
  std::vector<Matrix> result;
  for(size_t k=0;k<8;++k) {
    NonlinearFactorGraph graph; Values initial;
    initial.insert(X(k),Pose3());
    if(k==0)graph.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise);
    else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),Pose3(),noise);
    if(k==2&&external) {
      cbs::BPSAM::CbsOdometryBelief b;
      b.source_agent=static_cast<cbs::AgentId>('g');b.from_pose_key=X(0);b.to_pose_key=X(1);
      b.sender_from_pose_key=Symbol('g',0);b.sender_to_pose_key=Symbol('g',1);
      b.measured_from_to=Pose3();b.covariance=Matrix::Identity(6,6)*.0025;b.relax_factor=1;
      auto a=s.addOdometryBeliefsDetailed({b});require(a.details.size()==1,"missing admission");
      require(a.details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"admission failed");
    }
    s.update(graph,initial,{{X(k),double(k)}});
    result.push_back(s.bpsam().marginalCovariance(X(k)));
    require(s.calculateEstimate().at<Pose3>(X(k)).equals(Pose3(),1e-12),"state moved");
    if(k>=3) {
      require(!s.calculateEstimate().exists(X(k-3)),"expired key retained");
      if(!persistent)require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==0,"stale tracker");
    }
  }
  return result;
}
int main() { try {
  auto a=run(false,false,true),b=run(true,false,true);
  auto native=run(false,true,false),persistent=run(false,true,true);
  for(size_t k=0;k<8;++k) {
    if(k<3)require((a[k]-b[k]).norm()<1e-12,"pre-retirement difference");
    else {
      require((a[k]-native[k]).norm()<1e-10,"removed factor inherited");
      require((b[k]-persistent[k]).norm()<1e-10,"retained factor not inherited");
      require((a[k]-b[k]).norm()>1e-3,"fixture no retirement contrast");
    }
  }
  std::cout<<"PASS pre-retirement parity; removed matches native; retained matches persistent; no stale keys/tracking\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;} }
