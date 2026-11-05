// Audit the unchanged actual FL4q wrapper, not a replacement smoother.
#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <iostream>
using namespace gtsam;
using symbol_shorthand::X;
using Smoother = cbs::IncrementalFixedLagBpsamSmoother;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
cbs::BPSAM::Params parameters() {
  cbs::BPSAM::Params p; p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.outgoing_odom_covariance_mode="schur_relative_between";
  p.sam_params_.relinearizeSkip=1; p.sam_params_.relinearizeThreshold=0.;
  return p;
}
int main() {
  try {
    setenv("FL4Q_ENABLE","1",1); setenv("FL4M_RETIREMENT","remove",1);
    Smoother native(3.5,parameters()), peer(3.5,parameters());
    const Pose3 step(Rot3(),Point3(.1,0,0));
    const auto noise=noiseModel::Isotropic::Sigma(6,.1);
    double worst=0; int checks=0; bool main_differs=false;
    for (int k=0;k<10;++k) {
      NonlinearFactorGraph graph; Values init;
      init.insert(X(k), Pose3(Rot3(),Point3(.1*k,0,0)));
      if (!k) graph.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise);
      else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,noise);
      if (k==2) {
        cbs::BPSAM::CbsOdometryBelief b;
        b.source_agent=static_cast<cbs::AgentId>('g');
        b.from_pose_key=X(0); b.to_pose_key=X(1);
        b.sender_from_pose_key=Symbol('g',0); b.sender_to_pose_key=Symbol('g',1);
        b.measured_from_to=step; b.covariance=Matrix6::Identity()*.0001; b.relax_factor=1;
        check(peer.addOdometryBeliefsDetailed({b}).details.at(0).status==
            cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"CBS admission failed");
      }
      native.update(graph,init,{{X(k),double(k)}});
      peer.update(graph,init,{{X(k),double(k)}});
      if (!k) continue;
      native.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      peer.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      auto before=peer.calculateEstimate();
      for (int j=std::max(0,k-3);j<k;++j) {
        const KeySet keys{X(j),X(j+1)};
        auto a=native.getOdometryBeliefs(keys,static_cast<cbs::AgentId>('g'));
        auto b=peer.getOdometryBeliefs(keys,static_cast<cbs::AgentId>('g'));
        auto again=peer.getOdometryBeliefs(keys,static_cast<cbs::AgentId>('g'));
        check(a.size()==1&&b.size()==1&&again.size()==1,"missing LOCAL pair");
        const double err=(a[0].covariance-b[0].covariance).norm();
        worst=std::max(worst,err); check(err<1e-9,"explicit peer precision inherited");
        check((b[0].covariance-again[0].covariance).norm()==0,"same-update revision differs");
        check(b[0].measured_from_to.equals(before.at<Pose3>(X(j)).between(before.at<Pose3>(X(j+1))),1e-12),"mean not MAIN");
        ++checks;
      }
      if(k==2) {
        peer.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::FULL);
        auto full=peer.getOdometryBeliefs({X(0),X(1)},static_cast<cbs::AgentId>('g'));
        auto loc=native.getOdometryBeliefs({X(0),X(1)},static_cast<cbs::AgentId>('g'));
        main_differs=(full.at(0).covariance-loc.at(0).covariance).norm()>.001;
      }
      if(k>=4)check(peer.bpsam().activeWindowTemporaryCbsOdomFactorCount()==0,"external factor survived retirement");
      check(before.equals(peer.calculateEstimate(),0),"query changed MAIN");
    }
    check(main_differs,"test did not exercise peer information");
    std::cout<<"PASS Kimera unchanged LOCAL first/revision, temporary removal, native-information prior fixture; pairs="<<checks<<" max_covariance_difference="<<worst<<"\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
