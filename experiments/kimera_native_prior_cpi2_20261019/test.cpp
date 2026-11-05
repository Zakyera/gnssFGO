// Actual FL4q update with read-only pre-marginalization snapshots.
#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Cholesky>
#include <iostream>
#include <iomanip>
using namespace gtsam;
using symbol_shorthand::X;
using Smoother=cbs::IncrementalFixedLagBpsamSmoother;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
double relative(const Matrix& a,const Matrix& b){return (a-b).norm()/std::max(1.,b.norm());}
int main(){try{
  setenv("FL4Q_ENABLE","1",1);setenv("FL4M_RETIREMENT","remove",1);
  cbs::BPSAM::Params p;p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.outgoing_odom_covariance_mode="schur_relative_between";
  p.sam_params_.relinearizeSkip=1;p.sam_params_.relinearizeThreshold=0.;
  Smoother s(4.5,p);
  Matrix6 L=Matrix6::Identity()*.12;
  L(3,0)=.02;L(4,1)=-.03;L(5,2)=.04;L(5,3)=.025;
  auto native=noiseModel::Gaussian::Covariance(L*L.transpose());
  Pose3 step(Rot3::RzRyRx(.015,-.01,.035),Point3(.12,.025,-.01));
  int retirements=0,deleted_native=0,retained_peer=0,admitted=0,queries=0;
  double maxH=0,maxEta=0,maxCov=0,maxCorrection=0;
  for(int k=0;k<18;++k){
    NonlinearFactorGraph added;Values init;
    if(k==0){init.insert(X(0),Pose3());added.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),native);}
    else{
      init.insert(X(k),s.calculateEstimate().at<Pose3>(X(k-1)).compose(step));
      added.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,native);
      if(k>=2)added.emplace_shared<BetweenFactor<Pose3>>(X(k-2),X(k),step.compose(step),native);
    }
    if(k>=2){
      cbs::BPSAM::CbsOdometryBelief b;b.source_agent=static_cast<cbs::AgentId>('g');
      b.from_pose_key=X(k-2);b.to_pose_key=X(k-1);
      b.sender_from_pose_key=Symbol('g',k-2);b.sender_to_pose_key=Symbol('g',k-1);
      Vector6 shift;shift<<.015,-.008,.012,.018,-.012,.008;
      b.measured_from_to=step.compose(Pose3::Expmap(shift));
      b.covariance=L*L.transpose()*.04;b.relax_factor=1;
      require(s.addOdometryBeliefsDetailed({b}).details.at(0).status==
        cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"peer admission");++admitted;
    }
    s.captureBadUpdateStagesOnce();s.update(added,init,{{X(k),double(k)}});
    const auto& a=*s.badUpdateStageAudit();
    const auto estimates=s.calculateEstimate();
    if(k>=2)maxCorrection=std::max(maxCorrection,Pose3::Logmap(step.between(
      estimates.at<Pose3>(X(k-2)).between(estimates.at<Pose3>(X(k-1))))).norm());
    if(!a.keys_to_marginalize.empty()){
      ++retirements;
      KeySet gone(a.keys_to_marginalize.begin(),a.keys_to_marginalize.end());
      GaussianFactorGraph removed,created;
      for(size_t slot:a.deleted_marginal_factor_slots){
        require(std::find(a.cpi2_peer_slots.begin(),a.cpi2_peer_slots.end(),slot)==a.cpi2_peer_slots.end(),"peer absorbed in marginal prior");
        require(bool(a.cpi2_cached.at(slot)),"missing cached native equations");
        removed.push_back(a.cpi2_cached.at(slot));++deleted_native;
      }
      for(size_t slot:a.cpi2_peer_slots){
        ++retained_peer;
        for(Key key:a.graph_after_optimization.at(slot)->keys())require(!gone.count(key),"peer touches retiring state");
        require(s.bpsam().isTrackedCbsFactorSlot(slot,s.getFactors().at(slot)),"surviving peer tracking lost");
      }
      KeySet separators=removed.keys();for(Key key:gone)separators.erase(key);
      Ordering order;for(Key key:gone)order.push_back(key);for(Key key:separators)order.push_back(key);
      auto hg=removed.hessian(order);const int n=6*gone.size(),m=6*separators.size();
      Eigen::LDLT<Matrix> solve(hg.first.topLeftCorner(n,n));require(solve.info()==Eigen::Success,"dense elimination");
      Matrix cross=hg.first.topRightCorner(n,m);
      Matrix H=hg.first.bottomRightCorner(m,m)-cross.transpose()*solve.solve(cross);
      Vector eta=hg.second.tail(m)-cross.transpose()*solve.solve(hg.second.head(n));
      for(size_t slot:a.created_marginal_factor_slots)created.push_back(s.getFactors().at(slot)->linearize(a.cpi2_theta));
      auto actual=created.hessian(Ordering(KeyVector(separators.begin(),separators.end())));
      double eh=relative(actual.first,H),eg=relative(actual.second,eta);
      maxH=std::max(maxH,eh);maxEta=std::max(maxEta,eg);
      std::cout<<std::setprecision(17)<<"PRIOR,"<<k<<','<<eh<<','<<eg<<','<<a.cpi2_peer_slots.size()<<'\n';
      require(eh<1e-8&&eg<1e-8,"native prior differs from dense Schur");
    }
    if(k){
      auto before=s.calculateEstimate();
      s.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      auto b=s.getOdometryBeliefs({X(k-1),X(k)},static_cast<cbs::AgentId>('g'));
      auto again=s.getOdometryBeliefs({X(k-1),X(k)},static_cast<cbs::AgentId>('g'));
      require(b.size()==1&&again.size()==1,"missing LOCAL covariance");
      maxCov=std::max(maxCov,(b[0].covariance-again[0].covariance).norm());++queries;
      require(maxCov==0&&before.equals(s.calculateEstimate(),0),"LOCAL query mutation/repeat");
      require(b[0].measured_from_to.equals(before.at<Pose3>(X(k-1)).between(before.at<Pose3>(X(k))),1e-12),"mean not MAIN");
    }
  }
  require(retirements>=10&&retained_peer>10&&maxCorrection>.001,"unexercised nonzero correction/retirement");
  std::cout<<std::setprecision(17)<<"PASS retirements="<<retirements<<" deleted_native="<<deleted_native
    <<" retained_peer_observations="<<retained_peer<<" admitted="<<admitted<<" query_pairs="<<queries
    <<" max_H="<<maxH<<" max_eta="<<maxEta<<" repeat_cov="<<maxCov<<" max_correction="<<maxCorrection<<'\n';
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
