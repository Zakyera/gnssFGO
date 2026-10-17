#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/linear/JacobianFactor.h>
#include <iostream>
#include <sstream>
using namespace gtsam;
using symbol_shorthand::X;
using Smoother=cbs::IncrementalFixedLagBpsamSmoother;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
template<class F> void rejects(F f){bool b=false;try{f();}catch(const std::exception&){b=true;}require(b,"expected failure");}
cbs::BPSAM::Params params(){
  cbs::BPSAM::Params p;p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.sam_params_.relinearizeSkip=1;p.sam_params_.relinearizeThreshold=0.;
  return p;
}
class Square:public NoiseModelFactor1<double>{
 public:
  Square():NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1),symbol_shorthand::X(0)){}
  Vector evaluateError(const double& x,OptionalMatrixType H=nullptr)const override{
    if(H)*H=(Matrix(1,1)<<2*x).finished();return (Vector(1)<<x*x-1).finished();
  }
  NonlinearFactor::shared_ptr clone()const override{return std::make_shared<Square>(*this);}
};
void policy(){
  NonlinearFactorGraph g;g.emplace_shared<Square>();Values a,b;a.insert(X(0),.1);b.insert(X(0),5.05);
  auto d=fl4o::select(g,a,a,b);
  require(d.alpha==.25&&d.after<d.before&&d.full>d.before,"overshoot not reduced");
  require(a.at<double>(X(0))==.1&&b.at<double>(X(0))==5.05,"policy mutation");
  rejects([&]{fl4o::select(g,a,a,b,0);});
  Values worse;worse.insert(X(0),-.2);
  // At x=1 the objective is zero: all nonzero shortening is uphill.
  Values optimum;optimum.insert(X(0),1.);
  rejects([&]{fl4o::select(g,optimum,optimum,worse);});
  Values nan;nan.insert(X(0),std::numeric_limits<double>::quiet_NaN());
  rejects([&]{fl4o::select(g,a,a,nan);});
  Values full;full.insert(X(0),1.);
  require(fl4o::select(g,a,a,full).alpha==1,"ordinary step changed");
  auto jf=std::make_shared<JacobianFactor>(X(0),(Matrix(1,1)<<2).finished(),(Vector(1)<<1).finished());
  NonlinearFactorGraph prior;prior.emplace_shared<LinearContainerFactor>(jf);
  Values theta,v;theta.insert(X(0),10.);v.insert(X(0),12.);
  require(prior.error(v)==0&&std::abs(fl4o::cost(prior,v,theta)-4.5)<1e-12,"prior chart");
  Values missing;rejects([&]{fl4o::cost(prior,v,missing);});
  std::cout<<"PASS bounded nonlinear descent, fast path, failure, input nonmutation, original prior chart\n";
}
void deltaPersistence(){
  cbs::BPSAM b(params());NonlinearFactorGraph g;g.emplace_shared<Square>();
  Values start;start.insert(X(0),.1);b.update(g,start);
  auto native=b.calculateEstimate();auto theta=b.getLinearizationPoint();
  const auto C=b.marginalCovariance(X(0));
  auto tree=GaussianFactorGraph(b).augmentedHessian();
  auto cache=b.fl4nLinearCache().augmentedHessian();
  auto d=fl4o::select(g,theta,start,native);require(d.alpha<1,"fixture did not overshoot");
  b.fl4oCommitAcceptedValues(d.accepted);
  require(b.calculateEstimate().equals(d.accepted,1e-14),"accepted read lost");
  require(b.getLinearizationPoint().at<double>(X(0))==theta.at<double>(X(0)),"theta changed");
  require((C-b.marginalCovariance(X(0))).norm()==0,"covariance changed");
  require((tree-GaussianFactorGraph(b).augmentedHessian()).norm()==0,"tree changed");
  require((cache-b.fl4nLinearCache().augmentedHessian()).norm()==0,"cache changed");
  b.update();
  require(b.getLinearizationPoint().equals(d.accepted,1e-14),"next update ignored accepted step");
  std::cout<<"PASS accepted delta survives reads and seeds next update; unchanged theta/tree/H/g/covariance\n";
}
struct Output{Values values;std::vector<Matrix> covariance;};
Output chain(bool enabled,bool inject_failure){
  setenv("FL4O_ENABLE",enabled?"1":"0",1);setenv("FL4M_RETIREMENT","remove",1);
  unsetenv("FL4M_AUDIT");unsetenv("FL4N_CAPTURE");unsetenv("FL4O_LOG");
  Smoother s(3.5,params());auto noise=noiseModel::Isotropic::Sigma(6,.1);Output result;
  Pose3 step(Rot3::RzRyRx(.005,.01,.02),Point3(.1,.02,-.01)),truth;
  for(size_t k=0;k<10;++k){
    NonlinearFactorGraph graph;Values init;if(k)truth=truth.compose(step);
    init.insert(X(k),truth.retract((Vector6()<<.001,-.002,.003,.01,-.02,.01).finished()));
    if(!k)graph.emplace_shared<PriorFactor<Pose3>>(X(k),Pose3(),noise);
    else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,noise);
    if(k==2){
      cbs::BPSAM::CbsOdometryBelief b;b.source_agent=static_cast<cbs::AgentId>('g');
      b.from_pose_key=X(0);b.to_pose_key=X(1);b.sender_from_pose_key=Symbol('g',0);b.sender_to_pose_key=Symbol('g',1);
      b.measured_from_to=step;b.covariance=Matrix6::Identity()*.0025;b.relax_factor=1;
      require(s.addOdometryBeliefsDetailed({b}).details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"admission");
    }
    if(k==4&&inject_failure){
      const auto v=s.calculateEstimate(),theta=s.bpsam().getLinearizationPoint();
      const auto times=s.timestamps();const auto factors=s.getFactors();
      const auto tree=GaussianFactorGraph(s.bpsam()).augmentedHessian();
      const auto cov=s.bpsam().marginalCovariance(X(3));
      const auto count=s.bpsam().activeWindowTemporaryCbsOdomFactorCount();
      rejects([&]{fl4o::transaction(s,graph,[&]{
        s.update(graph,init,{{X(k),double(k)}});
        throw std::runtime_error("injected AFTER retirement and marginalization");return 0;
      });});
      require(s.calculateEstimate().equals(v,0)&&s.bpsam().getLinearizationPoint().equals(theta,0),"rollback state");
      require(s.timestamps()==times,"rollback times");require(s.getFactors().size()==factors.size(),"rollback slots");
      for(size_t i=0;i<factors.size();++i)require(s.getFactors()[i]==factors[i],"rollback factor pointer");
      require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==count,"rollback CBS accounting");
      require((GaussianFactorGraph(s.bpsam()).augmentedHessian()-tree).norm()==0,"rollback tree");
      require((s.bpsam().marginalCovariance(X(3))-cov).norm()==0,"rollback covariance");
    }
    s.update(graph,init,{{X(k),double(k)}});
    result.values.insert(X(k),s.calculateEstimate().at<Pose3>(X(k)));
    result.covariance.push_back(s.bpsam().marginalCovariance(X(k)));
    if(k>=4){require(!s.calculateEstimate().exists(X(k-4)),"expired key retained");
      require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==0,"external factor inherited");}
  }
  return result;
}
void marginalDelta(){
  cbs::BPSAM b(params());NonlinearFactorGraph g;Values init;
  const auto n=noiseModel::Isotropic::Sigma(6,.1);
  g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),n);
  g.emplace_shared<BetweenFactor<Pose3>>(X(0),X(1),Pose3(Rot3(),Point3(1,0,0)),n);
  init.insert(X(0),Pose3());init.insert(X(1),Pose3(Rot3(),Point3(1.2,0,0)));
  cbs::BPSAM::UpdateParams p;FastMap<Key,int> order;order[X(0)]=0;order[X(1)]=1;p.constrainedKeys=order;
  b.update(g,init,p);auto accepted=b.calculateEstimate();
  accepted.update(X(1),Pose3(Rot3(),Point3(1.1,0,0)));
  b.fl4oCommitAcceptedValues(accepted);b.marginalizeLeaves(KeyList{X(0)});
  require(b.calculateEstimate().at<Pose3>(X(1)).equals(accepted.at<Pose3>(X(1)),1e-14),"marginalization reset delta");
  require(!b.calculateEstimate().exists(X(0)),"marginalization failed");
  std::cout<<"PASS committed delta retained through actual leaf marginalization\n";
}
void smartCache(){
  auto K=std::make_shared<Cal3_S2Stereo>(500,500,0,320,240,.2);
  Pose3 a,b(Rot3(),Point3(1,0,0));Point3 point(.2,.3,5);
  auto s=std::make_shared<SmartStereoProjectionPoseFactor>(noiseModel::Isotropic::Sigma(3,1));
  s->add(StereoCamera(a,K).project(point),X(0),K);s->add(StereoCamera(b,K).project(point),X(1),K);
  Values v;v.insert(X(0),a);v.insert(X(1),b);s->error(v);require(bool(s->point()),"initial smart point");
  const Point3 old=*s->point();NonlinearFactorGraph g;g.push_back(s);
  fl4o::SmartBackup backup(g,{});const auto ptr=g[0].get();
  Values shifted;Pose3 G(Rot3(),Point3(2,3,1));shifted.insert(X(0),G*a);shifted.insert(X(1),G*b);
  fl4o::cost(g,shifted,v);require((*s->point()-old).norm()==0,"trial contaminated smart cache");
  s->error(shifted);require((*s->point()-old).norm()>1,"cache fixture did not mutate");
  backup.restore();require(g[0].get()==ptr&&(*s->point()-old).norm()==0,"smart cache/pointer rollback");
  NonlinearFactorGraph unsupported;unsupported.emplace_shared<Square>();rejects([&]{fl4o::SmartBackup bad(unsupported,{});});
  std::cout<<"PASS smart clone isolation, in-place cache rollback, pointer identity, unknown family rejected\n";
}
int main(){try{
  std::cout<<std::unitbuf;
  unsetenv("FL4O_LOG");policy();deltaPersistence();marginalDelta();smartCache();
  auto a=chain(false,false),b=chain(true,false),c=chain(true,true),d=chain(true,false);
  for(const auto* r:{&b,&c,&d}){
    require(a.values.equals(r->values,0),"fast path/rollback/repeat state differs");
    for(size_t i=0;i<a.covariance.size();++i)require((a.covariance[i]-r->covariance[i]).norm()==0,"fast path/rollback/repeat covariance differs");
  }
  std::cout<<"PASS exact ordinary-update parity, deterministic repeat, retirement-before-prior, whole-wrapper rollback\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"TEST FAILURE: "<<e.what()<<'\n';return 1;}}
