#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>
using namespace gtsam;
using symbol_shorthand::X;
using Smoother=cbs::IncrementalFixedLagBpsamSmoother;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
template<class F>void rejects(F f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}require(bad,"expected failure");}
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
void policyAndCommit(){
  cbs::BPSAM b(params());NonlinearFactorGraph graph;graph.emplace_shared<Square>();
  Values start;start.insert(X(0),.1);b.update(graph,start);
  const auto full=b.calculateEstimate();
  auto d=fl4q::accept(1,b,start,{});
  require(d.fallback&&d.after<1e-12&&d.full>d.before,"current model descent");
  require(b.getFactorsUnsafe()[0]==graph[0],"native pointer changed");
  require(b.calculateEstimate().equals(d.accepted,1e-14),"accepted read lost");
  require(b.getLinearizationPoint().at<double>(X(0))==d.accepted.at<double>(X(0)),"accepted cache chart");
  const double expected=1./(4*std::pow(d.accepted.at<double>(X(0)),2));
  require(std::abs(b.marginalCovariance(X(0))(0,0)-expected)<1e-10,"stale/damped covariance");
  auto repeated=fl4q::select(graph,start,start,full);
  require(repeated.accepted.at<double>(X(0))==d.accepted.at<double>(X(0)),"not deterministic");
  b.update();require(std::abs(b.calculateEstimate().at<double>(X(0))-1)<1e-6,"next update inconsistent");
  require(!fl4q::select(graph,start,start,d.accepted).fallback,"ordinary update changed");
  rejects([&]{fl4q::select(graph,start,start,full,0);});
  Values optimum;optimum.insert(X(0),1.);rejects([&]{fl4q::select(graph,optimum,optimum,full);});
  std::cout<<"PASS current-point LM, same-objective descent, native fast path, deterministic repeat, accepted model/covariance/next-update\n";
}
void priorAndMarginal(){
  cbs::BPSAM b(params());NonlinearFactorGraph g;Values init;
  const auto n=noiseModel::Isotropic::Sigma(6,.1);
  Pose3 step(Rot3::RzRyRx(.01,.02,.04),Point3(.2,.03,.01));
  g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),n);
  g.emplace_shared<BetweenFactor<Pose3>>(X(0),X(1),step,n);
  g.push_back(NonlinearFactor::shared_ptr());g.emplace_shared<BetweenFactor<Pose3>>(X(1),X(2),step,n);
  init.insert(X(0),Pose3());init.insert(X(1),step);init.insert(X(2),step*step);
  cbs::BPSAM::UpdateParams up;FastMap<Key,int> order;
  order[X(0)]=0;order[X(1)]=1;order[X(2)]=2;up.constrainedKeys=order;
  b.update(g,init,up);b.marginalizeLeaves(KeyList{X(0)});
  auto original=b.getFactorsUnsafe();auto anchor=b.getLinearizationPoint();
  auto wrapped=fl4q::anchored(original,anchor,false);
  auto moved=b.calculateEstimate();
  Vector6 displacement;displacement<<.1,-.15,.05,.2,.1,-.1;
  moved.update(X(1),moved.at<Pose3>(X(1)).retract(displacement));
  moved.update(X(2),moved.at<Pose3>(X(2)).retract(.7*displacement));
  require(std::abs(fl4q::cost(original,moved,anchor)-fl4q::cost(wrapped,moved,moved))<1e-10,"prior reanchored");
  const auto linear=wrapped.clone().linearize(moved);
  const auto hg=linear->hessian(Ordering(moved.keys()));
  VectorValues dir=moved.zeroVectors();dir.at(X(1))=displacement;dir.at(X(2))=-.3*displacement;
  auto plus=dir,minus=dir;for(auto& x:plus)x.second*=1e-6;for(auto& x:minus)x.second*=-1e-6;
  // Test the adapter derivative, not the pinned BetweenFactor's optional
  // approximate Log Jacobian (the source measurement implementation is frozen).
  NonlinearFactorGraph priors;
  for(const auto& f:wrapped)if(dynamic_cast<const fl4l::OriginalChartPrior*>(f.get()))priors.push_back(f);
  const auto prior_rhs=priors.linearize(moved)->gradientAtZero();
  double analytic=0;for(const auto& kv:prior_rhs)analytic+=kv.second.dot(dir.at(kv.first));
  const double numerical=(fl4q::cost(priors,moved.retract(plus),moved)-fl4q::cost(priors,moved.retract(minus),moved))/2e-6;
  require(std::abs(numerical-analytic)<1e-5,"original prior/current Jacobian");
  FastMap<Key,int> next;next[X(1)]=0;next[X(2)]=1;
  b.fl4qRebuild(wrapped,moved,next);
  require(b.getFactorsUnsafe().size()==original.size(),"sparse slots lost");
  for(size_t i=0;i<original.size();++i)if(original[i]&&!dynamic_cast<const LinearContainerFactor*>(original[i].get()))
    require(b.getFactorsUnsafe()[i]==original[i],"native pointer lost");
  const Matrix dense=hg.first.inverse();
  require((b.marginalCovariance(X(2))-dense.bottomRightCorner(6,6)).norm()<1e-8,"dense covariance mismatch");
  b.marginalizeLeaves(KeyList{X(1)});
  require(b.calculateEstimate().at<Pose3>(X(2)).equals(moved.at<Pose3>(X(2)),1e-12),"post-marg estimate changed");
  require((b.marginalCovariance(X(2))-dense.bottomRightCorner(6,6)).norm()<1e-8,"post-marg covariance mismatch");
  std::cout<<"PASS original prior objective/derivatives, sparse slots/native identity, undamped dense covariance, actual leaf marginalization\n";
}
struct Output{Values values;std::vector<Matrix> cov;};
Output chain(bool enabled,bool inject,bool force_rebuild=false){
  setenv("FL4Q_ENABLE",enabled?"1":"0",1);setenv("FL4M_RETIREMENT","remove",1);
  unsetenv("FL4M_AUDIT");unsetenv("FL4N_CAPTURE");unsetenv("FL4Q_LOG");
  Smoother s(3.5,params());auto n=noiseModel::Isotropic::Sigma(6,.1);Output out;
  Pose3 step(Rot3::RzRyRx(.005,.01,.02),Point3(.1,.02,-.01)),truth;
  for(size_t k=0;k<10;++k){
    NonlinearFactorGraph graph;Values init;if(k)truth=truth*step;
    init.insert(X(k),truth.retract((Vector6()<<.001,-.002,.003,.01,-.02,.01).finished()));
    if(!k)graph.emplace_shared<PriorFactor<Pose3>>(X(k),Pose3(),n);
    else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,n);
    if(k==2){
      cbs::BPSAM::CbsOdometryBelief belief;belief.source_agent=static_cast<cbs::AgentId>('g');
      belief.from_pose_key=X(0);belief.to_pose_key=X(1);belief.sender_from_pose_key=Symbol('g',0);belief.sender_to_pose_key=Symbol('g',1);
      belief.measured_from_to=step;belief.covariance=Matrix6::Identity()*.0025;belief.relax_factor=1;
      require(s.addOdometryBeliefsDetailed({belief}).details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"admission");
    }
    if(k==4&&inject){
      s.bpsam().setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      const auto v=s.calculateEstimate(),theta=s.bpsam().getLinearizationPoint();
      const auto times=s.timestamps();const auto factors=s.getFactors();
      const auto tree=GaussianFactorGraph(s.bpsam()).augmentedHessian();
      const auto cov=s.bpsam().marginalCovariance(X(3));
      const auto count=s.bpsam().activeWindowTemporaryCbsOdomFactorCount();
      rejects([&]{fl4q::transaction(s,graph,[&]{
        auto live=s.calculateEstimate();auto wrapped=fl4q::anchored(s.getFactors(),s.bpsam().getLinearizationPoint(),false);
        s.bpsam().fl4qRebuild(wrapped,live,{});
        s.update(graph,init,{{X(k),double(k)}});
        throw std::runtime_error("injected after rebuild+retirement+marginalization");return 0;
      });});
      require(s.calculateEstimate().equals(v,0)&&s.bpsam().getLinearizationPoint().equals(theta,0),"rollback state");
      require(s.timestamps()==times,"rollback times");require(s.getFactors().size()==factors.size(),"rollback slots");
      for(size_t i=0;i<factors.size();++i)require(s.getFactors()[i]==factors[i],"rollback factor pointer");
      require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==count,"rollback CBS bookkeeping");
      require((GaussianFactorGraph(s.bpsam()).augmentedHessian()-tree).norm()==0,"rollback tree");
      require((s.bpsam().marginalCovariance(X(3))-cov).norm()==0,"rollback covariance");
    }
    if(k==4&&force_rebuild){
      const auto live=s.calculateEstimate();
      s.bpsam().fl4qRebuild(fl4q::anchored(s.getFactors(),s.bpsam().getLinearizationPoint(),false),live,{});
    }
    s.update(graph,init,{{X(k),double(k)}});out.values.insert(X(k),s.calculateEstimate().at<Pose3>(X(k)));
    out.cov.push_back(s.bpsam().marginalCovariance(X(k)));
    if(k>=4){require(!s.calculateEstimate().exists(X(k-4)),"expired key retained");
      require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==0,"CBS retained across marginalization");}
  }return out;
}
void smartCache(){
  auto K=std::make_shared<Cal3_S2Stereo>(500,500,0,320,240,.2);
  Pose3 a,b(Rot3(),Point3(1,0,0));Point3 pt(.2,.3,5);
  auto s=std::make_shared<SmartStereoProjectionPoseFactor>(noiseModel::Isotropic::Sigma(3,1));
  s->add(StereoCamera(a,K).project(pt),X(0),K);s->add(StereoCamera(b,K).project(pt),X(1),K);
  Values v;v.insert(X(0),a);v.insert(X(1),b);s->error(v);const Point3 old=*s->point();
  NonlinearFactorGraph g;g.push_back(s);fl4q::SmartBackup saved(g,{});
  Values shifted;Pose3 G(Rot3(),Point3(2,3,1));shifted.insert(X(0),G*a);shifted.insert(X(1),G*b);
  fl4q::cost(g,shifted,v);require((*s->point()-old).norm()==0,"trial contaminated live cache");
  s->error(shifted);require((*s->point()-old).norm()>1,"cache fixture not mutated");
  saved.restore();require((*s->point()-old).norm()==0&&g[0]==s,"mutable cache rollback");
  std::cout<<"PASS independent smart trials and in-place cache rollback\n";
}
int main(){try{
  std::cout<<std::unitbuf;unsetenv("FL4Q_LOG");policyAndCommit();priorAndMarginal();smartCache();
  auto a=chain(false,false),b=chain(true,false),c=chain(true,true),d=chain(true,false);
  for(const auto* r:{&b,&c,&d}){require(a.values.equals(r->values,0),"fast path/rollback/repeat state mismatch");
    for(size_t i=0;i<a.cov.size();++i)require((a.cov[i]-r->cov[i]).norm()==0,"fast path/rollback/repeat covariance mismatch");}
  auto rebuilt1=chain(true,false,true),rebuilt2=chain(true,false,true);
  require(rebuilt1.values.equals(rebuilt2.values,0),"rebuilt chain repeat states");
  for(size_t i=0;i<rebuilt1.cov.size();++i)require((rebuilt1.cov[i]-rebuilt2.cov[i]).norm()==0,"rebuilt chain repeat covariance");
  std::cout<<"PASS disabled/ordinary exact parity, successful rebuilt chain through future marginals, CBS retirement, cached-LOCAL transaction rollback, deterministic repeat\n";return 0;
}catch(const std::exception& e){std::cerr<<"TEST FAILURE: "<<e.what()<<'\n';return 1;}}
