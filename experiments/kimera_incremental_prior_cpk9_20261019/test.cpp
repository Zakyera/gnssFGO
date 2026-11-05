#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/Marginals.h>
#include <Eigen/Cholesky>
#include <iostream>
#include <iomanip>
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using Smoother=cbs::IncrementalFixedLagBpsamSmoother;
using Smart=SmartStereoProjectionPoseFactor;
void require(bool b,const std::string& s){if(!b)throw std::runtime_error(s);}
template<class F>void rejects(F action){bool caught=false;try{action();}catch(const std::exception&){caught=true;}require(caught,"expected rejection");}
void metric(const char* s,double x){require(std::isfinite(x),s);std::cout<<s<<'='<<std::setprecision(17)<<x<<'\n';}
double rel(const Matrix& a,const Matrix& b){return cpk9::relative(a,b);}
void mode(bool on){setenv("CPK9_PRIOR_REFRESH",on?"1":"0",1);setenv("FL4Q_ENABLE","0",1);setenv("FL4M_RETIREMENT","remove",1);}
cbs::BPSAM::Params params(){
  cbs::BPSAM::Params p;p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.sam_params_.relinearizeSkip=1;p.sam_params_.relinearizeThreshold=.01;
  p.sam_params_.optimizationParams=ISAM2GaussNewtonParams(.001);
  return p;
}
struct Fixture {
  imuBias::ConstantBias bias;
  PreintegratedImuMeasurements pim;
  std::vector<NavState> states;
  std::shared_ptr<Cal3_S2Stereo> camera=std::make_shared<Cal3_S2Stereo>(500,500,0,320,240,.2);
  Fixture():pim([]{auto p=PreintegrationParams::MakeSharedU(9.81);
    p->accelerometerCovariance=Matrix3::Identity()*.01;
    p->gyroscopeCovariance=Matrix3::Identity()*.0001;
    p->integrationCovariance=Matrix3::Identity()*1e-6;return p;}(),bias) {
    for(int i=0;i<10;++i)pim.integrateMeasurement(Vector3(.03,-.01,9.81),Vector3(.01,.02,-.01),.02);
    NavState state(Pose3(),Vector3(.7,.1,0));
    for(int i=0;i<14;++i){states.push_back(state);state=pim.predict(state,bias);}
  }
  Values initial(int i)const{
    Values v;Vector6 d;d<<.0002,-.0003,.0001,.002,-.001,.001;
    v.insert(X(i),states.at(i).pose().retract(d));
    v.insert(V(i),Vector3(states.at(i).velocity()+Vector3(.001,-.002,.001)));
    v.insert(B(i),bias);return v;
  }
  NonlinearFactorGraph factors(int i,bool visual=true)const{
    NonlinearFactorGraph g;
    if(!i){
      g.emplace_shared<PriorFactor<Pose3>>(X(0),states[0].pose(),noiseModel::Isotropic::Sigma(6,.1));
      g.emplace_shared<PriorFactor<Vector3>>(V(0),states[0].velocity(),noiseModel::Isotropic::Sigma(3,.2));
      g.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(0),bias,noiseModel::Isotropic::Sigma(6,.02));
    }else{
      g.emplace_shared<ImuFactor>(X(i-1),V(i-1),X(i),V(i),B(i-1),pim);
      g.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(i-1),B(i),bias,noiseModel::Isotropic::Sigma(6,.01));
      if(visual){
        auto s=std::make_shared<Smart>(noiseModel::Isotropic::Sigma(3,1));
        const Point3 point(.2,.3,5);
        for(int j=i-1;j<=i;++j)s->add(StereoCamera(states[j].pose(),camera).project(point),X(j),camera);
        g.push_back(s);
      }
    }return g;
  }
  auto belief(int a,int b)const{
    cbs::BPSAM::CbsOdometryBelief m;m.source_agent=static_cast<cbs::AgentId>('g');
    m.from_pose_key=X(a);m.to_pose_key=X(b);m.sender_from_pose_key=Symbol('g',a);m.sender_to_pose_key=Symbol('g',b);
    Vector6 d;d<<.002,-.001,.001,.01,-.003,.002;
    m.measured_from_to=states[a].pose().between(states[b].pose()).retract(d);
    Matrix6 L=Matrix6::Identity()*.05;L(3,0)=.004;L(4,1)=-.003;L(5,3)=.006;
    m.covariance=L*L.transpose();m.relax_factor=1;return m;
  }
};
FastMap<Key,int> groups(int a,int b){FastMap<Key,int> g;for(int i=a;i<=b;++i){g[X(i)]=i;g[V(i)]=i;g[B(i)]=i;}return g;}
KeyList retired(int i){return KeyList{X(i),V(i),B(i)};}
double equations(const cbs::BPSAM& b){
  const Ordering o(b.getLinearizationPoint().keys());
  const auto cached=b.fl4nLinearCache().hessian(o);
  const auto fresh=b.getFactorsUnsafe().clone().linearize(b.getLinearizationPoint())->hessian(o);
  const auto tree=GaussianFactorGraph(b).hessian(o);
  return std::max({rel(cached.first,fresh.first),rel(cached.second,fresh.second),
                   rel(cached.first,tree.first),rel(cached.second,tree.second)});
}
void directIntegration(){
  Fixture f;cbs::BPSAM b(params());NonlinearFactorGraph graph;Values init;
  for(int i=0;i<3;++i){graph.push_back(f.factors(i,false));init.insert(f.initial(i));}
  cbs::BPSAM::UpdateParams up;up.constrainedKeys=groups(0,2);
  b.update(graph,init,up);b.marginalizeLeaves(retired(0));
  const auto before=b.calculateEstimate(),theta=b.getLinearizationPoint();
  const auto delta=b.getDelta();const auto fixed=b.getFixedVariables();
  const auto pointers=b.getFactorsUnsafe();const auto cache=b.fl4nLinearCache();
  const Matrix tree=GaussianFactorGraph(b).augmentedHessian();
  require(!fixed.empty(),"fixture needs native marginal locks");
  // Invalid ownership and corrupted cache must reject atomically.
  auto rejectGraph=pointers;auto rejectFixed=fixed;rejectFixed.insert(X(999));
  rejects([&]{cpk9::promote(rejectGraph,theta,cache,rejectFixed);});
  require(rejectFixed.count(X(999)),"rejection removed unknown lock");
  for(size_t i=0;i<pointers.size();++i)require(rejectGraph[i]==pointers[i],"rejection partially promoted");
  auto badCache=cache;
  for(size_t i=0;i<pointers.size();++i)if(dynamic_cast<LinearContainerFactor*>(pointers[i].get())){badCache[i].reset();break;}
  rejectFixed=fixed;rejects([&]{cpk9::promote(rejectGraph,theta,badCache,rejectFixed);});
  require(rejectFixed==fixed,"cache rejection changed locks");
  const auto promoted=b.cpk9PromotePriorCharts();
  require(promoted.priors>0 && promoted.released==fixed.size(),"no native prior promotion");
  require(b.calculateEstimate().equals(before,0)&&b.getLinearizationPoint().equals(theta,0),"promotion moved states");
  require(b.getDelta().equals(delta,0),"promotion changed delta");
  require((GaussianFactorGraph(b).augmentedHessian()-tree).norm()==0,"promotion rebuilt tree");
  double energy=0;
  Values displaced=before;Vector6 shift;shift<<.1,-.07,.04,.2,-.1,.05;
  displaced.update(X(1),before.at<Pose3>(X(1)).retract(shift));
  for(size_t i=0;i<pointers.size();++i){
    if(auto p=dynamic_cast<LinearContainerFactor*>(pointers[i].get())){
      const double expected=cbs::fixedLagFactorErrorInChart(*p,displaced,theta);
      energy=std::max(energy,std::abs(expected-b.getFactorsUnsafe()[i]->error(displaced))/(1+std::abs(expected)));
    }else require(b.getFactorsUnsafe()[i]==pointers[i],"nonprior pointer changed");
    require(b.fl4nLinearCache()[i]==cache[i],"promotion changed Gaussian cache");
  }
  require(energy<1e-10,"original prior energy changed");
  // Real nonlinear information change followed by native iSAM2 refresh.
  NonlinearFactorGraph peer;
  auto m=f.belief(1,2);peer.emplace_shared<BetweenFactor<Pose3>>(X(1),X(2),m.measured_from_to,noiseModel::Gaussian::Covariance(m.covariance));
  up.constrainedKeys=groups(1,2);up.force_relinearize=true;up.forceFullSolve=true;
  const auto result=b.update(peer,{},up);const auto slot=result.newFactorsIndices.back();
  const auto preserved=b.getFactorsUnsafe();
  b.update({}, {}, up);
  const double cacheError=equations(b);require(cacheError<1e-7,"refresh cache/tree/dense mismatch");
  require(!b.getLinearizationPoint().equals(theta,1e-10),"native theta did not refresh");
  // Remove the peer, then compute an independent native dense marginal.
  up.removeFactorIndices={slot};b.update({}, {}, up);
  require(!b.getFactorsUnsafe()[slot],"peer not removed");
  const Ordering eliminate=Ordering(KeyVector{X(1),V(1),B(1)});
  const Ordering remain=Ordering(KeyVector{X(2),V(2),B(2)});
  const auto linear=b.fl4nLinearCache();
  const auto dense=linear.eliminatePartialSequential(eliminate,EliminateCholesky).second->hessian(remain);
  b.marginalizeLeaves(retired(1));
  const auto actual=b.fl4nLinearCache().hessian(remain);
  const double marginalError=std::max(rel(actual.first,dense.first),rel(actual.second,dense.second));
  require(marginalError<1e-7,"second native marginal differs from peer-removed reference");
  const auto second=b.cpk9PromotePriorCharts();require(second.priors>0,"second lifecycle promotion missing");
  // Surviving first wrapper object/energy never changed even when absorbed later.
  for(size_t i=0;i<pointers.size();++i)if(dynamic_cast<LinearContainerFactor*>(pointers[i].get())){
    const double e=cbs::fixedLagFactorErrorInChart(*pointers[i],displaced,theta);
    require(std::abs(preserved[i]->error(displaced)-e)<1e-10*(1+std::abs(e)),"old prior anchor mutated");
  }
  metric("promotion_cache_relative_error",promoted.cache_error);
  metric("original_prior_energy_relative_error",energy);
  metric("fresh_cache_tree_relative_error",cacheError);
  metric("second_marginal_relative_error",marginalError);
}
struct Output{Values latest;std::vector<Matrix> cov;size_t promoted=0;};
Output chain(bool candidate,bool peer,bool injectFailure=false){
  mode(candidate);Fixture f;Smoother s(3.5,params());Output out;
  for(int k=0;k<12;++k){
    auto graph=f.factors(k);auto init=f.initial(k);
    if(peer&&k>=2){const auto b=f.belief(k-2,k-1);
      const auto result=s.addOdometryBeliefsDetailed({b});
      require(result.details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"CBS admission failed");}
    FixedLagSmoother::KeyTimestampMap stamps{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}};
    if(injectFailure&&k==6){
      s.bpsam().setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
      const auto old=s.calculateEstimate(),theta=s.bpsam().getLinearizationPoint();
      const auto times=s.timestamps();const auto factors=s.getFactors();const auto fixed=s.bpsam().getFixedVariables();
      const auto tree=GaussianFactorGraph(s.bpsam()).augmentedHessian();
      const auto cov=s.bpsam().marginalCovariance(X(k-1));
      const auto count=s.bpsam().activeWindowTemporaryCbsOdomFactorCount();
      std::vector<std::pair<std::shared_ptr<Smart>,Point3>> smart;
      for(const auto& x:factors)if(auto p=std::dynamic_pointer_cast<Smart>(x)){
        p->error(old);if(p->point())smart.emplace_back(p,*p->point());}
      rejects([&]{fl4q::transaction(s,graph,[&]{s.update(graph,init,stamps);
        throw std::runtime_error("injected after actual promotion/update/marginalization");return 0;});});
      require(s.calculateEstimate().equals(old,0)&&s.bpsam().getLinearizationPoint().equals(theta,0),"transaction states");
      require(s.timestamps()==times&&s.bpsam().getFixedVariables()==fixed,"transaction timestamps/locks");
      require(s.getFactors().size()==factors.size(),"transaction slots");
      for(size_t i=0;i<factors.size();++i)require(factors[i]==s.getFactors()[i],"transaction factor identity");
      require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==count,"transaction CBS tracking");
      require((GaussianFactorGraph(s.bpsam()).augmentedHessian()-tree).norm()==0,"transaction tree");
      require((s.bpsam().marginalCovariance(X(k-1))-cov).norm()==0,"transaction LOCAL covariance");
      for(const auto& x:smart)require((*x.first->point()-x.second).norm()==0,"transaction smart cache");
    }
    s.update(graph,init,stamps);
    const auto current=s.calculateEstimate();out.latest.insert(X(k),current.at<Pose3>(X(k)));
    out.cov.push_back(s.bpsam().marginalCovariance(X(k)));
    // Newly added native smart object must remain the same object, not a new solver.
    for(const auto& x:graph)if(std::dynamic_pointer_cast<Smart>(x)){
      require(std::find(s.getFactors().begin(),s.getFactors().end(),x)!=s.getFactors().end(),"native smart replaced");}
    for(const auto& x:s.getFactors())if(dynamic_cast<fl4l::OriginalChartPrior*>(x.get()))++out.promoted;
    if(k>=4){require(!current.exists(X(k-4)),"fixed-lag state not retired");
      for(const auto& x:s.getFactors())if(x)for(Key key:x->keys())require(current.exists(key),"dangling factor key");}
  }
  return out;
}
void equal(const Output& a,const Output& b){
  require(a.latest.equals(b.latest,0),"exact output parity");require(a.cov.size()==b.cov.size(),"cov count");
  for(size_t i=0;i<a.cov.size();++i)require((a.cov[i]-b.cov[i]).norm()==0,"exact covariance parity");
}
int main()try{
  std::cout<<std::unitbuf;spdlog::set_level(spdlog::level::err);
  for(const char* e:{"FL4M_AUDIT","FL4N_CAPTURE","FL4Q_LOG","CPK1_PROBE","FL4U_PROBE"})unsetenv(e);
  mode(true);directIntegration();
  auto off0=chain(false,false),off1=chain(true,false);equal(off0,off1);
  require(off1.promoted==0,"OFF modified prior representation");
  auto on0=chain(false,true),on=chain(true,true),repeat=chain(true,true),rollback=chain(true,true,true);
  equal(on,repeat);equal(on,rollback);
  const double candidateEffect=on0.latest.localCoordinates(on.latest).norm();
  require(candidateEffect>1e-10,"actual wrapper candidate not discriminating");
  mode(true);setenv("FL4Q_ENABLE","1",1);rejects([]{cpk9::enabled();});
  mode(true);setenv("FL4M_RETIREMENT","retain",1);rejects([]{cpk9::enabled();});
  mode(true);setenv("CPK9_PRIOR_REFRESH","typo",1);rejects([]{cpk9::enabled();});mode(false);
  metric("actual_wrapper_candidate_state_difference",candidateEffect);
  std::cout<<"PASS CP-K9 incremental/retirement integration; no real-data stability certificate\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
