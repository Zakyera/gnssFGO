#include "off_tests.inc"

void onRefresh() { offRefresh(false); setenv("CPV40_REFRESH","1",1); }
double localOn(cbs::BPSAM& b,Key a,Key z) {
  auto graph=b.getFactorsUnsafe(); const auto peers=b.trackedCbsOdomFactors();
  size_t removed=0;
  for(auto& p:graph) if(p && std::find(peers.begin(),peers.end(),p)!=peers.end()){p.reset();++removed;}
  require(removed==peers.size(),"independent LOCAL exclusion count");
  const auto values=b.calculateEstimate();const Ordering order(values.keys());
  const auto H=graph.clone().linearize(values)->hessian(order).first;
  Eigen::LLT<Matrix> llt(H); require(llt.info()==Eigen::Success,"native LOCAL not SPD");
  const Matrix inverse=llt.solve(Matrix::Identity(H.rows(),H.cols()));
  std::map<Key,size_t> offset;size_t cursor=0;
  for(Key k:order){offset[k]=cursor;cursor+=values.at(k).dim();}
  Matrix expected(12,12);
  expected<<inverse.block(offset[a],offset[a],6,6),inverse.block(offset[a],offset[z],6,6),
            inverse.block(offset[z],offset[a],6,6),inverse.block(offset[z],offset[z],6,6);
  b.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
  const Matrix info=b.jointMarginalInformation({a,z}).fullMatrix();
  const Matrix actual=info.llt().solve(Matrix::Identity(12,12));
  require(actual.allFinite()&&Eigen::LLT<Matrix>(actual).info()==Eigen::Success,"LOCAL nonfinite/nonSPD");
  require(expected.block(0,6,6,6).norm()>1e-8,"missing LOCAL cross blocks");
  const double e=rel(actual,expected);require(e<=1e-6,"peer-excluded LOCAL covariance mismatch");return e;
}

void onReference() {
  onRefresh(); Fixture f; cbs::BPSAM b(params());NonlinearFactorGraph g;Values v;
  for(int i=0;i<3;++i){g.push_back(f.factors(i));v.insert(f.initial(i));}
  cbs::BPSAM::UpdateParams u;u.constrainedKeys=groups(0,2);
  b.update(g,v,u);b.marginalizeLeaves(retired(0));
  auto admission=b.addOdometryBeliefsDetailed({f.belief(1,2)});
  require(admission.details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"direct ON admission");
  const auto bounds=b.cpv40PreBounds({});require(bounds.size()==1,"staged bound absent");
  u.constrainedKeys=groups(1,2);b.update({}, {}, u);
  require(b.activeWindowTemporaryCbsOdomFactorCount()==1,"direct actual insertion");
  const auto theta=b.getLinearizationPoint(),before=b.calculateBestEstimate();const auto original=b.getFactorsUnsafe();
  auto objective=fl4q::anchored(original,theta,true);
  const auto expected=before.retract(objective.linearize(before)->optimize());
  const auto receipt=b.cpv40TrackingReceipt();
  const auto r=b.cpv40Refresh(bounds,{},groups(1,2));
  require(receipt==b.cpv40TrackingReceipt(),"refresh processed CBS again/counter changed");
  require(r.variablesRelinearized==before.size(),"ON not all keys refreshed");
  const double stateError=expected.localCoordinates(b.calculateEstimate()).norm()/(1+before.localCoordinates(expected).norm());
  const double thetaError=before.localCoordinates(b.getLinearizationPoint()).norm();
  require(stateError<=1e-7&&thetaError<=1e-9,"ON independent current GN reference");
  double energyError=0;size_t priors=0;Values displaced=before;Vector6 d;d<<.1,-.07,.04,.2,-.1,.05;
  displaced.update(X(1),before.at<Pose3>(X(1)).retract(d));
  for(size_t i=0;i<original.size();++i){
    if(auto p=dynamic_cast<LinearContainerFactor*>(original[i].get())){
      ++priors;double e=cbs::fixedLagFactorErrorInChart(*p,displaced,theta);
      energyError=std::max(energyError,std::abs(b.getFactorsUnsafe()[i]->error(displaced)-e)/(1+std::abs(e)));
    }else require(original[i]==b.getFactorsUnsafe()[i],"ON native/peer pointer changed");
  }
  require(priors&&energyError<=1e-10,"ON original prior function");
  const double eq=equations(b);require(eq<=1e-7,"ON fresh/cache/tree equations");
  const double cov=localOn(b,X(1),X(2));
  // Same unchanged guard must reject; failure itself is wrapped transactionally
  // in the smoother. Exercise active and staged bound paths explicitly.
  auto bad=bounds;bad[0].maximum=-1;rejects([&]{b.cpv40ValidateActive(bad,{});});
  auto removals=b.activeWindowTemporaryCbsOdomFactorRemovalsOutsideKeys({X(2),V(2),B(2)});
  require(removals.size()==1,"direct retirement fixture");
  u.removeFactorIndices=b.activeWindowTemporaryCbsOdomFactorRemovalSlots(removals);
  b.update({}, {},u);b.cpv40Refresh({},removals,groups(1,2));
  b.commitActiveWindowTemporaryCbsOdomFactorRemovals(removals);
  require(b.activeWindowTemporaryCbsOdomFactorCount()==0&&b.trackedCbsFactorSlotCount()==0,"direct retirement tracking");
  const auto dense=b.fl4nLinearCache().eliminatePartialSequential(Ordering(KeyVector{X(1),V(1),B(1)}),EliminateCholesky)
      .second->hessian(Ordering(KeyVector{X(2),V(2),B(2)}));
  b.marginalizeLeaves(retired(1));const auto actual=b.fl4nLinearCache().hessian(Ordering(KeyVector{X(2),V(2),B(2)}));
  const double schur=std::max(rel(actual.first,dense.first),rel(actual.second,dense.second));
  require(schur<=1e-7,"ON peer-removed next Schur");
  metric("ON_independent_GN",stateError);metric("ON_theta",thetaError);metric("ON_prior_function",energyError);
  metric("ON_fresh_cache_tree",eq);metric("ON_LOCAL_dense_inverse",cov);metric("ON_next_Schur",schur);
}

class FailAfterPeerCommit : public PriorFactor<Pose3> {
  const cbs::BPSAM& b_; Key from_,to_; mutable size_t faults_=0;
 public:
  FailAfterPeerCommit(Key k,const Pose3& p,const cbs::BPSAM& b,Key from,Key to)
    :PriorFactor<Pose3>(k,p,noiseModel::Isotropic::Sigma(6,1)),b_(b),from_(from),to_(to){}
  GaussianFactor::shared_ptr linearize(const Values& v)const override{
    bool registered=false;
    for(const auto& factor:b_.trackedCbsOdomFactors())
      if(factor && factor->keys()==KeyVector{from_,to_})registered=true;
    if(registered && b_.cpv40PreBounds({}).empty()){
      ++faults_;throw std::runtime_error("injected after peer commit at extra refresh");
    }
    return PriorFactor<Pose3>::linearize(v);
  }
  size_t faults()const{return faults_;}
};

Output onLifecycle(bool inject, bool retain=false) {
  onRefresh();Fixture f;auto p=params();p.active_cbs_odom_factor_diagnostics_enable=true;
  setenv("CPV69_RETAIN_COMPAT",retain?"1":"0",1);
  if(retain)setenv("FL4M_RETIREMENT","retain",1);
  Smoother s(3.5,p);Output out;size_t retirements=0;double cov=0;
  for(int k=0;k<13;++k){
    auto graph=f.factors(k);auto init=f.initial(k);
    const FixedLagSmoother::KeyTimestampMap times{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}};
    if(k>=2){
      const auto m=f.belief(k-2,k-1);auto a=s.addOdometryBeliefsDetailed({m});
      require(a.details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"lifecycle pending admission");
      const auto receipt=s.bpsam().cpv40TrackingReceipt();s.addOdometryBeliefsDetailed({m});
      require(receipt==s.bpsam().cpv40TrackingReceipt(),"pending duplicate changed tracking");
    }
    if(inject&&(k==2||k==6)){
      Snapshot snapshot(s,X(k-1));const auto receipt=s.bpsam().cpv40TrackingReceipt();
      auto fail=std::make_shared<FailAfterPeerCommit>(X(k),f.states[k].pose(),s.bpsam(),X(k-2),X(k-1));auto failing=graph;failing.push_back(fail);
      const auto smart=smartState(graph);
      rejects([&]{s.update(failing,init,times);});require(fail->faults()==1,"ON failure not at extra step");
      snapshot.check(s,X(k-1));require(receipt==s.bpsam().cpv40TrackingReceipt(),"ON rollback pending/active/counters");
      require(smart==smartState(graph),"ON rollback new smart cache");
      rejects([&]{fl4q::transaction(s,graph,[&]{s.update(graph,init,times);throw std::runtime_error("post marginalization fault");return 0;});});
      snapshot.check(s,X(k-1));require(receipt==s.bpsam().cpv40TrackingReceipt(),"ON post-marg tracking rollback");
      require(smart==smartState(graph),"ON post-marg new smart cache");
      metric("ON_fault_rollback_at_state",k);
    }
    const auto result=s.update(graph,init,times);require(result.iterations==2,"ON iterations");
    const auto state=s.calculateEstimate();out.latest.insert(X(k),state.at<Pose3>(X(k)));
    const size_t expected=k<2?0:(k==2?1:2);
    require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==expected &&
            s.bpsam().trackedCbsFactorSlotCount()==expected,"ON active count/retirement mismatch");
    s.bpsam().cpv40ValidateActive({},{});
    if(k>=2){const auto receipt=s.bpsam().cpv40TrackingReceipt();s.addOdometryBeliefsDetailed({f.belief(k-2,k-1)});
      require(receipt==s.bpsam().cpv40TrackingReceipt(),"active duplicate changed tracking");}
    for(const auto& p:graph)if(std::dynamic_pointer_cast<Smart>(p))
      require(std::find(s.getFactors().begin(),s.getFactors().end(),p)!=s.getFactors().end(),"ON native smart identity");
    if(k>=1)cov=std::max(cov,localOn(s.bpsam(),X(k-1),X(k)));
    out.cov.push_back(s.bpsam().marginalCovariance(X(k)));
    if(k>=4){++retirements;require(!state.exists(X(k-4)),"ON stale state");}
    for(const auto& p:s.getFactors())if(p)for(Key key:p->keys())require(state.exists(key),"ON dangling keys");
  }
  require(retirements==9,"ON marginal lifecycle coverage");metric("ON_lifecycle_LOCAL_covariance",cov);return out;
}

int cpv40_saved_main()try{
  setenv("CPV40_REFRESH","0",1);require(cpv38_saved_main()==0,"legacy OFF suite");
  onRefresh();setenv("CPV38_OFF_REFRESH","1",1);rejects([]{cpv40::enabled();});
  onRefresh();setenv("CPV40_REFRESH","typo",1);rejects([]{cpv40::enabled();});
  onRefresh();setenv("CPK9_PRIOR_REFRESH","0",1);rejects([]{cpv40::enabled();});
  onRefresh();onReference();auto a=onLifecycle(false),b=onLifecycle(false),c=onLifecycle(true);equal(a,b);equal(a,c);
  std::cout<<"PASS CP-V40 private ON refresh admission; no real-data accuracy claim\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL CPV40 "<<e.what()<<'\n';return 1;}
