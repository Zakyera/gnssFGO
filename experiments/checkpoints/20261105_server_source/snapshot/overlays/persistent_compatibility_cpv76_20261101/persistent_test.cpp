#include "legacy_test.inc"

void persistentMode() {
  onRefresh();setenv("CPV69_RETAIN_COMPAT","0",1);setenv("CPV76_PERSISTENT_COMPAT","1",1);
}
auto persistentParams() {auto p=params();p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::Persistent;return p;}
void persistentGuards() {
  persistentMode();cbs::BPSAM b(persistentParams());b.cpv40ValidateMode();
  setenv("CPV76_PERSISTENT_COMPAT","0",1);rejects([&]{b.cpv40ValidateMode();});
  persistentMode();cbs::BPSAM wrong(params());rejects([&]{wrong.cpv40ValidateMode();});
  for(const auto& pair:std::vector<std::pair<const char*,const char*>>{{"CPV76_PERSISTENT_COMPAT","typo"},
      {"CPV69_RETAIN_COMPAT","1"},{"FL4M_RETIREMENT","retain"},{"CPV40_REFRESH","0"},
      {"CPV38_OFF_REFRESH","1"},{"CPK9_PRIOR_REFRESH","0"},{"FL4Q_ENABLE","1"}}) {
    persistentMode();setenv(pair.first,pair.second,1);rejects([]{cpv76::enabled();});
  }
  persistentMode();metric("persistent_selector_negative_checks",9);
}
void persistentReplacement() {
  persistentMode();Fixture f;cbs::BPSAM b(persistentParams());NonlinearFactorGraph g;Values v;
  for(int i=0;i<4;++i){g.push_back(f.factors(i));v.insert(f.initial(i));}
  cbs::BPSAM::UpdateParams u;u.constrainedKeys=groups(0,3);b.update(g,v,u);
  auto a=f.belief(1,2),version=a,alternative=a,rebind=f.belief(2,3);
  Vector6 d=Vector6::Zero();d[3]=.004;version.measured_from_to=a.measured_from_to.retract(d);
  version.covariance*=1.2;
  alternative.sender_from_pose_key=Symbol('g',41);alternative.sender_to_pose_key=Symbol('g',42);
  rebind.sender_from_pose_key=alternative.sender_from_pose_key;rebind.sender_to_pose_key=alternative.sender_to_pose_key;
  size_t cases=0;
  for(const auto& pending:std::vector<std::vector<cbs::BPSAM::CbsOdometryBelief>>{
        {a,version},{a},{version},{alternative},{rebind}}) {
    for(const auto& m:pending)b.addOdometryBeliefsDetailed({m});
    require(b.cpv40PreBounds({}).size()==1,"pending versions stacked");
    auto plan=b.cpv76Plan({}, {}, {});auto bounds=b.cpv40PreBounds({});
    b.update({}, {},u);b.cpv76ValidatePlan(plan);const auto receipt=b.cpv40TrackingReceipt();
    b.cpv40Refresh(bounds,{},groups(0,3));
    require(receipt==b.cpv40TrackingReceipt(),"persistent double ingestion");
    require(b.trackedCbsFactorSlotCount()==1&&b.activeWindowTemporaryCbsOdomFactorCount()==0,"persistent replacement stacking");
    const auto& expected=pending.back();auto peer=std::dynamic_pointer_cast<BetweenFactor<Pose3>>(b.trackedCbsOdomFactors().at(0));
    require(peer->keys()==KeyVector{expected.from_pose_key,expected.to_pose_key}&&peer->measured().equals(expected.measured_from_to,1e-12),"native replacement wrong mean/edge");
    require(rel(std::dynamic_pointer_cast<noiseModel::Gaussian>(peer->noiseModel())->covariance(),expected.covariance)<1e-12,"native covariance changed");
    auto bad=bounds;bad[0].maximum=-1;rejects([&]{b.cpv40ValidateActive(bad,{});});
    auto invalid=plan;invalid.factors.begin()->second.covariance*=2;
    rejects([&]{b.cpv76ValidatePlan(invalid);});++cases;
  }
  b.addOdometryBeliefsDetailed({a,alternative,rebind});
  require(b.cpv40PreBounds({}).size()==1,"pending alias/rebind stacked");
  auto plan=b.cpv76Plan({}, {}, {});auto bounds=b.cpv40PreBounds({});b.update({}, {},u);
  b.cpv76ValidatePlan(plan);b.cpv40Refresh(bounds,{},groups(0,3));
  require(b.trackedCbsFactorSlotCount()==1,"pending alternative replacement count");
  metric("persistent_replacement_cases",cases+1);
}

class FailOrdinary : public PriorFactor<Pose3> {
 public:
  mutable size_t calls=0;
  FailOrdinary(Key k,const Pose3& p):PriorFactor<Pose3>(k,p,noiseModel::Isotropic::Sigma(6,1)){}
  GaussianFactor::shared_ptr linearize(const Values&)const override{++calls;throw std::runtime_error("CP76 injected ordinary insertion/replacement failure");}
};
Output persistentLifecycle(bool inject) {
  persistentMode();Fixture f;Smoother s(3.5,persistentParams());Output out;
  size_t faults=0,absorbed=0;
  for(int k=0;k<13;++k) {
    auto graph=f.factors(k);auto initial=f.initial(k);
    FixedLagSmoother::KeyTimestampMap times{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}};
    if(k>=2)s.addOdometryBeliefsDetailed({f.belief(k-2,k-1)});
    if(k>=3){auto m=f.belief(k-3,k-2);Vector6 d=Vector6::Zero();d[4]=.002;
      m.measured_from_to=m.measured_from_to.retract(d);s.addOdometryBeliefsDetailed({m});}
    if(inject&&(k==2||k==3||k==6)) {
      Snapshot saved(s,X(k-1));const auto receipt=s.bpsam().cpv40TrackingReceipt();
      const auto smart=smartState(graph);
      const auto check=[&]{saved.check(s,X(k-1));require(receipt==s.bpsam().cpv40TrackingReceipt(),"persistent rollback ownership");require(smart==smartState(graph),"persistent rollback smart cache");};
      auto ordinary=std::make_shared<FailOrdinary>(X(k),f.states[k].pose());auto bad=graph;bad.push_back(ordinary);
      rejects([&]{s.update(bad,initial,times);});require(ordinary->calls==1,"ordinary fault was retried");check();++faults;
      auto refresh=std::make_shared<FailAfterPeerCommit>(X(k),f.states[k].pose(),s.bpsam(),X(k-2),X(k-1));bad=graph;bad.push_back(refresh);
      rejects([&]{s.update(bad,initial,times);});require(refresh->faults()==1,"persistent refresh fault missing");check();++faults;
      rejects([&]{fl4q::transaction(s,graph,[&]{s.update(graph,initial,times);throw std::runtime_error("CP76 post-marginal fault");return 0;});});check();++faults;
    }
    auto r=s.update(graph,initial,times);require(r.iterations==2,"persistent ordinary+refresh count");
    auto values=s.calculateEstimate();out.latest.insert(X(k),values.at<Pose3>(X(k)));
    out.cov.push_back(s.bpsam().marginalCovariance(X(k)));
    require(s.bpsam().trackedCbsFactorSlotCount()==size_t(k<2?0:k==2?1:2),"persistent unbounded factor tracking");
    require(s.bpsam().activeWindowTemporaryCbsOdomFactorCount()==0,"persistent polluted active-window ownership");
    s.bpsam().cpv40ValidateActive({},{});
    for(const auto& p:graph)if(std::dynamic_pointer_cast<Smart>(p))
      require(std::find(s.getFactors().begin(),s.getFactors().end(),p)!=s.getFactors().end(),"persistent replaced smart factor");
    if(k>=4){++absorbed;require(!values.exists(X(k-4)),"persistent stale endpoint");}
  }
  metric("persistent_absorption_boundaries",absorbed);metric("persistent_injected_rollback_cases",faults);
  return out;
}
int main(int argc,char**argv)try {
  require(argc==2,"snapshot root required");unsetenv("CPV76_PERSISTENT_COMPAT");
  std::filesystem::path root(argv[1]);std::filesystem::create_directories(root);
  std::string old=(root/"legacy").string();char* oldargs[]={argv[0],old.data()};
  require(cpv69_saved_main(2,oldargs)==0,"inherited corrected OFF/REMOVE/RETAIN regression");
  persistentGuards();persistentReplacement();
  auto a=persistentLifecycle(false),b=persistentLifecycle(false),c=persistentLifecycle(true);equal(a,b);equal(a,c);
  std::string snap=(root/"persistent").string();setenv("CPV66_AUDIT_DIR",snap.c_str(),1);setenv("CPV66_TEST_ALL","1",1);setenv("FL4M_AUDIT","/dev/null",1);
  auto observed=persistentLifecycle(false);unsetenv("CPV66_AUDIT_DIR");unsetenv("FL4M_AUDIT");equal(a,observed);
  boundaryChecks(snap);
  std::cout<<"PASS CP76 persistent native compatibility. No sensor replay, no accuracy or native-only LOCAL certificate.\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL CP76 "<<e.what()<<'\n';return 1;}
