// Reuse existing fixture helpers, not its main/test execution.
#define main CPK19_LEGACY_UNUSED_MAIN
#include "legacy_test.cpp"
#undef main

void SameGraph(const f::State& a,const f::State& b) {
  Check(SameValues(a.solver.estimate(),b.solver.estimate()),"no-op complete X/V/W estimate exact");
  Check(SameValues(a.solver.getLinearizationPoint(),b.solver.getLinearizationPoint()),"no-op theta exact");
  Check(a.solver.timestamps()==b.solver.timestamps(),"no-op timestamps exact");
  Check(a.solver.getFactors().size()==b.solver.getFactors().size(),"no-op slot extent exact");
  for(size_t i=0;i<a.solver.getFactors().size();++i)
    Check(a.solver.getFactors()[i]==b.solver.getFactors()[i],"no-op factor/prior pointer exact");
  Check(a.active.size()==b.active.size()&&a.solver.retirements==b.solver.retirements,
        "no-op active IDs/retirements exact");
  const auto va=a.solver.estimate(),vb=b.solver.estimate();
  const auto ha=a.solver.getFactors().linearize(va)->augmentedHessian(Ordering(va.keys()));
  const auto hb=b.solver.getFactors().linearize(vb)->augmentedHessian(Ordering(vb.keys()));
  Check((ha-hb).cwiseAbs().maxCoeff()==0,"no-op complete linearized objective exact");
  const auto ca=Cov(a.solver.getFactors(),va),cb=Cov(b.solver.getFactors(),vb);
  Check((ca-cb).cwiseAbs().maxCoeff()==0,"no-op full joint covariance exact");
}

void Finalization() {
  f::Receiver r(testRoot/"sealed",1.,f::AbsorbedRevisionPolicy::SealAtAbsorption);
  f::Receiver filtered(testRoot/"filtered_strict",1.);
  Boot(r);Boot(filtered);
  auto b=Bel("camera",23000000,171000000,200000000);
  for(int i=1;i<=11;++i) {
    const auto t=Origin+i*f::Period;r.imuClock(t,t);filtered.imuClock(t,t);
    if(i==2){r.belief(b);filtered.belief(b);}
    if(i==10){b.revision=1;b.covariance*=1.1;b.arrival=t;r.belief(b);filtered.belief(b);
      Check(r.state().active.count(b.id())==1,"exact lag boundary retains factor");}
    Check(SameValues(r.state().solver.estimate(),filtered.state().solver.estimate()),"strict/sealed active prefix exact");
  }
  Check(r.state().counts.absorbed==1&&r.state().active.empty(),"fractional first bracket absorption seals ID");
  const auto terminal=r.archive().terminal(b.id());
  Check(terminal.belief.revision==1&&terminal.belief.start==b.start&&terminal.belief.end==b.end,
        "terminal accepted revision/endpoints retained");
  Check((terminal.belief.covariance-b.covariance).cwiseAbs().maxCoeff()==0,"terminal full dense covariance exact");
  b.arrival=Origin+11*f::Period;b.revision=2;b.mean=b.mean.retract(Vector6::Constant(.0003));b.covariance*=1.2;
  const auto before=r.state();const auto files=Files(r.archive().root());
  Throws([&]{filtered.belief(b);},"after absorption");
  for(auto fail:{f::Failure::BeforeOutput,f::Failure::OutputStaged}) {
    Throws([&]{r.belief(b,fail);},"injected_");Unchanged(r,before,files);
  }
  r.belief(b);SameGraph(r.state(),before);
  Check(r.state().counts.expired==1&&r.archive().seenExpired(b),"new expired revision audited atomically");
  auto audit=Files(r.archive().root());bool fullAudit=false;
  for(const auto& [path,txt]:audit)if(txt.find("belief,expired_revision,")!=std::string::npos)
    fullAudit=txt.find("belief,sealed_reference,")!=std::string::npos;
  Check(fullAudit,"expired payload plus sealed reference recorded");
  r.belief(b);Check(r.state().counts.expiredDuplicate==1,"expired duplicate accounted separately");
  auto accepted=terminal.belief;accepted.arrival=b.arrival;r.belief(accepted);
  Check(r.state().counts.expiredDuplicate==2,"duplicate of absorbed accepted payload checked");
  accepted.revision=0;r.belief(accepted);
  Check(r.state().counts.expiredStale==1,"older sealed revision classified stale not accepted");
  auto stable=r.state();auto stableFiles=Files(r.archive().root());
  auto bad=b;bad.mean=bad.mean.retract(Vector6::Constant(.01));
  Throws([&]{r.belief(bad);},"conflicting duplicate");Unchanged(r,stable,stableFiles);
  bad=terminal.belief;bad.arrival=b.arrival;bad.covariance*=2;
  Throws([&]{r.belief(bad);},"conflicting duplicate");Unchanged(r,stable,stableFiles);
  bad=b;bad.start=Origin-1;Throws([&]{r.belief(bad);},"endpoints changed");Unchanged(r,stable,stableFiles);
  bad=b;bad.end++;Throws([&]{r.belief(bad);},"endpoints changed");
  bad=b;bad.covariance(0,0)=-1;Throws([&]{r.belief(bad);},"nonpositive covariance");
  bad=b;bad.covariance(0,1)+=.1;Throws([&]{r.belief(bad);},"asymmetric covariance");
  bad=b;bad.mean=Pose3(Rot3(),Point3(NAN,0,0));Throws([&]{r.belief(bad);},"nonfinite belief");
  bad=b;bad.source="unknown";Throws([&]{r.belief(bad);},"retired support");
  Unchanged(r,stable,stableFiles);
  SameGraph(r.state(),before);
  // Future trajectory and covariance equal an independent strict receiver with
  // only the explicitly expired events filtered out; no extra optimization.
  for(int i=12;i<=35;++i) {
    const auto t=Origin+i*f::Period;r.imuClock(t,t);filtered.imuClock(t,t);
    auto fresh=Bel("lidar",(i-2)*f::Period,i*f::Period,i*f::Period);
    r.belief(fresh);filtered.belief(fresh);
    if(i==15){b.arrival=t;b.revision=3;const auto snap=r.state();r.belief(b);SameGraph(r.state(),snap);}
    Check(SameValues(r.state().solver.estimate(),filtered.state().solver.estimate()),"filtered oracle future state exact");
    Check(r.state().solver.timestamps().size()==33,"bounded 1s test lag even with expired versions");
  }
  const auto va=r.state().solver.estimate(),vb=filtered.state().solver.estimate();
  Check((Cov(r.state().solver.getFactors(),va)-Cov(filtered.state().solver.getFactors(),vb)).cwiseAbs().maxCoeff()==0,
        "filtered oracle complete final covariance exact");
  r.end(Origin+36*f::Period);filtered.end(Origin+36*f::Period);
  const auto& c=r.state().counts;
  Check(c.received==c.inserted+c.replaced+c.expired+c.duplicate+c.stale+c.excluded+c.unsupportedEnd,
        "expired messages included in conservation not acceptance");
  Check(c.inserted==r.state().active.size()+c.absorbed,"ID conservation unchanged");
  Check(c.expired==5&&c.expiredDuplicate==2&&c.expiredStale==1,"exact expiration counts");
  ArchiveCoverage(r,36);
}

int main(){try{
  const char* root=std::getenv("FL3I_TEST_ROOT");f::Require(root,"missing test root");testRoot=root;
  f::Require(fs::create_directory(testRoot),"test root already exists");
  Finalization();std::cout<<"CHECKS "<<checks<<"\nCPK19_FINALIZATION_GATE PASS\n";return 0;
}catch(const std::exception& e){std::cout<<"FAIL "<<e.what()<<'\n';return 1;}}
