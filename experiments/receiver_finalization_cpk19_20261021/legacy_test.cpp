#include "Receiver.h"
#include "solver/IncrementalFixedLagSmoother.h"
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>
#include <unistd.h>

using namespace gtsam;using namespace fgo::offline;
namespace f=fl3i;
namespace fs=std::filesystem;
static size_t checks=0;
static fs::path testRoot;
constexpr int64_t Origin=1727775000145000000LL;
void Check(bool yes,const std::string& name){++checks;if(!yes)throw std::runtime_error(name);std::cout<<"PASS "<<name<<'\n';}
void Metric(const std::string& name,double x){std::cout<<"METRIC "<<name<<' '<<std::setprecision(17)<<x<<'\n';}
template<class Fn>void Throws(Fn fn,const std::string& phrase) {
  bool caught=false;try{fn();}catch(const std::exception& e){caught=std::string(e.what()).find(phrase)!=std::string::npos;if(!caught)throw;}
  Check(caught,"fail closed: "+phrase);
}
bool SameValues(const Values& a,const Values& b) {
  if(a.keys()!=b.keys())return false;
  for(auto k:a.keys()) {
    if(Symbol(k).chr()=='x'){if(!(a.at<Pose3>(k).matrix().array()==b.at<Pose3>(k).matrix().array()).all())return false;}
    else if(!(a.at<Vector3>(k).array()==b.at<Vector3>(k).array()).all())return false;
  }return true;
}
double Difference(const Values& a,const Values& b){double d=0;Check(a.keys()==b.keys(),"same state keys");for(auto k:a.keys())d=std::max(d,a.at(k).localCoordinates_(b.at(k)).cwiseAbs().maxCoeff());return d;}
Matrix Cov(const NonlinearFactorGraph& graph,const Values& value) {
  const auto ab=graph.linearize(value)->jacobian(Ordering(value.keys()));
  Eigen::HouseholderQR<Matrix> qr(ab.first);
  Matrix r=qr.matrixQR().topRows(ab.first.cols()).triangularView<Eigen::Upper>();
  Matrix ri=r.triangularView<Eigen::Upper>().solve(Matrix::Identity(r.rows(),r.cols()));return ri*ri.transpose();
}
std::map<std::string,std::string> Files(const fs::path& root) {
  std::map<std::string,std::string> out;
  for(const auto& p:fs::recursive_directory_iterator(root))if(p.is_regular_file()) {
    std::ifstream in(p.path());std::ostringstream s;s<<in.rdbuf();out[p.path().lexically_relative(root).string()]=s.str();
  }return out;
}
void Unchanged(const f::Receiver& r,const f::State& before,const std::map<std::string,std::string>& files) {
  const auto& s=r.state();
  Check(SameValues(s.solver.estimate(),before.solver.estimate())&&SameValues(s.solver.getLinearizationPoint(),before.solver.getLinearizationPoint()),"exact solver rollback");
  Check(s.event==before.event&&s.lastArrival==before.lastArrival&&s.newest==before.newest&&s.oldest==before.oldest&&
    s.initialized==before.initialized&&s.closed==before.closed&&s.counts==before.counts&&
    s.solver.timestamps()==before.solver.timestamps()&&s.solver.publicationCursor==before.solver.publicationCursor&&
    s.solver.windowRebuilds==before.solver.windowRebuilds&&s.solver.retiredBlocks==before.solver.retiredBlocks&&
    s.solver.convertedMarginals==before.solver.convertedMarginals&&s.solver.retirements==before.solver.retirements,"exact maps/counters/output cursor rollback");
  Check(s.pending.size()==before.pending.size()&&s.buffered.size()==before.buffered.size()&&s.active.size()==before.active.size(),"queues/ledger rollback");
  for(const auto& [id,e]:before.active){const auto& now=s.active.at(id);Check(e.owner==now.owner&&e.slot==now.slot&&e.latest.revision==now.latest.revision,"source owner rollback");}
  Check(s.solver.getFactors().size()==before.solver.getFactors().size(),"slot extent rollback");
  for(size_t i=0;i<s.solver.getFactors().size();++i)f::Require(s.solver.getFactors()[i]==before.solver.getFactors()[i],"slot pointer rollback");
  Check(Files(r.archive().root())==files,"no partial or rewritten archive output");
}
void Boot(f::Receiver& r,int64_t availability=0) {
  for(int i=0;i<59;++i){int64_t t=Origin-(58-i)*2500000;r.imuClock(t,std::max(t,availability));}
  Check(r.state().initialized&&r.state().origin==Origin&&r.state().newest==0,"59-sample independent clock; no IMU measurement factor");
}
Pose3 Truth(double t){return Pose3(Rot3::Expmap(Vector3(.012*t,-.005*t,.026*t)),Point3(.16*t,.002*t*t,.001*t*t));}
f::Belief Bel(const std::string& source,int64_t start,int64_t end,int64_t now,uint64_t rev=0) {
  f::Belief b;b.source=source;b.session="one";b.from=uint64_t(start);b.to=uint64_t(end);b.ordinal=b.to;b.revision=rev;
  b.start=Origin+start;b.end=Origin+end;b.arrival=Origin+now;
  b.mean=Truth(start*1e-9).between(Truth(end*1e-9));
  if(rev)b.mean=b.mean.retract((Vector6()<<.0001,-.00007,.00003,.0002,-.0001,.00008).finished());
  Matrix6 l=Matrix6::Identity();l(3,0)=.13;l(1,4)=-.17;l(5,2)=.11;b.covariance=(rev?1.2e-4:1e-4)*l*l.transpose();return b;
}

// Independent bare-iSAM2 control, no lifecycle adapter and no retirement.
class Control {
 public:
  ISAM2 solver{fl3i_solver::Parameters()};std::map<f::Id,std::pair<f::Belief,size_t>> slots;size_t newest=0;
  Control(){NonlinearFactorGraph g;Values v;InsertSourceWnoaState(v,0,{});
    g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-5)));
    g.emplace_shared<PriorFactor<Vector3>>(V(0),Vector3::Zero(),noiseModel::Diagonal::Sigmas(Vector3::Constant(10.)));
    g.emplace_shared<PriorFactor<Vector3>>(W(0),Vector3::Zero(),noiseModel::Diagonal::Sigmas(Vector3::Constant(3.)));solver.update(g,v);refine();}
  void support(){++newest;Values v;InsertSourceWnoaState(v,newest,PredictSourceWnoa(ReadSourceWnoaState(solver.calculateEstimate(),newest-1),.1));
    NonlinearFactorGraph g;g.emplace_shared<fgo::factor::GPWNOAPrior>(X(newest-1),V(newest-1),W(newest-1),X(newest),V(newest),W(newest),.1,f::Qc(),true,true);solver.update(g,v);}
  WnoaEndpoint endpoint(int64_t stamp){const auto ns=stamp-Origin;const size_t i=ns/f::Period;return ns%f::Period?WnoaEndpoint::Between(i,i+1,.1,(ns%f::Period)*1e-9,f::Qc(),stamp).withCache(nullptr):WnoaEndpoint::Exact(i);}
  void belief(const f::Belief& b){auto old=slots.find(b.id());if(old!=slots.end()&&b.revision<=old->second.first.revision)return;
    NonlinearFactorGraph g;g.emplace_shared<WnoaRelativeFactor>(endpoint(b.start),endpoint(b.end),b.mean,noiseModel::Gaussian::Covariance(b.covariance));
    const auto r=solver.update(g,{},old==slots.end()?FactorIndices{}:FactorIndices{old->second.second},boost::none,boost::none,boost::none,true);slots[b.id()]={b,r.newFactorsIndices.at(0)};}
  void refine(){} // Native policy: no nonlinear convergence loop.
};
void NoRetirement() {
  f::Receiver r(testRoot/"no_retire");Boot(r);Control c;double maxState=0,maxCov=0;
  for(size_t i=1;i<=12;++i){int64_t now=i*f::Period;c.support();r.imuClock(Origin+now,Origin+now);c.refine();
    maxState=std::max(maxState,Difference(r.state().solver.estimate(),c.solver.calculateEstimate()));
    if(i>=2) {
      std::vector<f::Belief> beliefs{Bel("lidarA",now-2*f::Period,now,now),
        Bel("cameraB",now-2*f::Period+23000000,now-f::Period+71000000,now)};
      if(i==4){auto same=Bel("cameraC",now-f::Period+23000000,now-f::Period+71000000,now);beliefs.push_back(same);}
      for(auto b:beliefs){r.belief(b);c.belief(b);b.revision=1;b.covariance*=1.3;r.belief(b);c.belief(b);}
    }
  }
  c.refine();r.end(Origin+12*f::Period);
  maxState=std::max(maxState,Difference(r.state().solver.estimate(),c.solver.calculateEstimate()));
  auto expected=Cov(c.solver.getFactorsUnsafe(),c.solver.calculateEstimate());
  maxCov=(Cov(r.state().solver.getFactors(),r.state().solver.estimate())-expected).cwiseAbs().maxCoeff();
  Metric("no_retirement_state_max",maxState);Metric("no_retirement_joint_covariance_max",maxCov);
  Check(maxState<1e-8&&maxCov<1e-8+1e-6*expected.cwiseAbs().maxCoeff(),"no-retirement equals full-history model and update order");
  Check(r.state().counts.replaced>0&&r.state().counts.finalized==13,"full cov revisions and EOF finalization");
  Throws([&]{r.end(Origin+13*f::Period);},"already closed");
}
void PendingAndValidation() {
  f::Receiver r(testRoot/"pending");Boot(r);
  auto a=Bel("cam",23000000,71000000,80000000);
  r.belief(a);auto changedPending=a;--changedPending.start;
  Throws([&]{r.belief(changedPending);},"endpoints changed");
  a.revision=2;r.belief(a);a.revision=1;r.belief(a);a.revision=2;r.belief(a);
  Check(r.state().pending.size()==4&&r.state().active.empty(),"all pending versions retained, none applied before right bracket");
  r.imuClock(Origin+f::Period,Origin+f::Period);
  Check(r.state().counts.inserted==1&&r.state().counts.replaced==1&&r.state().counts.stale==1&&r.state().counts.duplicate==1,"ordered pending dispatch without coalescing");
  const auto& e=r.state().active.at(a.id());
  Check(e.owner->keys().size()==6,"same bracket endpoints share variables");
  const auto* gaussian=dynamic_cast<const noiseModel::Gaussian*>(dynamic_cast<const NoiseModelFactor*>(e.owner.get())->noiseModel().get());
  Check((gaussian->covariance()-a.covariance).cwiseAbs().maxCoeff()<1e-12,"dense wire covariance unchanged");
  a.arrival=Origin+f::Period;
  auto before=r.state();const auto files=Files(r.archive().root());
  auto bad=a;bad.mean=bad.mean.retract(Vector6::Constant(.01));Throws([&]{r.belief(bad);},"conflicting duplicate");Unchanged(r,before,files);
  bad=a;++bad.end;Throws([&]{r.belief(bad);},"endpoints changed");Unchanged(r,before,files);
  bad=a;bad.start=Origin-1;Throws([&]{r.belief(bad);},"endpoints changed");Unchanged(r,before,files);
  bad=a;bad.covariance(0,0)=-1;Throws([&]{r.belief(bad);},"nonpositive covariance");
  bad=a;bad.covariance(0,1)+=1;Throws([&]{r.belief(bad);},"asymmetric covariance");
  bad=a;bad.arrival=bad.end-1;Throws([&]{r.belief(bad);},"physical/arrival");
  bad=a;bad.source="../escape";Throws([&]{r.belief(bad);},"invalid source/session");
  auto replaced=a;replaced.revision=3;replaced.covariance*=1.4;
  Throws([&]{r.belief(replaced,f::Failure::AfterReplacement);},"injected_");Unchanged(r,before,files);
  r.belief(replaced);Check(r.state().active.at(a.id()).latest.revision==3,"replacement retry succeeds after exact rollback");
  auto reset=a;reset.session="two";reset.revision=7;r.belief(reset); // existing independent clock accepts nonzero first revision.
  auto other=a;other.source="lidar_extra";r.belief(other);
  Check(r.state().active.size()==3,"source/session namespace not fixed sensor count");
  auto pendingEnd=Bel("cam",123000000,171000000,180000000);r.belief(pendingEnd);r.end(Origin+190000000);
  Check(r.state().counts.unsupportedEnd==1&&r.state().pending.empty(),"EOF future bracket explicitly excluded, not fabricated");
}
void Reference(const f::Receiver& r) {
  const auto& solver=r.state().solver;
  Check(solver.windowRebuilds==0&&solver.convertedMarginals==0,"no custom chart or reload");
  Check(!solver.isam().getFixedVariables().empty(),"stock fixed linearization variables retained");
  Check(!solver.marginals.empty(),"raw joint marginals captured");
  for(const auto& [slot,m]:solver.marginals) {
    Check(dynamic_cast<const LinearContainerFactor*>(solver.getFactors()[slot].get())!=nullptr,"raw linear marginal used by optimizer");
    for(auto key:m.factor->keys())Check(m.anchor.at(key).equals_(solver.getLinearizationPoint().at(key),1e-14),"prior original anchor remains fixed");
  }
}
void ArchiveCoverage(const f::Receiver& r,size_t n) {
  std::set<size_t> online,finalized;size_t duplicated=0;
  for(const auto& [path,text]:Files(r.archive().root()))if(path.ends_with("records.txt")) {
    std::istringstream lines(text);std::string line;
    while(std::getline(lines,line)){auto pos=line.find(',');auto kind=line.substr(0,pos);
      if(kind!="online_incremental"&&kind!="finalized_incremental")continue;
      const size_t index=std::stoull(line.substr(pos+1));auto& target=kind=="online_incremental"?online:finalized;duplicated+=!target.insert(index).second;}
  }
  Check(online.size()==n&&finalized.size()==n&&duplicated==0,"one immutable online row and one finalized row per physical support");
}
void Turnovers() {
  f::Receiver r(testRoot/"turnover");Boot(r);size_t recycled=0;std::map<size_t,f::Id> previousSlots;
  for(size_t i=1;i<=144;++i){const int64_t t=i*f::Period;
    if(i==72){
      const auto before=r.state();const auto files=Files(r.archive().root());
      for(auto fail:{f::Failure::AfterRetirement,f::Failure::AfterReload,f::Failure::BeforeOutput,f::Failure::OutputStaged}){
        Throws([&]{r.imuClock(Origin+t,Origin+t,fail);},"injected_");Unchanged(r,before,files);}
    }
    r.imuClock(Origin+t,Origin+t);
    Check(r.state().solver.timestamps().size()<=213,"receiver supports bounded after publication");
    if(i==70)Check(r.state().oldest==0,"exact 7s cutoff retains boundary");
    if(i==71) {
      Check(r.state().oldest==1&&r.state().counts.absorbed>0,"actual source-factor absorption");
      auto late=Bel("lidar",0,2*f::Period,t,1);const auto before=r.state();const auto files=Files(r.archive().root());
      Throws([&]{r.belief(late);},"after absorption");Unchanged(r,before,files);
      late.start=Origin+69*f::Period;late.end=Origin+70*f::Period;Throws([&]{r.belief(late);},"after absorption");
      auto unknown=Bel("new_source",0,2*f::Period,t);Throws([&]{r.belief(unknown);},"retired support");
    }
    // Include a source stall; supports are independent and keep advancing.
    if(i>=2&&!(i>=100&&i<105)) {
      auto g=Bel("lidar",t-2*f::Period,t,t);
      if(i==20){const auto before=r.state();const auto files=Files(r.archive().root());Throws([&]{r.belief(g,f::Failure::AfterReplacement);},"injected_");Unchanged(r,before,files);}
      r.belief(g);
      auto k=Bel("camera",t-2*f::Period+23000000,t-f::Period+71000000,t);r.belief(k);
      if(i>=5&&i<100)r.belief(Bel("lidar",t-5*f::Period,t-3*f::Period,t,1));
    }
    for(const auto& [id,e]:r.state().active){if(previousSlots.count(e.slot)&&previousSlots.at(e.slot)!=id)++recycled;previousSlots[e.slot]=id;}
    if(i==80||i==144){r.state().solver.estimate(); /* query has no mutation */}
  }
  r.end(Origin+145*f::Period);Reference(r);ArchiveCoverage(r,145);
  const auto& s=r.state();Metric("absorbed_ids",s.counts.absorbed);Metric("active_ids",s.active.size());
  Metric("peak_active_ids",s.counts.peakActive);Metric("peak_slots",s.counts.peakSlots);
  Metric("peak_transient_supports",s.counts.peakSupports);Metric("retirements",s.solver.retirements);
  Metric("max_gradient",s.maxGradient);Metric("max_publication_steps",s.maxPublicationSteps);
  Check(s.counts.peakSupports==72&&s.solver.retirements==74&&s.oldest==74,"two full lag durations and exact retirement count");
  Check(s.active.size()<=142&&s.counts.peakActive<=145&&s.counts.peakSlots<=220&&recycled>0,"bounded active ID/slot storage and actual slot reuse");
  Check(s.maxPublicationSteps<=1&&s.solver.windowRebuilds==0,"bounded stock retirement; no convergence claim");
  Check(s.counts.finalized==145&&s.counts.inserted==s.active.size()+s.counts.absorbed,"finalized and source ID conservation");
}
void StartupAndPendingFailure() {
  f::Receiver r(testRoot/"startup");auto early=Bel("lidar",-10000000,10000000,20000000);r.belief(early);Boot(r,Origin+20000000);
  Check(r.state().counts.excluded==1&&r.state().buffered.empty(),"startup exclusion not conflated with retirement");
  Throws([&]{r.imuClock(Origin,Origin+20000000);},"non-monotonic IMU");
  r.end(Origin+20000000);
  f::Receiver q(testRoot/"pending_failure");Boot(q);
  for(size_t i=1;i<=70;++i)q.imuClock(Origin+i*f::Period,Origin+i*f::Period);
  auto span=Bel("stalled_source",0,8200000000LL,9000000000LL);q.belief(span);
  const auto before=q.state();const auto files=Files(q.archive().root());
  Throws([&]{q.imuClock(Origin+71*f::Period,Origin+9000000000LL);},"pending old bracket");Unchanged(q,before,files);
  // No end marker => caller leaves run incomplete, no silently fabricated final trajectory.
  Check(!q.state().closed&&q.state().counts.finalized==0,"truncated stream not finalized");
  Throws([&]{f::Receiver duplicate(testRoot/"startup");},"new directory");
  Throws([&]{f::Receiver invalid(testRoot/"invalid_lag",std::numeric_limits<double>::quiet_NaN());},"whole support ticks");
}
void WireFixture() {
  std::ofstream out(testRoot/"wire_events.csv");out<<std::setprecision(17);
  auto imu=[&](int64_t t){out<<"I,"<<t<<','<<t<<",100,200,300,4,5,6\n";};
  auto belief=[&](const f::Belief& b){out<<"B,"<<b.arrival<<','<<b.source<<','<<b.session<<','<<b.ordinal<<','<<b.revision<<','<<b.from<<','<<b.to<<','<<b.start<<','<<b.end;
    for(double x:Pose3::Logmap(b.mean))out<<','<<x;for(int i=0;i<6;++i)for(int j=0;j<6;++j)out<<','<<b.covariance(i,j);out<<'\n';};
  for(int i=0;i<59;++i)imu(Origin-(58-i)*2500000);
  for(int i=1;i<=80;++i){auto t=i*f::Period;imu(Origin+t);
    if(i>=2){belief(Bel("g",t-2*f::Period,t,t));belief(Bel("k",t-2*f::Period+23000000,t-f::Period+71000000,t));}
    if(i>=5)belief(Bel("g",t-5*f::Period,t-3*f::Period,t,1));}
  out<<"E,"<<Origin+81*f::Period<<'\n';out.close();Check(bool(out),"synthetic original-schema wire fixture saved");
}

void StockReference() {
  fl3i_solver::NativeSmoother candidate(1.);
  fgo::solvers::IncrementalFixedLagSmoother stock(10.,fl3i_solver::Parameters());
  double maxState=0,maxCov=0;size_t captured=0;
  stock.setNotMarginalizing();
  for(size_t i=0;i<35;++i) {
    NonlinearFactorGraph graph;Values values;
    const Key key=X(i);
    values.insert(key,Vector3(.11*i,-.02*i,.003*i*i));
    if(!i)graph.emplace_shared<PriorFactor<Vector3>>(key,Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.1));
    else graph.emplace_shared<BetweenFactor<Vector3>>(X(i-1),key,Vector3(.1,-.02,.003*(2*i-1)),noiseModel::Isotropic::Sigma(3,.03));
    candidate.add(graph,values,{{key,double(i)}});
    stock.update(graph,values,{{key,double(i)}});
    if(i>10) {
      auto outgoing=candidate.retire();++captured;
      Check(outgoing.exists(X(i-11)),"capture includes exact outgoing support");
      stock.setMarginalizing();stock.update();stock.setNotMarginalizing();
    }
    auto x=candidate.estimate(),y=stock.calculateEstimate();
    Check(x.keys()==y.keys(),"stock retained keys");
    for(auto k:x.keys()) {
      maxState=std::max(maxState,(x.at<Vector3>(k)-y.at<Vector3>(k)).norm());
      maxCov=std::max(maxCov,(candidate.marginalCovariance(k)-stock.marginalCovariance(k)).cwiseAbs().maxCoeff());
    }
    Check(candidate.getFactors().size()<=stock.getFactors().size(),"candidate does not grow beyond stock slots");
    const auto expected=stock.getFactors().linearize(stock.getLinearizationPoint())->augmentedHessian(Ordering(y.keys()));
    const auto actual=candidate.getFactors().linearize(candidate.getLinearizationPoint())->augmentedHessian(Ordering(x.keys()));
    Check((actual-expected).cwiseAbs().maxCoeff()<1e-10,"stock whole linear objective including prior");
    if(i>=2) {
      NonlinearFactorGraph revision;
      revision.emplace_shared<BetweenFactor<Vector3>>(X(i-2),X(i-1),Vector3(.1001,-.0199,.003*(2*(i-1)-1)),noiseModel::Isotropic::Sigma(3,.04));
      auto ci=candidate.add(revision,{},{});auto si=stock.update(revision);
      (void)si;const auto siIndices=stock.getISAM2Result().newFactorsIndices;
      candidate.add(revision,{}, {},ci);stock.update(revision,{}, {},siIndices);
      Check((candidate.estimate().at<Vector3>(key)-stock.calculateEstimate().at<Vector3>(key)).norm()<1e-10,"stock replacement equivalence");
    }
  }
  Metric("stock_state_max",maxState);Metric("stock_covariance_max",maxCov);
  Check(maxState<1e-10&&maxCov<1e-10,"unmodified library stock state/full objective/covariance equivalence");
  Check(captured==24&&candidate.retirements==24,"stock retirement count");
}

#include "SlotTests.h"
int main(){try{
  const char* root=std::getenv("FL3I_TEST_ROOT");f::Require(root,"missing test root");testRoot=root;
  f::Require(fs::create_directory(testRoot),"test root already exists");
  StockReference();SlotTurnover(true,true);SlotTurnover(false,true);SlotTurnover(true,false);
  NonlinearStockReference();NoRetirement();PendingAndValidation();Turnovers();StartupAndPendingFailure();WireFixture();
  std::cout<<"CHECKS "<<checks<<"\nFL3I_LIFECYCLE_GATE PASS\n";return 0;
}catch(const std::exception& e){std::cout<<"FAIL "<<e.what()<<'\n';return 1;}}
