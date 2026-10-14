// Frozen fixture/oracle helper reuse; its original main is not executed.
#define main frozen_fl2_main
#include "../fixed_lag_receiver_fl2_20261013/test_gate.cpp"
#undef main
#include "BoundarySmoother.h"
using fl2a::BoundarySmoother;using fl2a::AnchoredMarginal;

struct Fixture {
  fl2::StagedSmoother solver;std::vector<size_t> slots;
  Fixture() {
    NonlinearFactorGraph unused;Values initial;
    for(size_t i=0;i<=70;++i)AddState(solver,i,unused,initial,slots);
    Check(solver.refine().passed,"fixture before cutoff");solver.retire();
    AddState(solver,71,unused,initial,slots);Check(solver.refine().passed,"fixture before first retirement");solver.retire();
  }
};
void JacobianAndConversion(Fixture& f) {
  const auto& graph=f.solver.getFactors();auto displaced=f.solver.estimate();
  size_t tested=0;
  for(const auto& [slot,m]:f.solver.marginals) {
    const auto* raw=dynamic_cast<const LinearContainerFactor*>(m.factor.get());
    AnchoredMarginal analytic(raw->factor(),m.anchor);ChartMarginal numeric(raw->factor(),m.anchor);
    for(auto k:analytic.keys())if(Symbol(k).chr()=='x')
      displaced.update(k,displaced.at<Pose3>(k).retract((Vector6()<<.17,-.09,.12,.21,-.13,.08).finished()));
    const double cost=analytic.error(displaced),expected=raw->factor()->error(analytic.delta(displaced));
    Check(std::abs(cost-expected)<1e-9+1e-10*std::abs(expected),"original Gaussian cost retained");
    auto a=analytic.linearize(displaced)->jacobian(),b=numeric.linearize(displaced)->jacobian();
    Metric("analytic_numeric_J_max",(a.first-b.first).cwiseAbs().maxCoeff());
    Check((a.first-b.first).cwiseAbs().maxCoeff()<2e-6+2e-5*b.first.cwiseAbs().maxCoeff(),"analytic Local derivative vs numeric reference");
    const Matrix ha=a.first.transpose()*a.first,hb=b.first.transpose()*b.first;
    const Vector ga=-a.first.transpose()*a.second,gb=-b.first.transpose()*b.second;
    Check((ha-hb).cwiseAbs().maxCoeff()<1e-7+1e-7*hb.cwiseAbs().maxCoeff(),"full joint marginal Hessian preserved");
    Check((ga-gb).cwiseAbs().maxCoeff()<1e-7+1e-7*gb.cwiseAbs().maxCoeff(),"marginal gradient preserved");
    Check(RawEqual(analytic.anchor(),m.anchor),"immutable original anchors copied exactly");++tested;
  }
  Check(tested>0,"tested runtime marginal not invented prior");
  BoundarySmoother candidate(f.solver);const auto before=candidate.getFactors();const auto times=candidate.timestamps();
  Check(candidate.adoptCharts()>0,"convert actual raw marginal");
  Check(candidate.timestamps()==times,"no timestamp refresh on conversion");
  Check(candidate.params().findUnusedFactorSlots&&candidate.isam().getFixedVariables().empty(),"reuse restored; chart prior owns immutable anchor");
  for(size_t slot=0;slot<before.size();++slot) {
    if(auto* raw=dynamic_cast<const LinearContainerFactor*>(before[slot].get())) {
      const AnchoredMarginal* converted=nullptr;
      for(const auto& factor:candidate.getFactors())if(auto* p=dynamic_cast<const AnchoredMarginal*>(factor.get()))
        if(p->gaussian().equals(*raw->factor(),1e-14))converted=p;
      Check(converted&&RawEqual(converted->anchor(),f.solver.marginals.at(slot).anchor),"retained marginal keeps exact creation anchor");
      Check(converted->gaussian().equals(*raw->factor(),1e-14),"entire joint Gaussian preserved");
    } else if(before[slot])Check(slot<candidate.getFactors().size()&&before[slot]==candidate.getFactors()[slot],"source factor slot preserved");
  }
}
void ReferenceCheck(const BoundarySmoother& s,const std::string& label) {
  const auto value=s.estimate();auto graph=s.referenceGraph();
  Check(Gradient(graph,value)<=1e-5,label+" independent numeric-chart stationarity");
  LevenbergMarquardtParams p;p.maxIterations=100;p.relativeErrorTol=1e-12;p.absoluteErrorTol=1e-12;
  const auto reference=LevenbergMarquardtOptimizer(graph,value,p).optimize();
  Check(Gradient(graph,reference)<=1e-5,label+" reference stationary");
  double position=0,rotation=0,velocity=0,omega=0;
  for(auto k:value.keys()) {
    if(Symbol(k).chr()=='x') {
      position=std::max(position,(value.at<Pose3>(k).translation()-reference.at<Pose3>(k).translation()).norm());
      rotation=std::max(rotation,Rot3::Logmap(value.at<Pose3>(k).rotation().between(reference.at<Pose3>(k).rotation())).norm());
    } else if(Symbol(k).chr()=='v')velocity=std::max(velocity,(value.at<Vector3>(k)-reference.at<Vector3>(k)).norm());
    else omega=std::max(omega,(value.at<Vector3>(k)-reference.at<Vector3>(k)).norm());
  }
  Metric(label+"_position_m",position);Metric(label+"_rotation_rad",rotation);
  Metric(label+"_velocity_mps",velocity);Metric(label+"_omega_radps",omega);
  Check(StateDifference(value,reference)<1e-5,label+" independent same-prior state");
  const auto currentCov=DenseCov(s.getFactors(),value),refCov=DenseCov(graph,reference);
  Metric(label+"_joint_covariance_max",(currentCov-refCov).cwiseAbs().maxCoeff());
  Check((currentCov-refCov).cwiseAbs().maxCoeff()<1e-7+1e-4*refCov.cwiseAbs().maxCoeff(),label+" full joint covariance");
  const auto keys=value.keys();size_t offset=0;
  for(size_t j=0;j<keys.size();++j){
    const auto k=keys[j];const size_t d=value.at(k).dim();
    if(j==0||j==keys.size()/2||j+1==keys.size()) {
      const auto expected=refCov.block(offset,offset,d,d);
      const Matrix solver=s.marginalCovariance(k);
      Metric(label+"_solver_covariance_block_max",(solver-expected).cwiseAbs().maxCoeff());
      Check((solver-expected).cwiseAbs().maxCoeff()<1e-7+1e-4*expected.cwiseAbs().maxCoeff(),label+" actual solver covariance");
    }
    offset+=d;
  }
}
BoundarySmoother RevisedBoundary(Fixture& f) {
  BoundarySmoother s(f.solver);s.adoptCharts();
  Vector6 offset;offset<<.025,-.017,.021,.032,-.019,.014;
  NonlinearFactorGraph revision;revision.push_back(Belief(1,3,Truth(1).pose.between(Truth(3).pose).retract(offset)));
  auto slots=s.add(revision,{}, {},{f.slots.at(1)});Check(slots.size()==1,"one replacement one factor");
  const auto r=s.solve();Metric("boundary_gradient_before",r.before);Metric("boundary_gradient_after",r.after);Metric("boundary_refinement_steps",r.steps);
  Check(r.passed&&r.steps<=20,"previously failed boundary now strictly stationary");
  ReferenceCheck(s,"boundary");return s;
}
void InterpolationChecks(const BoundarySmoother& prior) {
  Values truth;for(size_t i=1;i<=71;++i)InsertSourceWnoaState(truth,i,Truth(i));
  for(int kind=0;kind<3;++kind){BoundarySmoother s(prior);
    auto from=WnoaEndpoint::Between(1,2,.1,.023,Qc());
    auto to=kind==0?WnoaEndpoint::Exact(3):(kind==1?WnoaEndpoint::Between(3,4,.1,.071,Qc()):WnoaEndpoint::Between(1,2,.1,.071,Qc()));
    auto measurement=from.pose(truth).between(to.pose(truth));
    Vector6 d;d<<.002,-.001,.001,.003,-.002,.001;
    NonlinearFactorGraph newFactor;newFactor.emplace_shared<WnoaRelativeFactor>(from,to,measurement.retract(d),noiseModel::Isotropic::Sigma(6,.01));
    auto slot=s.add(newFactor).at(0);Check(s.solve().passed,"interpolated boundary addition stationary");
    Matrix6 L=Matrix6::Identity();L(1,4)=.15;L(5,0)=-.12;L(3,2)=.18;
    NonlinearFactorGraph revised;revised.emplace_shared<WnoaRelativeFactor>(from,to,measurement,noiseModel::Gaussian::Covariance(1e-4*L*L.transpose()));
    s.add(revised,{}, {},{slot});Check(s.solve().passed,"dense covariance interpolated revision stationary");
    ReferenceCheck(s,"interpolated"+std::to_string(kind));
  }
}
void LinearTurnovers() {
  BoundarySmoother s;NonlinearFactorGraph full;Values zero;
  double maxMean=0,maxCov=0;size_t maxSlots=0;
  for(size_t i=0;i<30;++i) {
    const auto k=Symbol('l',i);Values value;value.insert(k,Vector3::Zero().eval());zero.insert(value);
    NonlinearFactorGraph add;
    if(!i)add.emplace_shared<PriorFactor<Vector3>>(k,Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.5));
    else add.emplace_shared<BetweenFactor<Vector3>>(Symbol('l',i-1),k,Vector3(1,.2,-.1),noiseModel::Isotropic::Sigma(3,.2));
    full.push_back(add);auto slot=s.add(add,value,{{k,double(i*10)}}).at(0);
    if(i>0) {
      Matrix3 C;C<<.04,.003,-.002,.003,.09,.001,-.002,.001,.06;
      NonlinearFactorGraph revise;revise.emplace_shared<BetweenFactor<Vector3>>(Symbol('l',i-1),k,Vector3(1.01,.19,-.12),noiseModel::Gaussian::Covariance(C));
      s.add(revise,{}, {},{slot});full[i]=revise[0];
    }
    const auto r=s.publish();Check(r.passed&&r.steps<=20,"linear bounded publication");
    maxSlots=std::max(maxSlots,s.getFactors().size());
    if(i<8)continue;
    const auto ab=full.linearize(zero)->jacobian(Ordering(zero.keys()));
    const Vector mean=ab.first.colPivHouseholderQr().solve(ab.second);const Matrix cov=DenseCov(full,zero);
    const auto v=s.estimate();const auto keys=v.keys();const Matrix observed=DenseCov(s.getFactors(),v);
    Matrix expected(observed.rows(),observed.cols());
    for(size_t a=0;a<keys.size();++a){const size_t ia=Symbol(keys[a]).index();
      maxMean=std::max(maxMean,(v.at<Vector3>(keys[a])-mean.segment<3>(3*ia)).cwiseAbs().maxCoeff());
      for(size_t b=0;b<keys.size();++b)expected.block<3,3>(3*a,3*b)=cov.block<3,3>(3*ia,3*Symbol(keys[b]).index());}
    maxCov=std::max(maxCov,(observed-expected).cwiseAbs().maxCoeff());
    Check(maxMean<1e-9,"repeated linear mean vs independent dense QR");
    Check((observed-expected).cwiseAbs().maxCoeff()<1e-9+1e-8*expected.cwiseAbs().maxCoeff(),"repeated linear full covariance vs dense elimination");
    Check(v.size()==8,"linear window does not grow");
  }
  Metric("linear_turnover_mean_max",maxMean);Metric("linear_turnover_cov_max",maxCov);Metric("linear_slot_high_water",maxSlots);
  Check(maxSlots<=11&&s.params().findUnusedFactorSlots,"slots reused across active-window rebuilds");
}
void CheckRollback(const BoundarySmoother& s,const BoundarySmoother& before) {
  Check(RawEqual(s.estimate(),before.estimate())&&RawEqual(s.getLinearizationPoint(),before.getLinearizationPoint()),"exact solver state rollback");
  Check(s.timestamps()==before.timestamps()&&s.publicationCursor==before.publicationCursor&&
    s.windowRebuilds==before.windowRebuilds&&s.convertedMarginals==before.convertedMarginals&&
    s.retiredBlocks==before.retiredBlocks&&s.disabled()==before.disabled(),"transaction metadata counters cursor rollback");
  Check(s.getFactors().size()==before.getFactors().size(),"rollback graph size");
  for(size_t i=0;i<s.getFactors().size();++i)if(s.getFactors()[i]!=before.getFactors()[i])throw std::runtime_error("rollback slot owner");
  Check(s.isam().getFixedVariables()==before.isam().getFixedVariables(),"rollback linearization flags");
}
Matrix QRCov(const NonlinearFactorGraph& graph,const Values& theta) {
  const auto ab=graph.linearize(theta)->jacobian(Ordering(theta.keys()));
  const Eigen::HouseholderQR<Matrix> qr(ab.first);
  const Matrix R=qr.matrixQR().topRows(ab.first.cols()).triangularView<Eigen::Upper>();
  const Matrix inverse=R.triangularView<Eigen::Upper>().solve(Matrix::Identity(R.rows(),R.cols()));
  return inverse*inverse.transpose();
}
void NonlinearMarginalReference() {
  BoundarySmoother s;NonlinearFactorGraph unused;Values initial;std::vector<size_t> slots;
  for(size_t i=0;i<=70;++i)AddState(s,i,unused,initial,slots);
  Check(s.solve().passed,"nonlinear covariance fixture starts stationary");
  for(size_t i=71;i<=73;++i) {
    AddState(s,i,unused,initial,slots);Check(s.solve().passed,"reference pre-retirement solve");
    const auto theta=s.getLinearizationPoint();
    double deltaMax=0;for(const auto& kv:s.getDelta())deltaMax=std::max(deltaMax,kv.second.cwiseAbs().maxCoeff());
    Check(deltaMax<.01,"reference has no keys eligible for ordinary relinearization");
    const Matrix fullCov=QRCov(s.getFactors(),theta);
    std::map<Key,std::pair<size_t,size_t>> offsets;size_t column=0;
    for(auto k:theta.keys()){offsets[k]={column,theta.at(k).dim()};column+=theta.at(k).dim();}
    // Existing GNSSFGO retirement, then recover its actual joint marginal chart.
    s.fl2::StagedSmoother::retire();
    Check(RawEqual(s.getLinearizationPoint(),Subset(theta,s.getLinearizationPoint().keys())),"surviving reference charts unchanged during elimination");
    // Independent dense QR of the full graph -> select retained covariance.
    const auto kept=s.getLinearizationPoint();const Matrix observed=QRCov(s.auditGraph(),kept);
    Matrix expected(observed.rows(),observed.cols());size_t row=0;
    for(auto a:kept.keys()){size_t col=0;auto [oa,da]=offsets.at(a);
      for(auto b:kept.keys()){auto [ob,db]=offsets.at(b);expected.block(row,col,da,db)=fullCov.block(oa,ob,da,db);col+=db;}row+=da;}
    const double error=(observed-expected).cwiseAbs().maxCoeff();Metric("nonlinear_elimination_covariance_max",error);
    Check(error<1e-7+1e-5*expected.cwiseAbs().maxCoeff(),"nonlinear retirement joint covariance vs independent full QR");
    s.adoptCharts();Check(s.solve().passed,"same objective after conversion still stationary");
  }
}
void StreamingTurnovers() {
  BoundarySmoother s;NonlinearFactorGraph unused;Values initial;std::vector<size_t> slots;
  double maxGradient=0;size_t maxVariables=0,maxSlots=0,maxSteps=0;
  for(size_t i=0;i<=144;++i) {
    if(i==72)for(auto failure:{BoundarySmoother::Failure::AfterRetirement,BoundarySmoother::Failure::AfterRebuild,BoundarySmoother::Failure::BeforeCommit}){
      BoundarySmoother trial(s),before(trial);NonlinearFactorGraph scratch;Values v;std::vector<size_t> ids;
      bool caught=false;try {trial.transaction([&]{AddState(trial,i,scratch,v,ids);trial.publish(failure);});}catch(const std::exception& e){caught=std::string(e.what()).find("injected_")==0;}
      Check(caught,"injected failure reached intended boundary");CheckRollback(trial,before);
    }
    AddState(s,i,unused,initial,slots);
    if(i>=80&&i%10==0){const size_t from=i-69;Vector6 d;d<<.002,-.001,.001,.003,-.002,.001;
      NonlinearFactorGraph rev;rev.push_back(Belief(from,from+2,Truth(from).pose.between(Truth(from+2).pose).retract(d)));
      slots.at(from)=s.add(rev,{}, {},{slots.at(from)}).at(0);
    }
    const auto result=s.publish();Check(result.passed&&result.steps<=20,"streaming publication stationary");
    maxGradient=std::max(maxGradient,result.after);maxVariables=std::max(maxVariables,s.estimate().size());
    maxSlots=std::max(maxSlots,s.getFactors().size());maxSteps=std::max(maxSteps,result.steps);
    if(i>=71)Check(s.estimate().size()==213,"streaming retains exactly71 X/V/W supports");
    if(i==71||i==80||i==100||i==144)ReferenceCheck(s,"turnover"+std::to_string(i));
  }
  Metric("stream_max_gradient",maxGradient);Metric("stream_max_steps",maxSteps);Metric("stream_variables",maxVariables);
  Metric("stream_slot_high_water",maxSlots);Metric("stream_rebuilds",s.windowRebuilds);Metric("stream_retired_blocks",s.retiredBlocks);
  Check(s.windowRebuilds==74&&s.retiredBlocks==222,"two full lag durations and74 actual retirements");
  Check(maxVariables==213&&maxSlots<=150,"retained variables and slots bounded in fixture");
}
int main(){try{
  std::cout<<std::unitbuf;LinearGate();Check(!NonlinearGate(),"frozen FL2 negative control remains blocked");
  Fixture f;JacobianAndConversion(f);auto revised=RevisedBoundary(f);InterpolationChecks(revised);
  LinearTurnovers();NonlinearMarginalReference();StreamingTurnovers();
  std::cout<<"CHECKS "<<checks<<"\nFL2A_BOUNDARY_GATE PASS\n";return 0;
}catch(const std::exception& e){std::cout<<"FAIL "<<e.what()<<"\nCHECKS "<<checks<<'\n';return 1;}}
