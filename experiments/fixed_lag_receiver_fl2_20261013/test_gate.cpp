#include "StagedSmoother.h"
#include "SourceWnoaModel.h"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <iostream>
#include <iomanip>
#include <functional>

using namespace gtsam;using namespace fl2;using namespace fgo::offline;
static int checks=0;
void Check(bool ok,const std::string& text) {
  ++checks; if(!ok)throw std::runtime_error(text);
  std::cout<<"PASS "<<text<<'\n';
}
void Metric(const std::string& s,double x){std::cout<<"METRIC "<<s<<' '<<std::setprecision(17)<<x<<'\n';}
Matrix DenseCov(const NonlinearFactorGraph& g,const Values& v) {
  const auto ab=g.linearize(v)->jacobian(Ordering(v.keys()));
  const Matrix H=ab.first.transpose()*ab.first;
  return H.ldlt().solve(Matrix::Identity(H.rows(),H.cols()));
}
double StateDifference(const Values&a,const Values&b) {
  double d=0;for(auto k:a.keys())d=std::max(d,a.at(k).localCoordinates_(b.at(k)).cwiseAbs().maxCoeff());return d;
}
bool RawEqual(const Values&a,const Values&b) {
  if(a.keys()!=b.keys())return false;
  for(auto k:a.keys()) {
    if(Symbol(k).chr()=='x') {
      if(!(a.at<Pose3>(k).matrix().array()==b.at<Pose3>(k).matrix().array()).all())return false;
    } else if(!(a.at<Vector3>(k).array()==b.at<Vector3>(k).array()).all())return false;
  }
  return true;
}
void LinearGate() {
  StagedSmoother s; NonlinearFactorGraph full; Values zero;
  std::vector<size_t> slots;
  for(size_t i=0;i<12;++i) {
    const Key k=Symbol('l',i);Values v;v.insert(k,Vector3::Zero().eval());zero.insert(k,Vector3::Zero().eval());
    NonlinearFactorGraph g;
    if(i==0)g.emplace_shared<PriorFactor<Vector3>>(k,Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.5));
    else g.emplace_shared<BetweenFactor<Vector3>>(Symbol('l',i-1),k,Vector3(1,.2,-.1),noiseModel::Isotropic::Sigma(3,.2));
    full.push_back(g);slots.push_back(s.add(g,v,{{k,double(i*10)}}).at(0));
    if(i>=2) {
      // Change both mean and dense covariance of newest active edge.
      Matrix3 C;C<<.04,.003,-.002,.003,.09,.001,-.002,.001,.06;
      auto f=boost::make_shared<BetweenFactor<Vector3>>(Symbol('l',i-1),k,Vector3(1.01,.19,-.12),noiseModel::Gaussian::Covariance(C));
      NonlinearFactorGraph revision;revision.push_back(f);
      slots.back()=s.add(revision,{}, {},{slots.back()}).at(0);full[i]=f;
    }
    s.retire();
    if(i==7)Check(s.timestamps().size()==8,"cutoff equality retained");
    if(i<8)continue;
    const auto ab=full.linearize(zero)->jacobian(Ordering(zero.keys()));
    const Vector mean=ab.first.colPivHouseholderQr().solve(ab.second);
    const Matrix cov=DenseCov(full,zero);const auto current=s.estimate();
    const auto keys=current.keys(); const Matrix actual=DenseCov(s.auditGraph(),current);
    Matrix expected(actual.rows(),actual.cols());double err=0;
    for(size_t a=0;a<keys.size();++a) {
      const size_t ia=Symbol(keys[a]).index();err=std::max(err,(current.at<Vector3>(keys[a])-mean.segment<3>(3*ia)).cwiseAbs().maxCoeff());
      for(size_t b=0;b<keys.size();++b)
        expected.block<3,3>(3*a,3*b)=cov.block<3,3>(3*ia,3*Symbol(keys[b]).index());
      Check((s.marginalCovariance(keys[a])-cov.block<3,3>(3*ia,3*ia)).cwiseAbs().maxCoeff()<1e-9,"solver covariance block vs independent full graph");
    }
    Metric("linear_mean_max",err);Metric("linear_full_covariance_max",(actual-expected).cwiseAbs().maxCoeff());
    Check(err<1e-9,"linear retained means vs dense QR");
    Check((actual-expected).cwiseAbs().maxCoeff()<1e-9+1e-8*expected.cwiseAbs().maxCoeff(),"linear full joint covariance vs independent elimination");
    Check(current.size()==8,"linear bounded state count through repeated retirements");
    Check(s.refine().passed,"linear chart-correct stationary");
  }
  // Existing unanchored wrapper's generic error can be zero although prior is not.
  size_t unanchored=0;double correct=0,generic=0;
  const auto x=s.estimate();
  for(const auto& kv:s.marginals) {
    auto* p=dynamic_cast<const LinearContainerFactor*>(kv.second.factor.get());
    unanchored+=!p->hasLinearizationPoint();generic+=p->error(x);
    correct+=ChartMarginal(p->factor(),kv.second.anchor).error(x);
  }
  Metric("runtime_unanchored_marginals",unanchored);Metric("generic_marginal_error",generic);Metric("correct_marginal_error",correct);
  Check(!s.marginals.empty(),"actual runtime marginalization occurred");
  const auto& m=s.marginals.begin()->second;
  auto* wrapped=dynamic_cast<const LinearContainerFactor*>(m.factor.get());
  auto displaced=Perturb(x,m.factor->keys().front(),0,.5);
  Metric("displaced_generic_prior_error",wrapped->error(displaced));
  Metric("displaced_chart_prior_error",ChartMarginal(wrapped->factor(),m.anchor).error(displaced));
  StagedSmoother before(s);bool caught=false;
  try{s.transaction([&]{s.add({}, {},{{Symbol('l',100),1000.}});s.publicationCursor=42;throw std::runtime_error("injected");});}
  catch(...){caught=true;}
  Check(caught&&s.timestamps()==before.timestamps()&&StateDifference(s.estimate(),before.estimate())==0&&
    s.publicationCursor==before.publicationCursor&&s.disabled()==before.disabled()&&
    s.getFactors().size()==before.getFactors().size(),"transaction restores solver timestamps flags and output cursor");
  for(size_t i=0;i<s.getFactors().size();++i)Check(s.getFactors()[i]==before.getFactors()[i],"rollback exact factor slots");
  // Inject failure AFTER the existing smoother has actually marginalized again.
  caught=false;
  try {s.transaction([&]{
    NonlinearFactorGraph g;g.emplace_shared<BetweenFactor<Vector3>>(Symbol('l',11),Symbol('l',12),Vector3(1,0,0),noiseModel::Isotropic::Sigma(3,.2));
    Values v;v.insert(Symbol('l',12),Vector3::Zero().eval());s.add(g,v,{{Symbol('l',12),120.}});s.retire();
    throw std::runtime_error("after_marginalization");
  });} catch(...){caught=true;}
  Check(caught&&RawEqual(s.estimate(),before.estimate())&&s.timestamps()==before.timestamps()&&
    s.isam().getFixedVariables()==before.isam().getFixedVariables(),"rollback after actual retirement restores states timestamps and anchors");
  for(size_t i=0;i<s.getFactors().size();++i)if(s.getFactors()[i]!=before.getFactors()[i])throw std::runtime_error("retirement rollback slot mismatch");
  Check(s.getFactors().size()==before.getFactors().size(),"rollback after retirement preserves slot ownership");
}

SharedNoiseModel Qc(){Vector6 q;q<<.00375,.00375,.00375,.015,.015,.015;return noiseModel::Diagonal::Variances(q);}
SourceWnoaState Truth(size_t i) {
  const double t=.1*i;
  return {Pose3(Rot3::Expmap(Vector3(.12*t,-.04*t,.23*t)),Point3(.65*t,.02*t*t,.015*t*t)),
          Vector3(.65,.04*t,.03*t),Vector3(.12,-.04,.23)};
}
NonlinearFactor::shared_ptr Belief(size_t a,size_t b,const Pose3& z) {
  Matrix6 L=Matrix6::Identity();L(4,0)=.13;L(5,2)=-.21;L(3,4)=.17;
  Matrix6 C=1e-4*L*L.transpose();
  return boost::make_shared<WnoaRelativeFactor>(WnoaEndpoint::Exact(a),WnoaEndpoint::Exact(b),z,noiseModel::Gaussian::Covariance(C));
}
void AddState(StagedSmoother& s,size_t i,NonlinearFactorGraph& full,Values& initial,std::vector<size_t>& beliefSlots) {
  Values values;InsertSourceWnoaState(values,i,Truth(i));initial.insert(values);
  NonlinearFactorGraph g;
  if(i==0) {
    g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noiseModel::Isotropic::Sigma(6,1e-5));
    g.emplace_shared<PriorFactor<Vector3>>(V(0),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,10.));
    g.emplace_shared<PriorFactor<Vector3>>(W(0),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,3.));
  } else {
    g.emplace_shared<fgo::factor::GPWNOAPrior>(X(i-1),V(i-1),W(i-1),X(i),V(i),W(i),.1,Qc(),true,true);
    if(i>=2)g.push_back(Belief(i-2,i,Truth(i-2).pose.between(Truth(i).pose)));
  }
  auto inserted=s.add(g,values,{{X(i),double(i)},{V(i),double(i)},{W(i),double(i)}});
  if(i>=2)beliefSlots.push_back(inserted.back());full.push_back(g);
}
void ChartCheck(const StagedSmoother& s) {
  auto graph=s.auditGraph();auto x=s.estimate();
  // Deliberately displace boundary from its anchor in all SE(3) coordinates.
  const auto& marginal=s.marginals.begin()->second;
  Key pose=0;for(auto k:marginal.factor->keys())if(Symbol(k).chr()=='x'){pose=k;break;}
  Check(pose!=0,"marginal contains a pose boundary");
  x.update(pose,x.at<Pose3>(pose).retract((Vector6()<<.17,-.09,.12,.21,-.13,.08).finished()));
  auto* wrapper=dynamic_cast<const LinearContainerFactor*>(marginal.factor.get());
  ChartMarginal audit(wrapper->factor(),marginal.anchor);
  const auto d=marginal.anchor.localCoordinates(Subset(x,marginal.factor->keys()));
  const double original=wrapper->factor()->error(d);
  Check(std::abs(audit.error(x)-original)<1e-9+1e-10*std::abs(original),"chart marginal cost matches original joint Gaussian");
  const auto gradient=audit.linearize(x)->gradientAtZero();double maxratio=0;
  for(double h:{1e-5,1e-6})for(auto k:audit.keys())for(size_t j=0;j<x.at(k).dim();++j) {
    const double numerical=(audit.error(Perturb(x,k,j,h))-audit.error(Perturb(x,k,j,-h)))/(2*h);
    const double analytic=gradient.at(k)[j];
    maxratio=std::max(maxratio,std::abs(numerical-analytic)/(2e-5+2e-5*std::max(std::abs(numerical),std::abs(analytic))));
  }
  Metric("chart_derivative_tolerance_ratio",maxratio);Check(maxratio<=1,"Pose3 chart gradient independently differentiated at large displacement");
}
bool NonlinearGate() {
  StagedSmoother s;NonlinearFactorGraph full;Values initial;std::vector<size_t> slots;
  // Same 100 ms grid, build a 7 s synthetic section before its first retirement.
  for(size_t i=0;i<=70;++i)AddState(s,i,full,initial,slots);
  auto pre=s.refine();Metric("pre_retirement_gradient",pre.after);
  Check(pre.passed,"nonlinear pre-retirement publication");
  s.retire();Check(s.marginals.empty()&&s.estimate().size()==213,"7 s exact cutoff retains initial X/V/W");
  // Identical standalone full-history iSAM2 follows the same updates/refinement.
  StagedSmoother longlag(1000.);NonlinearFactorGraph other;Values guess;std::vector<size_t> otherSlots;
  for(size_t i=0;i<=70;++i)AddState(longlag,i,other,guess,otherSlots);
  Check(longlag.refine().passed,"long-lag reference stationary");
  Metric("no_retirement_state_max",StateDifference(s.estimate(),longlag.estimate()));
  Check(StateDifference(s.estimate(),longlag.estimate())<1e-8,"no-retirement regression");
  AddState(s,71,full,initial,slots);Check(s.refine().passed,"pre-first-retirement refined");
  s.retire();Check(!s.estimate().exists(X(0))&&!s.estimate().exists(V(0))&&!s.estimate().exists(W(0)),"retire X/V/W together after 7.1 s");
  Check(s.estimate().size()==213,"nonlinear retained window bounded");ChartCheck(s);
  Metric("post_retirement_gradient",Gradient(s.auditGraph(),s.estimate()));
  // Mean revision retaining full dense covariance of an explicit boundary edge.
  // Independent linear gate above also changes the full covariance on revision.
  Vector6 offset;offset<<.025,-.017,.021,.032,-.019,.014;
  auto revised=Belief(1,3,Truth(1).pose.between(Truth(3).pose).retract(offset));
  NonlinearFactorGraph replacement;replacement.push_back(revised);
  const size_t edge13slot=slots.at(1);
  Check(s.getFactors().at(edge13slot)!=nullptr,"boundary edge remains explicit before revision");
  // Same revision with marginalization disabled: isolate retirement, not data.
  AddState(longlag,71,other,guess,otherSlots);
  Check(longlag.refine().passed,"no-retirement control extended to identical 7.1 s window");
  longlag.add(replacement,{}, {},{otherSlots.at(1)});
  const auto noMarginalResult=longlag.refine();
  Metric("same_revision_without_marginalization_gradient",noMarginalResult.after);
  Check(noMarginalResult.passed,"same nonlinear revision passes without retirement");
  StagedSmoother before(s);bool publication=false;
  try {s.transaction([&] {
    s.add(replacement,{}, {},{edge13slot});auto result=s.refine();
    Metric("revised_retained_gradient_before",result.before);Metric("revised_retained_gradient_after",result.after);
    Metric("revised_refinement_steps",result.iterations);
    // Independently assembled retained objective with its frozen joint prior.
    auto graph=s.auditGraph();LevenbergMarquardtParams p;p.maxIterations=100;p.relativeErrorTol=1e-12;p.absoluteErrorTol=1e-12;
    auto reference=LevenbergMarquardtOptimizer(graph,s.estimate(),p).optimize();
    Metric("reference_gradient",Gradient(graph,reference));
    Metric("candidate_reference_state_max",StateDifference(s.estimate(),reference));
    Metric("candidate_cost",graph.error(s.estimate()));Metric("reference_cost",graph.error(reference));
    const auto candidate=s.estimate();
    const auto linear=graph.linearize(candidate);const auto gradients=linear->gradientAtZero(),diagonal=linear->hessianDiagonal();
    double fixedGrad=0,freeGrad=0,anchorMove=0;
    for(const auto& kv:gradients) {
      const double norm=(kv.second.array().abs()/diagonal.at(kv.first).array().sqrt()).maxCoeff();
      if(s.isam().getFixedVariables().count(kv.first)) {
        fixedGrad=std::max(fixedGrad,norm);
        anchorMove=std::max(anchorMove,s.getLinearizationPoint().at(kv.first).localCoordinates_(candidate.at(kv.first)).norm());
      } else freeGrad=std::max(freeGrad,norm);
    }
    Metric("boundary_normalized_gradient",fixedGrad);Metric("nonboundary_normalized_gradient",freeGrad);
    Metric("boundary_delta_from_fixed_anchor",anchorMove);
    // Independent objective difference at the largest gradient coordinate.
    Key worstKey=0;Eigen::Index worstCoordinate=0;double largest=-1;
    for(const auto& kv:gradients)for(Eigen::Index j=0;j<kv.second.size();++j){
      const double z=std::abs(kv.second[j])/std::sqrt(diagonal.at(kv.first)[j]);
      if(z>largest){largest=z;worstKey=kv.first;worstCoordinate=j;}}
    for(double h:{1e-5,1e-6}) {
      const double fd=(graph.error(Perturb(candidate,worstKey,worstCoordinate,h))-
                       graph.error(Perturb(candidate,worstKey,worstCoordinate,-h)))/(2*h);
      Metric("boundary_total_objective_fd_normalized",std::abs(fd)/std::sqrt(diagonal.at(worstKey)[worstCoordinate]));
      const double g=gradients.at(worstKey)[worstCoordinate];
      Check(std::abs(fd-g)<2e-5+2e-5*std::max(std::abs(fd),std::abs(g)),"full retained objective gradient matches independent cost differences");
    }
    // Compare the full joint retained covariance of the same objective at these
    // two estimates; NOT full-history covariance after retirement.
    const Matrix c1=DenseCov(graph,candidate),c2=DenseCov(graph,reference);
    Metric("same_prior_covariance_max_difference",(c1-c2).cwiseAbs().maxCoeff());
    if(!result.passed)throw std::runtime_error("strict_retained_publication_failed");
    Check(StateDifference(s.estimate(),reference)<1e-5,"nonlinear same-prior reference state");
    ++s.publicationCursor;publication=true;
  });}catch(const std::exception& e) {
    std::cout<<"BLOCKER "<<e.what()<<'\n';
    Metric("rollback_local_self_roundoff",StateDifference(s.estimate(),before.estimate()));
    Check(RawEqual(s.estimate(),before.estimate())&&s.timestamps()==before.timestamps()&&
       s.publicationCursor==before.publicationCursor,"failed nonlinear publication fully rolls back visible state");
    Check(RawEqual(s.getLinearizationPoint(),before.getLinearizationPoint())&&
      s.isam().getFixedVariables()==before.isam().getFixedVariables()&&s.disabled()==before.disabled(),
      "rollback preserves fixed anchors and flags");
    for(size_t i=0;i<s.getFactors().size();++i)if(s.getFactors()[i]!=before.getFactors()[i])
      throw std::runtime_error("rollback changed factor pointer slots");
    Check(s.getFactors().size()==before.getFactors().size()&&s.marginals.size()==before.marginals.size(),
      "rollback preserves factor ownership and marginal count");
  }
  return publication;
}
int main(){try {
  std::cout<<std::unitbuf;LinearGate();const bool passed=NonlinearGate();
  std::cout<<"CHECKS "<<checks<<"\nGATE "<<(passed?"PASS":"BLOCKED")<<'\n';return passed?0:2;
}catch(const std::exception&e){std::cout<<"FAIL "<<e.what()<<"\nCHECKS "<<checks<<'\n';return 1;}}
