#define main cpk9_unused_main
#include "/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019/test.cpp"
#undef main
#include <sstream>

void offRefresh(bool enabled) {
  mode(true);
  setenv("CPV38_OFF_REFRESH", enabled ? "1" : "0", 1);
}

// Independent dense inverse, retaining cross-state blocks (not just diagonals).
double covarianceCheck(cbs::BPSAM& b, Key a, Key z) {
  auto values = b.calculateEstimate();
  const Ordering order(values.keys());
  const Matrix H = b.getFactorsUnsafe().clone().linearize(values)->hessian(order).first;
  Eigen::LLT<Matrix> llt(H);
  require(llt.info() == Eigen::Success, "LOCAL information not SPD");
  const Matrix inverse = llt.solve(Matrix::Identity(H.rows(), H.cols()));
  std::map<Key, size_t> offsets;
  size_t cursor = 0;
  for (Key key : order) { offsets[key] = cursor; cursor += values.at(key).dim(); }
  Matrix expected(12, 12);
  expected << inverse.block(offsets[a], offsets[a], 6, 6),
              inverse.block(offsets[a], offsets[z], 6, 6),
              inverse.block(offsets[z], offsets[a], 6, 6),
              inverse.block(offsets[z], offsets[z], 6, 6);
  b.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
  const Matrix info = b.jointMarginalInformation({a,z}).fullMatrix();
  const Matrix actual = info.llt().solve(Matrix::Identity(12,12));
  require(actual.allFinite() && Eigen::LLT<Matrix>(actual).info() == Eigen::Success,
          "LOCAL covariance not finite SPD");
  const double error = rel(actual, expected);
  require(error <= 1e-6, "LOCAL dense inverse mismatch");
  require(expected.block(0,6,6,6).norm() > 1e-8, "fixture lacks cross covariance");
  return error;
}

void incrementalReference() {
  offRefresh(true);
  Fixture f; cbs::BPSAM b(params());
  NonlinearFactorGraph graph; Values initial;
  for (int i=0; i<3; ++i) { graph.push_back(f.factors(i)); initial.insert(f.initial(i)); }
  cbs::BPSAM::UpdateParams update; update.constrainedKeys = groups(0,2);
  b.update(graph,initial,update); b.marginalizeLeaves(retired(0));
  const auto theta = b.getLinearizationPoint();
  const auto original = b.getFactorsUnsafe();
  auto badGraph=original;
  auto badFixed=b.getFixedVariables(); badFixed.insert(X(999));
  rejects([&]{cpk9::promote(badGraph,theta,b.fl4nLinearCache(),badFixed);});
  require(badFixed.count(X(999)),"unknown lock silently released");
  for(size_t i=0;i<original.size();++i)require(badGraph[i]==original[i],"partial invalid promotion");
  auto badCache=b.fl4nLinearCache(); badFixed=b.getFixedVariables();
  for(size_t i=0;i<original.size();++i)
    if(dynamic_cast<LinearContainerFactor*>(original[i].get())){badCache[i].reset();break;}
  rejects([&]{cpk9::promote(badGraph,theta,badCache,badFixed);});
  require(badFixed==b.getFixedVariables(),"bad-cache rejection changed locks");
  const auto before = b.calculateBestEstimate();
  auto objective = fl4q::anchored(original,theta,true);
  const auto expected = before.retract(objective.linearize(before)->optimize());
  const auto refreshed = b.cpv38Refresh(groups(1,2));
  require(refreshed.variablesRelinearized == before.size(), "not all keys refreshed");
  const double thetaError = before.localCoordinates(b.getLinearizationPoint()).norm();
  const double stateError = expected.localCoordinates(b.calculateEstimate()).norm() /
                           (1 + before.localCoordinates(expected).norm());
  require(thetaError <= 1e-9, "theta was not previous current estimate");
  require(stateError <= 1e-7, "incremental versus independent GN mismatch");
  require(b.getFactorsUnsafe().size() == original.size(), "slots changed");
  Values displaced = before; Vector6 shift;
  shift << .1,-.07,.04,.2,-.1,.05;
  displaced.update(X(1),before.at<Pose3>(X(1)).retract(shift));
  double priorError = 0;
  std::vector<std::pair<NonlinearFactor::shared_ptr,double>> savedPriors;
  for (size_t i=0;i<original.size();++i) {
    if (auto p=dynamic_cast<LinearContainerFactor*>(original[i].get())) {
      const double energy=cbs::fixedLagFactorErrorInChart(*p,displaced,theta);
      auto q=b.getFactorsUnsafe()[i];
      priorError=std::max(priorError,std::abs(q->error(displaced)-energy)/(1+std::abs(energy)));
      savedPriors.emplace_back(q,energy);
    } else require(original[i] == b.getFactorsUnsafe()[i], "native pointer changed");
  }
  require(!savedPriors.empty() && priorError <= 1e-10, "original prior function changed");
  const double eq = equations(b);
  require(eq <= 1e-7, "refreshed equations inconsistent");
  const double covariance = covarianceCheck(b,X(1),X(2));
  const Ordering eliminated(KeyVector{X(1),V(1),B(1)});
  const Ordering retained(KeyVector{X(2),V(2),B(2)});
  const auto dense=b.fl4nLinearCache().eliminatePartialSequential(
      eliminated,EliminateCholesky).second->hessian(retained);
  b.marginalizeLeaves(retired(1));
  const auto actual=b.fl4nLinearCache().hessian(retained);
  const double marginal=std::max(rel(actual.first,dense.first),rel(actual.second,dense.second));
  require(marginal <= 1e-7, "next marginal differs from dense Schur");
  require(b.cpk9PromotePriorCharts().priors > 0, "next prior not promoted");
  for (const auto& p:savedPriors)
    require(std::abs(p.first->error(displaced)-p.second) <= 1e-10*(1+std::abs(p.second)),
            "absorbed original prior object mutated");
  metric("current_theta_error",thetaError);
  metric("independent_native_GN_state_error",stateError);
  metric("original_prior_function_error",priorError);
  metric("refresh_cache_tree_fresh_error",eq);
  metric("LOCAL_joint_covariance_error",covariance);
  metric("next_marginal_Schur_error",marginal);
}

std::string smartState(const NonlinearFactorGraph& graph) {
  std::ostringstream out; out << std::setprecision(17);
  for (const auto& p:graph) if (auto s=std::dynamic_pointer_cast<Smart>(p)) s->cpk1Dump(out);
  return out.str();
}
struct Snapshot {
  Values state,theta; VectorValues delta; NonlinearFactorGraph graph;
  FixedLagSmoother::KeyTimestampMap times; KeySet fixed;
  Matrix tree,cov; std::string smart;
  Snapshot(Smoother& s, Key key) {
    s.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
    state=s.calculateEstimate(); theta=s.bpsam().getLinearizationPoint();
    delta=s.bpsam().getDelta(); graph=s.getFactors(); times=s.timestamps();
    fixed=s.bpsam().getFixedVariables(); tree=GaussianFactorGraph(s.bpsam()).augmentedHessian();
    cov=s.bpsam().marginalCovariance(key); smart=smartState(graph);
  }
  void check(Smoother& s, Key key) const {
    require(state.equals(s.calculateEstimate(),0) && theta.equals(s.bpsam().getLinearizationPoint(),0),"rollback state/theta");
    require(delta.equals(s.bpsam().getDelta(),0),"rollback delta");
    require(times==s.timestamps() && fixed==s.bpsam().getFixedVariables(),"rollback times/locks");
    require(graph.size()==s.getFactors().size(),"rollback slots");
    for(size_t i=0;i<graph.size();++i)require(graph[i]==s.getFactors()[i],"rollback pointer");
    require((tree-GaussianFactorGraph(s.bpsam()).augmentedHessian()).norm()==0,"rollback tree");
    require((cov-s.bpsam().marginalCovariance(key)).norm()==0,"rollback LOCAL covariance");
    require(smart==smartState(s.getFactors()),"rollback smart triangulation cache");
  }
};

// Controlled exception in the second linearization: normal update succeeds,
// extra refresh fails. No fault hooks enter the candidate implementation.
class FailOnRefresh : public PriorFactor<Pose3> {
  mutable size_t calls_=0;
 public:
  FailOnRefresh(Key k,const Pose3& p):PriorFactor<Pose3>(k,p,noiseModel::Isotropic::Sigma(6,1)){}
  GaussianFactor::shared_ptr linearize(const Values& x) const override {
    if (++calls_ == 2) throw std::runtime_error("injected refresh failure");
    return PriorFactor<Pose3>::linearize(x);
  }
  size_t calls() const {return calls_;}
};

Output lifecycle(bool enabled, bool failure) {
  offRefresh(enabled); Fixture f; Smoother s(3.5,params()); Output result;
  double covError=0; size_t retiredCount=0;
  for(int k=0;k<13;++k) {
    auto graph=f.factors(k); auto initial=f.initial(k);
    FixedLagSmoother::KeyTimestampMap times{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}};
    if(failure && k==6) {
      Snapshot before(s,X(k-1));
      auto fail=std::make_shared<FailOnRefresh>(X(k),f.states[k].pose());
      auto failing=graph; failing.push_back(fail);
      const auto newSmart=smartState(graph);
      rejects([&]{s.update(failing,initial,times);});
      require(fail->calls()==2,"fault not in extra refresh");
      before.check(s,X(k-1)); require(newSmart==smartState(graph),"new smart cache not restored");
      rejects([&]{fl4q::transaction(s,graph,[&]{s.update(graph,initial,times);
        throw std::runtime_error("injected after marginalization");return 0;});});
      before.check(s,X(k-1)); require(newSmart==smartState(graph),"post-marg new smart cache");
    }
    const auto update=s.update(graph,initial,times);
    require(update.iterations==(enabled?2:1),"iteration count does not report extra step");
    auto state=s.calculateEstimate(); result.latest.insert(X(k),state.at<Pose3>(X(k)));
    for(const auto& p:graph)if(std::dynamic_pointer_cast<Smart>(p))
      require(std::find(s.getFactors().begin(),s.getFactors().end(),p)!=s.getFactors().end(),"native smart pointer replaced");
    if(k>=1)covError=std::max(covError,covarianceCheck(s.bpsam(),X(k-1),X(k)));
    result.cov.push_back(s.bpsam().marginalCovariance(X(k)));
    for(const auto& p:s.getFactors())if(dynamic_cast<fl4l::OriginalChartPrior*>(p.get()))++result.promoted;
    if(k>=4){++retiredCount;require(!state.exists(X(k-4)),"state did not retire");}
    for(const auto& p:s.getFactors())if(p)for(Key key:p->keys())require(state.exists(key),"dangling factor");
  }
  require(retiredCount==9,"retirement count");
  // This pairwise fixture absorbs each promoted boundary prior in the SAME
  // cycle's marginalization. Post-marginalization adapter count is not a
  // promotion counter. Direct-reference and second-linearization fault tests
  // verify refresh execution; the changed trajectory must also discriminate.
  if(!enabled)require(result.promoted==0,"disabled OFF promotion activated");
  metric(enabled?"candidate_lifecycle_LOCAL_cov_error":"disabled_lifecycle_LOCAL_cov_error",covError);
  return result;
}

void guardTests() {
  offRefresh(true); Fixture f; Smoother s(3.5,params());
  for(int k=0;k<3;++k)s.update(f.factors(k),f.initial(k),{{X(k),double(k)},{V(k),double(k)},{B(k),double(k)}});
  // Stage incoming data solely to verify rejection, never solve a CBS-ON graph.
  auto admission=s.addOdometryBeliefsDetailed({f.belief(0,1)});
  require(admission.details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"guard fixture admission");
  Snapshot before(s,X(2));
  rejects([&]{s.update(f.factors(3),f.initial(3),{{X(3),3},{V(3),3},{B(3),3}});});
  before.check(s,X(2));
  setenv("CPV38_OFF_REFRESH","typo",1);rejects([]{cpv38::enabled();});
  offRefresh(true);setenv("CPK9_PRIOR_REFRESH","0",1);rejects([]{cpv38::enabled();});
  offRefresh(true);setenv("FL4Q_ENABLE","1",1);rejects([]{cpv38::enabled();});
  offRefresh(true);setenv("FL4M_RETIREMENT","retain",1);rejects([]{cpv38::enabled();});
  offRefresh(true);auto p=params();p.sam_params_.optimizationParams=ISAM2DoglegParams();
  cbs::BPSAM wrong(p);rejects([&]{wrong.cpv38ValidateOff();});
  offRefresh(false);
}

int cpv38_saved_main() try {
  std::cout<<std::unitbuf;spdlog::set_level(spdlog::level::err);
  for(const char* key:{"FL4M_AUDIT","FL4N_CAPTURE","FL4Q_LOG","CPK1_PROBE","FL4U_PROBE"})unsetenv(key);
  incrementalReference();
  auto off=lifecycle(false,false),candidate=lifecycle(true,false),repeat=lifecycle(true,false),rollback=lifecycle(true,true);
  equal(candidate,repeat);equal(candidate,rollback);
  const double effect=off.latest.localCoordinates(candidate.latest).norm();
  require(effect>1e-10,"candidate fixture not discriminating");
  metric("candidate_state_change_not_accuracy",effect);
  guardTests();
  std::cout<<"PASS CP-V38 OFF incremental refresh; no dataset accuracy claim\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
