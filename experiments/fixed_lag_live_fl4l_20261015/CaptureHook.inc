// Included in the private VioBackend translation unit's diagnostic namespace.
void captureOriginalChart438(
    const gtsam::NonlinearFactorGraph& old_graph,
    const gtsam::Values& before, const gtsam::Values& old_theta,
    const cbs::IncrementalFixedLagBpsamSmoother::BadUpdateStageAudit* stage,
    const gtsam::Values& surviving_theta) {
  using namespace gtsam;
  const auto dir=std::filesystem::path(FLAGS_kimera_bad_update_audit_dir)/"update_438";
  if (std::filesystem::exists(dir)) throw std::runtime_error("refuse existing capture");
  std::filesystem::create_directories(dir);
  std::ofstream status(dir/"status.txt");
  status << "started\n" << std::flush;
  if (!stage) throw std::runtime_error("missing pre-marginalization snapshot");
  const auto graph=stage->graph_after_optimization.clone();
  const auto keys_set=graph.keys(); KeyVector keys(keys_set.begin(),keys_set.end());
  const Values initial=fl4l::subset(before,keys);
  const Values production=fl4l::subset(stage->values_after_optimization,keys);
  size_t priors=0,prior_keys=0; double chart_difference=0;
  for(size_t i=0;i<graph.size();++i) {
    const auto* p=dynamic_cast<const LinearContainerFactor*>(graph[i].get());
    if(!p) continue;
    ++priors;
    if(i>=old_graph.size()||!old_graph[i]||!p->equals(*old_graph[i],0))
      throw std::runtime_error("prior not identical to pre-update prior at same slot");
    for(Key k:p->keys()) {
      ++prior_keys;
      if(!old_theta.exists(k)) throw std::runtime_error("prior origin missing");
      if(surviving_theta.exists(k)) chart_difference=std::max(chart_difference,
        old_theta.at(k).localCoordinates_(surviving_theta.at(k)).norm());
    }
  }
  if(!priors||chart_difference>1e-12) throw std::runtime_error("prior chart check failed");
  saveBadUpdateValues(dir/"theta_before.csv",old_theta);
  saveBadUpdateValues(dir/"values_before.csv",initial);
  saveBadUpdateValues(dir/"values_production.csv",production);
  saveBadUpdateVisualObservations(dir/"visual_observations.csv",graph);
  // Explicitly retain original quadratics. A full graph archive is optional:
  // smart-factor serialization is not assumed to reproduce hidden caches.
  for(size_t i=0;i<graph.size();++i) {
    if(const auto* p=dynamic_cast<const LinearContainerFactor*>(graph[i].get())) {
      std::ofstream s(dir/("prior_"+std::to_string(i)+".txt"));
      for(Key k:p->keys()) s<<DefaultKeyFormatter(k)<<' ';
      s<<'\n'<<std::setprecision(17)<<p->factor()->augmentedInformation()<<'\n';
    }
  }
  auto reference_graph=fl4l::anchored(graph,old_theta);
  double exact_before=0,exact_after=0;
  for(const auto& f:graph) if(f) {
    exact_before+=cbs::fixedLagFactorErrorInChart(*f,initial,old_theta);
    exact_after+=cbs::fixedLagFactorErrorInChart(*f,production,old_theta);
  }
  auto check=[](double a,double b) {
    if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>1e-8*(1+std::abs(a)))
      throw std::runtime_error("anchored objective parity failed");
  };
  check(exact_before,reference_graph.error(initial));
  check(exact_after,reference_graph.error(production));
  std::ofstream objective(dir/"objective.txt"); objective<<std::setprecision(17);
  objective<<"prior_count="<<priors<<"\nprior_key_entries="<<prior_keys
           <<"\nmax_surviving_chart_change="<<chart_difference<<'\n';
  auto record=[&](const std::string& label,const Values& values) {
    auto copy=reference_graph.clone(); const auto linear=copy.linearize(values);
    auto grad=linear->gradientAtZero(); auto diag=linear->hessianDiagonal();
    double scaled=0;
    for(const auto& item:grad) for(int j=0;j<item.second.size();++j)
      scaled=std::max(scaled,std::abs(item.second[j])/std::sqrt(std::max(1e-30,diag.at(item.first)[j])));
    objective<<label<<"_cost="<<copy.error(values)<<'\n'
             <<label<<"_scaled_gradient="<<scaled<<'\n'<<std::flush;
    std::ofstream factors(dir/(label+"_factors.txt")); factors<<std::setprecision(17);
    auto fresh=graph.clone();
    for(size_t i=0;i<fresh.size();++i) if(fresh[i]) {
      const auto& f=*fresh[i]; std::string family=badUpdateFactorFamily(f);
      if(dynamic_cast<const gtsam::ImuFactorT<>*>(&f)) family="native_imu";
      if(dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(&f)) family="bias_between";
      factors<<i<<' '<<family<<' '<<cbs::fixedLagFactorErrorInChart(f,values,old_theta);
      if(auto* smart=dynamic_cast<SmartStereoFactor*>(fresh[i].get())) {
        const auto point=smart->point(values);
        factors<<" visual_valid="<<point.valid();
      }
      factors<<'\n';
    }
  };
  record("before",initial); record("production",production);
  // Same measured factors and same immutable joint prior, different optimizer
  // only in this private cloned reference. No reference values reach Kimera.
  LevenbergMarquardtParams params;
  params.maxIterations=30; params.lambdaInitial=1e-3;
  params.setDiagonalDamping(true); params.relativeErrorTol=1e-9;
  params.absoluteErrorTol=1e-9;
  auto solve_graph=reference_graph.clone();
  const auto start=std::chrono::steady_clock::now();
  LevenbergMarquardtOptimizer lm(solve_graph,initial,params);
  const Values reference=lm.optimize();
  objective<<"reference_iterations="<<lm.iterations()<<"\nreference_wall_sec="
           <<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<'\n';
  saveBadUpdateValues(dir/"values_reference.csv",reference);
  record("reference",reference);
  // Log the exact cost along the actual production correction, not a new run.
  const auto production_delta=initial.localCoordinates(production);
  for(double alpha:{0.001,0.01,0.1,0.25,0.5,1.0}) {
    auto d=production_delta; for(auto& kv:d) kv.second*=alpha;
    auto copy=reference_graph.clone();
    objective<<"production_step_fraction_"<<alpha<<"_cost="<<copy.error(initial.retract(d))<<'\n';
  }
  objective.close(); status<<"complete\n";
  LOG(INFO)<<"FL4L_CAPTURE_COMPLETE update=438 priors="<<priors;
}
