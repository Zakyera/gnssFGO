  auto fl4qMarginalCacheRestoreAction() {
    const auto saved=tmp_marginals_;
    return [this,saved] { tmp_marginals_=saved; };
  }
  // Private FL4q candidate. Rebuild only the ISAM2 core, not CBS ownership maps.
  // All linear containers must already have been given immutable original
  // charts; no arbitrary change of an unanchored Gaussian's coordinate system.
  void fl4qRebuild(const NonlinearFactorGraph& graph, const Values& accepted,
                  const std::optional<FastMap<Key,int>>& constraints) {
    if (!std::holds_alternative<ISAM2GaussNewtonParams>(BaseSAM::params_.optimizationParams) ||
        !BaseSAM::params_.cacheLinearizedFactors ||
        params_.use_temporary_cbs_linear_factors ||
        params_.cbs_odom_factor_mode != CbsOdomFactorMode::ActiveWindowTemporary)
      throw std::runtime_error("FL4Q incompatible core mode");
    if (graph.size()!=nonlinearFactors_.size() || accepted.keys()!=theta_.keys())
      throw std::runtime_error("FL4Q changed core keys/slots");
    for (size_t i=0;i<graph.size();++i) {
      if (bool(graph[i])!=bool(nonlinearFactors_[i]) ||
          (graph[i] && graph[i]->keys()!=nonlinearFactors_[i]->keys()))
        throw std::runtime_error("FL4Q changed factor identity/keys");
    }
    const auto saved_params=BaseSAM::params_;
    auto build_params=saved_params;
    build_params.findUnusedFactorSlots=false; // Keep holes and exact slot IDs.
    build_params.evaluateNonlinearError=false;
    BaseSAM rebuilt(build_params);
    ISAM2UpdateParams update;
    update.constrainedKeys=constraints;
    const auto result=rebuilt.update(graph,accepted,update);
    for(size_t i=0;i<graph.size();++i)
      if(result.newFactorsIndices.at(i)!=i)
        throw std::runtime_error("FL4Q rebuilt factor slot changed");
    const int saved_update_count=update_count_;
    BaseSAM::operator=(rebuilt);
    BaseSAM::params_=saved_params;
    update_count_=saved_update_count;
    // Tree and cached Gaussians are undamped at accepted. The accepted
    // nonlinear step is zero in this new chart, not the next raw GN step.
    delta_=theta_.zeroVectors();
    deltaNewton_=delta_;
    RgProd_=delta_;
    deltaReplacedMask_.clear();
    tmp_marginals_.reset();
  }
