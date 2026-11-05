  // Private OFF-only hook. No class layout change and no model/weight changes.
  void cpv38ValidateOff() const {
    cpk9ValidateMode();
    if (!gbp_graph_.empty() || !gbp_values_.empty() ||
        !pending_cbs_odom_beliefs_.empty() ||
        !pending_active_window_cbs_odom_keys_by_receiver_.empty() ||
        !active_cbs_odom_slots_.empty() ||
        !active_window_cbs_odom_factors_.empty() ||
        !accepted_odom_pose_beliefs_by_source_.empty() ||
        !last_applied_temporary_cbs_linear_beliefs_.empty() ||
        !last_applied_temporary_cbs_linear_odom_beliefs_.empty())
      throw std::runtime_error("CPV38 requires OFF with no incoming beliefs");
  }

  ISAM2Result cpv38Refresh(
      const std::optional<FastMap<Key, int>>& constraints = {}) {
    if (!cpv38::enabled())
      throw std::runtime_error("CPV38 refresh without explicit opt-in");
    cpv38ValidateOff();
    cpk9PromotePriorCharts();
    if (!fixedVariables_.empty())
      throw std::runtime_error("CPV38 unaccounted fixed keys");
    ISAM2UpdateParams refresh;
    refresh.constrainedKeys = constraints;
    refresh.force_relinearize = true;
    refresh.forceFullSolve = true;
    const int saved_update_count = update_count_;
    // Whole wrapper transaction owns rollback, including smart mutable caches.
    auto result = BaseSAM::update({}, {}, refresh);
    updateDelta(true);
    update_count_ = saved_update_count;  // Do not shift normal update cadence.
    tmp_marginals_.reset();
    if (result.variablesRelinearized != theta_.size() ||
        !result.newFactorsIndices.empty() || !result.unusedKeys.empty())
      throw std::runtime_error("CPV38 incomplete refresh or topology change");
    return result;
  }
