// Private header-only opt-in: no members or ABI changes.
struct Cpv40Bound {
  AgentId source; Key from,to,sender_from,sender_to; double maximum;
};
void cpv40ValidateMode() const {
  cpk9ValidateMode();
  if (!gbp_graph_.empty() || !gbp_values_.empty() ||
      !last_applied_temporary_cbs_linear_beliefs_.empty() ||
      !last_applied_temporary_cbs_linear_odom_beliefs_.empty())
    throw std::runtime_error("CPV40 only supports sender-computed relative CBS");
}
std::vector<Cpv40Bound> cpv40PreBounds(const Values& newTheta) const {
  cpv40ValidateMode();
  std::vector<Cpv40Bound> bounds;
  for (const auto& [key,belief]:pending_cbs_odom_beliefs_) {
    std::optional<Pose3> relative;
    if (valueExists(key.from_pose_key) && valueExists(key.to_pose_key))
      relative=calculateEstimate<Pose3>(key.from_pose_key).between(calculateEstimate<Pose3>(key.to_pose_key));
    else if (newTheta.exists(key.from_pose_key) && newTheta.exists(key.to_pose_key))
      relative=newTheta.at<Pose3>(key.from_pose_key).between(newTheta.at<Pose3>(key.to_pose_key));
    const double bound=relative ? std::max(1.25*Pose3::Logmap(
        belief.measured_from_to.inverse().compose(*relative)).norm()+0.25,0.5)
        : std::numeric_limits<double>::infinity();
    bounds.push_back({key.source_agent,key.from_pose_key,key.to_pose_key,
                      key.sender_from_pose_key,key.sender_to_pose_key,bound});
  }
  return bounds;
}
void cpv40ValidateActive(const std::vector<Cpv40Bound>& staged,
    const std::vector<ActiveWindowCbsOdomFactorRemoval>& removals) const {
  cpv40ValidateMode();
  if (!pending_cbs_odom_beliefs_.empty() || !pending_active_window_cbs_odom_keys_by_receiver_.empty())
    throw std::runtime_error("CPV40 ordinary update did not drain pending CBS");
  if(cpv76::enabled()) {cpv76ValidateSlots(staged);return;}
  if (active_cbs_odom_slots_.size()!=active_window_cbs_odom_factors_.size() ||
      active_window_cbs_odom_keys_by_receiver_.size()!=active_window_cbs_odom_factors_.size())
    throw std::runtime_error("CPV40 inconsistent CBS tracking sizes");
  const Values values=calculateEstimate();
  for (const auto& [key,active]:active_window_cbs_odom_factors_) {
    const auto slot=active_cbs_odom_slots_.find(key);
    const auto receiver=active_window_cbs_odom_keys_by_receiver_.find(key.receiverKey());
    if (slot==active_cbs_odom_slots_.end() || receiver==active_window_cbs_odom_keys_by_receiver_.end() ||
        !(receiver->second==key) || !(active.belief.key==key) || slot->second.factor!=active.factor)
      throw std::runtime_error("CPV40 inconsistent CBS ownership");
    bool scheduled=false;
    for (const auto& r:removals)
      if (r.source_agent==key.source_agent && r.from_pose_key==key.from_pose_key && r.to_pose_key==key.to_pose_key &&
          r.sender_from_pose_key==key.sender_from_pose_key && r.sender_to_pose_key==key.sender_to_pose_key) {
        if (!r.factor_slot_valid || r.factor_index!=slot->second.factor_index)
          throw std::runtime_error("CPV40 mismatched retirement receipt");
        scheduled=true;
      }
    const auto index=slot->second.factor_index;
    if (scheduled && !cpv69::retain()) {
      if (std::find(nonlinearFactors_.begin(),nonlinearFactors_.end(),active.factor)!=nonlinearFactors_.end())
        throw std::runtime_error("CPV40 retired factor still present");
      continue; // Commit of this exact scheduled removal follows the refresh.
    }
    if (index>=nonlinearFactors_.size() || nonlinearFactors_[index]!=active.factor ||
        !active.factor || active.factor->keys()!=KeyVector{key.from_pose_key,key.to_pose_key} ||
        !values.exists(key.from_pose_key) || !values.exists(key.to_pose_key))
      throw std::runtime_error("CPV40 missing/changed active CBS slot or endpoint");
    double bound=2.0; // Existing active-window native residual guard.
    for (const auto& b:staged)
      if (b.source==key.source_agent && b.from==key.from_pose_key && b.to==key.to_pose_key &&
          b.sender_from==key.sender_from_pose_key && b.sender_to==key.sender_to_pose_key) bound=b.maximum;
    const double residual=Pose3::Logmap(active.belief.measured_from_to.inverse().compose(
        values.at<Pose3>(key.from_pose_key).between(values.at<Pose3>(key.to_pose_key)))).norm();
    if (!std::isfinite(residual) || residual>bound)
      throw std::runtime_error("CPV40 post-refresh CBS residual guard");
  }
}
ISAM2Result cpv40Refresh(const std::vector<Cpv40Bound>& staged,
    const std::vector<ActiveWindowCbsOdomFactorRemoval>& removals,
    const std::optional<FastMap<Key,int>>& constraints={}) {
  if (!cpv40::enabled()) throw std::runtime_error("CPV40 refresh without opt-in");
  cpv40ValidateActive(staged,removals);
  const auto persistent_receipt=cpv76::enabled() ? cpv76Plan({}, {}, {}) : Cpv76Plan{};
  cpk9PromotePriorCharts();
  if (!fixedVariables_.empty()) throw std::runtime_error("CPV40 unaccounted fixed keys");
  ISAM2UpdateParams refresh;
  refresh.constrainedKeys=constraints; refresh.force_relinearize=true; refresh.forceFullSolve=true;
  const int saved_update_count=update_count_;
  auto result=BaseSAM::update({}, {}, refresh); // Never consume CBS a second time.
  updateDelta(true); update_count_=saved_update_count; tmp_marginals_.reset();
  if (result.variablesRelinearized!=theta_.size() || !result.newFactorsIndices.empty() || !result.unusedKeys.empty())
    throw std::runtime_error("CPV40 incomplete refresh or topology change");
  cpv40ValidateActive(staged,removals);
  if(cpv76::enabled()) cpv76ValidatePlan(persistent_receipt, true);
  return result;
}
// Read-only fixture receipt: compare before/after rollback, including pending
// data, factor identity, counters and cached ownership. Never used for solving.
std::string cpv40TrackingReceipt() const {
  std::ostringstream out; out<<std::setprecision(17)<<update_count_<<'\n';
  const auto keyOut=[&](const auto& k){out<<int(k.source_agent)<<' '<<k.from_pose_key<<' '<<k.to_pose_key<<' '
    <<k.sender_from_pose_key<<' '<<k.sender_to_pose_key<<' ';};
  const auto beliefOut=[&](const auto& b){keyOut(b.key);out<<b.measured_from_to.matrix()<<'\n'
    <<b.covariance<<'\n'<<b.raw_covariance<<'\n'<<b.relax_factor<<' '<<b.sender_computed<<'\n';};
  for(const auto& [k,b]:pending_cbs_odom_beliefs_){out<<"pending ";beliefOut(b);}
  for(const auto& [r,k]:pending_active_window_cbs_odom_keys_by_receiver_){out<<"pending_receiver ";keyOut(k);out<<'\n';}
  for(const auto& [k,s]:active_cbs_odom_slots_){out<<"slot ";keyOut(k);out<<s.factor_index<<' '<<s.factor.get()<<'\n';}
  for(const auto& [r,k]:active_window_cbs_odom_keys_by_receiver_){out<<"active_receiver ";keyOut(k);out<<'\n';}
  for(const auto& [k,a]:active_window_cbs_odom_factors_){out<<"active ";beliefOut(a.belief);out<<a.factor.get()<<' '<<a.active_update_count<<'\n';}
  return out.str();
}
