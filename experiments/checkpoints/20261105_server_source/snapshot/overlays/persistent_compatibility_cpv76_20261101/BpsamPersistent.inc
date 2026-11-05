// Private header-only integration. No BPSAM data members or ABI changes.
// Existing native Persistent code still performs admission/replacement/solve.
struct Cpv76Expected {
  Pose3 mean; Matrix covariance; NonlinearFactor::shared_ptr pointer;
};
struct Cpv76Plan {
  std::map<CbsOdomKey,Cpv76Expected> factors;
  NonlinearFactorGraph native;
  NonlinearFactorGraph removed;
};
void cpv76ValidateSlots(const std::vector<Cpv40Bound>& bounds, bool pending_ok=false) const {
  cpv40ValidateMode();
  if(!cpv76::enabled() || params_.enable_belief_dcs || params_.health_aware_receiver_params.enable ||
     !active_window_cbs_odom_factors_.empty() || !active_window_cbs_odom_keys_by_receiver_.empty() ||
     !pending_active_window_cbs_odom_keys_by_receiver_.empty() || (!pending_ok&&!pending_cbs_odom_beliefs_.empty()))
    throw std::runtime_error("CPV76 incompatible persistent ownership/weighting");
  std::set<CbsOdomReceiverKey> receivers;
  std::set<std::tuple<AgentId,Key,Key>> senders;
  std::set<FactorIndex> indices;
  const auto values=calculateEstimate();
  for(const auto& [key,slot]:active_cbs_odom_slots_) {
    const auto factor=std::dynamic_pointer_cast<BetweenFactor<Pose3>>(slot.factor);
    if(!factor || slot.factor_index>=nonlinearFactors_.size() || nonlinearFactors_[slot.factor_index]!=factor ||
       factor->keys()!=KeyVector{key.from_pose_key,key.to_pose_key} ||
       !values.exists(key.from_pose_key) || !values.exists(key.to_pose_key) ||
       std::count(nonlinearFactors_.begin(),nonlinearFactors_.end(),slot.factor)!=1 ||
       !receivers.insert(key.receiverKey()).second || !indices.insert(slot.factor_index).second ||
       !senders.emplace(key.source_agent,key.identityFromPoseKey(),key.identityToPoseKey()).second)
      throw std::runtime_error("CPV76 invalid/stacked persistent factor slot");
    auto noise=std::dynamic_pointer_cast<noiseModel::Gaussian>(factor->noiseModel());
    if(!noise || !noise->covariance().allFinite()) throw std::runtime_error("CPV76 non-Gaussian payload");
    double maximum=2.0; // Preserve CP40 residual safeguard, not a tuned weight.
    for(const auto& b:bounds)
      if(b.source==key.source_agent&&b.from==key.from_pose_key&&b.to==key.to_pose_key&&
         b.sender_from==key.sender_from_pose_key&&b.sender_to==key.sender_to_pose_key)maximum=b.maximum;
    const double residual=Pose3::Logmap(factor->measured().inverse().compose(
        values.at<Pose3>(key.from_pose_key).between(values.at<Pose3>(key.to_pose_key)))).norm();
    if(!std::isfinite(residual)||residual>maximum)throw std::runtime_error("CPV76 persistent residual guard");
  }
}
Cpv76Plan cpv76Plan(const NonlinearFactorGraph& added,const Values& initial,const FactorIndices& removed) const {
  cpv76ValidateSlots({},true);
  Cpv76Plan plan;
  for(const auto& [k,s]:active_cbs_odom_slots_) {
    auto f=std::dynamic_pointer_cast<BetweenFactor<Pose3>>(s.factor);
    auto n=std::dynamic_pointer_cast<noiseModel::Gaussian>(f->noiseModel());
    plan.factors.emplace(k,Cpv76Expected{f->measured(),n->covariance(),f});
    if(std::find(removed.begin(),removed.end(),s.factor_index)!=removed.end())
      throw std::runtime_error("CPV76 explicit peer removal outside native replacement");
  }
  for(size_t i=0;i<nonlinearFactors_.size();++i) {
    const auto& f=nonlinearFactors_[i];if(!f)continue;
    bool peer=false;for(const auto& [k,s]:active_cbs_odom_slots_)if(s.factor==f)peer=true;
    if(!peer&&std::find(removed.begin(),removed.end(),i)==removed.end())plan.native.push_back(f);
  }
  for(const auto& f:added)if(f)plan.native.push_back(f);
  for(const auto& [k,b]:pending_cbs_odom_beliefs_) {
    if(!b.sender_computed)throw std::runtime_error("CPV76 requires sender-relative beliefs");
    // Mirror native connectivity gate, including its existing graph.exists call.
    const auto connected=[&](Key key){return variableIndex_.find(key)!=variableIndex_.end()||added.exists(key)||initial.exists(key);};
    if(!connected(k.from_pose_key)||!connected(k.to_pose_key))continue;
    for(auto it=plan.factors.begin();it!=plan.factors.end();) {
      const auto& old=it->first;
      if(old.receiverKey()==k.receiverKey() || (old.source_agent==k.source_agent&&
         old.identityFromPoseKey()==k.identityFromPoseKey()&&old.identityToPoseKey()==k.identityToPoseKey())) {
        if(it->second.pointer)plan.removed.push_back(it->second.pointer);
        it=plan.factors.erase(it);
      }else ++it;
    }
    plan.factors.emplace(k,Cpv76Expected{b.measured_from_to,b.covariance,{}});
  }
  return plan;
}
void cpv76ValidatePlan(const Cpv76Plan& plan,bool promoted=false) const {
  if(plan.factors.size()!=active_cbs_odom_slots_.size())throw std::runtime_error("CPV76 missing/unexpected insertion");
  for(const auto& [k,e]:plan.factors) {
    const auto it=active_cbs_odom_slots_.find(k);
    if(it==active_cbs_odom_slots_.end())throw std::runtime_error("CPV76 wrong inserted identity");
    const auto f=std::dynamic_pointer_cast<BetweenFactor<Pose3>>(it->second.factor);
    const auto n=f?std::dynamic_pointer_cast<noiseModel::Gaussian>(f->noiseModel()):nullptr;
    if(!n||!f->measured().equals(e.mean,1e-12)||cpk9::relative(n->covariance(),e.covariance)>1e-12||
       (e.pointer&&e.pointer!=f))throw std::runtime_error("CPV76 changed persistent payload/identity");
  }
  for(const auto& f:plan.removed)
    if(std::find(nonlinearFactors_.begin(),nonlinearFactors_.end(),f)!=nonlinearFactors_.end())
      throw std::runtime_error("CPV76 replaced factor still in graph");
  for(const auto& f:plan.native) {
    if(promoted&&dynamic_cast<LinearContainerFactor*>(f.get()))continue; // CP-K9 owns this checked conversion.
    if(std::count(nonlinearFactors_.begin(),nonlinearFactors_.end(),f)!=1)
      throw std::runtime_error("CPV76 native factor lost/duplicated, including internal recovery");
  }
}
struct Cpv76MarginalReceipt {FactorIndex slot;NonlinearFactor::shared_ptr factor;bool consumed;};
std::vector<Cpv76MarginalReceipt> cpv76MarginalReceipts(const KeyVector& keys) const {
  cpv76ValidateSlots({});std::vector<Cpv76MarginalReceipt> result;
  for(const auto& [k,s]:active_cbs_odom_slots_) {
    const bool consumed=std::find(keys.begin(),keys.end(),k.from_pose_key)!=keys.end()||
                        std::find(keys.begin(),keys.end(),k.to_pose_key)!=keys.end();
    result.push_back({s.factor_index,s.factor,consumed});
  }
  return result;
}
void cpv76ValidateMarginal(const std::vector<Cpv76MarginalReceipt>& receipt,
    const FactorIndices& deleted,const KeyVector& keys) const {
  const auto values=calculateEstimate();size_t survivors=0;
  for(const auto& r:receipt) {
    const auto count=std::count(deleted.begin(),deleted.end(),r.slot);
    const bool present=std::find(nonlinearFactors_.begin(),nonlinearFactors_.end(),r.factor)!=nonlinearFactors_.end();
    if(r.consumed ? count!=1||present : count!=0||!present)
      throw std::runtime_error("CPV76 marginal slot consumption mismatch");
    if(!r.consumed)++survivors;
  }
  if(survivors!=active_cbs_odom_slots_.size())throw std::runtime_error("CPV76 unbounded/stale tracking");
  for(Key key:keys)if(values.exists(key))throw std::runtime_error("CPV76 retired key remains");
  for(const auto& f:nonlinearFactors_)if(f)for(Key key:f->keys())
    if(!values.exists(key))throw std::runtime_error("CPV76 dangling factor");
  cpv76ValidateSlots({});
}
