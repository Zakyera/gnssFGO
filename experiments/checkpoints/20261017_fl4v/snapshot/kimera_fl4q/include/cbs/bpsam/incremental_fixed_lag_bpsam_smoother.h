#pragma once
#include "RetirementAudit.h"
#include "LinearParityAudit.h"
#include "CurrentChartFallback.h"

#include <cbs/bpsam/bpsam.h>
#include <cbs/bpsam/fixed_lag_objective_audit.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/base/debug.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JointMarginal.h>
#include <gtsam/nonlinear/BayesTreeMarginalizationHelper.h>
#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <Eigen/Eigenvalues>

#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cbs {

class IncrementalFixedLagBpsamSmoother
    : public gtsam::FixedLagSmoother {
 public:
  using Base = gtsam::FixedLagSmoother;
  using shared_ptr = boost::shared_ptr<IncrementalFixedLagBpsamSmoother>;

  struct ShadowAcceptanceFactorErrorSummary {
    size_t count = 0u;
    size_t before_valid = 0u;
    size_t after_valid = 0u;
    double before_error = 0.0;
    double after_error = 0.0;
  };

  struct ShadowAcceptancePoseMovementSummary {
    size_t pose_count = 0u;
    double translation_mean_m = 0.0;
    double translation_max_m = 0.0;
    double rotation_mean_deg = 0.0;
    double rotation_max_deg = 0.0;
  };

  struct ShadowAcceptanceAudit {
    bool valid = false;
    double timestamp = 0.0;
    size_t latest_frame = 0u;
    double objective_before = std::numeric_limits<double>::quiet_NaN();
    double objective_after = std::numeric_limits<double>::quiet_NaN();
    ShadowAcceptancePoseMovementSummary movement;
    size_t variables_relinearized = 0u;
    size_t variables_reeliminated = 0u;
    std::map<std::string, ShadowAcceptanceFactorErrorSummary> factor_errors;
    gtsam::Values candidate_values;
  };

  // One-update, read-only diagnostic. This is not a restart checkpoint: the
  // frontend and the incremental Bayes tree are deliberately not copied.
  struct BadUpdateStageAudit {
    gtsam::NonlinearFactorGraph graph_after_optimization;
    gtsam::Values values_after_optimization;
    gtsam::KeyVector keys_to_marginalize;
    gtsam::FactorIndices created_marginal_factor_slots;
    gtsam::FactorIndices deleted_marginal_factor_slots;
  };

  void captureBadUpdateStagesOnce() {
    capture_bad_update_stages_once_ = true;
    bad_update_stage_audit_.reset();
  }

  const std::optional<BadUpdateStageAudit>& badUpdateStageAudit() const {
    return bad_update_stage_audit_;
  }

  IncrementalFixedLagBpsamSmoother(
      double smoother_lag = 0.0,
      gtsam::ISAM2Params isam2_params = defaultIsam2Params(),
      cbs::AgentId robot_id = static_cast<cbs::AgentId>('k'))
      : Base(smoother_lag),
        isam_(makeBpsamParams(isam2_params, robot_id)) {}

  IncrementalFixedLagBpsamSmoother(double smoother_lag,
                                   const cbs::BPSAM::Params& bpsam_params)
      : Base(smoother_lag), isam_(bpsam_params) {}

  void print(const std::string& s = "IncrementalFixedLagBpsamSmoother:\n",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
  }

  bool equals(const gtsam::FixedLagSmoother& rhs,
              double tol = 1e-9) const override {
    const auto* e =
        dynamic_cast<const IncrementalFixedLagBpsamSmoother*>(&rhs);
    return e != nullptr && Base::equals(*e, tol) &&
           isam_.equals(e->isam_, tol);
  }

  gtsam::FixedLagSmoother::Result update(
      const gtsam::NonlinearFactorGraph& newFactors =
          gtsam::NonlinearFactorGraph(),
      const gtsam::Values& newTheta = gtsam::Values(),
      const gtsam::FixedLagSmoother::KeyTimestampMap& timestamps =
          gtsam::FixedLagSmoother::KeyTimestampMap(),
      const gtsam::FactorIndices& factorsToRemove =
          gtsam::FactorIndices()) override {
    if (fl4q::enabled()) {
      return fl4q::transaction(*this, newFactors, [&] {
        return fl4qUpdateImpl(newFactors,newTheta,timestamps,factorsToRemove);
      });
    }
    return fl4qUpdateImpl(newFactors,newTheta,timestamps,factorsToRemove);
  }

  gtsam::FixedLagSmoother::Result fl4qUpdateImpl(
      const gtsam::NonlinearFactorGraph& newFactors,
      const gtsam::Values& newTheta,
      const gtsam::FixedLagSmoother::KeyTimestampMap& timestamps,
      const gtsam::FactorIndices& factorsToRemove) {
    const bool debug = ISDEBUG("IncrementalFixedLagBpsamSmoother update");

    const bool fl4m_audit = std::getenv("FL4M_AUDIT") != nullptr || fl4q::enabled();
    gtsam::Values fl4m_before, fl4m_theta;
    if (fl4m_audit) {
      fl4m_before = isam_.calculateEstimate();
      for (const auto& kv : newTheta)
        if (!fl4m_before.exists(kv.key)) fl4m_before.insert(kv.key,kv.value);
      fl4m_theta = isam_.getLinearizationPoint();
    }
    updateKeyTimestampMap(timestamps);

    const double current_timestamp = getCurrentTimestamp();
    gtsam::KeyVector marginalizableKeys =
        findKeysBefore(current_timestamp - smootherLag_);
    const bool marginalization_diagnostics =
        isam_.getParams().active_cbs_odom_factor_diagnostics_enable;
    const bool marginalization_detail_diagnostics =
        isam_.getParams().active_cbs_odom_factor_detail_diagnostics_enable;
    std::optional<gtsam::Values> pre_update_values;
    std::optional<gtsam::VectorValues> pre_update_delta;
    std::vector<MarginalPriorAudit> pre_update_marginal_priors;
    if (marginalization_diagnostics) {
      pre_update_values = isam_.calculateEstimate();
      if (marginalization_detail_diagnostics) {
        pre_update_delta = isam_.getDelta();
        pre_update_marginal_priors = captureMarginalPriorAudits(
            isam_.getFactorsUnsafe(), *pre_update_delta);
      }
    }

    std::optional<gtsam::FastMap<gtsam::Key, int>> constrainedKeys;
    createOrderingConstraints(marginalizableKeys, constrainedKeys);

    std::unordered_set<gtsam::Key> additionalKeys =
        gtsam::BayesTreeMarginalizationHelper<
            cbs::BPSAM>::gatherAdditionalKeysToReEliminate(isam_,
                                                           marginalizableKeys);
    gtsam::KeyList additionalMarkedKeys(additionalKeys.begin(),
                                        additionalKeys.end());

    gtsam::FactorIndices factors_to_remove = factorsToRemove;
    std::optional<gtsam::KeySet> active_window_keys_after_update;
    std::vector<cbs::BPSAM::ActiveWindowCbsOdomFactorRemoval>
        active_window_cbs_removals;
    if (isam_.usesActiveWindowTemporaryCbsOdomFactors()) {
      gtsam::KeySet active_keys_after_update;
      const double oldest_active_timestamp =
          current_timestamp - smootherLag_;
      for (const auto& [key, timestamp] : keyTimestampMap_) {
        if (timestamp >= oldest_active_timestamp) {
          active_keys_after_update.insert(key);
        }
      }
      for (const auto key : marginalizableKeys) {
        active_keys_after_update.erase(key);
      }
      active_window_cbs_removals =
          isam_.activeWindowTemporaryCbsOdomFactorRemovalsOutsideKeys(
              active_keys_after_update);
      const gtsam::FactorIndices cbs_slots_to_remove =
          isam_.activeWindowTemporaryCbsOdomFactorRemovalSlots(
              active_window_cbs_removals);
      if (!cbs::fl4mRetain()) {
        factors_to_remove.insert(factors_to_remove.end(),
                                 cbs_slots_to_remove.begin(),
                                 cbs_slots_to_remove.end());
      } else {
        for (const auto& removal : active_window_cbs_removals) {
          if (!removal.factor_slot_valid ||
              (std::find(marginalizableKeys.begin(), marginalizableKeys.end(), removal.from_pose_key) == marginalizableKeys.end() &&
               std::find(marginalizableKeys.begin(), marginalizableKeys.end(), removal.to_pose_key) == marginalizableKeys.end()))
            throw std::runtime_error("FL4m retained expiry has no marginalizing endpoint");
        }
      }
      active_window_keys_after_update = active_keys_after_update;
      std::sort(factors_to_remove.begin(), factors_to_remove.end());
      factors_to_remove.erase(
          std::unique(factors_to_remove.begin(), factors_to_remove.end()),
          factors_to_remove.end());
      spdlog::info(
          "CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,{},{},{},{},{},{},{}",
          isam_.id(),
          current_timestamp,
          smootherLag_,
          active_keys_after_update.size(),
          marginalizableKeys.size(),
          isam_.activeWindowTemporaryCbsOdomFactorCount(),
          active_window_cbs_removals.size());
    }

    std::vector<gtsam::NonlinearFactor::shared_ptr> fl4n_retired;
    for (const auto& removal : active_window_cbs_removals)
      if (removal.factor_slot_valid)
        fl4n_retired.push_back(isam_.getFactorsUnsafe().at(removal.factor_index));

    std::optional<gtsam::Values> reference_pre_update_values;
    std::optional<gtsam::Values> reference_post_update_values;
    std::optional<gtsam::ISAM2Result> reference_update_result;
    if (marginalization_detail_diagnostics &&
        unmarginalized_reference_isam_) {
      try {
        const auto reference_removals = mapFactorRemovalIndicesByIdentity(
            isam_.getFactorsUnsafe(),
            unmarginalized_reference_isam_->getFactorsUnsafe(),
            factors_to_remove);
        if (!reference_removals) {
          throw std::runtime_error(
              "could not map every factor removal into reference graph");
        }
        reference_pre_update_values =
            unmarginalized_reference_isam_->calculateEstimate();
        reference_update_result = unmarginalized_reference_isam_->update(
            newFactors,
            newTheta,
            *reference_removals,
            constrainedKeys,
            {},
            additionalMarkedKeys);
        reference_post_update_values =
            unmarginalized_reference_isam_->calculateEstimate();
      } catch (const std::exception& e) {
        spdlog::warn(
            "CBS fixed-lag unmarginalized reference update failed: {}",
            e.what());
        unmarginalized_reference_isam_.reset();
      }
    }

    if (shadow_acceptance_force_full_solve_) {
      cbs::BPSAM::UpdateParams update_params;
      update_params.removeFactorIndices = factors_to_remove;
      update_params.constrainedKeys = constrainedKeys;
      update_params.extraReelimKeys = additionalMarkedKeys;
      update_params.force_relinearize = true;
      update_params.forceFullSolve = true;
      isamResult_ = isam_.update(newFactors, newTheta, update_params);
    } else {
      isamResult_ = isam_.update(newFactors,
                                 newTheta,
                                 factors_to_remove,
                                 constrainedKeys,
                                 {},
                                 additionalMarkedKeys);
    }

    if (fl4q::enabled()) fl4q::accept(current_timestamp,isam_,fl4m_before,constrainedKeys);

    if (fl4m_audit)
      cbs::fl4mAudit(current_timestamp, isam_.getFactorsUnsafe(),
          fl4m_before, isam_.calculateEstimate(), fl4m_theta,
          isam_.getLinearizationPoint(), active_window_cbs_removals.size());



    if (shadow_acceptance_audit_out_ &&
        shadow_acceptance_initialized_values_) {
      try {
        const gtsam::NonlinearFactorGraph audit_graph =
            isam_.getFactorsUnsafe().clone();
        *shadow_acceptance_audit_out_ = captureShadowAcceptanceAudit(
            current_timestamp,
            audit_graph,
            *shadow_acceptance_initialized_values_,
            isam_.calculateEstimate(),
            isam_.getLinearizationPoint(),
            isamResult_);
      } catch (const std::exception&) {
        shadow_acceptance_audit_out_->timestamp = current_timestamp;
        shadow_acceptance_audit_out_->valid = false;
      }
    }

    if (capture_bad_update_stages_once_) {
      BadUpdateStageAudit audit;
      audit.graph_after_optimization = isam_.getFactorsUnsafe().clone();
      audit.values_after_optimization = isam_.calculateEstimate();
      bad_update_stage_audit_ = std::move(audit);
    }

    std::optional<gtsam::Values> post_update_values;
    if (marginalization_diagnostics) {
      post_update_values = isam_.calculateEstimate();
      const gtsam::KeySet excluded_keys(marginalizableKeys.begin(),
                                        marginalizableKeys.end());
      logPoseTransition(current_timestamp,
                        "isam_update",
                        *pre_update_values,
                        *post_update_values,
                        excluded_keys);
      if (marginalization_detail_diagnostics) {
        logFixedVariableAudit(
            current_timestamp, "post_update", isam_, isamResult_);
        if (reference_pre_update_values && reference_post_update_values &&
            reference_update_result) {
          const gtsam::KeySet excluded_keys(marginalizableKeys.begin(),
                                            marginalizableKeys.end());
          logPoseTransition(current_timestamp,
                            "unmarginalized_reference_update",
                            *reference_pre_update_values,
                            *reference_post_update_values,
                            excluded_keys);
          logPoseTransition(current_timestamp,
                            "marginalized_vs_unmarginalized",
                            *post_update_values,
                            *reference_post_update_values,
                            excluded_keys);
          logFixedVariableAudit(current_timestamp,
                                "unmarginalized_reference_post_update",
                                *unmarginalized_reference_isam_,
                                *reference_update_result);
        }
        const gtsam::VectorValues post_update_delta = isam_.getDelta();
        logMarginalPriorUpdateAudits(current_timestamp,
                                     pre_update_marginal_priors,
                                     post_update_delta);
        logFactorErrorAudits(current_timestamp,
                             "existing",
                             isam_.getFactorsUnsafe(),
                             *pre_update_values,
                             *post_update_values);
        logCommonGaugeCurvatureAudits(current_timestamp,
                                      "post_solve",
                                      isam_.getFactorsUnsafe(),
                                      *post_update_values);

        gtsam::Values initialized_values(*pre_update_values);
        for (const auto& key_value : newTheta) {
          if (!initialized_values.exists(key_value.key)) {
            initialized_values.insert(key_value.key, key_value.value);
          }
        }
        logFactorErrorAudits(current_timestamp,
                             "new",
                             newFactors,
                             initialized_values,
                             *post_update_values);
      }
    }

    if (active_window_keys_after_update) {
      isam_.commitActiveWindowTemporaryCbsOdomFactorRemovals(
          active_window_cbs_removals);
    }

    if (!isamResult_.unusedKeys.empty()) {
      eraseKeyTimestampMap(gtsam::KeyVector{isamResult_.unusedKeys.begin(),
                                            isamResult_.unusedKeys.end()});

      if (!marginalizableKeys.empty()) {
        marginalizableKeys.erase(
            std::remove_if(marginalizableKeys.begin(),
                           marginalizableKeys.end(),
                           [&](const auto& key) {
                             return isamResult_.unusedKeys.find(key) !=
                                    isamResult_.unusedKeys.end();
                           }),
            marginalizableKeys.end());
      }
    }

    gtsam::FactorIndices marginal_factor_indices;
    gtsam::FactorIndices deleted_factor_indices;
    std::optional<MarginalizationCovarianceAudit> covariance_audit;
    if (marginalization_detail_diagnostics && post_update_values &&
        !marginalizableKeys.empty()) {
      covariance_audit = captureMarginalizationCovarianceAudit(
          marginalizableKeys, *post_update_values);
    }
    if (marginalization_detail_diagnostics) {
      try {
        unmarginalized_reference_isam_.emplace(
            static_cast<const gtsam::ISAM2&>(isam_));
      } catch (const std::exception& e) {
        spdlog::warn(
            "CBS fixed-lag unmarginalized reference capture failed: {}",
            e.what());
        unmarginalized_reference_isam_.reset();
      }
    }
    if (!marginalizableKeys.empty()) {
      if (capture_bad_update_stages_once_ && bad_update_stage_audit_) {
        bad_update_stage_audit_->keys_to_marginalize = marginalizableKeys;
      }
      gtsam::FastList<gtsam::Key> leafKeys(marginalizableKeys.begin(),
                                           marginalizableKeys.end());
      if (marginalization_diagnostics || capture_bad_update_stages_once_) {
        isam_.marginalizeLeaves(
            leafKeys, &marginal_factor_indices, &deleted_factor_indices);
      } else {
        isam_.marginalizeLeaves(leafKeys);
      }
    }

    if (capture_bad_update_stages_once_ && bad_update_stage_audit_) {
      bad_update_stage_audit_->created_marginal_factor_slots =
          marginal_factor_indices;
      bad_update_stage_audit_->deleted_marginal_factor_slots =
          deleted_factor_indices;
    }
    capture_bad_update_stages_once_ = false;

    if (marginalization_diagnostics && post_update_values) {
      const gtsam::Values post_marginalization_values =
          isam_.calculateEstimate();
      logPoseTransition(current_timestamp,
                        "marginalize_leaves",
                        *post_update_values,
                        post_marginalization_values,
                        {});
      logMarginalizationFactors(current_timestamp,
                                marginalizableKeys,
                                marginal_factor_indices,
                                deleted_factor_indices,
                                post_marginalization_values);
      if (covariance_audit) {
        logMarginalizationCovarianceAudit(current_timestamp,
                                          *covariance_audit);
      }
      if (marginalization_detail_diagnostics) {
        logFixedVariableAudit(
            current_timestamp, "post_marginalization", isam_, isamResult_);
      }
    }

    eraseKeyTimestampMap(marginalizableKeys);
    isam_.pruneTrackedCbsFactors();

    gtsam::FixedLagSmoother::Result result;
    result.iterations = 1;
    result.linearVariables = 0;
    result.nonlinearVariables = 0;
    result.error = 0.0;

    if (debug) {
      isam_.printStats();
    }
    return result;
  }

  gtsam::FixedLagSmoother::Result updateWithShadowAcceptanceAudit(
      const gtsam::NonlinearFactorGraph& new_factors,
      const gtsam::Values& new_theta,
      const gtsam::FixedLagSmoother::KeyTimestampMap& timestamps,
      const gtsam::FactorIndices& factors_to_remove,
      bool force_full_solve,
      ShadowAcceptanceAudit* audit_out) {
    if (!audit_out) {
      throw std::invalid_argument(
          "shadow acceptance audit output must not be null");
    }

    shadow_acceptance_force_full_solve_ = force_full_solve;
    shadow_acceptance_audit_out_ = audit_out;
    shadow_acceptance_initialized_values_ =
        initializedShadowAcceptanceValues(isam_.calculateEstimate(),
                                          new_theta);
    try {
      const auto result =
          update(new_factors, new_theta, timestamps, factors_to_remove);
      clearShadowAcceptanceRequest();
      return result;
    } catch (...) {
      clearShadowAcceptanceRequest();
      throw;
    }
  }

  gtsam::Values calculateEstimate() const override {
    return isam_.calculateEstimate();
  }

  template <class VALUE>
  VALUE calculateEstimate(gtsam::Key key) const {
    return isam_.template calculateEstimate<VALUE>(key);
  }

  const gtsam::ISAM2Params& params() const { return isam_.params(); }

  const gtsam::NonlinearFactorGraph& getFactors() const {
    return isam_.getFactorsUnsafe();
  }

  const gtsam::Values& getLinearizationPoint() const {
    return isam_.getLinearizationPoint();
  }

  const gtsam::VectorValues& getDelta() const { return isam_.getDelta(); }

  gtsam::Matrix marginalCovariance(gtsam::Key key) const {
    return isam_.marginalCovariance(key);
  }

  gtsam::JointMarginal jointMarginalInformation(
      const gtsam::KeyVector& keys) const {
    return isam_.jointMarginalInformation(keys);
  }

  const gtsam::ISAM2Result& getISAM2Result() const { return isamResult_; }

  const cbs::BPSAM& getISAM2() const { return isam_; }

  gtsam::NonlinearFactorGraph activeWindowTemporaryCbsOdomFactors() const {
    return isam_.activeWindowTemporaryCbsOdomFactors();
  }

  gtsam::NonlinearFactorGraph trackedCbsOdomFactors() const {
    return isam_.trackedCbsOdomFactors();
  }

  int addBeliefs(
      std::map<gtsam::Key, std::vector<std::pair<cbs::AgentId, gbp::Gaussian>>>
          beliefs) {
    return static_cast<int>(addBeliefsDetailed(std::move(beliefs)).rejected());
  }

  cbs::BPSAM::AddBeliefsResult addBeliefsDetailed(
      std::map<gtsam::Key, std::vector<std::pair<cbs::AgentId, gbp::Gaussian>>>
          beliefs) {
    cbs::BPSAM::AddBeliefsResult result;
    const auto& params = isam_.getParams();
    for (auto it = beliefs.begin(); it != beliefs.end();) {
      if (!isActiveLocalKey(it->first)) {
        for (const auto& [agent_id, gaussian] : it->second) {
          cbs::BPSAM::AddBeliefDetail detail;
          detail.source_agent = agent_id;
          detail.pose_key = it->first;
          detail.status = cbs::BPSAM::AddBeliefStatus::RejectedInactiveWindow;
          detail.enable_soft_reset = params.gbp_update_params.enable_soft_reset;
          detail.metric_type = params.gbp_update_params.metric_type;
          detail.d_reset = params.gbp_update_params.d_reset;
          detail.contract_alpha = params.gbp_update_params.contract_alpha;
          detail.raw_previous_gate_enabled =
              params.use_raw_previous_belief_gate;
          detail.gbp_update_soft_reset_effective =
              params.gbp_update_params.enable_soft_reset;
          detail.message = "target key outside IFLS active local window";
          if (gaussian.key() == it->first) {
            detail.dxycurr = gaussian.dxycurr();
            detail.dxy = gaussian.dxy();
            detail.contraction_step_size = gaussian.contractionStepSize();
          }
          result.addDetail(detail);
        }
        it = beliefs.erase(it);
      } else {
        ++it;
      }
    }

    result.merge(isam_.addBeliefsDetailed(std::move(beliefs)));
    return result;
  }

  int addBeliefs(
      std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> beliefs) {
    std::map<gtsam::Key, std::vector<std::pair<cbs::AgentId, gbp::Gaussian>>>
        beliefs_vec;
    for (const auto& [key, belief] : beliefs) {
      for (const auto& [agent_id, gauss] : belief) {
        beliefs_vec[key].emplace_back(agent_id, gauss);
      }
    }
    return addBeliefs(std::move(beliefs_vec));
  }

  cbs::BPSAM::AddBeliefsResult addBeliefsDetailed(
      std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> beliefs) {
    std::map<gtsam::Key, std::vector<std::pair<cbs::AgentId, gbp::Gaussian>>>
        beliefs_vec;
    for (const auto& [key, belief] : beliefs) {
      for (const auto& [agent_id, gauss] : belief) {
        beliefs_vec[key].emplace_back(agent_id, gauss);
      }
    }
    return addBeliefsDetailed(std::move(beliefs_vec));
  }

  cbs::BPSAM::AddOdometryBeliefsResult addOdometryBeliefsDetailed(
      std::vector<cbs::BPSAM::CbsOdometryBelief> beliefs) {
    cbs::BPSAM::AddOdometryBeliefsResult result;
    for (auto it = beliefs.begin(); it != beliefs.end();) {
      if (!isActiveLocalKey(it->from_pose_key) ||
          !isActiveLocalKey(it->to_pose_key)) {
        cbs::BPSAM::AddOdometryBeliefDetail detail;
        detail.source_agent = it->source_agent;
        detail.from_pose_key = it->from_pose_key;
        detail.to_pose_key = it->to_pose_key;
        detail.status =
            cbs::BPSAM::AddOdometryBeliefStatus::RejectedInactiveWindow;
        detail.covariance_trace = it->covariance.trace();
        detail.message = "target edge outside IFLS active local window";
        result.addDetail(detail);
        it = beliefs.erase(it);
      } else {
        ++it;
      }
    }

    auto inner_result = isam_.addOdometryBeliefsDetailed(std::move(beliefs));
    for (const auto& detail : inner_result.details) {
      result.addDetail(detail);
    }
    return result;
  }

  int addOdometryBeliefs(std::vector<cbs::BPSAM::CbsOdometryBelief> beliefs) {
    return static_cast<int>(
        addOdometryBeliefsDetailed(std::move(beliefs)).rejected());
  }

  void setMarginalizationGraph(cbs::BPSAM::MarginalizationType type) {
    if (type == cbs::BPSAM::MarginalizationType::FULL) {
      isam_.setMarginalizationGraph(type);
      return;
    }
    isam_.setMarginalizationGraph(type, activeLocalKeys());
  }

  std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> getBeliefs(
      gtsam::KeySet keys,
      bool use_other_robots = false) const {
    return isam_.getBeliefs(std::move(keys), use_other_robots);
  }

  std::map<gtsam::Key, std::map<cbs::AgentId, gbp::Gaussian>> getBeliefs(
      gtsam::KeySet keys,
      cbs::AgentId requesting_robot,
      bool use_other_robots = false) const {
    return isam_.getBeliefs(std::move(keys), requesting_robot, use_other_robots);
  }

  std::vector<cbs::BPSAM::CbsOdometryBelief> getOdometryBeliefs(
      gtsam::KeySet keys,
      cbs::AgentId requesting_robot) const {
    return isam_.getOdometryBeliefs(std::move(keys), requesting_robot);
  }

  size_t marginalizationGraphFactorCount() const {
    return isam_.marginalizationGraphFactorCount();
  }

  bool valueExists(gtsam::Key key) const { return isam_.valueExists(key); }

  cbs::BPSAM& bpsam() { return isam_; }
  const cbs::BPSAM& bpsam() const { return isam_; }

 private:
  static gtsam::Values initializedShadowAcceptanceValues(
      const gtsam::Values& before,
      const gtsam::Values& new_values) {
    gtsam::Values initialized(before);
    for (const auto& key_value : new_values) {
      if (!initialized.exists(key_value.key)) {
        initialized.insert(key_value.key, key_value.value);
      }
    }
    return initialized;
  }

  static ShadowAcceptancePoseMovementSummary
  summarizeShadowAcceptancePoseMovement(const gtsam::Values& before,
                                        const gtsam::Values& after) {
    ShadowAcceptancePoseMovementSummary summary;
    double translation_sum = 0.0;
    double rotation_sum_deg = 0.0;
    constexpr double kRadiansToDegrees =
        57.295779513082320876798154814105;
    for (const gtsam::Key key : after.keys()) {
      if (gtsam::Symbol(key).chr() != 'x' || !before.exists(key)) {
        continue;
      }
      try {
        const gtsam::Pose3 before_pose = before.at<gtsam::Pose3>(key);
        const gtsam::Pose3 after_pose = after.at<gtsam::Pose3>(key);
        const gtsam::Pose3 delta = before_pose.between(after_pose);
        const double translation_m = delta.translation().norm();
        const double rotation_deg =
            gtsam::Rot3::Logmap(delta.rotation()).norm() *
            kRadiansToDegrees;
        ++summary.pose_count;
        translation_sum += translation_m;
        rotation_sum_deg += rotation_deg;
        summary.translation_max_m =
            std::max(summary.translation_max_m, translation_m);
        summary.rotation_max_deg =
            std::max(summary.rotation_max_deg, rotation_deg);
      } catch (const std::exception&) {
      }
    }
    if (summary.pose_count > 0u) {
      summary.translation_mean_m =
          translation_sum / static_cast<double>(summary.pose_count);
      summary.rotation_mean_deg =
          rotation_sum_deg / static_cast<double>(summary.pose_count);
    }
    return summary;
  }

  static ShadowAcceptanceAudit captureShadowAcceptanceAudit(
      const double timestamp,
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& before,
      const gtsam::Values& after,
      const gtsam::Values& owning_theta,
      const gtsam::ISAM2Result& result) {
    ShadowAcceptanceAudit audit;
    audit.timestamp = timestamp;
    audit.candidate_values = after;
    audit.movement =
        summarizeShadowAcceptancePoseMovement(before, after);
    audit.variables_relinearized = result.variablesRelinearized;
    audit.variables_reeliminated = result.variablesReeliminated;

    for (const gtsam::Key key : after.keys()) {
      const gtsam::Symbol symbol(key);
      if (symbol.chr() == 'x') {
        audit.latest_frame = std::max(audit.latest_frame, symbol.index());
      }
    }

    double before_total = 0.0;
    double after_total = 0.0;
    bool all_before_valid = true;
    bool all_after_valid = true;
    for (const auto& factor : graph) {
      if (!factor) {
        continue;
      }
      ShadowAcceptanceFactorErrorSummary& summary =
          audit.factor_errors[factorAuditCategory(factor)];
      ++summary.count;

      const double before_error =
          fixedLagFactorErrorInChartOrNan(*factor, before, owning_theta);
      if (std::isfinite(before_error)) {
        ++summary.before_valid;
        summary.before_error += before_error;
        before_total += before_error;
      } else {
        all_before_valid = false;
      }

      const double after_error =
          fixedLagFactorErrorInChartOrNan(*factor, after, owning_theta);
      if (std::isfinite(after_error)) {
        ++summary.after_valid;
        summary.after_error += after_error;
        after_total += after_error;
      } else {
        all_after_valid = false;
      }
    }

    audit.objective_before =
        all_before_valid ? before_total
                         : std::numeric_limits<double>::quiet_NaN();
    audit.objective_after =
        all_after_valid ? after_total
                        : std::numeric_limits<double>::quiet_NaN();
    audit.valid = all_before_valid && all_after_valid &&
                  std::isfinite(audit.objective_before) &&
                  std::isfinite(audit.objective_after);
    return audit;
  }

  void clearShadowAcceptanceRequest() {
    shadow_acceptance_force_full_solve_ = false;
    shadow_acceptance_audit_out_ = nullptr;
    shadow_acceptance_initialized_values_.reset();
  }

  struct PoseTransitionSummary {
    size_t pose_count = 0u;
    double translation_mean_m = 0.0;
    double translation_max_m = 0.0;
    double rotation_mean_deg = 0.0;
    double rotation_max_deg = 0.0;
    gtsam::Key first_key = 0;
    double first_translation_m = 0.0;
    gtsam::Key last_key = 0;
    double last_translation_m = 0.0;
    double common_world_translation_x_m = 0.0;
    double common_world_translation_y_m = 0.0;
    double common_world_translation_z_m = 0.0;
    double common_world_rotation_x_deg = 0.0;
    double common_world_rotation_y_deg = 0.0;
    double common_world_rotation_z_deg = 0.0;
    double rigid_residual_translation_mean_m = 0.0;
    double rigid_residual_translation_max_m = 0.0;
    double rigid_residual_rotation_mean_deg = 0.0;
    double rigid_residual_rotation_max_deg = 0.0;
  };

  struct MarginalPriorAudit {
    gtsam::FactorIndex factor_index = 0u;
    gtsam::GaussianFactor::shared_ptr linear_factor;
    size_t key_count = 0u;
    size_t dimension = 0u;
    size_t rank = 0u;
    double min_eigenvalue = std::numeric_limits<double>::quiet_NaN();
    double min_positive_eigenvalue =
        std::numeric_limits<double>::quiet_NaN();
    double max_eigenvalue = std::numeric_limits<double>::quiet_NaN();
    double condition_estimate = std::numeric_limits<double>::quiet_NaN();
    double pre_error = std::numeric_limits<double>::quiet_NaN();
  };

  struct FactorErrorSummary {
    size_t count = 0u;
    size_t pre_valid = 0u;
    size_t post_valid = 0u;
    double pre_error = 0.0;
    double post_error = 0.0;
  };

  struct CommonGaugeMode {
    std::string name;
    double finite_difference_step = 0.0;
    gtsam::VectorValues tangent;
    double tangent_squared_norm = 0.0;
  };

  struct CommonGaugeModeErrors {
    double plus_error = 0.0;
    double minus_error = 0.0;
  };

  struct CommonGaugeCategorySummary {
    size_t factor_count = 0u;
    double base_error = 0.0;
    std::map<std::string, CommonGaugeModeErrors> mode_errors;
  };

  struct MarginalizationCovarianceAudit {
    gtsam::Key first_pose_key = 0;
    gtsam::Key last_pose_key = 0;
    gtsam::Matrix first_covariance;
    gtsam::Matrix last_covariance;
    gtsam::Matrix joint_covariance;
  };

  static std::optional<gtsam::FactorIndices>
  mapFactorRemovalIndicesByIdentity(
      const gtsam::NonlinearFactorGraph& source_graph,
      const gtsam::NonlinearFactorGraph& target_graph,
      const gtsam::FactorIndices& source_indices) {
    gtsam::FactorIndices target_indices;
    target_indices.reserve(source_indices.size());
    for (const gtsam::FactorIndex source_index : source_indices) {
      if (source_index >= source_graph.size() ||
          !source_graph.exists(source_index)) {
        return std::nullopt;
      }
      const auto& source_factor = source_graph.at(source_index);
      bool found = false;
      for (gtsam::FactorIndex target_index = 0u;
           target_index < target_graph.size();
           ++target_index) {
        if (target_graph.exists(target_index) &&
            target_graph.at(target_index).get() == source_factor.get()) {
          target_indices.push_back(target_index);
          found = true;
          break;
        }
      }
      if (!found) {
        return std::nullopt;
      }
    }
    return target_indices;
  }

  void logFixedVariableAudit(
      const double current_timestamp,
      const char* stage,
      const gtsam::ISAM2& isam,
      const gtsam::ISAM2Result& result) const {
    size_t pose_count = 0u;
    size_t velocity_count = 0u;
    size_t bias_count = 0u;
    gtsam::Key first_pose_key = 0;
    gtsam::Key last_pose_key = 0;
    bool found_pose = false;
    for (const gtsam::Key key : isam.getFixedVariables()) {
      switch (gtsam::Symbol(key).chr()) {
        case 'x':
          ++pose_count;
          if (!found_pose || key < first_pose_key) {
            first_pose_key = key;
          }
          if (!found_pose || key > last_pose_key) {
            last_pose_key = key;
          }
          found_pose = true;
          break;
        case 'v':
          ++velocity_count;
          break;
        case 'b':
          ++bias_count;
          break;
        default:
          break;
      }
    }
    spdlog::info(
        "CBS_FIXED_LAG_FIXED_VARIABLE_ROW,{},{},{},{},{},{},{},{},{},{},{},{}",
        isam_.id(),
        current_timestamp,
        stage,
        isam.getLinearizationPoint().size(),
        isam.getFixedVariables().size(),
        pose_count,
        velocity_count,
        bias_count,
        found_pose ? MultiRobotKeyFormatter(first_pose_key) : "-",
        found_pose ? MultiRobotKeyFormatter(last_pose_key) : "-",
        result.variablesRelinearized,
        result.variablesReeliminated);
  }

  static double relativeMatrixDifference(const gtsam::Matrix& lhs,
                                         const gtsam::Matrix& rhs) {
    if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols() ||
        !lhs.allFinite() || !rhs.allFinite()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return (lhs - rhs).norm() / std::max(1e-15, lhs.norm());
  }

  std::optional<MarginalizationCovarianceAudit>
  captureMarginalizationCovarianceAudit(
      const gtsam::KeyVector& marginalized_keys,
      const gtsam::Values& values) const {
    const gtsam::KeySet excluded(marginalized_keys.begin(),
                                 marginalized_keys.end());
    gtsam::Key first_pose_key = 0;
    gtsam::Key last_pose_key = 0;
    bool found_pose = false;
    for (const gtsam::Key key : values.keys()) {
      if (excluded.exists(key) || gtsam::Symbol(key).chr() != 'x') {
        continue;
      }
      if (!found_pose || key < first_pose_key) {
        first_pose_key = key;
      }
      if (!found_pose || key > last_pose_key) {
        last_pose_key = key;
      }
      found_pose = true;
    }
    if (!found_pose) {
      return std::nullopt;
    }

    try {
      MarginalizationCovarianceAudit audit;
      audit.first_pose_key = first_pose_key;
      audit.last_pose_key = last_pose_key;
      audit.first_covariance = isam_.marginalCovariance(first_pose_key);
      audit.last_covariance = isam_.marginalCovariance(last_pose_key);
      if (first_pose_key == last_pose_key) {
        audit.joint_covariance = audit.first_covariance;
      } else {
        audit.joint_covariance =
            isam_
                .jointMarginalCovariance(
                    gtsam::KeyVector{first_pose_key, last_pose_key})
                .fullMatrix();
      }
      return audit;
    } catch (const std::exception& e) {
      spdlog::warn("CBS fixed-lag covariance audit capture failed: {}",
                   e.what());
      return std::nullopt;
    }
  }

  void logMarginalizationCovarianceAudit(
      const double current_timestamp,
      const MarginalizationCovarianceAudit& before) const {
    try {
      const gtsam::Matrix first_after =
          isam_.marginalCovariance(before.first_pose_key);
      const gtsam::Matrix last_after =
          isam_.marginalCovariance(before.last_pose_key);
      const gtsam::Matrix joint_after =
          before.first_pose_key == before.last_pose_key
              ? first_after
              : isam_
                    .jointMarginalCovariance(gtsam::KeyVector{
                        before.first_pose_key, before.last_pose_key})
                    .fullMatrix();
      spdlog::info(
          "CBS_FIXED_LAG_COVARIANCE_PRESERVATION_ROW,{},{},{},{},{},{},{},"
          "{},{},{},{}",
          isam_.id(),
          current_timestamp,
          MultiRobotKeyFormatter(before.first_pose_key),
          MultiRobotKeyFormatter(before.last_pose_key),
          before.first_covariance.trace(),
          first_after.trace(),
          relativeMatrixDifference(before.first_covariance, first_after),
          before.last_covariance.trace(),
          last_after.trace(),
          relativeMatrixDifference(before.last_covariance, last_after),
          relativeMatrixDifference(before.joint_covariance, joint_after));
    } catch (const std::exception& e) {
      spdlog::warn("CBS fixed-lag covariance audit comparison failed: {}",
                   e.what());
    }
  }

  static std::string factorAuditCategory(
      const gtsam::NonlinearFactor::shared_ptr& factor) {
    if (!factor) {
      return "null";
    }
    if (dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get())) {
      return "marginal_prior";
    }

    const std::string type_name = typeid(*factor).name();
    if (type_name.find("SmartStereoProjection") != std::string::npos) {
      return "smart_stereo";
    }
    if (type_name.find("ImuFactor") != std::string::npos) {
      return "imu";
    }
    if (type_name.find("PriorFactor") != std::string::npos) {
      return "prior";
    }
    if (type_name.find("BetweenFactor") != std::string::npos) {
      bool has_pose = false;
      bool has_bias = false;
      for (const gtsam::Key key : factor->keys()) {
        const char symbol = gtsam::Symbol(key).chr();
        has_pose = has_pose || symbol == 'x';
        has_bias = has_bias || symbol == 'b';
      }
      if (has_bias) {
        return "bias_between";
      }
      if (has_pose) {
        return "pose_between";
      }
      return "between_other";
    }
    return "other";
  }

  static double factorErrorOrNan(
      const gtsam::NonlinearFactor::shared_ptr& factor,
      const gtsam::Values& values) {
    if (!factor) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    try {
      return factor->error(values);
    } catch (const std::exception&) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  static std::vector<MarginalPriorAudit> captureMarginalPriorAudits(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::VectorValues& delta) {
    std::vector<MarginalPriorAudit> audits;
    for (gtsam::FactorIndex factor_index = 0u; factor_index < graph.size();
         ++factor_index) {
      if (!graph.exists(factor_index)) {
        continue;
      }
      const auto& factor = graph.at(factor_index);
      const auto* marginal_prior =
          dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get());
      if (!marginal_prior || !marginal_prior->factor()) {
        continue;
      }

      MarginalPriorAudit audit;
      audit.factor_index = factor_index;
      audit.linear_factor = marginal_prior->factor();
      audit.key_count = factor->keys().size();
      try {
        audit.pre_error = audit.linear_factor->error(delta);
      } catch (const std::exception&) {
      }

      try {
        gtsam::GaussianFactorGraph linear_graph;
        linear_graph.push_back(marginal_prior->factor());
        gtsam::Matrix hessian = linear_graph.hessian().first;
        hessian = 0.5 * (hessian + hessian.transpose());
        audit.dimension = static_cast<size_t>(hessian.rows());
        if (hessian.rows() > 0) {
          Eigen::SelfAdjointEigenSolver<gtsam::Matrix> solver(hessian);
          if (solver.info() == Eigen::Success) {
            const gtsam::Vector eigenvalues = solver.eigenvalues();
            audit.min_eigenvalue = eigenvalues.minCoeff();
            audit.max_eigenvalue = eigenvalues.maxCoeff();
            const double tolerance = std::max(
                1e-12, std::abs(audit.max_eigenvalue) * 1e-10);
            double min_positive = std::numeric_limits<double>::infinity();
            for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
              if (eigenvalues(index) > tolerance) {
                ++audit.rank;
                min_positive = std::min(min_positive, eigenvalues(index));
              }
            }
            if (std::isfinite(min_positive)) {
              audit.min_positive_eigenvalue = min_positive;
              audit.condition_estimate =
                  audit.max_eigenvalue / min_positive;
            }
          }
        }
      } catch (const std::exception&) {
      }
      audits.push_back(std::move(audit));
    }
    return audits;
  }

  void logMarginalPriorUpdateAudits(
      const double current_timestamp,
      const std::vector<MarginalPriorAudit>& audits,
      const gtsam::VectorValues& post_update_delta) const {
    for (const MarginalPriorAudit& audit : audits) {
      double post_error = std::numeric_limits<double>::quiet_NaN();
      try {
        post_error = audit.linear_factor->error(post_update_delta);
      } catch (const std::exception&) {
      }
      spdlog::info(
          "CBS_FIXED_LAG_PRIOR_UPDATE_ROW,{},{},{},{},{},{},{},{},{},{},{},{},{}",
          isam_.id(),
          current_timestamp,
          audit.factor_index,
          audit.key_count,
          audit.dimension,
          audit.rank,
          audit.dimension >= audit.rank ? audit.dimension - audit.rank : 0u,
          audit.min_eigenvalue,
          audit.min_positive_eigenvalue,
          audit.max_eigenvalue,
          audit.condition_estimate,
          audit.pre_error,
          post_error);
    }
  }

  void logFactorErrorAudits(const double current_timestamp,
                            const char* source,
                            const gtsam::NonlinearFactorGraph& graph,
                            const gtsam::Values& pre_values,
                            const gtsam::Values& post_values) const {
    std::map<std::string, FactorErrorSummary> summaries;
    for (const auto& factor : graph) {
      if (!factor) {
        continue;
      }
      FactorErrorSummary& summary = summaries[factorAuditCategory(factor)];
      ++summary.count;
      const double pre_error = factorErrorOrNan(factor, pre_values);
      if (std::isfinite(pre_error)) {
        ++summary.pre_valid;
        summary.pre_error += pre_error;
      }
      const double post_error = factorErrorOrNan(factor, post_values);
      if (std::isfinite(post_error)) {
        ++summary.post_valid;
        summary.post_error += post_error;
      }
    }

    for (const auto& [category, summary] : summaries) {
      spdlog::info(
          "CBS_FIXED_LAG_FACTOR_ERROR_ROW,{},{},{},{},{},{},{},{},{}",
          isam_.id(),
          current_timestamp,
          source,
          category,
          summary.count,
          summary.pre_valid,
          summary.post_valid,
          summary.pre_error,
          summary.post_error);
    }
  }

  static gtsam::Point3 activePoseCentroid(const gtsam::Values& values) {
    gtsam::Point3 centroid = gtsam::Point3::Zero();
    size_t pose_count = 0u;
    for (const gtsam::Key key : values.keys()) {
      if (gtsam::Symbol(key).chr() != 'x') {
        continue;
      }
      centroid += values.at<gtsam::Pose3>(key).translation();
      ++pose_count;
    }
    return pose_count > 0u ? centroid / static_cast<double>(pose_count)
                           : centroid;
  }

  static gtsam::Values applyCommonWorldTransform(
      const gtsam::Values& values,
      const gtsam::Pose3& world_change) {
    gtsam::Values transformed(values);
    for (const gtsam::Key key : values.keys()) {
      switch (gtsam::Symbol(key).chr()) {
        case 'x':
          transformed.update(
              key, world_change.compose(values.at<gtsam::Pose3>(key)));
          break;
        case 'v':
          transformed.update(
              key,
              world_change.rotation().rotate(values.at<gtsam::Vector3>(key)));
          break;
        case 'l':
          transformed.update(
              key,
              world_change.transformFrom(values.at<gtsam::Point3>(key)));
          break;
        default:
          break;
      }
    }
    return transformed;
  }

  static CommonGaugeMode makeCommonGaugeMode(
      const gtsam::Values& values,
      const std::string& name,
      const gtsam::Vector3& axis,
      const bool rotation,
      const double finite_difference_step,
      const gtsam::Point3& rotation_center) {
    const auto make_world_change = [&](const double signed_step) {
      if (!rotation) {
        return gtsam::Pose3(gtsam::Rot3(), signed_step * axis);
      }
      const gtsam::Rot3 world_rotation =
          gtsam::Rot3::Expmap(signed_step * axis);
      const gtsam::Point3 world_translation =
          rotation_center - world_rotation.rotate(rotation_center);
      return gtsam::Pose3(world_rotation, world_translation);
    };

    const gtsam::Values plus = applyCommonWorldTransform(
        values, make_world_change(finite_difference_step));
    const gtsam::Values minus = applyCommonWorldTransform(
        values, make_world_change(-finite_difference_step));
    gtsam::VectorValues tangent =
        values.localCoordinates(plus) - values.localCoordinates(minus);
    tangent *= 0.5 / finite_difference_step;

    CommonGaugeMode mode;
    mode.name = name;
    mode.finite_difference_step = finite_difference_step;
    mode.tangent = std::move(tangent);
    mode.tangent_squared_norm = mode.tangent.dot(mode.tangent);
    return mode;
  }

  static std::vector<CommonGaugeMode> makeCommonGaugeModes(
      const gtsam::Values& values) {
    constexpr double kTranslationStepM = 1e-4;
    constexpr double kRotationStepRad = 1e-5;
    const gtsam::Point3 rotation_center = activePoseCentroid(values);
    const gtsam::Vector3 x_axis(1.0, 0.0, 0.0);
    const gtsam::Vector3 y_axis(0.0, 1.0, 0.0);
    const gtsam::Vector3 z_axis(0.0, 0.0, 1.0);

    std::vector<CommonGaugeMode> modes;
    modes.reserve(6u);
    modes.push_back(makeCommonGaugeMode(values,
                                        "tx",
                                        x_axis,
                                        false,
                                        kTranslationStepM,
                                        rotation_center));
    modes.push_back(makeCommonGaugeMode(values,
                                        "ty",
                                        y_axis,
                                        false,
                                        kTranslationStepM,
                                        rotation_center));
    modes.push_back(makeCommonGaugeMode(values,
                                        "tz",
                                        z_axis,
                                        false,
                                        kTranslationStepM,
                                        rotation_center));
    modes.push_back(makeCommonGaugeMode(values,
                                        "roll",
                                        x_axis,
                                        true,
                                        kRotationStepRad,
                                        rotation_center));
    modes.push_back(makeCommonGaugeMode(values,
                                        "pitch",
                                        y_axis,
                                        true,
                                        kRotationStepRad,
                                        rotation_center));
    modes.push_back(makeCommonGaugeMode(values,
                                        "yaw",
                                        z_axis,
                                        true,
                                        kRotationStepRad,
                                        rotation_center));
    return modes;
  }

  void logCommonGaugeCurvatureAudits(
      const double current_timestamp,
      const char* stage,
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& values) const {
    try {
      const std::vector<CommonGaugeMode> modes =
          makeCommonGaugeModes(values);
      if (modes.empty()) {
        return;
      }
      const gtsam::VectorValues zero =
          gtsam::VectorValues::Zero(modes.front().tangent);
      std::map<std::string, CommonGaugeCategorySummary> summaries;

      for (const auto& nonlinear_factor : graph) {
        if (!nonlinear_factor) {
          continue;
        }
        gtsam::GaussianFactor::shared_ptr linear_factor;
        try {
          linear_factor = nonlinear_factor->linearize(values);
        } catch (const std::exception&) {
          continue;
        }
        if (!linear_factor) {
          continue;
        }

        const std::string category =
            factorAuditCategory(nonlinear_factor);
        const double base_error = linear_factor->error(zero);
        for (const std::string& summary_name :
             std::vector<std::string>{"all", category}) {
          CommonGaugeCategorySummary& summary = summaries[summary_name];
          ++summary.factor_count;
          summary.base_error += base_error;
          for (const CommonGaugeMode& mode : modes) {
            CommonGaugeModeErrors& errors =
                summary.mode_errors[mode.name];
            errors.plus_error += linear_factor->error(mode.tangent);
            errors.minus_error += linear_factor->error(-1.0 * mode.tangent);
          }
        }
      }

      size_t pose_count = 0u;
      for (const gtsam::Key key : values.keys()) {
        pose_count += gtsam::Symbol(key).chr() == 'x' ? 1u : 0u;
      }
      for (const auto& [category, summary] : summaries) {
        for (const CommonGaugeMode& mode : modes) {
          const CommonGaugeModeErrors& errors =
              summary.mode_errors.at(mode.name);
          const double directional_gradient =
              0.5 * (errors.plus_error - errors.minus_error);
          const double curvature =
              errors.plus_error + errors.minus_error -
              2.0 * summary.base_error;
          const double normalized_curvature =
              curvature / std::max(1e-15, mode.tangent_squared_norm);
          spdlog::info(
              "CBS_FIXED_LAG_GAUGE_CURVATURE_ROW,{},{},{},{},{},{},{},{},{},"
              "{},{},{},{}",
              isam_.id(),
              current_timestamp,
              stage,
              category,
              summary.factor_count,
              pose_count,
              mode.name,
              mode.finite_difference_step,
              mode.tangent_squared_norm,
              summary.base_error,
              directional_gradient,
              curvature,
              normalized_curvature);
        }
      }
    } catch (const std::exception& e) {
      spdlog::warn("CBS fixed-lag gauge curvature audit failed: {}",
                   e.what());
    }
  }

  static PoseTransitionSummary summarizePoseTransition(
      const gtsam::Values& before,
      const gtsam::Values& after,
      const gtsam::KeySet& excluded_keys) {
    PoseTransitionSummary summary;
    double translation_sum = 0.0;
    double rotation_sum_deg = 0.0;
    constexpr double kRadiansToDegrees =
        57.295779513082320876798154814105;

    for (const gtsam::Key key : before.keys()) {
      if (excluded_keys.exists(key) || !after.exists(key) ||
          gtsam::Symbol(key).chr() != 'x') {
        continue;
      }

      const gtsam::Pose3 before_pose = before.at<gtsam::Pose3>(key);
      const gtsam::Pose3 after_pose = after.at<gtsam::Pose3>(key);
      const gtsam::Pose3 change = before_pose.between(after_pose);
      const double translation_m = change.translation().norm();
      const double rotation_deg =
          gtsam::Rot3::Logmap(change.rotation()).norm() * kRadiansToDegrees;

      if (summary.pose_count == 0u || key < summary.first_key) {
        summary.first_key = key;
        summary.first_translation_m = translation_m;
      }
      if (summary.pose_count == 0u || key > summary.last_key) {
        summary.last_key = key;
        summary.last_translation_m = translation_m;
      }
      ++summary.pose_count;
      translation_sum += translation_m;
      rotation_sum_deg += rotation_deg;
      summary.translation_max_m =
          std::max(summary.translation_max_m, translation_m);
      summary.rotation_max_deg =
          std::max(summary.rotation_max_deg, rotation_deg);
    }

    if (summary.pose_count > 0u) {
      summary.translation_mean_m =
          translation_sum / static_cast<double>(summary.pose_count);
      summary.rotation_mean_deg =
          rotation_sum_deg / static_cast<double>(summary.pose_count);

      const gtsam::Pose3 first_before =
          before.at<gtsam::Pose3>(summary.first_key);
      const gtsam::Pose3 first_after =
          after.at<gtsam::Pose3>(summary.first_key);
      const gtsam::Pose3 common_world_change =
          first_after.compose(first_before.inverse());
      const gtsam::Point3 common_translation =
          common_world_change.translation();
      const gtsam::Vector3 common_rotation_deg =
          gtsam::Rot3::Logmap(common_world_change.rotation()) *
          kRadiansToDegrees;
      summary.common_world_translation_x_m = common_translation.x();
      summary.common_world_translation_y_m = common_translation.y();
      summary.common_world_translation_z_m = common_translation.z();
      summary.common_world_rotation_x_deg = common_rotation_deg.x();
      summary.common_world_rotation_y_deg = common_rotation_deg.y();
      summary.common_world_rotation_z_deg = common_rotation_deg.z();

      double rigid_translation_sum = 0.0;
      double rigid_rotation_sum_deg = 0.0;
      for (const gtsam::Key key : before.keys()) {
        if (excluded_keys.exists(key) || !after.exists(key) ||
            gtsam::Symbol(key).chr() != 'x') {
          continue;
        }
        const gtsam::Pose3 predicted_after = common_world_change.compose(
            before.at<gtsam::Pose3>(key));
        const gtsam::Pose3 residual =
            predicted_after.between(after.at<gtsam::Pose3>(key));
        const double residual_translation_m = residual.translation().norm();
        const double residual_rotation_deg =
            gtsam::Rot3::Logmap(residual.rotation()).norm() *
            kRadiansToDegrees;
        rigid_translation_sum += residual_translation_m;
        rigid_rotation_sum_deg += residual_rotation_deg;
        summary.rigid_residual_translation_max_m = std::max(
            summary.rigid_residual_translation_max_m,
            residual_translation_m);
        summary.rigid_residual_rotation_max_deg =
            std::max(summary.rigid_residual_rotation_max_deg,
                     residual_rotation_deg);
      }
      summary.rigid_residual_translation_mean_m =
          rigid_translation_sum / static_cast<double>(summary.pose_count);
      summary.rigid_residual_rotation_mean_deg =
          rigid_rotation_sum_deg / static_cast<double>(summary.pose_count);
    }
    return summary;
  }

  void logPoseTransition(const double current_timestamp,
                         const char* stage,
                         const gtsam::Values& before,
                         const gtsam::Values& after,
                         const gtsam::KeySet& excluded_keys) const {
    const PoseTransitionSummary summary =
        summarizePoseTransition(before, after, excluded_keys);
    spdlog::info(
        "CBS_FIXED_LAG_STAGE_ROW,{},{},{},{},{},{},{},{},{},{},{},{},"
        "{},{},{},{},{},{},{},{},{},{}",
        isam_.id(),
        current_timestamp,
        stage,
        summary.pose_count,
        summary.translation_mean_m,
        summary.translation_max_m,
        summary.rotation_mean_deg,
        summary.rotation_max_deg,
        summary.pose_count > 0u ? MultiRobotKeyFormatter(summary.first_key)
                                : "-",
        summary.first_translation_m,
        summary.pose_count > 0u ? MultiRobotKeyFormatter(summary.last_key)
                                : "-",
        summary.last_translation_m,
        summary.common_world_translation_x_m,
        summary.common_world_translation_y_m,
        summary.common_world_translation_z_m,
        summary.common_world_rotation_x_deg,
        summary.common_world_rotation_y_deg,
        summary.common_world_rotation_z_deg,
        summary.rigid_residual_translation_mean_m,
        summary.rigid_residual_translation_max_m,
        summary.rigid_residual_rotation_mean_deg,
        summary.rigid_residual_rotation_max_deg);
  }

  void logMarginalizationFactors(
      const double current_timestamp,
      const gtsam::KeyVector& marginalized_keys,
      const gtsam::FactorIndices& marginal_factor_indices,
      const gtsam::FactorIndices& deleted_factor_indices,
      const gtsam::Values& values) const {
    size_t pose_keys = 0u;
    size_t velocity_keys = 0u;
    size_t bias_keys = 0u;
    for (const gtsam::Key key : marginalized_keys) {
      switch (gtsam::Symbol(key).chr()) {
        case 'x':
          ++pose_keys;
          break;
        case 'v':
          ++velocity_keys;
          break;
        case 'b':
          ++bias_keys;
          break;
        default:
          break;
      }
    }

    spdlog::info(
        "CBS_FIXED_LAG_MARGINALIZATION_ROW,{},{},{},{},{},{},{},{}",
        isam_.id(),
        current_timestamp,
        marginalized_keys.size(),
        pose_keys,
        velocity_keys,
        bias_keys,
        marginal_factor_indices.size(),
        deleted_factor_indices.size());

    const auto& graph = isam_.getFactorsUnsafe();
    for (const gtsam::FactorIndex factor_index : marginal_factor_indices) {
      if (factor_index >= graph.size() || !graph.exists(factor_index)) {
        spdlog::warn(
            "CBS_FIXED_LAG_PRIOR_ROW,{},{},{},missing_factor_slot,-,nan",
            isam_.id(),
            current_timestamp,
            factor_index);
        continue;
      }

      const auto& factor = graph.at(factor_index);
      std::ostringstream keys;
      bool first = true;
      for (const gtsam::Key key : factor->keys()) {
        if (!first) {
          keys << '|';
        }
        first = false;
        keys << MultiRobotKeyFormatter(key);
      }

      double error = std::numeric_limits<double>::quiet_NaN();
      try {
        error = factor->error(values);
      } catch (const std::exception&) {
      }
      spdlog::info("CBS_FIXED_LAG_PRIOR_ROW,{},{},{},{},{},{}",
                   isam_.id(),
                   current_timestamp,
                   factor_index,
                   factor->keys().size(),
                   keys.str(),
                   error);
    }
  }

  void createOrderingConstraints(
      const gtsam::KeyVector& marginalizableKeys,
      std::optional<gtsam::FastMap<gtsam::Key, int>>& constrainedKeys) const {
    if (marginalizableKeys.empty()) {
      return;
    }

    constrainedKeys = gtsam::FastMap<gtsam::Key, int>();
    for (const auto& timestamp_key : timestampKeyMap_) {
      (*constrainedKeys)[timestamp_key.second] = 1;
    }
    for (gtsam::Key key : marginalizableKeys) {
      (*constrainedKeys)[key] = 0;
    }
  }

  bool isActiveLocalKey(gtsam::Key key) const {
    const auto timestamp_it = keyTimestampMap_.find(key);
    if (timestamp_it == keyTimestampMap_.end()) {
      return false;
    }
    if (!isam_.valueExists(key)) {
      return false;
    }
    const double oldest_active_timestamp =
        getCurrentTimestamp() - smootherLag_;
    return timestamp_it->second >= oldest_active_timestamp;
  }

  gtsam::KeySet activeLocalKeys() const {
    gtsam::KeySet keys;
    if (keyTimestampMap_.empty()) {
      return keys;
    }

    const double oldest_active_timestamp =
        getCurrentTimestamp() - smootherLag_;
    for (const auto& [key, timestamp] : keyTimestampMap_) {
      if (timestamp >= oldest_active_timestamp && isam_.valueExists(key)) {
        keys.insert(key);
      }
    }
    return keys;
  }

  static gtsam::ISAM2Params defaultIsam2Params() {
    gtsam::ISAM2Params params;
    return params;
  }

  static cbs::BPSAM::Params makeBpsamParams(
      const gtsam::ISAM2Params& isam2_params,
      cbs::AgentId robot_id) {
    cbs::BPSAM::Params params;
    params.robot_id = robot_id;
    params.sam_params_ = isam2_params;
    return params;
  }

  cbs::BPSAM isam_;
  gtsam::ISAM2Result isamResult_;
  std::optional<gtsam::ISAM2> unmarginalized_reference_isam_;
  bool shadow_acceptance_force_full_solve_ = false;
  ShadowAcceptanceAudit* shadow_acceptance_audit_out_ = nullptr;
  std::optional<gtsam::Values> shadow_acceptance_initialized_values_;
  bool capture_bad_update_stages_once_ = false;
  std::optional<BadUpdateStageAudit> bad_update_stage_audit_;
};

}  // namespace cbs
