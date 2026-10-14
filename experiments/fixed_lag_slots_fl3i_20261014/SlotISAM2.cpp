/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.cpp
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

#include "SlotISAM2.h"
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <algorithm>
#include <map>
#include <vector>
using namespace std;
using namespace gtsam;
namespace fl3i_slots {
void SlotISAM2::marginalizeLeaves(
    const FastList<Key>& leafKeysList,
    boost::optional<FactorIndices&> marginalFactorsIndices,
    boost::optional<FactorIndices&> deletedFactorsIndices) {
  // Convert to ordered set
  KeySet leafKeys(leafKeysList.begin(), leafKeysList.end());

  // Keep track of marginal factors - map from clique to the marginal factors
  // that should be incorporated into it, passed up from it's children.
  //  multimap<sharedClique, GaussianFactor::shared_ptr> marginalFactors;
  map<Key, vector<GaussianFactor::shared_ptr> > marginalFactors;

  // Keep track of variables removed in subtrees
  KeySet leafKeysRemoved;

  // Keep track of factors that get summarized by removing cliques
  FactorIndexSet factorIndicesToRemove;

  // Remove the subtree and throw away the cliques
  auto trackingRemoveSubtree = [&](const sharedClique& subtreeRoot) {
    const Cliques removedCliques = this->removeSubtree(subtreeRoot);
    for (const sharedClique& removedClique : removedCliques) {
      auto cg = removedClique->conditional();
      marginalFactors.erase(cg->front());
      leafKeysRemoved.insert(cg->beginFrontals(), cg->endFrontals());
      for (Key frontal : cg->frontals()) {
        // Add to factors to remove
        const auto& involved = variableIndex_[frontal];
        factorIndicesToRemove.insert(involved.begin(), involved.end());
#if !defined(NDEBUG)
        // Check for non-leaf keys
        if (!leafKeys.exists(frontal))
          throw std::runtime_error(
              "Requesting to marginalize variables that are not leaves, "
              "the ISAM2 object is now in an inconsistent state so should "
              "no longer be used.");
#endif
      }
    }
    return removedCliques;
  };

  // Remove each variable and its subtrees
  for (Key j : leafKeys) {
    if (!leafKeysRemoved.exists(j)) {  // If the index was not already removed
                                       // by removing another subtree

      // Traverse up the tree to find the root of the marginalized subtree
      sharedClique clique = nodes_[j];
      while (!clique->parent_._empty()) {
        // Check if parent contains a marginalized leaf variable.  Only need to
        // check the first variable because it is the closest to the leaves.
        sharedClique parent = clique->parent();
        if (leafKeys.exists(parent->conditional()->front()))
          clique = parent;
        else
          break;
      }

      // See if we should remove the whole clique
      bool marginalizeEntireClique = true;
      for (Key frontal : clique->conditional()->frontals()) {
        if (!leafKeys.exists(frontal)) {
          marginalizeEntireClique = false;
          break;
        }
      }

      // Remove either the whole clique or part of it
      if (marginalizeEntireClique) {
        // Remove the whole clique and its subtree, and keep the marginal
        // factor.
        auto marginalFactor = clique->cachedFactor();
        // We do not need the marginal factors associated with this clique
        // because their information is already incorporated in the new
        // marginal factor.  So, now associate this marginal factor with the
        // parent of this clique.
        marginalFactors[clique->parent()->conditional()->front()].push_back(
            marginalFactor);
        // Now remove this clique and its subtree - all of its marginal
        // information has been stored in marginalFactors.
        trackingRemoveSubtree(clique);
      } else {
        // Reeliminate the current clique and the marginals from its children,
        // then keep only the marginal on the non-marginalized variables.  We
        // get the childrens' marginals from any existing children, plus
        // the marginals from the marginalFactors multimap, which come from any
        // subtrees already marginalized out.

        // Add child marginals and remove marginalized subtrees
        GaussianFactorGraph graph;
        KeySet factorsInSubtreeRoot;
        Cliques subtreesToRemove;
        for (const sharedClique& child : clique->children) {
          // Remove subtree if child depends on any marginalized keys
          for (Key parent : child->conditional()->parents()) {
            if (leafKeys.exists(parent)) {
              subtreesToRemove.push_back(child);
              graph.push_back(child->cachedFactor());  // Add child marginal
              break;
            }
          }
        }
        Cliques childrenRemoved;
        for (const sharedClique& subtree : subtreesToRemove) {
          const Cliques removed = trackingRemoveSubtree(subtree);
          childrenRemoved.insert(childrenRemoved.end(), removed.begin(),
                                 removed.end());
        }

        // Add the factors that are pulled into the current clique by the
        // marginalized variables. These are the factors that involve
        // *marginalized* frontal variables in this clique but do not involve
        // frontal variables of any of its children.
        // TODO(dellaert): reuse cached linear factors
        KeySet factorsFromMarginalizedInClique_step1;
        for (Key frontal : clique->conditional()->frontals()) {
          if (leafKeys.exists(frontal))
            factorsFromMarginalizedInClique_step1.insert(
                variableIndex_[frontal].begin(), variableIndex_[frontal].end());
        }
        // Remove any factors in subtrees that we're removing at this step
        for (const sharedClique& removedChild : childrenRemoved) {
          for (Key indexInClique : removedChild->conditional()->frontals()) {
            for (Key factorInvolving : variableIndex_[indexInClique]) {
              factorsFromMarginalizedInClique_step1.erase(factorInvolving);
            }
          }
        }
        // Create factor graph from factor indices
        for (const auto index : factorsFromMarginalizedInClique_step1) {
          graph.push_back(nonlinearFactors_[index]->linearize(theta_));
        }

        // Reeliminate the linear graph to get the marginal and discard the
        // conditional
        auto cg = clique->conditional();
        const KeySet cliqueFrontals(cg->beginFrontals(), cg->endFrontals());
        KeyVector cliqueFrontalsToEliminate;
        std::set_intersection(cliqueFrontals.begin(), cliqueFrontals.end(),
                              leafKeys.begin(), leafKeys.end(),
                              std::back_inserter(cliqueFrontalsToEliminate));
        auto eliminationResult1 = params_.getEliminationFunction()(
            graph, Ordering(cliqueFrontalsToEliminate));

        // Add the resulting marginal
        if (eliminationResult1.second)
          marginalFactors[cg->front()].push_back(eliminationResult1.second);

        // Split the current clique
        // Find the position of the last leaf key in this clique
        DenseIndex nToRemove = 0;
        while (leafKeys.exists(cg->keys()[nToRemove])) ++nToRemove;

        // Make the clique's matrix appear as a subset
        const DenseIndex dimToRemove = cg->matrixObject().offset(nToRemove);
        cg->matrixObject().firstBlock() = nToRemove;
        cg->matrixObject().rowStart() = dimToRemove;

        // Change the keys in the clique
        KeyVector originalKeys;
        originalKeys.swap(cg->keys());
        cg->keys().assign(originalKeys.begin() + nToRemove, originalKeys.end());
        cg->nrFrontals() -= nToRemove;

        // Add to factorIndicesToRemove any factors involved in frontals of
        // current clique
        for (Key frontal : cliqueFrontalsToEliminate) {
          const auto& involved = variableIndex_[frontal];
          factorIndicesToRemove.insert(involved.begin(), involved.end());
        }

        // Add removed keys
        leafKeysRemoved.insert(cliqueFrontalsToEliminate.begin(),
                               cliqueFrontalsToEliminate.end());
      }
    }
  }

  // At this point we have updated the BayesTree, now update the remaining iSAM2
  // data structures

  // Slot lifecycle only: elimination above is byte-identical to pinned GTSAM.
  // Remove summarized factors first, so their vacant slots can be reused.
  NonlinearFactorGraph removedFactors;
  for (const auto index : factorIndicesToRemove) {
    removedFactors.push_back(nonlinearFactors_[index]);
    nonlinearFactors_.remove(index);
    if (params_.cacheLinearizedFactors) linearFactors_.remove(index);
  }
  variableIndex_.remove(factorIndicesToRemove.begin(),
                        factorIndicesToRemove.end(), removedFactors);

  GaussianFactorGraph factorsToAdd;
  NonlinearFactorGraph nonlinearFactorsToAdd;
  for (const auto& key_factors : marginalFactors) {
    for (const auto& factor : key_factors.second) {
      if (factor) {
        factorsToAdd.push_back(factor);
        nonlinearFactorsToAdd.emplace_shared<LinearContainerFactor>(factor);
        for (Key factorKey : *factor) fixedVariables_.insert(factorKey);
      }
    }
  }
  const auto added = nonlinearFactors_.add_factors(
      nonlinearFactorsToAdd, params_.findUnusedFactorSlots);
  if (params_.cacheLinearizedFactors) {
    linearFactors_.resize(nonlinearFactors_.size());
    for (size_t i = 0; i < added.size(); ++i) linearFactors_[added[i]] = factorsToAdd[i];
  }
  variableIndex_.augment(factorsToAdd, added);
  if (marginalFactorsIndices)
    marginalFactorsIndices->insert(marginalFactorsIndices->end(), added.begin(), added.end());
  if (deletedFactorsIndices)
    deletedFactorsIndices->assign(factorIndicesToRemove.begin(), factorIndicesToRemove.end());
  removeVariables(KeySet(leafKeys.begin(), leafKeys.end()));
}
} // namespace fl3i_slots
