#pragma once
#include "SlotISAM2.h"
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <iostream>
#include <cstdlib>
namespace fl3j {
using namespace gtsam;
inline bool done=false;
inline void Require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
inline double NormDifference(const Matrix& a,const Matrix& b){return (a-b).norm()/std::max(1.,b.norm());}
inline void Compare(const fl3i_slots::SlotISAM2& before,const FastList<Key>& leaves) {
  size_t newest=0;for(auto k:before.getLinearizationPoint().keys())newest=std::max(newest,Symbol(k).index());
  if(newest!=1018)return;
  Require(!done,"duplicate target retirement");
  fl3i_slots::SlotISAM2 candidate(before);ISAM2 stock(before);
  const auto original=before.getFactorsUnsafe();
  const Values initial=before.getLinearizationPoint();
  FactorIndices ca,cr,sa,sr;
  candidate.marginalizeLeaves(leaves,ca,cr);stock.marginalizeLeaves(leaves,sa,sr);
  Require(cr==sr&&ca.size()==sa.size(),"removed slots/marginal count differ");
  Require(candidate.getFixedVariables()==stock.getFixedVariables(),"fixed key sets differ");
  const auto& cv=candidate.getLinearizationPoint();const auto& sv=stock.getLinearizationPoint();
  Require(cv.keys()==sv.keys(),"retained keys differ");
  for(auto k:cv.keys()) {
    Require(cv.at(k).equals_(sv.at(k),0),"candidate/stock linearization points differ");
    Require(cv.at(k).equals_(initial.at(k),0),"retained anchor changed");
  }
  for(auto k:leaves)Require(!cv.exists(k),"retired key remains");
  const auto& cg=candidate.getFactorsUnsafe();const auto& sg=stock.getFactorsUnsafe();
  GaussianFactorGraph canonicalC,canonicalS;
  size_t survived=0;double marginal=0;
  for(size_t i=0;i<original.size();++i)if(original[i]&&std::find(cr.begin(),cr.end(),i)==cr.end()) {
    Require(cg[i]==original[i]&&sg[i]==original[i],"surviving factor pointer/slot changed");
    canonicalC.push_back(cg[i]->linearize(cv));canonicalS.push_back(sg[i]->linearize(cv));++survived;
  }
  for(size_t i=0;i<ca.size();++i) {
    const auto* a=dynamic_cast<const LinearContainerFactor*>(cg[ca[i]].get());
    const auto* b=dynamic_cast<const LinearContainerFactor*>(sg[sa[i]].get());
    Require(a&&b&&a->keys()==b->keys(),"marginal type/keys differ");
    Require(a->hasLinearizationPoint()==b->hasLinearizationPoint(),"marginal anchor convention differs");
    if(a->hasLinearizationPoint())Require(a->linearizationPoint()->equals(*b->linearizationPoint(),0),"marginal anchor differs");
    marginal=std::max(marginal,NormDifference(a->factor()->augmentedInformation(),b->factor()->augmentedInformation()));
    canonicalC.push_back(a->factor());canonicalS.push_back(b->factor());
  }
  Require(canonicalC.size()==cg.nrFactors()&&canonicalS.size()==sg.nrFactors(),"unaccounted factor");
  const Ordering order(cv.keys());
  const Matrix ac=canonicalC.augmentedHessian(order),as=canonicalS.augmentedHessian(order);
  const auto n=ac.rows()-1;
  const double h=NormDifference(ac.topLeftCorner(n,n),as.topLeftCorner(n,n));
  const double g=NormDifference(ac.topRightCorner(n,1),as.topRightCorner(n,1));
  const double c=std::abs(ac(n,n)-as(n,n))/std::max(1.,std::abs(as(n,n)));
  const auto ce=candidate.calculateEstimate(),se=stock.calculateEstimate();double state=0;
  for(auto k:ce.keys())state=std::max(state,ce.at(k).localCoordinates_(se.at(k)).norm());
  const Matrix rawC=cg.linearize(cv)->augmentedHessian(order),rawS=sg.linearize(cv)->augmentedHessian(order);
  const double raw=NormDifference(rawC,rawS);
  std::ofstream out(std::filesystem::path(std::getenv("FL3J_ROOT"))/"comparison.json");
  out<<std::setprecision(17)<<"{\n\"support\":1018,\"retired_keys\":"<<leaves.size()
    <<",\"surviving_factors\":"<<survived<<",\"new_marginals\":"<<ca.size()
    <<",\"candidate_slots\":"<<cg.size()<<",\"stock_slots\":"<<sg.size()
    <<",\"canonical_H_relative\":"<<h<<",\"canonical_g_relative\":"<<g
    <<",\"canonical_constant_relative\":"<<c<<",\"new_marginal_relative\":"<<marginal
    <<",\"immediate_state_max\":"<<state<<",\"slot_order_augmented_H_relative\":"<<raw
    <<",\"ownership_keys_anchors_exact\":true,\"passed\":"
    <<((h<=1e-10&&g<=1e-10&&c<=1e-10&&marginal<=1e-10&&state<=1e-9)?"true":"false")<<"}\n";
  out.close();Require(bool(out),"failed comparison output");
  Require(h<=1e-10&&g<=1e-10&&c<=1e-10&&marginal<=1e-10&&state<=1e-9,"same-input numerical mismatch");
  done=true;
}
}
