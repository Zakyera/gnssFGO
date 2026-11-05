#pragma once
// Experimental representation adapter, NOT a new optimizer or a prior reset.
#include "OriginalChartPrior.h"
#include <gtsam/nonlinear/ISAM2Params.h>
#include <cstdlib>
#include <string>
#include <vector>

namespace cpk9 {
using namespace gtsam;
inline bool enabled() {
  const char* e=std::getenv("CPK9_PRIOR_REFRESH");
  if (!e || std::string(e)=="0") return false;
  if (std::string(e)!="1") throw std::runtime_error("CPK9 invalid selector");
  const char* q=std::getenv("FL4Q_ENABLE");
  const char* r=std::getenv("FL4M_RETIREMENT");
  if (!q || std::string(q)!="0" || !r || std::string(r)!="remove")
    throw std::runtime_error("CPK9 requires explicit FL4Q_ENABLE=0 and retirement=remove");
  return true;
}
struct Promotion {
  size_t priors=0, released=0;
  double cache_error=0;
};
inline double relative(const Matrix& a,const Matrix& b) {
  if (a.rows()!=b.rows() || a.cols()!=b.cols() || !a.allFinite() || !b.allFinite())
    throw std::runtime_error("CPK9 invalid prior equations");
  return (a-b).norm()/(1+b.norm());
}
// No tree rebuild, delta write, nonlinear step or new Gaussian coefficients.
// Validate all replacements before changing any pointer/lock. In particular,
// never unlock a key whose fixed-origin ownership cannot be established.
inline Promotion promote(NonlinearFactorGraph& graph,const Values& theta,
                         const GaussianFactorGraph& cache,KeySet& fixed) {
  if (graph.size()!=cache.size()) throw std::runtime_error("CPK9 cache slots differ");
  std::vector<std::pair<size_t,NonlinearFactor::shared_ptr>> replacements;
  KeySet owned;
  Promotion result;
  for (size_t i=0;i<graph.size();++i) {
    const auto* p=dynamic_cast<const LinearContainerFactor*>(graph[i].get());
    if (!p) continue;
    // The supported case is the unanchored Gaussian made by marginalizeLeaves.
    if (p->hasLinearizationPoint())
      throw std::runtime_error("CPK9 unsupported pre-anchored linear container");
    if (!cache[i] || cache[i]->keys()!=p->keys())
      throw std::runtime_error("CPK9 missing/mismatched marginal cache");
    for (Key key:p->keys()) {
      if (!theta.exists(key) || !fixed.count(key))
        throw std::runtime_error("CPK9 unowned native prior key");
      owned.insert(key);
    }
    auto preserved=std::make_shared<fl4l::OriginalChartPrior>(*p,theta);
    const Matrix old=cache[i]->augmentedInformation();
    const double err=std::max(relative(p->factor()->augmentedInformation(),old),
                             relative(preserved->linearize(theta)->augmentedInformation(),old));
    if (err>1e-8) throw std::runtime_error("CPK9 promotion changes cached prior");
    result.cache_error=std::max(result.cache_error,err);
    replacements.emplace_back(i,preserved);
  }
  if (owned!=fixed) throw std::runtime_error("CPK9 unexplained fixed key");
  result.priors=replacements.size();result.released=fixed.size();
  for (const auto& item:replacements) graph[item.first]=item.second;
  fixed.clear(); // every former lock now belongs to an immutable-anchor adapter
  return result;
}
} // namespace cpk9
