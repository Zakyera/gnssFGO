#pragma once
// CP-V101 isolated candidate: same guard and transaction, richer failure evidence.
#include "OriginalChartPrior.h"
#include "RetainCompatibility.h"
#include "PersistentCompatibility.h"
#include <gtsam/nonlinear/ISAM2Params.h>
#include <cstdlib>
#include <iomanip>
#include <sstream>
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
  if (!q || std::string(q)!="0" || !r || (std::string(r)!="remove" && !cpv69::retain()))
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
// Allocation/copy happens only on failure. Existing std::exception handlers see
// both errors/slot/keys; a typed handler can also archive the exact matrices and
// anchor. No hidden file writes or logging work on a successful promotion.
class PriorPreservationError final : public std::runtime_error {
  static std::string message(size_t slot, const KeyVector& keys,
                             double original_error, double converted_error) {
    std::ostringstream out;
    out << std::setprecision(17)
        << "CPK9 promotion changes cached prior; slot=" << slot
        << " original_cache_error=" << original_error
        << " converted_cache_error=" << converted_error
        << " threshold=1e-8 keys=";
    for (Key key : keys) out << key << ',';
    return out.str();
  }
 public:
  const size_t slot;
  const KeyVector keys;
  const double original_cache_error, converted_cache_error;
  const Matrix original, cached, converted;
  const Values anchor;
  PriorPreservationError(size_t index, const KeyVector& factor_keys,
      double original_error, double converted_error, const Matrix& original_matrix,
      const Matrix& cache_matrix, const Matrix& converted_matrix, const Values& theta)
      : std::runtime_error(message(index, factor_keys, original_error, converted_error)),
        slot(index), keys(factor_keys), original_cache_error(original_error),
        converted_cache_error(converted_error), original(original_matrix),
        cached(cache_matrix), converted(converted_matrix),
        anchor(fl4l::subset(theta, factor_keys)) {}
};
// Validate every replacement before changing any graph pointer or lock.
inline Promotion promote(NonlinearFactorGraph& graph,const Values& theta,
                         const GaussianFactorGraph& cache,KeySet& fixed) {
  if (graph.size()!=cache.size()) throw std::runtime_error("CPK9 cache slots differ");
  std::vector<std::pair<size_t,NonlinearFactor::shared_ptr>> replacements;
  KeySet owned;
  Promotion result;
  for (size_t i=0;i<graph.size();++i) {
    const auto* p=dynamic_cast<const LinearContainerFactor*>(graph[i].get());
    if (!p) continue;
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
    const Matrix original=p->factor()->augmentedInformation();
    const double original_error=relative(original,old);
    const Matrix converted=preserved->linearize(theta)->augmentedInformation();
    const double converted_error=relative(converted,old);
    const double err=std::max(original_error,converted_error);
    if (err>1e-8)
      throw PriorPreservationError(i,p->keys(),original_error,converted_error,
                                   original,old,converted,theta);
    result.cache_error=std::max(result.cache_error,err);
    replacements.emplace_back(i,preserved);
  }
  if (owned!=fixed) throw std::runtime_error("CPK9 unexplained fixed key");
  result.priors=replacements.size();result.released=fixed.size();
  for (const auto& item:replacements) graph[item.first]=item.second;
  fixed.clear();
  return result;
}
} // namespace cpk9
