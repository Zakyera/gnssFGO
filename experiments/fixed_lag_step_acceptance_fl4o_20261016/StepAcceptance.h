#pragma once
#include <cbs/bpsam/bpsam.h>
#include <cbs/bpsam/fixed_lag_objective_audit.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fl4o {
using namespace gtsam;
inline bool enabled() {
  const char* e=std::getenv("FL4O_ENABLE");
  if(!e || std::string(e)=="0")return false;
  if(std::string(e)=="1")return true;
  throw std::runtime_error("FL4O invalid selector");
}
inline Values subset(const Values& v,const KeySet& keys) {
  Values r;for(Key k:keys)r.insert(k,v.at(k));return r;
}
// A fresh clone per evaluation prevents smart-factor evaluation caches at one
// trial point from affecting the next trial or the real estimator.
inline double cost(const NonlinearFactorGraph& graph,const Values& v,
                   const Values& theta) {
  auto copy=graph.clone();double e=0;
  for(const auto& f:copy)if(f)e+=cbs::fixedLagFactorErrorInChart(*f,v,theta);
  if(!std::isfinite(e))throw std::runtime_error("FL4O invalid objective");
  return e;
}
struct Decision {
  Values accepted;
  double before=0,full=0,after=0,alpha=1;
  size_t evaluations=0;
};
inline Decision select(const NonlinearFactorGraph& graph,const Values& theta,
                       const Values& initialized,const Values& proposed,
                       size_t max_halvings=16) {
  if(max_halvings>16)throw std::runtime_error("FL4O invalid search bound");
  Decision r;
  const auto keys=graph.keys();
  const auto before=subset(initialized,keys), full=subset(proposed,keys);
  const auto anchor=subset(theta,keys);
  r.before=cost(graph,before,anchor);++r.evaluations;
  // Invalid full cost fails closed, rather than hiding a broken factor/type.
  r.full=cost(graph,full,anchor);++r.evaluations;
  const double tolerance=1e-10*(1+std::abs(r.before));
  if(r.full<=r.before+tolerance) {
    r.accepted=full;r.after=r.full;return r;
  }
  const auto d0=anchor.localCoordinates(before);
  const auto d1=anchor.localCoordinates(full);
  double best=r.full;
  for(size_t n=1;n<=max_halvings;++n) {
    r.alpha=std::ldexp(1.,-static_cast<int>(n));
    auto d=d0;
    for(auto& kv:d)kv.second=(1-r.alpha)*kv.second+r.alpha*d1.at(kv.first);
    auto trial=anchor.retract(d);
    const double e=cost(graph,trial,anchor);++r.evaluations;
    best=std::min(best,e);
    if(e<r.before-tolerance) {
      r.after=e;r.accepted=trial;return r;
    }
  }
  std::ostringstream why;why<<std::setprecision(17)
    <<"FL4O no decreasing step within "<<max_halvings<<" halvings: before="
    <<r.before<<" full="<<r.full<<" best_trial="<<best;
  throw std::runtime_error(why.str());
}

// Restore mutable triangulation caches by value, NOT by replacing shared_ptrs.
// Kimera's old_smart_factors_ explicitly depends on original pointer identity.
class SmartBackup {
  using Smart=SmartStereoProjectionPoseFactor;
  std::vector<std::pair<std::shared_ptr<Smart>,Smart>> items_;
 public:
  SmartBackup(const NonlinearFactorGraph& old,const NonlinearFactorGraph& added) {
    std::set<const void*> seen;
    auto add=[&](const NonlinearFactorGraph& g) {
      for(const auto& f:g)if(f&&seen.insert(f.get()).second) {
        if(auto s=std::dynamic_pointer_cast<Smart>(f))items_.emplace_back(s,*s);
        else if(!(dynamic_cast<const LinearContainerFactor*>(f.get()) ||
                  dynamic_cast<const ImuFactorT<>*>(f.get()) ||
                  dynamic_cast<const BetweenFactor<Pose3>*>(f.get()) ||
                  dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(f.get()) ||
                  dynamic_cast<const PriorFactor<Pose3>*>(f.get()) ||
                  dynamic_cast<const PriorFactor<Vector3>*>(f.get()) ||
                  dynamic_cast<const PriorFactor<imuBias::ConstantBias>*>(f.get())))
          throw std::runtime_error("FL4O unsupported rollback factor family");
      }
    };
    add(old);add(added);
  }
  void restore() {for(auto& item:items_)item.first->fl4oRestoreMutableCaches(item.second);}
};
template<class Smoother,class F>
auto transaction(Smoother& smoother,const NonlinearFactorGraph& added,F action)
    -> decltype(action()) {
  // Copy before timestamp updates/removals. GTSAM copies the Bayes tree; all
  // mutable smart objects referenced by that copy are backed up separately.
  Smoother backup(smoother);
  SmartBackup caches(smoother.getFactors(),added);
  try {return action();}
  catch(...) {
    smoother=backup;
    caches.restore();
    throw;
  }
}
inline Decision accept(double update,cbs::BPSAM& isam,
                       const Values& initialized) {
  auto decision=select(isam.getFactorsUnsafe(),isam.getLinearizationPoint(),
                       initialized,isam.calculateEstimate());
  if(decision.alpha<1)isam.fl4oCommitAcceptedValues(decision.accepted);
  const auto accepted=isam.calculateEstimate();
  const auto d=decision.accepted.localCoordinates(accepted);
  for(const auto& kv:d)if(!kv.second.allFinite()||kv.second.norm()>1e-10)
    throw std::runtime_error("FL4O accepted estimate did not persist");
  if(const char* path=std::getenv("FL4O_LOG")) {
    std::ofstream f(path,std::ios::app);
    f<<std::setprecision(17)<<update<<','<<decision.before<<','<<decision.full
     <<','<<decision.after<<','<<decision.alpha<<','<<decision.evaluations<<'\n';
    if(!f)throw std::runtime_error("FL4O cannot write acceptance log");
  }
  return decision;
}
} // namespace fl4o
