#pragma once
#include "OriginalChartPrior.h"
#include <cbs/bpsam/bpsam.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <set>

namespace fl4q {
using namespace gtsam;
inline bool enabled() {
  const char* e=std::getenv("FL4Q_ENABLE");
  if(!e || std::string(e)=="0")return false;
  if(std::string(e)=="1")return true;
  throw std::runtime_error("FL4Q invalid selector");
}
inline Values subset(const Values& values,const KeySet& keys) {
  Values v;for(Key k:keys)v.insert(k,values.at(k));return v;
}
// Preserve holes, payloads and native pointer identity for the committed graph.
// Only the representation of a Gaussian prior changes, not its objective.
inline NonlinearFactorGraph anchored(const NonlinearFactorGraph& graph,
                                     const Values& theta,bool clone) {
  NonlinearFactorGraph out;
  for(const auto& f:graph) {
    if(!f)out.push_back(NonlinearFactor::shared_ptr());
    else if(auto p=dynamic_cast<const LinearContainerFactor*>(f.get()))
      out.push_back(std::make_shared<fl4l::OriginalChartPrior>(*p,theta));
    else out.push_back(clone?f->clone():f);
  }
  return out;
}
inline double cost(const NonlinearFactorGraph& graph,const Values& values,
                   const Values& theta) {
  auto cloned=graph.clone();double e=0;
  for(const auto& f:cloned)if(f)e+=cbs::fixedLagFactorErrorInChart(*f,values,theta);
  if(!std::isfinite(e))throw std::runtime_error("FL4Q nonfinite objective");
  return e;
}
struct Decision {
  Values accepted;
  double before=0,full=0,after=0,seconds=0;
  size_t iterations=0;
  bool fallback=false;
};
inline Decision select(const NonlinearFactorGraph& graph,const Values& theta,
                       const Values& initialized,const Values& proposed,
                       size_t limit=10) {
  if(!limit || limit>10)throw std::runtime_error("FL4Q iteration bound");
  auto start=std::chrono::steady_clock::now();
  Decision r;const auto keys=graph.keys();
  const Values before=subset(initialized,keys),full=subset(proposed,keys);
  r.before=cost(graph,before,theta);r.full=cost(graph,full,theta);
  const double tolerance=1e-10*(1+std::abs(r.before));
  r.accepted=full;r.after=r.full;
  if(r.full>r.before+tolerance) {
    r.fallback=true;
    auto objective=anchored(graph,theta,true);
    // The exact original prior function must survive conversion.
    for(const auto* v:{&before,&full}) {
      const double a=cost(graph,*v,theta),b=cost(objective,*v,theta);
      if(std::abs(a-b)>1e-10*(1+std::abs(a)))
        throw std::runtime_error("FL4Q prior objective changed");
    }
    LevenbergMarquardtParams p;p.maxIterations=limit;p.lambdaInitial=1e-3;
    p.lambdaUpperBound=1e12;p.setDiagonalDamping(true);
    p.relativeErrorTol=1e-9;p.absoluteErrorTol=1e-9;
    LevenbergMarquardtOptimizer lm(objective,before,p);
    double previous=r.before;
    for(size_t i=0;i<limit;++i) {
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>5)
        throw std::runtime_error("FL4Q fallback wall budget exceeded");
      lm.iterate();++r.iterations;
      const auto v=lm.values();
      // Independent cost on fresh clones, not LM's mutable smart cache error.
      const double e=cost(graph,v,theta);
      if(e<r.after && e<r.before-tolerance) {r.accepted=v;r.after=e;}
      if(std::abs(previous-e)<=1e-9*(1+std::abs(previous)))break;
      previous=e;
    }
    if(!(r.after<r.before-tolerance))
      throw std::runtime_error("FL4Q no decreasing current-chart LM step");
  }
  r.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  return r;
}
class SmartBackup {
  using Smart=SmartStereoProjectionPoseFactor;
  std::vector<std::pair<std::shared_ptr<Smart>,Smart>> saved_;
 public:
  SmartBackup(const NonlinearFactorGraph& graph,const NonlinearFactorGraph& added) {
    std::set<const void*> seen;
    for(const auto* g:{&graph,&added})for(const auto& f:*g)if(f&&seen.insert(f.get()).second) {
      if(auto s=std::dynamic_pointer_cast<Smart>(f))saved_.emplace_back(s,*s);
      else if(!(dynamic_cast<const LinearContainerFactor*>(f.get()) ||
                dynamic_cast<const fl4l::OriginalChartPrior*>(f.get()) ||
                dynamic_cast<const ImuFactorT<>*>(f.get()) ||
                dynamic_cast<const BetweenFactor<Pose3>*>(f.get()) ||
                dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(f.get()) ||
                dynamic_cast<const PriorFactor<Pose3>*>(f.get()) ||
                dynamic_cast<const PriorFactor<Vector3>*>(f.get()) ||
                dynamic_cast<const PriorFactor<imuBias::ConstantBias>*>(f.get())))
        throw std::runtime_error("FL4Q unsupported transaction factor");
    }
  }
  void restore(){for(auto& s:saved_)s.first->fl4oRestoreMutableCaches(s.second);}
};
template<class S,class F>auto transaction(S& smoother,const NonlinearFactorGraph& added,F action)
    -> decltype(action()) {
  S saved(smoother);SmartBackup caches(smoother.getFactors(),added);
  auto restore_marginal_cache=smoother.bpsam().fl4qMarginalCacheRestoreAction();
  try{return action();}catch(...){smoother=saved;caches.restore();restore_marginal_cache();throw;}
}
inline double relative(const Matrix& a,const Matrix& b) {
  if(a.rows()!=b.rows()||a.cols()!=b.cols()||!a.allFinite()||!b.allFinite())
    throw std::runtime_error("FL4Q invalid equations");
  return (a-b).norm()/std::max(1.,b.norm());
}
inline Decision accept(double stamp,cbs::BPSAM& isam,const Values& initialized,
    const std::optional<FastMap<Key,int>>& constraints) {
  const auto graph=isam.getFactorsUnsafe();const auto theta=isam.getLinearizationPoint();
  auto r=select(graph,theta,initialized,isam.calculateEstimate());
  double fresh_error=0,tree_error=0;
  if(r.fallback) {
    auto committed=anchored(graph,theta,false);
    isam.fl4qRebuild(committed,r.accepted,constraints);
    const auto current=isam.calculateEstimate();
    if(!current.equals(r.accepted,1e-10) || !isam.getLinearizationPoint().equals(r.accepted,1e-14))
      throw std::runtime_error("FL4Q accepted state/chart lost");
    const auto order=Ordering(current.keys());
    const auto cached=isam.fl4nLinearCache().hessian(order);
    const auto fresh=committed.clone().linearize(current)->hessian(order);
    const auto tree=GaussianFactorGraph(isam).hessian(order);
    fresh_error=std::max(relative(cached.first,fresh.first),relative(cached.second,fresh.second));
    tree_error=std::max(relative(cached.first,tree.first),relative(cached.second,tree.second));
    if(fresh_error>1e-8 || tree_error>1e-8)
      throw std::runtime_error("FL4Q cache/tree/current model mismatch");
    const double after=cost(committed,current,current);
    if(after>r.before+1e-10*(1+std::abs(r.before)) ||
       std::abs(after-r.after)>1e-8*(1+std::abs(r.after)))
      throw std::runtime_error("FL4Q committed objective mismatch");
  }
  if(const char* path=std::getenv("FL4Q_LOG")) {
    std::ofstream f(path,std::ios::app);
    f<<std::setprecision(17)<<stamp<<','<<r.before<<','<<r.full<<','<<r.after<<','
     <<r.fallback<<','<<r.iterations<<','<<r.seconds<<','<<fresh_error<<','<<tree_error<<'\n';
    if(!f)throw std::runtime_error("FL4Q log failed");
  }
  return r;
}
} // namespace fl4q
