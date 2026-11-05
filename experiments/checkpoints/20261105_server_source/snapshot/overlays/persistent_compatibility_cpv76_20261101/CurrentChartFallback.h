#pragma once
#include "OriginalChartPrior.h"
#include <cbs/bpsam/bpsam.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <chrono>
#include <array>
#include <Eigen/Eigenvalues>
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
// Read-only localization on independent factor clones, only after failure.
inline std::string cpk1Family(const NonlinearFactor& f) {
  if(dynamic_cast<const SmartStereoProjectionPoseFactor*>(&f))return "smart_stereo";
  if(dynamic_cast<const fl4l::OriginalChartPrior*>(&f))return "marginal_prior";
  if(dynamic_cast<const ImuFactorT<>*>(&f))return "imu";
  if(dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(&f))return "bias_between";
  if(dynamic_cast<const BetweenFactor<Pose3>*>(&f))return "pose_between";
  return typeid(f).name();
}
inline void cpk1Matrix(std::ostream& out,const std::string& label,const Matrix& m) {
  out<<label<<','<<m.rows()<<','<<m.cols();
  for(Eigen::Index i=0;i<m.rows();++i)for(Eigen::Index j=0;j<m.cols();++j)out<<','<<m(i,j);
  out<<'\n';
}
inline std::pair<double,int> cpk1Cost(const NonlinearFactor& f,const Values& v,bool refresh=false) {
  auto q=f.clone();auto smart=std::dynamic_pointer_cast<SmartStereoProjectionPoseFactor>(q);
  if(refresh&&smart)smart->cpk1ClearCache();
  double e=q->error(v);if(!std::isfinite(e))throw std::runtime_error("CPK1 nonfinite factor error");
  return {e,smart?int(smart->point().status):-1};
}
inline void factorProbe(double stamp,const NonlinearFactorGraph& graph,const Values& theta,
                        const Values& before) {
  const char* path=std::getenv("CPK1_PROBE");if(!path)return;
  if(std::ifstream(path).good())throw std::runtime_error("CPK1 refuse overwrite");
  std::ofstream out(path), snapshot(std::string(path)+".snapshot");
  out<<std::setprecision(17)<<std::unitbuf;snapshot<<std::setprecision(17);
  auto objective=anchored(graph,theta,true);
  const Ordering order(before.keys());auto hg=objective.clone().linearize(before)->hessian(order);
  Matrix H=.5*(hg.first+hg.first.transpose());Vector g=-hg.second;
  if(!H.allFinite()||!g.allFinite()||(H.diagonal().array()<=0).any())
    throw std::runtime_error("CPK1 invalid model");
  Vector scale=H.diagonal().array().sqrt().inverse(),scaled=scale.asDiagonal()*g;
  Vector direction=Vector::Zero(g.size());
  if(scaled.norm()>0)direction=-(scale.asDiagonal()*scaled)/scaled.norm();
  VectorValues d=before.zeroVectors();size_t offset=0;
  for(Key k:order){d.at(k)=direction.segment(offset,d.at(k).size());offset+=d.at(k).size();}
  cpk1Matrix(snapshot,"H",H);cpk1Matrix(snapshot,"g",g);cpk1Matrix(snapshot,"direction",direction);
  out<<"decision,"<<stamp<<','<<cost(graph,before,theta)<<','<<g.dot(direction)<<'\n';
  // fixed, independently declared numerical derivative scales; no state update.
  const std::array<double,3> alphas{.001,.01,.1};
  std::vector<Values> plus,minus;
  for(double a:alphas){auto p=d,m=d;for(auto& kv:p)kv.second*=a;for(auto& kv:m)kv.second*=-a;
    plus.push_back(before.retract(p));minus.push_back(before.retract(m));}
  double sum_cost=0,sum_analytic=0;std::array<double,3> sum_fd{0,0,0};
  for(size_t slot=0;slot<objective.size();++slot) {
    const auto& f=objective[slot];if(!f)continue;
    auto linear=f->clone()->linearize(before);double analytic=0;
    if(linear)for(const auto& kv:linear->gradientAtZero())analytic+=kv.second.dot(d.at(kv.first));
    auto initial=cpk1Cost(*f,before);sum_cost+=initial.first;sum_analytic+=analytic;
    out<<"factor,"<<slot<<','<<cpk1Family(*f)<<','<<f->keys().size()<<','<<initial.first<<','<<analytic<<','<<initial.second;
    for(size_t j=0;j<alphas.size();++j){auto p=cpk1Cost(*f,plus[j]),m=cpk1Cost(*f,minus[j]);
      double fd=(p.first-m.first)/(2*alphas[j]);sum_fd[j]+=fd;
      out<<','<<fd<<','<<p.second<<','<<m.second;}
    auto smart=std::dynamic_pointer_cast<SmartStereoProjectionPoseFactor>(f);
    if(smart){
      auto p=cpk1Cost(*f,plus[1],true),m=cpk1Cost(*f,minus[1],true);
      auto fresh=cpk1Cost(*f,before,true);
      out<<','<<fresh.first<<','<<(p.first-m.first)/(.02)<<','<<fresh.second<<','<<p.second<<','<<m.second;
      snapshot<<"smart,"<<slot<<'\n';smart->cpk1Dump(snapshot);
      for(Key k:f->keys()){
        cpk1Matrix(snapshot,"pose_"+std::to_string(k),before.at<Pose3>(k).matrix());
        cpk1Matrix(snapshot,"dir_"+std::to_string(k),d.at(k));}
    }
    out<<'\n';
  }
  out<<"sums,"<<sum_cost<<','<<sum_analytic;
  for(double fd:sum_fd)out<<','<<fd;out<<'\n';
  out<<"repeat,"<<cost(graph,before,theta)<<"\ndone,1\n";
  if(!out||!snapshot)throw std::runtime_error("CPK1 write failed");
}

// Called only AFTER the unchanged decision has failed. All factor evaluations
// and linearizations below use clones; no candidate is committed.
inline void stoppingProbe(double stamp,const NonlinearFactorGraph& graph,
    const Values& theta,const Values& before,const Values& full,
    const Values& last,const Decision& decision,
    const std::vector<std::array<double,4>>& trials) {
  const char* path=std::getenv("FL4U_PROBE");if(!path)return;
  if(std::ifstream(path).good())throw std::runtime_error("FL4U refuse probe overwrite");
  std::ofstream out(path);out<<std::setprecision(17)<<std::unitbuf;
  if(!out)throw std::runtime_error("FL4U probe open failed");
  const double f0=cost(graph,before,theta);
  out<<"decision,"<<stamp<<','<<decision.before<<','<<decision.full<<','
     <<decision.after<<','<<decision.iterations<<','<<1e-10*(1+std::abs(f0))<<'\n';
  for(const auto& t:trials)out<<"trial,"<<t[0]<<','<<t[1]<<','<<t[2]<<','<<t[3]<<'\n';
  out<<"last,"<<cost(graph,last,theta)<<','<<before.localCoordinates(last).norm()<<'\n';
  auto objective=anchored(graph,theta,true);
  const Ordering ordering(before.keys());
  auto linear=objective.clone().linearize(before);
  const auto hg=linear->hessian(ordering);
  const Matrix H=.5*(hg.first+hg.first.transpose());
  const Vector g=-hg.second;
  Vector diag=H.diagonal();
  out<<"finite,"<<H.allFinite()<<','<<g.allFinite()<<'\n';
  if(!H.allFinite()||!g.allFinite()||(diag.array()<=0).any())
    throw std::runtime_error("FL4U invalid/zero diagonal for scaling");
  Vector scale=diag.array().sqrt().inverse();
  const Vector scaled=scale.asDiagonal()*g;
  const Matrix C=scale.asDiagonal()*H*scale.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Matrix> eig(C);
  out<<"model,"<<g.size()<<','<<g.norm()<<','<<scaled.norm()<<','
     <<scaled.lpNorm<Eigen::Infinity>()<<','<<eig.eigenvalues().minCoeff()<<','
     <<eig.eigenvalues().maxCoeff()<<'\n';
  if(scaled.norm()>0) {
    const Vector direction=-(scale.asDiagonal()*scaled)/scaled.norm();
    VectorValues d=before.zeroVectors();size_t offset=0;
    for(Key key:ordering){d.at(key)=direction.segment(offset,d.at(key).size());offset+=d.at(key).size();}
    out<<"direction,"<<g.dot(direction)<<','<<direction.norm()<<'\n';
    for(double alpha:{1e-4,1e-3,1e-2,.1,1.}) {
      auto plus=d,minus=d;
      for(auto& kv:plus)kv.second*=alpha;
      for(auto& kv:minus)kv.second*=-alpha;
      const double fp=cost(graph,before.retract(plus),theta);
      const double fm=cost(graph,before.retract(minus),theta);
      out<<"probe,"<<alpha<<','<<fp<<','<<fm<<','<<(fp-fm)/(2*alpha)<<','<<fp-f0<<'\n';
    }
  }
  out<<"repeat_cost,"<<cost(graph,before,theta)<<','<<cost(graph,full,theta)<<'\n';
  factorProbe(stamp,graph,theta,before);
  out<<"done,1\n";
}

inline Decision select(const NonlinearFactorGraph& graph,const Values& theta,
                       const Values& initialized,const Values& proposed,
                       size_t limit=10,double audit_stamp=-1) {
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
    std::vector<std::array<double,4>> trials;
    for(size_t i=0;i<limit;++i) {
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()>5)
        throw std::runtime_error("FL4Q fallback wall budget exceeded");
      lm.iterate();++r.iterations;
      const auto v=lm.values();
      // Independent cost on fresh clones, not LM's mutable smart cache error.
      const double e=cost(graph,v,theta);
      trials.push_back({double(r.iterations),e,lm.error(),lm.lambda()});
      if(e<r.after && e<r.before-tolerance) {r.accepted=v;r.after=e;}
      if(std::abs(previous-e)<=1e-9*(1+std::abs(previous)))break;
      previous=e;
    }
    if(!(r.after<r.before-tolerance)) {
      stoppingProbe(audit_stamp,graph,theta,before,full,lm.values(),r,trials);
      throw std::runtime_error("FL4Q no decreasing current-chart LM step");
    }
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
  auto r=select(graph,theta,initialized,isam.calculateEstimate(),10,stamp);
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
