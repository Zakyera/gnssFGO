#pragma once
// Read-only update observer. No result is returned to the production solver.
#include "OriginalChartPrior.h"
#include "LinearParityAudit.h"
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace fl4p {
using namespace gtsam;
using Smart = SmartStereoProjectionPoseFactor;
inline std::string family(const NonlinearFactor& f) {
  if (dynamic_cast<const Smart*>(&f)) return "visual_smart";
  return fl4n::family(f);
}
inline int status(const NonlinearFactor& f) {
  const auto* s=dynamic_cast<const Smart*>(&f);
  if(!s)return -1;
  return s->isValid()+2*s->isDegenerate()+4*s->isPointBehindCamera()+
      8*s->isOutlier()+16*s->isFarPoint();
}
inline NonlinearFactor::shared_ptr copy(const NonlinearFactor& f,bool reset) {
  auto result=f.clone();
  if(reset)if(auto* s=dynamic_cast<Smart*>(result.get()))s->fl4pResetCache();
  return result;
}
inline double cost(const NonlinearFactor& f,const Values& x,const Values& theta,
                   bool reset,int* state=nullptr) {
  auto c=copy(f,reset);
  const double e=cbs::fixedLagFactorErrorInChart(*c,x,theta);
  if(state)*state=status(*c);
  return e;
}
inline Values path(const Values& theta,const VectorValues& d0,
                   const VectorValues& s,double alpha) {
  VectorValues d=d0;
  for(auto& kv:d)kv.second+=alpha*s.at(kv.first);
  return theta.retract(d);
}
inline double modelDerivative(const GaussianFactor& f,const VectorValues& d,
                              const VectorValues& s) {
  GaussianFactorGraph graph;graph.push_back(f.clone());
  const Ordering order(f.keys());const auto h=graph.hessian(order);
  return (h.first*d.vector(order)-h.second).dot(s.vector(order));
}
inline void writeMatrix(const std::filesystem::path& p,const Matrix& m) {
  std::ofstream out(p);out<<m.format(Eigen::IOFormat(17,Eigen::DontAlignCols,",","\n"))<<'\n';
  if(!out)throw std::runtime_error("FL4p matrix output failure");
}
struct Result {double cost0=0,cost1=0,cached_slope=0,current_slope=0,fd_slope=0;};
inline Result inspect(const cbs::BPSAM& isam,const Values& before,
                      const std::filesystem::path& dir) {
  if(!std::filesystem::create_directories(dir))throw std::runtime_error("FL4p output already exists");
  const Values theta=isam.getLinearizationPoint(),after=isam.calculateEstimate();
  const auto d0=theta.localCoordinates(before),d1=isam.getDelta();
  VectorValues s=d1;for(auto& kv:s)kv.second-=d0.at(kv.first);
  const auto& live=isam.getFactorsUnsafe();const auto& cache=isam.fl4nLinearCache();
  // Differential of this exact theta-coordinate path in the local chart at X0.
  const double eps=1e-6;
  const auto vp=before.localCoordinates(path(theta,d0,s,eps));
  const auto vm=before.localCoordinates(path(theta,d0,s,-eps));
  VectorValues tangent=vp;
  for(auto& kv:tangent)kv.second=(kv.second-vm.at(kv.first))/(2*eps);
  const VectorValues zero=VectorValues::Zero(tangent);
  const auto keys=theta.keys();const Ordering order(keys);
  std::ofstream factors(dir/"factors.csv"),series(dir/"cost_curve.csv"),keyfile(dir/"keys.txt");
  factors<<std::setprecision(17);series<<std::setprecision(17);
  factors<<"slot,family,keys,reset,cost0,cost1,cached_slope,current_slope,fd_1e5,fd_1e6,status0,status1\n";
  series<<"slot,family,reset,alpha,cost,status\n";
  GaussianFactorGraph current,compact;
  for(auto f:cache)if(f)compact.push_back(f);
  const auto system_before=compact.hessian(order);
  Result result;
  for(size_t i=0;i<live.size();++i) {
    if(!live[i])continue;
    if(i>=cache.size()||!cache[i])throw std::runtime_error("FL4p missing cached factor");
    const auto& f=*live[i];const auto name=family(f);
    std::ostringstream ids;for(auto k:f.keys())ids<<DefaultKeyFormatter(k)<<';';
    const int original_status=status(f);
    const std::string original_cache=dynamic_cast<const Smart*>(&f)?dynamic_cast<const Smart*>(&f)->fl4pCacheSignature():"";
    const double cached=modelDerivative(*cache[i],d0,s);
    for(bool reset:{false,true}) {
      std::map<double,double> costs;
      std::map<double,int> states;
      for(double a:{-1e-2,-1e-3,-1e-4,-1e-5,-1e-6,-1e-7,-1e-8,0.,1e-8,1e-7,1e-6,1e-5,1e-4,1e-3,1e-2,.25,.5,1.}) {
        int st;const double e=cost(f,path(theta,d0,s,a),theta,reset,&st);
        costs[a]=e;states[a]=st;
        series<<i<<','<<name<<','<<reset<<','<<a<<','<<e<<','<<st<<'\n';
      }
      auto local=copy(f,reset);
      GaussianFactor::shared_ptr fresh;
      if(const auto* p=dynamic_cast<const LinearContainerFactor*>(&f))
        fresh=fl4l::OriginalChartPrior(*p,theta).linearize(before);
      else fresh=local->linearize(before);
      if(!fresh)throw std::runtime_error("FL4p null current linearization");
      const double slope=modelDerivative(*fresh,zero,tangent);
      const double fd5=(costs.at(1e-5)-costs.at(-1e-5))/2e-5;
      const double fd6=(costs.at(1e-6)-costs.at(-1e-6))/2e-6;
      factors<<i<<','<<name<<','<<ids.str()<<','<<reset<<','<<costs[0.]<<','<<costs[1.]<<','<<cached<<','<<slope<<','<<fd5<<','<<fd6<<','<<states[0.]<<','<<states[1.]<<'\n';
      if(!reset) {
        current.push_back(fresh);
        result.cost0+=costs[0.];result.cost1+=costs[1.];result.cached_slope+=cached;
        result.current_slope+=slope;result.fd_slope+=fd6;
      }
      if(cost(f,path(theta,d0,s,0.),theta,reset)!=costs[0.])throw std::runtime_error("FL4p nondeterministic objective");
    }
    if(status(f)!=original_status || (dynamic_cast<const Smart*>(&f)&&dynamic_cast<const Smart*>(&f)->fl4pCacheSignature()!=original_cache))
      throw std::runtime_error("FL4p live smart cache changed");
  }
  const auto fresh=current.hessian(order);
  writeMatrix(dir/"cache_H.txt",system_before.first);writeMatrix(dir/"cache_g.txt",system_before.second);
  writeMatrix(dir/"current_H.txt",fresh.first);writeMatrix(dir/"current_g.txt",fresh.second);
  writeMatrix(dir/"d0.txt",d0.vector(order));writeMatrix(dir/"d1.txt",d1.vector(order));
  writeMatrix(dir/"path_tangent_at_before.txt",tangent.vector(order));
  for(auto k:keys)keyfile<<DefaultKeyFormatter(k)<<' '<<theta.at(k).dim()<<' '<<isam.getFixedVariables().count(k)<<'\n';
  // State/cache identities and numeric equations must survive the observer.
  const auto system_after=compact.hessian(order);
  if(!theta.equals(isam.getLinearizationPoint(),1e-14)||!after.equals(isam.calculateEstimate(),1e-14)||
     (system_before.first-system_after.first).norm()!=0||
     (system_before.second-system_after.second).norm()!=0)
    throw std::runtime_error("FL4p observer mutated live solver");
  std::ofstream summary(dir/"summary.txt");summary<<std::setprecision(17);
  summary<<"cost_before="<<result.cost0<<"\ncost_full="<<result.cost1<<"\ncached_slope="<<result.cached_slope
      <<"\ncurrent_slope="<<result.current_slope<<"\nfinite_difference_slope="<<result.fd_slope
      <<"\nfixed_keys="<<isam.getFixedVariables().size()<<"\nkeys="<<keys.size()<<"\ncomplete=1\n";
  if(!summary||!factors||!series||!keyfile)throw std::runtime_error("FL4p write failed");
  return result;
}
inline void capture(double update,const cbs::BPSAM& isam,const Values& before) {
  const char* root=std::getenv("FL4P_CAPTURE");
  if(update==89&&root)inspect(isam,before,std::filesystem::path(root)/"update_89");
}
} // namespace fl4p
