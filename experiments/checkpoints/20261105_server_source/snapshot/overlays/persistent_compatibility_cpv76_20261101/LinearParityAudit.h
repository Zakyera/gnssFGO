#pragma once
#include <cbs/bpsam/bpsam.h>
#include <cbs/bpsam/fixed_lag_objective_audit.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/BetweenFactor.h>
#include <Eigen/Cholesky>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>

namespace fl4n {
using namespace gtsam;
struct Systems {
  Matrix cache_H, tree_H, fresh_H;
  Vector cache_g, tree_g, fresh_g, delta;
  double tree_H_relative=0, tree_g_relative=0;
  double fresh_H_relative=0, fresh_g_relative=0;
  double tree_H_scaled=0, fresh_H_scaled=0;
  double cached_normal_residual=0, fresh_normal_residual=0;
  size_t cliques=0, cache_orphans=0, nonlinear_without_cache=0;
};
inline std::string family(const NonlinearFactor& f) {
  if (dynamic_cast<const LinearContainerFactor*>(&f)) return "prior";
  if (dynamic_cast<const ImuFactorT<>*>(&f)) return "native_imu";
  if (dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(&f)) return "bias_between";
  if (dynamic_cast<const BetweenFactor<Pose3>*>(&f)) return "pose_between";
  return typeid(f).name();
}
inline double normalResidual(const Matrix& H,const Vector& g,const Vector& d) {
  Vector scale=H.diagonal().cwiseAbs().cwiseMax(1e-24).cwiseSqrt();
  return ((H*d-g).cwiseQuotient(scale)).norm()/(1+g.cwiseQuotient(scale).norm());
}
inline Systems inspect(const cbs::BPSAM& isam,
    const GaussianFactorGraph& cached, std::ostream& factors) {
  Systems s;
  const Values& theta=isam.getLinearizationPoint();
  const auto live=isam.getFactorsUnsafe().clone();
  GaussianFactorGraph fresh,tree,compact_cache;
  factors<<std::setprecision(17)<<"slot,family,key_count,cached,fresh,H_relative,g_relative,keys_identical\n";
  for(size_t i=0;i<std::max(live.size(),cached.size());++i) {
    auto n=i<live.size()?live[i]:NonlinearFactor::shared_ptr();
    auto c=i<cached.size()?cached[i]:GaussianFactor::shared_ptr();
    if(c)compact_cache.push_back(c);
    if(!n) {if(c)++s.cache_orphans;continue;}
    auto f=n->linearize(theta);
    if(f)fresh.push_back(f);
    if(!c)++s.nonlinear_without_cache;
    double hd=-1,gd=-1;bool identical=c&&f&&c->keys()==f->keys();
    if(identical&&!f->keys().empty()) {
      GaussianFactorGraph cg,fg;cg.push_back(c);fg.push_back(f);
      const Ordering order(f->keys());
      const auto ch=cg.hessian(order),fh=fg.hessian(order);
      hd=(ch.first-fh.first).norm()/std::max(1.,fh.first.norm());
      gd=(ch.second-fh.second).norm()/std::max(1.,fh.second.norm());
    }
    factors<<i<<','<<family(*n)<<','<<n->size()<<','<<bool(c)<<','<<bool(f)<<','<<hd<<','<<gd<<','<<identical<<'\n';
  }
  std::set<const void*> visited;
  for(const auto& kv:isam.nodes()) if(visited.insert(kv.second.get()).second)
    tree.push_back(kv.second->conditional());
  s.cliques=visited.size();
  const auto keys=compact_cache.keys();
  const Ordering order(KeyVector(keys.begin(),keys.end()));
  if(tree.keys()!=keys || fresh.keys()!=keys) throw std::runtime_error("FL4n system key sets differ");
  const auto C=compact_cache.hessian(order),T=tree.hessian(order),F=fresh.hessian(order);
  s.cache_H=C.first;s.cache_g=C.second;s.tree_H=T.first;s.tree_g=T.second;s.fresh_H=F.first;s.fresh_g=F.second;
  s.delta=isam.getDelta().vector(order);
  for(const Matrix* H:{&s.cache_H,&s.tree_H,&s.fresh_H})
    if(!H->allFinite())throw std::runtime_error("nonfinite Gaussian system");
  if(!s.delta.allFinite()||!s.cache_g.allFinite()||!s.fresh_g.allFinite()||!s.tree_g.allFinite())throw std::runtime_error("nonfinite Gaussian rhs/delta");
  s.tree_H_relative=(s.cache_H-s.tree_H).norm()/std::max(1.,s.cache_H.norm());
  s.tree_g_relative=(s.cache_g-s.tree_g).norm()/std::max(1.,s.cache_g.norm());
  s.fresh_H_relative=(s.cache_H-s.fresh_H).norm()/std::max(1.,s.fresh_H.norm());
  s.fresh_g_relative=(s.cache_g-s.fresh_g).norm()/std::max(1.,s.fresh_g.norm());
  const Vector inv=s.cache_H.diagonal().cwiseAbs().cwiseMax(1e-24).cwiseSqrt().cwiseInverse();
  s.tree_H_scaled=(inv.asDiagonal()*(s.cache_H-s.tree_H)*inv.asDiagonal()).cwiseAbs().maxCoeff();
  s.fresh_H_scaled=(inv.asDiagonal()*(s.cache_H-s.fresh_H)*inv.asDiagonal()).cwiseAbs().maxCoeff();
  s.cached_normal_residual=normalResidual(s.cache_H,s.cache_g,s.delta);
  s.fresh_normal_residual=normalResidual(s.fresh_H,s.fresh_g,s.delta);
  return s;
}
inline void capture(double update,const cbs::BPSAM& isam,
    const std::vector<NonlinearFactor::shared_ptr>& retired) {
  if(update!=28&&update!=89&&update!=438)return;
  const char* root=std::getenv("FL4N_CAPTURE");if(!root)return;
  const auto dir=std::filesystem::path(root)/("update_"+std::to_string(int(update)));
  if(!std::filesystem::create_directories(dir))throw std::runtime_error("refuse existing FL4n capture");
  for(const auto& old:retired)for(const auto& f:isam.getFactorsUnsafe())
    if(old&&f==old)throw std::runtime_error("removed factor identity survived");
  std::ofstream factors(dir/"factors.csv");
  const Systems s=inspect(isam,isam.fl4nLinearCache(),factors);
  std::ofstream out(dir/"summary.txt");out<<std::setprecision(17);
  #define FL4N_VALUE(x) out<<#x<<'='<<s.x<<'\n'
  FL4N_VALUE(cliques);FL4N_VALUE(cache_orphans);FL4N_VALUE(nonlinear_without_cache);
  FL4N_VALUE(tree_H_relative);FL4N_VALUE(tree_g_relative);FL4N_VALUE(tree_H_scaled);
  FL4N_VALUE(fresh_H_relative);FL4N_VALUE(fresh_g_relative);FL4N_VALUE(fresh_H_scaled);
  FL4N_VALUE(cached_normal_residual);FL4N_VALUE(fresh_normal_residual);
  #undef FL4N_VALUE
  out<<"dimension="<<s.delta.size()<<"\nretired_identities_absent="<<retired.size()
     <<"\nfixed_keys="<<isam.getFixedVariables().size()<<'\n';
  const Eigen::IOFormat fmt(17,Eigen::DontAlignCols,",","\n");
  auto matrix=[&](const std::string& n,const Matrix& M){std::ofstream f(dir/n);f<<M.format(fmt)<<'\n';};
  matrix("cache_H.txt",s.cache_H);matrix("tree_H.txt",s.tree_H);matrix("fresh_H.txt",s.fresh_H);
  matrix("cache_g.txt",s.cache_g);matrix("tree_g.txt",s.tree_g);matrix("fresh_g.txt",s.fresh_g);matrix("delta.txt",s.delta);
  const auto keys=isam.fl4nLinearCache().keys();
  std::ofstream keyfile(dir/"keys.txt");
  for(Key k:keys)keyfile<<DefaultKeyFormatter(k)<<' '<<isam.getLinearizationPoint().at(k).dim()<<' '<<isam.getFixedVariables().count(k)<<'\n';
  const Vector inv=s.cache_H.diagonal().cwiseAbs().cwiseMax(1e-24).cwiseSqrt().cwiseInverse();
  const Matrix scaled=inv.asDiagonal()*s.cache_H*inv.asDiagonal();
  Eigen::LDLT<Matrix> solver(scaled);
  if(solver.info()==Eigen::Success) {
    const Vector d=inv.asDiagonal()*solver.solve(inv.asDiagonal()*s.cache_g);
    out<<"dense_normal_residual="<<normalResidual(s.cache_H,s.cache_g,d)<<'\n'
       <<"dense_delta_relative="<<(d-s.delta).norm()/std::max(1.,d.norm())<<'\n';
    matrix("dense_delta.txt",d);
  }
  out<<"complete=1\n";
  if(!out||!factors)throw std::runtime_error("FL4n write failed");
}
} // namespace fl4n
