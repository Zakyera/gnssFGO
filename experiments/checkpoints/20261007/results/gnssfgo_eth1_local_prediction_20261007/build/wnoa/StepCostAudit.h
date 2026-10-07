#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <cstdlib>
namespace step_cost_audit {
inline void Values(const std::filesystem::path& path,const gtsam::Values& a,const gtsam::Values& b) {
  std::ofstream f(path);f<<std::setprecision(17)<<"key,component,before,after,local_delta\n";
  auto d=a.localCoordinates(b);
  for(const auto& [k,v]:d) for(int j=0;j<v.size();++j)
    f<<gtsam::DefaultKeyFormatter(k)<<','<<j<<",,,"<<v[j]<<'\n';
  a.print("audit before"); b.print("audit candidate");
}
inline void Compare(const std::filesystem::path& dir,const gtsam::NonlinearFactorGraph& graph,
                    const gtsam::Values& a,const gtsam::Values& b) {
  std::filesystem::create_directories(dir);
  auto linear=graph.linearize(a);const auto delta=a.localCoordinates(b);
  long double e0=0,e1=0,difference=0,factor_difference=0,absolute=0,prediction=0;
  std::ofstream f(dir/"factor_costs.csv");f<<std::setprecision(21)
    <<"slot,type,old_error,new_error,residual_cost_delta,residual_delta_norm,predicted_delta\n";
  for(size_t i=0;i<graph.size();++i) if(graph[i]) {
    auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[i]);
    if(!nf || !boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(nf->noiseModel()))throw std::runtime_error("non Gaussian audit factor");
    const auto r0=nf->whitenedError(a),r1=nf->whitenedError(b);
    const auto jf=boost::dynamic_pointer_cast<gtsam::JacobianFactor>((*linear)[i]);
    if(!jf)throw std::runtime_error("expected Jacobian factor");
    gtsam::Vector ad=gtsam::Vector::Zero(jf->rows());const auto rhs=jf->getb();
    for(auto it=jf->begin();it!=jf->end();++it)ad.noalias()+=jf->getA(it)*delta.at(*it);
    long double change=0,pred=0;
    for(int j=0;j<r0.size();++j) {
      const long double x=r0[j],y=r1[j];
      e0+=x*x/2;e1+=y*y/2;change+=(y-x)*(y+x)/2;
      pred+=(long double)ad[j]*((long double)ad[j]/2-(long double)rhs[j]);
    }
    difference+=change;absolute+=std::abs(change);prediction+=pred;
    factor_difference+=(long double)nf->error(b)-(long double)nf->error(a);
    f<<i<<','<<typeid(*nf).name()<<','<<nf->error(a)<<','<<nf->error(b)<<','<<change<<','<<(r1-r0).norm()<<','<<pred<<'\n';
  }
  std::ofstream s(dir/"cost_summary.csv");s<<std::setprecision(21)
    <<"naive_before,naive_after,extended_before,extended_after,factorwise_delta,residual_delta,abs_factor_delta_sum,predicted_delta,gradient_before,gradient_after,step_norm\n"
    <<graph.error(a)<<','<<graph.error(b)<<','<<e0<<','<<e1<<','<<factor_difference<<','<<difference<<','<<absolute<<','<<prediction<<','
    <<fgo::offline::NormalizedGradient(graph,a)<<','<<fgo::offline::NormalizedGradient(graph,b)<<','<<delta.norm()<<'\n';
  Values(dir/"state_delta.csv",a,b);
}
inline void Dump(const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& a,const gtsam::Values& b) {
  const char* root=std::getenv("COST_AUDIT_DIR");if(!root)return;
  Compare(std::filesystem::path(root)/"isam_candidate",graph,a,b);
  auto linear=graph.linearize(a);auto delta=linear->optimize(gtsam::EliminateQR);
  Compare(std::filesystem::path(root)/"independent_qr",graph,a,a.retract(delta));
}
}
