#pragma once
#include "offline_process/BoundedISAM2Refinement.h"
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <algorithm>

namespace failure_probe {
// Read-only; invoked after the publication decision and its timing are recorded.
inline void Run(const std::filesystem::path& dir, const gtsam::NonlinearFactorGraph& graph,
                const gtsam::Values& values) {
  std::filesystem::create_directories(dir);
  const auto linear=graph.linearize(values);
  const auto gradient=linear->gradientAtZero(),diagonal=linear->hessianDiagonal();
  struct Coordinate {gtsam::Key key; int column; double normalized;};
  std::vector<Coordinate> ranked;
  for(const auto& [key,g]:gradient) for(int c=0;c<g.size();++c)
    if(diagonal.at(key)[c]>0)ranked.push_back({key,c,std::abs(g[c])/std::sqrt(diagonal.at(key)[c])});
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b){return a.normalized>b.normalized;});
  std::ofstream states(dir/"simultaneous_values.csv");states<<std::setprecision(17)<<"key,values\n";
  for(auto key:values.keys()) {
    states<<gtsam::DefaultKeyFormatter(key);
    if(gtsam::Symbol(key).chr()=='x') {
      const auto p=values.at<gtsam::Pose3>(key); const auto q=p.rotation().toQuaternion();
      states<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
    } else {for(double x:values.at<gtsam::Vector3>(key))states<<','<<x;}
    states<<'\n';
  }
  std::ofstream factors(dir/"factor_probe.txt");factors<<std::setprecision(17);
  for(size_t slot=0;slot<graph.size();++slot)if(graph[slot]) {
    const auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[slot]);
    const auto noise=boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(nf->noiseModel());
    factors<<slot<<' '<<typeid(*nf).name()<<'\n';
    for(auto key:nf->keys())factors<<gtsam::DefaultKeyFormatter(key)<<' ';
    factors<<"\ncovariance\n"<<noise->covariance()<<"\nresidual\n"<<nf->unwhitenedError(values)<<'\n';
  }
  std::ofstream csv(dir/"gradient_step_sensitivity.csv");csv<<std::setprecision(17)
    <<"rank,key,column,step,native_gradient,native_diagonal,native_normalized,numeric_gradient,numeric_diagonal,numeric_normalized,contributor_count,absolute_gradient_contributions,max_whitened_jacobian_difference\n";
  std::ofstream contrib(dir/"native_gradient_contributions.csv");contrib<<std::setprecision(17)
    <<"rank,key,column,slot,type,gradient_contribution,diagonal_contribution\n";
  for(size_t rank=0;rank<std::min<size_t>(20,ranked.size());++rank) {
    const auto [key,c,norm]=ranked[rank];
    for(double eps:{1e-3,1e-4,3e-5,1e-5,3e-6,1e-6,1e-7}) {
      double ng=0,nd=0,absolute=0,max_difference=0;size_t count=0;
      for(size_t slot=0;slot<graph.size();++slot)if(graph[slot]) {
        const auto& keys=graph[slot]->keys();
        if(std::find(keys.begin(),keys.end(),key)==keys.end())continue;
        const auto nf=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[slot]);
        gtsam::Values local;
        for(auto k:keys)local.insert(k,values.at(k));
        const auto r=nf->whitenedError(local);
        auto delta=local.zeroVectors();delta.at(key)[c]=eps;
        const auto plus=nf->whitenedError(local.retract(delta));delta.at(key)[c]=-eps;
        const auto minus=nf->whitenedError(local.retract(delta));
        const gtsam::Vector j=(plus-minus)/(2*eps);
        const double term=j.dot(r);ng+=term;nd+=j.squaredNorm();absolute+=std::abs(term);++count;
        const auto jf=boost::dynamic_pointer_cast<gtsam::JacobianFactor>((*linear)[slot]);
        auto it=std::find(jf->begin(),jf->end(),key);
        const gtsam::Vector native=jf->getA(it).col(c);
        max_difference=std::max(max_difference,(native-j).cwiseAbs().maxCoeff());
        if(eps==1e-5)contrib<<rank<<','<<gtsam::DefaultKeyFormatter(key)<<','<<c<<','<<slot<<','
          <<typeid(*nf).name()<<','<<-native.dot(jf->getb())<<','<<native.squaredNorm()<<'\n';
      }
      csv<<rank<<','<<gtsam::DefaultKeyFormatter(key)<<','<<c<<','<<eps<<','<<gradient.at(key)[c]<<','
        <<diagonal.at(key)[c]<<','<<norm<<','<<ng<<','<<nd<<','<<std::abs(ng)/std::sqrt(nd)<<','
        <<count<<','<<absolute<<','<<max_difference<<'\n';
    }
  }
  std::ofstream summary(dir/"summary.txt");summary<<std::setprecision(17)
    <<"values="<<values.size()<<" factors="<<graph.nrFactors()<<" slots="<<graph.size()
    <<" objective="<<graph.error(values)<<" maximum_native_gradient="<<ranked.front().normalized<<'\n'
    <<"Read-only top-20 coordinate diagnostic. Not a complete independent stationarity certificate.\n";
}
} // namespace failure_probe
