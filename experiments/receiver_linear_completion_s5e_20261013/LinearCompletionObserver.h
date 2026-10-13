#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "SourceWnoaModel.h"

namespace s5e {
using Clock=std::chrono::steady_clock;
inline double Seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now()-start).count();
}
// Slicing the memo wrapper removes its mutable cache. Endpoint interpolators
// are const; uncachedClone disconnects their only mutable evaluation cache.
inline gtsam::NonlinearFactorGraph Uncached(const gtsam::NonlinearFactorGraph& in) {
  gtsam::NonlinearFactorGraph out;
  for(const auto& f:in) {
    if(!f){out.push_back(gtsam::NonlinearFactor::shared_ptr());continue;}
    if(const auto* p=dynamic_cast<const fgo::offline::WnoaRelativeFactor*>(f.get()))
      out.push_back(p->uncachedClone());
    else if(const auto* p=dynamic_cast<const fgo::factor::GPWNOAPrior*>(f.get()))
      out.push_back(boost::make_shared<fgo::factor::GPWNOAPrior>(*p));
    else if(dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(f.get()) ||
            dynamic_cast<const gtsam::PriorFactor<gtsam::Vector3>*>(f.get()))
      out.push_back(f->clone());
    else throw std::runtime_error("S5e unsupported shadow factor type");
    if(out.back()->keys()!=f->keys())throw std::runtime_error("S5e clone key mismatch");
  }
  return out;
}
struct Check {
  double error{},gradient{};
  gtsam::VectorValues g,d;
};
inline Check Verify(const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& x) {
  Check c;c.error=graph.error(x);const auto a=graph.linearize(x);
  c.g=a->gradientAtZero();c.d=a->hessianDiagonal();
  if(!std::isfinite(c.error))throw std::runtime_error("S5e nonfinite objective");
  for(const auto& [key,g]:c.g) {
    const auto& d=c.d.at(key);
    if(!g.allFinite()||!d.allFinite()||(d.array()<0).any())throw std::runtime_error("S5e invalid gradient");
    for(int j=0;j<g.size();++j) {
      if(d[j]==0&&g[j]!=0)throw std::runtime_error("S5e zero-information gradient");
      if(d[j]>0)c.gradient=std::max(c.gradient,std::abs(g[j])/std::sqrt(d[j]));
    }
  }return c;
}
inline bool Selected(size_t index,const std::string& phase) {
  if(phase!="support")return false;
  const char* text=std::getenv("S5E_INDICES");if(!text)return false;
  std::istringstream in(text);std::string item;
  while(std::getline(in,item,','))if(std::stoul(item)==index)return true;
  return false;
}
// Own all mutable solver state. No update() is performed on this copy; factor
// pointers shared by ISAM2 are never evaluated here. Verification uses Uncached.
struct Observation {
  size_t index;int64_t physical;std::string output;
  std::unique_ptr<gtsam::ISAM2> solver;
  double copy_seconds{};
  Observation(const gtsam::ISAM2& input,size_t i,int64_t p,
              const std::string& phase,const std::string& out):index(i),physical(p),output(out) {
    if(!Selected(i,phase))return;
    const auto start=Clock::now();solver=std::make_unique<gtsam::ISAM2>(input);copy_seconds=Seconds(start);
  }
  void Finish() {
    if(!solver)return;
    const auto initial=solver->calculateEstimate();
    const auto start=Clock::now();const auto full=solver->calculateBestEstimate();const double solve_seconds=Seconds(start);
    const auto verify_start=Clock::now();const auto graph=Uncached(solver->getFactorsUnsafe());
    const auto a=Verify(graph,initial),b=Verify(graph,full);const double verify_seconds=Seconds(verify_start);
    const auto dir=std::filesystem::path(output)/("s5e_support_"+std::to_string(index));
    if(!std::filesystem::create_directory(dir))throw std::runtime_error("S5e refuses overwrite");
    std::ofstream states(dir/"states.csv"),meta(dir/"frames.csv"),gradient(dir/"gradients.csv");
    for(auto* f:{&states,&meta,&gradient}){f->exceptions(std::ios::badbit|std::ios::failbit);*f<<std::setprecision(17);}
    states<<"frame,state_index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
    meta<<"frame,publication_index,physical_ns,error,normalized_gradient,copy_seconds,full_solve_seconds,verification_seconds\n";
    gradient<<"frame,key,coordinate,gradient,diagonal,normalized\n";
    using namespace gtsam::symbol_shorthand;
    for(int n=0;n<2;++n){const auto& x=n?full:initial;const auto& c=n?b:a;
      meta<<n<<','<<index<<','<<physical<<','<<c.error<<','<<c.gradient<<','<<copy_seconds<<','<<solve_seconds<<','<<verify_seconds<<'\n';
      for(size_t j=0;j<=index;++j){const auto p=x.at<gtsam::Pose3>(X(j));const auto q=p.rotation().toQuaternion();
        states<<n<<','<<j<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
        for(double v:x.at<gtsam::Vector3>(V(j)))states<<','<<v;
        for(double v:x.at<gtsam::Vector3>(W(j)))states<<','<<v;states<<'\n';
      }
      for(const auto& [key,g]:c.g){const auto& d=c.d.at(key);
        for(int j=0;j<g.size();++j)gradient<<n<<','<<gtsam::DefaultKeyFormatter(key)<<','<<j<<','<<g[j]<<','<<d[j]<<','<<(d[j]>0?std::abs(g[j])/std::sqrt(d[j]):0.)<<'\n';
      }
    }
  }
};
} // namespace s5e
