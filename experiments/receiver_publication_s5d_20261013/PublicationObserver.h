#pragma once
// Diagnostic copies only. Never calls a factor, linearizer or solver.
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace s5d {
struct Frame {
  gtsam::Values values;
  gtsam::VectorValues gradient, diagonal;
  double maximum;
};
inline thread_local bool active = false;
inline thread_local std::vector<Frame> frames;
inline bool Selected(size_t index) {
  const char* text = std::getenv("S5D_INDICES");
  if (!text) return false;
  std::istringstream in(text); std::string item;
  while (std::getline(in,item,',')) if (std::stoul(item)==index) return true;
  return false;
}
inline void Capture(const gtsam::Values& values, const gtsam::VectorValues& g,
                    const gtsam::VectorValues& d, double maximum) {
  if (active) frames.push_back({values,g,d,maximum});
}
struct Scope {
  size_t index; int64_t physical; std::string phase, output;
  Scope(size_t i,int64_t p,const std::string& s,const std::string& o)
      : index(i),physical(p),phase(s),output(o) {
    if (active) throw std::runtime_error("nested S5d observer");
    frames.clear(); active=Selected(index);
  }
  ~Scope() { active=false;frames.clear(); }
  void Finish() const {
    if (!active) return;
    active=false;
    const auto dir=std::filesystem::path(output)/("s5d_"+phase+"_"+std::to_string(index));
    if (!std::filesystem::create_directory(dir)) throw std::runtime_error("refuse S5d overwrite");
    std::ofstream states(dir/"states.csv"),gradients(dir/"gradients.csv"),metadata(dir/"frames.csv");
    for(auto* f:{&states,&gradients,&metadata}) { f->exceptions(std::ios::badbit|std::ios::failbit); *f<<std::setprecision(17); }
    metadata<<"frame,publication_index,physical_ns,normalized_gradient,keys\n";
    states<<"frame,state_index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
    gradients<<"frame,key,coordinate,gradient,diagonal,normalized\n";
    using namespace gtsam::symbol_shorthand;
    for(size_t n=0;n<frames.size();++n) {
      const auto& f=frames[n];
      metadata<<n<<','<<index<<','<<physical<<','<<f.maximum<<','<<f.values.size()<<'\n';
      for(size_t j=0;j<=index;++j) {
        const auto p=f.values.at<gtsam::Pose3>(X(j));const auto q=p.rotation().toQuaternion();
        states<<n<<','<<j<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
        for(double v:f.values.at<gtsam::Vector3>(V(j)))states<<','<<v;
        for(double v:f.values.at<gtsam::Vector3>(W(j)))states<<','<<v;
        states<<'\n';
      }
      for(const auto& [key,g]:f.gradient) {
        const auto& d=f.diagonal.at(key);
        for(int j=0;j<g.size();++j) gradients<<n<<','<<gtsam::DefaultKeyFormatter(key)<<','<<j
          <<','<<g[j]<<','<<d[j]<<','<<(d[j]>0?std::abs(g[j])/std::sqrt(d[j]):0.)<<'\n';
      }
    }
  }
};
} // namespace s5d
