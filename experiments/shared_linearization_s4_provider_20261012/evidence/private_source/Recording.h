#pragma once
#include "NativeGaussianHistory.h"
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <map>
#include <stdexcept>

namespace shared_native {
// Private diagnostic journal. Numeric payloads are little-endian IEEE doubles
// on the pinned x86_64 build. A stage record is not a successful commit.
class Recording {
 public:
  explicit Recording(const char* path) {
    int fd=::open(path,O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0)throw std::runtime_error("cannot exclusively create native journal");
    file_=::fdopen(fd,"wb");
    if(!file_) { ::close(fd);throw std::runtime_error("native journal fdopen failed"); }
    try { u(0x314a4753);flush(); } catch(...) { std::fclose(file_);file_=nullptr;throw; }
  }
  ~Recording() { if(file_)std::fclose(file_); }
  Recording(const Recording&)=delete;
  Recording& operator=(const Recording&)=delete;
  void stage(const Snapshot& s) {
    const auto start=std::chrono::steady_clock::now();
    tag('S');u(s.epoch);u(s.generation);ids(s.retiring);ids(s.fixed_after);ids(s.removed);
    u(s.theta.size());
    for(const auto& kv:s.theta) {
      u(kv.key);u(kv.value.dim());
      const char c=gtsam::Symbol(kv.key).chr();u(c);
      if(c=='x')mat(s.theta.at<gtsam::Pose3>(kv.key).matrix());
      else if(c=='v')mat(s.theta.at<gtsam::Vector3>(kv.key));
      else if(c=='b')mat(s.theta.at<gtsam::imuBias::ConstantBias>(kv.key).vector());
      else throw std::runtime_error("unhandled native journal Value type");
    }
    u(s.native.size());std::set<Id> keep;
    for(const auto& ref:s.native) {
      const auto& f=s.cache.at(ref.slot);const auto a=f->augmentedInformation();
      u(ref.id);u(ref.version);u(ref.slot);keep.insert(ref.id);
      auto old=blocks_.find(ref.id);
      bool changed=old==blocks_.end() || old->second.first!=f->keys() ||
        old->second.second.rows()!=a.rows() || !old->second.second.isApprox(a,0.);
      u(changed);
      if(changed) { factor(*f,a);blocks_[ref.id]={f->keys(),a}; }
    }
    for(auto it=blocks_.begin();it!=blocks_.end();)
      if(!keep.count(it->first))it=blocks_.erase(it);else ++it;
    flush();io_seconds_+=seconds(start);
  }
  void commit(const NativeGaussianHistory& h) {
    tag('C');u(h.epoch());u(h.admittedCount());u(h.activeCount());u(h.absorbedCount());u(h.removedCount());
    scalar(io_seconds_);flush();
  }
  bool due(Id epoch) const { return !last_query_ || epoch>=last_query_+40; }
  void queryInput(const NativeGaussianHistory& h,gtsam::Key from,gtsam::Key to,
                  const gtsam::Pose3& pf,const gtsam::Pose3& pt) {
    last_query_=h.epoch();tag('P');u(last_query_);u(from);u(to);mat(pf.matrix());mat(pt.matrix());
    const auto theta=h.theta(last_query_);
    mat(theta.at<gtsam::Pose3>(from).localCoordinates(pf));
    mat(theta.at<gtsam::Pose3>(to).localCoordinates(pt));
    const auto graph=h.graph(last_query_);u(graph.size());
    for(const auto& f:graph)factor(*f,f->augmentedInformation());flush();
  }
  void queryOutput(Id epoch,const EndpointCovariance& c,double duration) {
    tag('Q');u(epoch);mat(c.joint_theta);mat(c.transport);mat(c.joint_current);mat(c.relative);
    scalar(duration);flush();
  }
  void failure(Id epoch,const char* message) noexcept {
    try { tag('F');u(epoch);u(std::strlen(message));bytes(message,std::strlen(message));flush(); } catch(...) {}
  }
  static double seconds(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  }
 private:
  void bytes(const void* p,std::size_t n) { if(std::fwrite(p,1,n,file_)!=n)throw std::runtime_error("native journal write failed"); }
  void flush() { if(std::fflush(file_))throw std::runtime_error("native journal flush failed"); }
  void tag(char x) { bytes(&x,1); }
  void u(std::uint64_t x) { bytes(&x,8); }
  void scalar(double x) { bytes(&x,8); }
  template<class T> void ids(const T& values) { u(values.size());for(auto x:values)u(x); }
  void mat(const gtsam::Matrix& m) { u(m.rows());u(m.cols());for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)scalar(m(i,j)); }
  void factor(const gtsam::GaussianFactor& f,const gtsam::Matrix& a) {
    u(f.size());for(auto it=f.begin();it!=f.end();++it){u(*it);u(f.getDim(it));}mat(a);
  }
  FILE* file_=nullptr;
  std::map<Id,std::pair<gtsam::KeyVector,gtsam::Matrix>> blocks_;
  Id last_query_=0;
  double io_seconds_=0;
};
}
