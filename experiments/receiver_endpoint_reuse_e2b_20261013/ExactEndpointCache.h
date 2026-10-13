#pragma once
// Private E2b: exact memoization only. No factor/noise/solver mathematics here.
#include "ExactLinearizationProbe.h"
#include <cfenv>
#include <cstdlib>
#include <list>
#include <memory>
#include <unordered_map>

namespace e2b {
using Key = std::vector<uint64_t>;
struct Hash {
  size_t operator()(const Key& k) const noexcept {
    uint64_t h=1469598103934665603ULL;
    for(auto x:k) { h^=x;h*=1099511628211ULL; }
    return static_cast<size_t>(h);
  }
};
template<class Derived> inline void Append(Key& key,const Eigen::MatrixBase<Derived>& x) {
  // Explicit column-major traversal, independent of the expression's storage.
  key.push_back(x.rows());key.push_back(x.cols());
  for(Eigen::Index c=0;c<x.cols();++c)for(Eigen::Index r=0;r<x.rows();++r)
    key.push_back(std::bit_cast<uint64_t>(static_cast<double>(x(r,c))));
}
struct Value {
  gtsam::Pose3 pose;
  std::vector<gtsam::Matrix> jacobians;
  bool finite() const {
    if(!pose.matrix().allFinite())return false;
    for(const auto& j:jacobians)if(!j.allFinite())return false;
    return true;
  }
  size_t bytes() const {
    size_t n=sizeof(Value);
    for(const auto& j:jacobians)n+=sizeof(gtsam::Matrix)+sizeof(double)*j.size();
    return n;
  }
};
struct Counts {
  uint64_t calls{},hits{},misses{},evaluations{},failures{},nonfinite{},evictions{};
  size_t entries{},peak_entries{},payload_bytes{},peak_payload_bytes{};
};
class Cache {
 public:
  explicit Cache(size_t capacity=8192):capacity_(capacity) { entries_.reserve(capacity); }
  Cache(const Cache&)=delete;
  Cache& operator=(const Cache&)=delete;
  template<class Compute> Value get(Key key,Compute&& compute) {
    {
      std::lock_guard<std::mutex> lock(mutex_);++counts_.calls;
      auto it=entries_.find(key);
      if(it!=entries_.end()) {
        ++counts_.hits;lru_.splice(lru_.begin(),lru_,it->second.position);
        return it->second.value; // Deep copy: callers cannot mutate cache storage.
      }
      ++counts_.misses;++counts_.evaluations;
    }
    Value value;
    try { value=compute(); }
    catch(...) { std::lock_guard<std::mutex> lock(mutex_);++counts_.failures;throw; }
    std::lock_guard<std::mutex> lock(mutex_);
    if(!value.finite()) { ++counts_.nonfinite;return value; }
    if(!capacity_)return value;
    // A concurrent miss may have completed. Its identical key is safe, but
    // return this call's computed value; do not alter the computation's result.
    if(entries_.find(key)!=entries_.end())return value;
    while(entries_.size()>=capacity_) {
      auto old=entries_.find(lru_.back());counts_.payload_bytes-=old->second.bytes;
      entries_.erase(old);lru_.pop_back();++counts_.evictions;
      counts_.entries=entries_.size();
    }
    const size_t bytes=value.bytes()+2*sizeof(uint64_t)*key.size();
    lru_.push_front(key);
    try { entries_.emplace(std::move(key),Entry{value,lru_.begin(),bytes}); }
    catch(...) { lru_.pop_front();throw; }
    counts_.payload_bytes+=bytes;
    counts_.entries=entries_.size();
    counts_.peak_entries=std::max(counts_.peak_entries,counts_.entries);
    counts_.peak_payload_bytes=std::max(counts_.peak_payload_bytes,counts_.payload_bytes);
    return value;
  }
  Counts counts() const { std::lock_guard<std::mutex> lock(mutex_);return counts_; }
 private:
  struct Entry { Value value; std::list<Key>::iterator position; size_t bytes; };
  const size_t capacity_;
  mutable std::mutex mutex_;
  Counts counts_;
  std::list<Key> lru_;
  std::unordered_map<Key,Entry,Hash> entries_;
};
inline std::shared_ptr<Cache> DefaultCache() {
  // Read once, before endpoint construction. Production defaults remain OFF.
  static const auto cache=[]()->std::shared_ptr<Cache> {
    const char* enabled=std::getenv("E2B_ENDPOINT_CACHE");
    return enabled && std::string(enabled)=="1" ? std::make_shared<Cache>() : nullptr;
  }();
  return cache;
}
inline void Flush(const std::string& directory,size_t index,int64_t physical,const std::string& phase) {
  const auto cache=DefaultCache();if(!cache)return;
  const auto c=cache->counts();
  const auto path=std::filesystem::path(directory)/"e2b_endpoints.csv";
  const bool exists=std::filesystem::exists(path);
  std::ofstream out(path,std::ios::app);
  if(!exists)out<<"phase,index,physical_ns,calls,hits,misses,evaluations,failures,nonfinite,evictions,entries,peak_entries,payload_bytes,peak_payload_bytes\n";
  out<<phase<<','<<index<<','<<physical<<','<<c.calls<<','<<c.hits<<','<<c.misses<<','
     <<c.evaluations<<','<<c.failures<<','<<c.nonfinite<<','<<c.evictions<<','<<c.entries<<','
     <<c.peak_entries<<','<<c.payload_bytes<<','<<c.peak_payload_bytes<<'\n';
}
} // namespace e2b
