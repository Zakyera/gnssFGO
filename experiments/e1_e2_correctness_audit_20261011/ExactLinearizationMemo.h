#pragma once
// Private opt-in engineering candidate for the frozen immutable X/V/W factors.
// Does NOT change residuals, Jacobians, whitening, factor slots or solver policy.
#include "ExactLinearizationProbe.h" // shared bit-exact signature, not probe calls
#include <atomic>
#include <cstdlib>
#include <gtsam/linear/JacobianFactor.h>

namespace e2_memo {
struct Counts {
  std::atomic<uint64_t> calls{0}, hits{0}, misses{0}, invalidations{0}, failures{0};
  std::atomic<uint64_t> live_entries{0}, peak_entries{0}, live_bytes{0}, peak_bytes{0};
};
inline std::array<Counts,2> counts;
inline bool Enabled() {
  const char* value=std::getenv("E2_EXACT_LINEARIZATION_CACHE");
  return value && std::string(value)=="1";
}
inline void Peak(std::atomic<uint64_t>& peak,uint64_t value) {
  auto old=peak.load();while(old<value && !peak.compare_exchange_weak(old,value)) {}
}

template<class Base,size_t Kind> class MemoFactor : public Base {
 public:
  using Base::Base;
  explicit MemoFactor(const Base& base,bool enabled=Enabled()) : Base(base),enabled_(enabled) {}
  ~MemoFactor() override { Clear(); }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    // Copy only immutable factor data: never cache state or a cached noise model.
    return boost::make_shared<MemoFactor>(static_cast<const Base&>(*this),enabled_);
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override {
    if(!enabled_) return Base::linearize(values);
    std::lock_guard<std::mutex> lock(mutex_);
    auto& c=counts.at(Kind);++c.calls;
    try {
      if(!this->active(values)) { Clear();++c.misses;return Base::linearize(values); }
      auto signature=e2_probe::Signature(this->keys(),values,true);
      if(cached_ && signature==signature_) {
        ++c.hits;
        // GTSAM/callers may modify Gaussian factors: never expose our stored copy.
        return cached_->clone();
      }
      if(cached_)++c.invalidations;
      Clear();++c.misses;
      auto result=Base::linearize(values);
      if(result) {
        auto jac=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(result);
        // The frozen families return JacobianFactors. Do not cache unexpected
        // result types or nonfinite numerical failures; preserve their result.
        if(jac) {
          // This allocates AND whitens. Reuse it for byte accounting instead
          // of repeating that work solely to query the matrix dimensions.
          const auto weighted=jac->augmentedJacobian();
          if(weighted.allFinite()) {
          auto stored=result->clone();
          signature_=std::move(signature);cached_=std::move(stored);
          bytes_=sizeof(uint64_t)*signature_.size()+sizeof(double)*weighted.size();
          Peak(c.peak_entries,++c.live_entries);
          Peak(c.peak_bytes,c.live_bytes.fetch_add(bytes_)+bytes_);
          }
        }
      }
      return result;
    } catch(...) {
      // No failed/mismatched computation can become a valid cached entry.
      Clear();++c.failures;throw;
    }
  }
 private:
  void Clear() const {
    if(cached_) {
      --counts.at(Kind).live_entries;counts.at(Kind).live_bytes.fetch_sub(bytes_);
    }
    cached_.reset();signature_.clear();bytes_=0;
  }
  const bool enabled_{Enabled()};
  mutable std::mutex mutex_;
  mutable std::vector<uint64_t> signature_;
  mutable gtsam::GaussianFactor::shared_ptr cached_;
  mutable uint64_t bytes_{0};
};

inline void Flush(const std::string& directory,size_t index,int64_t physical,const std::string& phase) {
  const auto path=std::filesystem::path(directory)/"e2_memo.csv";
  const bool exists=std::filesystem::exists(path);
  std::ofstream out(path,std::ios::app);
  if(!exists)out<<"phase,index,physical_ns,kind,calls,hits,misses,invalidations,failures,live_entries,peak_entries,live_matrix_signature_bytes,peak_matrix_signature_bytes\n";
  for(size_t i=0;i<counts.size();++i) {
    const auto& c=counts[i];
    out<<phase<<','<<index<<','<<physical<<','<<i<<','<<c.calls<<','<<c.hits<<','<<c.misses
       <<','<<c.invalidations<<','<<c.failures<<','<<c.live_entries<<','<<c.peak_entries
       <<','<<c.live_bytes<<','<<c.peak_bytes<<'\n';
  }
}
} // namespace e2_memo
