#pragma once
// Observing-only E2 diagnostic. Every call still executes the original method.
#include <gtsam/config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>

namespace e2_probe {
using Clock = std::chrono::steady_clock;
struct Counts {
  uint64_t calls{}, repeats{}, compute_ns{}, repeat_compute_ns{}, signature_ns{};
};
inline std::array<Counts, 2> counts;
inline std::mutex counts_mutex;
inline void Add(size_t kind, bool repeated, uint64_t elapsed, uint64_t signature) {
  std::lock_guard<std::mutex> lock(counts_mutex);
  auto& c = counts.at(kind);
  ++c.calls; c.repeats += repeated; c.compute_ns += elapsed;
  if (repeated) c.repeat_compute_ns += elapsed;
  c.signature_ns += signature;
}
inline uint64_t Nanos(Clock::time_point start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now()-start).count();
}
inline std::vector<uint64_t> Signature(const gtsam::KeyVector& keys,
                                      const gtsam::Values& values, bool active) {
  std::vector<uint64_t> result; result.reserve(20 * keys.size() + 1);
  result.push_back(active);
  auto append = [&](const auto& matrix) {
    for (int i=0; i<matrix.size(); ++i)
      result.push_back(std::bit_cast<uint64_t>(matrix.data()[i]));
  };
  for (auto key : keys) {
    result.push_back(key);
    const auto kind = gtsam::Symbol(key).chr();
    if (kind == 'x') {
      const auto& pose = values.at<gtsam::Pose3>(key);
#ifdef GTSAM_USE_QUATERNIONS
      append(pose.rotation().toQuaternion().coeffs());
#else
      append(pose.rotation().matrix());
#endif
      append(pose.translation());
    } else if (kind == 'v' || kind == 'w') {
      append(values.at<gtsam::Vector3>(key));
    } else {
      throw std::runtime_error("E2 probe requires frozen X/V/W factor keys");
    }
  }
  return result;
}

template<class Base, size_t Kind> class ObservedFactor : public Base {
 public:
  using Base::Base;
  explicit ObservedFactor(const Base& base) : Base(base) {}
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    // No signature follows a factor clone/version into a new object.
    return boost::make_shared<ObservedFactor>(static_cast<const Base&>(*this));
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto start = Clock::now();
    auto signature = Signature(this->keys(), values, this->active(values));
    const bool repeated = valid_ && signature == last_;
    const auto signature_ns = Nanos(start);
    const auto compute_start = Clock::now();
    auto result = Base::linearize(values); // ALWAYS compute, even for a repeat.
    const auto elapsed = Nanos(compute_start);
    // Only successful calls become the next comparison reference.
    last_ = std::move(signature); valid_ = true;
    Add(Kind, repeated, elapsed, signature_ns);
    return result;
  }
 private:
  mutable std::mutex mutex_;
  mutable std::vector<uint64_t> last_;
  mutable bool valid_{false};
};
inline void Flush(const std::string& directory, size_t index,
                  int64_t physical, const std::string& phase) {
  std::lock_guard<std::mutex> lock(counts_mutex);
  const auto path = std::filesystem::path(directory)/"e2_probe.csv";
  const bool exists = std::filesystem::exists(path);
  std::ofstream out(path, std::ios::app);
  if (!exists) out << "phase,index,physical_ns,kind,calls,repeats,compute_ns,repeat_compute_ns,signature_ns\n";
  for(size_t i=0;i<counts.size();++i) {
    const auto& c=counts[i];
    out << phase << ',' << index << ',' << physical << ',' << i << ',' << c.calls
        << ',' << c.repeats << ',' << c.compute_ns << ',' << c.repeat_compute_ns
        << ',' << c.signature_ns << '\n';
  }
  counts = {};
}
} // namespace e2_probe
