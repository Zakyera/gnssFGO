#pragma once
#include "ShadowObserver.h"
#include <gtsam/linear/Scatter.h>
#include <cbs/utils/relative_pose_covariance.h>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace shared_native {
inline bool parseProvider(const char* flag) {
  if (!flag || !*flag || std::strcmp(flag,"independent_local")==0) return false;
  if (std::strcmp(flag,"shared_native")==0) return true;
  throw std::runtime_error("unsupported GLIM_SHARED_NATIVE_PROVIDER");
}
inline bool providerEnabled() { return parseProvider(std::getenv("GLIM_SHARED_NATIVE_PROVIDER")); }
struct PublishedCovariance {
  EndpointCovariance endpoints;
  cbs::relative_pose_covariance::Result relative;
};
// Transactional adapter. No access to MAIN marginals or nonlinear factors.
inline Result publicationCovariance(const void* owner, gtsam::Key from, gtsam::Key to,
    const gtsam::Pose3& pf, const gtsam::Pose3& pt, PublishedCovariance& output) noexcept {
  auto observer=lookup(owner);
  try {
    if (!observer || !observer->valid()) throw std::runtime_error("provider history unavailable");
    PublishedCovariance candidate;
    const auto q=observer->query(from,to,pf,pt,candidate.endpoints);
    if (!q.ok) throw std::runtime_error(q.message);
    const gtsam::Matrix info=candidate.endpoints.joint_current.llt().solve(gtsam::Matrix::Identity(12,12));
    gtsam::Scatter scatter;scatter.emplace_back(from,6);scatter.emplace_back(to,6);
    candidate.relative=cbs::relative_pose_covariance::compute(gtsam::JointMarginal(info,scatter),
      from,to,pf,pt,pf.between(pt),cbs::relative_pose_covariance::Mode::SchurRelativeBetween);
    if (candidate.relative.used_jitter || !candidate.relative.covariance.allFinite() ||
        (candidate.relative.covariance-candidate.endpoints.relative).norm()>
          1e-10*std::max(1e-30,candidate.endpoints.relative.norm()))
      throw std::runtime_error("provider extraction/chart mismatch");
    output=std::move(candidate);
    return {true,"shared native publication covariance"};
  } catch(const std::exception& e) {
    if(observer)observer->fail(e.what());
    return {false,e.what()};
  } catch(...) {
    if(observer)observer->fail("unknown provider failure");
    return {false,"unknown provider failure"};
  }
}
} // namespace shared_native
