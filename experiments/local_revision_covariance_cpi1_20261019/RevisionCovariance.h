#pragma once
#include "Provider.h"

// CP-I1: covariance routing only. The caller still owns means, IDs, versions,
// retirement, publication and transport. The historical default stays MAIN.
namespace shared_native {
enum class RevisionCovariancePolicy { Main, NativeLocal };

inline RevisionCovariancePolicy parseRevisionCovariance(const char* value) {
  if (!value || !*value || std::strcmp(value, "main") == 0)
    return RevisionCovariancePolicy::Main;
  if (std::strcmp(value, "native_local") == 0)
    return RevisionCovariancePolicy::NativeLocal;
  throw std::runtime_error("unsupported GLIM_REVISION_COVARIANCE_POLICY");
}

inline RevisionCovariancePolicy revisionCovariancePolicy() {
  return parseRevisionCovariance(std::getenv("GLIM_REVISION_COVARIANCE_POLICY"));
}

inline const char* revisionCovarianceName(RevisionCovariancePolicy policy) {
  return policy == RevisionCovariancePolicy::NativeLocal ? "native_local" : "main";
}

inline void validateRevisionCovariance(RevisionCovariancePolicy policy,
                                       bool shared_provider) {
  if (policy == RevisionCovariancePolicy::NativeLocal && !shared_provider)
    throw std::runtime_error("native_local revisions require shared_native provider");
}

template <class MainQuery>
inline cbs::relative_pose_covariance::Result revisionCovariance(
    RevisionCovariancePolicy policy, const void* owner,
    gtsam::Key from, gtsam::Key to, const gtsam::Pose3& pf,
    const gtsam::Pose3& pt, MainQuery&& main_query) {
  if (policy == RevisionCovariancePolicy::Main) return main_query();
  PublishedCovariance local;
  const auto status = publicationCovariance(owner, from, to, pf, pt, local);
  if (!status.ok)
    throw std::runtime_error("SHARED_NATIVE_PROVIDER_FAILURE revision: " + status.message);
  return local.relative;
}
}  // namespace shared_native
