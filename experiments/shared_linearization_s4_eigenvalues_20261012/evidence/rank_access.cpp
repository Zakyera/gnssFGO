#include "candidate.cpp"
namespace shared_native {
void probeRank(const gtsam::Matrix& h) { positiveDefinite(h); }
}
namespace original_rank {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void finiteSymmetric(const gtsam::Matrix& h) {
  require(h.rows() == h.cols() && h.allFinite(), "nonfinite Gaussian equations");
  require((h-h.transpose()).norm() <= 1e-12 * std::max(1.0,h.norm()),
          "asymmetric Gaussian equations");
}
void positiveDefinite(const gtsam::Matrix& h) {
  finiteSymmetric(h);
  require(h.rows() > 0, "empty native information");
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h);
  require(eigen.info() == Eigen::Success, "eigenvalue failure");
  // Explicit rejection only: never regularize an unobservable covariance query.
  require(eigen.eigenvalues().minCoeff() >
      1e-12 * std::max(1.0, eigen.eigenvalues().maxCoeff()),
      "unobservable or numerically singular native information");
}
}
