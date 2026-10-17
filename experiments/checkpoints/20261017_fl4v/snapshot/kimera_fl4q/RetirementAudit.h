#pragma once
#include <cbs/bpsam/fixed_lag_objective_audit.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace cbs {
inline bool fl4mRetain() {
  const char* p = std::getenv("FL4M_RETIREMENT");
  if (!p || std::string(p) == "remove") return false;
  if (std::string(p) == "retain") return true;
  throw std::runtime_error("invalid FL4M_RETIREMENT");
}
inline void fl4mAudit(double timestamp,
    const gtsam::NonlinearFactorGraph& live_graph,
    const gtsam::Values& before, const gtsam::Values& after,
    const gtsam::Values& old_theta, const gtsam::Values& current_theta,
    size_t retirements) {
  const char* path = std::getenv("FL4M_AUDIT");
  if (!path) return;
  auto g0 = live_graph.clone(), g1 = live_graph.clone();
  double costs[2] = {0, 0}, imu[2] = {0, 0}, prior[2] = {0, 0};
  size_t npriors = 0;
  for (size_t i = 0; i < g0.size(); ++i) {
    if (!g0[i]) continue;
    const auto* p = dynamic_cast<const gtsam::LinearContainerFactor*>(g0[i].get());
    if (p && !p->hasLinearizationPoint()) {
      ++npriors;
      for (auto k : p->keys())
        if (!old_theta.exists(k) || !current_theta.exists(k) ||
            old_theta.at(k).localCoordinates_(current_theta.at(k)).norm() > 1e-12)
          throw std::runtime_error("FL4m prior chart changed");
    }
    const double e0 = fixedLagFactorErrorInChart(*g0[i], before, old_theta);
    const double e1 = fixedLagFactorErrorInChart(*g1[i], after, old_theta);
    costs[0] += e0; costs[1] += e1;
    if (p) { prior[0] += e0; prior[1] += e1; }
    if (dynamic_cast<const gtsam::ImuFactorT<>*>(g0[i].get())) {
      imu[0] += e0; imu[1] += e1;
    }
  }
  std::ofstream out(path, std::ios::app);
  if (!out) throw std::runtime_error("FL4m audit cannot open");
  out << std::setprecision(17) << timestamp << ',' << fl4mRetain() << ','
      << retirements << ',' << npriors << ',' << costs[0] << ',' << costs[1]
      << ',' << imu[0] << ',' << imu[1] << ',' << prior[0] << ',' << prior[1]
      << '\n';
  if (!out) throw std::runtime_error("FL4m audit write failed");
}
} // namespace cbs
