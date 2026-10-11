// Read-only numerical-environment probe, NOT a receiver or estimator change.
#include <Eigen/Core>
#include <sched.h>
#include <iostream>
int main() {
  std::cout << "cpu=" << sched_getcpu() << " eigen_l1=" << Eigen::l1CacheSize()
            << " eigen_l2=" << Eigen::l2CacheSize() << " eigen_l3=" << Eigen::l3CacheSize() << '\n';
}
