#pragma once
#include <gtsam/nonlinear/ISAM2.h>
#include <functional>
namespace fl3i_capture {
// Observation only. Never installed in any production solver.
inline thread_local std::function<void(const gtsam::ISAM2&)> before;
}
