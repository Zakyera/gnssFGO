#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <map>
#include <memory>
#include <vector>

namespace gtsam_points {
struct SnapshotGeometry {
  std::map<const PointCloud*,std::pair<PointCloud::ConstPtr,PointCloud::Ptr>> clouds;
  std::map<const GaussianVoxelMapGPU*,
           std::pair<GaussianVoxelMapGPU::ConstPtr,GaussianVoxelMapGPU::Ptr>> maps;
};
// Test-only friend access. Requires serialized/quiescent producer at boundary.
// No change to production clone(), layout, kernels or factor mathematics.
class ExactGpuSnapshot {
 public:
  static gtsam::NonlinearFactor::shared_ptr copy(
      const IntegratedVGICPFactorGPU& source);
  static gtsam::NonlinearFactor::shared_ptr copy(
      const IntegratedVGICPFactorGPU& source,SnapshotGeometry& geometry);
  // Defined, semantically live bytes only: excludes addresses, padding,
  // uninitialized constructor fields and write-only CUDA reduction scratch.
  static std::vector<unsigned char> state(const IntegratedVGICPFactorGPU&);
};
}

namespace e1b4check {
class FactorCopies {
 public:
  gtsam::NonlinearFactor::shared_ptr copy(
      const gtsam::NonlinearFactor::shared_ptr& factor);
  gtsam::NonlinearFactorGraph graph(const gtsam::NonlinearFactorGraph&);
 private:
  gtsam_points::SnapshotGeometry geometry_;
  // Keep original alive so an allocator cannot reuse a key address in this map.
  std::map<const gtsam::NonlinearFactor*,
           std::pair<gtsam::NonlinearFactor::shared_ptr,
                     gtsam::NonlinearFactor::shared_ptr>> copies_;
};
}
