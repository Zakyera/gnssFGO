// Diagnostic interposer only. Forwards every actual call to the frozen DSO.
#include <cuda_runtime.h>
#include "ExactGpuSnapshot.h"
#include "RuntimeProfile.h"
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_derivatives.cuh>
#include <gtsam_points/cuda/kernels/linearized_system.cuh>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <cuda_runtime.h>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>

namespace fs = std::filesystem;
using GPU = gtsam_points::IntegratedVGICPFactorGPU;
using Snap = gtsam_points::ExactGpuSnapshot;
namespace {
thread_local bool probing = false;
std::mutex mutex;
size_t sequence = 0;
void require(bool v, const std::string& s) { if (!v) throw std::runtime_error(s); }
void checked(cudaError_t r) { require(r == cudaSuccess, cudaGetErrorString(r)); }
void matrix(const fs::path& p, const gtsam::Matrix& m) {
  require(m.allFinite(), "nonfinite diagnostic matrix");
  std::ofstream f(p); require(bool(f), "cannot create matrix");
  f << std::setprecision(17) << m.rows() << " " << m.cols() << "\n";
  for (int i=0;i<m.rows();++i) {
    for (int j=0;j<m.cols();++j) f << m(i,j) << " ";
    f << "\n";
  }
}
template<class T> void device(const fs::path& p, const T* ptr, size_t n) {
  if (!ptr) return;
  std::vector<unsigned char> bytes(n*sizeof(T));
  checked(cudaMemcpy(bytes.data(), ptr, bytes.size(), cudaMemcpyDeviceToHost));
  std::ofstream f(p, std::ios::binary); require(bool(f), "cannot create device dump");
  f.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
using Fn = gtsam::GaussianFactor::shared_ptr (*)(const GPU*, const gtsam::Values&);
Fn original() {
  static Fn fn = reinterpret_cast<Fn>(dlsym(RTLD_NEXT,
    "_ZNK12gtsam_points24IntegratedVGICPFactorGPU9linearizeERKN5gtsam6ValuesE"));
  require(fn != nullptr, "cannot resolve frozen GPU::linearize");
  return fn;
}
}
namespace gtsam_points {
void ExactGpuSnapshot::discardDiagnosticPending(IntegratedVGICPFactorGPU& f) {
  f.linearization_result.reset();
}
void ExactGpuSnapshot::dumpDiagnostic(const IntegratedVGICPFactorGPU& f,
                                     const std::string& directory) {
  checked(cudaDeviceSynchronize());
  fs::path p(directory); require(fs::create_directory(p), "dump already exists");
  const auto& c=*f.source; const auto& v=*f.target; const auto& d=*f.derivatives;
  std::ofstream info(p/"metadata.txt");
  info << std::setprecision(17)
       << "points " << c.size() << "\nresolution " << v.voxelmap_info.voxel_resolution
       << "\nvoxels " << v.voxelmap_info.num_voxels
       << "\nbuckets " << v.voxelmap_info.num_buckets
       << "\ninliers " << d.num_inliers << "\nhas_inlier_buffer " << bool(d.source_inliers)
       << "\npending " << bool(f.linearization_result) << "\nlinearized " << f.linearized
       << "\nsurface " << d.enable_surface_validation << "\noffloading " << d.enable_offloading
       << "\ntrans_threshold " << d.inlier_update_thresh_trans
       << "\nangle_threshold " << d.inlier_update_thresh_angle << "\n";
  device(p/"source_points.bin",c.points_gpu,c.size());
  device(p/"source_covs.bin",c.covs_gpu,c.size());
  device(p/"source_normals.bin",c.normals_gpu,c.size());
  device(p/"source_times.bin",c.times_gpu,c.size());
  device(p/"source_intensities.bin",c.intensities_gpu,c.size());
  device(p/"voxel_info.bin",v.voxelmap_info_ptr,1);
  device(p/"voxel_buckets.bin",v.buckets,v.voxelmap_info.num_buckets);
  device(p/"voxel_counts.bin",v.num_points,v.voxelmap_info.num_voxels);
  device(p/"voxel_means.bin",v.voxel_means,v.voxelmap_info.num_voxels);
  device(p/"voxel_covs.bin",v.voxel_covs,v.voxelmap_info.num_voxels);
  device(p/"voxel_intensities.bin",v.voxel_intensities,v.voxelmap_info.num_voxels);
  if(d.source_inliers) {
    device(p/"inliers.bin",d.source_inliers,d.num_inliers);
    matrix(p/"inlier_anchor.txt",d.inlier_evaluation_point.matrix().cast<double>());
  }
}
gtsam::GaussianFactor::shared_ptr GPU::linearize(const gtsam::Values& values) const {
  const bool selected = !probing && e1b::current &&
    e1b::current->record->context == e1b::Context::Main &&
    keys().size()==2 && keys()[0]==gtsam::Symbol('x',0) &&
    keys()[1]==gtsam::Symbol('x',1) &&
    values.keys().back()==gtsam::Symbol('x',1);
  if (!selected) return original()(this,values);
  std::lock_guard<std::mutex> lock(mutex);
  const char* root=std::getenv("CPK16E_PROBE_ROOT");
  require(root && sequence<16, "missing root or first-update probe bound exceeded");
  auto p=fs::path(root)/("call_"+std::to_string(sequence++));
  require(fs::create_directory(p), "probe call already exists");
  struct Guard {Guard(){probing=true;} ~Guard(){probing=false;}} guard;
  const bool pending=has_cached_linearization();
  Snap::dumpDiagnostic(*this,(p/"actual_before").string());
  for(auto k:keys()) matrix(p/("pose_"+std::to_string(k)+".txt"),
                            values.at<gtsam::Pose3>(k).matrix());
  auto a=Snap::copy(*this), b=Snap::copy(*this);
  auto& ga=static_cast<GPU&>(*a); auto& gb=static_cast<GPU&>(*b);
  // On batched actual calls, independently recompute from the same inlier
  // state: never remove or change the actual pending result.
  Snap::discardDiagnosticPending(ga); Snap::discardDiagnosticPending(gb);
  require(Snap::state(ga)==Snap::state(gb), "independent branch input differs");
  auto ref=original()(&ga,values);
  NonlinearFactorSetGPU batch; batch.add(b);
  batch.linearizePreservingSynchronousVGICP(values);
  auto test=original()(&gb,values);
  const bool state_equal=Snap::state(ga)==Snap::state(gb);
  auto actual=original()(this,values);
  matrix(p/"reference.txt",ref->augmentedInformation());
  matrix(p/"batched.txt",test->augmentedInformation());
  matrix(p/"actual.txt",actual->augmentedInformation());
  Snap::dumpDiagnostic(*this,(p/"actual_after").string());
  std::ofstream receipt(p/"result.json");
  const auto r=ref->augmentedInformation();
  const double diff=(r-test->augmentedInformation()).cwiseAbs().maxCoeff();
  const double live=(r-actual->augmentedInformation()).cwiseAbs().maxCoeff();
  const double bound=1e-8+1e-6*r.cwiseAbs().maxCoeff();
  receipt << std::setprecision(17) << "{\"input_equal\":true,\"state_equal\":"
          << (state_equal?"true":"false") << ",\"actual_was_pending\":"
          << (pending?"true":"false") << ",\"equation_max\":" << diff
          << ",\"actual_reference_max\":" << live << ",\"bound\":" << bound
          << ",\"passed\":" << ((diff<=bound&&live<=bound&&state_equal)?"true":"false") << "}\n";
  receipt.flush();
  // Keep failed equation evidence and original output. This observer must not
  // turn a diagnostic discrepancy into an estimator recovery/fallback.
  return actual;
}
}
