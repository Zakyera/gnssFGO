#include <cuda_runtime.h>
#include "ExactGpuSnapshot.h"
#include "LocalUpdateBatching.h"
#include <gtsam_points/factors/integrated_vgicp_derivatives.cuh>
#include <gtsam_points/cuda/kernels/linearized_system.cuh>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <cstring>
#include <stdexcept>

namespace {
void checked(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}
template<class T> void deviceCopy(T*& dst, const T* src, size_t n) {
  if (!src || !n) return;
  checked(cudaMalloc(reinterpret_cast<void**>(&dst), n*sizeof(T)));
  checked(cudaMemcpy(dst, src, n*sizeof(T), cudaMemcpyDeviceToDevice));
}
// These buffers are read-only to VGICP. Copy exact device bytes, not CPU
// doubles or a new voxel insertion (which can choose a different hash layout).
gtsam_points::PointCloudGPU::Ptr geometry(
    const gtsam_points::PointCloud& p) {
  if (!p.points_gpu || !p.covs_gpu || !p.size())
    throw std::invalid_argument("snapshot needs resident points/covariances");
  auto out = std::make_shared<gtsam_points::PointCloudGPU>();
  out->num_points = p.size();
  deviceCopy(out->points_gpu,p.points_gpu,p.size());
  deviceCopy(out->covs_gpu,p.covs_gpu,p.size());
  deviceCopy(out->normals_gpu,p.normals_gpu,p.size());
  deviceCopy(out->times_gpu,p.times_gpu,p.size());
  deviceCopy(out->intensities_gpu,p.intensities_gpu,p.size());
  return out;
}
gtsam_points::GaussianVoxelMapGPU::Ptr geometry(
    const gtsam_points::GaussianVoxelMapGPU& p) {
  if (!p.loaded_on_gpu() || !p.voxelmap_info_ptr)
    throw std::invalid_argument("snapshot needs resident voxel geometry");
  checked(cudaStreamSynchronize(p.stream));
  auto out=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(
      p.voxelmap_info.voxel_resolution,p.init_num_buckets,
      p.voxelmap_info.max_bucket_scan_count,p.target_points_drop_rate);
  checked(cudaStreamSynchronize(out->stream));
  out->voxelmap_info=p.voxelmap_info;
  checked(cudaMemcpy(out->voxelmap_info_ptr,p.voxelmap_info_ptr,
                     sizeof(gtsam_points::VoxelMapInfo),cudaMemcpyDeviceToDevice));
  deviceCopy(out->buckets,p.buckets,p.voxelmap_info.num_buckets);
  deviceCopy(out->num_points,p.num_points,p.voxelmap_info.num_voxels);
  deviceCopy(out->voxel_means,p.voxel_means,p.voxelmap_info.num_voxels);
  deviceCopy(out->voxel_covs,p.voxel_covs,p.voxelmap_info.num_voxels);
  deviceCopy(out->voxel_intensities,p.voxel_intensities,p.voxelmap_info.num_voxels);
  return out;
}
struct OwnedFactor {
  std::shared_ptr<gtsam_points::IntegratedVGICPFactorGPU> factor;
  Eigen::Isometry3f* anchor=nullptr;
  ~OwnedFactor() {
    // Factor must finish before its optional inlier-evaluation anchor is freed.
    if (factor) factor->sync();
    factor.reset();
    if (anchor) cudaFree(anchor);
  }
};
template<class T> void append(std::vector<unsigned char>& bytes,const T& value) {
  const auto* ptr=reinterpret_cast<const unsigned char*>(&value);
  bytes.insert(bytes.end(),ptr,ptr+sizeof(T));
}
template<class M> void matrix(std::vector<unsigned char>& bytes,const M& m) {
  for (int j=0;j<m.cols();++j) for(int i=0;i<m.rows();++i) append(bytes,m(i,j));
}
template<class T> void deviceBytes(std::vector<unsigned char>& bytes,
                                  const T* data,size_t n) {
  append(bytes,bool(data));
  if (data && n) {
    auto offset=bytes.size();bytes.resize(offset+n*sizeof(T));
    checked(cudaMemcpy(bytes.data()+offset,data,n*sizeof(T),cudaMemcpyDeviceToHost));
  }
}
}

namespace gtsam_points {
gtsam::NonlinearFactor::shared_ptr ExactGpuSnapshot::copy(
    const IntegratedVGICPFactorGPU& src) {
  SnapshotGeometry memo;return copy(src,memo);
}
gtsam::NonlinearFactor::shared_ptr ExactGpuSnapshot::copy(
    const IntegratedVGICPFactorGPU& src,SnapshotGeometry& memo) {
  if(typeid(src)!=typeid(IntegratedVGICPFactorGPU))
    throw std::invalid_argument("unsupported GPU subclass");
  // Explicit diagnostic synchronization, never timed as optimization.
  checked(cudaDeviceSynchronize());
  const auto& d=*src.derivatives;
  if(d.num_inliers<0 || size_t(d.num_inliers)>src.source->size())
    throw std::invalid_argument("invalid inlier count");
  // Geometry is frozen at this checkpoint and read-only during factor
  // evaluation. Deduplicate ONLY within one branch, never across branches.
  auto m=memo.maps.find(src.target.get());
  if(m==memo.maps.end())m=memo.maps.emplace(src.target.get(),
      std::make_pair(src.target,geometry(*src.target))).first;
  auto c=memo.clouds.find(src.source.get());
  if(c==memo.clouds.end())c=memo.clouds.emplace(src.source.get(),
      std::make_pair(src.source,geometry(*src.source))).first;
  auto map=m->second.second;auto cloud=c->second.second;
  if(d.enable_surface_validation && !cloud->normals_gpu)
    throw std::invalid_argument("surface validation without normals");
  auto owner=std::make_shared<OwnedFactor>();
  if(src.is_binary) owner->factor=std::make_shared<IntegratedVGICPFactorGPU>(
      src.keys()[0],src.keys()[1],map,cloud);
  else owner->factor=std::make_shared<IntegratedVGICPFactorGPU>(
      gtsam::Pose3(src.fixed_target_pose.cast<double>().matrix()),
      src.keys()[0],map,cloud);
  auto& dst=*owner->factor;auto& e=*dst.derivatives;
  e.enable_offloading=d.enable_offloading;
  e.enable_surface_validation=d.enable_surface_validation;
  e.inlier_update_thresh_trans=d.inlier_update_thresh_trans;
  e.inlier_update_thresh_angle=d.inlier_update_thresh_angle;
  e.num_inliers=d.num_inliers;
  // Constructor leaves anchor fields uninitialized. Only read them when the
  // inlier buffer proves reset_inliers() has run. Scratch count is write-only.
  e.inlier_evaluation_point_gpu=nullptr;
  if(d.source_inliers) {
    e.inlier_evaluation_point=d.inlier_evaluation_point;
    // Preserve allocated-empty versus not-yet-initialized distinction.
    checked(cudaMalloc(reinterpret_cast<void**>(&e.source_inliers),
                       std::max(1,d.num_inliers)*sizeof(int)));
    if(d.num_inliers) checked(cudaMemcpy(e.source_inliers,d.source_inliers,
        d.num_inliers*sizeof(int),cudaMemcpyDeviceToDevice));
    if(d.inlier_evaluation_point_gpu) {
      // The original pointer can refer to a completed batch's retired scratch.
      // Never dereference it: reset_inliers recorded the identical CPU anchor.
      checked(cudaMalloc(reinterpret_cast<void**>(&owner->anchor),sizeof(Eigen::Isometry3f)));
      checked(cudaMemcpy(owner->anchor,d.inlier_evaluation_point.data(),
                         sizeof(Eigen::Isometry3f),cudaMemcpyHostToDevice));
      e.inlier_evaluation_point_gpu=owner->anchor;
    }
  }
  dst.linearized=src.linearized;
  dst.linearization_point=src.linearization_point;
  dst.evaluation_result=src.evaluation_result;
  if(src.linearization_result)
    dst.linearization_result=std::make_unique<LinearizedSystem6>(*src.linearization_result);
  checked(cudaDeviceSynchronize());
  return gtsam::NonlinearFactor::shared_ptr(owner,owner->factor.get());
}

std::vector<unsigned char> ExactGpuSnapshot::state(const IntegratedVGICPFactorGPU& f) {
  checked(cudaDeviceSynchronize());
  std::vector<unsigned char> b;const auto& d=*f.derivatives;
  append(b,f.is_binary);for(auto key:f.keys())append(b,key);
  matrix(b,f.fixed_target_pose.matrix());append(b,f.linearized);
  matrix(b,f.linearization_point.matrix());append(b,bool(f.evaluation_result));
  if(f.evaluation_result)append(b,*f.evaluation_result);
  append(b,bool(f.linearization_result));
  if(f.linearization_result) {
    const auto& l=*f.linearization_result;
    matrix(b,l.H_target);matrix(b,l.H_target_source);matrix(b,l.H_source);
    matrix(b,l.b_target);matrix(b,l.b_source);append(b,l.error);append(b,l.num_inliers);
  }
  append(b,d.enable_offloading);append(b,d.enable_surface_validation);
  append(b,d.inlier_update_thresh_trans);append(b,d.inlier_update_thresh_angle);
  append(b,d.num_inliers);append(b,bool(d.source_inliers));
  if(d.source_inliers) {
    matrix(b,d.inlier_evaluation_point.matrix());
    append(b,bool(d.inlier_evaluation_point_gpu));
    deviceBytes(b,d.source_inliers,d.num_inliers);
  }
  const auto& p=*f.source;append(b,p.size());
  deviceBytes(b,p.points_gpu,p.size());deviceBytes(b,p.covs_gpu,p.size());
  deviceBytes(b,p.normals_gpu,p.size());deviceBytes(b,p.times_gpu,p.size());
  deviceBytes(b,p.intensities_gpu,p.size());
  const auto& v=*f.target;
  append(b,v.init_num_buckets);append(b,v.target_points_drop_rate);
  append(b,v.voxelmap_info);deviceBytes(b,v.voxelmap_info_ptr,1);
  deviceBytes(b,v.buckets,v.voxelmap_info.num_buckets);
  deviceBytes(b,v.num_points,v.voxelmap_info.num_voxels);
  deviceBytes(b,v.voxel_means,v.voxelmap_info.num_voxels);
  deviceBytes(b,v.voxel_covs,v.voxelmap_info.num_voxels);
  deviceBytes(b,v.voxel_intensities,v.voxelmap_info.num_voxels);
  return b;
}
}

namespace e1b4check {
gtsam::NonlinearFactor::shared_ptr FactorCopies::copy(
    const gtsam::NonlinearFactor::shared_ptr& f) {
  if(!f)return {};
  auto found=copies_.find(f.get());if(found!=copies_.end())return found->second.second;
  gtsam::NonlinearFactor::shared_ptr result;
  if(typeid(*f)==typeid(gtsam_points::IntegratedVGICPFactorGPU)) {
    result=gtsam_points::ExactGpuSnapshot::copy(
        static_cast<const gtsam_points::IntegratedVGICPFactorGPU&>(*f),geometry_);
  } else if(typeid(*f)==typeid(gtsam::LinearContainerFactor)) {
    const auto& l=static_cast<const gtsam::LinearContainerFactor&>(*f);
    result=std::make_shared<gtsam::LinearContainerFactor>(
        l.factor()->clone(),l.linearizationPoint().value_or(gtsam::Values()));
  } else if(typeid(*f)==typeid(gtsam_points::LinearDampingFactor)) {
    // Inherited LinearContainerFactor::clone() erases the derived type.
    const auto& l=static_cast<const gtsam::LinearContainerFactor&>(*f);
    auto damping=std::make_shared<gtsam_points::LinearDampingFactor>();
    static_cast<gtsam::LinearContainerFactor&>(*damping)=gtsam::LinearContainerFactor(
        l.factor()->clone(),l.linearizationPoint().value_or(gtsam::Values()));
    result=damping;
  } else {
    if(!glim::e1b4a::pureCpu(*f))
      throw std::invalid_argument(std::string("unsupported snapshot factor: ")+typeid(*f).name());
    result=f->clone();
  }
  if(!result || result.get()==f.get() || result->keys()!=f->keys() || typeid(*result)!=typeid(*f))
    throw std::logic_error("factor snapshot identity/type failure");
  copies_.emplace(f.get(),std::make_pair(f,result));return result;
}
gtsam::NonlinearFactorGraph FactorCopies::graph(const gtsam::NonlinearFactorGraph& g) {
  gtsam::NonlinearFactorGraph out;out.reserve(g.size());
  for(const auto& f:g)out.push_back(copy(f));return out;
}
}
