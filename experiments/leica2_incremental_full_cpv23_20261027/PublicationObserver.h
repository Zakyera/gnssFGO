#pragma once
// Read-only snapshot of two poses already in the publication estimate.
// No solver call, relative-factor construction, or state mutation.
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace cpv23 {
inline void ObservePublication(const gtsam::Values& values, size_t event,
                               size_t newest, int64_t origin, int64_t arrival) {
  const char* path=std::getenv("CPV23_PUBLICATION_FILE");
  if(!path)throw std::runtime_error("missing CPV23 publication output path");
  static std::ofstream file;
  if(!file.is_open()) {
    if(std::filesystem::exists(path))throw std::runtime_error("refuse observer overwrite");
    file.exceptions(std::ios::badbit|std::ios::failbit);
    file.open(path);
    file<<"event,newest,origin_ns,physical_ns,availability_ns,index,px,py,pz,qx,qy,qz,qw\n";
    file<<std::setprecision(17);
  }
  for(size_t i=newest?newest-1:0;i<=newest;++i) {
    const auto& pose=values.at<gtsam::Pose3>(gtsam::Symbol('x',i));
    const auto q=pose.rotation().toQuaternion();
    file<<event<<','<<newest<<','<<origin<<','<<(origin+int64_t(newest)*100000000)
        <<','<<arrival<<','<<i<<','<<pose.x()<<','<<pose.y()<<','<<pose.z()
        <<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
  }
  file.flush();
}
} // namespace cpv23
