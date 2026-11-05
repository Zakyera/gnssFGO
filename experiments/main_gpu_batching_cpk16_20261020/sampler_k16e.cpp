// Diagnostic only: run the original sampler repeatedly on one immutable scan,
// reset the identical RNG every call, record selected input indices.
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <fstream>
#include <filesystem>
#include <random>
#include <iostream>
#include <stdexcept>
int main(int argc,char** argv) {
  if(argc!=3) throw std::invalid_argument("inputXYZ outputDirectory");
  std::ifstream f(argv[1],std::ios::binary);
  std::vector<Eigen::Vector4d> pts;
  float xyz[3];
  while(f.read(reinterpret_cast<char*>(xyz),sizeof(xyz)))
    pts.emplace_back(xyz[0],xyz[1],xyz[2],1.);
  if(pts.size()<10000) throw std::runtime_error("unexpected short raw scan");
  auto cloud=std::make_shared<gtsam_points::PointCloudCPU>();
  cloud->add_points(pts);
  std::vector<double> ids(pts.size());
  for(size_t i=0;i<ids.size();++i) ids[i]=i;
  cloud->add_intensities(ids); // pass-through IDs, not used by sampling
  std::filesystem::path out(argv[2]);
  if(!std::filesystem::create_directory(out)) throw std::runtime_error("no overwrite");
  if(!gtsam_points::is_omp_default()) throw std::runtime_error("unexpected backend");
  for(int threads: {2,1}) {
    for(int repeat=0;repeat<6;++repeat) {
      std::mt19937 mt; // same exact standard default state every time
      auto sampled=gtsam_points::randomgrid_sampling(cloud,1.,10000./pts.size(),mt,threads);
      std::ofstream r(out/(std::to_string(threads)+"_"+std::to_string(repeat)+".txt"));
      for(size_t i=0;i<sampled->size();++i) r<<static_cast<size_t>(sampled->intensities[i])<<"\n";
      std::cout<<threads<<" "<<repeat<<" "<<sampled->size()<<"\n";
    }
  }
}
