#include "PriorFailureArchive.h"
#include "Checkpoint.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <unistd.h>

namespace cpv102 {
namespace {
// Only called on preservation failure. A failed archive must never suppress or
// replace the original guard exception. Each attempt owns a fresh0700 directory.
void recordFailure(const cpk9::PriorPreservationError& error,
                   int update, int agent) noexcept {
  try {
    const char* configured=std::getenv("CPV102_PRIOR_FAILURE_DIR");
    if (!configured || !*configured) {
      std::fprintf(stderr,"CPV102 prior archive disabled; %s\n",error.what());
      return;
    }
    const std::filesystem::path root(configured);
    if (!root.is_absolute() || !std::filesystem::is_directory(root))
      throw std::runtime_error("failure archive root must be an existing absolute directory");
    std::string pattern=(root/("cpk9-u"+std::to_string(update)+"-p"+
                              std::to_string(::getpid())+"-XXXXXX")).string();
    if (!::mkdtemp(pattern.data())) throw std::runtime_error("mkdtemp failed");
    const std::filesystem::path directory(pattern);
    const auto wall_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto writeMatrix=[&](const char* name,const gtsam::Matrix& matrix) {
      std::ofstream stream(directory/name);
      stream.exceptions(std::ios::badbit|std::ios::failbit);
      stream<<std::setprecision(17)<<matrix<<'\n';stream.close();
    };
    writeMatrix("original_augmented.txt",error.original);
    writeMatrix("cached_augmented.txt",error.cached);
    writeMatrix("converted_augmented.txt",error.converted);
    {
      std::ofstream stream(directory/"failure.bin",std::ios::binary);
      stream.exceptions(std::ios::badbit|std::ios::failbit);
      cpv36::BinaryOut archive(stream);
      archive<<std::string("CPV102_PRIOR_FAILURE_1_NATIVE_X86_64")
             <<update<<agent<<wall_ns<<error.slot<<error.keys
             <<error.original_cache_error<<error.converted_cache_error
             <<error.original<<error.cached<<error.converted;
      cpv36::saveValues(archive,error.anchor);
      stream.close();
    }
    {
      std::ofstream stream(directory/"metadata.txt");
      stream.exceptions(std::ios::badbit|std::ios::failbit);
      stream<<"CPV102 prior preservation failure; guard remains fatal\n"
            <<"isam_update_count="<<update<<"\nagent_id="<<agent
            <<"\nwall_time_ns="<<wall_ns<<"\n"<<error.what()<<'\n'
            <<"anchor is the exact relevant theta subset; matrices follow listed factor-key order\n"
            <<"binary uses existing CPV36 native-x86_64 Pose3/vector3/bias encoding\n";
      stream.close();
    }
    // Marker is written last and only after every output has closed successfully.
    {
      std::ofstream stream(directory/"complete.marker");
      stream.exceptions(std::ios::badbit|std::ios::failbit);
      stream<<"CPV102 prior failure archive complete\n";stream.close();
    }
    std::fprintf(stderr,"CPV102 prior failure archived at %s\n",pattern.c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr,"CPV102 archive failed; original guard still fatal: %s\n",e.what());
  } catch (...) {
    std::fprintf(stderr,"CPV102 archive failed with unknown error; original guard still fatal\n");
  }
}
}  // namespace

cpk9::Promotion promoteWithFailureArchive(
    gtsam::NonlinearFactorGraph& graph, const gtsam::Values& theta,
    const gtsam::GaussianFactorGraph& cache, gtsam::KeySet& fixed,
    int isam_update_count, int agent_id) {
  try {
    return cpk9::promote(graph,theta,cache,fixed);
  } catch (const cpk9::PriorPreservationError& error) {
    recordFailure(error,isam_update_count,agent_id);
    throw;
  }
}
}  // namespace cpv102
