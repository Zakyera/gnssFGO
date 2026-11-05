#define main cpk9_unused_main
#include "/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019/test.cpp"
#undef main
int main() try {
  spdlog::set_level(spdlog::level::err);
  auto out = chain(true, false);
  std::cout << std::setprecision(17);
  for (const auto& kv : out.latest)
    std::cout << out.latest.at<Pose3>(kv.key).matrix() << '\n';
  for (const auto& m : out.cov) std::cout << m << '\n';
  return 0;
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n'; return 1;
}
