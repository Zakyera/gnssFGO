#define main unused_cpk9_main
#include "/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019/test.cpp"
#undef main
int main() {
  mode(true);setenv("CPV40_REFRESH","1",1);setenv("CPV38_OFF_REFRESH","0",1);
  setenv("FL4M_AUDIT","/dev/null",1);unsetenv("CPV66_AUDIT_DIR");
  spdlog::set_level(spdlog::level::err);std::cout<<std::setprecision(17);
  for(bool peer:{false,true}) {
  std::cout<<(peer?"ON\n":"OFF\n");
  Fixture f;Smoother s(3.5,params());
  for(int i=0;i<12;++i) {
    if(peer && i>=2)s.addOdometryBeliefsDetailed({f.belief(i-2,i-1)});
    s.update(f.factors(i),f.initial(i),{{X(i),double(i)},{V(i),double(i)},{B(i),double(i)}});
    auto v=s.calculateEstimate();
    for(Key k:v.keys()){std::cout<<k<<'\n';char c=Symbol(k).chr();
      if(c=='x')std::cout<<v.at<Pose3>(k).matrix();
      else if(c=='v')std::cout<<v.at<Vector3>(k);
      else std::cout<<v.at<imuBias::ConstantBias>(k).vector();std::cout<<'\n';}
    std::cout<<s.bpsam().fl4nLinearCache().augmentedHessian()<<'\n'<<GaussianFactorGraph(s.bpsam()).augmentedHessian()<<'\n';
  }
  }
}
