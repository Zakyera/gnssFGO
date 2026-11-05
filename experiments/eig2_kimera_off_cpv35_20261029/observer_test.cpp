#define main unused_cpk9_test_main
#include "/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019/test.cpp"
#undef main
#include "OffObserver.h"
int main(int argc,char**argv) {
  require(argc==2,"observer output root required");mode(true);
  Fixture f;Smoother base_window(3.5,params()),observed_window(3.5,params());
  auto& baseline=base_window.bpsam();auto& observed=observed_window.bpsam();
  for(int i=0;i<12;++i) {
    auto update=[&](Smoother& s,bool audit) {
      if(audit)setenv("CPV35_AUDIT_DIR",argv[1],1);else unsetenv("CPV35_AUDIT_DIR");
      FixedLagSmoother::KeyTimestampMap stamps{{X(i),double(i)},{V(i),double(i)},{B(i),double(i)}};
      s.update(f.factors(i),f.initial(i),stamps);
    };
    update(base_window,false);update(observed_window,true);
    require(baseline.calculateEstimate().equals(observed.calculateEstimate(),0),"state mutation");
    require(baseline.getLinearizationPoint().equals(observed.getLinearizationPoint(),0),"theta mutation");
    require(baseline.getDelta().equals(observed.getDelta(),0),"delta mutation");
    require(baseline.getFixedVariables()==observed.getFixedVariables(),"lock mutation");
    require((GaussianFactorGraph(baseline).augmentedHessian()-GaussianFactorGraph(observed).augmentedHessian()).norm()==0,"tree mutation");
    require((baseline.fl4nLinearCache().augmentedHessian()-observed.fl4nLinearCache().augmentedHessian()).norm()==0,"cache mutation");
  }
  std::cout<<"PASS 12 OFF updates with visual/IMU/bias/marginal factors; exact state/theta/delta/locks/cache/tree parity\n";
}
