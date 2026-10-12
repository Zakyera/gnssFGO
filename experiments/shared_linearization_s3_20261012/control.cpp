// Baseline equivalence driver: deliberately has no dependency on ShadowObserver.
// Run the same executable against ORIGINAL and private CPU DSOs with mode off.
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <iomanip>
#include <iostream>
using namespace gtsam;
void dump(const gtsam_points::IncrementalFixedLagSmootherExtWithFallback& main) {
  auto estimate=main.calculateEstimate();
  for(auto key:estimate.keys())std::cout<<key<<" pose "<<estimate.at<Pose3>(key).matrix()<<"\n";
  for(auto key:main.getLinearizationPoint().keys())std::cout<<key<<" theta "<<main.getLinearizationPoint().at<Pose3>(key).matrix()<<"\n";
  const auto& factors=main.getLinearFactors();
  for(std::size_t i=0;i<factors.size();++i) {
    std::cout<<"slot "<<i<<" ";
    if(factors[i]) {
      for(auto key:factors[i]->keys())std::cout<<key<<" ";
      std::cout<<factors[i]->augmentedInformation();
    } else std::cout<<"null";
    std::cout<<"\n";
  }
}
int main() {
  std::cout<<std::setprecision(17);ISAM2Params p;p.findUnusedFactorSlots=true;p.relinearizeSkip=1;
  p.setRelinearizeThreshold(.001);gtsam_points::IncrementalFixedLagSmootherExtWithFallback main(2.5,p);
  const auto noise=noiseModel::Isotropic::Sigma(6,.3);
  auto x=[](int i){return Key(Symbol('x',i));};
  auto pose=[](int i){return Pose3(Rot3::Rz(.02*i),Point3(.8*i,.04*i*i,.02*i));};
  for(int i=0;i<18;++i) {
    NonlinearFactorGraph f;Values v;v.insert(x(i),pose(i).retract(Vector6::Constant(.01)));
    if(!i)f.add(PriorFactor<Pose3>(x(0),pose(0),noise));
    else {
      f.add(BetweenFactor<Pose3>(x(i-1),x(i),pose(i-1).between(pose(i)),noise));
      f.add(BetweenFactor<Pose3>(x(i-1),x(i),pose(i-1).between(pose(i)).retract(Vector6::Constant(.003)),noise));
    }
    main.update(f,v,{{x(i),double(i)}});dump(main);main.update();dump(main);
    if(main.fallbackHappened())return 2;
  }
}
