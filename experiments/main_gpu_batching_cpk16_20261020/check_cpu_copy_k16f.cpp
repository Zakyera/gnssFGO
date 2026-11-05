// Prove the observer's zero-tolerance assertion is invalid for native IMU.
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <iostream>
#include <iomanip>
int main() {
  using namespace gtsam;
  using namespace symbol_shorthand;
  auto p=PreintegrationParams::MakeSharedU(9.81);
  p->accelerometerCovariance=Matrix3::Identity()*.01;
  p->gyroscopeCovariance=Matrix3::Identity()*.01;
  p->integrationCovariance=Matrix3::Identity()*.001;
  PreintegratedImuMeasurements pim(p,imuBias::ConstantBias());
  pim.integrateMeasurement(Vector3(.1,.2,9.81),Vector3(.001,.002,.003),.1);
  ImuFactor original(X(0),V(0),X(1),V(1),B(0),pim);
  auto copy=original.clone();
  Values v;
  v.insert(X(0),Pose3());v.insert(X(1),Pose3());
  v.insert(V(0),Vector3::Zero().eval());v.insert(V(1),Vector3::Zero().eval());
  v.insert(B(0),imuBias::ConstantBias());
  auto a=original.linearize(v)->augmentedInformation();
  auto b=copy->linearize(v)->augmentedInformation();
  double diff=(a-b).cwiseAbs().maxCoeff();
  bool self0=original.equals(original,0),clone0=original.equals(*copy,0);
  bool selfpos=original.equals(original,1e-15),clonepos=original.equals(*copy,1e-15);
  std::cout<<std::boolalpha<<std::setprecision(17)
    <<"{\"self_equals_zero\":"<<self0<<",\"clone_equals_zero\":"<<clone0
    <<",\"self_equals_positive\":"<<selfpos<<",\"clone_equals_positive\":"<<clonepos
    <<",\"equation_max\":"<<diff<<"}\n";
  return (!self0&&!clone0&&selfpos&&clonepos&&diff==0)?0:1;
}
