// Plumbing test of the observer, not a new broad estimator experiment.
#define main old_fixture_main
#include "test_main_lifecycle.cpp"
#undef main
int main(int argc,char**argv) {
  require(argc==2,"frozen fixture path required");
  Geometry geometry(argv[1]);
  Main m(5.,params());auto observer=shared_native::attach(&m);
  const auto p=PreintegrationParams::MakeSharedU(9.81);
  p->accelerometerCovariance=Matrix3::Identity()*.01;
  p->gyroscopeCovariance=Matrix3::Identity()*.01;
  p->integrationCovariance=Matrix3::Identity()*.001;
  imuBias::ConstantBias bias;
  for(int i=0;i<3;++i) {
    NonlinearFactorGraph f;
    if(!i) {
      f.emplace_shared<PriorFactor<Pose3>>(X(i),Pose3(),noise());
      f.emplace_shared<PriorFactor<Vector3>>(V(i),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.1));
      f.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(i),bias,noise());
    }else {
      f.push_back(geometry.factor(X(i-1),X(i)));
      f.emplace_shared<BetweenFactor<Pose3>>(X(i-1),X(i),Pose3(),noise());
      PreintegratedImuMeasurements pim(p,bias);
      pim.integrateMeasurement(Vector3(0,0,9.81),Vector3::Zero(),.1);
      f.emplace_shared<ImuFactor>(X(i-1),V(i-1),X(i),V(i),B(i-1),pim);
      f.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(i-1),B(i),bias,noise(.01));
    }
    Values v;v.insert(X(i),guess(i));v.insert(V(i),Vector3::Zero().eval());v.insert(B(i),bias);
    observer->admit(f);
    m.update(f,v,{{X(i),.1*i},{V(i),.1*i},{B(i),.1*i}});
  }
  return 0;
}
