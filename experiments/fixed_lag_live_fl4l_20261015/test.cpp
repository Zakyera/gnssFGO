#include "OriginalChartPrior.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/JacobianFactor.h>
#include <iostream>
int main() {
  using namespace gtsam;
  Values a; a.insert(1,Pose3(Rot3::RzRyRx(.1,.2,.3),Point3(1,2,3)));
  a.insert(2,Vector3(.1,.2,.3)); a.insert(3,imuBias::ConstantBias());
  Matrix A(19,15); for(int i=0;i<19;++i) for(int j=0;j<15;++j)
    A(i,j)=std::sin(.7*i+.3*j)+(i==j?3.:0.);
  Vector b=Vector::LinSpaced(19,-.7,.8);
  auto gaussian=std::make_shared<JacobianFactor>(
    std::vector<std::pair<Key,Matrix>>{{1,A.leftCols(6)},{2,A.middleCols(6,3)},{3,A.rightCols(6)}},b);
  LinearContainerFactor p(gaussian); fl4l::OriginalChartPrior prior(p,a);
  VectorValues d; d.insert(1,(Vector6()<<.05,-.07,.03,.2,-.3,.1).finished());
  d.insert(2,Vector3(.1,.3,-.1)); d.insert(3,Vector6::Constant(.01));
  const Values x=a.retract(d); const auto lin=prior.linearize(x);
  double maxerr=0;
  const auto gradient=lin->gradientAtZero();
  for(auto key:prior.keys()) for(size_t j=0;j<x.at(key).dim();++j) {
    VectorValues v; for(auto k:prior.keys()) v.insert(k,Vector::Zero(x.at(k).dim()));
    v.at(key)[j]=1e-6; Values plus=x.retract(v); v.at(key)[j]=-1e-6;
    Values minus=x.retract(v);
    maxerr=std::max(maxerr,std::abs((prior.error(plus)-prior.error(minus))/2e-6-gradient.at(key)[j]));
  }
  const double dense=.5*(A*a.localCoordinates(x).vector()-b).squaredNorm();
  const double diff=std::abs(prior.error(x)-dense);
  VectorValues zero; for(auto key:prior.keys()) zero.insert(key,Vector::Zero(x.at(key).dim()));
  const double constantDiff=std::abs(lin->error(zero)-prior.error(x));
  std::cout << "gradient_max_abs="<<maxerr<<" dense_error="<<diff
            <<" linear_constant_error="<<constantDiff<<std::endl;
  if(maxerr>1e-6||diff>1e-10||constantDiff>1e-10) return 1;
}
