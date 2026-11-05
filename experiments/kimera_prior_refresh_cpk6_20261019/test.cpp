// CP-K6 isolated numerical regression. No estimator integration.
#include "OriginalChartPrior.h"
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <type_traits>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;

void require(bool ok, const std::string& label) {
  if (!ok) throw std::runtime_error(label);
}
void metric(const std::string& label, double value) {
  require(std::isfinite(value), label+" nonfinite");
  std::cout << label << "=" << std::setprecision(17) << value << '\n';
}
double rel(const Matrix& a, const Matrix& b) {
  return (a-b).norm()/(1+b.norm());
}
Ordering order(std::initializer_list<Key> keys) {
  return Ordering(KeyVector(keys));
}
Vector pack(const VectorValues& v, const Ordering& o) {
  size_t n=0; for (Key k:o) n+=v.at(k).size();
  Vector result(n); size_t p=0;
  for (Key k:o) { result.segment(p,v.at(k).size())=v.at(k); p+=v.at(k).size(); }
  return result;
}
VectorValues unpack(const Values& x, const Ordering& o, const Vector& d) {
  VectorValues result; size_t p=0;
  for (Key k:o) { const size_t n=x.at(k).dim(); result.insert(k,d.segment(p,n)); p+=n; }
  require(p==size_t(d.size()),"unpack dimension"); return result;
}
Values step(const Values& x, const Ordering& o, const Vector& d) {
  return x.retract(unpack(x,o,d));
}
Matrix schur(const Matrix& augmented, size_t n) {
  const Matrix a=augmented.topLeftCorner(n,n);
  const Matrix b=augmented.topRightCorner(n,augmented.cols()-n);
  Eigen::LDLT<Matrix> ldlt(a);
  require(ldlt.info()==Eigen::Success && ldlt.isPositive(),"dense eliminated block SPD");
  return augmented.bottomRightCorner(augmented.rows()-n,augmented.cols()-n)
      -b.transpose()*ldlt.solve(b);
}
Vector gradient(const NonlinearFactorGraph& graph, const Values& x, const Ordering& o) {
  return -graph.linearize(x)->hessian(o).second;
}
Vector numericalGradient(const NonlinearFactorGraph& graph, const Values& x,
                         const Ordering& o, double h) {
  size_t n=0; for (Key k:o) n+=x.at(k).dim();
  Vector result(n);
  for (size_t j=0;j<n;++j) {
    Vector d=Vector::Zero(n); d[j]=h;
    result[j]=(graph.error(step(x,o,d))-graph.error(step(x,o,-d)))/(2*h);
  }
  return result;
}
Matrix chartJacobian(const Values& anchor, const Values& current, const Ordering& o) {
  const Vector d=pack(anchor.localCoordinates(current),o);
  Matrix j(d.size(),d.size());
  for (int i=0;i<d.size();++i) {
    Vector e=Vector::Zero(d.size()); e[i]=1e-6;
    j.col(i)=(pack(anchor.localCoordinates(step(current,o,e)),o)
             -pack(anchor.localCoordinates(step(current,o,-e)),o))/(2e-6);
  }
  return j;
}

int main() try {
  static_assert(std::is_same<DefaultPreintegrationType,ManifoldPreintegration>::value,
                "fixture must use this workspace's manifold IMU implementation");
  auto pp=PreintegrationParams::MakeSharedU(9.81);
  pp->accelerometerCovariance=Matrix3::Identity()*0.01;
  pp->gyroscopeCovariance=Matrix3::Identity()*0.0001;
  pp->integrationCovariance=Matrix3::Identity()*1e-6;
  const imuBias::ConstantBias zeroBias;
  PreintegratedImuMeasurements pim(pp,zeroBias);
  for (int i=0;i<10;++i) pim.integrateMeasurement(Vector3(.3,-.1,9.7),Vector3(.05,.08,-.1),.02);
  Values truth;
  NavState state(Pose3(Rot3::RzRyRx(.1,-.08,.2),Point3(.1,-.2,.3)),Vector3(1,.2,-.1));
  for (int i=0;i<3;++i) {
    truth.insert(X(i),state.pose()); truth.insert(V(i),state.velocity()); truth.insert(B(i),zeroBias);
    state=pim.predict(state,zeroBias);
  }
  const Ordering all=order({X(0),V(0),B(0),X(1),V(1),B(1),X(2),V(2),B(2)});
  const Ordering retired=order({X(0),V(0),B(0)});
  const Ordering nativeOrder=order({X(0),V(0),B(0),X(1),V(1),B(1)});
  const Ordering priorOrder=order({X(1),V(1),B(1)});
  const Ordering survivingOrder=order({X(1),V(1),B(1),X(2),V(2),B(2)});
  Vector initial=Vector::Zero(45);
  for (int i=0;i<45;++i) initial[i]=.003*std::sin(.37*(i+1));
  Values anchor=step(truth,all,initial);
  const Values originalAnchor=anchor;
  NonlinearFactorGraph retiring,surviving;
  retiring.emplace_shared<PriorFactor<Pose3>>(X(0),truth.at<Pose3>(X(0)),noiseModel::Isotropic::Sigma(6,.1));
  retiring.emplace_shared<PriorFactor<Vector3>>(V(0),truth.at<Vector3>(V(0)),noiseModel::Isotropic::Sigma(3,.2));
  retiring.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(0),zeroBias,noiseModel::Isotropic::Sigma(6,.02));
  auto appendInterval=[&](NonlinearFactorGraph& graph,int i) {
    graph.emplace_shared<ImuFactor>(X(i),V(i),X(i+1),V(i+1),B(i),pim);
    graph.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(i),B(i+1),zeroBias,noiseModel::Isotropic::Sigma(6,.01));
  };
  appendInterval(retiring,0); appendInterval(surviving,1);
  Matrix6 l=Matrix6::Identity()*.025;
  l(3,0)=.004; l(4,1)=-.005; l(5,2)=.003; l(4,3)=.006;
  const Matrix6 peerCov=l*l.transpose();
  Vector6 peerShift; peerShift<<.04,-.02,.03,.25,-.1,.05;
  const Pose3 z=truth.at<Pose3>(X(0)).between(truth.at<Pose3>(X(1))).retract(peerShift);
  auto external=std::make_shared<BetweenFactor<Pose3>>(X(0),X(1),z,noiseModel::Gaussian::Covariance(peerCov));
  const Matrix originalRetiring=retiring.linearize(anchor)->augmentedHessian(nativeOrder);
  const Matrix originalSurviving=surviving.linearize(anchor)->augmentedHessian(survivingOrder);
  const double originalExternalError=external->error(anchor);
  const auto gaussian=retiring.linearize(anchor);
  const Matrix dense=schur(gaussian->augmentedHessian(nativeOrder),15);
  const auto eliminated=gaussian->eliminatePartialSequential(retired,EliminateCholesky);
  const Matrix gtsamMarginal=eliminated.second->augmentedHessian(priorOrder);
  metric("native_schur_relative_error",rel(gtsamMarginal,dense));
  require(rel(gtsamMarginal,dense)<1e-8,"native dense Schur mismatch");

  // Exercise actual native iSAM2 remove-before-marginalize with frozen charts.
  // Disabling relinearization is fixture control, NOT a production suggestion.
  ISAM2Params params; params.enableRelinearization=false;
  params.optimizationParams=ISAM2GaussNewtonParams(0.0);
  ISAM2 isam(params);
  NonlinearFactorGraph active=retiring; active.push_back(surviving); active.push_back(external);
  FastMap<Key,int> groups;
  for (int i=0;i<3;++i) { groups[X(i)]=i; groups[V(i)]=i; groups[B(i)]=i; }
  const auto result=isam.update(active,anchor,{},groups);
  const size_t externalSlot=result.newFactorsIndices.back();
  isam.update(NonlinearFactorGraph(),Values(),FactorIndices{externalSlot},groups);
  isam.marginalizeLeaves(FastList<Key>(retired.begin(),retired.end()));
  GaussianFactorGraph actualPriors;
  size_t nativeCount=0;
  for (const auto& f:isam.getFactorsUnsafe()) {
    if (!f) continue;
    require(!dynamic_cast<BetweenFactor<Pose3>*>(f.get()),"external factor survived removal");
    if (const auto* prior=dynamic_cast<LinearContainerFactor*>(f.get())) actualPriors.push_back(prior->factor());
    else ++nativeCount;
  }
  require(nativeCount==2 && actualPriors.size()>0,"unexpected post-marginal graph");
  require(!isam.getLinearizationPoint().exists(X(0)),"state0 not marginalized");
  require(isam.getFixedVariables().size()>0,"no genuine fixed boundary anchor");
  const Matrix actual=actualPriors.augmentedHessian(priorOrder);
  metric("isam_removed_peer_prior_relative_error",rel(actual,dense));
  metric("isam_fixed_boundary_keys",isam.getFixedVariables().size());
  require(rel(actual,dense)<1e-8,"iSAM prior differs from native-only Schur");

  NonlinearFactorGraph contaminated=retiring; contaminated.push_back(external);
  const Matrix inherited=schur(contaminated.linearize(anchor)->augmentedHessian(nativeOrder),15);
  const double inheritedChange=rel(inherited.topLeftCorner(15,15),dense.topLeftCorner(15,15));
  metric("retained_peer_information_change_negative_control",inheritedChange);
  require(inheritedChange>1e-4,"retained-peer negative control not discriminating");

  auto rawPrior=std::make_shared<HessianFactor>(actualPriors);
  const Matrix originalPrior=rawPrior->augmentedInformation();
  LinearContainerFactor unanchored(rawPrior);
  auto preserved=std::make_shared<fl4l::OriginalChartPrior>(unanchored,anchor);
  NonlinearFactorGraph refreshed=surviving; refreshed.push_back(preserved);
  Values boundary=fl4l::subset(anchor,KeyVector(survivingOrder.begin(),survivingOrder.end()));
  const auto stale=surviving.linearize(boundary)->hessian(survivingOrder);
  double worstEnergy=0,worstGradient=0,worstImuGradient=0,maxStale=0,minStale=1e100;
  double resetAnchorGap=0,zeroLimit=0;
  Vector displacement=Vector::Zero(30);
  displacement.segment<6>(0)<<.04,-.03,.02,.20,-.09,.06;
  displacement.segment<3>(6)<<.11,-.08,.04;
  displacement.segment<6>(9)<<.008,-.006,.004,.002,-.001,.003;
  displacement.segment<6>(15)<<-.02,.03,.01,-.07,.05,-.03;
  displacement.segment<3>(21)<<-.05,.04,-.02;
  displacement.segment<6>(24)<<-.005,.003,-.002,-.001,.002,-.002;
  for (double scale:{0.0,.25,.5,1.0,-.75}) {
    const Values current=step(boundary,survivingOrder,scale*displacement);
    VectorValues d;
    for (Key k:rawPrior->keys()) d.insert(k,anchor.at(k).localCoordinates_(current.at(k)));
    const double originalEnergy=rawPrior->error(d);
    const double error=std::abs(preserved->error(current)-originalEnergy)/(1+std::abs(originalEnergy));
    worstEnergy=std::max(worstEnergy,error);
    // The tempting but WRONG operation: preserve coefficients, reset origin.
    fl4l::OriginalChartPrior reset(unanchored,current);
    resetAnchorGap=std::max(resetAnchorGap,std::abs(reset.error(current)-originalEnergy));
    const Vector fresh=gradient(refreshed,current,survivingOrder);
    const Vector fd=numericalGradient(refreshed,current,survivingOrder,1e-6);
    const Vector fd2=numericalGradient(refreshed,current,survivingOrder,5e-7);
    const double discrepancy=std::max(rel(fresh,fd),rel(fresh,fd2));
    worstGradient=std::max(worstGradient,discrepancy);
    const Vector nativeFresh=gradient(surviving,current,survivingOrder);
    worstImuGradient=std::max(worstImuGradient,rel(nativeFresh,numericalGradient(surviving,current,survivingOrder,1e-6)));
    const Vector delta=pack(boundary.localCoordinates(current),survivingOrder);
    const Vector oldSlope=chartJacobian(boundary,current,survivingOrder).transpose()*(stale.first*delta-stale.second);
    const double staleMismatch=rel(oldSlope,nativeFresh);
    metric("fresh_gradient_relative_error_scale_"+std::to_string(scale),discrepancy);
    metric("stale_native_gradient_relative_error_scale_"+std::to_string(scale),staleMismatch);
    if (scale!=0) { maxStale=std::max(maxStale,staleMismatch); minStale=std::min(minStale,staleMismatch); }
    else zeroLimit=staleMismatch;
  }
  metric("max_prior_energy_relative_error",worstEnergy);
  metric("max_fresh_gradient_relative_error",worstGradient);
  metric("max_native_gradient_relative_error",worstImuGradient);
  metric("min_nonzero_stale_gradient_mismatch",minStale);
  metric("max_nonzero_stale_gradient_mismatch",maxStale);
  metric("wrong_anchor_reset_energy_change",resetAnchorGap);
  metric("zero_displacement_stale_gradient_mismatch",zeroLimit);
  require(worstEnergy<1e-10,"prior energy changed");
  require(worstGradient<1e-5 && worstImuGradient<1e-5,"current derivative mismatch");
  require(zeroLimit<1e-8 && minStale>1e-4,"stale vs fresh control failed");
  require(resetAnchorGap>1e-3,"anchor reset negative control failed");
  require(anchor.equals(originalAnchor,0),"anchor mutated");
  require((rawPrior->augmentedInformation()-originalPrior).norm()==0,"prior coefficients mutated");
  require((retiring.linearize(anchor)->augmentedHessian(nativeOrder)-originalRetiring).norm()==0,"retiring native factors mutated");
  require((surviving.linearize(anchor)->augmentedHessian(survivingOrder)-originalSurviving).norm()==0,"surviving native factors mutated");
  require(external->error(anchor)==originalExternalError,"peer mutated");
  std::cout << "PASS: CP-K6 coordinate/information gate only; no production convergence claim\n";
  return 0;
} catch (const std::exception& e) {
  std::cerr<<"FAIL: "<<e.what()<<'\n'; return 1;
}
