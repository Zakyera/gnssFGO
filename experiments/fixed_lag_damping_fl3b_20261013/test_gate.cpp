#include "BoundedAdaptive.h"
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <tbb/global_control.h>
#include <iostream>
#include <limits>
using namespace gtsam;
static size_t checks=0;
void Require(bool x,const char* message){++checks;if(!x)throw std::runtime_error(message);}
class Square : public NoiseModelFactor1<double> {
 public:
  Square():NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1.),1){}
  Vector evaluateError(const double& x,boost::optional<Matrix&> H=boost::none)const override {
    if(H)*H=Matrix::Constant(1,1,2*x);return Vector1(x*x-1.);
  }
};
int main(){try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  const auto p=fl3b::Params();
  Require(p.lambdaInitial==1e-4&&p.lambdaFactor==2&&p.lambdaLowerBound==1e-16&&p.lambdaUpperBound==1e32,"policy lambda");
  Require(p.minModelFidelity==1e-3&&p.diagonalDamping&&!p.useFixedLambdaFactor,"adaptive policy");
  Require(p.minDiagonal==1e-6&&p.maxDiagonal==1e32&&p.relativeErrorTol==0&&p.absoluteErrorTol==0,"diagonal/tolerance policy");
  NonlinearFactorGraph graph;const Matrix3 covariance=(Vector3(.2,.4,.8).asDiagonal());
  graph.emplace_shared<PriorFactor<Vector3>>(0,Vector3::Zero(),noiseModel::Gaussian::Covariance(covariance));
  Values stationary;stationary.insert(0,Vector3(0.,0.,0.));
  const auto zero=fl3b::Solve(graph,stationary);
  Require(zero.passed&&zero.attempts==0,"stationary must not solve");
  Values seed;seed.insert(0,Vector3(2.,-4.,8.));const Values original=seed;
  const auto ptr=graph[0];auto initialLinear=graph.linearize(seed)->augmentedHessian();
  auto result=fl3b::Solve(graph,seed);
  Require(result.passed&&result.attempts<=20,"linear solution");
  Require(result.diagnosticValues.at<Vector3>(0).norm()<1e-5,"linear mean");
  Require(seed.equals(original,0)&&graph.size()==1&&graph[0]==ptr,"input mutation");
  Require((graph.linearize(seed)->augmentedHessian()-initialLinear).norm()==0,"initial linearization changed");
  Marginals marginal(graph,result.diagnosticValues,Marginals::QR);
  Require((marginal.marginalCovariance(0)-covariance).norm()<1e-12,"undamped covariance changed");
  LevenbergMarquardtOptimizer damping(graph,seed,p);auto linear=damping.linearize();
  auto damped=damping.buildDampedSystem(*linear,fl3b::SqrtDiagonal(*linear,p));
  Require(damped.size()>linear->size()&&graph.size()==1,"temporary damping separation");
  NonlinearFactorGraph nonlinear;nonlinear.emplace_shared<Square>();Values bad;bad.insert(1,.1);const Values saved=bad;
  auto failed=fl3b::Solve(nonlinear,bad,1);
  Require(!failed.passed&&failed.attempts==1&&failed.rejected==1&&failed.accepted==0,"rejected trial must count");
  // GTSAM scalar Equals uses abs(a-b)<tol, so Equals(...,0) is false even
  // for identical doubles. Use exact scalar equality for mutation checks.
  Require(failed.stop=="budget_exhausted"&&bad.at<double>(1)==saved.at<double>(1),"failed transaction/input");
  auto solved=fl3b::Solve(nonlinear,bad);
  Require(solved.passed&&solved.rejected>0&&solved.accepted>0,"nonlinear rejection fixture convergence");
  Require(solved.attempts==solved.accepted+solved.rejected&&solved.trace.size()==solved.attempts,"trial accounting");
  for(const auto& row:solved.trace) {
    Require(row.costAfter<=row.costBefore,"nonlinear monotonic cost");
    Require(row.accepted||row.costAfter==row.costBefore,"rejection changed values/cost");
  }
  LevenbergMarquardtOptimizer native(nonlinear,bad,p);native.iterate();
  auto one=fl3b::Solve(nonlinear,bad,native.getInnerIterations());
  Require(one.accepted==1&&one.attempts==size_t(native.getInnerIterations()),"native iteration accounting");
  Require(one.diagnosticValues.at<double>(1)==native.values().at<double>(1),"bounded/native values differ");
  Require(one.trace.back().lambdaAfter==native.lambda(),"bounded/native adaptive lambda differ");
  Require(bad.at<double>(1)==saved.at<double>(1),"successful diagnostic mutated input");
  Values invalid;invalid.insert(1,std::numeric_limits<double>::quiet_NaN());
  auto nonfinite=fl3b::Solve(nonlinear,invalid);
  Require(!nonfinite.passed&&nonfinite.attempts==0&&nonfinite.stop.starts_with("exception:"),"nonfinite input accepted");
  Require(std::isnan(invalid.at<double>(1))&&nonlinear.size()==1,"nonfinite mutated caller");
  std::cout<<"PASS "<<checks<<" checks; nonlinear attempts="<<solved.attempts<<" accepted="<<solved.accepted<<" rejected="<<solved.rejected<<'\n';
  return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
