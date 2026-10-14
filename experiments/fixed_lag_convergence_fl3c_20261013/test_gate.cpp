#include "BoundedGN.h"
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <tbb/global_control.h>
#include <iostream>
#include <limits>
using namespace gtsam;
size_t checks=0;
void Require(bool ok,const char* s){++checks;if(!ok)throw std::runtime_error(s);}
class Square : public NoiseModelFactor1<double> {
 public:
  Square():NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1.),1){}
  Vector evaluateError(const double& x,boost::optional<Matrix&> H=boost::none)const override {
    if(H)*H=Matrix::Constant(1,1,2*x);return Vector1(x*x-1.);
  }
};
int main(){try {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  NonlinearFactorGraph graph;const Matrix3 cov=Vector3(.2,.4,.8).asDiagonal();
  graph.emplace_shared<PriorFactor<Vector3>>(0,Vector3(0.,0.,0.),noiseModel::Gaussian::Covariance(cov));
  Values zero;zero.insert(0,Vector3(0.,0.,0.));const auto stationary=fl3c::Solve(graph,zero);
  Require(stationary.passed&&stationary.attempts==0,"stationary must return immediately");
  Values initial;initial.insert(0,Vector3(2.,-4.,8.));const auto copy=initial;const auto factor=graph[0];
  auto sol=fl3c::Solve(graph,initial);
  Require(sol.passed&&sol.attempts==1,"linear solution in one step");
  Require(sol.diagnosticValues.at<Vector3>(0).norm()<1e-12,"linear solution accuracy");
  Require(initial.equals(copy,0)&&graph.size()==1&&graph[0]==factor,"input/graph mutation");
  Marginals marg(graph,sol.diagnosticValues,Marginals::QR);
  Require((marg.marginalCovariance(0)-cov).norm()<1e-12,"covariance changed");
  auto d=graph.linearize(initial)->optimize(EliminateQR);
  Require(sol.diagnosticValues.equals(initial.retract(d),1e-12),"native versus direct QR");
  auto none=fl3c::Solve(graph,initial,0);
  Require(!none.passed&&none.attempts==0&&none.stop=="budget_exhausted","zero budget");
  NonlinearFactorGraph nonlinear;nonlinear.emplace_shared<Square>();Values good;good.insert(1,2.);
  auto one=fl3c::Solve(nonlinear,good,1);
  Require(!one.passed&&one.attempts==1&&one.accepted==1&&one.stop=="budget_exhausted","one-step budget");
  size_t observed=0;auto full=fl3c::Solve(nonlinear,good,200,[&](auto step,const auto& old,const auto& next){
    ++observed;Require(step.attempt==observed,"observer ordering");
    Require(nonlinear.error(next)<=nonlinear.error(old),"nonlinear cost monotonicity");
  });
  Require(full.passed&&full.finalGradient<=1e-5&&full.attempts==observed,"nonlinear convergence");
  Require(good.at<double>(1)==2.,"success mutated input");
  Values bad;bad.insert(1,.1);auto reject=fl3c::Solve(nonlinear,bad);
  Require(!reject.passed&&reject.attempts==1&&reject.accepted==0&&reject.stop=="cost_rejected","original cost guard");
  Require(bad.at<double>(1)==.1&&reject.diagnosticValues.at<double>(1)==.1,"rejection rollback");
  Values nan;nan.insert(1,std::numeric_limits<double>::quiet_NaN());auto failure=fl3c::Solve(nonlinear,nan);
  Require(!failure.passed&&failure.attempts==0&&failure.stop.starts_with("exception:"),"nonfinite input");
  Require(std::isnan(nan.at<double>(1))&&nonlinear.size()==1,"nonfinite mutation");
  std::cout<<"PASS "<<checks<<" checks; nonlinear attempts="<<full.attempts<<'\n';return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
