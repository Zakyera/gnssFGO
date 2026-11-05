#include "GravityProjection.h"
#include <gtsam/base/numericalDerivative.h>
#include <Eigen/Eigenvalues>
#include <random>
// Reuse the already reviewed CP-K9 actual fixed-lag/IMU/smart-factor fixture.
// Its old main is compiled but never run; frozen fixture source is not edited.
#define main cpk9_unused_main
#include "/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019/test.cpp"
#undef main

void geometry() {
  unsetenv("CPV9_G2K_GRAVITY_PROJECTION");require(!cpv9::enabled(),"default on");
  setenv("CPV9_G2K_GRAVITY_PROJECTION","0",1);require(!cpv9::enabled(),"zero on");
  setenv("CPV9_G2K_GRAVITY_PROJECTION","bad",1);rejects([]{cpv9::enabled();});
  setenv("CPV9_G2K_GRAVITY_PROJECTION","1",1);
  require(cpv9::selected('k','g')&&!cpv9::selected('g','k')&&!cpv9::selected('k','l'),"direction isolation");
  rejects([]{cpv9::gravity({},{});});
  std::mt19937 rng(20261026);std::normal_distribution<double> normal;
  double jac=0,parity=0,loss=0,invariance=0;
  for(int k=0;k<1000;++k) {
    Matrix6 A;Vector6 a,b,z;
    for(int i=0;i<6;++i){a[i]=normal(rng)*.3;b[i]=normal(rng)*.3;z[i]=normal(rng)*.3;
      for(int j=0;j<6;++j)A(i,j)=normal(rng);}
    const Matrix6 C=A*A.transpose()+.1*Matrix6::Identity();
    const Pose3 Xi=Pose3::Expmap(a),Xj=Pose3::Expmap(b),Z=Pose3::Expmap(z);
    const Vector3 g(.7,-2.1,-9.3);
    const auto M=cpv9::projection(Xi,Z,g);
    const Matrix Cy=cpv9::covariance(C,M);
    cpv9::GravityProjectedBetweenFactor f(X(0),X(1),Z,C,M);
    Matrix Hi,Hj;Vector y=f.evaluateError(Xi,Xj,Hi,Hj);
    BetweenFactor<Pose3> old(X(0),X(1),Z,noiseModel::Gaussian::Covariance(C));
    parity=std::max(parity,(y-M*old.evaluateError(Xi,Xj)).norm());
    std::function<Vector(const Pose3&,const Pose3&)> fn=[&](const Pose3& x,const Pose3& q){return f.evaluateError(x,q,{},{});};
    jac=std::max({jac,(Hi-numericalDerivative21<Vector,Pose3,Pose3>(fn,Xi,Xj,1e-6)).norm(),
      (Hj-numericalDerivative22<Vector,Pose3,Pose3>(fn,Xi,Xj,1e-6)).norm()});
    Matrix6 diff=C.inverse()-M.transpose()*Cy.inverse()*M;
    loss=std::min(loss,Eigen::SelfAdjointEigenSolver<Matrix6>(diff).eigenvalues().minCoeff());
    const Vector6 optimum=C*M.transpose()*Cy.inverse()*y;
    require(std::abs(optimum.dot(C.inverse()*optimum)-y.dot(Cy.inverse()*y))<1e-9,"not Gaussian marginal cost");
    const Pose3 W=Pose3::Expmap(b);
    const auto N=cpv9::projection(W*Xi,Z,W.rotation().rotate(g));
    cpv9::GravityProjectedBetweenFactor rotated(X(0),X(1),Z,C,N);
    invariance=std::max(invariance,(rotated.evaluateError(W*Xi,W*Xj,{},{})-y).norm());
    const auto frozen=f.M();(void)f.evaluateError(W*Xi,Xj,{},{});
    require((f.M()-frozen).norm()==0,"projection changed during evaluation");
    Values values;values.insert(X(0),Xi);values.insert(X(1),Xj);
    require(std::abs(2*f.error(values)-y.dot(Cy.inverse()*y))<1e-9,"wrong noise whitening");
    require(f.clone()->equals(f),"clone identity");
  }
  require(jac<1e-7&&parity<1e-12&&loss>-1e-10&&invariance<1e-12,"geometry gate");
  metric("max_exact_jacobian_error",jac);metric("max_base_residual_projection_error",parity);
  metric("minimum_information_loss_eigenvalue",loss);metric("max_world_frame_invariance_error",invariance);
}
Output projectionChain(bool projection_on,bool peer,bool failure=false) {
  setenv("CPV9_G2K_GRAVITY_PROJECTION",projection_on?"1":"0",1);
  // Fixture chain exercises the real CP-K9 prior-refresh/retirement and LOCAL
  // transaction rollback with smart factors. It is not a real-data capture.
  return chain(true,peer,failure);
}
void actualFactor() {
  mode(true);setenv("CPV9_G2K_GRAVITY_PROJECTION","1",1);
  Fixture f;cbs::BPSAM b(params());NonlinearFactorGraph graph;Values init;
  for(int i=0;i<3;++i){graph.push_back(f.factors(i,false));init.insert(f.initial(i));}
  b.update(graph,init);const auto m=f.belief(1,2);
  require(b.addOdometryBeliefsDetailed({m}).accepted==1,"admission");b.update({},{});
  auto external=b.activeWindowTemporaryCbsOdomFactors();require(external.size()==1,"duplicate factor");
  const auto p=std::dynamic_pointer_cast<cpv9::GravityProjectedBetweenFactor>(external[0]);
  require(p&&p->dim()==4,"actual insertion was not projected");const auto M=p->M();
  const auto expected=cpv9::covariance(m.covariance,M);
  require((std::dynamic_pointer_cast<noiseModel::Gaussian>(p->noiseModel())->covariance()-expected).norm()<1e-12,"actual covariance");
  b.update({},{});require((p->M()-M).norm()==0,"lifetime projection changed");
  require(b.addOdometryBeliefsDetailed({m}).accepted_but_skipped_already_applied==1,"duplicate admitted");
  b.setMarginalizationGraph(cbs::BPSAM::MarginalizationType::LOCAL);
  const Matrix local=b.marginalCovariance(X(2));require(local.allFinite(),"LOCAL unavailable");
  NonlinearFactorGraph native_only;
  for(const auto& factor:b.getFactorsUnsafe())if(factor&&factor!=p)native_only.push_back(factor);
  const Matrix independent=Marginals(native_only,b.calculateEstimate()).marginalCovariance(X(2));
  require((local-independent).norm()<1e-8*std::max(1.,independent.norm()),"LOCAL retained projected factor");
  const auto removals=b.activeWindowTemporaryCbsOdomFactorRemovalsOutsideKeys(KeySet{X(2)});
  require(removals.size()==1,"retirement not tracked");
  cbs::BPSAM::UpdateParams u;u.removeFactorIndices=b.activeWindowTemporaryCbsOdomFactorRemovalSlots(removals);
  b.update({}, {},u);b.commitActiveWindowTemporaryCbsOdomFactorRemovals(removals);
  require(b.activeWindowTemporaryCbsOdomFactorCount()==0,"retirement count");
  for(const auto& factor:b.getFactorsUnsafe())require(!std::dynamic_pointer_cast<cpv9::GravityProjectedBetweenFactor>(factor),"retired factor still present");
}
int main(int argc,char** argv)try {
  std::cout<<std::unitbuf<<std::setprecision(17);spdlog::set_level(spdlog::level::err);
  if(argc>1&&std::string(argv[1])=="default") {
    const auto out=projectionChain(false,true);
    for(const auto& kv:out.latest){const auto v=Pose3::Logmap(out.latest.at<Pose3>(kv.key));std::cout<<kv.key<<' '<<v.transpose()<<'\n';}
    for(const auto& c:out.cov)std::cout<<c<<'\n';return 0;
  }
  geometry();actualFactor();
  const auto off=projectionChain(false,false),off_on=projectionChain(true,false);equal(off,off_on);
  const auto original=projectionChain(false,true),on=projectionChain(true,true);
  const auto repeat=projectionChain(true,true),rollback=projectionChain(true,true,true);
  equal(on,repeat);equal(on,rollback);
  require(!original.latest.equals(on.latest,1e-12),"test treatment has no effect");
  std::cout<<"PASS CPV9 geometry/default-isolation/actual-insertion/retirement/LOCAL-rollback\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
