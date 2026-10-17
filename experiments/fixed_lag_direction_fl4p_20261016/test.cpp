#include <cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/geometry/StereoCamera.h>
#include <iostream>
#include <unistd.h>
using namespace gtsam;
using symbol_shorthand::X;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
cbs::BPSAM::Params params(){
  cbs::BPSAM::Params p;p.robot_id=static_cast<cbs::AgentId>('k');
  p.use_temporary_cbs_linear_factors=false;
  p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;
  p.sam_params_.relinearizeSkip=1;p.sam_params_.relinearizeThreshold=0.;return p;
}
class Square:public NoiseModelFactor1<double>{
 public:
  Square():NoiseModelFactor1<double>(noiseModel::Isotropic::Sigma(1,1),symbol_shorthand::X(0)){}
  Vector evaluateError(const double& x,OptionalMatrixType H=nullptr)const override{
    if(H)*H=(Matrix(1,1)<<2*x).finished();return (Vector(1)<<x*x-1).finished();
  }
  NonlinearFactor::shared_ptr clone()const override{return std::make_shared<Square>(*this);}
};
std::filesystem::path root;
void direction(){
  cbs::BPSAM b(params());NonlinearFactorGraph g;g.emplace_shared<Square>();
  Values v;v.insert(X(0),.1);b.update(g,v);
  const auto a=fl4p::inspect(b,v,root/"scalar1"),r=fl4p::inspect(b,v,root/"scalar2");
  require(a.cost1>a.cost0&&a.cached_slope<0&&a.fd_slope<0,"overshoot classified incorrectly");
  require(std::abs(a.fd_slope-a.current_slope)<1e-7,"scalar derivative mismatch");
  require(a.fd_slope==r.fd_slope,"repeat mismatch");
  Square f;Values x;x.insert(X(0),2.);
  VectorValues d,s;d.insert(X(0),Vector1(0));s.insert(X(0),Vector1(1));
  const double deriv=(fl4p::cost(f,fl4p::path(x,d,s,1e-6),x,false)-fl4p::cost(f,fl4p::path(x,d,s,-1e-6),x,false))/2e-6;
  require(std::abs(deriv-12)<1e-7,"known uphill derivative");
  auto jf=std::make_shared<JacobianFactor>(X(0),Matrix11::Constant(2),Vector1(1));
  LinearContainerFactor prior(jf);Values anchor,z;anchor.insert(X(0),10.);z.insert(X(0),12.);
  const fl4l::OriginalChartPrior wrapped(prior,anchor);
  require(prior.error(z)==0&&wrapped.error(z)==4.5,"prior cost omitted");
  const double gd=fl4p::modelDerivative(*wrapped.linearize(z),d,s);
  require(std::abs(gd-6)<1e-7,"original chart derivative incorrect");
  std::cout<<"PASS known overshoot, uphill direction, original-prior derivative, repeat\n";
}
struct Output{Values values;std::vector<Matrix> covariance;};
Output chain(bool observe) {
  cbs::IncrementalFixedLagBpsamSmoother s(3.5,params());
  auto noise=noiseModel::Isotropic::Sigma(6,.1);Output result;
  Pose3 step(Rot3::RzRyRx(.005,.01,.02),Point3(.1,.02,-.01)),truth;
  for(size_t k=0;k<10;++k){
    NonlinearFactorGraph graph;Values init;if(k)truth=truth.compose(step);
    init.insert(X(k),truth.retract((Vector6()<<.001,-.002,.003,.01,-.02,.01).finished()));
    if(!k)graph.emplace_shared<PriorFactor<Pose3>>(X(k),Pose3(),noise);
    else graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),step,noise);
    if(k==2){
      cbs::BPSAM::CbsOdometryBelief b;b.source_agent=static_cast<cbs::AgentId>('g');
      b.from_pose_key=X(0);b.to_pose_key=X(1);b.sender_from_pose_key=Symbol('g',0);b.sender_to_pose_key=Symbol('g',1);
      b.measured_from_to=step;b.covariance=Matrix6::Identity()*.0025;b.relax_factor=1;
      require(s.addOdometryBeliefsDetailed({b}).details[0].status==cbs::BPSAM::AddOdometryBeliefStatus::Accepted,"admission");
    }
    s.update(graph,init,{{X(k),double(k)}});
    if(observe){
      Values before=s.calculateEstimate();
      before.update(X(k),before.at<Pose3>(X(k)).retract(Vector6::Constant(.01)));
      const auto r=fl4p::inspect(s.bpsam(),before,root/("chain"+std::to_string(k)));
      require(std::abs(r.current_slope-r.fd_slope)<1e-5*(1+std::abs(r.fd_slope)),"pose/prior current derivative");
    }
    result.values.insert(X(k),s.calculateEstimate().at<Pose3>(X(k)));
    result.covariance.push_back(s.bpsam().marginalCovariance(X(k)));
    if(k>=4)require(!s.calculateEstimate().exists(X(k-4)),"marginalization missing");
  }
  return result;
}
void smart(){
  auto K=std::make_shared<Cal3_S2Stereo>(100,100,0,320,240,.2);
  SmartStereoProjectionParams p;
  fl4p::Smart f(noiseModel::Isotropic::Sigma(3,1),p);
  Values v;Point3 point(.2,.1,5);
  for(size_t i=0;i<3;++i){Pose3 pose(Rot3(),Point3(i*.3,0,0));v.insert(X(i),pose);f.add(StereoCamera(pose,K).project(point),X(i),K);}
  f.error(v);const auto signature=f.fl4pCacheSignature();
  require(f.isValid(),"smart fixture invalid");
  for(bool reset:{false,true}){
    const double e=fl4p::cost(f,v,v,reset);require(e<1e-15,"smart reprojection");
    auto c=fl4p::copy(f,reset);c->linearize(v);
    require(f.fl4pCacheSignature()==signature,"live smart cache mutated");
  }
  std::cout<<"PASS independent smart clone reset/evaluation; original cache unchanged\n";
}
int main(){try{
  unsetenv("FL4P_CAPTURE");unsetenv("FL4M_AUDIT");setenv("FL4M_RETIREMENT","remove",1);
  root=std::filesystem::path("/tmp")/("fl4p_test_"+std::to_string(getpid()));
  require(std::filesystem::create_directory(root),"test directory exists");
  direction();smart();auto a=chain(false),b=chain(true);
  require(a.values.equals(b.values,0),"observer changed poses");
  for(size_t i=0;i<a.covariance.size();++i)require((a.covariance[i]-b.covariance[i]).norm()==0,"observer changed covariance");
  std::cout<<"PASS actual fixed-lag temporary retirement/marginalization; exact state/covariance nonmutation\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
