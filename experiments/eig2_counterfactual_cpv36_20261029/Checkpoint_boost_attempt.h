#pragma once
// Private passive capture; never replace or optimize the live graph.
#include "OriginalChartPrior.h"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/base/serialization.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace cpv36 {
using namespace gtsam;
using Smart=SmartStereoProjectionPoseFactor;
inline void require(bool ok,const std::string& msg){if(!ok)throw std::runtime_error("CPV36 "+msg);}
template<class A> void registration(A& a) {
  a.template register_type<noiseModel::Gaussian>();a.template register_type<noiseModel::Diagonal>();
  a.template register_type<noiseModel::Isotropic>();a.template register_type<noiseModel::Unit>();
}
template<class A> void saveValues(A& a,const Values& v) {
  size_t n=v.size();a<<n;
  for(const auto& kv:v){Key k=kv.key;a<<k;char c=Symbol(k).chr();
    if(c=='x')a<<v.at<Pose3>(k);else if(c=='v')a<<v.at<Vector3>(k);
    else if(c=='b')a<<v.at<imuBias::ConstantBias>(k);else require(false,"unexpected state");}
}
template<class A> Values loadValues(A& a) {
  Values v;size_t n;a>>n;for(size_t i=0;i<n;++i){Key k;a>>k;char c=Symbol(k).chr();
    if(c=='x'){Pose3 x;a>>x;v.insert(k,x);}else if(c=='v'){Vector3 x;a>>x;v.insert(k,x);}
    else if(c=='b'){imuBias::ConstantBias x;a>>x;v.insert(k,x);}else require(false,"unknown state");}return v;
}
template<class A> void saveSmart(A& a,const Smart& s) {
  auto params=s.cpv36Params();auto sigma=s.cpv36Sigma();auto body=s.body_P_sensor();
  auto keys=s.keys();auto measured=s.measured();auto cal=s.calibration();
  a<<params<<sigma<<body<<keys;size_t n=keys.size();a<<n;
  for(size_t i=0;i<n;++i)a<<measured[i]<<*cal[i];
  auto point=s.point();int status=point.status;bool valid=bool(point);a<<status<<valid;
  if(valid)a<<*point;auto cache=s.cpv36CachedPoses();a<<cache;
}
template<class A> std::shared_ptr<Smart> loadSmart(A& a) {
  SmartStereoProjectionParams params;double sigma;Pose3 body;KeyVector keys;size_t n;
  a>>params>>sigma>>body>>keys>>n;require(n==keys.size(),"smart keys");
  auto s=std::make_shared<Smart>(noiseModel::Isotropic::Sigma(3,sigma),params,body);
  for(size_t i=0;i<n;++i){StereoPoint2 z;Cal3_S2Stereo c;a>>z>>c;s->add(z,keys[i],std::make_shared<Cal3_S2Stereo>(c));}
  int status;bool valid;a>>status>>valid;auto r=TriangulationResult::Degenerate();
  if(valid){Point3 p;a>>p;r=TriangulationResult(p);}r.status=TriangulationResult::Status(status);
  std::vector<Pose3> cache;a>>cache;s->cpv36RestoreCache(r,cache);return s;
}
inline std::shared_ptr<HessianFactor> hessian(const KeyVector& keys,const std::vector<size_t>& dims,const Matrix& m){
  std::vector<Matrix> blocks;std::vector<Vector> rhs;size_t oi=0;
  for(size_t i=0;i<keys.size();++i){size_t oj=oi;rhs.push_back(m.block(oi,m.cols()-1,dims[i],1));
    for(size_t j=i;j<keys.size();++j){blocks.push_back(m.block(oi,oj,dims[i],dims[j]));oj+=dims[j];}oi+=dims[i];}
  require(oi+1==size_t(m.rows()),"prior dimension");return std::make_shared<HessianFactor>(keys,blocks,rhs,m(m.rows()-1,m.cols()-1));
}
struct Snapshot{Values theta,initial,estimate;NonlinearFactorGraph graph;KeyVector fixed;Matrix cached,tree;};
inline void save(const std::filesystem::path& path,const Snapshot& s) {
  require(!std::filesystem::exists(path),"refuse overwrite");std::ofstream out(path,std::ios::binary);boost::archive::binary_oarchive a(out);registration(a);
  std::string magic="CPV36_1";a<<magic;saveValues(a,s.theta);saveValues(a,s.initial);saveValues(a,s.estimate);a<<s.fixed<<s.cached<<s.tree;
  size_t total=s.graph.size(),n=0;for(auto f:s.graph)if(f)++n;a<<total<<n;
  for(size_t i=0;i<total;++i){auto f=s.graph[i];if(!f)continue;a<<i;
    if(auto p=dynamic_cast<const Smart*>(f.get())){int tag=1;a<<tag;saveSmart(a,*p);}
    else if(auto p=dynamic_cast<const ImuFactor*>(f.get())){int tag=2;a<<tag<<*p;}
    else if(auto p=dynamic_cast<const BetweenFactor<imuBias::ConstantBias>*>(f.get())){int tag=3;a<<tag<<*p;}
    else if(auto p=dynamic_cast<const LinearContainerFactor*>(f.get())){int tag=4;a<<tag;require(!p->hasLinearizationPoint(),"expected native prior");
      KeyVector keys=p->keys();std::vector<size_t> dims;for(auto k=p->factor()->begin();k!=p->factor()->end();++k)dims.push_back(p->factor()->getDim(k));
      Matrix m=p->factor()->augmentedInformation();a<<keys<<dims<<m;}
    else if(auto p=dynamic_cast<const PriorFactor<Pose3>*>(f.get())){int tag=5;a<<tag<<*p;}
    else if(auto p=dynamic_cast<const PriorFactor<Vector3>*>(f.get())){int tag=6;a<<tag<<*p;}
    else if(auto p=dynamic_cast<const PriorFactor<imuBias::ConstantBias>*>(f.get())){int tag=7;a<<tag<<*p;}
    else require(false,std::string("unsupported factor ")+typeid(*f).name());
  }require(bool(out),"checkpoint write");
}
inline Snapshot load(const std::filesystem::path& path) {
  Snapshot s;std::ifstream in(path,std::ios::binary);require(bool(in),"checkpoint missing");boost::archive::binary_iarchive a(in);registration(a);
  std::string magic;a>>magic;require(magic=="CPV36_1","magic");s.theta=loadValues(a);s.initial=loadValues(a);s.estimate=loadValues(a);a>>s.fixed>>s.cached>>s.tree;
  size_t total,n;a>>total>>n;s.graph.resize(total);
  for(size_t i=0;i<n;++i){size_t slot;int tag;a>>slot>>tag;NonlinearFactor::shared_ptr f;
    if(tag==1)f=loadSmart(a);
    else if(tag==2){auto p=std::make_shared<ImuFactor>();a>>*p;f=p;}
    else if(tag==3){auto p=std::make_shared<BetweenFactor<imuBias::ConstantBias>>();a>>*p;f=p;}
    else if(tag==4){KeyVector keys;std::vector<size_t> dims;Matrix m;a>>keys>>dims>>m;f=std::make_shared<LinearContainerFactor>(hessian(keys,dims,m));}
    else if(tag==5){auto p=std::make_shared<PriorFactor<Pose3>>();a>>*p;f=p;}
    else if(tag==6){auto p=std::make_shared<PriorFactor<Vector3>>();a>>*p;f=p;}
    else if(tag==7){auto p=std::make_shared<PriorFactor<imuBias::ConstantBias>>();a>>*p;f=p;}
    else require(false,"unknown factor tag");s.graph[slot]=f;
  }return s;
}
inline NonlinearFactorGraph clone(const NonlinearFactorGraph& g){NonlinearFactorGraph c;c.resize(g.size());for(size_t i=0;i<g.size();++i)if(g[i])c[i]=g[i]->clone();return c;}
inline void parity(const Snapshot& s,const Snapshot& r,std::ostream& out) {
  require(s.theta.equals(r.theta,0)&&s.initial.equals(r.initial,0)&&s.estimate.equals(r.estimate,0),"value roundtrip");
  require(s.fixed==r.fixed && (s.cached-r.cached).norm()==0 && (s.tree-r.tree).norm()==0,"equation roundtrip");
  require(s.graph.size()==r.graph.size(),"slots roundtrip");size_t n=0;double maxerr=0,maxlin=0;
  for(size_t i=0;i<s.graph.size();++i){require(bool(s.graph[i])==bool(r.graph[i]),"slot occupancy");if(!s.graph[i])continue;++n;
    for(const auto* values:{&s.theta,&s.initial,&s.estimate}){
      auto x=s.graph[i]->clone(),y=r.graph[i]->clone();double ex=cbs::fixedLagFactorErrorInChart(*x,*values,s.theta),ey=cbs::fixedLagFactorErrorInChart(*y,*values,r.theta);
      require(std::abs(ex-ey)<=1e-9*(1+std::abs(ex)),"factor error roundtrip");maxerr=std::max(maxerr,std::abs(ex-ey));
      x=s.graph[i]->clone();y=r.graph[i]->clone();auto gx=x->linearize(*values),gy=y->linearize(*values);require(bool(gx)==bool(gy),"linear null roundtrip");
      if(gx){auto mx=gx->augmentedInformation(),my=gy->augmentedInformation();double rel=(mx-my).norm()/(1+mx.norm());require(rel<1e-12,"linear roundtrip");maxlin=std::max(maxlin,rel);}
    }
  }out<<std::setprecision(17)<<"factors,"<<n<<"\nmax_error_difference,"<<maxerr<<"\nmax_relative_linear_difference,"<<maxlin<<'\n';
}
template<class Solver> void capture(double update,const Solver& solver,const Values& initial) {
  const char* root=std::getenv("CPV36_AUDIT_DIR");if(!root)return;int k=int(update);
  if(!std::getenv("CPV36_TEST_ALL") && k!=800&&k!=934&&k!=1000&&k!=1335&&k!=1336&&k!=1337&&k!=1338)return;
  std::filesystem::path d=std::filesystem::path(root)/("update_"+std::to_string(k));require(std::filesystem::create_directories(d),"duplicate snapshot");
  Snapshot s;s.theta=solver.getLinearizationPoint();s.initial=initial;s.estimate=solver.calculateEstimate();s.graph=clone(solver.getFactorsUnsafe());
  for(Key key:solver.getFixedVariables())s.fixed.push_back(key);
  s.cached=solver.fl4nLinearCache().augmentedHessian();s.tree=GaussianFactorGraph(solver).augmentedHessian();
  save(d/"checkpoint.bin",s);auto restored=load(d/"checkpoint.bin");std::ofstream checked(d/"roundtrip.txt");parity(s,restored,checked);require(bool(checked),"roundtrip receipt");
  std::ofstream done(d/"complete.txt");done<<"independent checkpoint readback PASS\n";
}
}
