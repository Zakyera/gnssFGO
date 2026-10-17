#include <gtest/gtest.h>
#include <gtsam/linear/JacobianFactor.h>
#include <fstream>
#include <sstream>
#include "SourceWnoaModel.h"
#include "NativeSourceWnoaModel.h"

using namespace fgo::offline;
namespace native = fgo::offline::native_reference;
using NF = gtsam::NoiseModelFactor;
static auto Qc() { return gtsam::noiseModel::Diagonal::Variances(
  (gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished()); }
static native::WnoaEndpoint Native(const WnoaEndpoint& e) {
  return {e.exact,e.left,e.right,e.interpolator};
}
static void Exact(const gtsam::Matrix& a,const gtsam::Matrix& b) {
  ASSERT_EQ(a.rows(),b.rows()); ASSERT_EQ(a.cols(),b.cols());
  EXPECT_TRUE((a.array()==b.array()).all()) << (a-b).norm();
}
static gtsam::Values States(double offset) {
  gtsam::Values v;
  for(size_t i=0;i<4;++i) {
    SourceWnoaState s;
    s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.15+.03*i,-.22+.04*i,.3+.12*i),
                       {offset+.5*i,-offset+.2*i,.1*i});
    s.velocity={.6+.1*i,-.2+.1*i,.3};s.omega={.11,-.21+.03*i,.16};
    InsertSourceWnoaState(v,i,s);
  }
  return v;
}
static gtsam::Matrix Difference(const NF& factor,const gtsam::Values& v,
                                gtsam::Key key,double step) {
  const auto r=factor.whitenedError(v);gtsam::Matrix j(r.size(),v.at(key).dim());
  for(int c=0;c<j.cols();++c) {
    auto d=v.zeroVectors();
    d.at(key)[c]=2*step;const auto p2=factor.whitenedError(v.retract(d));
    d.at(key)[c]=step;const auto p1=factor.whitenedError(v.retract(d));
    d.at(key)[c]=-step;const auto m1=factor.whitenedError(v.retract(d));
    d.at(key)[c]=-2*step;const auto m2=factor.whitenedError(v.retract(d));
    j.col(c)=(8*(p1-m1)-(p2-m2))/(12*step);
  }return j;
}
static double Check(const WnoaRelativeFactor& candidate,const native::WnoaRelativeFactor& old,
                    const gtsam::Values& v,double tolerance) {
  EXPECT_EQ(candidate.keys(),old.keys());
  Exact(candidate.unwhitenedError(v),old.unwhitenedError(v));
  Exact(candidate.whitenedError(v),old.whitenedError(v));
  std::vector<gtsam::Matrix> hc,hn;
  Exact(candidate.unwhitenedError(v,hc),old.unwhitenedError(v,hn));
  auto jc=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(candidate.linearize(v));
  auto jn=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(old.linearize(v));
  Exact(jc->getb(),jn->getb());double worst=0;
  for(size_t k=0;k<candidate.keys().size();++k) {
    auto key=candidate.keys()[k];auto symbol=gtsam::Symbol(key);
    auto ci=std::find(jc->begin(),jc->end(),key),ni=std::find(jn->begin(),jn->end(),key);
    if(symbol.chr()=='x') { Exact(hc[k],hn[k]);Exact(jc->getA(ci),jn->getA(ni)); }
    else {
      const gtsam::Matrix direct=Difference(candidate,v,key,1e-3);
      const double error=(jc->getA(ci)-direct).cwiseAbs().maxCoeff();worst=std::max(worst,error);
      EXPECT_LT(error,tolerance) << gtsam::DefaultKeyFormatter(key);
    }
  }
  auto again=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(candidate.linearize(v));
  Exact(jc->augmentedJacobian(),again->augmentedJacobian());
  return worst;
}

TEST(TwistDerivative, EndpointResidualAndPoseBlocksExactlyPreserved) {
  for(double offset:{0.,50.,1000.})for(double dt:{.1,1.})
    for(double fraction:{0.,1e-6,.23,.71,1.-1e-6,1.}) {
      auto v=States(offset);auto ep=WnoaEndpoint::Between(0,1,dt,dt*fraction,Qc());
      std::vector<gtsam::Matrix> hc,hn;
      Exact(ep.pose(v).matrix(),Native(ep).pose(v).matrix());
      Exact(ep.pose(v,&hc).matrix(),Native(ep).pose(v,&hn).matrix());
      Exact(hc[0],hn[0]);Exact(hc[3],hn[3]);
      std::vector<gtsam::Matrix> repeat;ep.pose(v,&repeat);
      for(size_t k=0;k<6;++k)Exact(hc[k],repeat[k]);
    }
}
class TwistTopology : public ::testing::TestWithParam<int> {};
TEST_P(TwistTopology, SharedKeysDenseNoiseDirectDifference) {
  const auto qc=Qc();
  auto e=[](size_t a,size_t b,double t){return WnoaEndpoint::Between(a,b,.1,t,Qc());};
  std::vector<std::pair<WnoaEndpoint,WnoaEndpoint>> choices={
    {WnoaEndpoint::Exact(0),WnoaEndpoint::Exact(1)},
    {WnoaEndpoint::Exact(0),e(1,2,.071)}, {e(0,1,.023),WnoaEndpoint::Exact(2)},
    {e(0,1,.023),e(0,1,.071)}, {e(0,1,.023),e(1,2,.071)},
    {e(0,1,.023),e(2,3,.071)}, {e(0,1,0.),e(1,2,.1)}};
  auto [a,b]=choices.at(GetParam());
  gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.3;l(2,1)=-.2;l(5,3)=.4;
  auto noise=gtsam::noiseModel::Gaussian::Covariance(.0004*l*l.transpose(),false);
  for(double offset:{0.,50.,1000.}) {
    auto v=States(offset);
    const auto z=a.pose(v).between(b.pose(v)).compose(gtsam::Pose3::Expmap(
      (gtsam::Vector6()<<.02,-.01,.03,.1,.04,-.02).finished()));
    WnoaRelativeFactor c(a,b,z,noise);native::WnoaRelativeFactor n(Native(a),Native(b),z,noise);
    Check(c,n,v,5e-6);
    EXPECT_EQ(c.noiseModel().get(),n.noiseModel().get());
    gtsam::Values moved=v;auto d=v.zeroVectors();d.at(V(1))[0]=.4;moved=v.retract(d);
    Check(c,n,moved,5e-6);
  }
}
INSTANTIATE_TEST_SUITE_P(AllEndpoints,TwistTopology,::testing::Range(0,7));
static std::vector<std::string> Split(const std::string& line) {
  std::stringstream s(line);std::string x;std::vector<std::string> v;
  while(std::getline(s,x,','))v.push_back(x);return v;
}
TEST(TwistDerivative, SavedFailureAllSourceBlocksAndOriginalCovariance) {
  std::ifstream states("/home/yeranis/results/gnssfgo_publication_cache_20261010/cached/failure_probe/simultaneous_values.csv");ASSERT_TRUE(states.good());
  gtsam::Values v;std::string line;std::getline(states,line);
  while(std::getline(states,line)) {
    auto f=Split(line);auto k=gtsam::Symbol(f[0][0],std::stoull(f[0].substr(1)));
    if(f[0][0]=='x')v.insert(k,gtsam::Pose3(gtsam::Rot3::Quaternion(std::stod(f[7]),std::stod(f[4]),std::stod(f[5]),std::stod(f[6])),
      {std::stod(f[1]),std::stod(f[2]),std::stod(f[3])}));
    else v.insert(k,gtsam::Vector3(std::stod(f[1]),std::stod(f[2]),std::stod(f[3])));
  }
  auto endpoint=[](int64_t t) {
    const auto dt=t-1727775008583475000LL;const size_t left=dt/100000000LL;const auto rem=dt%100000000LL;
    return rem==0 ? WnoaEndpoint::Exact(left):WnoaEndpoint::Between(left,left+1,.1,rem*1e-9,Qc());
  };
  std::ifstream factors("/home/yeranis/results/gnssfgo_derivative_check_20261011/selected_factors.csv");ASSERT_TRUE(factors.good());size_t count=0;double worst=0;
  while(std::getline(factors,line)) {
    auto f=Split(line);if(f[1]!="source")continue;
    gtsam::Vector6 mu;gtsam::Matrix6 cov;for(int c=0;c<6;++c)mu[c]=std::stod(f[4+c]);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)cov(i,j)=std::stod(f[10+6*i+j]);
    auto a=endpoint(std::stoll(f[2])),b=endpoint(std::stoll(f[3]));
    auto noise=gtsam::noiseModel::Gaussian::Covariance(cov,false);auto z=gtsam::Pose3::Expmap(mu);
    WnoaRelativeFactor c(a,b,z,noise);native::WnoaRelativeFactor n(Native(a),Native(b),z,noise);
    gtsam::Values local;for(auto k:c.keys())local.insert(k,v.at(k));
    worst=std::max(worst,Check(c,n,local,1e-6));++count;
  }
  EXPECT_EQ(count,27);std::cout<<"Saved source factors="<<count<<" max whitened VW discrepancy="<<worst<<'\n';
}
TEST(TwistDerivative, ExactEndpointNeedsNoInterpolator) {
  auto v=States(0);auto e=WnoaEndpoint::Exact(2);std::vector<gtsam::Matrix> h;
  Exact(e.pose(v,&h).matrix(),v.at<gtsam::Pose3>(X(2)).matrix());
  ASSERT_EQ(h.size(),1);Exact(h[0],gtsam::Matrix6::Identity());
}
