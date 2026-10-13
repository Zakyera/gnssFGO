#include <gtest/gtest.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <atomic>
#include <cstring>
#include <thread>
#include "SourceWnoaModel.h"
#include "OriginalSourceWnoaModel.h"

namespace {
using namespace fgo::offline;
auto Qc(double scale=1.) {
  return gtsam::noiseModel::Diagonal::Variances(scale*(gtsam::Vector6()<<.00375,.00375,.00375,.015,.015,.015).finished());
}
gtsam::Values States(double offset=0.) {
  gtsam::Values v;
  for(size_t i=0;i<5;++i) {
    SourceWnoaState s;
    s.pose=gtsam::Pose3(gtsam::Rot3::RzRyRx(.12+.01*i,-.2+.03*i,.3+.11*i),{offset+.3*i,-.5+.4*i,.1*i});
    s.velocity={.4+.2*i,-.1,.2};s.omega={.1,.2,.03*i};InsertSourceWnoaState(v,i,s);
  }return v;
}
void Bits(const gtsam::Matrix& a,const gtsam::Matrix& b) {
  ASSERT_EQ(a.rows(),b.rows());ASSERT_EQ(a.cols(),b.cols());
  EXPECT_EQ(0,std::memcmp(a.data(),b.data(),a.size()*sizeof(double)))<<(a-b).norm();
}
void Check(const WnoaEndpoint& e,const e2b_original::WnoaEndpoint& oracle,const gtsam::Values& v) {
  Bits(e.pose(v).matrix(),oracle.pose(v).matrix());
  std::vector<gtsam::Matrix> a,b;
  Bits(e.pose(v,&a).matrix(),oracle.pose(v,&b).matrix());ASSERT_EQ(a.size(),b.size());
  for(size_t i=0;i<a.size();++i)Bits(a[i],b[i]);
}
auto Endpoint(size_t a,size_t b,double tau,std::shared_ptr<e2b::Cache> cache) {
  return WnoaEndpoint::Between(a,b,.1,tau,Qc()).withCache(cache);
}
void FactorEqual(const gtsam::NoiseModelFactor& a,const gtsam::NoiseModelFactor& b,const gtsam::Values& v) {
  EXPECT_EQ(a.keys(),b.keys());Bits(a.unwhitenedError(v),b.unwhitenedError(v));
  Bits(a.whitenedError(v),b.whitenedError(v));
  auto ja=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(a.linearize(v));
  auto jb=boost::dynamic_pointer_cast<gtsam::JacobianFactor>(b.linearize(v));
  Bits(ja->augmentedJacobian(),jb->augmentedJacobian());
}
TEST(E2b, DefaultOffUnlessExplicitEnvironment) {
  const char* env=std::getenv("E2B_ENDPOINT_CACHE");
  EXPECT_EQ(bool(e2b::DefaultCache()),env && std::string(env)=="1");
}
TEST(E2b, ExactEndpointBypassesCache) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();
  auto e=WnoaEndpoint::Exact(2).withCache(cache);
  Check(e,e2b_original::WnoaEndpoint::Exact(2),v);EXPECT_EQ(cache->counts().calls,0);
}
TEST(E2b, IndependentOracleAllEndpointCasesAndCopies) {
  auto cache=std::make_shared<e2b::Cache>();
  for(double offset:{0.,50.,1000.})for(double dt:{.1,1.})
    for(double f:{0.,1e-6,.23,.71,1.-1e-6,1.}) {
      auto e=WnoaEndpoint::Between(0,1,dt,dt*f,Qc()).withCache(cache);
      auto o=e2b_original::WnoaEndpoint::Between(0,1,dt,dt*f,Qc());auto v=States(offset);
      Check(e,o,v);auto n=cache->counts().evaluations;Check(e,o,v);
      auto copy=e;Check(copy,o,v);EXPECT_EQ(cache->counts().evaluations,n);
    }
}
TEST(E2b, IndependentlyConstructedEndpointsShareAndOutputsAreCopies) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();
  auto a=Endpoint(0,1,.023,cache),b=Endpoint(0,1,.023,cache);
  std::vector<gtsam::Matrix> h;a.pose(v,&h);auto n=cache->counts().evaluations;
  h[0].setConstant(999.);auto o=e2b_original::WnoaEndpoint::Between(0,1,.1,.023,Qc());
  std::vector<gtsam::Matrix> expected,actual;Bits(o.pose(v,&expected).matrix(),b.pose(v,&actual).matrix());
  for(size_t i=0;i<6;++i)Bits(expected[i],actual[i]);EXPECT_EQ(cache->counts().evaluations,n);
}
TEST(E2b, EveryVectorCoordinateAndPoseEndpointInvalidates) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();auto e=Endpoint(0,1,.023,cache);
  auto o=e2b_original::WnoaEndpoint::Between(0,1,.1,.023,Qc());Check(e,o,v);
  for(auto k:{V(0),W(0),V(1),W(1)})for(int axis=0;axis<3;++axis) {
    auto n=cache->counts().evaluations;auto x=v.at<gtsam::Vector3>(k);
    x[axis]=std::nextafter(x[axis],INFINITY);v.update(k,x);Check(e,o,v);
    EXPECT_EQ(cache->counts().evaluations,n+2);
  }
  for(auto k:{X(0),X(1)})for(int axis=0;axis<6;++axis) {
    auto n=cache->counts().evaluations;auto d=gtsam::Vector6::Zero().eval();d[axis]=1e-8;
    v.update(k,v.at<gtsam::Pose3>(k).retract(d));Check(e,o,v);EXPECT_EQ(cache->counts().evaluations,n+2);
  }
  auto n=cache->counts().evaluations;v.update(V(4),gtsam::Vector3(9,8,7));Check(e,o,v);
  EXPECT_EQ(cache->counts().evaluations,n); // unrelated Values do not invalidate
}
TEST(E2b, SupportKeysTimesQcAndOutputModeDistinguished) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();
  auto eval=[&](size_t i,size_t j,double dt,double tau,double scale,int64_t ns) {
    auto n=cache->counts().evaluations;
    auto e=WnoaEndpoint::Between(i,j,dt,tau,Qc(scale),ns).withCache(cache);
    auto o=e2b_original::WnoaEndpoint::Between(i,j,dt,tau,Qc(scale));Check(e,o,v);
    EXPECT_EQ(cache->counts().evaluations,n+2);
  };
  eval(0,1,.1,.023,1,10);eval(0,1,.1,.023,1,11);
  eval(0,1,std::nextafter(.1,1.),.023,1,11);
  eval(0,1,.1,std::nextafter(.023,1.),1,11);
  eval(0,1,.1,.023,1.01,11);eval(1,2,.1,.023,1,11);
}
TEST(E2b, ReplacedInterpolatorBypassesTrustedConfiguration) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();auto e=Endpoint(0,1,.023,cache);
  std::vector<gtsam::Matrix> h;e.pose(v,&h);auto n=cache->counts().calls;
  e.interpolator=std::make_shared<fgo::models::GPWNOAInterpolator>(Qc(),.1,.071,true,true);
  auto raw=e.withCache(nullptr);Bits(e.pose(v,&h).matrix(),raw.pose(v).matrix());
  EXPECT_EQ(cache->counts().calls,n);
}
TEST(E2b, DenseRevisedMeasurementAndNoiseRemainOutsideCache) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();
  auto a=Endpoint(0,1,.023,cache),b=Endpoint(1,2,.071,cache);
  auto oa=e2b_original::WnoaEndpoint::Between(0,1,.1,.023,Qc());
  auto ob=e2b_original::WnoaEndpoint::Between(1,2,.1,.071,Qc());
  for(int rev=0;rev<8;++rev) {
    gtsam::Matrix6 l=gtsam::Matrix6::Identity();l(4,0)=.1*rev;l(2,1)=-.07*rev;
    auto noise=gtsam::noiseModel::Gaussian::Covariance(.004*l*l.transpose(),false);
    auto z=oa.pose(v).between(ob.pose(v)).compose(gtsam::Pose3::Expmap(
      (gtsam::Vector6()<<.001*rev,.002,0.,.01*rev,.04,0.).finished()));
    WnoaRelativeFactor f(a,b,z,noise);e2b_original::WnoaRelativeFactor o(oa,ob,z,noise);
    FactorEqual(f,o,v);auto cloned=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f.clone());
    FactorEqual(*cloned,o,v);EXPECT_EQ(cache->counts().evaluations,4);
  }
}
TEST(E2b, ChangedOneEndpointKeepsOtherReusable) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();auto a=Endpoint(0,1,.023,cache),b=Endpoint(2,3,.071,cache);
  std::vector<gtsam::Matrix> h;a.pose(v,&h);b.pose(v,&h);auto n=cache->counts().evaluations;
  v.update(V(3),gtsam::Vector3(2,3,4));a.pose(v,&h);b.pose(v,&h);EXPECT_EQ(cache->counts().evaluations,n+1);
}
TEST(E2b, MissingValuesFailureAndRollbackPreservePriorEntry) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();auto e=Endpoint(0,1,.023,cache);
  std::vector<gtsam::Matrix> h;const auto expected=e.pose(v,&h);auto n=cache->counts().evaluations;
  auto bad=v;bad.erase(V(0));EXPECT_ANY_THROW(e.pose(bad,&h));
  Bits(e.pose(v,&h).matrix(),expected.matrix());EXPECT_EQ(cache->counts().evaluations,n);
  auto trial=v;trial.update(W(0),gtsam::Vector3(3,2,1));e.pose(trial,&h);
  Bits(e.pose(v,&h).matrix(),expected.matrix());EXPECT_EQ(cache->counts().evaluations,n+1);
  Bits(v.at<gtsam::Pose3>(X(0)).matrix(),States().at<gtsam::Pose3>(X(0)).matrix());
}
TEST(E2b, ExceptionAndNonfiniteComputationsNeverEnterCache) {
  e2b::Cache cache(4);e2b::Key k{7};int invoked=0;
  for(int i=0;i<2;++i)EXPECT_THROW(cache.get(k,[&]()->e2b::Value {++invoked;throw std::runtime_error("fixture");}),std::runtime_error);
  for(int i=0;i<2;++i)cache.get(k,[&] {++invoked;e2b::Value v;v.jacobians={gtsam::Matrix::Constant(2,2,NAN)};return v;});
  EXPECT_EQ(invoked,4);EXPECT_EQ(cache.counts().entries,0);
  cache.get(k,[]{return e2b::Value{};});cache.get(k,[]{throw std::runtime_error("must hit");return e2b::Value{};});
  EXPECT_EQ(cache.counts().hits,1);EXPECT_EQ(cache.counts().failures,2);EXPECT_EQ(cache.counts().nonfinite,2);
}
TEST(E2b, BoundedEvictionAndZeroCapacity) {
  for(size_t cap:{size_t(0),size_t(3)}) {
    e2b::Cache cache(cap);
    for(uint64_t i=0;i<100;++i)cache.get({i},[]{return e2b::Value{};});
    EXPECT_LE(cache.counts().entries,cap);EXPECT_LE(cache.counts().peak_entries,cap);
    EXPECT_EQ(cache.counts().evictions,cap?97:0);
  }
}
TEST(E2b, ConcurrentColdMissesAndImmutableWarmReads) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();auto e=Endpoint(0,1,.023,cache);
  auto o=e2b_original::WnoaEndpoint::Between(0,1,.1,.023,Qc());std::vector<gtsam::Matrix> expected;
  auto pose=o.pose(v,&expected);std::atomic<bool> good{true};std::vector<std::thread> threads;
  for(int k=0;k<8;++k)threads.emplace_back([&]{for(int repeat=0;repeat<30;++repeat){
    std::vector<gtsam::Matrix> h;auto p=e.pose(v,&h);
    if((p.matrix().array()!=pose.matrix().array()).any())good=false;
    for(size_t j=0;j<h.size();++j)if((h[j].array()!=expected[j].array()).any())good=false;
    h[0].setZero();
  }});
  for(auto& t:threads)t.join();EXPECT_TRUE(good);EXPECT_EQ(cache->counts().entries,1);
  EXPECT_GT(cache->counts().hits,200);EXPECT_LE(cache->counts().evaluations,8);
}
TEST(E2b, GraphCovarianceAndTransactionalBranchMatchOriginal) {
  auto cache=std::make_shared<e2b::Cache>();auto v=States();gtsam::NonlinearFactorGraph a,b;
  for(size_t i=0;i<5;++i) {
    for(auto* graph:{&a,&b}) {
      graph->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(i),v.at<gtsam::Pose3>(X(i)),gtsam::noiseModel::Isotropic::Sigma(6,.1));
      graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(i),v.at<gtsam::Vector3>(V(i)),gtsam::noiseModel::Isotropic::Sigma(3,.2));
      graph->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(W(i),v.at<gtsam::Vector3>(W(i)),gtsam::noiseModel::Isotropic::Sigma(3,.2));
    }
  }
  auto x=Endpoint(0,1,.023,cache),y=Endpoint(1,2,.071,cache);
  auto ox=e2b_original::WnoaEndpoint::Between(0,1,.1,.023,Qc()),oy=e2b_original::WnoaEndpoint::Between(1,2,.1,.071,Qc());
  auto z=ox.pose(v).between(oy.pose(v));auto noise=gtsam::noiseModel::Isotropic::Sigma(6,.05);
  a.emplace_shared<WnoaRelativeFactor>(x,y,z,noise);b.emplace_shared<e2b_original::WnoaRelativeFactor>(ox,oy,z,noise);
  gtsam::ISAM2Params params;params.factorization=gtsam::ISAM2Params::QR;
  gtsam::ISAM2 sa(params),sb(params);sa.update(a,v);sb.update(b,v);
  for(int iteration=0;iteration<3;++iteration){sa.update();sb.update();}
  for(size_t i=0;i<5;++i){Bits(sa.calculateEstimate<gtsam::Pose3>(X(i)).matrix(),sb.calculateEstimate<gtsam::Pose3>(X(i)).matrix());
    Bits(sa.marginalCovariance(X(i)),sb.marginalCovariance(X(i)));}
  auto before=sa.calculateEstimate<gtsam::Pose3>(X(0));
  {auto trial=sa;gtsam::NonlinearFactorGraph invalid;
   invalid.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(99),gtsam::Pose3(),noise);
   EXPECT_ANY_THROW(trial.update(invalid,{}));}
  Bits(sa.calculateEstimate<gtsam::Pose3>(X(0)).matrix(),before.matrix());
  Bits(sa.marginalCovariance(X(0)),sb.marginalCovariance(X(0)));
}
} // namespace
