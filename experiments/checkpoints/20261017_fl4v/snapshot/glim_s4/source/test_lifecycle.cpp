#include "Provider.h"
#include "ShadowObserver.h"
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <iostream>
#include <thread>

using namespace gtsam;
using namespace shared_native;
using Main=gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
namespace {
Key x(int i) { return Symbol('x',i); }
ISAM2Params params() {
  ISAM2Params p;p.findUnusedFactorSlots=true;p.cacheLinearizedFactors=true;
  p.relinearizeSkip=1;p.setRelinearizeThreshold(.001);return p;
}
Pose3 pose(int i) { return Pose3(Rot3::Rz(.02*i),Point3(.8*i,.04*i*i,.02*i)); }
auto noise=noiseModel::Isotropic::Sigma(6,.3);
struct Counts { int linear=0,error=0; };
class Counted : public BetweenFactor<Pose3> {
 public:
  Counted(Key a,Key b,Pose3 z,std::shared_ptr<Counts> c)
    : BetweenFactor<Pose3>(a,b,z,noise),c_(std::move(c)) {}
  GaussianFactor::shared_ptr linearize(const Values& v) const override {
    ++c_->linear;return BetweenFactor<Pose3>::linearize(v);
  }
  double error(const Values& v) const override { ++c_->error;return BetweenFactor<Pose3>::error(v); }
 private: std::shared_ptr<Counts> c_;
};
class ThrowOnce : public BetweenFactor<Pose3> {
 public:
  ThrowOnce(Key a,Key b,Pose3 z):BetweenFactor<Pose3>(a,b,z,noise) {}
  GaussianFactor::shared_ptr linearize(const Values& v) const override {
    if(first_) { first_=false;throw std::runtime_error("intentional native linearization failure"); }
    return BetweenFactor<Pose3>::linearize(v);
  }
 private: mutable bool first_=true;
};
void exactMain(const Main& a,const Main& b) {
  EXPECT_TRUE(a.getLinearizationPoint().equals(b.getLinearizationPoint(),0));
  EXPECT_TRUE(a.calculateEstimate().equals(b.calculateEstimate(),0));
  const auto& ga=a.getLinearFactors();const auto& gb=b.getLinearFactors();
  ASSERT_EQ(ga.size(),gb.size());
  for(std::size_t i=0;i<ga.size();++i) {
    ASSERT_EQ(bool(ga[i]),bool(gb[i]));
    if(ga[i]) {
      EXPECT_EQ(ga[i]->keys(),gb[i]->keys());
      EXPECT_TRUE(ga[i]->augmentedInformation().isApprox(gb[i]->augmentedInformation(),0));
    }
  }
}
void initial(Main& main,const std::shared_ptr<ShadowObserver>& o={}) {
  NonlinearFactorGraph f;f.add(PriorFactor<Pose3>(x(0),pose(0),noise));
  Values v;v.insert(x(0),pose(0));if(o)o->admit(f);main.update(f,v,{{x(0),0}});
}
// Independent dense Schur history: explicit block scatter and LDLT. It does not
// call candidate elimination, GTSAM Hessian assembly or candidate prior queries.
struct DenseOracle {
  KeyVector prior_keys,keys;
  Matrix prior,full;
  double max_error=0,max_cov_error=0;
  void capture(const Snapshot& s) {
    std::set<Key> all(prior_keys.begin(),prior_keys.end());
    for(const auto& r:s.native)for(auto key:s.cache[r.slot]->keys())all.insert(key);
    KeyVector ordering;
    for(auto key:all)if(s.retiring.count(key))ordering.push_back(key);
    const int m=6*ordering.size();
    for(auto key:all)if(!s.retiring.count(key))ordering.push_back(key);
    std::map<Key,int> off;for(std::size_t i=0;i<ordering.size();++i)off[ordering[i]]=6*i;
    const int n=6*ordering.size();Matrix h=Matrix::Zero(n+1,n+1);
    auto scatter=[&](const KeyVector& ks,const Matrix& aug) {
      std::vector<int> idx;for(auto key:ks)for(int j=0;j<6;++j)idx.push_back(off.at(key)+j);
      idx.push_back(n);ASSERT_EQ(aug.rows(),idx.size());
      for(std::size_t i=0;i<idx.size();++i)for(std::size_t j=0;j<idx.size();++j)h(idx[i],idx[j])+=aug(i,j);
    };
    if(prior.size())scatter(prior_keys,prior);
    // Only incident native contributions belong in the new prior.
    for(const auto& r:s.native) {
      bool incident=false;for(auto k:s.cache[r.slot]->keys())incident|=s.retiring.count(k)>0;
      if(incident)scatter(s.cache[r.slot]->keys(),s.cache[r.slot]->augmentedInformation());
    }
    keys.assign(ordering.begin()+m/6,ordering.end());
    if(m) {
      Matrix a=h.topLeftCorner(m,m),cross=h.topRightCorner(m,n+1-m);
      Eigen::LDLT<Matrix> ldlt(a);ASSERT_EQ(ldlt.info(),Eigen::Success);
      full=(h.bottomRightCorner(n+1-m,n+1-m)-cross.transpose()*ldlt.solve(cross)).eval();
      // Compact zero rows for nonincident variables; preserve constant.
      prior_keys.clear();std::vector<int> keep;
      for(std::size_t k=0;k<keys.size();++k) if(full.middleRows(6*k,6).norm()>1e-14) {
        prior_keys.push_back(keys[k]);for(int d=0;d<6;++d)keep.push_back(6*k+d);
      }
      keep.push_back(full.rows()-1);prior.resize(keep.size(),keep.size());
      for(std::size_t i=0;i<keep.size();++i)for(std::size_t j=0;j<keep.size();++j)prior(i,j)=full(keep[i],keep[j]);
    } else full=h;
    off.clear();for(std::size_t i=0;i<keys.size();++i)off[keys[i]]=6*i;
    // Add each surviving active native contribution exactly once.
    for(const auto& r:s.native) {
      const auto& f=s.cache[r.slot];bool incident=false;
      for(auto k:f->keys())incident|=s.retiring.count(k)>0;
      if(incident)continue;
      std::vector<int> idx;for(auto k:f->keys())for(int d=0;d<6;++d)idx.push_back(off.at(k)+d);
      idx.push_back(full.rows()-1);const Matrix aug=f->augmentedInformation();
      for(std::size_t i=0;i<idx.size();++i)for(std::size_t j=0;j<idx.size();++j)full(idx[i],idx[j])+=aug(i,j);
    }
  }
  void compare(const ShadowObserver& obs,Key from,Key to) {
    const auto graph=obs.history().graph(obs.history().epoch());
    Ordering order(keys.begin(),keys.end());
    Matrix candidate=graph.augmentedHessian(order);
    const double error=(candidate-full).norm()/std::max(1.,full.norm());max_error=std::max(max_error,error);
    EXPECT_LT(error,1e-9);
    if(from==to)return;
    Matrix h=full.topLeftCorner(full.rows()-1,full.cols()-1);
    Matrix cov=h.ldlt().solve(Matrix::Identity(h.rows(),h.cols()));
    const auto out=obs.history().covariance(obs.history().epoch(),from,to,Vector6::Zero(),Vector6::Zero());
    std::map<Key,int> off;for(std::size_t i=0;i<keys.size();++i)off[keys[i]]=6*i;
    Matrix expected(12,12);expected<<cov.block<6,6>(off.at(from),off.at(from)),cov.block<6,6>(off.at(from),off.at(to)),
        cov.block<6,6>(off.at(to),off.at(from)),cov.block<6,6>(off.at(to),off.at(to));
    const double ce=(out.joint_theta-expected).norm()/expected.norm();max_cov_error=std::max(max_cov_error,ce);
    EXPECT_LT(ce,1e-8);
  }
};
}

TEST(Lifecycle, DisabledDoesNotRegisterOrCompute) {
  Main main(2,params());initial(main);EXPECT_FALSE(lookup(&main));main.update();EXPECT_FALSE(lookup(&main));
}
TEST(Lifecycle, ActualWrapperMultipleRetirementsDenseOracleAndExactNoninterference) {
  Main baseline(2.5,params()),main(2.5,params());auto observer=attach(&main);DenseOracle oracle;
  if(const char* p=std::getenv("SHARED_NATIVE_TEST_JOURNAL"))observer->setRecording(p);
  observer->setAudit([&](const Snapshot& s){oracle.capture(s);});
  auto c=std::make_shared<Counts>(),b=std::make_shared<Counts>();
  initial(main,observer);initial(baseline);ASSERT_TRUE(observer->valid())<<observer->error();
  oracle.compare(*observer,x(0),x(0));
  for(int i=1;i<=16;++i) {
    auto make=[&](const std::shared_ptr<Counts>& counts) {
      NonlinearFactorGraph native;
      native.push_back(std::make_shared<Counted>(x(i-1),x(i),pose(i-1).between(pose(i)),counts));
      return native;
    };
    auto native=make(c),all=make(b);observer->admit(native);
    // Same class as native: provenance, not RTTI, distinguishes CBS.
    native.push_back(std::make_shared<Counted>(x(i-1),x(i),
          pose(i-1).between(pose(i)).retract(Vector6::Constant(.003)),c));
    all.push_back(std::make_shared<Counted>(x(i-1),x(i),
          pose(i-1).between(pose(i)).retract(Vector6::Constant(.003)),b));
    Values values;values.insert(x(i),pose(i).retract(Vector6::Constant(.01)));
    baseline.update(all,values,{{x(i),double(i)}});main.update(native,values,{{x(i),double(i)}});
    ASSERT_FALSE(main.fallbackHappened());ASSERT_TRUE(observer->valid())<<observer->error();
    oracle.compare(*observer,x(i-1),x(i));exactMain(main,baseline);
    EXPECT_EQ(c->linear,b->linear);EXPECT_EQ(c->error,b->error);
    main.update();baseline.update();ASSERT_TRUE(observer->valid())<<observer->error();
    oracle.compare(*observer,x(i-1),x(i));exactMain(main,baseline);
    const auto before=*c;EndpointCovariance output;
    const auto result=observer->query(x(i-1),x(i),main.calculateEstimate<Pose3>(x(i-1)),
                                   main.calculateEstimate<Pose3>(x(i)),output);
    ASSERT_TRUE(result.ok)<<result.message;
    observer->recordQueryInput(x(i-1),x(i),main.calculateEstimate<Pose3>(x(i-1)),main.calculateEstimate<Pose3>(x(i)));
    observer->recordQueryOutput(output,0.);
    EXPECT_EQ(c->linear,before.linear);EXPECT_EQ(c->error,before.error);
    EXPECT_EQ(observer->history().admittedCount(),std::size_t(i+1));
    EXPECT_EQ(observer->history().admittedCount(),observer->history().activeCount()+observer->history().absorbedCount());
  }
  EXPECT_EQ(observer->history().epoch(),33u);EXPECT_EQ(observer->captures(),33u);
  EXPECT_GE(observer->history().absorbedCount(),14u);
  std::cout<<"LIFECYCLE dense_relative="<<oracle.max_error<<" covariance_relative="<<oracle.max_cov_error
    <<" captures="<<observer->captures()<<" admitted="<<observer->history().admittedCount()
    <<" absorbed="<<observer->history().absorbedCount()<<" active="<<observer->history().activeCount()
    <<" extra_native_calls=0\n";
}
TEST(Lifecycle, ObserverExceptionDoesNotTriggerFallbackOrMutateMain) {
  Main baseline(2,params()),main(2,params());auto o=attach(&main);initial(main,o);initial(baseline);
  const auto epoch=o->history().epoch();const auto before=o->history().graph(epoch).augmentedHessian();
  o->setAudit([](const Snapshot&){throw std::runtime_error("intentional observer fault");});
  main.update();baseline.update();EXPECT_FALSE(main.fallbackHappened());EXPECT_FALSE(o->valid());
  EXPECT_EQ(o->history().epoch(),epoch);EXPECT_TRUE(o->history().graph(epoch).augmentedHessian().isApprox(before,0));
  exactMain(main,baseline);
}
TEST(Lifecycle, ActualFallbackInvalidatesEvenAfterNextEmptyUpdate) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  NonlinearFactorGraph graph;graph.push_back(std::make_shared<ThrowOnce>(x(0),x(1),pose(0).between(pose(1))));
  o->admit(graph);Values values;values.insert(x(1),pose(1));
  EXPECT_NO_THROW(main.update(graph,values,{{x(1),1.}}));
  EXPECT_TRUE(main.fallbackHappened());EXPECT_FALSE(o->valid());EXPECT_EQ(o->resets(),1u);
  EXPECT_EQ(o->history().epoch(),1u);main.update();EXPECT_FALSE(main.fallbackHappened());EXPECT_FALSE(o->valid());
}
TEST(Lifecycle, ExplicitRemovalAndNativeSlotReuse) {
  Main main(10,params());auto o=attach(&main);initial(main,o);
  NonlinearFactorGraph native;native.add(BetweenFactor<Pose3>(x(0),x(1),pose(1),noise));
  native.add(PriorFactor<Pose3>(x(1),pose(1),noise));o->admit(native);Values v;v.insert(x(1),pose(1));
  main.update(native,v,{{x(1),1}});ASSERT_TRUE(o->valid())<<o->error();
  const auto removed_slot=main.getISAM2Result().newFactorsIndices[1];
  main.update({}, {}, {}, {removed_slot});ASSERT_TRUE(o->valid())<<o->error();
  NonlinearFactorGraph replacement;replacement.add(PriorFactor<Pose3>(x(1),pose(1),noise));o->admit(replacement);
  main.update(replacement);ASSERT_TRUE(o->valid())<<o->error();
  EXPECT_EQ(main.getISAM2Result().newFactorsIndices[0],removed_slot);
  EXPECT_EQ(o->history().admittedCount(),4u);EXPECT_EQ(o->history().removedCount(),1u);EXPECT_EQ(o->history().activeCount(),3u);
}
TEST(Lifecycle, SameSlotExternalReplacementIsNotNative) {
  Main main(10,params());auto o=attach(&main);initial(main,o);
  NonlinearFactorGraph extra;extra.add(PriorFactor<Pose3>(x(0),pose(0),noise));o->admit(extra);main.update(extra);
  auto slot=main.getISAM2Result().newFactorsIndices[0];main.update({}, {}, {}, {slot});
  main.update(extra);ASSERT_TRUE(o->valid())<<o->error();
  EXPECT_EQ(main.getISAM2Result().newFactorsIndices[0],slot);EXPECT_EQ(o->history().activeCount(),1u);
}
TEST(Lifecycle, MissingCacheInvalidatesButMainContinues) {
  auto p=params();p.cacheLinearizedFactors=false;Main main(2,p);auto o=attach(&main);
  EXPECT_NO_THROW(initial(main,o));EXPECT_FALSE(o->valid());EXPECT_FALSE(main.fallbackHappened());
  EXPECT_NE(std::string(o->error()).find("cache disabled"),std::string::npos);
}
TEST(Lifecycle, DuplicateAdmissionFailsBeforeMain) {
  Main main(2,params());auto o=attach(&main);NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose3>(x(0),pose(0),noise));o->admit(graph);o->admit(graph);EXPECT_FALSE(o->valid());
  Values v;v.insert(x(0),pose(0));EXPECT_NO_THROW(main.update(graph,v,{{x(0),0}}));EXPECT_FALSE(main.fallbackHappened());
}
TEST(Lifecycle, MissingAdmittedFactorInvalidates) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  NonlinearFactorGraph graph;graph.add(PriorFactor<Pose3>(x(0),pose(0),noise));o->admit(graph);
  main.update();EXPECT_FALSE(o->valid());EXPECT_FALSE(main.fallbackHappened());
}
TEST(Lifecycle, QueryFailureDoesNotChangeOutputOrMain) {
  Main main(2,params());auto o=attach(&main);initial(main,o);auto before=main.calculateEstimate();
  EndpointCovariance out;out.relative=Matrix6::Constant(17);
  EXPECT_FALSE(o->query(x(0),x(9),pose(0),pose(9),out).ok);
  EXPECT_TRUE(out.relative.isApprox(Matrix6::Constant(17),0));EXPECT_TRUE(main.calculateEstimate().equals(before,0));
}
TEST(Lifecycle, DestructorUnregistersOwnerAndPreventsAddressReuse) {
  const void* owner=nullptr;std::shared_ptr<ShadowObserver> o;
  { Main main(2,params());owner=&main;o=attach(owner);initial(main,o);ASSERT_EQ(lookup(owner),o); }
  EXPECT_FALSE(lookup(owner));
}
TEST(Lifecycle, DefaultOffEnvironmentAndMidrunEnableFailsClosed) {
  const char* env=std::getenv("GLIM_SHARED_NATIVE_SHADOW");const std::string old=env?env:"";const bool had=env;
  unsetenv("GLIM_SHARED_NATIVE_SHADOW");Main main(2,params());NonlinearFactorGraph f;
  admitFromEnvironment(&main,true,f);EXPECT_FALSE(lookup(&main));initial(main);
  setenv("GLIM_SHARED_NATIVE_SHADOW","1",1);admitFromEnvironment(&main,false,f);
  ASSERT_TRUE(lookup(&main));EXPECT_FALSE(lookup(&main)->valid());
  if(had)setenv("GLIM_SHARED_NATIVE_SHADOW",old.c_str(),1);else unsetenv("GLIM_SHARED_NATIVE_SHADOW");
}
TEST(Lifecycle, SeparateOwnersDoNotCrossAdmit) {
  Main a(2,params()),b(2,params());auto oa=attach(&a),ob=attach(&b);
  initial(a,oa);initial(b,ob);a.update();EXPECT_EQ(oa->history().epoch(),2u);EXPECT_EQ(ob->history().epoch(),1u);
  reset(&a);b.update();EXPECT_FALSE(oa->valid());EXPECT_TRUE(ob->valid());
}
TEST(Lifecycle, WrapperExceptionBeforeInnerUpdateRollsBackShadow) {
  Main main(2,params());auto o=attach(&main);initial(main,o);Values duplicate;duplicate.insert(x(0),pose(0));
  EXPECT_THROW(main.update({},duplicate),std::exception);
  EXPECT_FALSE(o->valid());EXPECT_EQ(o->history().epoch(),1u);
  EXPECT_NE(std::string(o->error()).find("unwound"),std::string::npos);
}
TEST(Lifecycle, EstimateFallbackOutsideUpdateInvalidatesShadow) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  EXPECT_THROW(main.calculateEstimate<Pose3>(x(999)),std::exception);
  EXPECT_FALSE(o->valid());EXPECT_EQ(o->resets(),1u);EXPECT_EQ(o->history().epoch(),1u);
}
TEST(Lifecycle, RemoveAndInsertInOneUpdateKeepsStableIdentities) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  NonlinearFactorGraph extra;extra.add(PriorFactor<Pose3>(x(0),pose(0),noise));o->admit(extra);main.update(extra);
  const auto old=main.getISAM2Result().newFactorsIndices[0];
  NonlinearFactorGraph replacement;replacement.add(PriorFactor<Pose3>(x(0),pose(0),noise));o->admit(replacement);
  main.update(replacement,{}, {},{old});ASSERT_TRUE(o->valid())<<o->error();
  EXPECT_EQ(o->history().admittedCount(),3u);EXPECT_EQ(o->history().removedCount(),1u);
  EXPECT_EQ(o->history().activeCount(),2u);
}
TEST(Lifecycle, ActualNativeImuVelocityBiasAndPoseRetireTogether) {
  Main main(.25,params());auto o=attach(&main);
  auto v=[](int i){return Key(Symbol('v',i));};auto b=[](int i){return Key(Symbol('b',i));};
  const auto pre=PreintegrationParams::MakeSharedU(9.81);
  pre->accelerometerCovariance=Matrix3::Identity()*.01;
  pre->gyroscopeCovariance=Matrix3::Identity()*.01;
  pre->integrationCovariance=Matrix3::Identity()*.001;
  const imuBias::ConstantBias bias;
  for(int i=0;i<10;++i) {
    Values values;values.insert(x(i),Pose3());values.insert(v(i),Vector3::Zero().eval());values.insert(b(i),bias);
    NonlinearFactorGraph f;
    if(!i) {
      f.add(PriorFactor<Pose3>(x(i),Pose3(),noise));
      f.add(PriorFactor<Vector3>(v(i),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.3)));
      f.add(PriorFactor<imuBias::ConstantBias>(b(i),bias,noise));
    } else {
      PreintegratedImuMeasurements pim(pre,bias);
      pim.integrateMeasurement(Vector3(0,0,9.81),Vector3::Zero(),.1);
      f.add(ImuFactor(x(i-1),v(i-1),x(i),v(i),b(i-1),pim));
      f.add(BetweenFactor<imuBias::ConstantBias>(b(i-1),b(i),bias,noise));
      f.add(BetweenFactor<Pose3>(x(i-1),x(i),Pose3(),noise));
    }
    o->admit(f);main.update(f,values,{{x(i),i*.1},{v(i),i*.1},{b(i),i*.1}});
    ASSERT_TRUE(o->valid())<<o->error();ASSERT_FALSE(main.fallbackHappened());
    if(i) {
      EndpointCovariance cov;auto result=o->query(x(i-1),x(i),main.calculateEstimate<Pose3>(x(i-1)),
                                  main.calculateEstimate<Pose3>(x(i)),cov);
      ASSERT_TRUE(result.ok)<<result.message;EXPECT_TRUE(cov.relative.allFinite());
    }
  }
  EXPECT_EQ(o->history().admittedCount(),30u);EXPECT_GE(o->history().absorbedCount(),20u);
  EXPECT_EQ(o->history().theta(o->history().epoch()).size(),9u);
}

TEST(Recording, FailureContainedAndNoExistingFileOverwritten) {
  Main main(2,params()),baseline(2,params());auto o=attach(&main);
  o->setRecording("/dev/null");EXPECT_FALSE(o->valid());
  initial(main,o);initial(baseline);exactMain(main,baseline);EXPECT_FALSE(main.fallbackHappened());
}
TEST(Recording, NoLateBootstrap) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  o->setRecording("/path/that/does/not/exist/journal");EXPECT_FALSE(o->valid());
  EXPECT_NE(std::string(o->error()).find("before initialization"),std::string::npos);
}

TEST(Provider, SelectorIsExplicitAndDefaultOff) {
  EXPECT_FALSE(parseProvider(nullptr));EXPECT_FALSE(parseProvider(""));
  EXPECT_FALSE(parseProvider("independent_local"));EXPECT_TRUE(parseProvider("shared_native"));
  EXPECT_THROW(parseProvider("unknown"),std::runtime_error);
}
TEST(Provider, SameHelperNoExtraEvaluationAndNoMainMutation) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  auto counts=std::make_shared<Counts>();
  NonlinearFactorGraph f;f.add(std::make_shared<Counted>(x(0),x(1),pose(0).between(pose(1)),counts));
  Values v;v.insert(x(1),pose(1));o->admit(f);main.update(f,v,{{x(1),1}});
  auto prior=main.calculateEstimate();auto before=*counts;
  PublishedCovariance result;
  ASSERT_TRUE(publicationCovariance(&main,x(0),x(1),prior.at<Pose3>(x(0)),prior.at<Pose3>(x(1)),result).ok);
  EXPECT_LT((result.relative.covariance-result.endpoints.relative).norm(),1e-12);
  auto oracle=cbs::relative_pose_covariance::compute(main.jointMarginalInformation({x(0),x(1)}),
    x(0),x(1),prior.at<Pose3>(x(0)),prior.at<Pose3>(x(1)),
    prior.at<Pose3>(x(0)).between(prior.at<Pose3>(x(1))),cbs::relative_pose_covariance::Mode::SchurRelativeBetween);
  EXPECT_LT((result.relative.covariance-oracle.covariance).norm(),1e-10);
  EXPECT_EQ(counts->linear,before.linear);EXPECT_EQ(counts->error,before.error);
  EXPECT_TRUE(main.calculateEstimate().equals(prior,0));
}
TEST(Provider, UnavailableAndBadEndpointLeaveOutputUntouched) {
  PublishedCovariance output;output.relative.covariance.setConstant(17);
  int unknown=0;
  EXPECT_FALSE(publicationCovariance(&unknown,x(0),x(1),pose(0),pose(1),output).ok);
  EXPECT_TRUE((output.relative.covariance.array()==17).all());
  Main main(2,params());auto o=attach(&main);initial(main,o);auto prior=main.calculateEstimate();
  EXPECT_FALSE(publicationCovariance(&main,x(0),x(9),pose(0),pose(9),output).ok);
  EXPECT_FALSE(o->valid());EXPECT_TRUE((output.relative.covariance.array()==17).all());
  EXPECT_TRUE(main.calculateEstimate().equals(prior,0));
}
