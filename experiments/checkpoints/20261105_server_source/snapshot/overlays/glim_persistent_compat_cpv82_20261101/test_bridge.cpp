#include "PersistentCompatibility.h"
#include "test_revision.cpp"
#include <glim_ros/cbs_bridge.hpp>
#include <glim_ros/cbs_native_imu_mean_bridge.hpp>
#include <ros/ros.h>
#include <fstream>
#include <iomanip>

namespace {
void rosInit() {
  if (!ros::isInitialized()) {
    int argc=1; char name[]="cpv82_bridge_tests"; char* argv[]={name,nullptr};
    ros::init(argc,argv,name,ros::init_options::NoSigintHandler);
  }
  // ROS shuts down when the last NodeHandle is destroyed. Keep the test node
  // alive across TEST cases, otherwise later setParam calls silently fail.
  static ros::NodeHandle keep_alive;
  ASSERT_TRUE(ros::ok());
}
void configure(ros::NodeHandle& nh,const std::string& mode) {
  nh.setParam("glim_cbs_bridge_enable",true);
  nh.setParam("glim_cbs_mode",mode);
  nh.setParam("glim_cbs_odom_covariance_mode","schur_relative_between");
  nh.setParam("glim_cbs_outgoing_marginal_source","direct");
  nh.setParam("glim_cbs_live_revision_enable",false);
  nh.setParam("glim_cbs_publish_beliefs",false);
  nh.setParam("glim_cbs_receive_external_beliefs",true);
  nh.setParam("glim_cbs_experimental_native_imu_mean_alignment",true);
}
struct ScopedEnvironment {
  std::map<std::string,std::optional<std::string>> old;
  void set(const char* key,const char* value) {
    if(!old.count(key)) {const char* p=getenv(key);old[key]=p?std::optional<std::string>(p):std::nullopt;}
    if(value)setenv(key,value,1);else unsetenv(key);
  }
  ~ScopedEnvironment(){for(auto& [key,value]:old)if(value)setenv(key.c_str(),value->c_str(),1);else unsetenv(key.c_str());}
};
}

TEST(PersistentCompatibility, StrictOptIn) {
  EXPECT_FALSE(cpv82::persistentCompatibility(nullptr));
  EXPECT_FALSE(cpv82::persistentCompatibility(""));
  EXPECT_FALSE(cpv82::persistentCompatibility("0"));
  EXPECT_TRUE(cpv82::persistentCompatibility("1"));
  EXPECT_THROW(cpv82::persistentCompatibility("true"),std::runtime_error);
}

TEST(PersistentCompatibility, ActualConstructorPreservesOtherRestrictions) {
  rosInit();ros::NodeHandle nh("/cpv82_admission");ScopedEnvironment env;
  env.set("GLIM_SHARED_NATIVE_PROVIDER","shared_native");
  env.set("GLIM_REVISION_COVARIANCE_POLICY","native_local");
  env.set("CPV82_GLIM_PERSISTENT_COMPAT",nullptr);
  configure(nh,"inject_persistent");
  EXPECT_THROW({glim::GlimCbsBridge bridge(nh);},std::runtime_error);
  env.set("CPV82_GLIM_PERSISTENT_COMPAT","1");
  EXPECT_NO_THROW({glim::GlimCbsBridge bridge(nh);EXPECT_TRUE(bridge.enabled());});
  nh.setParam("glim_cbs_live_revision_enable",true);
  EXPECT_THROW({glim::GlimCbsBridge bridge(nh);},std::runtime_error);
  configure(nh,"inject_persistent");nh.setParam("glim_cbs_outgoing_marginal_source","sidecar_batch");
  EXPECT_THROW({glim::GlimCbsBridge bridge(nh);},std::runtime_error);
  configure(nh,"temporary_linear");
  EXPECT_THROW({glim::GlimCbsBridge bridge(nh);},std::runtime_error);
  configure(nh,"observe_only");
  EXPECT_THROW({glim::GlimCbsBridge bridge(nh);},std::runtime_error);
  configure(nh,"active_window_temporary");
  env.set("CPV82_GLIM_PERSISTENT_COMPAT",nullptr);
  EXPECT_NO_THROW({glim::GlimCbsBridge bridge(nh);});
  env.set("CPV82_GLIM_PERSISTENT_COMPAT","1");
  EXPECT_NO_THROW({glim::GlimCbsBridge bridge(nh);});
}

TEST(PersistentCompatibility, ActualPersistentBridgeDispatchAndRetirement) {
  rosInit();ros::NodeHandle nh("/cpv82_dispatch");ScopedEnvironment env;
  env.set("GLIM_SHARED_NATIVE_PROVIDER","shared_native");
  env.set("GLIM_REVISION_COVARIANCE_POLICY","native_local");
  env.set("CPV82_GLIM_PERSISTENT_COMPAT","1");
  configure(nh,"inject_persistent");glim::GlimCbsBridge bridge(nh);
  ASSERT_TRUE(bridge.enabled());
  ASSERT_EQ(bridge.mode_,glim::GlimCbsBridge::Mode::InjectPersistent);
  Main main(5.0,params());
  size_t retired=0;std::vector<NonlinearFactor::shared_ptr> peers;
  for(int k=0;k<18;++k) {
    bridge.native_imu_mean_bridge_->append(100.+k,Vector3(0,0,9.81),Vector3::Zero());
    NonlinearFactorGraph native;
    if(k==0)native.emplace_shared<PriorFactor<Pose3>>(x(0),pose(0),noiseModel::Isotropic::Sigma(6,.1));
    else native.emplace_shared<BetweenFactor<Pose3>>(x(k-1),x(k),pose(k-1).between(pose(k)),noiseModel::Isotropic::Sigma(6,.1));
    Values values;values.insert(x(k),pose(k));
    std::map<std::uint64_t,double> stamps{{x(k),100.+k}};FactorIndices remove;
    if(k) {
      glim::GlimCbsBridge::IncomingBelief incoming;
      incoming.source_agent='k';incoming.from_pose_index=k-1;incoming.to_pose_index=k;
      incoming.from_stamp_sec=99.+k;incoming.to_stamp_sec=100.+k;
      const auto z=Pose3::Logmap(pose(k-1).between(pose(k)));
      for(int i=0;i<6;++i)incoming.relative_mu[i]=z[i];
      for(int i=0;i<6;++i)incoming.covariance[i*6+i]=.001;
      bridge.incoming_.push_back(incoming);bridge.incoming_.push_back(incoming);
    }
    const auto native_size=native.size();
    bridge.onSmootherUpdate(main,native,values,stamps,remove);
    ASSERT_TRUE(remove.empty());
    ASSERT_EQ(native.size(),native_size+(k?1:0));
    if(k) {
      const auto* factor=dynamic_cast<const BetweenFactor<Pose3>*>(native.back().get());
      ASSERT_NE(factor,nullptr);
      EXPECT_TRUE(factor->measured().equals(pose(k-1).between(pose(k)),1e-12));
      const auto gaussian=std::dynamic_pointer_cast<noiseModel::Gaussian>(factor->noiseModel());
      ASSERT_TRUE(gaussian);EXPECT_TRUE(gaussian->covariance().isApprox(Matrix6::Identity()*.001,1e-12));
      peers.push_back(native.back());
    }
    main.update(native,values,stamps,remove);
    auto obs=lookup(&main);ASSERT_TRUE(obs);ASSERT_TRUE(obs->valid())<<obs->error();
    EXPECT_EQ(obs->history().admittedCount(),size_t(k+1));
    EXPECT_EQ(obs->history().admittedCount(),obs->history().activeCount()+obs->history().absorbedCount());
    EXPECT_EQ(obs->history().removedCount(),0u);
    for(int j=0;j<int(peers.size());++j) {
      bool present=false;for(auto& f:main.getFactors())present|=f==peers[j];
      const bool endpoint_alive=main.getLinearizationPoint().exists(x(j));
      EXPECT_EQ(present,endpoint_alive);
      if(!present)++retired;
    }
    if(k) {
      const auto a=main.calculateEstimate<Pose3>(x(k-1)),b=main.calculateEstimate<Pose3>(x(k));
      PublishedCovariance first;ASSERT_TRUE(publicationCovariance(&main,x(k-1),x(k),a,b,first).ok);
      const auto revised=revisionCovariance(RevisionCovariancePolicy::NativeLocal,&main,x(k-1),x(k),a,b,
          []()->cbs::relative_pose_covariance::Result{throw std::runtime_error("MAIN fallback");});
      EXPECT_TRUE(first.relative.covariance.isApprox(revised.covariance,0));
    }
    EXPECT_TRUE(bridge.active_window_external_factors_.empty());
    EXPECT_TRUE(bridge.live_revision_factors_.empty());
  }
  EXPECT_EQ(bridge.injected_total_.load(),17u);
  EXPECT_EQ(bridge.duplicate_total_.load(),17u);
  EXPECT_GE(retired,10u);
  EXPECT_GE(lookup(&main)->history().absorbedCount(),10u);
}

// Executed against both the original frozen DSO and the private candidate in
// fresh processes. The serialized solver state, factor slots, Gaussian cache,
// native marginal history and exported covariance must be byte-identical.
TEST(PersistentCompatibility, TemporaryDisabledParity) {
  rosInit();ros::NodeHandle nh("/cpv82_parity");ScopedEnvironment env;
  env.set("GLIM_SHARED_NATIVE_PROVIDER","shared_native");
  env.set("GLIM_REVISION_COVARIANCE_POLICY","native_local");
  configure(nh,"active_window_temporary");glim::GlimCbsBridge bridge(nh);
  ASSERT_TRUE(bridge.enabled());Main main(5.0,params());
  std::ostringstream data;data<<std::setprecision(17);
  auto matrix=[&](const Matrix& m) {
    data<<m.rows()<<','<<m.cols()<<':';
    for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)data<<m(i,j)<<',';
    data<<'\n';
  };
  auto graph=[&](const GaussianFactorGraph& g) {
    data<<g.size()<<'\n';for(auto& f:g) {
      data<<bool(f)<<':';if(f){for(auto key:f->keys())data<<key<<',';matrix(f->augmentedInformation());}
      data<<'\n';
    }
  };
  size_t removals=0;
  for(int k=0;k<18;++k) {
    bridge.native_imu_mean_bridge_->append(100.+k,Vector3(0,0,9.81),Vector3::Zero());
    NonlinearFactorGraph native;
    if(!k)native.emplace_shared<PriorFactor<Pose3>>(x(0),pose(0),noiseModel::Isotropic::Sigma(6,.1));
    else native.emplace_shared<BetweenFactor<Pose3>>(x(k-1),x(k),pose(k-1).between(pose(k)),noiseModel::Isotropic::Sigma(6,.1));
    Values values;values.insert(x(k),pose(k));
    std::map<std::uint64_t,double> stamps{{x(k),100.+k}};FactorIndices remove;
    if(k) {
      glim::GlimCbsBridge::IncomingBelief incoming;
      incoming.source_agent='k';incoming.from_pose_index=k-1;incoming.to_pose_index=k;
      incoming.from_stamp_sec=99.+k;incoming.to_stamp_sec=100.+k;
      const auto z=Pose3::Logmap(pose(k-1).between(pose(k)).retract(Vector6::Constant(.001*(k%3+1))));
      for(int i=0;i<6;++i)incoming.relative_mu[i]=z[i];
      for(int i=0;i<6;++i)incoming.covariance[i*6+i]=.001;
      bridge.incoming_.push_back(incoming);
    }
    bridge.onSmootherUpdate(main,native,values,stamps,remove);
    ASSERT_EQ(native.size(),size_t(k?2:1));removals+=remove.size();
    main.update(native,values,stamps,remove);
    bridge.commitPendingActiveWindowFactorRemovals();
    bridge.registerPendingActiveWindowFactors(main);
    auto obs=lookup(&main);ASSERT_TRUE(obs);ASSERT_TRUE(obs->valid())<<obs->error();
    data<<k<<':'<<obs->history().admittedCount()<<','<<obs->history().activeCount()<<','
        <<obs->history().absorbedCount()<<','<<obs->history().removedCount()<<'\n';
    for(auto key:main.getLinearizationPoint().keys()) {
      data<<key<<':';matrix(main.getLinearizationPoint().at<Pose3>(key).matrix());
      matrix(main.calculateEstimate<Pose3>(key).matrix());
    }
    graph(main.getLinearFactors());graph(obs->history().graph(obs->history().epoch()));
    for(auto& f:main.getFactors()) {
      data<<bool(f)<<':';if(f)for(auto key:f->keys())data<<key<<',';data<<'\n';
    }
    if(k) {
      PublishedCovariance covariance;
      ASSERT_TRUE(publicationCovariance(&main,x(k-1),x(k),main.calculateEstimate<Pose3>(x(k-1)),
          main.calculateEstimate<Pose3>(x(k)),covariance).ok);
      matrix(covariance.relative.covariance);
    }
  }
  EXPECT_GE(removals,10u);EXPECT_EQ(bridge.injected_total_.load(),17u);
  if(const char* path=getenv("CPV82_PARITY_PATH")) {
    std::ifstream existing(path);ASSERT_FALSE(existing.good());
    std::ofstream out(path);out<<data.str();out.flush();ASSERT_TRUE(out.good());
  }
}
