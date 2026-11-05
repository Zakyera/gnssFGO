// Diagnostic only; reuse the passed K16b geometry, solver and output helpers.
#define main k16b_unexecuted_main
#include "test_main_lifecycle.cpp"
#undef main
#include <chrono>
#include <sys/resource.h>
#include <openssl/sha.h>

void compactFinal(Output& o, Main& main, int k) {
  o.values("theta", main.getLinearizationPoint());
  auto estimate=main.calculateEstimate();
  o.values("estimate", estimate);
  o.gaussian("cache", cacheOf(main));
  auto obs=lookup(&main);
  o.gaussian("native",obs->history().graph(obs->history().epoch()));
  o.counts("conservation",{obs->history().admittedCount(),obs->history().activeCount(),
    obs->history().absorbedCount(),obs->history().removedCount()});
  require(obs->history().admittedCount()==obs->history().activeCount()+
    obs->history().absorbedCount()+obs->history().removedCount(),"native conservation");
  for(size_t i=0;i<main.getFactors().size();++i) {
    auto f=main.getFactors()[i];o.counts("present/"+std::to_string(i),{bool(f)});
    if(!f)continue;
    o.exact("keys/"+std::to_string(i),f->keys());
    if(auto* gpu=dynamic_cast<GPU*>(f.get())) {
      require(!gpu->has_cached_linearization(),"pending cache after update");
      auto bytes=gtsam_points::ExactGpuSnapshot::state(*gpu);
      unsigned char hash[SHA256_DIGEST_LENGTH];
      SHA256(bytes.data(),bytes.size(),hash);
      std::vector<unsigned> digest(hash,hash+SHA256_DIGEST_LENGTH);
      o.exact("gpu_sha256/"+std::to_string(i),digest);
    }
  }
  PublishedCovariance first;
  auto a=estimate.at<Pose3>(X(k-1)),b=estimate.at<Pose3>(X(k));
  auto result=publicationCovariance(&main,X(k-1),X(k),a,b,first);
  require(result.ok,result.message);
  auto later=revisionCovariance(RevisionCovariancePolicy::NativeLocal,&main,
    X(k-1),X(k),a,b,[]()->cbs::relative_pose_covariance::Result {
      throw std::runtime_error("unexpected MAIN covariance fallback");
    });
  o.matrix("joint",first.endpoints.joint_current,"cov");
  o.matrix("relative",later.covariance,"cov");
  require((first.relative.covariance-later.covariance).cwiseAbs().maxCoeff()==0,
          "first/revised LOCAL mismatch");
}

void history(Output& o,Geometry& geom,int window,int links,bool audit,
             std::ofstream& times) {
  Main main((window-.5)*.1,params());auto obs=attach(&main);
  auto pre=PreintegrationParams::MakeSharedU(9.81);
  pre->accelerometerCovariance=Matrix3::Identity()*.01;
  pre->gyroscopeCovariance=Matrix3::Identity()*.01;
  pre->integrationCovariance=Matrix3::Identity()*.001;
  imuBias::ConstantBias bias;
  std::map<int,size_t> peers;
  size_t start_free,total_mem,min_free;
  Geometry::cuda(cudaDeviceSynchronize());
  Geometry::cuda(cudaMemGetInfo(&start_free,&total_mem));min_free=start_free;
  const int warm=window+5, count=warm+20;
  for(int k=0;k<count;++k) {
    NonlinearFactorGraph native;
    if(k==0) {
      native.emplace_shared<PriorFactor<Pose3>>(X(k),Pose3(),noise());
      native.emplace_shared<PriorFactor<Vector3>>(V(k),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.1));
      native.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(k),bias,noise());
    } else {
      PreintegratedImuMeasurements pim(pre,bias);
      pim.integrateMeasurement(Vector3(0,0,9.81),Vector3::Zero(),.1);
      native.emplace_shared<ImuFactor>(X(k-1),V(k-1),X(k),V(k),B(k-1),pim);
      native.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(k-1),B(k),bias,noise(.01));
      native.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),Pose3(),noise());
      for(int j=1;j<=std::min(k,links);++j)native.push_back(geom.factor(X(k-j),X(k)));
    }
    obs->admit(native);
    auto graph=native;
    if(k)graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),
      Pose3::Expmap(Vector6::Constant(.0007*(k%3+1))),noise(.04));
    Values v;v.insert(X(k),guess(k));v.insert(V(k),Vector3::Zero().eval());v.insert(B(k),bias);
    FactorIndices remove;
    // At k, oldest retained pose is k-window+1. Remove its incident peer
    // before the older endpoint exits the window in this same native update.
    if(k>=window&&peers.count(k-window+1)) {
      remove.push_back(peers.at(k-window+1));peers.erase(k-window+1);
    }
    double elapsed_cycle=0;
    for(int phase=0;phase<3;++phase) {
      o.prefix=std::to_string(window)+"/"+std::to_string(k)+"/"+std::to_string(phase);
      e1b::Record rec;
      std::unique_ptr<e1b::Session> profile;
      if(audit)profile=std::make_unique<e1b::Session>(k,e1b::Context::Main,&rec);
      Geometry::cuda(cudaDeviceSynchronize());
      const auto begin=std::chrono::steady_clock::now();
      if(phase==0)main.update(graph,v,{{X(k),.1*k},{V(k),.1*k},{B(k),.1*k}},remove);
      else main.update({}, {}, {}, {});
      Geometry::cuda(cudaDeviceSynchronize());
      const double ms=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-begin).count();
      profile.reset();
      elapsed_cycle+=ms;
      require(!main.fallbackHappened(),"unexpected MAIN fallback");
      require(obs->valid(),obs->error());require(obs->resets()==0,"provider reset");
      o.result(main.getISAM2Result());
      if(audit)o.profile(rec);
      if(k&&phase==0)peers[k]=main.getISAM2Result().newFactorsIndices.back();
      size_t active_gpu=0;
      for(const auto& f:main.getFactors())active_gpu+=dynamic_cast<GPU*>(f.get())!=nullptr;
      o.counts("load",{main.getLinearizationPoint().size(),main.getFactors().size(),active_gpu});
      if(k>=warm&&!audit)times<<window<<" "<<k-warm<<" "<<phase<<" "<<ms<<"\n";
    }
    if(k>=warm&&!audit)times<<window<<" "<<k-warm<<" 3 "<<elapsed_cycle<<"\n";
    size_t free_now;Geometry::cuda(cudaMemGetInfo(&free_now,&total_mem));
    min_free=std::min(min_free,free_now);
  }
  o.prefix=std::to_string(window)+"/final";compactFinal(o,main,count-1);
  rusage usage;require(getrusage(RUSAGE_SELF,&usage)==0,"getrusage");
  std::cout<<"MEMORY "<<window<<" "<<start_free-min_free<<" "<<usage.ru_maxrss<<"\n";
}

int main(int argc,char** argv) {
  try {
    require(argc==6,"output times enabled audit frozen_geometry");
    bool enabled=std::string(argv[3])=="1",audit=std::string(argv[4])=="1";
    if(enabled)setenv("CPK16_MAIN_GPU_BATCHING","1",1);
    else unsetenv("CPK16_MAIN_GPU_BATCHING");
    Output o(argv[1]);std::ofstream timing(argv[2]);
    require(bool(timing),"timing output");timing<<std::setprecision(17);
    Geometry geom(argv[5]);
    history(o,geom,5,2,audit,timing);history(o,geom,50,15,audit,timing);
    o.f.flush();timing.flush();require(bool(o.f)&&bool(timing),"write failure");
    std::cout<<"K16C_COMPLETED\n";return 0;
  } catch(const std::exception& e) {
    std::cerr<<"K16C_FAILURE "<<e.what()<<"\n";return 1;
  }
}
