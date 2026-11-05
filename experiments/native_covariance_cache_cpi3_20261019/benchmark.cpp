// Saved-equation timing only: never runs an estimator or opens a ROS bag.
#include "JournalReader.h"
#include "OracleHistory.h"
#include <gtsam/inference/Symbol.h>

uncached_native::Snapshot oracleSnapshot(const Snapshot& s) {
  uncached_native::Snapshot o;
  o.generation=s.generation; o.epoch=s.epoch; o.cache=s.cache; o.theta=s.theta;
  o.retiring=s.retiring; o.fixed_after=s.fixed_after; o.removed=s.removed;
  for(auto& r:s.native)o.native.push_back({r.id,r.version,r.slot});
  return o;
}
template<class A,class B> double difference(const A& a,const B& b) {
  return std::max({error(a.joint_theta,b.joint_theta),error(a.joint_current,b.joint_current),
                  error(a.transport,b.transport),error(a.relative,b.relative)});
}
int main(int argc,char** argv) {
 try {
  check(argc==2,"usage: benchmark saved_native_journal");
  std::cout<<std::setprecision(17);
  Reader r(argv[1]); NativeGaussianHistory h(Mode::Shadow);
  Id generation=0; Key from=0,to=0; Vector6 df,dt;
  size_t batches=0,pairs=0; double worst=0,recorded_worst=0;
  while(true) {
    int tag=r.stream.get(); if(tag==EOF) break; Id epoch=r.u();
    if(tag=='S') {
      auto s=r.snapshot(epoch); generation=s.generation;
      auto result=h.stage(s); check(result.ok,result.message.c_str());
    } else if(tag=='C') {
      auto admitted=r.u(),active=r.u(),absorbed=r.u(),removed=r.u();r.scalar();
      auto result=h.commit(generation,epoch);check(result.ok,result.message.c_str());
      check(admitted==h.admittedCount()&&active==h.activeCount()&&absorbed==h.absorbedCount()&&removed==h.removedCount(),"conservation");
    } else if(tag=='P') {
      from=r.u();to=r.u();r.mat();r.mat();df=r.mat();dt=r.mat();
      auto n=r.count();GaussianFactorGraph expected;
      for(size_t i=0;i<n;++i)expected.push_back(r.factor());
      check(error(h.graph(epoch).augmentedHessian(),expected.augmentedHessian())<1e-9,"native equations changed");
    } else if(tag=='Q') {
      auto joint=r.mat(),transport=r.mat(),current=r.mat(),relative=r.mat();r.scalar();
      auto actual=h.covariance(epoch,from,to,df,dt);
      double saved_error=std::max({error(actual.joint_theta,joint),error(actual.transport,transport),error(actual.joint_current,current),error(actual.relative,relative)});
      check(saved_error<1e-8,"saved covariance mismatch");recorded_worst=std::max(recorded_worst,saved_error);
      // All native pose pairs i->i+2 at each recorded query epoch. Same real
      // Gaussian equations; zero chart deltas for the added workload, NOT
      // a reconstruction of missing live revision messages or their timing.
      Snapshot s;s.cache=h.graph(epoch);s.theta=h.theta(epoch);
      for(size_t i=0;i<s.cache.size();++i)s.native.push_back({i+1,1,i});
      std::vector<Key> poses;for(auto k:s.theta.keys())if(Symbol(k).chr()=='x')poses.push_back(k);
      std::vector<std::pair<Key,Key>> endpoints{{from,to}};
      for(size_t i=0;i+2<poses.size();++i)endpoints.emplace_back(poses[i],poses[i+2]);
      for(int pass=0;pass<4;++pass) {
        NativeGaussianHistory cached(Mode::Shadow);
        uncached_native::NativeGaussianHistory original(uncached_native::Mode::Shadow);
        check(cached.stage(s).ok&&cached.commit(1,1).ok,"cached setup");
        check(original.stage(oracleSnapshot(s)).ok&&original.commit(1,1).ok,"original setup");
        std::vector<EndpointCovariance> a;std::vector<uncached_native::EndpointCovariance> b;
        double ca=0,un=0;
        auto run_cached=[&]{auto t=Clock::now();for(size_t i=0;i<endpoints.size();++i) {
          (void)cached.theta(1);
          a.push_back(cached.covariance(1,endpoints[i].first,endpoints[i].second,i?Vector6::Zero().eval():df,i?Vector6::Zero().eval():dt));
        }ca=elapsed(t);};
        auto run_original=[&]{auto t=Clock::now();for(size_t i=0;i<endpoints.size();++i) {
          (void)original.theta(1);
          b.push_back(original.covariance(1,endpoints[i].first,endpoints[i].second,i?Vector6::Zero().eval():df,i?Vector6::Zero().eval():dt));
        }un=elapsed(t);};
        if(pass%2){run_cached();run_original();}else{run_original();run_cached();}
        double d=0;for(size_t i=0;i<a.size();++i)d=std::max(d,difference(a[i],b[i]));
        check(d<=1e-12,"cached vs original covariance mismatch");
        check(cached.queryPreparations()==1,"not one preparation per epoch");
        worst=std::max(worst,d);pairs+=a.size();++batches;
        row("batch",epoch,{{"cached_s",ca},{"original_s",un}},{{"pass",double(pass)},{"pairs",double(a.size())},{"max_covariance_relative_error",d},{"preparations",double(cached.queryPreparations())}});
        std::cout.flush();
      }
    } else throw std::runtime_error("unknown journal tag");
  }
  check(batches>0,"no queries");
  row("summary",h.epoch(),{},{{"batches",double(batches)},{"pairs",double(pairs)},{"max_covariance_relative_error",worst},{"recorded_max_covariance_relative_error",recorded_worst}});
  return 0;
 }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
