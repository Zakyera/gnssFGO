// K16b: fresh independent processes, actual solver updates, no global GPU hook.
#include "ExactGpuSnapshot.h"
#include "RuntimeProfile.h"
#include "ShadowObserver.h"
#include "RevisionCovariance.h"
#ifdef K16_CANDIDATE
#include "MainGpuBatch.h"
#endif
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <cuda_runtime.h>

using namespace gtsam;
using namespace shared_native;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
using GPU = gtsam_points::IntegratedVGICPFactorGPU;
using Main = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
using Solver = gtsam_points::ISAM2Ext;

void require(bool b, const std::string& message) {
  if (!b) throw std::runtime_error(message);
}
struct Output {
  std::ofstream f;
  std::string prefix;
  explicit Output(const char* path):f(path) {
    require(bool(f),"cannot open output"); f<<std::setprecision(17);
  }
  template<class Range> void exact(const std::string& name, const Range& r) {
    f<<prefix<<"/"<<name<<" exact";
    for (auto x:r) f<<" "<<x;
    f<<"\n";
  }
  void counts(const std::string& name, std::initializer_list<size_t> r) {exact(name,r);}
  void matrix(const std::string& name,const Matrix& m,const char* kind="matrix") {
    require(m.allFinite(),"nonfinite "+prefix+"/"+name);
    f<<prefix<<"/"<<name<<" "<<kind<<" "<<m.rows()<<" "<<m.cols();
    for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)f<<" "<<m(i,j);
    f<<"\n";
  }
  void values(const std::string& name,const Values& v) {
    exact(name+"/keys",v.keys());
    for(auto k:v.keys()) {
      auto tag=name+"/"+std::to_string(k);
      switch(Symbol(k).chr()) {
        case 'x':matrix(tag,v.at<Pose3>(k).matrix(),"pose");break;
        case 'v':matrix(tag,v.at<Vector3>(k),"state");break;
        case 'b':matrix(tag,v.at<imuBias::ConstantBias>(k).vector(),"state");break;
        default:throw std::runtime_error("unexpected state type");
      }
    }
  }
  void gaussian(const std::string& name,const GaussianFactorGraph& graph) {
    counts(name+"/size",{graph.size()});
    for(size_t i=0;i<graph.size();++i) {
      const auto tag=name+"/"+std::to_string(i);
      counts(tag+"/present",{bool(graph[i])});
      if(graph[i]) {
        exact(tag+"/keys",graph[i]->keys());
        matrix(tag+"/H",graph[i]->augmentedInformation());
      }
    }
  }
  void gpu(const std::string& tag, const GPU& factor) {
    auto bytes=gtsam_points::ExactGpuSnapshot::state(factor);
    require(!factor.has_cached_linearization(),"undrained GPU cache");
    f<<prefix<<"/"<<tag<<" bytes ";
    static const char* h="0123456789abcdef";
    for(auto b:bytes)f<<h[b>>4]<<h[b&15];
    f<<"\n";
  }
  void result(const ISAM2Result& r) {
    counts("result/counts",{r.variablesRelinearized,r.variablesReeliminated,
      r.factorsRecalculated,r.cliques});
    exact("result/new_slots",r.newFactorsIndices);
    exact("result/unused",r.unusedKeys);exact("result/observed",r.observedKeys);
    exact("result/removed",r.keysWithRemovedFactors);exact("result/marked",r.markedKeys);
    require(bool(r.detail),"missing detail");
    for(const auto& [k,s]:r.detail->variableStatus)
      counts("result/status/"+std::to_string(k),{s.isReeliminated,s.isAboveRelinThreshold,
        s.isRelinearizeInvolved,s.isRelinearized,s.isObserved,s.isNew,s.inRootClique});
  }
  void profile(const e1b::Record& r) {
    // Counts only; times intentionally not exported or interpreted as a benchmark.
    f<<prefix<<"/profile counts";
    for(auto x:{e1b::Stage::GpuLinearize,e1b::Stage::GpuSyncLinearize,
                e1b::Stage::GpuError,e1b::Stage::GpuSyncError,
                e1b::Stage::IsamBatch,e1b::Stage::IsamIncremental,
                e1b::Stage::IsamMarginalize}) {
      auto i=static_cast<size_t>(x);f<<" "<<r.owner[i].calls+r.worker[i].calls;
    }
    f<<" "<<r.metrics[static_cast<size_t>(e1b::Metric::GpuLinearizeHits)]<<"\n";
  }
};
struct Geometry {
  gtsam_points::PointCloudGPU::Ptr cloud;
  gtsam_points::GaussianVoxelMapGPU::Ptr map;
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> streams;
  static void cuda(cudaError_t code) {require(code==cudaSuccess,cudaGetErrorString(code));}
  template<class T> static void save(std::ostream& out,const T& v) {
    out.write(reinterpret_cast<const char*>(&v),sizeof(T));
  }
  template<class T> static void load(std::istream& in,T& v) {
    in.read(reinterpret_cast<char*>(&v),sizeof(T));
  }
  template<class T> static void saveDevice(std::ostream& out,const T* p,size_t n) {
    std::vector<T> bytes(n);cuda(cudaMemcpy(bytes.data(),p,n*sizeof(T),cudaMemcpyDeviceToHost));
    out.write(reinterpret_cast<const char*>(bytes.data()),n*sizeof(T));
  }
  template<class T> static void loadDevice(std::istream& in,T*& p,size_t n) {
    require(p==nullptr,"fixture destination is not empty");
    std::vector<T> bytes(n);in.read(reinterpret_cast<char*>(bytes.data()),n*sizeof(T));
    require(bool(in),"truncated frozen geometry");
    cuda(cudaMalloc(reinterpret_cast<void**>(&p),n*sizeof(T)));
    cuda(cudaMemcpy(p,bytes.data(),n*sizeof(T),cudaMemcpyHostToDevice));
  }
  explicit Geometry(const char* frozen) {
    std::ifstream in(frozen,std::ios::binary);
    if(in) {
      uint64_t magic=0,n=0;load(in,magic);load(in,n);
      require(magic==0x4b313642&&n==864,"wrong frozen geometry header");
      cloud=std::make_shared<gtsam_points::PointCloudGPU>();cloud->num_points=n;
      loadDevice(in,cloud->points_gpu,n);loadDevice(in,cloud->covs_gpu,n);
      gtsam_points::VoxelMapInfo info;load(in,info);
      require(info.num_voxels>0&&info.num_voxels<=864&&info.num_buckets==16384,"invalid frozen voxel dimensions");
      map=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
      cuda(cudaDeviceSynchronize());map->voxelmap_info=info;
      cuda(cudaMemcpy(map->voxelmap_info_ptr,&info,sizeof(info),cudaMemcpyHostToDevice));
      loadDevice(in,map->buckets,info.num_buckets);loadDevice(in,map->num_points,info.num_voxels);
      loadDevice(in,map->voxel_means,info.num_voxels);loadDevice(in,map->voxel_covs,info.num_voxels);
      loadDevice(in,map->voxel_intensities,info.num_voxels);
      require(in.peek()==std::char_traits<char>::eof(),"trailing frozen geometry bytes");
      streams=std::make_unique<gtsam_points::StreamTempBufferRoundRobin>(4);
      return;
    }
    std::vector<Eigen::Vector4d> pts;std::vector<Eigen::Matrix4d> covs;
    for(int x=0;x<12;++x)for(int y=0;y<12;++y)for(int z=0;z<6;++z) {
      pts.emplace_back(.31*x+.001*y*y,.32*y,.33*z+.002*x*x,1);
      Eigen::Matrix4d c=Eigen::Matrix4d::Identity()*.02;c(3,3)=0;covs.push_back(c);
    }
    auto cpu=std::make_shared<gtsam_points::PointCloudCPU>();
    cpu->add_points(pts);cpu->add_covs(covs);
    cloud=gtsam_points::PointCloudGPU::clone(*cpu);
    map=std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);map->insert(*cloud);
    streams=std::make_unique<gtsam_points::StreamTempBufferRoundRobin>(4);
    cuda(cudaDeviceSynchronize());
    // Freeze actual device bytes, including voxel hash layout and reduction
    // output. Re-inserting identical CPU points is NOT an exact device oracle.
    std::ofstream out(frozen,std::ios::binary);require(bool(out),"cannot freeze geometry");
    save(out,uint64_t(0x4b313642));save(out,uint64_t(cloud->size()));
    saveDevice(out,cloud->points_gpu,cloud->size());saveDevice(out,cloud->covs_gpu,cloud->size());
    save(out,map->voxelmap_info);
    saveDevice(out,map->buckets,map->voxelmap_info.num_buckets);
    saveDevice(out,map->num_points,map->voxelmap_info.num_voxels);
    saveDevice(out,map->voxel_means,map->voxelmap_info.num_voxels);
    saveDevice(out,map->voxel_covs,map->voxelmap_info.num_voxels);
    saveDevice(out,map->voxel_intensities,map->voxelmap_info.num_voxels);
    out.flush();require(bool(out),"geometry write failed");
  }
  std::shared_ptr<GPU> factor(Key a,Key b) {
    auto [stream,temp]=streams->get_stream_buffer();
    auto f=std::make_shared<GPU>(a,b,map,cloud,stream,temp);
    f->set_inlier_update_thresh(.007,.008);return f;
  }
};
SharedNoiseModel noise(double sigma=.1) {
  Matrix6 a=Matrix6::Identity()*sigma;a(3,0)=sigma*.2;a(4,2)=sigma*.1;
  return noiseModel::Gaussian::Covariance(a*a.transpose());
}
ISAM2Params params() {
  ISAM2Params p;p.findUnusedFactorSlots=true;p.cacheLinearizedFactors=true;
  p.relinearizeSkip=1;p.setRelinearizeThreshold(.00001);
  p.evaluateNonlinearError=false;p.enableDetailedResults=true;return p;
}
Pose3 guess(int i) {
  Vector6 d;d<<.001*std::sin(i),.001*std::cos(i),-.001*i,
      .003*std::sin(i),.002*std::cos(i),.001;
  return Pose3::Expmap(d);
}
const NonlinearFactorGraph& factorsOf(const Main& s) {return s.getFactors();}
const NonlinearFactorGraph& factorsOf(const Solver& s) {return s.getFactorsUnsafe();}
const GaussianFactorGraph& cacheOf(const Main& s) {return s.getLinearFactors();}
const GaussianFactorGraph& cacheOf(const Solver& s) {return s.getLinearFactorsUnsafe();}
template<class S> void inspect(Output& o,const S& s,const ISAM2Result& result) {
  o.values("theta",s.getLinearizationPoint());o.values("estimate",s.calculateEstimate());
  o.result(result);o.gaussian("cache",cacheOf(s));
  const auto& factors=factorsOf(s);o.counts("nonlinear/size",{factors.size()});
  for(size_t i=0;i<factors.size();++i) {
    auto tag="nonlinear/"+std::to_string(i);
    o.counts(tag+"/present",{bool(factors[i])});
    if(factors[i]) {
      o.exact(tag+"/keys",factors[i]->keys());
      if(auto* g=dynamic_cast<GPU*>(factors[i].get()))o.gpu(tag+"/gpu",*g);
    }
  }
}
// Independently owned wrapper without observer: the opt-in must not activate it.
void unattached(Output& o,Geometry& geom) {
  Main main(.5,params());
  NonlinearFactorGraph g;
  g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise());
  g.push_back(geom.factor(X(0),X(1)));
  g.emplace_shared<BetweenFactor<Pose3>>(X(0),X(1),Pose3(),noise());
  Values v;v.insert(X(0),guess(0));v.insert(X(1),guess(1));
  e1b::Record rec;{e1b::Session s(0,e1b::Context::Main,&rec);main.update(g,v,{{X(0),0},{X(1),.1}});}
  require(!lookup(&main),"unattached owner registered");
  require(rec.metrics[static_cast<size_t>(e1b::Metric::GpuLinearizeHits)]==0,"cross-owner activation");
  o.prefix="unattached";inspect(o,main,main.getISAM2Result());o.profile(rec);
}
void wrapper(Output& o,Geometry& geom,bool enabled) {
  Main main(.45,params());auto obs=attach(&main);
  const auto pre=PreintegrationParams::MakeSharedU(9.81);
  pre->accelerometerCovariance=Matrix3::Identity()*.01;
  pre->gyroscopeCovariance=Matrix3::Identity()*.01;
  pre->integrationCovariance=Matrix3::Identity()*.001;
  const imuBias::ConstantBias bias;
  size_t calls=0,queries=0,hits=0,retirements=0,reused=0,extra_queries=0;
  std::vector<NonlinearFactor::shared_ptr> owned;std::map<const NonlinearFactor*,size_t> ids;
  std::vector<NonlinearFactor::shared_ptr> admitted;
  std::set<const NonlinearFactor*> external;
  std::map<int,size_t> peer_slots;
  std::set<size_t> past_removed;
  std::map<size_t,size_t> pair_versions;
  obs->setAudit([&](const Snapshot& s) {
    o.exact("snapshot/retiring",s.retiring);o.exact("snapshot/removed",s.removed);
    retirements+=!s.retiring.empty();
    for(const auto& ref:s.native) {
      // The hook is invoked AFTER retirement but s is the saved BEFORE snapshot.
      // A retired slot may already contain a new marginal prior. Use the
      // positive admission identity, not the post-retirement slot occupant.
      const auto& f=admitted.at(ref.id-1);
      require(!external.count(f.get()),"CBS factor admitted into native cache");
      require(f->keys()==s.cache.at(ref.slot)->keys(),"snapshot keys differ from admitted input");
      bool retiring=false;for(auto key:f->keys())retiring|=s.retiring.count(key)>0;
      if(!retiring)require(main.getFactors().at(ref.slot)==f,"surviving slot identity changed");
      o.counts("snapshot/native/"+std::to_string(ref.id),{ref.version,ref.slot,ids.at(f.get())});
      o.matrix("snapshot/H/"+std::to_string(ref.id),s.cache.at(ref.slot)->augmentedInformation());
    }
  });
  auto query=[&] {
    const auto states=main.calculateEstimate();auto keys=states.keys();
    std::vector<Key> poses;for(auto k:keys)if(Symbol(k).chr()=='x')poses.push_back(k);
    e1b::Record rec;
    {e1b::Session s(0,e1b::Context::Covariance,&rec);
      for(size_t i=1;i<poses.size();++i) {
        auto a=states.at<Pose3>(poses[i-1]),b=states.at<Pose3>(poses[i]);
        const auto tag="publication/"+std::to_string(poses[i]);
        PublishedCovariance first;
        auto result=publicationCovariance(&main,poses[i-1],poses[i],a,b,first);
        require(result.ok,result.message);
        auto later=revisionCovariance(RevisionCovariancePolicy::NativeLocal,
          &main,poses[i-1],poses[i],a,b,[&]()->cbs::relative_pose_covariance::Result {
            ++extra_queries;throw std::runtime_error("unexpected MAIN covariance query");
          });
        require((first.relative.covariance-later.covariance).cwiseAbs().maxCoeff()==0,"first/revision mismatch");
        o.matrix(tag+"/mean",a.between(b).matrix(),"pose");
        o.matrix(tag+"/joint",first.endpoints.joint_current,"cov");
        o.matrix(tag+"/relative",later.covariance,"cov");
        o.counts(tag+"/version",{pair_versions[poses[i]]++});++queries;
      }
    }
    require(rec.owner[static_cast<size_t>(e1b::Stage::GpuLinearize)].calls==0,"query reevaluated GPU");
    require(rec.owner[static_cast<size_t>(e1b::Stage::GpuError)].calls==0,"query called GPU error");
    require(extra_queries==0,"query used MAIN");
    o.counts("native/conservation",{obs->history().admittedCount(),obs->history().activeCount(),
      obs->history().absorbedCount(),obs->history().removedCount(),obs->history().queryPreparations()});
    o.gaussian("native/history",obs->history().graph(obs->history().epoch()));
    require(obs->history().admittedCount()==obs->history().activeCount()+
      obs->history().absorbedCount()+obs->history().removedCount(),"native conservation");
  };
  auto update=[&](std::string phase,const NonlinearFactorGraph& f,const Values& v,
                  const Main::KeyTimestampMap& stamps,const FactorIndices& remove) {
    o.prefix="wrapper/"+std::to_string(calls++)+"/"+phase;
    for(auto slot:remove)past_removed.insert(slot);
    e1b::Record rec;
    {e1b::Session s(0,e1b::Context::Main,&rec);main.update(f,v,stamps,remove);}
    require(!main.fallbackHappened(),"unexpected MAIN fallback");
    require(obs->valid(),obs->error());require(obs->resets()==0,"provider reset");
    hits+=rec.metrics[static_cast<size_t>(e1b::Metric::GpuLinearizeHits)];
    for(auto slot:main.getISAM2Result().newFactorsIndices)reused+=past_removed.count(slot);
    inspect(o,main,main.getISAM2Result());o.profile(rec);
    for(size_t j=0;j<main.getFactors().size();++j)if(main.getFactors()[j]) {
      auto* ptr=main.getFactors()[j].get();
      // Positive source identity; synthesized marginal priors have no input ID.
      o.counts("factor_identity/"+std::to_string(j),{ids.count(ptr)?ids.at(ptr):0,external.count(ptr)});
    }
    query();
  };
  auto remember=[&](const NonlinearFactorGraph& g,bool peer) {
    for(auto& f:g) {
      owned.push_back(f);ids.emplace(f.get(),owned.size());
      if(peer)external.insert(f.get());else admitted.push_back(f);
    }
  };
  for(int k=0;k<24;++k) {
    NonlinearFactorGraph native;
    if(!k) {
      native.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise());
      native.emplace_shared<PriorFactor<Vector3>>(V(0),Vector3::Zero(),noiseModel::Isotropic::Sigma(3,.1));
      native.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(0),bias,noise());
    } else {
      PreintegratedImuMeasurements pim(pre,bias);
      pim.integrateMeasurement(Vector3(0,0,9.81),Vector3::Zero(),.1);
      native.emplace_shared<ImuFactor>(X(k-1),V(k-1),X(k),V(k),B(k-1),pim);
      native.push_back(geom.factor(X(k-1),X(k)));
      native.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(B(k-1),B(k),bias,noise(.01));
      native.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),Pose3(),noise());
      if(k>1)native.push_back(geom.factor(X(k-2),X(k)));
    }
    remember(native,false);obs->admit(native);
    auto all=native;
    if(k) {
      NonlinearFactorGraph peer;
      peer.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),
        Pose3::Expmap(Vector6::Constant(.0007*(k%3+1))),noise(.04));
      remember(peer,true);all.push_back(peer);
    }
    Values v;v.insert(X(k),guess(k));v.insert(V(k),Vector3::Zero().eval());v.insert(B(k),bias);
    FactorIndices remove;
    if(k>=3&&peer_slots.count(k-3)){remove.push_back(peer_slots.at(k-3));peer_slots.erase(k-3);}
    update("arrival",all,v,{{X(k),.1*k},{V(k),.1*k},{B(k),.1*k}},remove);
    if(k)peer_slots[k]=main.getISAM2Result().newFactorsIndices.back();
    if(k&&k%4==0) {
      NonlinearFactorGraph revised;
      revised.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),
        Pose3::Expmap(Vector6::Constant(-.0004)),noise(.04));
      remember(revised,true);
      update("peer_replace",revised,{}, {},{peer_slots.at(k)});
      peer_slots[k]=main.getISAM2Result().newFactorsIndices.front();
    }
    update("relinearize",{}, {}, {},{});
    update("cached",{}, {}, {},{});
    if(k==10)unattached(o,geom);
  }
  require(retirements>0&&reused>0&&queries>100,"missing wrapper coverage");
  require(enabled?hits>0:hits==0,"wrapper activation disagrees with mode");
  o.prefix="wrapper";o.counts("coverage",{calls,queries,retirements,reused});
  std::cout<<"WRAPPER calls="<<calls<<" queries="<<queries<<" retirements="<<retirements
    <<" reused="<<reused<<" prepared_hits="<<hits<<"\n";
}

void direct(Output& o,Geometry& geom,bool enabled) {
  Solver main(params());
#ifdef K16_CANDIDATE
  cpk16::Stats stats;
  cpk16::Scope scope(&main,enabled,&stats);
#endif
  // Real update calls, including a forced full reorder; never call tryLinearize.
  for(int k=0;k<24;++k) {
    NonlinearFactorGraph g;
    if(!k)g.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise());
    else {
      g.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),Pose3(),noise());
      g.push_back(geom.factor(X(k-1),X(k)));
      if(k>1)g.push_back(geom.factor(X(k-2),X(k)));
    }
    Values v;v.insert(X(k),guess(k));
    ISAM2UpdateParams p;
    if(k==16) {
      p.extraReelimKeys=FastList<Key>();
      for(auto key:main.getLinearizationPoint().keys())p.extraReelimKeys->push_back(key);
      p.forceFullSolve=true;
    }
    e1b::Record rec;ISAM2Result r;
    {e1b::Session s(k,e1b::Context::Main,&rec);r=main.update(g,v,p);}
    o.prefix="direct/"+std::to_string(k);inspect(o,main,r);o.profile(rec);
  }
  // Explicit constrain-then-retire of a leaf in a multi-frontal clique.
  ISAM2UpdateParams p;p.extraReelimKeys=FastList<Key>();p.constrainedKeys=FastMap<Key,int>();
  for(auto key:main.getLinearizationPoint().keys()) {
    p.extraReelimKeys->push_back(key);(*p.constrainedKeys)[key]=key==X(0)?0:1;
  }
  main.update({}, {},p);main.calculateEstimate();
  FactorIndices marginal,deleted;
  e1b::Record rec;
  {e1b::Session s(30,e1b::Context::Main,&rec);main.marginalizeLeaves({X(0)},&marginal,&deleted);}
  o.prefix="direct/retire";o.exact("marginal_slots",marginal);o.exact("deleted_slots",deleted);
  o.values("estimate",main.calculateEstimate());o.gaussian("cache",cacheOf(main));o.profile(rec);
#ifdef K16_CANDIDATE
  o.prefix="direct";o.f<<o.prefix<<"/sites counts";
  for(auto n:stats.sites)o.f<<" "<<n;
  o.f<<"\n";
  std::cout<<"DIRECT sites";
  for(auto n:stats.sites)std::cout<<" "<<n;
  std::cout<<" prepared="<<stats.work.prepared_factors<<"\n";
#endif
}
void partialClique(Output& o,Geometry& geom,bool enabled) {
  Solver main(params());
#ifdef K16_CANDIDATE
  cpk16::Stats stats;cpk16::Scope scope(&main,enabled,&stats);
#endif
  NonlinearFactorGraph graph;Values values;
  for(int i=0;i<3;++i)values.insert(X(i),guess(i));
  graph.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise());
  for(int i=0;i<3;++i)for(int j=i+1;j<3;++j) {
    graph.push_back(geom.factor(X(i),X(j)));
    graph.emplace_shared<BetweenFactor<Pose3>>(X(i),X(j),Pose3(),noise());
  }
  ISAM2UpdateParams p;p.constrainedKeys=FastMap<Key,int>();
  for(int i=0;i<3;++i)(*p.constrainedKeys)[X(i)]=i?1:0;
  main.update(graph,values,p);main.calculateEstimate();
  FactorIndices marginal,deleted;main.marginalizeLeaves({X(0)},&marginal,&deleted);
  o.prefix="partial";o.exact("marginal_slots",marginal);o.exact("deleted_slots",deleted);
  o.values("theta",main.getLinearizationPoint());o.values("estimate",main.calculateEstimate());
  o.gaussian("cache",cacheOf(main));
  for(size_t i=0;i<factorsOf(main).size();++i)
    if(auto* gpu=dynamic_cast<GPU*>(factorsOf(main)[i].get()))o.gpu("gpu/"+std::to_string(i),*gpu);
#ifdef K16_CANDIDATE
  o.f<<"partial/sites counts";for(auto n:stats.sites)o.f<<" "<<n;o.f<<"\n";
  std::cout<<"PARTIAL sites";for(auto n:stats.sites)std::cout<<" "<<n;std::cout<<"\n";
  if(enabled)require(stats.sites[static_cast<size_t>(cpk16::Site::Marginalize)]>0,
                     "partial-clique marginalization path was not reached");
#endif
}
int main(int argc,char** argv) {
  try {
    require(argc==4,"usage: executable output enabled frozen_geometry");
    const bool enabled=std::string(argv[2])=="1";
    if(enabled)setenv("CPK16_MAIN_GPU_BATCHING","1",1);
    else unsetenv("CPK16_MAIN_GPU_BATCHING");
    Output o(argv[1]);Geometry geometry(argv[3]);
    o.prefix="input";o.gpu("geometry",*geometry.factor(X(0),X(1)));
    wrapper(o,geometry,enabled);direct(o,geometry,enabled);partialClique(o,geometry,enabled);
    o.f.flush();require(bool(o.f),"output failure");
    std::cout<<"K16B_COMPLETED\n";return 0;
  } catch(const std::exception& e) {
    std::cerr<<"K16B_FAILURE "<<e.what()<<"\n";return 1;
  }
}
