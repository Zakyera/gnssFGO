// Bounded diagnostic only: one source, independently owned same-input companion.
#include <cuda_runtime.h>
#include "ExactGpuSnapshot.h"
#include "RuntimeProfile.h"
#include "ShadowObserver.h"
#include "RevisionCovariance.h"
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/factors/integrated_vgicp_derivatives.cuh>
#include <gtsam_points/cuda/kernels/linearized_system.cuh>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <dlfcn.h>
#include <unistd.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <set>
#include <sstream>

using Main = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
using GPU = gtsam_points::IntegratedVGICPFactorGPU;
using Snap = gtsam_points::ExactGpuSnapshot;
using namespace gtsam;
namespace fs = std::filesystem;
namespace {
thread_local bool companion_route = false;
thread_local bool observing = false;
std::mutex lock;
void require(bool b,const std::string& message) {
  if(!b) throw std::runtime_error(message);
}
void cuda(cudaError_t r) {require(r==cudaSuccess,cudaGetErrorString(r));}
template<class T> void append(std::vector<unsigned char>& b,const T& v) {
  const auto* p=reinterpret_cast<const unsigned char*>(&v);b.insert(b.end(),p,p+sizeof(v));
}
template<class M> void appendMatrix(std::vector<unsigned char>& b,const M& m) {
  for(int c=0;c<m.cols();++c)for(int r=0;r<m.rows();++r)append(b,m(r,c));
}
using Update = Main::Result (*)(Main*,const NonlinearFactorGraph&,const Values&,
                               const Main::KeyTimestampMap&,const FactorIndices&);
Update nextUpdate() {
  static auto fn=reinterpret_cast<Update>(dlsym(RTLD_NEXT,
    "_ZN12gtsam_points42IncrementalFixedLagSmootherExtWithFallback6updateERKN5gtsam20NonlinearFactorGraphERKNS1_6ValuesERKSt3mapImdSt4lessImESaISt4pairIKmdEEERKSt6vectorImSaImEE"));
  require(fn,"missing frozen MAIN update symbol");return fn;
}
struct Compare {
  double matrix_max=0,cov_max=0,p_max=0,r_max=0,v_max=0,b_max=0;
  size_t matrices=0,covariances=0,gpus=0,queries=0;
  void matrix(const Matrix& a,const Matrix& b,bool covariance=false) {
    require(a.rows()==b.rows()&&a.cols()==b.cols(),"matrix shape");
    require(a.allFinite()&&b.allFinite(),"nonfinite matrix");
    if(!a.size()) return;
    const double d=(a-b).cwiseAbs().maxCoeff();
    const double bound=(covariance?1e-10:1e-8)+1e-6*a.cwiseAbs().maxCoeff();
    if(covariance){cov_max=std::max(cov_max,d);++covariances;}
    else{matrix_max=std::max(matrix_max,d);++matrices;}
    require(d<=bound,"Gaussian/covariance mismatch: "+std::to_string(d));
  }
  void pose(const Pose3& a,const Pose3& b) {
    const double p=(a.translation()-b.translation()).cwiseAbs().maxCoeff();
    const double r=Rot3::Logmap(a.rotation().between(b.rotation())).norm();
    p_max=std::max(p_max,p);r_max=std::max(r_max,r);
    require(p<=1e-9&&r<=1e-10,"pose mismatch");
  }
  void values(const Values& a,const Values& b) {
    require(a.keys()==b.keys(),"state keys");
    for(auto k:a.keys())switch(Symbol(k).chr()) {
      case 'x':pose(a.at<Pose3>(k),b.at<Pose3>(k));break;
      case 'v':{
        double d=(a.at<Vector3>(k)-b.at<Vector3>(k)).cwiseAbs().maxCoeff();
        v_max=std::max(v_max,d);require(d<=1e-9,"velocity mismatch");break;
      }
      case 'b':{
        double d=(a.at<imuBias::ConstantBias>(k).vector()-
                  b.at<imuBias::ConstantBias>(k).vector()).cwiseAbs().maxCoeff();
        b_max=std::max(b_max,d);require(d<=1e-9,"bias mismatch");break;
      }
      default:throw std::runtime_error("unexpected state key");
    }
  }
  void gaussian(const GaussianFactorGraph& a,const GaussianFactorGraph& b) {
    require(a.size()==b.size(),"Gaussian slot count");
    for(size_t i=0;i<a.size();++i) {
      require(bool(a[i])==bool(b[i]),"Gaussian null slot");
      if(a[i]) {
        require(a[i]->keys()==b[i]->keys(),"Gaussian key order");
        matrix(a[i]->augmentedInformation(),b[i]->augmentedInformation());
      }
    }
  }
  void results(const ISAM2Result& a,const ISAM2Result& b) {
    require(a.variablesRelinearized==b.variablesRelinearized &&
      a.variablesReeliminated==b.variablesReeliminated &&
      a.factorsRecalculated==b.factorsRecalculated && a.cliques==b.cliques &&
      a.newFactorsIndices==b.newFactorsIndices && a.unusedKeys==b.unusedKeys &&
      a.observedKeys==b.observedKeys && a.keysWithRemovedFactors==b.keysWithRemovedFactors &&
      a.markedKeys==b.markedKeys,"ISAM2 decisions differ");
  }
};
struct Paired {
  Main* owner;
  std::unique_ptr<Main> candidate;
  std::shared_ptr<shared_native::ShadowObserver> ref_provider,candidate_provider;
  e1b4check::FactorCopies copies;
  std::map<const NonlinearFactor*,size_t> ref_ids,test_ids;
  std::ofstream log;
  size_t updates=0,inputs=0,initial_gpu_checks=0,retirement_updates=0;
  size_t sync_calls=0,prepared_hits=0,total_queries=0;
  double first_stamp=0,last_stamp=0;
  fs::path root;
  Compare totals;
  Paired(Main* source):owner(source) {
    root=std::getenv("CPK16F_PROBE_ROOT");
    require(fs::exists(root)&&!fs::exists(root/"updates.jsonl"),"diagnostic root unavailable/reused");
    require(source->getFactors().empty(),"observer attached after initialization");
    require(!source->params().evaluateNonlinearError,"unexpected nonlinear error mode");
    candidate=std::make_unique<Main>(source->smootherLag(),source->params());
    ref_provider=shared_native::lookup(owner);require(bool(ref_provider),"missing actual provider");
    candidate_provider=shared_native::attach(candidate.get());
    log.open(root/"updates.jsonl");log<<std::setprecision(17);
  }
  void snapshot(const shared_native::Snapshot& a,const shared_native::Snapshot& b) {
    require(a.retiring==b.retiring&&a.removed==b.removed,"retirement mismatch");
    require(a.native.size()==b.native.size(),"native input identity count");
    for(size_t i=0;i<a.native.size();++i)
      require(a.native[i].id==b.native[i].id&&a.native[i].version==b.native[i].version&&
              a.native[i].slot==b.native[i].slot,"native ID/revision/slot");
    totals.gaussian(a.cache,b.cache);
    if(!a.retiring.empty())++retirement_updates;
  }
  void inspect() {
    require(!owner->fallbackHappened()&&!candidate->fallbackHappened(),"unexpected fallback");
    require(ref_provider->valid(),ref_provider->error());
    require(candidate_provider->valid(),candidate_provider->error());
    require(ref_provider->resets()==0&&candidate_provider->resets()==0,"provider reset");
    totals.values(owner->getLinearizationPoint(),candidate->getLinearizationPoint());
    auto a=owner->calculateEstimate(),b=candidate->calculateEstimate();
    totals.values(a,b);
    totals.results(owner->getISAM2Result(),candidate->getISAM2Result());
    totals.gaussian(owner->getLinearFactors(),candidate->getLinearFactors());
    const auto& fg=owner->getFactors();const auto& tg=candidate->getFactors();
    require(fg.size()==tg.size(),"nonlinear slot count");
    for(size_t i=0;i<fg.size();++i) {
      require(bool(fg[i])==bool(tg[i]),"nonlinear null slot");
      if(!fg[i])continue;
      require(typeid(*fg[i])==typeid(*tg[i])&&fg[i]->keys()==tg[i]->keys(),"factor type/keys");
      const auto f=ref_ids.find(fg[i].get()),t=test_ids.find(tg[i].get());
      require((f==ref_ids.end())==(t==test_ids.end()),"factor admission identity");
      if(f!=ref_ids.end())require(f->second==t->second,"factor stable identity");
      if(auto* g=dynamic_cast<GPU*>(fg[i].get())) {
        auto* h=dynamic_cast<GPU*>(tg[i].get());
        require(!g->has_cached_linearization()&&!h->has_cached_linearization(),"pending GPU cache");
        require(Snap::runtimeState(*g)==Snap::runtimeState(*h),"GPU runtime inlier/cache mismatch");
        ++totals.gpus;
      }
    }
    snapshot(ref_provider->snapshot(),candidate_provider->snapshot());
    const auto& rh=ref_provider->history();const auto& th=candidate_provider->history();
    require(rh.epoch()==th.epoch()&&rh.admittedCount()==th.admittedCount()&&
      rh.activeCount()==th.activeCount()&&rh.absorbedCount()==th.absorbedCount()&&
      rh.removedCount()==th.removedCount(),"LOCAL history conservation mismatch");
    require(rh.admittedCount()==rh.activeCount()+rh.absorbedCount()+rh.removedCount(),
      "LOCAL input conservation");
    totals.gaussian(rh.graph(rh.epoch()),th.graph(th.epoch()));
    std::vector<Key> keys;for(auto k:a.keys())if(Symbol(k).chr()=='x')keys.push_back(k);
    e1b::Record query_record;
    {e1b::Session session(0,e1b::Context::Covariance,&query_record);
      for(size_t i=2;i<keys.size();++i) {
        const Key l=keys[i-2],r=keys[i];
        shared_native::PublishedCovariance ca,cb;
        auto qa=shared_native::publicationCovariance(owner,l,r,a.at<Pose3>(l),a.at<Pose3>(r),ca);
        auto qb=shared_native::publicationCovariance(candidate.get(),l,r,b.at<Pose3>(l),b.at<Pose3>(r),cb);
        require(qa.ok,qa.message);require(qb.ok,qb.message);
        totals.matrix(ca.endpoints.joint_current,cb.endpoints.joint_current,true);
        totals.matrix(ca.relative.covariance,cb.relative.covariance,true);
        auto fail=[]()->cbs::relative_pose_covariance::Result{
          throw std::runtime_error("unexpected MAIN revision covariance fallback");};
        auto ra=shared_native::revisionCovariance(shared_native::RevisionCovariancePolicy::NativeLocal,
            owner,l,r,a.at<Pose3>(l),a.at<Pose3>(r),fail);
        auto rb=shared_native::revisionCovariance(shared_native::RevisionCovariancePolicy::NativeLocal,
            candidate.get(),l,r,b.at<Pose3>(l),b.at<Pose3>(r),fail);
        totals.matrix(ra.covariance,rb.covariance,true);
        require((ca.relative.covariance-ra.covariance).cwiseAbs().maxCoeff()==0 &&
                (cb.relative.covariance-rb.covariance).cwiseAbs().maxCoeff()==0,
                "first/revision covariance changed");
        totals.pose(a.at<Pose3>(l).between(a.at<Pose3>(r)),
                    b.at<Pose3>(l).between(b.at<Pose3>(r)));
        ++total_queries;
      }
    }
    require(query_record.owner[size_t(e1b::Stage::GpuLinearize)].calls==0&&
            query_record.owner[size_t(e1b::Stage::GpuError)].calls==0,"LOCAL reevaluates GPU");
    ++updates;
    log<<"{\"update\":"<<updates<<",\"stamp\":"<<last_stamp
       <<",\"states\":"<<keys.size()<<",\"nonlinear_slots\":"<<fg.size()
       <<",\"inputs\":"<<inputs<<",\"initial_gpu_checks\":"<<initial_gpu_checks
       <<",\"retirement_updates\":"<<retirement_updates<<",\"pair_queries\":"<<total_queries
       <<",\"sync_calls\":"<<sync_calls<<",\"prepared_hits\":"<<prepared_hits
       <<",\"gaussian_checks\":"<<totals.matrices<<",\"covariance_checks\":"<<totals.covariances
       <<",\"gpu_runtime_checks\":"<<totals.gpus<<",\"p_max\":"<<totals.p_max
       <<",\"r_max\":"<<totals.r_max<<",\"v_max\":"<<totals.v_max<<",\"b_max\":"<<totals.b_max
       <<",\"gaussian_max\":"<<totals.matrix_max<<",\"covariance_max\":"<<totals.cov_max
       <<",\"passed\":true}\n";log.flush();
  }
};
std::unique_ptr<Paired> paired;
}
// Only the test companion enables batching. No process-wide env mutation.
namespace cpk16 {bool requested(){return companion_route;}}
namespace gtsam_points {
std::vector<unsigned char> ExactGpuSnapshot::runtimeState(const GPU& f) {
  cuda(cudaDeviceSynchronize());
  std::vector<unsigned char> b;const auto& d=*f.derivatives;
  append(b,f.is_binary);for(auto k:f.keys())append(b,k);
  appendMatrix(b,f.fixed_target_pose.matrix());append(b,f.linearized);
  appendMatrix(b,f.linearization_point.matrix());append(b,bool(f.evaluation_result));
  if(f.evaluation_result)append(b,*f.evaluation_result);
  append(b,bool(f.linearization_result));
  if(f.linearization_result) {
    const auto& l=*f.linearization_result;
    appendMatrix(b,l.H_target);appendMatrix(b,l.H_target_source);appendMatrix(b,l.H_source);
    appendMatrix(b,l.b_target);appendMatrix(b,l.b_source);append(b,l.error);append(b,l.num_inliers);
  }
  append(b,d.enable_offloading);append(b,d.enable_surface_validation);
  append(b,d.inlier_update_thresh_trans);append(b,d.inlier_update_thresh_angle);
  append(b,d.num_inliers);append(b,bool(d.source_inliers));
  if(d.source_inliers) {
    appendMatrix(b,d.inlier_evaluation_point.matrix());append(b,bool(d.inlier_evaluation_point_gpu));
    std::vector<int> indices(d.num_inliers);
    cuda(cudaMemcpy(indices.data(),d.source_inliers,indices.size()*sizeof(int),cudaMemcpyDeviceToHost));
    for(auto i:indices)append(b,i);
  }
  return b;
}
Main::Result Main::update(const NonlinearFactorGraph& factors,const Values& values,
    const KeyTimestampMap& stamps,const FactorIndices& remove) {
  if(observing||!shared_native::lookup(this))
    return nextUpdate()(this,factors,values,stamps,remove);
  std::lock_guard<std::mutex> guard(lock);
  observing=true;
  try {
    if(!paired)paired=std::make_unique<Paired>(this);
    auto& p=*paired;require(p.owner==this,"unexpected second attached MAIN");
    require(p.updates<180,"8s diagnostic update limit");
    if(!stamps.empty()) {
      double t=stamps.begin()->second;
      if(!p.first_stamp)p.first_stamp=t;
      p.last_stamp=t;require(t-p.first_stamp<8.1,"diagnostic physical-window bound");
    }
    auto copied=p.copies.graph(factors);
    for(size_t i=0;i<factors.size();++i) {
      require(factors[i]&&copied[i],"unexpected null new factor");
      auto fresh=p.ref_ids.emplace(factors[i].get(),p.inputs+1);
      if(fresh.second) {
        ++p.inputs;p.test_ids.emplace(copied[i].get(),p.inputs);
        if(auto* g=dynamic_cast<GPU*>(factors[i].get())) {
          require(Snap::state(*g)==Snap::state(static_cast<GPU&>(*copied[i])),
                  "exact prepared GPU input snapshot differs");++p.initial_gpu_checks;
        } else {
          // Native IMU preintegration checks |dt_a-dt_b| < tol. tol=0
          // rejects even self-equality. This is a clone sanity check, NOT
          // the separately enforced Gaussian/state/covariance acceptance gate.
          require(factors[i]->equals(*copied[i],1e-15),"CPU input clone differs");
        }
      }
    }
    p.candidate_provider->admit(copied);
    const Values copied_values(values);
    const KeyTimestampMap copied_stamps(stamps);
    const FactorIndices copied_remove(remove);
    e1b::Record r,t;
    Main::Result result;
    {e1b::Session session(0,e1b::Context::Main,&r);
      result=nextUpdate()(this,factors,values,stamps,remove);}
    companion_route=true;
    {e1b::Session session(0,e1b::Context::Main,&t);
      nextUpdate()(p.candidate.get(),copied,copied_values,copied_stamps,copied_remove);}
    companion_route=false;
    require(r.metrics[size_t(e1b::Metric::GpuLinearizeHits)]==0,"reference batching enabled");
    const size_t rc=r.owner[size_t(e1b::Stage::GpuSyncLinearize)].calls+
                    r.worker[size_t(e1b::Stage::GpuSyncLinearize)].calls;
    const size_t tc=t.owner[size_t(e1b::Stage::GpuSyncLinearize)].calls+
                    t.worker[size_t(e1b::Stage::GpuSyncLinearize)].calls;
    const size_t hits=t.metrics[size_t(e1b::Metric::GpuLinearizeHits)];
    require(tc==0&&rc==hits,"same GPU derivative requests not batched");
    p.sync_calls+=rc;p.prepared_hits+=hits;
    p.inspect();observing=false;return result;
  }catch(const std::exception& e) {
    const char* root=std::getenv("CPK16F_PROBE_ROOT");
    std::ofstream failure(fs::path(root)/"failure.txt");
    failure<<"Update "<<(paired?paired->updates:0)<<": "<<e.what()<<"\n";
    failure.flush();
    // Do not let GLIM interpret a failed diagnostic as recoverable solver trouble.
    _exit(93);
  }
}
}
