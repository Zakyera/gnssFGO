#include "Provider.h"
#include "Recording.h"
#include "ShadowObserver.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>

namespace shared_native {
namespace {
void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
std::mutex registry_mutex;
std::map<const void*, std::shared_ptr<ShadowObserver>> registry;
thread_local ShadowObserver* current = nullptr;
constexpr std::size_t unassigned = std::numeric_limits<std::size_t>::max();
}
struct ShadowObserver::Impl {
  struct Entry { Id id; gtsam::NonlinearFactor::shared_ptr factor; std::size_t slot=unassigned; };
  std::unique_ptr<Recording> recording;
  NativeGaussianHistory history{Mode::Shadow};
  std::map<Id, Entry> active, staged;
  std::vector<Entry> admissions, adding;
  gtsam::NonlinearFactorGraph new_factors;
  Snapshot snapshot;
  Id next_id=0;
  std::size_t captures=0, resets=0;
  bool valid=true, begun=false, captured=false, ready=false;
  char error[320]{};
  std::function<void(const Snapshot&)> audit;
};
ShadowObserver::ShadowObserver() : p_(new Impl) {}
ShadowObserver::~ShadowObserver() = default;
void ShadowObserver::setRecording(const char* path) noexcept {
  try {
    check(!p_->recording && !p_->begun && !p_->captures,"recording must start before initialization");
    p_->recording=std::make_unique<Recording>(path);
  } catch(const std::exception& e){fail(e.what());}catch(...){fail("recording setup failure");}
}
bool ShadowObserver::logDue() const { return !p_->recording || p_->recording->due(p_->history.epoch()); }
void ShadowObserver::recordQueryInput(gtsam::Key a,gtsam::Key b,const gtsam::Pose3& pa,const gtsam::Pose3& pb) {
  if(p_->recording)p_->recording->queryInput(p_->history,a,b,pa,pb);
}
void ShadowObserver::recordQueryOutput(const EndpointCovariance& c,double seconds) {
  if(p_->recording)p_->recording->queryOutput(p_->history.epoch(),c,seconds);
}
bool ShadowObserver::valid() const { return p_->valid; }
const char* ShadowObserver::error() const { return p_->error; }
const NativeGaussianHistory& ShadowObserver::history() const { return p_->history; }
const Snapshot& ShadowObserver::snapshot() const { return p_->snapshot; }
std::size_t ShadowObserver::captures() const { return p_->captures; }
std::size_t ShadowObserver::resets() const { return p_->resets; }
void ShadowObserver::setAudit(std::function<void(const Snapshot&)> f) { p_->audit=std::move(f); }
void ShadowObserver::fail(const char* reason) noexcept {
  if (!p_->valid) return;
  p_->valid=false;
  if(p_->recording)p_->recording->failure(p_->history.epoch(),reason);
  std::snprintf(p_->error,sizeof(p_->error),"%s",reason);
  p_->history.abort();
  p_->admissions.clear(); p_->adding.clear(); p_->staged.clear();
  std::fprintf(stderr,"SHARED_NATIVE_SHADOW_FAILURE epoch=%llu reason=%s\n",
      static_cast<unsigned long long>(p_->history.epoch()),p_->error);
}
void ShadowObserver::reset() noexcept {
  ++p_->resets;
  fail("MAIN fallback/reset; native history cannot continue from rebuilt MAIN prior");
}
void ShadowObserver::admit(const gtsam::NonlinearFactorGraph& graph) noexcept {
  if (!valid()) return;
  try {
    check(!p_->begun && p_->admissions.empty(),"duplicate or concurrent native admission");
    std::set<const gtsam::NonlinearFactor*> seen;
    for (const auto& item:p_->active) seen.insert(item.second.factor.get());
    for (const auto& f:graph) {
      check(f && seen.insert(f.get()).second,"duplicate/null native admission");
      p_->admissions.push_back({++p_->next_id,f,unassigned});
    }
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown admission error"); }
}
void ShadowObserver::begin(const gtsam::NonlinearFactorGraph& adding,
                           const gtsam::FactorIndices& removing) noexcept {
  if (!valid()) return;
  try {
    check(!p_->begun,"nested update of same shadow history");
    p_->begun=true; p_->captured=false; p_->ready=false;
    p_->snapshot=Snapshot(); p_->snapshot.epoch=p_->history.epoch()+1;
    p_->staged=p_->active; p_->new_factors=adding;
    for (auto slot:removing) {
      for (auto it=p_->staged.begin();it!=p_->staged.end();) {
        if (it->second.slot==slot) {
          p_->snapshot.removed.insert(it->first); it=p_->staged.erase(it);
        } else ++it;
      }
    }
    p_->adding=std::move(p_->admissions); p_->admissions.clear();
    for (const auto& entry:p_->adding) {
      std::size_t matches=0;
      for (const auto& f:adding) matches+=f.get()==entry.factor.get();
      check(matches==1,"native admission missing/duplicated in actual MAIN update");
    }
    // Any new non-admitted factor remains external/unclassified, never guessed
    // native by its class, residual, keys, covariance, or position in the graph.
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown update-begin error"); }
}
void ShadowObserver::beforeRetire(const gtsam_points::ISAM2Ext& main,
    const gtsam::ISAM2Result& result, const gtsam::KeyVector& retiring) noexcept {
  if (!valid()) return;
  try {
    check(p_->begun && !p_->captured,"unexpected pre-retirement lifecycle");
    check(main.params().cacheLinearizedFactors,"MAIN Gaussian cache disabled");
    check(result.newFactorsIndices.size()==p_->new_factors.size(),"insertion slot count mismatch");
    for (auto entry:p_->adding) {
      for (std::size_t i=0;i<p_->new_factors.size();++i)
        if (p_->new_factors[i].get()==entry.factor.get()) entry.slot=result.newFactorsIndices[i];
      check(entry.slot!=unassigned,"native insertion slot absent");
      check(p_->staged.emplace(entry.id,entry).second,"duplicate stable ID");
    }
    const auto& nonlinear=main.getFactorsUnsafe();
    const auto& cache=main.getLinearFactorsUnsafe();
    auto& s=p_->snapshot;
    s.theta=main.getLinearizationPoint();
    s.retiring.insert(retiring.begin(),retiring.end());
    s.cache.resize(cache.size());
    for (const auto& item:p_->staged) {
      const auto& entry=item.second;
      check(entry.slot<nonlinear.size() && nonlinear[entry.slot].get()==entry.factor.get(),
            "native slot identity mismatch or undeclared removal");
      check(entry.slot<cache.size() && cache[entry.slot],"missing native Gaussian cache");
      check(cache[entry.slot]->keys()==entry.factor->keys(),"native Gaussian key mismatch");
      s.cache[entry.slot]=cache[entry.slot]->clone();
      s.native.push_back({entry.id,s.epoch,entry.slot});
    }
    p_->captured=true; ++p_->captures;
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown pre-retirement error"); }
}
void ShadowObserver::afterRetire(const gtsam_points::ISAM2Ext& main) noexcept {
  if (!valid()) return;
  try {
    check(p_->captured && !p_->ready,"unexpected post-retirement lifecycle");
    auto& s=p_->snapshot; s.fixed_after=main.getFixedVariables();
    const auto& nonlinear=main.getFactorsUnsafe();
    for (auto it=p_->staged.begin();it!=p_->staged.end();) {
      bool absorbed=false;
      for (auto key:it->second.factor->keys()) absorbed|=s.retiring.count(key)!=0;
      if (absorbed) it=p_->staged.erase(it);
      else {
        check(it->second.slot<nonlinear.size() &&
              nonlinear[it->second.slot].get()==it->second.factor.get(),
              "MAIN retired a nonincident native factor");
        ++it;
      }
    }
    if (p_->recording) p_->recording->stage(s);
    if (p_->audit) p_->audit(s);
    const auto result=p_->history.stage(s);
    check(result.ok,result.message.c_str());
    p_->ready=true;
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown post-retirement error"); }
}
void ShadowObserver::commit() noexcept {
  if (!valid()) return;
  try {
    check(p_->begun && p_->ready,"outer commit without complete inner lifecycle");
    const auto result=p_->history.commit(1,p_->snapshot.epoch);
    check(result.ok,result.message.c_str());
    p_->active.swap(p_->staged); p_->staged.clear();
    p_->adding.clear(); p_->new_factors.resize(0);
    p_->begun=false; p_->ready=false;
    if(p_->recording)p_->recording->commit(p_->history);
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown commit error"); }
}
Result ShadowObserver::query(gtsam::Key from,gtsam::Key to,const gtsam::Pose3& pf,
    const gtsam::Pose3& pt,EndpointCovariance& out) noexcept {
  try {
    if (!valid()) return {false,error()};
    check(!p_->begun && p_->admissions.empty(),"query before MAIN transaction completed");
    const auto theta=p_->history.theta(p_->history.epoch());
    const auto df=theta.at<gtsam::Pose3>(from).localCoordinates(pf);
    const auto dt=theta.at<gtsam::Pose3>(to).localCoordinates(pt);
    auto result=p_->history.tryCovariance(p_->history.epoch(),from,to,df,dt,out);
    if (!result.ok) fail(result.message.c_str());
    return result;
  } catch (const std::exception& e) { fail(e.what()); }
    catch (...) { fail("unknown covariance query error"); }
  return {false,error()};
}
std::shared_ptr<ShadowObserver> attach(const void* owner) {
  std::lock_guard<std::mutex> lock(registry_mutex);
  check(owner && !registry.count(owner),"duplicate/null shadow owner");
  auto result=std::make_shared<ShadowObserver>(); registry.emplace(owner,result); return result;
}
std::shared_ptr<ShadowObserver> lookup(const void* owner) noexcept {
  try {
    std::lock_guard<std::mutex> lock(registry_mutex);
    const auto it=registry.find(owner);
    return it==registry.end()?nullptr:it->second;
  } catch (...) { return nullptr; }
}
void detach(const void* owner) noexcept {
  try { std::lock_guard<std::mutex> lock(registry_mutex); registry.erase(owner); } catch (...) {}
}
void reset(const void* owner) noexcept { if(auto p=lookup(owner)) p->reset(); }
void admitFromEnvironment(const void* owner,bool main_empty,
    const gtsam::NonlinearFactorGraph& graph) noexcept {
  const char* flag=std::getenv("GLIM_SHARED_NATIVE_SHADOW");
  if ((!flag || std::strcmp(flag,"1")!=0) && !providerEnabled()) return;
  try {
    auto p=lookup(owner);
    if (!p) {
      p=attach(owner);
      if(const char* path=std::getenv("GLIM_SHARED_NATIVE_JOURNAL"))p->setRecording(path);
      if (!main_empty) p->fail("shadow must start with native initialization; no mid-run bootstrap");
    }
    p->admit(graph);
  } catch (...) { std::fprintf(stderr,"SHARED_NATIVE_SHADOW_SETUP_FAILURE\n"); }
}
void queryForLog(const void* owner,gtsam::Key from,gtsam::Key to,
    const gtsam::Pose3& pf,const gtsam::Pose3& pt) noexcept {
  auto p=lookup(owner); if (!p || !p->valid() || !p->logDue()) return;
  try {
    p->recordQueryInput(from,to,pf,pt);
    const auto started=std::chrono::steady_clock::now();
    EndpointCovariance out;
    const auto result=p->query(from,to,pf,pt,out);
    if (!result.ok) return;
    p->recordQueryOutput(out,Recording::seconds(started));
    std::fprintf(stderr,"SHARED_NATIVE_SHADOW_COV epoch=%llu from=%llu to=%llu matrix=",
        static_cast<unsigned long long>(p->history().epoch()),
        static_cast<unsigned long long>(from),static_cast<unsigned long long>(to));
    for (int r=0;r<6;++r) for(int c=0;c<6;++c)
      std::fprintf(stderr,"%s%.17g",r||c?";":"",out.relative(r,c));
    std::fprintf(stderr," admitted=%zu active=%zu absorbed=%zu removed=%zu\n",
        p->history().admittedCount(),p->history().activeCount(),
        p->history().absorbedCount(),p->history().removedCount());
  } catch (...) { p->fail("shadow covariance logging error"); }
}
UpdateScope::UpdateScope(const void* owner,const gtsam::NonlinearFactorGraph& graph,
    const gtsam::FactorIndices& removing) noexcept : previous_(current),observer_(lookup(owner)) {
  current=observer_.get();
  if(observer_) observer_->begin(graph,removing);
}
UpdateScope::~UpdateScope() noexcept {
  if(observer_ && !completed_) observer_->fail("MAIN update unwound before commit");
  current=previous_;
}
void UpdateScope::commit() noexcept { if(observer_) observer_->commit(); completed_=true; }
void beforeRetire(const gtsam_points::ISAM2Ext& main,const gtsam::ISAM2Result& result,
    const gtsam::KeyVector& retiring) noexcept { if(current) current->beforeRetire(main,result,retiring); }
void afterRetire(const gtsam_points::ISAM2Ext& main) noexcept { if(current) current->afterRetire(main); }
}  // namespace shared_native
