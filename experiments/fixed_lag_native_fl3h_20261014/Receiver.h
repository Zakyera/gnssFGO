#pragma once
// Private FL3h: FL2b lifecycle with stock fixed-linearization publication policy.
// No production receiver entry point or mathematical factory is modified.
#include "NativeSmoother.h"
#include "SourceWnoaModel.h"
#include "offline_process/CausalSupportClock.h"
#include <gtsam/slam/PriorFactor.h>
#include <filesystem>
#include <fstream>
#include <deque>
#include <tuple>
#include <iomanip>
#include <set>

namespace fl3h {
namespace fs=std::filesystem;
using namespace gtsam;
using namespace fgo::offline;
using Id=std::tuple<std::string,std::string,uint64_t,uint64_t>;
constexpr int64_t Period=100000000;
inline SharedNoiseModel Qc() {
  Vector6 q;q<<.00375,.00375,.00375,.015,.015,.015;
  return noiseModel::Diagonal::Variances(q);
}
inline void Require(bool yes,const char* why){if(!yes)throw std::runtime_error(why);}
inline bool Safe(const std::string& s) {
  return !s.empty()&&s.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")==std::string::npos;
}
inline std::string Name(const Id& id) {
  std::ostringstream out;out<<std::get<0>(id)<<','<<std::get<1>(id)<<','<<std::get<2>(id)<<','<<std::get<3>(id);return out.str();
}
// Same wire fields, stable key and tolerance as E2b DiagnosticReplay.cpp.
struct Belief {
  std::string source,session="default";
  uint64_t from=0,to=0,revision=0,ordinal=0;
  int64_t start=0,end=0,arrival=0;
  Pose3 mean;
  Matrix6 covariance=Matrix6::Identity();
  Id id() const{return {source,session,from,to};}
  void validate() const {
    Require(Safe(source)&&Safe(session),"invalid source/session");
    Require(end>start&&arrival>=end,"invalid physical/arrival timestamps");
    Require(mean.matrix().allFinite()&&covariance.allFinite(),"nonfinite belief");
    Require((covariance-covariance.transpose()).cwiseAbs().maxCoeff()<=1e-12,"asymmetric covariance");
    Eigen::SelfAdjointEigenSolver<Matrix6> eig(covariance);
    Require(eig.info()==Eigen::Success&&eig.eigenvalues().minCoeff()>0,"nonpositive covariance");
  }
};
struct Absorbed {Belief belief;size_t slot=0;};
enum class Failure {None,AfterReplacement,AfterRetirement,AfterReload,BeforeOutput,OutputStaged};

// Per-event immutable output. Atomic visibility; deliberately not crash-resumable.
// The absorbed lookup is a disk index, not an all-history in-memory factor ledger.
class Archive {
  fs::path root_;
  static void write(const fs::path& path,const std::string& data) {
    Require(!fs::exists(path),"refuse archive overwrite");
    std::ofstream out(path);out.exceptions(std::ios::failbit|std::ios::badbit);out<<data;out.close();
  }
  fs::path marker(const Id& id) const {
    return root_/"ids"/std::get<0>(id)/std::get<1>(id)/(std::to_string(std::get<2>(id))+"_"+std::to_string(std::get<3>(id)));
  }
 public:
  explicit Archive(const fs::path& root):root_(root) {
    Require(fs::create_directory(root_),"archive must be a new directory");
    fs::create_directory(root_/"commits");fs::create_directory(root_/"ids");
  }
  const fs::path& root()const{return root_;}
  bool absorbed(const Id& id) const {
    const auto p=marker(id);if(!fs::exists(p))return false;
    uint64_t seq;std::ifstream in(p);Require(bool(in>>seq),"invalid absorption marker");
    Require(fs::is_directory(root_/"commits"/std::to_string(seq)),"orphaned absorption marker");
    return true;
  }
  void commit(uint64_t seq,const std::string& rows,const std::vector<Absorbed>& dead,Failure failure) {
    if(rows.empty()&&dead.empty())return;
    const auto staging=root_/(".stage_"+std::to_string(seq));
    const auto dest=root_/"commits"/std::to_string(seq);
    Require(!fs::exists(dest)&&fs::create_directory(staging),"archive transaction collision");
    std::vector<fs::path> ownMarkers;
    try {
      write(staging/"records.txt",rows);
      for(const auto& a:dead) {
        const auto p=marker(a.belief.id());Require(!fs::exists(p),"BeliefID already archived");
        fs::create_directories(p.parent_path());
        // Register before opening so an I/O failure cannot strand a partial file.
        ownMarkers.push_back(p);write(p,std::to_string(seq)+"\n");
      }
      if(failure==Failure::OutputStaged)throw std::runtime_error("injected_output_staged");
      fs::rename(staging,dest); // Last potentially throwing operation; single writer.
    } catch(...) {
      for(const auto& p:ownMarkers)fs::remove(p);
      fs::remove(staging/"records.txt");fs::remove(staging);throw;
    }
  }
};

class Solver : public fl3h_solver::NativeSmoother {
 public:
  explicit Solver(double lag):NativeSmoother(lag){}
  // E2b ReplaceFactor forces ordinary relinearization for each admitted version.
  FactorIndex replace(const NonlinearFactor::shared_ptr& factor,const FactorIndices& remove) {
    NonlinearFactorGraph g;g.push_back(factor);
    isamResult_=isam_.update(g,{},remove,boost::none,boost::none,boost::none,true);
    Require(isamResult_.newFactorsIndices.size()==1,"one version must create one factor");
    return isamResult_.newFactorsIndices.front();
  }
};
struct Entry {
  Belief latest;FactorIndex slot=0;NonlinearFactor::shared_ptr owner;
};
struct Counts {
  uint64_t received=0,inserted=0,replaced=0,duplicate=0,stale=0,excluded=0,
    unsupportedEnd=0,absorbed=0,publications=0,finalized=0,queued=0;
  size_t peakActive=0,peakPending=0,peakSupports=0,peakSlots=0;
  bool operator==(const Counts&)const=default;
};
struct State {
  Solver solver;
  CausalSupportClock clock{59,Period};
  std::map<Id,Entry> active;
  std::deque<Belief> pending,buffered;
  Counts counts;
  int64_t lastArrival=std::numeric_limits<int64_t>::min(),origin=0;
  uint64_t event=0;size_t newest=0,oldest=0;
  bool initialized=false,closed=false;
  double maxGradient=0;size_t maxPublicationSteps=0;
  explicit State(double lag):solver(lag){}
};
struct Bracket {size_t left,right;int64_t offset;};

class Receiver {
  static double validLag(double lag) {
    Require(std::isfinite(lag)&&lag>=.1&&lag<=1000000.&&std::abs(lag*10-std::round(lag*10))<1e-12,"lag must be whole support ticks");return lag;
  }
  std::unique_ptr<State> state_;
  Archive archive_;
  const size_t lagTicks_;
  const SharedNoiseModel qc_=Qc();
  struct Batch {std::ostringstream rows;std::vector<Absorbed> dead;Batch(){rows<<std::setprecision(17);}};
  Bracket bracket(const State& s,int64_t stamp)const {
    Require(stamp>=s.origin,"endpoint precedes initialization");
    // Avoid signed overflow even for a malformed far-future timestamp.
    const auto offset=uint64_t(stamp)-uint64_t(s.origin);
    Require(offset<=uint64_t(std::numeric_limits<int64_t>::max()),"endpoint span overflow");
    const auto left=offset/Period;Require(left<(uint64_t(1)<<52),"support index overflow");
    return {size_t(left),size_t(left+(offset%Period!=0)),int64_t(offset%Period)};
  }
  WnoaEndpoint endpoint(const Bracket& b,int64_t stamp)const {
    if(b.left==b.right)return WnoaEndpoint::Exact(b.left);
    return WnoaEndpoint::Between(b.left,b.right,.1,b.offset*1e-9,qc_,stamp).withCache(nullptr);
  }
  size_t required(const State& s,const Belief& b)const{return bracket(s,b.end).right;}
  void ownerCheck(const State& s,const Entry& e)const {
    const auto& graph=s.solver.getFactors();
    Require(e.slot<graph.size()&&graph[e.slot]==e.owner,"stale or recycled source slot");
    for(auto key:e.owner->keys())Require(s.solver.timestamps().count(key),"source references retired support");
  }
  void rowBelief(Batch& out,const Belief& b,const char* action,int64_t insertion,size_t slot)const {
    out.rows<<"belief,"<<action<<','<<Name(b.id())<<','<<b.revision<<','<<b.ordinal<<','
      <<b.start<<','<<b.end<<','<<b.arrival<<','<<insertion<<','<<slot;
    for(double x:Pose3::Logmap(b.mean))out.rows<<','<<x;
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)out.rows<<','<<b.covariance(i,j);
    out.rows<<'\n';
  }
  void rowState(Batch& out,const State& s,const Values& values,size_t index,int64_t availability,const char* kind)const {
    const auto v=ReadSourceWnoaState(values,index);const auto q=v.pose.rotation().toQuaternion();
    Require(v.pose.matrix().allFinite()&&v.velocity.allFinite()&&v.omega.allFinite(),"nonfinite output state");
    out.rows<<kind<<','<<index<<','<<(s.origin+int64_t(index)*Period)<<','<<availability
      <<','<<v.pose.x()<<','<<v.pose.y()<<','<<v.pose.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
    for(double x:v.velocity)out.rows<<','<<x;for(double x:v.omega)out.rows<<','<<x;out.rows<<'\n';
  }
  void dispatch(State& s,const Belief& b,int64_t now,Batch& out,Failure fail) {
    Require(now>=b.arrival&&required(s,b)<=s.newest,"unavailable belief/support");
    Require(!archive_.absorbed(b.id()),"revision after absorption");
    const auto from=bracket(s,b.start),to=bracket(s,b.end);
    Require(from.left>=s.oldest,"new belief references retired support");
    auto found=s.active.find(b.id());const bool known=found!=s.active.end();
    if(known) {
      const auto& previous=found->second.latest;ownerCheck(s,found->second);
      Require(b.start==previous.start&&b.end==previous.end,"stable-ID endpoints changed");
      if(b.revision<=previous.revision) {
        if(b.revision==previous.revision) {
          Require(b.mean.localCoordinates(previous.mean).norm()<=1e-12&&
            (b.covariance-previous.covariance).cwiseAbs().maxCoeff()<=1e-12,"conflicting duplicate");
          ++s.counts.duplicate;rowBelief(out,b,"duplicate",now,found->second.slot);
        } else {++s.counts.stale;rowBelief(out,b,"stale",now,found->second.slot);}
        return;
      }
    }
    auto factor=boost::make_shared<WnoaRelativeFactor>(endpoint(from,b.start),endpoint(to,b.end),
      b.mean,noiseModel::Gaussian::Covariance(b.covariance));
    const auto slot=s.solver.replace(factor,known?FactorIndices{found->second.slot}:FactorIndices{});
    if(fail==Failure::AfterReplacement)throw std::runtime_error("injected_after_replacement");
    if(known)++s.counts.replaced;else ++s.counts.inserted;
    s.active[b.id()]={b,slot,factor};ownerCheck(s,s.active.at(b.id()));
    rowBelief(out,b,known?"replace":"insert",now,slot);
  }
  void accept(State& s,const Belief& b,Batch& out,Failure fail) {
    // Identity must be checked BEFORE startup exclusion or pending classification:
    // a malformed revision cannot disguise an endpoint change as an exclusion.
    auto sameEndpoints=[&](const Belief& old) {
      if(old.id()==b.id())Require(b.start==old.start&&b.end==old.end,"stable-ID endpoints changed");
    };
    auto old=s.active.find(b.id());if(old!=s.active.end())sameEndpoints(old->second.latest);
    for(const auto& p:s.pending)sameEndpoints(p);
    for(const auto& p:s.buffered)sameEndpoints(p);
    if(!s.initialized){s.buffered.push_back(b);return;}
    // Match existing startup exclusions, but never use them to conceal absorbed-ID reuse.
    Require(!archive_.absorbed(b.id()),"revision after absorption");
    if(b.start<s.origin){++s.counts.excluded;rowBelief(out,b,"before_initialization",s.lastArrival,0);return;}
    Require(bracket(s,b.start).left>=s.oldest,"new belief references retired support");
    if(required(s,b)>s.newest){s.pending.push_back(b);++s.counts.queued;}
    else dispatch(s,b,s.lastArrival,out,fail);
  }
  void pending(State& s,Batch& out,Failure fail) {
    for(auto it=s.pending.begin();it!=s.pending.end();) {
      if(required(s,*it)<=s.newest){dispatch(s,*it,s.lastArrival,out,fail);it=s.pending.erase(it);}else ++it;
    }
  }
  void publish(State& s,Batch& out,Failure fail) {
    const size_t keep=s.newest>lagTicks_?s.newest-lagTicks_:0;
    for(const auto& b:s.pending)Require(bracket(s,b.start).left>=keep,"pending old bracket would retire");
    size_t steps=0;
    if(keep>s.oldest) {
      std::vector<Id> consumed;
      for(const auto& [id,e]:s.active) {
        ownerCheck(s,e);
        if(std::any_of(e.owner->keys().begin(),e.owner->keys().end(),[&](Key k){return Symbol(k).index()<keep;}))consumed.push_back(id);
      }
      const auto before=s.solver.timestamps().size();
      // Exactly the stock retirement update. Capture from a COPY immediately
      // before its marginalizeLeaves, not the estimate before that update.
      const auto outgoing=s.solver.retire();++steps;
      for(size_t i=s.oldest;i<keep;++i){rowState(out,s,outgoing,i,s.lastArrival,"finalized_incremental");++s.counts.finalized;}
      s.solver.retiredBlocks+=before-s.solver.timestamps().size();
      if(fail==Failure::AfterRetirement)throw std::runtime_error("injected_after_retirement");
      // Compatibility failure point only: NO chart reload occurs.
      if(fail==Failure::AfterReload)throw std::runtime_error("injected_after_retirement_audit");
      s.oldest=keep;
      for(const auto& id:consumed) {
        const auto& e=s.active.at(id);const auto& g=s.solver.getFactors();
        Require(e.slot>=g.size()||g[e.slot]!=e.owner,"consumed source still explicit");
        out.rows<<"absorbed,"<<Name(id)<<','<<e.latest.revision<<','<<e.slot<<','<<s.lastArrival<<'\n';
        out.dead.push_back({e.latest,e.slot});s.active.erase(id);++s.counts.absorbed;
      }
    }
    s.maxPublicationSteps=std::max(s.maxPublicationSteps,steps);
    Require(s.solver.timestamps().size()==3*(s.newest-s.oldest+1),"retained support count mismatch");
    for(const auto& [key,tick]:s.solver.timestamps())Require(Symbol(key).index()>=s.oldest&&tick==double(Symbol(key).index()),"retired or refreshed timestamp");
    for(const auto& [id,e]:s.active)ownerCheck(s,e);
    const auto value=s.solver.estimate();rowState(out,s,value,s.newest,s.lastArrival,"online_incremental");
    out.rows<<"publication_incremental,"<<s.newest<<','<<s.lastArrival<<",not_checked,"<<steps<<','<<s.oldest<<'\n';
    ++s.solver.publicationCursor;++s.counts.publications;
  }
  void support(State& s,const SupportReady& ready,Batch& out,Failure fail) {
    Values values;NonlinearFactorGraph graph;
    if(!s.initialized) {
      s.initialized=true;s.origin=ready.physical_ns;
      InsertSourceWnoaState(values,0,{});
      graph.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-5)));
      graph.emplace_shared<PriorFactor<Vector3>>(V(0),Vector3::Zero(),noiseModel::Diagonal::Sigmas(Vector3::Constant(10.)));
      graph.emplace_shared<PriorFactor<Vector3>>(W(0),Vector3::Zero(),noiseModel::Diagonal::Sigmas(Vector3::Constant(3.)));
    } else {
      ++s.newest;Require(ready.physical_ns==s.origin+int64_t(s.newest)*Period,"nonuniform support");
      const auto predicted=PredictSourceWnoa(ReadSourceWnoaState(s.solver.estimate(),s.newest-1),.1);
      InsertSourceWnoaState(values,s.newest,predicted);
      graph.emplace_shared<fgo::factor::GPWNOAPrior>(X(s.newest-1),V(s.newest-1),W(s.newest-1),X(s.newest),V(s.newest),W(s.newest),.1,qc_,true,true);
    }
    const size_t i=s.newest;s.solver.add(graph,values,{{X(i),double(i)},{V(i),double(i)},{W(i),double(i)}});
    s.counts.peakSupports=std::max(s.counts.peakSupports,s.solver.timestamps().size()/3);
    if(i)pending(s,out,fail);
    publish(s,out,fail);
    // Frozen streaming entry point publishes X0 before dispatching pre-start buffer.
    if(!i){auto buffered=std::move(s.buffered);s.buffered.clear();for(const auto& b:buffered)accept(s,b,out,fail);}
  }
  void check(State& s)const {
    size_t sources=0;for(const auto& f:s.solver.getFactors())sources+=dynamic_cast<const WnoaRelativeFactor*>(f.get())!=nullptr;
    Require(sources==s.active.size(),"explicit source factor/ID count mismatch");
    Require(s.counts.inserted==s.active.size()+s.counts.absorbed,"ID conservation");
    Require(s.counts.received==s.counts.inserted+s.counts.replaced+s.counts.duplicate+s.counts.stale+
      s.counts.excluded+s.counts.unsupportedEnd+s.pending.size()+s.buffered.size(),"version conservation");
    s.counts.peakActive=std::max(s.counts.peakActive,s.active.size());
    s.counts.peakPending=std::max(s.counts.peakPending,s.pending.size()+s.buffered.size());
    s.counts.peakSlots=std::max(s.counts.peakSlots,s.solver.getFactors().size());
  }
  template<class Fn>void transaction(int64_t now,Failure fail,Fn body) {
    Require(!state_->closed,"receiver already closed");Require(now>=state_->lastArrival,"availability moved backwards");
    auto next=std::make_unique<State>(*state_);next->lastArrival=now;++next->event;Batch out;
    body(*next,out);check(*next);
    if(fail==Failure::BeforeOutput)throw std::runtime_error("injected_before_output");
    archive_.commit(next->event,out.rows.str(),out.dead,fail);
    state_.swap(next); // noexcept: nothing throwable follows output commit.
  }
 public:
  explicit Receiver(const fs::path& output,double lag=7.):state_(std::make_unique<State>(validLag(lag))),archive_(output),lagTicks_(size_t(std::llround(lag*10))) {}
  const State& state()const{return *state_;}
  const Archive& archive()const{return archive_;}
  void imuClock(int64_t physical,int64_t arrival,Failure fail=Failure::None) {
    Require(!state_->closed,"receiver already closed");Require(arrival>=state_->lastArrival,"availability moved backwards");
    auto clock=state_->clock;const auto ready=clock.ObserveImu(physical,arrival);
    // A raw clock sample that emits no support cannot touch the graph/output.
    // Commit only its scalar clock state; do not copy iSAM2 at raw IMU frequency.
    if(ready.empty()&&fail==Failure::None) {
      static_assert(std::is_nothrow_copy_assignable_v<CausalSupportClock>);
      state_->clock=clock;state_->lastArrival=arrival;++state_->event;return;
    }
    transaction(arrival,fail,[&](State& s,Batch& out){s.clock=clock;for(const auto& r:ready)support(s,r,out,fail);});
  }
  void belief(const Belief& b,Failure fail=Failure::None) {
    b.validate();transaction(b.arrival,fail,[&](State& s,Batch& out){++s.counts.received;accept(s,b,out,fail);});
  }
  void end(int64_t arrival,Failure fail=Failure::None) {
    transaction(arrival,fail,[&](State& s,Batch& out){
      Require(s.initialized,"end before initialization");
      const auto value=s.solver.estimate();
      for(const auto& b:s.pending){Require(required(s,b)>s.newest,"ready belief left pending");rowBelief(out,b,"unsupported_at_end",arrival,0);++s.counts.unsupportedEnd;}
      s.pending.clear();
      for(size_t i=s.oldest;i<=s.newest;++i){rowState(out,s,value,i,arrival,"finalized_incremental");++s.counts.finalized;}
      rowState(out,s,value,s.newest,arrival,"drain_incremental");out.rows<<"end_incremental,"<<arrival<<",not_checked\n";s.closed=true;
    });
  }
};
} // namespace fl3h

