#define main FL3F_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "Observe.h"
#define BoundarySmoother ObservedIncremental
#include "Receiver.h"
#undef BoundarySmoother

namespace fl3f {
using namespace gtsam;
namespace fs=std::filesystem;
std::ofstream NewFile(const fs::path& p) {
  if(fs::exists(p))throw std::runtime_error("refuse diagnostic overwrite");
  std::ofstream f(p);f.exceptions(std::ios::failbit|std::ios::badbit);f<<std::setprecision(17);return f;
}
void ValuesFile(const fs::path& path,const Values& values) {
  auto out=NewFile(path);
  for(auto key:values.keys()) {
    const Symbol symbol(key);out<<symbol.chr()<<' '<<symbol.index();
    if(symbol.chr()=='x') {
      auto m=values.at<Pose3>(key).matrix();for(int r=0;r<3;++r)for(int c=0;c<4;++c)out<<' '<<m(r,c);
    } else for(double x:values.at<Vector3>(key))out<<' '<<x;
    out<<'\n';
  }
}
std::string Ledger(const fl2b::State& s) {
  std::ostringstream out;out<<std::setprecision(17);
  for(const auto& [id,e]:s.active) {
    const auto& b=e.latest;
    out<<fl2b::Name(id)<<','<<b.revision<<','<<b.ordinal<<','<<b.start<<','<<b.end<<','<<b.arrival<<','<<e.slot;
    auto m=b.mean.matrix();for(int r=0;r<3;++r)for(int c=0;c<4;++c)out<<','<<m(r,c);
    for(int r=0;r<6;++r)for(int c=0;c<6;++c)out<<','<<b.covariance(r,c);
    out<<'\n';
  }
  return out.str();
}
void StateInfo(std::ostream& out,const std::string& prefix,const fl2b::State& s) {
  out<<prefix<<"event "<<s.event<<'\n'<<prefix<<"newest "<<s.newest<<'\n'
    <<prefix<<"oldest "<<s.oldest<<'\n'<<prefix<<"supports "<<s.solver.timestamps().size()/3<<'\n'
    <<prefix<<"slots "<<s.solver.getFactors().size()<<'\n'<<prefix<<"active "<<s.active.size()<<'\n'
    <<prefix<<"pending "<<s.pending.size()<<'\n'<<prefix<<"buffered "<<s.buffered.size()<<'\n'
    <<prefix<<"max_steps "<<s.maxPublicationSteps<<'\n';
  const auto& c=s.counts;
  out<<prefix<<"received "<<c.received<<'\n'<<prefix<<"inserted "<<c.inserted<<'\n'
    <<prefix<<"replaced "<<c.replaced<<'\n'<<prefix<<"absorbed "<<c.absorbed<<'\n'
    <<prefix<<"excluded "<<c.excluded<<'\n'<<prefix<<"duplicate "<<c.duplicate<<'\n'
    <<prefix<<"stale "<<c.stale<<'\n'<<prefix<<"unsupported_end "<<c.unsupportedEnd<<'\n'
    <<prefix<<"publications "<<c.publications<<'\n'<<prefix<<"finalized "<<c.finalized<<'\n'
    <<prefix<<"cursor "<<s.solver.publicationCursor<<'\n'
    <<prefix<<"window_rebuilds "<<s.solver.windowRebuilds<<'\n'
    <<prefix<<"retired_blocks "<<s.solver.retiredBlocks<<'\n';
}
void DumpPriors(const fs::path& dir) {
  if(referenceMarginals.empty())return;
  ValuesFile(dir/"reference_pre.values",referencePre);
  ValuesFile(dir/"reference_final.values",referenceFinal);
  std::set<Key> keys;
  for(const auto& f:referenceMarginals)for(auto k:f->keys())keys.insert(k);
  Ordering order(keys.begin(),keys.end());
  auto keyfile=NewFile(dir/"prior.keys");
  for(auto k:order)keyfile<<Symbol(k).chr()<<' '<<Symbol(k).index()<<' '<<referenceFinal.at(k).dim()<<'\n';
  auto dump=[&](const NonlinearFactorGraph& graph,const std::string& name) {
    std::set<Key> other;
    for(const auto& f:graph)for(auto k:f->keys())other.insert(k);
    fl2b::Require(other==keys,"marginal key sets differ");
    const auto ab=graph.linearize(referenceFinal)->jacobian(order);
    auto a=NewFile(dir/(name+".A"));a<<ab.first.rows()<<' '<<ab.first.cols()<<'\n';
    for(int r=0;r<ab.first.rows();++r) {
      for(int c=0;c<ab.first.cols();++c)a<<ab.first(r,c)<<' ';
      a<<'\n';
    }
    auto b=NewFile(dir/(name+".b"));b<<ab.second.size()<<" 1\n";
    for(double x:ab.second)b<<x<<'\n';
  };
  dump(referenceMarginals,"reference_prior");
  if(!candidateMarginals.empty())dump(candidateMarginals,"candidate_prior");
}
void Analyze(const fs::path& dir,bool published,const std::string& failure,double eventSeconds,
             double replaySeconds,const std::string& beforeInfo,const std::string& beforeLedger,
             const Values& beforeValues,const fl2b::State& state) {
  auto result=NewFile(dir/"result.txt");
  result<<"published "<<published<<"\nfailure "<<(failure.empty()?"none":failure)
    <<"\ntotal_budget 4\ntarget_event_seconds "<<eventSeconds
    <<"\nreference_seconds "<<referenceSeconds
    <<"\ncandidate_event_seconds_excluding_reference "<<eventSeconds-referenceSeconds
    <<"\nreference_pre_steps "<<referencePreSteps<<"\nreference_post_steps "<<referencePostSteps
    <<"\nreplay_seconds "<<replaySeconds<<"\nretirement_calls "<<retirementCalls
    <<"\nreload_calls "<<reloadCalls<<"\nretirement_seconds "<<retirementSeconds
    <<"\nreload_seconds "<<reloadSeconds<<'\n'<<beforeInfo;
  DumpPriors(dir);
  StateInfo(result,"after_",state);
  auto a=NewFile(dir/"before_ledger.csv");a<<beforeLedger;
  auto b=NewFile(dir/"after_ledger.csv");b<<Ledger(state);
  ValuesFile(dir/"before_event.values",beforeValues);
  ValuesFile(dir/"after_event.values",state.solver.estimate());
  for(size_t phase=0;phase<captures.size();++phase) {
    const auto& c=captures[phase];const std::string name=phase==0?"pre":"post";
    const auto sub=dir/name;fs::create_directory(sub);
    result<<name<<"_budget "<<c.requestedBudget<<'\n'<<name<<"_numerically_valid "<<c.result.passed
      <<'\n'<<name<<"_converged "<<(std::isfinite(c.result.after)&&c.result.after<=1e-5)
      <<'\n'<<name<<"_steps "<<c.result.steps<<'\n'<<name<<"_stop "<<c.stop
      <<'\n'<<name<<"_gradient_before "<<c.result.before<<'\n'<<name<<"_gradient_after "<<c.result.after
      <<'\n'<<name<<"_cost_before "<<c.initialCost<<'\n'<<name<<"_cost_after "<<c.finalCost
      <<'\n'<<name<<"_seconds "<<c.seconds<<'\n';
    auto trace=NewFile(sub/"trials.csv");
    trace<<"step,accepted,old_cost,trial_cost,accepted_gradient,relinearized,reeliminated,update_seconds,check_seconds,total_seconds\n";
    ValuesFile(sub/"initial.values",c.initial);ValuesFile(sub/"last_accepted.values",c.last);
    for(size_t i=0;i<c.steps.size();++i) {
      const auto& s=c.steps[i];
      trace<<i+1<<','<<s.accepted<<','<<s.oldCost<<','<<s.newCost<<','<<s.gradient<<','
        <<s.relinearized<<','<<s.reeliminated<<','<<s.updateSeconds<<','<<s.checkSeconds<<','<<s.totalSeconds<<'\n';
      ValuesFile(sub/("trial_"+std::to_string(i+1)+".values"),s.value);
    }
  }
  if(published) {
    const double gradient=fl2a::CheckedGradient(state.solver.auditGraph(),state.solver.estimate());
    result<<"committed_gradient "<<gradient<<'\n';
    fl2b::Require(std::isfinite(gradient),"fresh candidate gradient nonfinite");
  } else {
    fl2b::Require(state.event==84957&&state.newest==1223,"failed update did not roll back");
    fl2b::Require(beforeLedger==Ledger(state)&&beforeValues.equals(state.solver.estimate(),0),"rollback state/ledger changed");
    std::ostringstream info;info<<std::setprecision(17);StateInfo(info,"before_",state);
    fl2b::Require(beforeInfo==info.str(),"rollback bookkeeping changed");
    result<<"rollback_verified 1\n";
  }
}
}

int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  std::unique_ptr<fl2b::Receiver> receiver;size_t ordinal=0;
  const auto start=fl3f::Clock::now();
  std::string beforeInfo,beforeLedger,failure;gtsam::Values beforeValues;
  double targetSeconds=0;bool published=false;
  try {
    if(argc!=4)throw std::runtime_error("usage: diagnose JOURNAL NEW_ARCHIVE NEW_DIAGNOSTICS");
    if(!std::filesystem::create_directory(argv[3]))throw std::runtime_error("refuse diagnostic directory reuse");
    EventStream input(argv[1]);receiver=std::make_unique<fl2b::Receiver>(argv[2]);
    while(ordinal<84958) {
      const auto e=input.Next();++ordinal;
      if(e.kind=='E')throw std::runtime_error("unexpected EOF before target");
      const bool target=ordinal==84958;
      if(target) {
        fl2b::Require(e.kind=='I'&&receiver->state().event==84957&&receiver->state().newest==1223,"target identity mismatch");
        std::ostringstream info;info<<std::setprecision(17);fl3f::StateInfo(info,"before_",receiver->state());
        beforeInfo=info.str();beforeLedger=fl3f::Ledger(receiver->state());beforeValues=receiver->state().solver.estimate();
      }
      const auto eventStart=fl3f::Clock::now();
      try {
        if(e.kind=='I')receiver->imuClock(e.imu.timestamp_ns,e.arrival_ns);
        else {const auto& r=e.belief;fl2b::Belief b;
          b.source=r.source_agent=="g"?"glim":r.source_agent=="k"?"kimera":r.source_agent;
          b.session=r.session_id;b.from=r.from_index;b.to=r.to_index;b.ordinal=r.ordinal;b.revision=r.revision;
          b.start=r.from_timestamp_ns;b.end=r.to_timestamp_ns;b.arrival=r.record_timestamp_ns;
          b.mean=r.measurement;b.covariance=r.covariance;receiver->belief(b);
        }
      } catch(...) {if(target)targetSeconds=fl3f::Seconds(eventStart);throw;}
      if(target){targetSeconds=fl3f::Seconds(eventStart);published=true;}
    }
  } catch(const std::exception& e) {
    failure=e.what();std::cerr<<"FL3F_STOP event="<<ordinal<<" reason="<<failure<<'\n';
    if(ordinal!=84958||!receiver||beforeInfo.empty())return 2;
  }
  try {
    fl3f::Analyze(argv[3],published,failure,targetSeconds,fl3f::Seconds(start),beforeInfo,beforeLedger,beforeValues,receiver->state());
    std::cout<<"FL3F_COMPLETE published="<<published<<" seconds="<<targetSeconds<<'\n';
    return published?0:10;
  } catch(const std::exception& e){std::cerr<<"DIAGNOSTIC_FAILURE "<<e.what()<<'\n';return 4;}
}

