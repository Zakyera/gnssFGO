#define main FL3D_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "Observe.h"
// Only this private receiver includes the observer; frozen files are unchanged.
#define BoundarySmoother ObservedISAMConvergence
#include "../fixed_lag_lifecycle_fl2b_20261013/Receiver.h"
#undef BoundarySmoother

namespace fl3d {
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
void Analyze(const fs::path& dir,double replaySeconds,const fl2b::State& prior) {
  if(!capture)throw std::runtime_error("missing target capture");
  const auto& c=*capture;
  auto result=NewFile(dir/"result.txt");
  result<<"stop "<<c.stop<<"\nbudget 200\nrequested_budget "<<c.requestedBudget
    <<"\nattempts "<<c.result.steps<<"\npassed "<<c.result.passed
    <<"\ninitial_gradient "<<c.result.before<<"\nfinal_accepted_gradient "<<c.result.after
    <<"\ninitial_cost "<<c.initialCost<<"\nfinal_cost "<<c.finalCost
    <<"\nfixed_keys "<<c.fixedKeys<<"\nvariables "<<c.initial.size()
    <<"\ntarget_seconds "<<c.seconds<<"\ninitial_check_seconds "<<c.initialCheckSeconds
    <<"\nreplay_through_target_seconds "<<replaySeconds
    <<"\nrollback_event "<<prior.event<<"\nrollback_newest "<<prior.newest<<'\n';
  auto trace=NewFile(dir/"trials.csv");
  trace<<"step,accepted,old_cost,trial_cost,accepted_gradient,relinearized,reeliminated,update_seconds,check_seconds,total_seconds\n";
  ValuesFile(dir/"initial.values",c.initial);
  for(size_t i=0;i<c.steps.size();++i) {
    const auto& s=c.steps[i];
    trace<<i+1<<','<<s.accepted<<','<<s.oldCost<<','<<s.newCost<<','<<s.gradient<<','
      <<s.relinearized<<','<<s.reeliminated<<','<<s.updateSeconds<<','<<s.checkSeconds<<','<<s.totalSeconds<<'\n';
    ValuesFile(dir/("trial_"+std::to_string(i+1)+".values"),s.value);
  }
  ValuesFile(dir/"last_accepted.values",c.last);
  std::cout<<"FL3D_COMPLETE stop="<<c.stop<<" attempts="<<c.result.steps
    <<" gradient="<<c.result.after<<" seconds="<<c.seconds<<'\n';
}
}

int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  std::unique_ptr<fl2b::Receiver> receiver;size_t ordinal=0;
  const auto start=fl3d::Clock::now();
  try {
    if(argc!=4)throw std::runtime_error("usage: diagnose JOURNAL NEW_ARCHIVE NEW_DIAGNOSTICS");
    if(!std::filesystem::create_directory(argv[3]))throw std::runtime_error("refuse diagnostic directory reuse");
    EventStream input(argv[1]);receiver=std::make_unique<fl2b::Receiver>(argv[2]);
    while(ordinal<84958) {
      const auto e=input.Next();++ordinal;
      if(e.kind=='E')throw std::runtime_error("unexpected EOF before target");
      if(e.kind=='I')receiver->imuClock(e.imu.timestamp_ns,e.arrival_ns);
      else {const auto& r=e.belief;fl2b::Belief b;
        b.source=r.source_agent=="g"?"glim":r.source_agent=="k"?"kimera":r.source_agent;
        b.session=r.session_id;b.from=r.from_index;b.to=r.to_index;b.ordinal=r.ordinal;b.revision=r.revision;
        b.start=r.from_timestamp_ns;b.end=r.to_timestamp_ns;b.arrival=r.record_timestamp_ns;
        b.mean=r.measurement;b.covariance=r.covariance;receiver->belief(b);
      }
    }
    throw std::runtime_error("expected diagnostic stop did not occur");
  }catch(const std::exception& e) {
    std::cerr<<"FL3D_STOP event="<<ordinal<<" reason="<<e.what()<<'\n';
    if(ordinal!=84958||std::string(e.what())!="FL3D_DIAGNOSTIC_STOP")return 2;
    if(!receiver||receiver->state().event!=84957||receiver->state().newest!=1223)return 3;
    try{fl3d::Analyze(argv[3],fl3d::Seconds(start),receiver->state());return 0;}
    catch(const std::exception& error){std::cerr<<"DIAGNOSTIC_FAILURE "<<error.what()<<'\n';return 4;}
  }
}
