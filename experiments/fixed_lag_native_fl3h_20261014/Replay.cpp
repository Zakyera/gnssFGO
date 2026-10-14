// FL3 external service observer. Includes UNCHANGED parser, model and receiver.
#define main FL3H_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "Receiver.h"
#include <set>
#include <sys/resource.h>
#include <chrono>
using Clock=std::chrono::steady_clock;
int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  std::unique_ptr<fl3h::Receiver> receiver;
  int64_t arrival=0;char kind='?';size_t ordinal=0;
  try {
    if(argc!=5)throw std::runtime_error("usage: timed EVENTS NEW_ARCHIVE NEW_SERVICE_CSV NEW_DIAGNOSTIC_DIR");
    if(std::filesystem::exists(argv[3]))throw std::runtime_error("refuse timing overwrite");
    std::ofstream log(argv[3]);log.exceptions(std::ios::failbit|std::ios::badbit);log<<std::setprecision(17);
    log<<"event,kind,arrival_ns,seconds,publications,newest,oldest,active,pending,supports,slots,nonempty,marginals,marginal_max_dimension,rss_kib,extra_retirement_updates_max,fixed_keys\n";
    if(!std::filesystem::create_directory(argv[4]))throw std::runtime_error("refuse diagnostics reuse");
    std::ofstream diagnostic(std::filesystem::path(argv[4])/"diagnostic.csv");diagnostic<<std::setprecision(17)<<"support,gradient,cost,seconds,fixed_keys\n";
    EventStream input(argv[1]);receiver=std::make_unique<fl3h::Receiver>(argv[2]);
    auto last=Clock::now();
    for(;;) {
      const auto event=input.Next();++ordinal;arrival=event.arrival_ns;kind=event.kind;
      const auto prior=receiver->state().counts.publications;
      if(kind=='E')receiver->end(arrival);
      else if(kind=='I')receiver->imuClock(event.imu.timestamp_ns,arrival);
      else {
        const auto& raw=event.belief;fl3h::Belief b;
        b.source=raw.source_agent=="g"?"glim":raw.source_agent=="k"?"kimera":raw.source_agent;
        b.session=raw.session_id;b.from=raw.from_index;b.to=raw.to_index;
        b.ordinal=raw.ordinal;b.revision=raw.revision;b.start=raw.from_timestamp_ns;b.end=raw.to_timestamp_ns;
        b.arrival=raw.record_timestamp_ns;b.mean=raw.measurement;b.covariance=raw.covariance;receiver->belief(b);
      }
      const auto& s=receiver->state();size_t nonempty=0,marginals=0,dimension=0;long rss=0;
      if(prior!=s.counts.publications||kind=='E') {
        for(const auto& f:s.solver.getFactors())if(f) {
          ++nonempty;
          if(dynamic_cast<const gtsam::LinearContainerFactor*>(f.get())) {
            ++marginals;size_t d=0;for(auto k:f->keys())d+=gtsam::Symbol(k).chr()=='x'?6:3;
            dimension=std::max(dimension,d);
          }
        }
        rusage usage{};getrusage(RUSAGE_SELF,&usage);rss=usage.ru_maxrss;
      }
      const auto now=Clock::now();const double elapsed=std::chrono::duration<double>(now-last).count();last=now;
      // This row's formatting/write time is charged to the following event.
      log<<ordinal<<','<<kind<<','<<arrival<<','<<elapsed<<','<<s.counts.publications<<','<<s.newest<<','<<s.oldest
        <<','<<s.active.size()<<','<<s.pending.size()+s.buffered.size()<<','<<s.solver.timestamps().size()/3
        <<','<<s.solver.getFactors().size()<<','<<nonempty<<','<<marginals<<','<<dimension<<','<<rss
        <<','<<s.maxPublicationSteps<<','<<s.solver.isam().getFixedVariables().size()<<'\n';
      if(prior!=s.counts.publications&&std::set<size_t>{0,70,200,600,1200,1224,1480}.count(s.newest)) {
        const auto started=Clock::now();auto copy=s.solver;
        const auto values=copy.estimate();const auto graph=copy.auditGraph();
        const double gradient=fl2::Gradient(graph,values),cost=graph.error(values);
        diagnostic<<s.newest<<','<<gradient<<','<<cost<<','<<std::chrono::duration<double>(Clock::now()-started).count()<<','<<copy.isam().getFixedVariables().size()<<'\n';
        std::ofstream state(std::filesystem::path(argv[4])/("snapshot_"+std::to_string(s.newest)+".values"));state<<std::setprecision(17);
        for(auto k:values.keys()) {
          const gtsam::Symbol symbol(k);state<<symbol.chr()<<' '<<symbol.index();
          if(symbol.chr()=='x'){const auto p=values.at<gtsam::Pose3>(k);for(int a=0;a<3;++a){for(int b=0;b<3;++b)state<<' '<<p.rotation().matrix()(a,b);state<<' '<<p.translation()[a];}}
          else for(double a:values.at<gtsam::Vector3>(k))state<<' '<<a;
          state<<'\n';
        }
        // Observation excluded from service timing; never changes receiver.
        last=Clock::now();
      }
      if(kind=='E') {
        log.flush();std::cout<<"FL3H_TIMING_TAIL_SECONDS "<<std::chrono::duration<double>(Clock::now()-last).count()<<'\n';
        std::cout<<"FL3H_STREAM_COMPLETE received="<<s.counts.received<<" inserted="<<s.counts.inserted
          <<" replaced="<<s.counts.replaced<<" absorbed="<<s.counts.absorbed<<" active="<<s.active.size()
          <<" supports="<<s.counts.publications<<" finalized="<<s.counts.finalized
          <<" remaining_supports="<<s.solver.timestamps().size()/3<<'\n';break;
      }
    }return 0;
  }catch(const std::exception& e) {
    std::cerr<<"FL3H_STREAM_FAILURE event="<<ordinal<<" kind="<<kind<<" arrival="<<arrival<<" reason="<<e.what()<<'\n';
    if(receiver){const auto& s=receiver->state();std::cerr<<"LAST_COMMITTED event="<<s.event<<" newest="<<s.newest
      <<" oldest="<<s.oldest<<" publications="<<s.counts.publications<<" received="<<s.counts.received<<'\n';}
    return 1;
  }
}

