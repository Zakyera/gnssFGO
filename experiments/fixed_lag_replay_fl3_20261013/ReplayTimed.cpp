// FL3 external service observer. Includes UNCHANGED parser, model and receiver.
#define main FL3_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "../fixed_lag_lifecycle_fl2b_20261013/Receiver.h"
#include <sys/resource.h>
#include <chrono>
using Clock=std::chrono::steady_clock;
int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  std::unique_ptr<fl2b::Receiver> receiver;
  int64_t arrival=0;char kind='?';size_t ordinal=0;
  try {
    if(argc!=4)throw std::runtime_error("usage: timed EVENTS NEW_ARCHIVE NEW_SERVICE_CSV");
    if(std::filesystem::exists(argv[3]))throw std::runtime_error("refuse timing overwrite");
    std::ofstream log(argv[3]);log.exceptions(std::ios::failbit|std::ios::badbit);log<<std::setprecision(17);
    log<<"event,kind,arrival_ns,seconds,publications,newest,oldest,active,pending,supports,slots,nonempty,marginals,marginal_max_dimension,rss_kib,steps_max,gradient_max\n";
    EventStream input(argv[1]);receiver=std::make_unique<fl2b::Receiver>(argv[2]);
    auto last=Clock::now();
    for(;;) {
      const auto event=input.Next();++ordinal;arrival=event.arrival_ns;kind=event.kind;
      const auto prior=receiver->state().counts.publications;
      if(kind=='E')receiver->end(arrival);
      else if(kind=='I')receiver->imuClock(event.imu.timestamp_ns,arrival);
      else {
        const auto& raw=event.belief;fl2b::Belief b;
        b.source=raw.source_agent=="g"?"glim":raw.source_agent=="k"?"kimera":raw.source_agent;
        b.session=raw.session_id;b.from=raw.from_index;b.to=raw.to_index;
        b.ordinal=raw.ordinal;b.revision=raw.revision;b.start=raw.from_timestamp_ns;b.end=raw.to_timestamp_ns;
        b.arrival=raw.record_timestamp_ns;b.mean=raw.measurement;b.covariance=raw.covariance;receiver->belief(b);
      }
      const auto& s=receiver->state();size_t nonempty=0,marginals=0,dimension=0;long rss=0;
      if(prior!=s.counts.publications||kind=='E') {
        for(const auto& f:s.solver.getFactors())if(f) {
          ++nonempty;
          if(dynamic_cast<const fl2a::AnchoredMarginal*>(f.get())) {
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
        <<','<<s.maxPublicationSteps<<','<<s.maxGradient<<'\n';
      if(kind=='E') {
        log.flush();std::cout<<"FL3_TIMING_TAIL_SECONDS "<<std::chrono::duration<double>(Clock::now()-last).count()<<'\n';
        std::cout<<"FL3_STREAM_COMPLETE received="<<s.counts.received<<" inserted="<<s.counts.inserted
          <<" replaced="<<s.counts.replaced<<" absorbed="<<s.counts.absorbed<<" active="<<s.active.size()
          <<" supports="<<s.counts.publications<<" finalized="<<s.counts.finalized
          <<" remaining_supports="<<s.solver.timestamps().size()/3<<'\n';break;
      }
    }return 0;
  }catch(const std::exception& e) {
    std::cerr<<"FL3_STREAM_FAILURE event="<<ordinal<<" kind="<<kind<<" arrival="<<arrival<<" reason="<<e.what()<<'\n';
    if(receiver){const auto& s=receiver->state();std::cerr<<"LAST_COMMITTED event="<<s.event<<" newest="<<s.newest
      <<" oldest="<<s.oldest<<" publications="<<s.counts.publications<<" received="<<s.counts.received<<'\n';}
    return 1;
  }
}
