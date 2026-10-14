#define main FL3J_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "../fixed_lag_slots_fl3i_20261014/Receiver.h"
#include "Audit.h"
int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  try {
    if(argc!=3)throw std::runtime_error("usage: probe EVENTS FRESH_ARCHIVE");
    EventStream input(argv[1]);fl3i::Receiver receiver(argv[2]);size_t events=0;
    for(;;){const auto event=input.Next();++events;
      if(event.kind=='E')throw std::runtime_error("EOF before target");
      if(event.kind=='I')receiver.imuClock(event.imu.timestamp_ns,event.arrival_ns);
      else {
        const auto& raw=event.belief;fl3i::Belief b;
        b.source=raw.source_agent=="g"?"glim":raw.source_agent=="k"?"kimera":raw.source_agent;
        b.session=raw.session_id;b.from=raw.from_index;b.to=raw.to_index;
        b.ordinal=raw.ordinal;b.revision=raw.revision;b.start=raw.from_timestamp_ns;b.end=raw.to_timestamp_ns;
        b.arrival=raw.record_timestamp_ns;b.mean=raw.measurement;b.covariance=raw.covariance;receiver.belief(b);
      }
      if(fl3j::done){std::cout<<"FL3J_TARGET_COMPLETE events="<<events<<" support="<<receiver.state().newest<<'\n';break;}
    }return 0;
  }catch(const std::exception& e){std::cerr<<"FL3J_FAIL "<<e.what()<<'\n';return 1;}
}
