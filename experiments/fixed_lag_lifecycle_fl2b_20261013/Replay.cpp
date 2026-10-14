// Reuse the EXACT frozen event parser and validation, not a second wire schema.
// The historical main/Run are compiled but never invoked. No frozen file edits.
#define main FL2B_UNUSED_BASELINE_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "Receiver.h"

int main(int argc,char** argv) {
  const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  try {
    if(argc!=3)throw std::runtime_error("usage: fixed_lag_replay EVENTS NEW_OUTPUT_DIRECTORY");
    EventStream input(argv[1]);fl2b::Receiver receiver(argv[2]);
    for(;;) {
      const auto event=input.Next();
      if(event.kind=='E'){receiver.end(event.arrival_ns);break;}
      if(event.kind=='I')receiver.imuClock(event.imu.timestamp_ns,event.arrival_ns);
      else {
        const auto& raw=event.belief;fl2b::Belief b;
        b.source=raw.source_agent=="g"?"glim":raw.source_agent=="k"?"kimera":raw.source_agent;
        b.session=raw.session_id;b.from=raw.from_index;b.to=raw.to_index;
        b.ordinal=raw.ordinal;b.revision=raw.revision;b.start=raw.from_timestamp_ns;b.end=raw.to_timestamp_ns;
        b.arrival=raw.record_timestamp_ns;b.mean=raw.measurement;b.covariance=raw.covariance;receiver.belief(b);
      }
    }
    const auto& s=receiver.state();
    std::cout<<"FL2B_STREAM_COMPLETE received="<<s.counts.received<<" inserted="<<s.counts.inserted
      <<" replaced="<<s.counts.replaced<<" absorbed="<<s.counts.absorbed<<" active="<<s.active.size()
      <<" supports="<<s.counts.publications<<" finalized="<<s.counts.finalized
      <<" remaining_supports="<<s.solver.timestamps().size()/3<<'\n';return 0;
  }catch(const std::exception& e){std::cerr<<"FL2B_STREAM_FAILURE "<<e.what()<<'\n';return 1;}
}
