// Same deterministic solver fixture; this is not a source-estimator benchmark.
#include "test_solver.cpp"
#include <algorithm>
#include <numeric>
#include <string>

int main(int argc,char** argv) {
  tbb::global_control limit(tbb::global_control::max_allowed_parallelism,4);
  const std::string mode=argc>1?argv[1]:"bench";
  if(mode=="sink") {
    const auto start=e1b::tick();auto r=run(false);const auto elapsed=e1b::tick()-start;
    std::cout<<"SINK,"<<r.numbers.size()<<','<<r.linearizations<<','<<r.errors<<','<<elapsed<<'\n';
    return 0;
  }
  if(mode=="file_bench") {
    for(int sample=0;sample<25;++sample) {
      const auto start=e1b::tick();
      for(int repetition=0;repetition<10;++repetition)run(false);
      std::cout<<"FILE_SAMPLE,"<<sample<<','<<e1b::tick()-start<<'\n';
    }
    return 0;
  }
  // Warm both paths, then alternate order; all samples compare identical math.
  auto reference=run(false);run(true);
  for(int sample=0;sample<25;++sample) {
    uint64_t times[2]{};
    for(int turn=0;turn<2;++turn) {
      const bool enabled=(sample+turn)%2;
      auto begin=e1b::tick();
      for(int repetition=0;repetition<10;++repetition) {
        auto result=run(enabled);
        if(result.numbers!=reference.numbers || result.linearizations!=reference.linearizations ||
           result.errors!=reference.errors) return 2;
      }
      times[enabled]=e1b::tick()-begin;
    }
    std::cout<<"PAIR,"<<sample<<','<<times[0]<<','<<times[1]<<'\n';
  }
  return 0;
}
