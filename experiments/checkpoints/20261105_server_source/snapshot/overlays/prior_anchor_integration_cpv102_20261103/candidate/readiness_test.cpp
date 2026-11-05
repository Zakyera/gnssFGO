#include "ReplayReadiness.h"
#include <array>
#include <atomic>
#include <cassert>
#include <future>
#include <iostream>
#include <thread>
struct B {
  uint8_t source_agent = 'g';
  uint64_t sender_from_pose_index = 685, sender_to_pose_index = 687;
  std::array<double,42> payload{};
  bool operator==(const B& b) const {
    return source_agent==b.source_agent && sender_from_pose_index==b.sender_from_pose_index &&
      sender_to_pose_index==b.sender_to_pose_index && payload==b.payload;
  }
};
template<class F> void fails(F fn,const std::string& text) {
  bool caught=false;
  try { fn(); } catch(const std::runtime_error& e) {
    caught=true;assert(std::string(e.what()).find(text)!=std::string::npos);
  }
  assert(caught);
}
int main() {
  using Gate=cpv70::RecordedUpdateGate;
  using Barrier=cpv73::ReplayReadiness;
  B b; for(size_t i=0;i<b.payload.size();++i)b.payload[i]=(i+1.)/117.;
  const std::vector<B> empty,one{b};
  Barrier disabled;
  auto untouched=disabled.collect(Gate{},346,[&]{return one;},0);
  assert(untouched.arrived==one && untouched.waits==0 && untouched.wall_ms==0);
  Barrier enabled(2000);
  fails([&]{enabled.collect(Gate{},346,[&]{return one;},100);},"schedule required");
  Gate g({{346,685,687}});
  assert(!g.ready(346,empty) && g.ready(346,one) && g.releasedCount()==0);
  auto early=enabled.collect(g,345,[&]{return one;},100);
  assert(early.waits==0 && early.arrived==one);
  auto held=g.select(345,early.arrived);assert(held.held==one);
  auto immediate=enabled.collect(g,346,[&]{return held.held;},100);
  assert(immediate.waits==0 && g.select(346,immediate.arrived).released==one);
  fails([&]{enabled.collect(g,347,[&]{return one;},100);},"already-released");
  fails([&]{enabled.collect(g,346,[&]{return empty;},100);},"repeated/reversed");
  Gate fresh({{346,685,687}});
  auto no_due=enabled.collect(fresh,345,[&]{return empty;},100);assert(no_due.waits==0);
  fails([&]{enabled.collect(fresh,346,[&]{return std::vector<B>{b,b};},100);},"duplicate pending");
  auto wrong=b;wrong.source_agent='k';
  fails([&]{enabled.collect(fresh,346,[&]{return std::vector<B>{wrong};},100);},"unexpected source");
  wrong=b;wrong.sender_to_pose_index=689;
  fails([&]{enabled.collect(fresh,346,[&]{return std::vector<B>{wrong};},100);},"unlisted");
  fails([&]{enabled.collect(fresh,347,[&]{return one;},100);},"late sender");
  fails([&]{enabled.collect(fresh,347,[&]{return empty;},100);},"missing at target");
  fails([&]{enabled.collect(fresh,346,[&]{return one;},0);},"queue cap");
  assert(fresh.releasedCount()==0);
  Barrier timeout(20);
  const auto start=Barrier::Clock::now();
  fails([&]{timeout.collect(fresh,346,[&]{return empty;},100);},"timeout at update 346");
  assert(std::chrono::duration<double>(Barrier::Clock::now()-start).count()<2.0);
  // Real two-thread handoff. Signal after empty snapshot; producer enqueues
  // independently. No queue/readiness mutex held across any wait.
  for(int run=0;run<100;++run) {
    Barrier barrier(2000); Gate gate({{346,685,687}});
    std::mutex queue_mutex;std::vector<B> queue;std::promise<void> empty_pop;
    auto seen_empty=empty_pop.get_future();bool first=true;
    std::thread producer([&]{seen_empty.wait();
      {std::lock_guard<std::mutex> lock(queue_mutex);queue.push_back(b);}
      barrier.acknowledgeEnqueue();
    });
    auto result=barrier.collect(gate,346,[&]{
      std::vector<B> out;
      {std::lock_guard<std::mutex> lock(queue_mutex);out.swap(queue);}
      if(first){first=false;empty_pop.set_value();}
      return out;
    },100);
    producer.join();assert(result.arrived==one && result.waits>=1);
    assert(gate.select(346,result.arrived).released==one);
  }
  // Deterministically notify in the gap AFTER empty pop BEFORE waiter lock.
  Barrier gap(2000); int pops=0;
  auto gap_result=gap.collect(fresh,346,[&]{
    if(++pops==1){gap.acknowledgeEnqueue();return empty;}return one;
  },100);
  assert(gap_result.arrived==one && gap_result.waits==1 && pops==2);
  // Unrelated wakeups do not extend the absolute deadline or satisfy input.
  Barrier unrelated(20);std::atomic<bool> stop{false};
  std::thread noise([&]{while(!stop.load()){unrelated.acknowledgeEnqueue();std::this_thread::yield();}});
  bool caught=false;
  try{unrelated.collect(fresh,346,[&]{return empty;},100);}
  catch(const std::runtime_error& e){caught=true;assert(std::string(e.what()).find("at update 346")!=std::string::npos);}
  stop=true;noise.join();assert(caught && fresh.releasedCount()==0);
  // Due batch waits for BOTH entries, releases in schedule order, exact bytes.
  auto second=b;second.sender_from_pose_index=687;second.sender_to_pose_index=689;
  Gate multi({{346,685,687},{346,687,689}});Barrier batch_wait(2000);pops=0;
  auto multi_result=batch_wait.collect(multi,346,[&]{
    if(++pops==1){batch_wait.acknowledgeEnqueue();return std::vector<B>{second};}return one;
  },100);
  assert(multi.select(346,multi_result.arrived).released==std::vector<B>({b,second}));
  // Full saved schedule via readiness path, exact payload/order + gate receipt.
  auto full=Gate::fromEnvironment();assert(full.enabled());
  std::ifstream stream(std::getenv("CPV70_RECORDED_UPDATE_SCHEDULE"));
  cpv70::Entry e{};std::vector<B> all;uint64_t last=0;
  while(stream>>e.update>>e.from>>e.to){auto item=b;item.sender_from_pose_index=e.from;item.sender_to_pose_index=e.to;all.push_back(item);last=e.update;}
  std::vector<B> pending=all,output;
  for(uint64_t update=0;update<=last;++update){
    auto ready=enabled.collect(full,update,[&]{auto out=pending;pending.clear();return out;},2000);
    auto selected=full.select(update,ready.arrived);pending=selected.held;
    output.insert(output.end(),selected.released.begin(),selected.released.end());
  }
  assert(all.size()==1743 && all==output && pending.empty());
  unsetenv("CPV73_REPLAY_READY_TIMEOUT_MS");assert(Barrier::environmentTimeout()==0);
  setenv("CPV73_REPLAY_READY_TIMEOUT_MS","2000",1);assert(Barrier::environmentTimeout()==2000);
  for(const char* value:{"-1","0","2","2001","2000ms","200000000000"}){
    setenv("CPV73_REPLAY_READY_TIMEOUT_MS",value,1);
    fails([]{Barrier::environmentTimeout();},"expected explicit");
  }
  unsetenv("CPV73_REPLAY_READY_TIMEOUT_MS");
  std::cout<<"PASS CP73 readiness: disabled parity, delayed concurrent100, lost-wakeup, fixed timeout, atomic negatives, bounds, due order, full1743 payloads\n";
}
