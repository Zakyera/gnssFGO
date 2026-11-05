#include "RecordedUpdateGate.h"
#include <cassert>
#include <iostream>
#include <functional>
#include <array>
struct Belief {
  uint8_t source_agent = 'g';
  uint64_t sender_from_pose_index, sender_to_pose_index;
  std::array<double, 42> payload{};
  bool operator==(const Belief& b) const {
    return source_agent==b.source_agent && sender_from_pose_index==b.sender_from_pose_index &&
      sender_to_pose_index==b.sender_to_pose_index && payload==b.payload;
  }
};
void fail(const std::function<void()>& fn) {
  bool caught=false; try {fn();} catch(const std::runtime_error&) {caught=true;}
  assert(caught);
}
Belief belief(uint64_t from, uint64_t to) {
  Belief b{'g',from,to,{}};
  for(size_t i=0;i<b.payload.size();++i)b.payload[i]=double(i+from)/117.;
  return b;
}
int main() {
  using Gate=cpv70::RecordedUpdateGate;
  const auto a=belief(405,407), b=belief(407,409);
  Gate disabled; auto d=disabled.select(206,std::vector<Belief>{b,a});
  assert(d.released==std::vector<Belief>({b,a}) && d.held.empty());
  Gate gate({{207,405,407},{207,407,409}});
  auto early=gate.select(206,std::vector<Belief>{a});
  assert(early.released.empty() && early.held==std::vector<Belief>{a});
  auto due=gate.select(207,std::vector<Belief>{b,early.held.front()});
  assert(due.released==std::vector<Belief>({a,b}) && due.held.empty());
  assert(due.receipts[0].first_seen==206 && due.receipts[1].first_seen==207);
  assert(gate.releasedCount()==2);
  fail([&]{gate.select(208,std::vector<Belief>{a});});
  fail([&]{gate.select(207,std::vector<Belief>{});});
  Gate missing({{207,405,407}});
  fail([&]{missing.select(207,std::vector<Belief>{});});
  assert(missing.releasedCount()==0);
  fail([&]{missing.select(208,std::vector<Belief>{a});});
  fail([&]{missing.select(206,std::vector<Belief>{a,a});});
  fail([&]{missing.select(206,std::vector<Belief>{b});});
  auto wrong=a;wrong.source_agent='k';
  fail([&]{missing.select(206,std::vector<Belief>{wrong});});
  fail([]{Gate g({{207,405,407},{207,405,407}});});
  fail([]{Gate g({{207,405,407},{206,407,409}});});
  fail([]{Gate g(std::vector<cpv70::Entry>{});});
  // Real full1743-row schedule, arbitrary early arrivals, identical exact payloads.
  auto real=Gate::fromEnvironment(); assert(real.enabled());
  std::ifstream file(std::getenv("CPV70_RECORDED_UPDATE_SCHEDULE"));
  std::vector<Belief> all;cpv70::Entry r{};uint64_t last=0;
  while(file>>r.update>>r.from>>r.to){all.push_back(belief(r.from,r.to));last=r.update;}
  std::vector<Belief> pending=all, output;
  for(uint64_t update=0;update<=last;++update) {
    auto sel=real.select(update,pending);pending=sel.held;
    output.insert(output.end(),sel.released.begin(),sel.released.end());
    for(const auto& x:sel.receipts)assert(x.first_seen<=x.update);
  }
  assert(all.size()==1743 && output==all && pending.empty() && real.releasedCount()==1743);
  std::cout<<"PASS CP73 inherited strict gate, disabled parity, negative guards, full1743 payload conservation\n";
}
