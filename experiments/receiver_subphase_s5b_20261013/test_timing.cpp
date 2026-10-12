#include "ReceiverTiming.h"
#include <cassert>
#include <stdexcept>
#include <thread>

void busy() { for (volatile int i=0; i<10000; ++i) {} }
int main() {
  { s5b::Scope root(s5b::Run);
    busy();
    { s5b::ContextScope context(s5b::Publishing);
      s5b::Scope pub(s5b::Publication);
      busy();
      { s5b::Scope child(s5b::SolverCopy); busy(); }
      try { s5b::Timed(s5b::IsamUpdate, [] { busy(); throw std::runtime_error("test"); }); }
      catch (const std::runtime_error&) {}
      assert(s5b::Timed(s5b::Estimate, [] { return 17; }) == 17);
    }
    assert(s5b::context == s5b::Other);
  }
  assert(s5b::top == nullptr);
  if (!s5b::Enabled()) {
    for (const auto& row : s5b::counts) for (const auto& c : row) assert(c.calls == 0);
    return 0;
  }
  const auto& root = s5b::counts[s5b::Other][s5b::Run];
  const auto& pub = s5b::counts[s5b::Publishing][s5b::Publication];
  uint64_t sum = 0;
  for (const auto& row : s5b::counts) for (const auto& c : row) sum += c.exclusive;
  assert(sum == root.inclusive);
  assert(root.calls == 1 && root.exclusive > 0 && pub.exclusive > 0);
  assert(root.inclusive >= pub.inclusive && root.cpu > 0 && pub.cpu > 0);
  assert(s5b::counts[s5b::Publishing][s5b::IsamUpdate].calls == 1);
  std::thread worker([] { s5b::Scope ignored(s5b::Estimate); }); worker.join();
  assert(s5b::foreign_scopes == 1);  // Explicitly detect unsupported worker attribution.
  s5b::Flush(1, 123, "test");
}
