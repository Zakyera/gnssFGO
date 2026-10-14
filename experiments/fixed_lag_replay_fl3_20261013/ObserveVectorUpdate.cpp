// Read-only call-site observation; forwards every call to the original library.
// No injected values, solver state change, suppression or numerical replacement.
#include <gtsam/linear/VectorValues.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <cstdio>
#include <cstdlib>
namespace gtsam {
void VectorValues::update(const VectorValues& values) {
  using Fn=void(*)(VectorValues*,const VectorValues&);
  static auto original=reinterpret_cast<Fn>(dlsym(RTLD_NEXT,"_ZN5gtsam12VectorValues6updateERKS0_"));
  if(!original)std::abort();
  static bool reported=false;
  for(const auto& kv:values)if(!exists(kv.first)&&!reported) {
    reported=true;std::fprintf(stderr,"FL3_FIRST_MISSING_KEY_CALL_STACK\n");
    void* trace[40];const int n=backtrace(trace,40);backtrace_symbols_fd(trace,n,2);
  }
  original(this,values);
}
}
