#pragma once
#include <chrono>
#include <map>
#include <string>
namespace diagnostic_timing {
using Clock = std::chrono::steady_clock;
inline thread_local std::map<std::string,double>* output = nullptr;
struct Span {
  std::string prefix;
  Clock::time_point last = Clock::now();
  explicit Span(const char* p) : prefix(p) {}
  void mark(const char* name) {
    const auto now=Clock::now();
    if(output)(*output)[prefix+name]+=std::chrono::duration<double>(now-last).count();
    last=Clock::now();
  }
};
}
