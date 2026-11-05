#pragma once
#include <cstdlib>
#include <stdexcept>
#include <string>
namespace cpv76 {
inline bool enabled() {
  const char* e=std::getenv("CPV76_PERSISTENT_COMPAT");
  if(!e || std::string(e)=="0") return false;
  const auto is=[](const char* k,const char* v){const char* x=std::getenv(k);return x&&std::string(x)==v;};
  const char* retain=std::getenv("CPV69_RETAIN_COMPAT");
  if(std::string(e)!="1" || !is("CPK9_PRIOR_REFRESH","1") ||
     !is("CPV40_REFRESH","1") || !is("CPV38_OFF_REFRESH","0") ||
     !is("FL4Q_ENABLE","0") || !is("FL4M_RETIREMENT","remove") ||
     (retain&&std::string(retain)!="0"))
    throw std::runtime_error("CPV76 requires corrected persistent CP40/K9, no FL4Q/CP38/CP69");
  // FL4M_REMOVE is deliberately inert in general Persistent mode. No ambiguous
  // combination with the separate active-window RETAIN compatibility switch.
  return true;
}
}
